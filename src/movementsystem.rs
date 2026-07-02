use super::costmatrixsystem::*;
use super::error::*;
use super::movementrequest::*;
use super::movementresult::*;
use super::resolver::*;
use super::traits::*;
use screeps::local::*;
use serde::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::hash::Hash;

/// Maximum number of rooms to consider for a single pathfinding search. Limits CPU
/// when pathing to far or impossible destinations (find_route can return long routes).
const MAX_PATHFIND_ROOMS: usize = 16;

/// Ceiling on pathfinder max_ops per search. 1 op ≈ 0.001 CPU, so 20_000 ops ≈ 20 CPU.
const MAX_PATHFIND_OPS: u32 = 20_000;

/// Default per-tick pathfinding ops budget (20 CPU). Enforced so movement cannot exhaust the tick.
const DEFAULT_PATHFIND_OPS_BUDGET: u32 = 20_000;

/// The share of the per-tick pathfinding ops pool RESERVED for first-path (`needs_path`)
/// searches, as a divisor of the cap (5 → the last 20% of the pool). OPTIONAL searches — stuck
/// and expiry repaths, where the creep still holds a usable path — are skipped once the
/// remaining pool falls into the reserve, so a creep with NO path (which cannot move at all)
/// always finds real search budget. See the reserve note at `should_pathfind`.
const NEEDS_PATH_OPS_RESERVE_DIV: u32 = 5;

/// Default ticks a cached path is followed before an expiry repath — i.e. PATH COMMITMENT.
/// TUNED 5 → 20 by the screeps-rover-eval parameter tournament (ADR 0033 §D5.4, 2026-07-01):
/// with 5, a creep whose stuck-escalation found a detour re-optimized back onto the blocked
/// optimistic path within 5 ticks and FLAPPED between the two forever (a livelock reproduced on
/// 2 of 13 real foreman-planned rooms — a finished hauler parking on a 1-wide road-corridor
/// mouth sealed its mate in; constant motion kept deadlock detectors silent). 20 commits to the
/// detour long enough to pass: corpus completion 0.844 → 1.000, wasted intents 832 → 28,
/// value-weighted efficiency H 0.680 → 0.769 — and FEWER expiry repaths = less CPU. Safe for
/// dynamic movement: a destination/range change still discards the path immediately (see
/// `dest_matches` below), so only stable-destination routes (hauling) feel the longer reuse.
const DEFAULT_REUSE_PATH_LENGTH: u32 = 20;

/// Configurable thresholds for stuck detection tiers.
/// Different job types can use different thresholds (e.g. military creeps
/// might have lower thresholds for faster reaction).
#[derive(Clone, Debug)]
pub struct StuckThresholds {
    /// Ticks immobile before avoiding *nearby* friendly creeps in pathfinding (tier 1).
    /// Only creeps within `friendly_creep_distance` rooms are avoided.
    pub avoid_friendly_creeps: u16,
    /// Ticks immobile before avoiding *all* friendly creeps regardless of
    /// distance (tier 1b). Escalation from the proximity-limited tier.
    pub avoid_all_friendly_creeps: u16,
    /// Ticks immobile before increasing search max_ops (tier 2).
    pub increase_ops: u16,
    /// Ticks immobile before enabling shoving in the resolver (tier 3). Consulted ONLY when
    /// [`MovementSystem::set_shove_escalation`]`(true)` opted in (default OFF preserves the
    /// historical behavior exactly): with escalation ON, a creep may INITIATE shoves only once
    /// `ticks_immobile` reaches this tier (`request.allow_shove` still required), and this value
    /// replaces the resolver's historical `STUCK_SHOVE_THRESHOLD` constant as the stuck gate in
    /// `try_shove` — tier 3 has ONE knob, per-request tunable via
    /// `MovementRequest::stuck_thresholds`. With escalation OFF, shoving stays governed
    /// per-request via `MovementRequest.allow_shove` plus the resolver's own priority/stuck gate
    /// at the constant threshold (5).
    pub enable_shoving: u16,
    /// Ticks immobile before reporting failure to the job layer (tier 4).
    pub report_failure: u16,
    /// Ticks of no progress (moving but not getting closer) before repathing.
    pub no_progress_repath: u16,
}

impl Default for StuckThresholds {
    fn default() -> Self {
        StuckThresholds {
            avoid_friendly_creeps: 2,
            avoid_all_friendly_creeps: 4,
            increase_ops: 5,
            enable_shoving: 7,
            report_failure: 12,
            no_progress_repath: 15,
        }
    }
}

/// Tiered stuck detection state. Tracks both immobility (creep didn't move)
/// and lack of progress (distance to target isn't decreasing).
#[derive(Clone, Serialize, Deserialize, Default)]
pub struct StuckState {
    /// How many consecutive ticks the creep has not changed position.
    pub ticks_immobile: u16,
    /// How many consecutive ticks distance to target hasn't decreased.
    pub ticks_no_progress: u16,
    /// Search attempts (successful or failed) this STUCK EPISODE — reset on real progress. The
    /// exponent of the storm-damper backoff (`should_stuck_repath_with`); previously a
    /// written-never-read per-destination counter (IBEX-016 flagged exactly this repurposing:
    /// cap/space repaths so a failed search cannot re-fire every tick).
    pub repath_count: u8,
    /// Distance to target last tick (for progress tracking).
    #[serde(default)]
    pub last_distance: u32,
    /// Set by `process()` when the resolver denied this creep's grant because of an IDLE
    /// occupant (avoidance sidestep, stay, or the sealed-corridor push) — consumed by the next
    /// tick's path-progress accounting, which then records IMMOBILE even if the creep physically
    /// moved (denial-as-stuck, ADR 0033 M5: the avoidance dance must feed the escalation
    /// ladder). NOTE: `#[serde(default)]` is forward-compat only — for positional consumers
    /// (screeps-ibex persists this struct via bincode in the world save) an appended field is
    /// still a shape change; ibex bumped WORLD_FORMAT_VERSION to 24 for this field and
    /// `ticks_since_repath` below (reconciliation REC-001).
    #[serde(default)]
    pub denied_by_idle: bool,
    /// Ticks since the last `generate_path` attempt (successful or failed) — the storm damper's
    /// clock (`should_stuck_repath_with`). Additive `#[serde(default)]`.
    #[serde(default)]
    pub ticks_since_repath: u16,
}

impl StuckState {
    /// Reset all stuck tracking (e.g. when destination changes).
    pub fn reset(&mut self) {
        self.ticks_immobile = 0;
        self.ticks_no_progress = 0;
        self.repath_count = 0;
        self.last_distance = 0;
        self.denied_by_idle = false;
        self.ticks_since_repath = 0;
    }

    /// Record that the creep moved this tick.
    pub fn record_moved(&mut self, current_distance: u32) {
        self.ticks_immobile = 0;
        self.ticks_since_repath = self.ticks_since_repath.saturating_add(1);

        if current_distance < self.last_distance {
            self.ticks_no_progress = 0;
            // Real progress ends the stuck episode: the backoff exponent re-arms so the NEXT
            // jam gets a fast first response (see `should_stuck_repath_with`).
            self.repath_count = 0;
        } else {
            self.ticks_no_progress += 1;
        }

        self.last_distance = current_distance;
    }

    /// Record that the creep did NOT move this tick.
    pub fn record_immobile(&mut self, current_distance: u32) {
        self.ticks_immobile += 1;
        self.ticks_no_progress += 1;
        self.ticks_since_repath = self.ticks_since_repath.saturating_add(1);
        self.last_distance = current_distance;
    }

    pub fn should_avoid_friendly_creeps(&self) -> bool {
        self.should_avoid_friendly_creeps_with(&StuckThresholds::default())
    }

    pub fn should_avoid_all_friendly_creeps(&self) -> bool {
        self.should_avoid_all_friendly_creeps_with(&StuckThresholds::default())
    }

    pub fn should_increase_ops(&self) -> bool {
        self.should_increase_ops_with(&StuckThresholds::default())
    }

    pub fn should_enable_shoving(&self) -> bool {
        self.should_enable_shoving_with(&StuckThresholds::default())
    }

    pub fn should_avoid_friendly_creeps_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.avoid_friendly_creeps
    }

    pub fn should_avoid_all_friendly_creeps_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.avoid_all_friendly_creeps
    }

    pub fn should_increase_ops_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.increase_ops
    }

    /// Tier 3: consulted by `process()` when shove-escalation mode is opted in — see
    /// `StuckThresholds::enable_shoving`. (Inert under the default escalation-OFF mode.)
    pub fn should_enable_shoving_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.enable_shoving
    }

    pub fn should_report_failure(&self) -> bool {
        self.should_report_failure_with(&StuckThresholds::default())
    }

    pub fn should_report_failure_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.report_failure
    }

    pub fn should_repath_no_progress(&self) -> bool {
        self.should_repath_no_progress_with(&StuckThresholds::default())
    }

    pub fn should_repath_no_progress_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_no_progress >= thresholds.no_progress_repath
    }

    pub fn needs_repath(&self) -> bool {
        self.needs_repath_with(&StuckThresholds::default())
    }

    pub fn needs_repath_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.avoid_friendly_creeps
            || self.should_repath_no_progress_with(thresholds)
    }

    /// The stuck-repath STORM DAMPER (the ADR 0004 repath-storm / CPU-death-spiral class,
    /// IBEX-016). INSIDE the designed escalation window (`ticks_immobile ≤ report_failure`) the
    /// historical every-tick cadence stands: successive searches carry NEW options as the tiers
    /// flip (friendly-avoid → all-friendly → more ops), and coordination-heavy movement relies on
    /// the fast recovery (the combat drain-soak bed regressed to RosterWiped under earlier
    /// damping — the adjudicated evidence). PAST tier 4 the ladder has nothing new to offer, so a
    /// long jam's every-tick searches are pure waste: spacing starts at the tier-1 threshold and
    /// doubles with each attempt this episode (`repath_count`, reset on real progress; ×16
    /// shift cap, 64-tick ceiling). In a real jam the world changes slowly and waiting IS the
    /// answer: re-searching every immobile tick let a dense crowd (rover-eval `shared_pinch`,
    /// N ≥ 40) saturate the entire per-tick pathfinding ops pool indefinitely, starving path-LESS
    /// creeps into doomed dreg-budget searches forever — the wedge nuclei of the dense-crowd
    /// livelock. Per-request by construction: both the window edge and the base come from the
    /// request's own `StuckThresholds` (the split-defaults mechanism).
    pub fn should_stuck_repath_with(&self, thresholds: &StuckThresholds) -> bool {
        if !self.needs_repath_with(thresholds) {
            return false;
        }
        if self.ticks_immobile <= thresholds.report_failure {
            return true;
        }
        let base = thresholds.avoid_friendly_creeps.max(1) as u32;
        let spacing = (base << self.repath_count.min(4)).min(64) as u16;
        self.ticks_since_repath >= spacing
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct CreepPathData {
    destination: Position,
    range: u32,
    path: Vec<Position>,
    time: u32,
    #[serde(default)]
    pub stuck_state: StuckState,
}

#[derive(Clone, Serialize, Deserialize, Default)]
pub struct CreepMovementData {
    path_data: Option<CreepPathData>,
}

#[derive(Default)]
pub struct MovementData<Handle>
where
    Handle: Hash + Eq,
{
    requests: HashMap<Handle, MovementRequest<Handle>>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<Handle> MovementData<Handle>
where
    Handle: Hash + Eq + Copy,
{
    pub fn new() -> MovementData<Handle> {
        MovementData {
            requests: HashMap::new(),
        }
    }

    pub fn move_to(
        &mut self,
        entity: Handle,
        destination: Position,
    ) -> MovementRequestBuilder<'_, Handle> {
        self.requests
            .entry(entity)
            .and_modify(|e| *e = MovementRequest::move_to(destination))
            .or_insert_with(|| MovementRequest::move_to(destination))
            .into()
    }

    pub fn follow(&mut self, entity: Handle, target: Handle) -> MovementRequestBuilder<'_, Handle> {
        self.requests
            .entry(entity)
            .and_modify(|e| *e = MovementRequest::follow(target))
            .or_insert_with(|| MovementRequest::follow(target))
            .into()
    }

    pub fn flee(
        &mut self,
        entity: Handle,
        targets: Vec<FleeTarget>,
    ) -> MovementRequestBuilder<'_, Handle> {
        self.requests
            .entry(entity)
            .and_modify(|e| *e = MovementRequest::flee(targets.clone()))
            .or_insert_with(|| MovementRequest::flee(targets))
            .into()
    }

    /// Number of movement requests this tick (for CPU budgeting; each move action costs 0.2 CPU).
    pub fn request_count(&self) -> usize {
        self.requests.len()
    }

    /// Whether `entity` has a movement request this tick. Used by callers building the
    /// idle-occupant registration ([`MovementSystem::set_idle_creep_positions`]): every living
    /// owned creep ABSENT from the request set is a known stationary occupant.
    pub fn contains_request(&self, entity: &Handle) -> bool {
        self.requests.contains_key(entity)
    }
}

/// External interface that the movement system uses to interact with the
/// game world. The `Creep` associated type must implement `CreepHandle`.
pub trait MovementSystemExternal<Handle> {
    type Creep: CreepHandle;

    fn get_creep(&self, entity: Handle) -> Result<Self::Creep, MovementError>;

    fn get_creep_movement_data(
        &mut self,
        entity: Handle,
    ) -> Result<&mut CreepMovementData, MovementError>;

    fn get_room_cost(
        &self,
        from_room_name: RoomName,
        to_room_name: RoomName,
        _room_options: &RoomOptions,
    ) -> Option<f64> {
        let _ = (from_room_name, to_room_name);
        Some(1.0)
    }

    fn get_entity_position(&self, entity: Handle) -> Option<Position>;
}

/// Default tile distance (Chebyshev) for proximity-limited friendly creep
/// avoidance. Creeps beyond this many tiles from the pathing origin are
/// ignored in the cost matrix, since they will likely have moved by the time
/// we arrive.
pub const DEFAULT_FRIENDLY_CREEP_DISTANCE: u32 = 5;

/// Tracks CPU spent on non-stuck path expiry repathing within a single tick.
pub struct RepathBudget<'a> {
    get_cpu: Box<dyn Fn() -> f64 + 'a>,
    budget: f64,
    start_cpu: f64,
}

impl<'a> RepathBudget<'a> {
    /// Returns `true` when the repath budget has been exhausted.
    fn is_exhausted(&self) -> bool {
        let spent = (self.get_cpu)() - self.start_cpu;
        spent >= self.budget
    }
}

/// (get_cpu, start_cpu, max_cpu) for hard movement CPU cap per tick.
type MovementCpuCap<'a> = (Box<dyn Fn() -> f64 + 'a>, f64, f64);

pub struct MovementSystem<'a, Handle> {
    cost_matrix_system: &'a mut CostMatrixSystem<'a>,
    pathfinder: &'a mut dyn PathfindingProvider,
    visualizer: Option<&'a mut dyn MovementVisualizer>,
    reuse_path_length: u32,
    max_shove_depth: u32,
    /// Maximum tile distance (Chebyshev) from the creep's position for the
    /// first tier of friendly creep avoidance. Creeps beyond this distance
    /// will not have their positions marked as impassable, since they will
    /// likely have moved by the time the pathing creep arrives. Set to 0 to
    /// disable proximity limiting (equivalent to the old behaviour of avoiding
    /// all creeps).
    friendly_creep_distance: u32,
    /// The stuck-escalation ladder (how many immobile ticks before each tier fires:
    /// friendly-avoid → all-friendly-avoid → more ops → shove → report failure). Injectable so the
    /// escalation SPEED is tunable per deployment (and benchmarkable offline); defaults preserve
    /// the historical ladder exactly.
    stuck_thresholds: StuckThresholds,
    /// Shove-as-ESCALATION opt-in (default OFF = the historical behavior: any winner may attempt
    /// a shove and the resolver's constant `STUCK_SHOVE_THRESHOLD` gates equal/lower-priority
    /// shovers). When ON: a creep may initiate shoves only when `request.allow_shove` AND its
    /// stuck ladder reached tier 3 (`StuckThresholds::enable_shoving`), and that same tier value
    /// replaces the resolver constant — ONE knob, per-request tunable.
    shove_escalation: bool,
    /// CPU budget for stuck repathing. When exhausted, the movement system
    /// skips pathfinding for stuck creeps (they keep their existing path and
    /// the resolver's shove/swap mechanisms can still help). Only
    /// `needs_path` (creeps with no path at all) is unconditional.
    cpu_budget: Option<RepathBudget<'a>>,
    /// CPU budget for non-stuck path expiry repathing. Paths older than
    /// `reuse_path_length` are eligible for re-evaluation but only if this
    /// budget has not been exhausted.
    repath_budget: Option<RepathBudget<'a>>,
    /// Per-tick pathfinding ops cap (1 op ≈ 0.001 CPU). Reset to this at start of process().
    pathfinding_ops_budget_cap: u32,
    /// Remaining pathfinding ops this tick. All pathfinding (including needs_path) deducts from this.
    pathfinding_ops_budget_remaining: u32,
    /// When set, process() skips all work for a creep once get_cpu() >= limit (avoids exceeding tick limit).
    tick_limit: Option<(Box<dyn Fn() -> f64 + 'a>, f64)>,
    /// When set, process() also skips work once (get_cpu() - start_cpu) >= max_cpu (hard movement CPU cap per tick).
    movement_cpu_cap: Option<MovementCpuCap<'a>>,
    /// When set, do not start pathfinding unless (used + headroom) <= max_cpu. Prevents one unbounded find_route
    /// from blowing the cap when we were just under it (e.g. at 79 CPU with cap 80, one pathfind can use 100+).
    pathfinding_headroom: Option<f64>,
    /// Repaths performed this tick (reset in process(); read via tick_stats()).
    repaths_this_tick: u32,
    /// Known stationary occupants OUTSIDE this tick's request set (position → handle), injected
    /// via [`set_idle_creep_positions`](Self::set_idle_creep_positions) and CONSUMED (taken, so
    /// cleared) by the next `process()`. Without it, a creep that reached its goal and left the
    /// request set is invisible to both the optimistic first-path (`friendly_creeps: false`) and
    /// the resolver — a later creep paths into it and burns `ticks_immobile ≥ 2` engine-rejected
    /// intents per blocking event before the friendly-avoid repath fires (ADR 0033 §M4 finding
    /// F2, `failed_into_parked`). Registered occupants are treated by `resolve_conflicts` as
    /// occupied tiles: the grant path denies them and offers local avoidance, so the mover
    /// routes around parked creeps deliberately. Default empty = the historical behavior.
    idle_creep_positions: HashMap<Position, Handle>,
    phantom: std::marker::PhantomData<Handle>,
}

/// A stationary, non-displaceable occupancy entry for a REQUESTED creep that must not be issued
/// any move intent this tick, but still occupies its tile through the whole movement phase:
///
/// - **fatigued / spawning** — the engine's `canMove` (movement.js) disqualifies a creep with
///   nonzero tick-start fatigue (only a pull bypasses it), so its own move AND any shove/swap
///   move planned for it would be dropped;
/// - **border-crossing** — a creep standing on an exit tile whose next path step is the adjacent
///   room's mirror tile: the cross is NOT a move (the engine drops an off-edge move intent at
///   registration, `move.js:32`) — the unconditional end-of-tick edge relocation
///   (`creeps/tick.js`) carries it across for free.
///
/// Either way the creep stays on its tile through movement resolution, so it must remain VISIBLE
/// to `resolve_conflicts` as a stationary occupant: the grant, avoidance, and shove paths all
/// treat its tile as taken. Dropping it from the resolver's world (the historical behavior for
/// both shapes) let the resolver plan other creeps straight through its tile and issue the
/// doomed cross-step itself — the two engine-rejected-intent classes the rover-eval failed-move
/// sentinel caught at room borders and on heavy-fatigue swamp routes (ADR 0033 M5, 2026-07-01).
fn stationary_occupant<Handle: Hash + Eq + Copy>(
    entity: Handle,
    creep_pos: Position,
    request: &MovementRequest<Handle>,
) -> ResolvedCreep<Handle> {
    ResolvedCreep {
        entity,
        current_pos: creep_pos,
        desired_pos: None,
        priority: request.priority,
        priority_value: request.effective_priority(),
        // Not displaceable THIS tick: a shove/swap move issued for it is engine-dropped by
        // construction (fatigue gate / the relocation already owns the crosser's tick).
        allow_shove: false,
        shove_enabled: false,
        shove_stuck_threshold: STUCK_SHOVE_THRESHOLD,
        allow_swap: false,
        stuck_ticks: 0,
        resolved: false,
        final_pos: creep_pos,
        has_request: true,
        denied_by_idle: false,
        anchor: request.anchor,
    }
}

/// Per-tick movement telemetry, read after `process()` (host telemetry
/// consumers — e.g. ibex's seg-57 metrics block).
#[derive(Debug, Clone, Copy, Default)]
pub struct MovementTickStats {
    /// The configured per-tick pathfinding ops budget.
    pub ops_budget_cap: u32,
    /// Ops actually consumed by pathfinding this tick.
    pub ops_consumed: u32,
    /// Paths regenerated this tick (stuck + expiry repaths; first-time
    /// paths are not repaths).
    pub repaths: u32,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a, Handle> MovementSystem<'a, Handle>
where
    Handle: Hash + Eq + Copy + Ord,
{
    pub fn new(
        cost_matrix_system: &'a mut CostMatrixSystem<'a>,
        pathfinder: &'a mut dyn PathfindingProvider,
        visualizer: Option<&'a mut dyn MovementVisualizer>,
    ) -> Self {
        Self {
            cost_matrix_system,
            pathfinder,
            visualizer,
            reuse_path_length: DEFAULT_REUSE_PATH_LENGTH,
            max_shove_depth: DEFAULT_MAX_SHOVE_DEPTH,
            friendly_creep_distance: DEFAULT_FRIENDLY_CREEP_DISTANCE,
            // NOTE: the same tournament found ladder(8) (a 4× slower escalation) adds a further
            // +0.08 H on the HAUL corpus — but it rewrites the report_failure job-layer contract
            // (12 → 48 ticks) and is unvalidated for combat movement (immobility under fire), so
            // it stays a recorded candidate, not the default. See ADR 0033 §D5.4.
            stuck_thresholds: StuckThresholds::default(),
            shove_escalation: false,
            cpu_budget: None,
            repath_budget: None,
            pathfinding_ops_budget_cap: DEFAULT_PATHFIND_OPS_BUDGET,
            pathfinding_ops_budget_remaining: DEFAULT_PATHFIND_OPS_BUDGET,
            tick_limit: None,
            movement_cpu_cap: None,
            pathfinding_headroom: None,
            repaths_this_tick: 0,
            idle_creep_positions: HashMap::new(),
            phantom: std::marker::PhantomData,
        }
    }

    /// Set pathfinding headroom: do not start pathfinding when (get_cpu() - start_cpu) + headroom > max_cpu.
    /// Use e.g. Some(movement_cap) to disable pathfinding when cap is tight; use None for previous behavior.
    pub fn set_pathfinding_headroom(&mut self, headroom: Option<f64>) {
        self.pathfinding_headroom = headroom;
    }

    /// Set a tick CPU limit. When set, process() skips all work for each creep once
    /// get_cpu() >= limit, inserting MovementResult::Moving so the tick does not exceed the limit.
    pub fn set_tick_limit(&mut self, get_cpu: impl Fn() -> f64 + 'a, limit: f64) {
        self.tick_limit = Some((Box::new(get_cpu), limit));
    }

    /// Set the per-tick pathfinding ops budget (1 op ≈ 0.001 CPU). E.g. 20_000 = 20 CPU max
    /// for all pathfinding this tick. Applies to every pathfinding call including first-time paths.
    pub fn set_pathfinding_ops_budget(&mut self, ops: u32) {
        self.pathfinding_ops_budget_cap = ops;
    }

    /// Per-tick telemetry for the LAST `process()` call.
    pub fn tick_stats(&self) -> MovementTickStats {
        MovementTickStats {
            ops_budget_cap: self.pathfinding_ops_budget_cap,
            ops_consumed: self
                .pathfinding_ops_budget_cap
                .saturating_sub(self.pathfinding_ops_budget_remaining),
            repaths: self.repaths_this_tick,
        }
    }

    /// Set a hard cap on CPU the movement system may use this tick. Once (get_cpu() - start_cpu) >= max_cpu,
    /// process() skips further work (same as tick limit). start_cpu should be captured at movement run start.
    pub fn set_movement_cpu_cap(&mut self, get_cpu: impl Fn() -> f64 + 'a, start_cpu: f64, max_cpu: f64) {
        self.movement_cpu_cap = Some((Box::new(get_cpu), start_cpu, max_cpu));
    }

    pub fn set_reuse_path_length(&mut self, length: u32) {
        self.reuse_path_length = length;
    }

    pub fn set_max_shove_depth(&mut self, depth: u32) {
        self.max_shove_depth = depth;
    }

    /// Set the maximum tile distance (Chebyshev) for proximity-limited
    /// friendly creep avoidance. Set to 0 to disable proximity limiting (all
    /// creeps get avoided when the tier is active).
    pub fn set_friendly_creep_distance(&mut self, distance: u32) {
        self.friendly_creep_distance = distance;
    }

    /// Set the SYSTEM-level stuck-escalation ladder (see [`StuckThresholds`]). Every stuck check
    /// the system performs routes through this — tune it to trade wasted-intent burn (slow
    /// escalation) against premature detours (fast escalation). A request carrying its own
    /// `stuck_thresholds` overrides this per creep (the ADR 0033 split-defaults end-state).
    pub fn set_stuck_thresholds(&mut self, thresholds: StuckThresholds) {
        self.stuck_thresholds = thresholds;
    }

    /// The effective ladder for one request: request-override else system default (ADR 0033 M5
    /// per-request `StuckThresholds` — haul jobs pass a slow ladder, military keeps the fast
    /// default, in the SAME process() call). Every stuck consult must route through this.
    fn thresholds_for<'r>(&'r self, request: &'r MovementRequest<Handle>) -> &'r StuckThresholds {
        request.stuck_thresholds.as_ref().unwrap_or(&self.stuck_thresholds)
    }

    /// Opt into shove-as-ESCALATION (see the `shove_escalation` field doc). OFF (default)
    /// preserves today's behavior EXACTLY — shove is a per-request default gated by the
    /// resolver's constant; ON makes tier 3 (`StuckThresholds::enable_shoving`) the single knob
    /// for when a stuck creep may start displacing others.
    pub fn set_shove_escalation(&mut self, enabled: bool) {
        self.shove_escalation = enabled;
    }

    /// Register creeps that are NOT in this tick's `MovementData` as stationary occupants
    /// (position → handle) for the NEXT `process()` call only (consumed, then cleared — per-tick
    /// data, like the request set). The resolver then denies grants/shove-landings/avoidance onto
    /// their tiles and sidesteps around them, instead of optimistically issuing a move the engine
    /// will reject (the `failed_into_parked` class, ADR 0033 §M4 F2 — see the field doc).
    /// Not calling this preserves the historical behavior exactly (empty map).
    pub fn set_idle_creep_positions(&mut self, positions: HashMap<Position, Handle>) {
        self.idle_creep_positions = positions;
    }

    /// Set a CPU budget for the movement system. `get_cpu` returns the
    /// current CPU usage; `budget` is the maximum CPU that may be spent on
    /// pathfinding for stuck creeps this tick. The start CPU is captured
    /// immediately when this method is called. Only `needs_path` (creeps
    /// with no path at all) is unconditional; stuck repathing is skipped
    /// once this budget is exhausted.
    pub fn set_cpu_budget(&mut self, get_cpu: impl Fn() -> f64 + 'a, budget: f64) {
        let start_cpu = get_cpu();
        self.cpu_budget = Some(RepathBudget {
            get_cpu: Box::new(get_cpu),
            budget,
            start_cpu,
        });
    }

    /// Set a CPU budget for non-stuck path expiry repathing. `get_cpu`
    /// returns the current CPU usage; `budget` is the maximum CPU that may
    /// be spent on expiry repathing this tick. The start CPU is captured
    /// immediately when this method is called.
    pub fn set_repath_budget(&mut self, get_cpu: impl Fn() -> f64 + 'a, budget: f64) {
        let start_cpu = get_cpu();
        self.repath_budget = Some(RepathBudget {
            get_cpu: Box::new(get_cpu),
            budget,
            start_cpu,
        });
    }

    /// Returns `true` when the movement CPU budget has been exhausted.
    ///
    /// `None` (no budget set) = UNLIMITED, consistent with `is_over_tick_limit` /
    /// `is_over_movement_cap` (`is_some_and`: an absent limit never binds). SEMANTICS CHANGED
    /// 2026-07-01: previously `is_none_or(exhausted)` — an ABSENT budget meant EXHAUSTED, so any
    /// consumer that never called `set_cpu_budget`/`set_repath_budget` (every headless driver;
    /// the sim-core kernel driver, hence the offline combat sim) silently ran with ALL
    /// stuck-escalation and expiry repathing disabled: a blocked creep re-issued its rejected
    /// move forever — the permanent-livelock class the rover-eval failed-move sentinel caught
    /// (ADR 0033 §M4 finding F1). The live bot always sets both budgets
    /// (ibex `MovementUpdateSystem` in pathing/movementsystem.rs) and sim-core's driver sets
    /// explicit unlimited ones, so this change only un-breaks future headless consumers.
    fn is_cpu_budget_exhausted(&self) -> bool {
        self.cpu_budget.as_ref().is_some_and(|b| b.is_exhausted())
    }

    /// Returns `true` when the repath budget for expiry repathing is exhausted.
    /// `None` = UNLIMITED (see `is_cpu_budget_exhausted` for the 2026-07-01 semantics change).
    fn is_repath_budget_exhausted(&self) -> bool {
        self.repath_budget.as_ref().is_some_and(|b| b.is_exhausted())
    }

    /// True when we have hit the tick CPU limit and should skip work for this creep.
    fn is_over_tick_limit(&self) -> bool {
        self.tick_limit
            .as_ref()
            .is_some_and(|(get_cpu, limit)| (get_cpu)() >= *limit)
    }

    /// True when the movement CPU cap is set and (get_cpu() - start_cpu) >= max_cpu.
    fn is_over_movement_cap(&self) -> bool {
        self.movement_cpu_cap.as_ref().is_some_and(|(get_cpu, start, max)| {
            (get_cpu)() - start >= *max
        })
    }

    /// Global movement resolution with conflict detection, shove/swap, and follow support.
    pub fn process<S>(
        &mut self,
        external: &mut S,
        data: MovementData<Handle>,
    ) -> MovementResults<Handle>
    where
        S: MovementSystemExternal<Handle>,
    {
        let mut results = MovementResults::new();

        // Per-tick injected state: consume the registered idle occupants (see
        // `set_idle_creep_positions`) so a stale registration can never leak into a later tick —
        // taken before the early return below for the same reason.
        let idle_creep_positions = std::mem::take(&mut self.idle_creep_positions);

        if data.requests.is_empty() {
            return results;
        }

        // Reset per-tick pathfinding ops budget so we don't exhaust CPU (1 op ≈ 0.001 CPU).
        self.pathfinding_ops_budget_remaining = self.pathfinding_ops_budget_cap;
        self.repaths_this_tick = 0;

        // --- Pass 0: Dependency analysis for Follow intents ---
        let (sorted_entities, broken_follows) = topological_sort_follows(&data.requests);

        // --- Pass 1: Compute desired next tile for each creep ---
        let mut leader_moves: HashMap<Handle, (Position, Option<Position>)> = HashMap::new();
        let mut resolved_creeps: HashMap<Handle, ResolvedCreep<Handle>> = HashMap::new();
        let mut pull_pairs: HashMap<Handle, Handle> = HashMap::new();
        let mut work_done_this_tick = false;

        for entity in &sorted_entities {
            if (self.is_over_tick_limit() || self.is_over_movement_cap()) && work_done_this_tick {
                results.insert(*entity, MovementResult::Moving);
                continue;
            }

            let request = match data.requests.get(entity) {
                Some(r) => r,
                None => continue,
            };

            let creep = match external.get_creep(*entity) {
                Ok(c) => c,
                Err(err) => {
                    results.insert(
                        *entity,
                        MovementResult::Failed(MovementFailure::InternalError(err)),
                    );
                    continue;
                }
            };

            let creep_pos = creep.pos();

            if creep.fatigue() > 0 || creep.spawning() {
                // Cannot execute any move this tick, but still occupies its tile through the
                // movement phase — keep it visible to the resolver as a stationary occupant
                // (see `stationary_occupant`: skipping it entirely made its tile invisible to
                // the grant/avoidance/shove paths, so the resolver planned other creeps
                // straight through it and the engine rejected every such intent — the
                // rover-eval swamp-contention `failed_coordination` class).
                leader_moves.insert(*entity, (creep_pos, None));
                resolved_creeps.insert(*entity, stationary_occupant(*entity, creep_pos, request));
                results.insert(*entity, MovementResult::Moving);
                continue;
            }

            let desired_result = match &request.intent {
                MovementIntent::MoveTo { destination, range } => {
                    if creep_pos.get_range_to(*destination) <= *range {
                        Ok(None)
                    } else {
                        self.compute_next_step_for_move_to(
                            external,
                            *entity,
                            creep_pos,
                            *destination,
                            *range,
                            request,
                        )
                    }
                }
                MovementIntent::Follow {
                    target,
                    range,
                    pull,
                    desired_offset,
                } => {
                    if *pull {
                        pull_pairs.insert(*entity, *target);
                    }

                    let effective_target = if broken_follows.contains_key(entity) {
                        external.get_entity_position(*target)
                    } else {
                        None
                    };

                    if let Some(target_pos) = effective_target {
                        if creep_pos.get_range_to(target_pos) <= *range {
                            Ok(None)
                        } else {
                            self.compute_next_step_for_move_to(
                                external, *entity, creep_pos, target_pos, *range, request,
                            )
                        }
                    } else {
                        self.compute_next_step_for_follow(
                            external,
                            *entity,
                            creep_pos,
                            *target,
                            *range,
                            *desired_offset,
                            request,
                            &leader_moves,
                        )
                    }
                }
                MovementIntent::Flee { targets, range } => {
                    self.compute_next_step_for_flee(creep_pos, targets, *range, request)
                }
            };

            match desired_result {
                // The next path step crosses a room border: the creep is standing on an exit
                // tile and the step is the adjacent room's mirror tile. That is NOT a move —
                // the engine drops an off-edge move intent at registration (`move.js:32`) and
                // instead relocates ANY creep standing on an exit tile at end of tick
                // (`creeps/tick.js`, unconditional — movement not required). Issuing the
                // outward step burned one guaranteed-rejected intent every tick a blocked
                // crosser bounced on the seam (the rover-eval border-wide `failed_wall`
                // class). Model it as a stationary occupant instead: it blocks its tile
                // through the movement phase, and the relocation does the cross for free.
                Ok(Some(desired_pos)) if desired_pos.room_name() != creep_pos.room_name() => {
                    leader_moves.insert(*entity, (creep_pos, None));
                    resolved_creeps.insert(*entity, stationary_occupant(*entity, creep_pos, request));
                    results.insert(*entity, MovementResult::Moving);
                }
                Ok(Some(desired_pos)) => {
                    leader_moves.insert(*entity, (creep_pos, Some(desired_pos)));
                    let creep_data = external.get_creep_movement_data(*entity).ok();
                    let stuck_ticks = creep_data
                        .and_then(|d| d.path_data.as_ref())
                        .map(|p| p.stuck_state.ticks_immobile as u32)
                        .unwrap_or(0);

                    // Shove-escalation wiring (opt-in; see `set_shove_escalation`): with the mode
                    // ON, this creep may INITIATE shoves only once its stuck ladder reached tier
                    // 3, and its per-request `enable_shoving` becomes the resolver's stuck gate
                    // (ONE knob). With the mode OFF both values reproduce today's behavior
                    // exactly (always-attempt + the historical constant).
                    let thresholds = self.thresholds_for(request);
                    let shove_enabled = request.allow_shove
                        && (!self.shove_escalation
                            || stuck_ticks >= thresholds.enable_shoving as u32);
                    let shove_stuck_threshold = if self.shove_escalation {
                        thresholds.enable_shoving as u32
                    } else {
                        STUCK_SHOVE_THRESHOLD
                    };

                    resolved_creeps.insert(
                        *entity,
                        ResolvedCreep {
                            entity: *entity,
                            current_pos: creep_pos,
                            desired_pos: Some(desired_pos),
                            priority: request.priority,
                            priority_value: request.effective_priority(),
                            allow_shove: request.allow_shove,
                            shove_enabled,
                            shove_stuck_threshold,
                            allow_swap: request.allow_swap,
                            stuck_ticks,
                            resolved: false,
                            final_pos: creep_pos,
                            has_request: true,
                            denied_by_idle: false,
                            anchor: request.anchor,
                        },
                    );
                }
                Ok(None) => {
                    leader_moves.insert(*entity, (creep_pos, None));

                    if request.allow_shove || request.allow_swap {
                        resolved_creeps.insert(
                            *entity,
                            ResolvedCreep {
                                entity: *entity,
                                current_pos: creep_pos,
                                desired_pos: None,
                                priority: request.priority,
                                priority_value: request.effective_priority(),
                                allow_shove: request.allow_shove,
                                // An arrived creep never initiates a shove (no desired tile), so
                                // the shover-side fields are inert; keep the OFF-mode constants.
                                shove_enabled: false,
                                shove_stuck_threshold: STUCK_SHOVE_THRESHOLD,
                                allow_swap: request.allow_swap,
                                stuck_ticks: 0,
                                resolved: false,
                                final_pos: creep_pos,
                                has_request: true,
                                denied_by_idle: false,
                                anchor: request.anchor,
                            },
                        );
                    } else {
                        results.insert(*entity, MovementResult::Arrived);
                    }
                }
                Err(err) => {
                    // A live requested creep whose next step could not be computed (typically
                    // `PathNotFound` under per-tick ops-budget exhaustion) still OCCUPIES its
                    // tile through the movement phase — the third instance of the Pass-1
                    // occupancy hole (fatigued, border-crosser, now path-error): dropped from
                    // the resolver's world, it was an invisible immovable post the resolver
                    // granted other creeps through, and the engine rejected every such intent
                    // (the dense-crowd `failed_coordination` flood — pathless creeps were the
                    // wedge nuclei of the N≥40 pinch livelock).
                    leader_moves.insert(*entity, (creep_pos, None));
                    resolved_creeps.insert(*entity, stationary_occupant(*entity, creep_pos, request));
                    results.insert(*entity, MovementResult::Failed(err));
                }
            }
            work_done_this_tick = true;
        }

        // --- Pass 2: Conflict resolution ---
        if !self.is_over_tick_limit() && !self.is_over_movement_cap() {
            // SHOVEABLE IDLES (ADR 0033 M5 follow-up #2): registered idle occupants get a
            // synthesized `ResolvedCreep` entry (`has_request: false`, LOWEST movable priority
            // anchor, shoveable) so `try_shove` can displace a corridor-mouth parker instead of
            // it being undisplaceable by construction (no entry -> try_shove returned false).
            // Their shove move surfaces through Pass 3 like any other resolved move. Entries are
            // inserted in position-sorted order with `or_insert` semantics so a (degenerate)
            // handle registered twice resolves to a pure function of the world, not HashMap
            // iteration order.
            let mut idle_entries: Vec<(Position, Handle)> = idle_creep_positions
                .iter()
                .map(|(pos, handle)| (*pos, *handle))
                .collect();
            idle_entries.sort_unstable_by_key(|(p, _)| (p.room_name(), p.x().u8(), p.y().u8()));
            for (pos, entity) in idle_entries {
                // Displacement consent requires the idle to be able to EXECUTE the evacuation
                // move this tick: a fatigued (or spawning) idle's issued move is engine-dropped
                // (`canMove`), which would also doom the shover's move into the tile — the same
                // wasted-intent pair the fatigued-requested fix above closes.
                let displaceable = external
                    .get_creep(entity)
                    .map(|c| c.fatigue() == 0 && !c.spawning())
                    .unwrap_or(false);
                resolved_creeps.entry(entity).or_insert_with(|| ResolvedCreep {
                    entity,
                    current_pos: pos,
                    desired_pos: None,
                    priority: MovementPriority::Low,
                    priority_value: MovementPriority::Low.anchor_value(),
                    // A parked creep consents to displacement (any mover with a real
                    // priority outranks the Low anchor); it never initiates shoves.
                    allow_shove: displaceable,
                    shove_enabled: false,
                    shove_stuck_threshold: STUCK_SHOVE_THRESHOLD,
                    allow_swap: false,
                    stuck_ticks: 0,
                    resolved: false,
                    final_pos: pos,
                    has_request: false,
                    denied_by_idle: false,
                    anchor: None,
                });
            }

            // Blocking-structure layer for shove/local-avoidance (IBEX-040):
            // terrain alone lets the resolver shove a creep onto a tile
            // occupied by a blocking structure (wall, spawn, hostile rampart)
            // or a blocked construction site, wasting the tick. Pre-build one
            // cached-layer cost matrix per involved room — the closure below
            // cannot call the &mut builder itself. Creep layers stay OFF: the
            // resolver models creep occupancy on its own, and a friendly-creep
            // MAX cost would wrongly veto shove/swap targets.
            let structure_options = CostMatrixOptions {
                structures: true,
                construction_sites: true,
                friendly_creeps: false,
                hostile_creeps: false,
                source_keeper_aggro: false,
                ..Default::default()
            };

            let involved_rooms: HashSet<RoomName> = resolved_creeps
                .values()
                .flat_map(|creep| {
                    std::iter::once(creep.current_pos.room_name())
                        .chain(creep.desired_pos.map(|pos| pos.room_name()))
                })
                .collect();

            let mut structure_costs: HashMap<RoomName, LocalCostMatrix> = HashMap::new();
            for room_name in involved_rooms {
                // Rooms without structure data (no visibility this tick) get
                // an empty matrix, which degrades to the terrain-only check.
                if let Ok(matrix) = self
                    .cost_matrix_system
                    .build_local_cost_matrix(room_name, &structure_options)
                {
                    structure_costs.insert(room_name, matrix);
                }
            }

            let pathfinder = &mut *self.pathfinder;
            let is_tile_walkable = |pos: Position| -> bool {
                if !pathfinder.is_tile_walkable(pos) {
                    return false;
                }
                structure_costs
                    .get(&pos.room_name())
                    .map(|matrix| matrix.get(pos.xy()) < u8::MAX)
                    .unwrap_or(true)
            };

            resolve_conflicts(
                &mut resolved_creeps,
                &idle_creep_positions,
                &is_tile_walkable,
                self.max_shove_depth,
            );
        }

        // --- Pass 3: Execute movement and record results ---
        let mut executed_one_move = false;
        for (entity, resolved) in &resolved_creeps {
            if (self.is_over_tick_limit() || self.is_over_movement_cap()) && executed_one_move {
                results.insert(*entity, MovementResult::Moving);
                continue;
            }
            if results.results.contains_key(entity) {
                continue;
            }

            // Synthesized idle occupants (no request this tick): the only way one surfaces is a
            // SHOVE — its displacement move must be issued exactly like a requested mover's
            // (both the offline driver and the live provider serve `get_creep` for any living
            // creep). An undisturbed idle gets no result entry: it has no job-side request to
            // answer, and running the stuck bookkeeping would fabricate cache state for it.
            if !resolved.has_request {
                if resolved.final_pos == resolved.current_pos {
                    continue;
                }
                if let Ok(creep) = external.get_creep(*entity) {
                    if let Some(dir) = resolved.current_pos.get_direction_to(resolved.final_pos) {
                        if creep.move_direction(dir).is_ok() {
                            results.insert(*entity, MovementResult::Moving);
                            executed_one_move = true;
                        }
                    }
                }
                continue;
            }

            // DENIAL-AS-STUCK (ADR 0033 M5 follow-up #2): an idle-occupant denial counts as
            // immobility EVEN IF the creep moved to an avoidance tile — mark the persisted stuck
            // state so next tick's path-progress accounting records IMMOBILE instead of resetting
            // the ladder (the sidestep DANCE must feed the escalation tiers; that is the whole
            // fix for the zero-failed-intent dance livelock). Stay-in-place denials also record
            // immobility below as before; the flag is what catches the *moved* case.
            if resolved.denied_by_idle {
                if let Ok(creep_data) = external.get_creep_movement_data(*entity) {
                    if let Some(path_data) = creep_data.path_data.as_mut() {
                        path_data.stuck_state.denied_by_idle = true;
                    }
                }
            }

            if resolved.final_pos == resolved.current_pos {
                if resolved.desired_pos.is_none() {
                    results.insert(*entity, MovementResult::Arrived);
                    continue;
                }

                // Per-request ladder: this entity's request may override the system thresholds
                // (resolved from the requests map — the resolved-creeps loop has no request in
                // scope directly).
                let report_thresholds = data
                    .requests
                    .get(entity)
                    .and_then(|r| r.stuck_thresholds.as_ref())
                    .unwrap_or(&self.stuck_thresholds);

                let stuck_ticks = if let Ok(creep_data) = external.get_creep_movement_data(*entity)
                {
                    if let Some(path_data) = creep_data.path_data.as_mut() {
                        let dist = resolved.current_pos.get_range_to(path_data.destination);
                        path_data.stuck_state.record_immobile(dist);

                        if path_data.stuck_state.should_report_failure_with(report_thresholds) {
                            results.insert(
                                *entity,
                                MovementResult::Failed(MovementFailure::StuckTimeout {
                                    ticks: path_data.stuck_state.ticks_immobile,
                                }),
                            );
                            continue;
                        }

                        path_data.stuck_state.ticks_immobile
                    } else {
                        1
                    }
                } else {
                    1
                };

                results.insert(*entity, MovementResult::Stuck { ticks: stuck_ticks });
                continue;
            }

            // Execute the move.
            let creep = match external.get_creep(*entity) {
                Ok(c) => c,
                Err(err) => {
                    results.insert(
                        *entity,
                        MovementResult::Failed(MovementFailure::InternalError(err)),
                    );
                    continue;
                }
            };

            // Check if this is a pull follower.
            if let Some(leader_handle) = pull_pairs.get(entity) {
                if let Ok(leader_creep) = external.get_creep(*leader_handle) {
                    let _ = leader_creep.pull(&creep);
                    let _ = creep.move_pulled_by(&leader_creep);
                    results.insert(*entity, MovementResult::Moving);
                    executed_one_move = true;
                    continue;
                }
            }

            let direction = resolved.current_pos.get_direction_to(resolved.final_pos);

            match direction {
                Some(dir) => match creep.move_direction(dir) {
                    Ok(()) => {
                        results.insert(*entity, MovementResult::Moving);
                        executed_one_move = true;
                    }
                    Err(e) => {
                        results.insert(
                            *entity,
                            MovementResult::Failed(MovementFailure::InternalError(format!(
                                "move_direction error: {:?}",
                                e
                            ))),
                        );
                    }
                },
                None => {
                    results.insert(*entity, MovementResult::Moving);
                }
            }
        }

        // --- Visualization ---
        if self.visualizer.is_some() && !self.is_over_tick_limit() && !self.is_over_movement_cap() {
            for entity in sorted_entities.iter() {
                if self.is_over_tick_limit() || self.is_over_movement_cap() {
                    break;
                }
                if let Some(request) = data.requests.get(entity) {
                    let result = results.get(entity);
                    self.visualize_entity(external, *entity, request, result);
                }
            }
        }

        results
    }

    /// Compute the next step for a MoveTo intent, handling path caching and generation.
    fn compute_next_step_for_move_to<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        creep_pos: Position,
        destination: Position,
        range: u32,
        request: &MovementRequest<Handle>,
    ) -> Result<Option<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        // Validate and reuse cached path.
        {
            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;

            if let Some(path_data) = &mut creep_data.path_data {
                let dest_matches = path_data.destination == destination && path_data.range == range;

                if !dest_matches {
                    creep_data.path_data = None;
                } else {
                    let on_path = path_data.path.iter().take(2).any(|p| *p == creep_pos);

                    if !on_path {
                        // Find the furthest path position (within a small
                        // window) adjacent to the creep. Using the furthest
                        // match lets us skip past tiles the creep already
                        // bypassed via local avoidance, preventing backtracking.
                        let mut best_nearby: Option<usize> = None;
                        for (i, p) in path_data.path.iter().take(4).enumerate() {
                            if creep_pos.get_range_to(*p) <= 1 {
                                best_nearby = Some(i);
                            }
                        }

                        if let Some(idx) = best_nearby {
                            path_data.path.drain(..idx);
                        } else {
                            creep_data.path_data = None;
                        }
                    }
                }
            }
        }

        // Track movement and stuck state.
        let current_distance = creep_pos.get_range_to(destination);
        let (path_time, stuck_state_snapshot) = {
            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;

            if let Some(path_data) = creep_data.path_data.as_mut() {
                path_data.time += 1;

                let path = &mut path_data.path;

                let current_index = path
                    .iter()
                    .take(2)
                    .enumerate()
                    .find(|(_, p)| **p == creep_pos)
                    .map(|(index, _)| index);

                let (effective_index, was_shoved_off) = match current_index {
                    Some(idx) => (Some(idx), false),
                    None => {
                        // Creep is not exactly on the path. This happens after
                        // local avoidance (side-step) or a shove. Find the
                        // furthest path position (within a small window) that
                        // the creep is adjacent to, so we skip past the tile
                        // that was blocked and avoid backtracking.
                        let mut best_adjacent: Option<usize> = None;
                        for (i, p) in path.iter().take(4).enumerate() {
                            if creep_pos.get_range_to(*p) <= 1 {
                                best_adjacent = Some(i);
                            }
                        }
                        if let Some(idx) = best_adjacent {
                            (Some(idx), true)
                        } else {
                            (None, false)
                        }
                    }
                };

                match effective_index {
                    Some(idx) => {
                        let moved = idx > 0;
                        path.drain(..idx);

                        if path.len() <= 1 {
                            // Path exhausted. If the creep is within range of
                            // the destination, it genuinely arrived. Otherwise
                            // the path was over-consumed (e.g. aggressive
                            // skip-ahead after local avoidance) and we need a
                            // fresh path from the current position.
                            if current_distance <= path_data.range {
                                return Ok(None);
                            }
                            // Clear path so a new one is generated below.
                            creep_data.path_data = None;
                            (None, None)
                        } else {
                            // DENIAL-AS-STUCK: a resolver denial by an idle occupant last tick
                            // (flag set in Pass 3) forces IMMOBILE accounting even when the
                            // creep physically advanced — the avoidance sidestep is a denial in
                            // motion, and it must feed the escalation ladder instead of
                            // resetting it every dance step. Consumed here (one tick's worth).
                            let denied_last_tick =
                                std::mem::take(&mut path_data.stuck_state.denied_by_idle);

                            if moved && !denied_last_tick {
                                // The creep advanced along the path — either it
                                // walked normally or it side-stepped via local
                                // avoidance and ended up adjacent to a further
                                // path position. Either way, it made progress.
                                path_data.stuck_state.record_moved(current_distance);
                            } else {
                                // Off-path without advancing (shoved sideways), on-path at the
                                // same position as last tick, or an idle-denied sidestep —
                                // all immobility for ladder purposes.
                                let _ = was_shoved_off;
                                path_data.stuck_state.record_immobile(current_distance);
                            }

                            (Some(path_data.time), Some(path_data.stuck_state.clone()))
                        }
                    }
                    None => {
                        creep_data.path_data = None;
                        (None, None)
                    }
                }
            } else {
                (None, None)
            }
        };

        let path_expired = path_time
            .map(|t| t >= self.reuse_path_length)
            .unwrap_or(false);
        let stuck_needs_repath = stuck_state_snapshot
            .as_ref()
            .map(|s| s.should_stuck_repath_with(self.thresholds_for(request)))
            .unwrap_or(false);
        let stuck_state_for_gen = stuck_state_snapshot.unwrap_or_default();

        let needs_path = {
            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;
            creep_data.path_data.is_none()
        };

        // Determine whether to pathfind, respecting CPU budgets.
        //
        // Priority:
        //   1. needs_path (no path at all) -- always pathfind, unconditional.
        //   2. stuck_needs_repath -- pathfind unless hard limit is hit.
        //   3. path_expired -- pathfind only if repath budget remains AND
        //      hard limit not hit. This is the lowest priority.
        //
        // When pathfinding is skipped for budget reasons, the path timer
        // resets so the creep continues on its existing path without
        // re-triggering on the very next tick.
        //
        // FIRST-PATH POOL RESERVE: repaths (stuck + expiry) are OPTIONAL work — the creep still
        // holds a usable path — while a `needs_path` creep cannot move AT ALL without a search.
        // Skipping optional searches once the remaining per-tick ops pool drops into the reserve
        // guarantees first-paths always find real budget. Without it a dense stuck crowd's
        // repaths drained the pool to dregs every tick and mid-order first-path searches ran
        // budget-incomplete FOREVER (permanently pathless, permanently immobile — the wedge
        // nuclei of the rover-eval dense-crowd livelock, ADR 0004's death-spiral class).
        let reserve = self.pathfinding_ops_budget_cap / NEEDS_PATH_OPS_RESERVE_DIV;
        let pool_reserved = self.pathfinding_ops_budget_remaining <= reserve;
        let should_pathfind = if needs_path {
            true
        } else if stuck_needs_repath {
            if self.is_cpu_budget_exhausted() || pool_reserved {
                // CPU budget exhausted (or the pool is down to the first-path reserve): skip
                // the stuck repath, keep the existing path.
                if let Ok(creep_data) = external.get_creep_movement_data(entity) {
                    if let Some(path_data) = creep_data.path_data.as_mut() {
                        path_data.time = 0;
                    }
                }
                false
            } else {
                true
            }
        } else if path_expired {
            if self.is_cpu_budget_exhausted() || self.is_repath_budget_exhausted() || pool_reserved {
                // CPU budget or repath budget exhausted (or the pool is down to the first-path
                // reserve): skip expiry repath, reset timer and keep existing path.
                if let Ok(creep_data) = external.get_creep_movement_data(entity) {
                    if let Some(path_data) = creep_data.path_data.as_mut() {
                        path_data.time = 0;
                    }
                }
                false
            } else {
                true
            }
        } else {
            false
        };

        if should_pathfind {
            match self.generate_path(
                external,
                destination,
                range,
                request,
                creep_pos,
                &stuck_state_for_gen,
            ) {
                Ok(path_points) => {
                    let creep_data = external
                        .get_creep_movement_data(entity)
                        .map_err(MovementFailure::InternalError)?;

                    let mut new_stuck_state = stuck_state_for_gen.clone();
                    new_stuck_state.repath_count = new_stuck_state.repath_count.saturating_add(1);
                    new_stuck_state.ticks_since_repath = 0; // the damper clock re-arms per attempt
                    self.repaths_this_tick = self.repaths_this_tick.saturating_add(1);

                    creep_data.path_data = Some(CreepPathData {
                        destination,
                        range,
                        path: path_points,
                        time: 0,
                        stuck_state: new_stuck_state,
                    });
                }
                Err(failure) => {
                    if needs_path {
                        // No existing path to fall back to -- propagate the error.
                        return Err(failure);
                    }
                    // A stuck-triggered or expiry repath failed (e.g. friendly
                    // creeps made the only corridor impassable). Keep the existing
                    // path so the resolver's shove/swap/local-avoidance mechanisms
                    // can clear the blockage as the stuck timer escalates. Without
                    // this fallback the creep would immediately report PathNotFound
                    // to the job layer, short-circuiting the tiered recovery.
                    //
                    // Reset the path timer so we don't immediately re-attempt a
                    // doomed repath on the very next tick. The stuck timer still
                    // advances, so escalation (shove/swap) continues normally. The
                    // FAILED attempt also arms the storm damper (clock + episode
                    // count) — a doomed search must back off, not re-fire every tick
                    // (the dense-crowd ops-saturation class).
                    if let Ok(creep_data) = external.get_creep_movement_data(entity) {
                        if let Some(path_data) = creep_data.path_data.as_mut() {
                            path_data.time = 0;
                            path_data.stuck_state.ticks_since_repath = 0;
                            path_data.stuck_state.repath_count =
                                path_data.stuck_state.repath_count.saturating_add(1);
                        }
                    }
                }
            }
        }

        // Extract next step.
        let creep_data = external
            .get_creep_movement_data(entity)
            .map_err(MovementFailure::InternalError)?;
        let path_data = creep_data
            .path_data
            .as_ref()
            .ok_or(MovementFailure::PathNotFound)?;

        let next_pos = if path_data.path.first() == Some(&creep_pos) {
            path_data.path.get(1).copied()
        } else {
            path_data.path.first().copied()
        };

        Ok(next_pos)
    }

    /// Compute the next step for a Follow intent.
    ///
    /// When `desired_offset` is `Some((dx, dy))`, the follower prefers the
    /// tile at `(leader_new_pos + offset)` instead of the leader's vacated
    /// tile.  This lets quad members maintain a 2×2 formation: each follower
    /// has a unique offset so they don't all compete for the same tile.
    ///
    /// If the offset tile is unreachable in one step (or out of bounds), the
    /// follower falls back to the default behaviour (vacated tile or
    /// pathfinding toward the leader).
    #[allow(clippy::too_many_arguments)]
    fn compute_next_step_for_follow<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        creep_pos: Position,
        target: Handle,
        range: u32,
        desired_offset: Option<(i32, i32)>,
        request: &MovementRequest<Handle>,
        leader_moves: &HashMap<Handle, (Position, Option<Position>)>,
    ) -> Result<Option<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        let (leader_old_pos, leader_new_pos) = match leader_moves.get(&target) {
            Some(positions) => *positions,
            None => match external.get_entity_position(target) {
                Some(pos) => (pos, None),
                None => {
                    return Err(MovementFailure::InternalError(
                        "Follow target entity not found".to_owned(),
                    ))
                }
            },
        };

        let leader_is_moving = leader_new_pos.is_some() && leader_new_pos != Some(leader_old_pos);
        let leader_dest = leader_new_pos.unwrap_or(leader_old_pos);

        // If a desired offset is set, try to reach the offset tile relative
        // to the leader's destination.  This is the primary mechanism for
        // maintaining a 2×2 quad shape.
        if let Some((dx, dy)) = desired_offset {
            if let Ok(offset_pos) = leader_dest.checked_add((dx, dy)) {
                // Already at the desired position – stay put.
                if creep_pos == offset_pos {
                    return Ok(None);
                }

                // If the offset tile is one step away, move there directly.
                if creep_pos.get_range_to(offset_pos) <= 1
                    && creep_pos.room_name() == offset_pos.room_name()
                {
                    return Ok(Some(offset_pos));
                }

                // Otherwise pathfind toward the offset tile.
                return self.compute_next_step_for_move_to(
                    external, entity, creep_pos, offset_pos, 0, request,
                );
            }
            // Offset out of bounds – fall through to default behaviour.
        }

        if leader_is_moving {
            let vacated_tile = leader_old_pos;

            if creep_pos.get_range_to(vacated_tile) <= 1 {
                if creep_pos == vacated_tile {
                    return Ok(None);
                }
                return Ok(Some(vacated_tile));
            }
            self.compute_next_step_for_move_to(
                external,
                entity,
                creep_pos,
                leader_dest,
                range,
                request,
            )
        } else {
            if creep_pos.get_range_to(leader_old_pos) <= range {
                return Ok(None);
            }
            self.compute_next_step_for_move_to(
                external,
                entity,
                creep_pos,
                leader_old_pos,
                range,
                request,
            )
        }
    }

    /// Compute the next step for a Flee intent.
    fn compute_next_step_for_flee(
        &mut self,
        creep_pos: Position,
        targets: &[FleeTarget],
        _range: u32,
        request: &MovementRequest<Handle>,
    ) -> Result<Option<Position>, MovementFailure> {
        if targets.is_empty() {
            return Ok(None);
        }

        let already_safe = targets
            .iter()
            .all(|t| creep_pos.get_range_to(t.pos) >= t.range);

        if already_safe {
            return Ok(None);
        }

        if self.is_over_tick_limit() {
            return Err(MovementFailure::PathNotFound);
        }

        let goals: Vec<(Position, u32)> = targets.iter().map(|t| (t.pos, t.range)).collect();

        let flee_ops = 2000u32;
        let mut allowed_ops = flee_ops.min(self.pathfinding_ops_budget_remaining);
        if let Some((get_cpu, limit)) = &self.tick_limit {
            let cpu_left = ((*limit - (get_cpu)()).max(0.0) * 1000.0) as u32;
            allowed_ops = allowed_ops.min(cpu_left);
        }
        if allowed_ops == 0 {
            return Err(MovementFailure::PathNotFound);
        }
        self.pathfinding_ops_budget_remaining -= allowed_ops;

        let cost_matrix_options = request.cost_matrix_options.unwrap_or_default();
        let cost_matrix_system = &mut self.cost_matrix_system;
        let creep_room_name = creep_pos.room_name();

        let result = self.pathfinder.search_many(
            creep_pos,
            &goals,
            true,
            &mut |room_name: RoomName| -> Option<LocalCostMatrix> {
                let distance = super::utility::room_linear_distance(creep_room_name, room_name);
                if distance > 2 {
                    return None;
                }
                cost_matrix_system
                    .build_local_cost_matrix(room_name, &cost_matrix_options)
                    .ok()
            },
            allowed_ops,
            cost_matrix_options.plains_cost,
            cost_matrix_options.swamp_cost,
        );

        if result.incomplete || result.path.is_empty() {
            return Err(MovementFailure::PathNotFound);
        }

        Ok(result.path.first().copied())
    }

    fn generate_path<S>(
        &mut self,
        external: &mut S,
        destination: Position,
        range: u32,
        request: &MovementRequest<Handle>,
        creep_pos: Position,
        stuck_state: &StuckState,
    ) -> Result<Vec<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        if self.is_over_tick_limit() {
            return Err(MovementFailure::PathNotFound);
        }
        if self.is_over_movement_cap() {
            return Err(MovementFailure::PathNotFound);
        }
        // Do not start pathfinding unless we have at least headroom CPU left under the cap (find_route is unbounded).
        if let (Some((get_cpu, start, max)), Some(headroom)) = (
            self.movement_cpu_cap.as_ref(),
            self.pathfinding_headroom,
        ) {
            let used = (get_cpu)() - start;
            if used + headroom > *max {
                return Err(MovementFailure::PathNotFound);
            }
        }

        let creep_room_name = creep_pos.room_name();
        let room_options = request.room_options.unwrap_or_default();
        let destination_room = destination.room_name();

        let room_path = self
            .pathfinder
            .find_route(
                creep_room_name,
                destination.room_name(),
                &|to_room_name, from_room_name| {
                    external
                        .get_room_cost(from_room_name, to_room_name, &room_options)
                        .unwrap_or(f64::INFINITY)
                },
            )
            .map_err(|e| {
                MovementFailure::InternalError(format!(
                    "Could not find path between rooms: {:?}",
                    e
                ))
            })?;

        // Cap rooms so we don't build cost matrices for 50+ rooms or give pathfinder
        // huge max_ops when the destination is far/impossible.
        let room_names: HashSet<_> = room_path
            .iter()
            .take(MAX_PATHFIND_ROOMS.saturating_sub(2)) // leave room for origin + dest
            .map(|step| step.room)
            .chain(std::iter::once(creep_room_name))
            .chain(std::iter::once(destination_room))
            .collect();

        let mut cost_matrix_options = request.cost_matrix_options.unwrap_or_default();

        // Per-request ladder (request-override else system default).
        let thresholds = self.thresholds_for(request);

        if stuck_state.should_avoid_all_friendly_creeps_with(thresholds) {
            // Tier 1b: avoid ALL friendly creeps in every room (escalation).
            cost_matrix_options.friendly_creeps = true;
            cost_matrix_options.friendly_creep_proximity = None;
        } else if stuck_state.should_avoid_friendly_creeps_with(thresholds) {
            // Tier 1: avoid friendly creeps only within a tile radius of the
            // creep. Creeps further away will have moved by the time we
            // arrive, so including them produces sub-optimal detours.
            cost_matrix_options.friendly_creeps = true;
            if self.friendly_creep_distance > 0 {
                cost_matrix_options.friendly_creep_proximity =
                    Some(FriendlyCreepProximity {
                        origin: creep_pos,
                        distance: self.friendly_creep_distance,
                    });
            }
        }

        let ops_multiplier = if stuck_state.should_increase_ops_with(thresholds) {
            2
        } else {
            1
        };
        let max_ops = (room_names.len() as u32 * 2000 * ops_multiplier).min(MAX_PATHFIND_OPS);

        // Deduct from per-tick pathfinding ops budget (1 op ≈ 0.001 CPU).
        let mut allowed_ops = max_ops.min(self.pathfinding_ops_budget_remaining);
        if let Some((get_cpu, limit)) = &self.tick_limit {
            let cpu_left = ((*limit - (get_cpu)()).max(0.0) * 1000.0) as u32;
            allowed_ops = allowed_ops.min(cpu_left);
        }
        if allowed_ops == 0 {
            return Err(MovementFailure::PathNotFound);
        }
        self.pathfinding_ops_budget_remaining -= allowed_ops;

        if self.is_over_tick_limit() {
            return Err(MovementFailure::PathNotFound);
        }

        let tick_check = self.tick_limit.as_ref().map(|(g, l)| (&**g, *l));
        let cost_matrix_system = &mut self.cost_matrix_system;

        let result = self.pathfinder.search(
            creep_pos,
            destination,
            range,
            &mut |room_name: RoomName| -> Option<LocalCostMatrix> {
                if let Some((get_cpu, limit)) = tick_check {
                    if (get_cpu)() >= limit {
                        return None;
                    }
                }
                if room_names.contains(&room_name) {
                    cost_matrix_system
                        .build_local_cost_matrix(room_name, &cost_matrix_options)
                        .ok()
                } else {
                    None
                }
            },
            allowed_ops,
            cost_matrix_options.plains_cost,
            cost_matrix_options.swamp_cost,
        );

        if result.incomplete {
            return Err(MovementFailure::PathNotFound);
        }

        let mut path_points = result.path;
        path_points.insert(0, creep_pos);

        Ok(path_points)
    }

    /// Report per-entity visualization intents to the visualizer.
    fn visualize_entity<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        request: &MovementRequest<Handle>,
        result: Option<&MovementResult>,
    ) where
        S: MovementSystemExternal<Handle>,
    {
        if !request.visualize {
            return;
        }

        let visualizer = match self.visualizer.as_deref_mut() {
            Some(v) => v,
            None => return,
        };

        let creep_pos = match external.get_creep(entity) {
            Ok(creep) => creep.pos(),
            Err(_) => return,
        };

        let room_name = creep_pos.room_name();

        match result {
            Some(MovementResult::Moving) => {
                let path: Vec<Position> = match external.get_creep_movement_data(entity) {
                    Ok(creep_data) => {
                        if let Some(path_data) = &creep_data.path_data {
                            path_data
                                .path
                                .iter()
                                .take_while(|p| p.room_name() == room_name)
                                .copied()
                                .collect()
                        } else {
                            return;
                        }
                    }
                    Err(_) => return,
                };

                visualizer.visualize_path(creep_pos, &path);
            }

            Some(MovementResult::Arrived) => {
                if request.priority == MovementPriority::Immovable {
                    visualizer.visualize_immovable(creep_pos);
                } else if let Some(anchor) = &request.anchor {
                    visualizer.visualize_anchor(creep_pos, anchor.position);
                }
            }

            Some(MovementResult::Stuck { ticks }) => {
                visualizer.visualize_stuck(creep_pos, *ticks);
            }

            Some(MovementResult::Failed(_)) => {
                visualizer.visualize_failed(creep_pos);
            }

            None => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resolver::DirectionExt;
    use screeps::constants::Direction;
    use std::cell::RefCell;
    use std::rc::Rc;

    fn pos(x: u8, y: u8) -> Position {
        Position::new(
            RoomCoordinate::new(x).unwrap(),
            RoomCoordinate::new(y).unwrap(),
            "W1N1".parse().unwrap(),
        )
    }

    /// Cost source with no game data — every layer is `None`, so cost-matrix
    /// builds degrade to terrain-only (the headless-consumer shape).
    struct NullCostSource;
    impl CostMatrixDataSource for NullCostSource {
        fn get_structure_costs(&self, _r: RoomName) -> Option<StuctureCostMatrixCache> {
            None
        }
        fn get_construction_site_costs(&self, _r: RoomName) -> Option<ConstructionSiteCostMatrixCache> {
            None
        }
        fn get_creep_costs(&self, _r: RoomName) -> Option<CreepCostMatrixCache> {
            None
        }
    }

    /// Counts `search` calls (plus each search's origin, for per-creep attribution) and returns a
    /// straight horizontal path origin→goal (same room, same y — the fixture worlds below keep y
    /// fixed). Open terrain everywhere.
    struct CountingPathfinder {
        searches: u32,
        origins: Vec<Position>,
    }
    impl PathfindingProvider for CountingPathfinder {
        fn search(
            &mut self,
            origin: Position,
            goal: Position,
            _range: u32,
            _room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
            _max_ops: u32,
            _plain_cost: u8,
            _swamp_cost: u8,
        ) -> PathfindingResult {
            self.searches += 1;
            self.origins.push(origin);
            let y = origin.y().u8();
            let room = origin.room_name();
            let path = (origin.x().u8() + 1..=goal.x().u8())
                .map(|x| {
                    Position::new(
                        RoomCoordinate::new(x).unwrap(),
                        RoomCoordinate::new(y).unwrap(),
                        room,
                    )
                })
                .collect();
            PathfindingResult { path, incomplete: false }
        }
        fn search_many(
            &mut self,
            _origin: Position,
            _goals: &[(Position, u32)],
            _flee: bool,
            _room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
            _max_ops: u32,
            _plain_cost: u8,
            _swamp_cost: u8,
        ) -> PathfindingResult {
            PathfindingResult { path: Vec::new(), incomplete: true }
        }
        fn find_route(
            &self,
            _from: RoomName,
            _to: RoomName,
            _room_callback: &dyn Fn(RoomName, RoomName) -> f64,
        ) -> Result<Vec<RouteStep>, String> {
            Ok(Vec::new()) // same-room fixtures: no inter-room legs
        }
        fn get_room_linear_distance(&self, _from: RoomName, _to: RoomName) -> u32 {
            0
        }
        fn is_tile_walkable(&self, _pos: Position) -> bool {
            true
        }
    }

    /// Records each creep's issued `move_direction` into a shared sink (the headless analogue of
    /// `creep.move(dir)` — same idiom as sim-core's `SimCreepHandle`).
    struct TestCreep {
        id: u32,
        pos: Position,
        sink: Rc<RefCell<HashMap<u32, Direction>>>,
    }
    impl CreepHandle for TestCreep {
        fn pos(&self) -> Position {
            self.pos
        }
        fn fatigue(&self) -> u32 {
            0
        }
        fn spawning(&self) -> bool {
            false
        }
        fn move_direction(&self, dir: Direction) -> Result<(), String> {
            self.sink.borrow_mut().insert(self.id, dir);
            Ok(())
        }
        fn pull(&self, _other: &Self) -> Result<(), String> {
            Ok(())
        }
        fn move_pulled_by(&self, _other: &Self) -> Result<(), String> {
            Ok(())
        }
    }

    /// Fixed-position external: the creep never actually moves (the engine "rejects" every move),
    /// which is exactly the immobile shape that must escalate through the stuck ladder.
    struct StubExternal {
        positions: HashMap<u32, Position>,
        data: HashMap<u32, CreepMovementData>,
        sink: Rc<RefCell<HashMap<u32, Direction>>>,
    }
    impl MovementSystemExternal<u32> for StubExternal {
        type Creep = TestCreep;
        fn get_creep(&self, entity: u32) -> Result<TestCreep, MovementError> {
            let pos = *self.positions.get(&entity).ok_or("no creep".to_owned())?;
            Ok(TestCreep { id: entity, pos, sink: self.sink.clone() })
        }
        fn get_creep_movement_data(&mut self, entity: u32) -> Result<&mut CreepMovementData, MovementError> {
            Ok(self.data.entry(entity).or_default())
        }
        fn get_entity_position(&self, entity: u32) -> Option<Position> {
            self.positions.get(&entity).copied()
        }
    }

    // The three limit predicates must agree that ABSENT means "never binds" (`is_some_and`).
    // Regression for the 2026-07-01 semantics fix: `is_none_or` made an absent budget EXHAUSTED,
    // silently disabling stuck-escalation + expiry repathing for budget-less consumers (F1).
    #[test]
    fn absent_budgets_mean_unlimited_not_exhausted() {
        let mut cache = CostMatrixCache::default();
        let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
        let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };
        let mut system: MovementSystem<'_, u32> = MovementSystem::new(&mut cms, &mut pf, None);

        assert!(!system.is_cpu_budget_exhausted(), "absent CPU budget = unlimited");
        assert!(!system.is_repath_budget_exhausted(), "absent repath budget = unlimited");
        assert!(!system.is_over_tick_limit(), "absent tick limit = unlimited (unchanged)");

        // A budget that IS set still binds: zero budget is exhausted immediately.
        system.set_cpu_budget(|| 1.0, 0.0);
        system.set_repath_budget(|| 1.0, 0.0);
        assert!(system.is_cpu_budget_exhausted());
        assert!(system.is_repath_budget_exhausted());
    }

    // End-to-end F1 regression: a permanently blocked creep, driven by a consumer that never sets
    // any budget, must still escalate into stuck repathing. Under the old `is_none_or` semantics
    // the pathfinder ran exactly once (the unconditional needs_path) and never again — the creep
    // re-issued its rejected move forever (the offline permanent-livelock class).
    #[test]
    fn stuck_repath_fires_without_any_budget_set() {
        let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
        let mut external = StubExternal {
            positions: [(1u32, pos(10, 25))].into_iter().collect(),
            data: HashMap::new(),
            sink: sink.clone(),
        };
        let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };

        // The system is rebuilt per tick (the live shape); external + pathfinder persist.
        for _ in 0..6 {
            let mut cache = CostMatrixCache::default();
            let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
            let mut system = MovementSystem::new(&mut cms, &mut pf, None);
            // Deliberately NO set_cpu_budget / set_repath_budget: the semantics under test.
            let mut data = MovementData::new();
            data.move_to(1u32, pos(20, 25));
            system.process(&mut external, data);
        }

        // needs_path (tick 1) + at least one stuck repath once ticks_immobile reaches
        // avoid_friendly_creeps (2). Old semantics: stuck at exactly 1.
        assert!(
            pf.searches >= 2,
            "a budget-less consumer must still stuck-repath (searches: {})",
            pf.searches
        );
    }

    // F2 end-to-end at the process() seam, post shoveable-idles (ADR 0033 M5 follow-up #2):
    // a parked creep registered via set_idle_creep_positions gets a synthesized shoveable entry
    // at the LOWEST movable anchor, so
    //   - a Normal mover DISPLACES it: the mover is granted the tile AND the idle is issued its
    //     evacuation move the same tick (a consistent pair — no engine-rejected intent);
    //   - a Low mover (ties the idle's anchor; the try_shove priority gate denies at stuck 0)
    //     is denied and sidesteps via local avoidance instead of pathing into the occupant;
    //   - the UNREGISTERED baseline still optimistically issues the doomed move into the tile
    //     (the `failed_into_parked` class this registration exists to eliminate).
    #[test]
    fn registered_idle_occupant_is_displaced_or_sidestepped_never_collided_with() {
        let parked = pos(15, 25);
        let mover_from = pos(14, 25);

        // Returns (mover's issued target, the idle's issued direction if any).
        let run = |register: bool, priority: Option<MovementPriority>| {
            let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
            let mut external = StubExternal {
                // The idle creep 99 is a LIVING creep the external can serve — it is only
                // absent from the REQUEST set (the parked, goal-reached shape).
                positions: [(1u32, mover_from), (99u32, parked)].into_iter().collect(),
                data: HashMap::new(),
                sink: sink.clone(),
            };
            let mut cache = CostMatrixCache::default();
            let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
            let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };
            let mut system = MovementSystem::new(&mut cms, &mut pf, None);
            if register {
                system.set_idle_creep_positions([(parked, 99u32)].into_iter().collect());
            }
            let mut data = MovementData::new();
            {
                let mut mr = data.move_to(1u32, pos(20, 25));
                if let Some(p) = priority {
                    mr.priority(p);
                }
            }
            system.process(&mut external, data);

            let mover_target = sink.borrow().get(&1).copied().map(|dir| {
                let off = dir.into_offset();
                pos(
                    (mover_from.x().u8() as i32 + off.0) as u8,
                    (mover_from.y().u8() as i32 + off.1) as u8,
                )
            });
            let idle_dir = sink.borrow().get(&99).copied();
            (mover_target, idle_dir)
        };

        // Baseline (no registration): the optimistic path runs straight through the parked tile
        // and the resolver cannot know better — the issued move targets it (engine would
        // reject), and the idle is of course never moved.
        let (target, idle_dir) = run(false, None);
        assert_eq!(target, Some(parked), "unregistered baseline paths into the parked tile");
        assert!(idle_dir.is_none(), "unregistered idle cannot be displaced");

        // Registered + Normal mover: the idle (Low anchor) is SHOVED — both moves are issued as
        // a consistent pair (idle evacuates, mover takes the tile).
        let (target, idle_dir) = run(true, None);
        assert_eq!(
            target,
            Some(parked),
            "a Normal mover displaces the registered idle and takes its tile"
        );
        assert!(
            idle_dir.is_some(),
            "the displaced idle must be issued its evacuation move the same tick"
        );

        // Registered + Low mover: the shove priority gate denies (Low ties the idle's anchor,
        // stuck 0 < threshold) — the mover sidesteps, the idle stays.
        let (target, idle_dir) = run(true, Some(MovementPriority::Low));
        let target = target.expect("denied mover must still be issued a sidestep, not stall");
        assert_ne!(target, parked, "a denied mover must not be issued into the occupied tile");
        assert!(idle_dir.is_none(), "the undisturbed idle gets no move");
    }

    // Per-request StuckThresholds (ADR 0033 M5 split-defaults end-state): a request-level ladder
    // override must change escalation timing for THAT creep only. Two permanently blocked creeps
    // (StubExternal never moves them) run side by side: the default-ladder creep starts stuck
    // repathing once ticks_immobile >= avoid_friendly_creeps (2); the overridden creep (a slow
    // ladder, tier 1 at 6) must not have repathed yet by then. Attribution is by search origin
    // (the two creeps live on different rows).
    #[test]
    fn request_level_ladder_override_changes_escalation_timing_per_creep() {
        let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
        let default_pos = pos(10, 20);
        let slow_pos = pos(10, 30);
        let mut external = StubExternal {
            positions: [(1u32, default_pos), (2u32, slow_pos)].into_iter().collect(),
            data: HashMap::new(),
            sink: sink.clone(),
        };
        let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };

        let slow_ladder = StuckThresholds {
            avoid_friendly_creeps: 6,
            avoid_all_friendly_creeps: 8,
            increase_ops: 10,
            enable_shoving: 12,
            report_failure: 24,
            no_progress_repath: 30,
        };

        // 5 ticks: the default creep's ticks_immobile reaches tier 1 (2) inside the window and
        // stuck-repaths; the slow creep's tier 1 (6) never trips.
        for _ in 0..5 {
            let mut cache = CostMatrixCache::default();
            let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
            let mut system = MovementSystem::new(&mut cms, &mut pf, None);
            let mut data = MovementData::new();
            data.move_to(1u32, pos(20, 20));
            data.move_to(2u32, pos(20, 30)).stuck_thresholds(slow_ladder.clone());
            system.process(&mut external, data);
        }

        let searches_default = pf.origins.iter().filter(|p| **p == default_pos).count();
        let searches_slow = pf.origins.iter().filter(|p| **p == slow_pos).count();
        assert!(
            searches_default >= 2,
            "default ladder must stuck-repath within the window (searches: {})",
            searches_default
        );
        assert_eq!(
            searches_slow, 1,
            "the overridden creep must only have its unconditional first path — its slow ladder \
             has not reached tier 1 yet (searches: {})",
            searches_slow
        );
    }

    // Escalation-tier shoving (the previously dead tier 3, opt-in via set_shove_escalation):
    //  - OFF (default): today's behavior exactly — a High-priority mover displaces a Normal
    //    arrived occupant on the FIRST tick (priority-superior, no stuck requirement);
    //  - ON: shove becomes an escalation — the same mover may not displace anyone until its own
    //    ticks_immobile reaches tier 3 (`enable_shoving`), which is also the resolver's stuck
    //    gate (one knob). The mover sidesteps (local avoidance) in the meantime.
    // StubExternal freezes positions, so the mover's immobility accrues tick over tick.
    #[test]
    fn shove_escalation_gates_displacement_on_tier_three() {
        let mover_from = pos(14, 25);
        let occupant_at = pos(15, 25);

        // Returns the 1-based tick at which the occupant was first issued a move (shoved).
        let first_shove_tick = |escalation: bool, ticks: u32| -> Option<u32> {
            let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
            let mut external = StubExternal {
                positions: [(1u32, mover_from), (2u32, occupant_at)].into_iter().collect(),
                data: HashMap::new(),
                sink: sink.clone(),
            };
            let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };
            for tick in 1..=ticks {
                let mut cache = CostMatrixCache::default();
                let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
                let mut system = MovementSystem::new(&mut cms, &mut pf, None);
                system.set_shove_escalation(escalation);
                let mut data = MovementData::new();
                data.move_to(1u32, pos(20, 25)).priority(MovementPriority::High);
                // The occupant is ARRIVED at its own destination (a stationed worker): it keeps
                // a request (so it is an ACTIVE, shove-consenting entry) but never moves itself.
                data.move_to(2u32, occupant_at);
                sink.borrow_mut().clear();
                system.process(&mut external, data);
                if sink.borrow().contains_key(&2) {
                    return Some(tick);
                }
            }
            None
        };

        // OFF: priority-superiority shoves immediately (the historical behavior, preserved).
        assert_eq!(
            first_shove_tick(false, 4),
            Some(1),
            "escalation OFF must preserve first-tick priority shoving"
        );

        // ON: nothing may be displaced until the mover's own tier 3 (default enable_shoving = 7)
        // fires; with frozen positions ticks_immobile reaches 7 around tick 8-9.
        let shoved_at = first_shove_tick(true, 15).expect("tier 3 must eventually fire");
        assert!(
            shoved_at > 7,
            "escalation ON must delay the shove until tier 3 (fired at tick {})",
            shoved_at
        );

        // ON + a fast per-request ladder: tier 3 at 2 fires much earlier — the escalation knob
        // is the RequestED creep's own thresholds (per-request, not system-wide).
        let fast_ladder = StuckThresholds {
            enable_shoving: 2,
            ..StuckThresholds::default()
        };
        let first_shove_fast = {
            let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
            let mut external = StubExternal {
                positions: [(1u32, mover_from), (2u32, occupant_at)].into_iter().collect(),
                data: HashMap::new(),
                sink: sink.clone(),
            };
            let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };
            let mut result = None;
            for tick in 1..=15u32 {
                let mut cache = CostMatrixCache::default();
                let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
                let mut system = MovementSystem::new(&mut cms, &mut pf, None);
                system.set_shove_escalation(true);
                let mut data = MovementData::new();
                data.move_to(1u32, pos(20, 25))
                    .priority(MovementPriority::High)
                    .stuck_thresholds(fast_ladder.clone());
                data.move_to(2u32, occupant_at);
                sink.borrow_mut().clear();
                system.process(&mut external, data);
                if sink.borrow().contains_key(&2) {
                    result = Some(tick);
                    break;
                }
            }
            result
        };
        let fast_tick = first_shove_fast.expect("fast ladder tier 3 must fire");
        assert!(
            fast_tick < shoved_at,
            "a per-request enable_shoving=2 must fire earlier than the default 7 ({} vs {})",
            fast_tick,
            shoved_at
        );
    }

    // DENIAL-AS-STUCK, the load-bearing conversion: a creep that physically ADVANCED along its
    // path (the record_moved shape — e.g. the avoidance dance stepping via tiles that alias
    // path progress) but whose last resolution was denied by an idle occupant must record
    // IMMOBILE, not reset the ladder. Pre-seeded path state isolates exactly the conversion:
    // without the flag consumption this creep's ticks_immobile would reset 3 -> 0.
    #[test]
    fn denied_by_idle_converts_path_progress_into_immobility() {
        let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
        let destination = pos(20, 25);
        let mut external = StubExternal {
            positions: [(1u32, pos(15, 25))].into_iter().collect(),
            data: HashMap::new(),
            sink: sink.clone(),
        };
        // Pre-seed: the creep advanced to path[1] since last tick (on-path, idx 1 = the
        // record_moved shape), with 3 immobile ticks accrued and the denial flag set by the
        // previous tick's Pass 3.
        let mut stuck_state = StuckState {
            ticks_immobile: 3,
            denied_by_idle: true,
            last_distance: 6,
            ..Default::default()
        };
        stuck_state.ticks_no_progress = 0;
        external.data.insert(
            1,
            CreepMovementData {
                path_data: Some(CreepPathData {
                    destination,
                    range: 0,
                    path: vec![pos(14, 25), pos(15, 25), pos(16, 25), pos(17, 25)],
                    time: 0,
                    stuck_state,
                }),
            },
        );

        let mut cache = CostMatrixCache::default();
        let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
        let mut pf = CountingPathfinder { searches: 0, origins: Vec::new() };
        let mut system = MovementSystem::new(&mut cms, &mut pf, None);
        let mut data = MovementData::new();
        data.move_to(1u32, destination);
        system.process(&mut external, data);

        let state = &external.data[&1].path_data.as_ref().unwrap().stuck_state;
        assert_eq!(
            state.ticks_immobile, 4,
            "an idle-denied 'advance' must ACCRUE immobility (would reset to 0 without the flag)"
        );
        assert!(!state.denied_by_idle, "the flag is one tick's worth — consumed");
    }
}
