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
    /// Ticks immobile before enabling shoving in the resolver (tier 3).
    /// NOTE: not currently consulted — nothing calls `should_enable_shoving*`; shoving is
    /// governed per-request via `MovementRequest.allow_shove` plus the resolver's own
    /// priority/stuck gate (`try_shove`'s `STUCK_SHOVE_THRESHOLD`). Wiring this tier (making
    /// shove an ESCALATION instead of a per-request default) is a recorded follow-up; doing it
    /// silently here would change live behavior, so the field stays documented-but-inert.
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
    /// How many times we've regenerated the path for the current destination.
    pub repath_count: u8,
    /// Distance to target last tick (for progress tracking).
    #[serde(default)]
    pub last_distance: u32,
}

impl StuckState {
    /// Reset all stuck tracking (e.g. when destination changes).
    pub fn reset(&mut self) {
        self.ticks_immobile = 0;
        self.ticks_no_progress = 0;
        self.repath_count = 0;
        self.last_distance = 0;
    }

    /// Record that the creep moved this tick.
    pub fn record_moved(&mut self, current_distance: u32) {
        self.ticks_immobile = 0;

        if current_distance < self.last_distance {
            self.ticks_no_progress = 0;
        } else {
            self.ticks_no_progress += 1;
        }

        self.last_distance = current_distance;
    }

    /// Record that the creep did NOT move this tick.
    pub fn record_immobile(&mut self, current_distance: u32) {
        self.ticks_immobile += 1;
        self.ticks_no_progress += 1;
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

    /// NOTE: currently unconsulted (no MovementSystem/resolver caller) — see
    /// `StuckThresholds::enable_shoving`.
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

    /// Set the stuck-escalation ladder (see [`StuckThresholds`]). Every stuck check the system
    /// performs routes through this — tune it to trade wasted-intent burn (slow escalation)
    /// against premature detours (fast escalation).
    pub fn set_stuck_thresholds(&mut self, thresholds: StuckThresholds) {
        self.stuck_thresholds = thresholds;
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
    /// (ibex pathing/movementsystem.rs:269,272) and sim-core's driver sets explicit unlimited
    /// ones, so this change only un-breaks future headless consumers.
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
                leader_moves.insert(*entity, (creep_pos, None));
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
                Ok(Some(desired_pos)) => {
                    leader_moves.insert(*entity, (creep_pos, Some(desired_pos)));
                    let creep_data = external.get_creep_movement_data(*entity).ok();
                    let stuck_ticks = creep_data
                        .and_then(|d| d.path_data.as_ref())
                        .map(|p| p.stuck_state.ticks_immobile as u32)
                        .unwrap_or(0);

                    resolved_creeps.insert(
                        *entity,
                        ResolvedCreep {
                            entity: *entity,
                            current_pos: creep_pos,
                            desired_pos: Some(desired_pos),
                            priority: request.priority,
                            allow_shove: request.allow_shove,
                            allow_swap: request.allow_swap,
                            stuck_ticks,
                            resolved: false,
                            final_pos: creep_pos,
                            has_request: true,
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
                                allow_shove: request.allow_shove,
                                allow_swap: request.allow_swap,
                                stuck_ticks: 0,
                                resolved: false,
                                final_pos: creep_pos,
                                has_request: true,
                                anchor: request.anchor,
                            },
                        );
                    } else {
                        results.insert(*entity, MovementResult::Arrived);
                    }
                }
                Err(err) => {
                    leader_moves.insert(*entity, (creep_pos, None));
                    results.insert(*entity, MovementResult::Failed(err));
                }
            }
            work_done_this_tick = true;
        }

        // --- Pass 2: Conflict resolution ---
        if !self.is_over_tick_limit() && !self.is_over_movement_cap() {
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

            if resolved.final_pos == resolved.current_pos {
                if resolved.desired_pos.is_none() {
                    results.insert(*entity, MovementResult::Arrived);
                    continue;
                }

                let stuck_ticks = if let Ok(creep_data) = external.get_creep_movement_data(*entity)
                {
                    if let Some(path_data) = creep_data.path_data.as_mut() {
                        let dist = resolved.current_pos.get_range_to(path_data.destination);
                        path_data.stuck_state.record_immobile(dist);

                        if path_data.stuck_state.should_report_failure_with(&self.stuck_thresholds) {
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
                            if moved {
                                // The creep advanced along the path — either it
                                // walked normally or it side-stepped via local
                                // avoidance and ended up adjacent to a further
                                // path position. Either way, it made progress.
                                path_data.stuck_state.record_moved(current_distance);
                            } else if was_shoved_off {
                                // Off-path but didn't advance (adjacent only to
                                // path start). Likely shoved sideways.
                                path_data.stuck_state.record_immobile(current_distance);
                            } else {
                                // On-path but at the same position as last tick.
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
            .map(|s| s.needs_repath_with(&self.stuck_thresholds))
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
        let should_pathfind = if needs_path {
            true
        } else if stuck_needs_repath {
            if self.is_cpu_budget_exhausted() {
                // CPU budget exhausted: skip stuck repath, keep existing path.
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
            if self.is_cpu_budget_exhausted() || self.is_repath_budget_exhausted() {
                // CPU budget or repath budget exhausted: skip expiry repath,
                // reset timer and keep existing path.
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
                    // advances, so escalation (shove/swap) continues normally.
                    if let Ok(creep_data) = external.get_creep_movement_data(entity) {
                        if let Some(path_data) = creep_data.path_data.as_mut() {
                            path_data.time = 0;
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

        if stuck_state.should_avoid_all_friendly_creeps_with(&self.stuck_thresholds) {
            // Tier 1b: avoid ALL friendly creeps in every room (escalation).
            cost_matrix_options.friendly_creeps = true;
            cost_matrix_options.friendly_creep_proximity = None;
        } else if stuck_state.should_avoid_friendly_creeps_with(&self.stuck_thresholds) {
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

        let ops_multiplier = if stuck_state.should_increase_ops_with(&self.stuck_thresholds) {
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

    /// Counts `search` calls and returns a straight horizontal path origin→goal (same room,
    /// same y — the fixture worlds below keep y fixed). Open terrain everywhere.
    struct CountingPathfinder {
        searches: u32,
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
        let mut pf = CountingPathfinder { searches: 0 };
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
        let mut pf = CountingPathfinder { searches: 0 };

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

    // F2 end-to-end at the process() seam: a parked creep registered via
    // set_idle_creep_positions makes the resolver route AROUND it, where the unregistered
    // baseline issues a move INTO it (the engine-rejected `failed_into_parked` intent).
    #[test]
    fn registered_idle_occupant_is_routed_around_not_moved_into() {
        let parked = pos(15, 25);
        let mover_from = pos(14, 25);

        let run = |register: bool| -> Option<Position> {
            let sink: Rc<RefCell<HashMap<u32, Direction>>> = Rc::new(RefCell::new(HashMap::new()));
            let mut external = StubExternal {
                positions: [(1u32, mover_from)].into_iter().collect(),
                data: HashMap::new(),
                sink: sink.clone(),
            };
            let mut cache = CostMatrixCache::default();
            let mut cms = CostMatrixSystem::new(&mut cache, Box::new(NullCostSource));
            let mut pf = CountingPathfinder { searches: 0 };
            let mut system = MovementSystem::new(&mut cms, &mut pf, None);
            if register {
                // Handle 99 is OUTSIDE the request set — a parked (goal-reached) creep.
                system.set_idle_creep_positions([(parked, 99u32)].into_iter().collect());
            }
            let mut data = MovementData::new();
            data.move_to(1u32, pos(20, 25));
            system.process(&mut external, data);

            let dir = sink.borrow().get(&1).copied()?;
            let off = dir.into_offset();
            Some(pos(
                (mover_from.x().u8() as i32 + off.0) as u8,
                (mover_from.y().u8() as i32 + off.1) as u8,
            ))
        };

        // Baseline (no registration): the optimistic path runs straight through the parked tile
        // and the resolver cannot know better — the issued move targets it (engine would reject).
        assert_eq!(run(false), Some(parked), "unregistered baseline paths into the parked tile");

        // Registered: the grant is denied and local avoidance sidesteps — the issued move exists
        // (no stall) and never targets the occupied tile.
        let target = run(true).expect("mover must still be issued a move (sidestep), not stall");
        assert_ne!(target, parked, "registered occupant tile must not be moved into");
    }
}
