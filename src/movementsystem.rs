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
        self.ticks_immobile >= StuckThresholds::default().avoid_friendly_creeps
            || self.should_repath_no_progress()
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
    phantom: std::marker::PhantomData<Handle>,
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
            reuse_path_length: 5,
            max_shove_depth: DEFAULT_MAX_SHOVE_DEPTH,
            friendly_creep_distance: DEFAULT_FRIENDLY_CREEP_DISTANCE,
            phantom: std::marker::PhantomData,
        }
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

        if data.requests.is_empty() {
            return results;
        }

        // --- Pass 0: Dependency analysis for Follow intents ---
        let (sorted_entities, broken_follows) = topological_sort_follows(&data.requests);

        // --- Pass 1: Compute desired next tile for each creep ---
        let mut leader_moves: HashMap<Handle, (Position, Option<Position>)> = HashMap::new();
        let mut resolved_creeps: HashMap<Handle, ResolvedCreep<Handle>> = HashMap::new();
        let mut pull_pairs: HashMap<Handle, Handle> = HashMap::new();

        for entity in &sorted_entities {
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
        }

        // --- Pass 2: Conflict resolution ---
        let idle_creep_positions: HashMap<Position, Handle> = HashMap::new();

        let pathfinder = &mut *self.pathfinder;
        let is_tile_walkable = |pos: Position| -> bool { pathfinder.is_tile_walkable(pos) };

        resolve_conflicts(
            &mut resolved_creeps,
            &idle_creep_positions,
            &is_tile_walkable,
            self.max_shove_depth,
        );

        // --- Pass 3: Execute movement and record results ---
        for (entity, resolved) in &resolved_creeps {
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

                        if path_data.stuck_state.should_report_failure() {
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
                    continue;
                }
            }

            let direction = resolved.current_pos.get_direction_to(resolved.final_pos);

            match direction {
                Some(dir) => match creep.move_direction(dir) {
                    Ok(()) => {
                        results.insert(*entity, MovementResult::Moving);
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
        if self.visualizer.is_some() {
            for entity in sorted_entities.iter() {
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
            .map(|s| s.needs_repath())
            .unwrap_or(false);
        let stuck_state_for_gen = stuck_state_snapshot.unwrap_or_default();

        let needs_path = {
            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;
            creep_data.path_data.is_none()
        };

        if needs_path || path_expired || stuck_needs_repath {
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
                        // No existing path to fall back to — propagate the error.
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

        let goals: Vec<(Position, u32)> = targets.iter().map(|t| (t.pos, t.range)).collect();

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
            2000,
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

        let room_names: HashSet<_> = room_path
            .iter()
            .map(|step| step.room)
            .chain(std::iter::once(creep_room_name))
            .chain(std::iter::once(destination_room))
            .collect();

        let mut cost_matrix_options = request.cost_matrix_options.unwrap_or_default();

        if stuck_state.should_avoid_all_friendly_creeps() {
            // Tier 1b: avoid ALL friendly creeps in every room (escalation).
            cost_matrix_options.friendly_creeps = true;
            cost_matrix_options.friendly_creep_proximity = None;
        } else if stuck_state.should_avoid_friendly_creeps() {
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

        let ops_multiplier = if stuck_state.should_increase_ops() {
            2
        } else {
            1
        };
        let max_ops = room_names.len() as u32 * 2000 * ops_multiplier;

        let cost_matrix_system = &mut self.cost_matrix_system;

        let result = self.pathfinder.search(
            creep_pos,
            destination,
            range,
            &mut |room_name: RoomName| -> Option<LocalCostMatrix> {
                if room_names.contains(&room_name) {
                    cost_matrix_system
                        .build_local_cost_matrix(room_name, &cost_matrix_options)
                        .ok()
                } else {
                    None
                }
            },
            max_ops,
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
