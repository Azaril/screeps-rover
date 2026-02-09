use super::costmatrixsystem::*;
use super::error::*;
use super::movementrequest::*;
use super::movementresult::*;
use super::resolver::*;
use super::utility::*;
use screeps::game::map::FindRouteOptions;
use screeps::pathfinder::*;
use screeps::*;
use serde::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::hash::Hash;

/// Configurable thresholds for stuck detection tiers.
/// Different job types can use different thresholds (e.g. military creeps
/// might have lower thresholds for faster reaction).
#[derive(Clone, Debug)]
pub struct StuckThresholds {
    /// Ticks immobile before avoiding friendly creeps in pathfinding (tier 1).
    pub avoid_friendly_creeps: u16,
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
            increase_ops: 4,
            enable_shoving: 6,
            report_failure: 10,
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

    /// Check stuck tier using default thresholds.
    pub fn should_avoid_friendly_creeps(&self) -> bool {
        self.should_avoid_friendly_creeps_with(&StuckThresholds::default())
    }

    pub fn should_increase_ops(&self) -> bool {
        self.should_increase_ops_with(&StuckThresholds::default())
    }

    pub fn should_enable_shoving(&self) -> bool {
        self.should_enable_shoving_with(&StuckThresholds::default())
    }

    /// Whether the creep should repath with friendly creep avoidance (tier 1).
    pub fn should_avoid_friendly_creeps_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.avoid_friendly_creeps
    }

    /// Whether the creep should repath with increased max_ops (tier 2).
    pub fn should_increase_ops_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.increase_ops
    }

    /// Whether the creep's stuck state should enable shoving in the resolver (tier 3).
    pub fn should_enable_shoving_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.enable_shoving
    }

    pub fn should_report_failure(&self) -> bool {
        self.should_report_failure_with(&StuckThresholds::default())
    }

    /// Whether to report stuck failure to the job layer (tier 4).
    pub fn should_report_failure_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_immobile >= thresholds.report_failure
    }

    pub fn should_repath_no_progress(&self) -> bool {
        self.should_repath_no_progress_with(&StuckThresholds::default())
    }

    /// Whether to repath due to lack of progress (circling detection).
    pub fn should_repath_no_progress_with(&self, thresholds: &StuckThresholds) -> bool {
        self.ticks_no_progress >= thresholds.no_progress_repath
    }

    /// Whether any form of repathing is needed based on stuck state.
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
    /// Tiered stuck detection state.
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

    pub fn follow(
        &mut self,
        entity: Handle,
        target: Handle,
    ) -> MovementRequestBuilder<'_, Handle> {
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

pub trait MovementSystemExternal<Handle> {
    fn get_creep(&self, entity: Handle) -> Result<Creep, MovementError>;

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
        if !can_traverse_between_rooms(from_room_name, to_room_name) {
            return None;
        }

        Some(1.0)
    }

    /// Get the position of an entity (used for Follow intents to find leader position).
    fn get_entity_position(&self, entity: Handle) -> Option<Position>;
}

pub struct MovementSystem<'a, Handle> {
    cost_matrix_system: &'a mut CostMatrixSystem,
    default_visualization_style: Option<PolyStyle>,
    reuse_path_length: u32,
    phantom: std::marker::PhantomData<Handle>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a, Handle> MovementSystem<'a, Handle>
where
    Handle: Hash + Eq + Copy + Ord,
{
    pub fn new(cost_matrix_system: &'a mut CostMatrixSystem) -> Self {
        Self {
            cost_matrix_system,
            default_visualization_style: None,
            reuse_path_length: 5,
            phantom: std::marker::PhantomData,
        }
    }

    pub fn set_default_visualization_style(&mut self, style: PolyStyle) {
        self.default_visualization_style = Some(style);
    }

    pub fn set_reuse_path_length(&mut self, length: u32) {
        self.reuse_path_length = length;
    }

    /// Legacy: process using the built-in Screeps `moveTo` API.
    pub fn process_inbuilt<S>(
        &mut self,
        external: &mut S,
        data: MovementData<Handle>,
    ) -> MovementResults<Handle>
    where
        S: MovementSystemExternal<Handle>,
    {
        let mut results = MovementResults::new();
        for (entity, request) in data.requests.into_iter() {
            match self.process_request_inbuilt(external, entity, request) {
                Ok(()) => {
                    results.insert(entity, MovementResult::Moving);
                }
                Err(err) => {
                    results.insert(
                        entity,
                        MovementResult::Failed(MovementFailure::InternalError(err)),
                    );
                }
            }
        }
        results
    }

    /// New global movement resolution with conflict detection, shove/swap, and follow support.
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
        // Track resolved leader positions so followers can reference them.
        let mut leader_moves: HashMap<Handle, (Position, Option<Position>)> = HashMap::new();
        let mut resolved_creeps: HashMap<Handle, ResolvedCreep<Handle>> = HashMap::new();
        // Pull pairs: follower -> leader (for pull mechanics).
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

            // Check if creep can move at all.
            if creep.fatigue() > 0 || creep.spawning() {
                leader_moves.insert(*entity, (creep_pos, None));
                results.insert(*entity, MovementResult::Moving);
                continue;
            }

            // Resolve the intent to a desired position.
            let desired_result = match &request.intent {
                MovementIntent::MoveTo { destination, range } => {
                    // Check if already at destination.
                    if creep_pos.get_range_to(*destination) <= *range {
                        Ok(None)
                    } else {
                        self.compute_next_step_for_move_to(
                            external,
                            *entity,
                            &creep,
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
                } => {
                    // Track pull pair if enabled.
                    if *pull {
                        pull_pairs.insert(*entity, *target);
                    }

                    // If this follow was broken (cycle), treat as MoveTo toward target's position.
                    let effective_target = if broken_follows.contains_key(entity) {
                        external.get_entity_position(*target)
                    } else {
                        None
                    };

                    if let Some(target_pos) = effective_target {
                        // Broken follow -> MoveTo toward target's current position.
                        if creep_pos.get_range_to(target_pos) <= *range {
                            Ok(None)
                        } else {
                            self.compute_next_step_for_move_to(
                                external,
                                *entity,
                                &creep,
                                target_pos,
                                *range,
                                request,
                            )
                        }
                    } else {
                        // Normal follow: use leader's resolved movement.
                        self.compute_next_step_for_follow(
                            external,
                            *entity,
                            &creep,
                            *target,
                            *range,
                            request,
                            &leader_moves,
                        )
                    }
                }
                MovementIntent::Flee { targets, range } => {
                    self.compute_next_step_for_flee(
                        external,
                        *entity,
                        &creep,
                        targets,
                        *range,
                        request,
                    )
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
                    // No movement needed (already arrived or staying put).
                    leader_moves.insert(*entity, (creep_pos, None));

                    // If the creep allows shoving, register it in resolved_creeps
                    // so the resolver can push it out of the way of other creeps.
                    // This is the case for idle/waiting creeps and anchored workers.
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
        // Build idle creep positions map: positions occupied by creeps that have no request.
        // We use entity_position from external to find all known creep positions.
        let idle_creep_positions: HashMap<Position, Handle> = HashMap::new();
        // Note: In a full implementation, we would enumerate all creeps and find
        // those without requests. For now, the resolver handles conflicts between
        // requesting creeps and will attempt shoves on idle creeps found at contested tiles.

        // Simple walkability check using cached structure data.
        let is_tile_walkable = |pos: Position| -> bool {
            // For edge tiles (room boundaries), allow traversal.
            let x = pos.x().u8();
            let y = pos.y().u8();
            if x == 0 || x == 49 || y == 0 || y == 49 {
                return true;
            }

            // Check terrain.
            if let Some(terrain) = game::map::get_room_terrain(pos.room_name()) {
                let t = terrain.get(x, y);
                if t == Terrain::Wall {
                    return false;
                }
            }

            true
        };

        resolve_conflicts(&mut resolved_creeps, &idle_creep_positions, &is_tile_walkable);

        // --- Pass 3: Execute movement and record results ---
        for (entity, resolved) in &resolved_creeps {
            if results.results.contains_key(entity) {
                continue; // Already has a result (arrived, fatigue, error).
            }

            if resolved.final_pos == resolved.current_pos {
                // Creep is staying put.
                // If the creep had no desired movement (desired_pos is None),
                // it intentionally stayed — mark as Arrived, not Stuck.
                if resolved.desired_pos.is_none() {
                    results.insert(*entity, MovementResult::Arrived);
                    continue;
                }

                // Creep wanted to move but couldn't (blocked).
                // Update stuck state.
                let stuck_ticks = if let Ok(creep_data) = external.get_creep_movement_data(*entity)
                {
                    if let Some(path_data) = creep_data.path_data.as_mut() {
                        let dist = resolved.current_pos.get_range_to(path_data.destination);
                        path_data.stuck_state.record_immobile(dist);

                        // Check if we should report failure (tier 4).
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

                results.insert(
                    *entity,
                    MovementResult::Stuck {
                        ticks: stuck_ticks,
                    },
                );
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
                // Pull mechanics: follower uses move_pulled_by, leader must call pull.
                if let Ok(leader_creep) = external.get_creep(*leader_handle) {
                    // Leader pulls the follower.
                    let _ = leader_creep.pull(&creep);
                    // Follower moves toward the leader.
                    let _ = creep.move_pulled_by(&leader_creep);
                    results.insert(*entity, MovementResult::Moving);
                    continue;
                }
            }

            let direction = resolved
                .current_pos
                .get_direction_to(resolved.final_pos);

            match direction {
                Some(dir) => {
                    match creep.move_direction(dir) {
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
                    }
                }
                None => {
                    // Same position or cross-room; shouldn't happen after resolution.
                    results.insert(*entity, MovementResult::Moving);
                }
            }
        }

        // --- Visualization ---
        if self.default_visualization_style.is_some() {
            for entity in sorted_entities.iter() {
                if let Some(request) = data.requests.get(entity) {
                    let result = results.get(entity);
                    self.visualize_entity(
                        external,
                        *entity,
                        request,
                        result,
                        resolved_creeps.get(entity),
                    );
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
        creep: &Creep,
        destination: Position,
        range: u32,
        request: &MovementRequest<Handle>,
    ) -> Result<Option<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep_pos = creep.pos();

        // Validate and reuse cached path. If the creep was shoved off its
        // path (within 1 tile of a path point but not on it), re-anchor to the
        // nearest path point instead of invalidating the entire path.
        {
            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;

            if let Some(path_data) = &mut creep_data.path_data {
                let dest_matches =
                    path_data.destination == destination && path_data.range == range;

                if !dest_matches {
                    creep_data.path_data = None;
                } else {
                    // Check if creep is on the path (first 2 points).
                    let on_path = path_data.path.iter().take(2).any(|p| *p == creep_pos);

                    if !on_path {
                        // Creep is not on the expected path points. Check if it
                        // was shoved nearby (within 1 tile of a path point in
                        // the first few positions). If so, re-anchor to that
                        // point to reuse the path instead of a full repath.
                        let nearby_index = path_data
                            .path
                            .iter()
                            .take(4)
                            .position(|p| creep_pos.get_range_to(*p) <= 1);

                        if let Some(idx) = nearby_index {
                            // Trim the path so the nearby point becomes the
                            // first element. The creep will pathfind one step
                            // to rejoin, then resume the cached path.
                            path_data.path.drain(..idx);
                        } else {
                            // Too far from any path point; invalidate.
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

                // Find the creep's position in the first 2 path points.
                let current_index = path
                    .iter()
                    .take(2)
                    .enumerate()
                    .find(|(_, p)| **p == creep_pos)
                    .map(|(index, _)| index);

                // If not found exactly, check if the creep is adjacent to
                // path[0] (shoved off-path last tick). Treat this as being
                // at index 0 (no progress along path) so the path is reused
                // and the next step will move toward path[0] or path[1].
                let (effective_index, was_shoved_off) = match current_index {
                    Some(idx) => (Some(idx), false),
                    None => {
                        let adjacent_to_start = path
                            .first()
                            .map(|p| creep_pos.get_range_to(*p) <= 1)
                            .unwrap_or(false);
                        if adjacent_to_start {
                            (Some(0), true)
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
                            return Ok(None); // Arrived.
                        }

                        // Update stuck state.
                        if was_shoved_off || !moved {
                            // Shoved off path or didn't advance: record as
                            // immobile so stuck detection can escalate.
                            path_data.stuck_state.record_immobile(current_distance);
                        } else {
                            path_data.stuck_state.record_moved(current_distance);
                        }

                        (Some(path_data.time), Some(path_data.stuck_state.clone()))
                    }
                    None => {
                        // Creep is not on or near the path; invalidate.
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

        // Generate path if required.
        let needs_path = {
            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;
            creep_data.path_data.is_none()
        };

        if needs_path || path_expired || stuck_needs_repath {
            let path_points = self.generate_path(
                external,
                destination,
                range,
                request,
                creep,
                &stuck_state_for_gen,
            )?;

            let creep_data = external
                .get_creep_movement_data(entity)
                .map_err(MovementFailure::InternalError)?;

            // Preserve repath_count from previous stuck state.
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

        // Extract next step.
        let creep_data = external
            .get_creep_movement_data(entity)
            .map_err(MovementFailure::InternalError)?;
        let path_data = creep_data
            .path_data
            .as_ref()
            .ok_or(MovementFailure::PathNotFound)?;

        // If the creep is on path[0], the next step is path[1].
        // If the creep was shoved off-path (adjacent to path[0] but not on it),
        // the next step is path[0] itself to rejoin the cached path.
        let next_pos = if path_data.path.first() == Some(&creep_pos) {
            path_data.path.get(1).copied()
        } else {
            path_data.path.first().copied()
        };

        Ok(next_pos)
    }

    /// Compute the next step for a Follow intent.
    #[allow(clippy::too_many_arguments)]
    fn compute_next_step_for_follow<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        creep: &Creep,
        target: Handle,
        range: u32,
        request: &MovementRequest<Handle>,
        leader_moves: &HashMap<Handle, (Position, Option<Position>)>,
    ) -> Result<Option<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep_pos = creep.pos();

        // Look up leader's resolved movement.
        let (leader_old_pos, leader_new_pos) = match leader_moves.get(&target) {
            Some(positions) => *positions,
            None => {
                // Leader has no movement data yet (maybe not a request entity).
                // Fall back to leader's current position.
                match external.get_entity_position(target) {
                    Some(pos) => (pos, None),
                    None => return Err(MovementFailure::InternalError(
                        "Follow target entity not found".to_owned(),
                    )),
                }
            }
        };

        let leader_is_moving = leader_new_pos.is_some() && leader_new_pos != Some(leader_old_pos);

        if leader_is_moving {
            // Leader is moving: follow into the leader's vacated tile.
            let vacated_tile = leader_old_pos;

            if creep_pos.get_range_to(vacated_tile) <= 1 {
                // We can step directly into the vacated tile (or stay if we're on it).
                if creep_pos == vacated_tile {
                    return Ok(None); // Already there.
                }
                return Ok(Some(vacated_tile));
            }
            // Leader's vacated tile is too far; path toward the leader's new position.
            let leader_dest = leader_new_pos.unwrap_or(leader_old_pos);
            self.compute_next_step_for_move_to(external, entity, creep, leader_dest, range, request)
        } else {
            // Leader is stationary.
            if creep_pos.get_range_to(leader_old_pos) <= range {
                return Ok(None); // Already within range, stay put.
            }
            // Path to within range of the leader.
            self.compute_next_step_for_move_to(
                external,
                entity,
                creep,
                leader_old_pos,
                range,
                request,
            )
        }
    }

    /// Compute the next step for a Flee intent using pathfinder with flee mode.
    fn compute_next_step_for_flee<S>(
        &mut self,
        _external: &mut S,
        _entity: Handle,
        creep: &Creep,
        targets: &[FleeTarget],
        _range: u32,
        request: &MovementRequest<Handle>,
    ) -> Result<Option<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep_pos = creep.pos();

        if targets.is_empty() {
            return Ok(None); // Nothing to flee from.
        }

        // Check if already safe (out of range of all targets).
        let already_safe = targets
            .iter()
            .all(|t| creep_pos.get_range_to(t.pos) >= t.range);

        if already_safe {
            return Ok(None);
        }

        // Build search goals from flee targets.
        let goals: Vec<SearchGoal> = targets
            .iter()
            .map(|t| SearchGoal::new(t.pos, t.range))
            .collect();

        let cost_matrix_options = request.cost_matrix_options.unwrap_or_default();
        let cost_matrix_system = &mut self.cost_matrix_system;

        // For flee, search the current room and adjacent rooms.
        let creep_room_name = creep_pos.room_name();

        let search_options = SearchOptions::new(move |room_name: RoomName| -> MultiRoomCostResult {
            // Allow current room and up to 2 adjacent rooms for flee.
            let distance =
                game::map::get_room_linear_distance(creep_room_name, room_name, false);
            if distance > 2 {
                return MultiRoomCostResult::Impassable;
            }

            match cost_matrix_system.build_local_cost_matrix(room_name, &cost_matrix_options) {
                Ok(local_cm) => {
                    let js_cm: CostMatrix = local_cm.into();
                    MultiRoomCostResult::CostMatrix(js_cm)
                }
                Err(_err) => MultiRoomCostResult::Impassable,
            }
        })
        .flee(true)
        .max_ops(2000)
        .plain_cost(cost_matrix_options.plains_cost)
        .swamp_cost(cost_matrix_options.swamp_cost);

        let search_result =
            pathfinder::search_many(creep_pos, goals.into_iter(), Some(search_options));

        if search_result.incomplete() || search_result.path().is_empty() {
            return Err(MovementFailure::PathNotFound);
        }

        let path = search_result.path();
        Ok(path.first().copied())
    }

    fn process_request_inbuilt<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        mut request: MovementRequest<Handle>,
    ) -> Result<(), MovementError>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep = external.get_creep(entity)?;

        let (destination, range) = match &request.intent {
            MovementIntent::MoveTo { destination, range } => (*destination, *range),
            MovementIntent::Follow { target, range, .. } => {
                match external.get_entity_position(*target) {
                    Some(pos) => (pos, *range),
                    None => return Err("Follow target not found".to_owned()),
                }
            }
            MovementIntent::Flee { .. } => {
                // Inbuilt pathfinding doesn't support flee well; skip.
                return Err("Flee not supported in inbuilt mode".to_owned());
            }
        };

        let move_options = MoveToOptions::new()
            .range(range)
            .reuse_path(self.reuse_path_length);

        let vis_move_options = if let Some(vis) = request.visualization.take() {
            move_options.visualize_path_style(vis)
        } else if let Some(vis) = self.default_visualization_style.clone() {
            move_options.visualize_path_style(vis)
        } else {
            move_options
        };

        creep
            .move_to_with_options(destination, Some(vis_move_options))
            .map_err(|e| format!("Move error: {:?}", e))?;

        Ok(())
    }

    fn generate_path<S>(
        &mut self,
        external: &mut S,
        destination: Position,
        range: u32,
        request: &MovementRequest<Handle>,
        creep: &Creep,
        stuck_state: &StuckState,
    ) -> Result<Vec<Position>, MovementFailure>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep_pos = creep.pos();
        let creep_room_name = creep_pos.room_name();

        let room_options = request.room_options.unwrap_or_default();

        let destination_room = destination.room_name();

        let room_path = game::map::find_route(
            creep_room_name,
            destination.room_name(),
            Some(
                FindRouteOptions::new().room_callback(|to_room_name, from_room_name| {
                    external
                        .get_room_cost(from_room_name, to_room_name, &room_options)
                        .unwrap_or(f64::INFINITY)
                }),
            ),
        )
        .map_err(|e| {
            MovementFailure::InternalError(format!("Could not find path between rooms: {:?}", e))
        })?;

        let room_names: HashSet<_> = room_path
            .iter()
            .map(|step| step.room)
            .chain(std::iter::once(creep_room_name))
            .chain(std::iter::once(destination_room))
            .collect();

        let mut cost_matrix_options = request.cost_matrix_options.unwrap_or_default();

        // Escalating stuck strategies applied to cost matrix and search options.
        // Tier 1 (2-3 ticks stuck): avoid friendly creeps in pathfinding.
        if stuck_state.should_avoid_friendly_creeps() {
            cost_matrix_options.friendly_creeps = true;
        }

        let cost_matrix_system = &mut self.cost_matrix_system;

        // Tier 2 (4-5 ticks stuck): increase max_ops for the search.
        let ops_multiplier = if stuck_state.should_increase_ops() {
            2
        } else {
            1
        };
        let max_ops = room_names.len() as u32 * 2000 * ops_multiplier;

        let search_options = SearchOptions::new(|room_name: RoomName| -> MultiRoomCostResult {
            if room_names.contains(&room_name) {
                // Build cost matrix in Rust memory (LocalCostMatrix), then convert
                // to JS CostMatrix only at the pathfinder boundary.
                match cost_matrix_system.build_local_cost_matrix(room_name, &cost_matrix_options) {
                    Ok(local_cm) => {
                        let js_cm: CostMatrix = local_cm.into();
                        MultiRoomCostResult::CostMatrix(js_cm)
                    }
                    Err(_err) => MultiRoomCostResult::Impassable,
                }
            } else {
                MultiRoomCostResult::Impassable
            }
        })
        .max_ops(max_ops)
        .plain_cost(cost_matrix_options.plains_cost)
        .swamp_cost(cost_matrix_options.swamp_cost);

        let search_result =
            pathfinder::search(creep_pos, destination, range, Some(search_options));

        if search_result.incomplete() {
            return Err(MovementFailure::PathNotFound);
        }

        let mut path_points = search_result.path();

        path_points.insert(0, creep_pos);

        Ok(path_points)
    }

    /// Draw per-entity visualization based on movement state:
    /// - Moving: blue path polyline (existing behavior)
    /// - Anchored worker (arrived with anchor): orange circle + line to anchor
    /// - Immovable (arrived, Immovable priority): small red X
    /// - Stuck: yellow circle with tick count
    /// - Failed: red circle
    fn visualize_entity<S>(
        &self,
        external: &mut S,
        entity: Handle,
        request: &MovementRequest<Handle>,
        result: Option<&MovementResult>,
        resolved: Option<&ResolvedCreep<Handle>>,
    ) where
        S: MovementSystemExternal<Handle>,
    {
        let creep_pos = match external.get_creep(entity) {
            Ok(creep) => creep.pos(),
            Err(_) => return,
        };

        let room_name = creep_pos.room_name();
        let cx = creep_pos.x().u8() as f32;
        let cy = creep_pos.y().u8() as f32;
        let visual = RoomVisual::new(Some(room_name));

        match result {
            Some(MovementResult::Moving) => {
                // Draw the cached path as a polyline.
                let points = match external.get_creep_movement_data(entity) {
                    Ok(creep_data) => {
                        if let Some(path_data) = &creep_data.path_data {
                            path_data
                                .path
                                .iter()
                                .take_while(|p| p.room_name() == room_name)
                                .map(|p| (p.x().u8() as f32, p.y().u8() as f32))
                                .collect::<Vec<_>>()
                        } else {
                            return;
                        }
                    }
                    Err(_) => return,
                };

                let style = request
                    .visualization
                    .clone()
                    .or_else(|| self.default_visualization_style.clone());
                if let Some(style) = style {
                    visual.poly(points, Some(style));
                }
            }

            Some(MovementResult::Arrived) => {
                if request.priority == MovementPriority::Immovable {
                    // Immovable: small red X to show the tile is locked.
                    let d = 0.15;
                    let line_style = LineStyle::default()
                        .color("#ff4444")
                        .opacity(0.6);
                    visual.line((cx - d, cy - d), (cx + d, cy + d), Some(line_style.clone()));
                    visual.line((cx - d, cy + d), (cx + d, cy - d), Some(line_style));
                } else if let Some(anchor) = &request.anchor {
                    // Anchored worker: orange circle on creep + faint line to anchor.
                    let circle_style = CircleStyle::default()
                        .fill("#ff8800")
                        .radius(0.15)
                        .opacity(0.5)
                        .stroke("#ff8800")
                        .stroke_width(0.02);
                    visual.circle(cx, cy, Some(circle_style));

                    let ax = anchor.position.x().u8() as f32;
                    let ay = anchor.position.y().u8() as f32;
                    if (ax - cx).abs() > 0.01 || (ay - cy).abs() > 0.01 {
                        let line_style = LineStyle::default()
                            .color("#ff8800")
                            .opacity(0.25);
                        visual.line((cx, cy), (ax, ay), Some(line_style));
                    }
                }
            }

            Some(MovementResult::Stuck { ticks }) => {
                // Stuck: yellow circle with tick count.
                let circle_style = CircleStyle::default()
                    .fill("#ffcc00")
                    .radius(0.2)
                    .opacity(0.6)
                    .stroke("#ffcc00")
                    .stroke_width(0.03);
                visual.circle(cx, cy, Some(circle_style));

                let text_style = TextStyle::default()
                    .color("#ffcc00")
                    .font(0.4)
                    .stroke("#000000")
                    .stroke_width(0.03);
                visual.text(cx, cy + 0.55, format!("{}", ticks), Some(text_style));
            }

            Some(MovementResult::Failed(_)) => {
                // Failed: red circle.
                let circle_style = CircleStyle::default()
                    .fill("#ff0000")
                    .radius(0.2)
                    .opacity(0.7)
                    .stroke("#ff0000")
                    .stroke_width(0.03);
                visual.circle(cx, cy, Some(circle_style));
            }

            None => {
                // No result yet — creep was not processed (shouldn't normally happen).
                // Show a dim gray circle as a fallback.
                if resolved.is_some() {
                    let circle_style = CircleStyle::default()
                        .fill("#888888")
                        .radius(0.1)
                        .opacity(0.3);
                    visual.circle(cx, cy, Some(circle_style));
                }
            }
        }
    }
}
