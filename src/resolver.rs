use super::movementrequest::*;
use screeps::constants::Direction;
use screeps::local::*;
use std::collections::HashMap;
use std::hash::Hash;

/// Maximum depth for shove chains to prevent unbounded recursion.
const MAX_SHOVE_DEPTH: u32 = 3;

/// Tracks per-creep state during a single tick of resolution.
#[derive(Clone)]
pub(crate) struct ResolvedCreep<Handle: Hash + Eq + Copy> {
    #[allow(dead_code)]
    pub entity: Handle,
    pub current_pos: Position,
    pub desired_pos: Option<Position>,
    pub priority: MovementPriority,
    pub allow_shove: bool,
    pub allow_swap: bool,
    pub stuck_ticks: u32,
    /// Was this creep's movement resolved (i.e. a direction was decided)?
    pub resolved: bool,
    /// Final position after resolution (current_pos if staying, desired_pos if moving).
    pub final_pos: Position,
    /// True if this creep had a movement request (vs idle creep).
    pub has_request: bool,
    /// Optional anchor constraint: if set, shoves/swaps must keep this creep
    /// within `anchor.range` of `anchor.position`.
    pub anchor: Option<AnchorConstraint>,
}

/// Topologically sorts entities based on follow dependencies.
/// Returns (sorted order, set of entities whose follow was broken into MoveTo).
pub(crate) fn topological_sort_follows<Handle: Hash + Eq + Copy>(
    requests: &HashMap<Handle, MovementRequest<Handle>>,
) -> (Vec<Handle>, HashMap<Handle, Handle>) {
    // Build adjacency: follower -> leader
    let mut follow_edges: HashMap<Handle, Handle> = HashMap::new();

    for (entity, request) in requests.iter() {
        if let MovementIntent::Follow { target, .. } = &request.intent {
            follow_edges.insert(*entity, *target);
        }
    }

    // Detect cycles using iterative path tracing.
    // For each node, follow the chain; if we revisit a node in the current path, it's a cycle.
    let mut broken_follows: HashMap<Handle, Handle> = HashMap::new();
    let mut visited: HashMap<Handle, bool> = HashMap::new(); // true = fully processed, false = in current path

    for start in follow_edges.keys().copied().collect::<Vec<_>>() {
        if visited.contains_key(&start) {
            continue;
        }

        let mut path = Vec::new();
        let mut current = start;

        loop {
            if let Some(&fully_processed) = visited.get(&current) {
                if !fully_processed {
                    // Cycle detected: current is in the current path.
                    // Break the cycle at the edge leading to `current`.
                    // Find which entity in the path points to current.
                    if let Some(&follower) = path.last() {
                        if follow_edges.contains_key(&follower) {
                            broken_follows.insert(follower, current);
                            follow_edges.remove(&follower);
                        }
                    }
                }
                break;
            }

            visited.insert(current, false); // mark as in-progress
            path.push(current);

            if let Some(&leader) = follow_edges.get(&current) {
                current = leader;
            } else {
                break;
            }
        }

        // Mark all nodes in path as fully processed
        for node in path {
            visited.insert(node, true);
        }
    }

    // Now do the actual topological sort (leaders before followers).
    // A leader has no follow edge (or its edge was broken).
    // We process in reverse dependency order: leaders first.
    let mut sorted = Vec::with_capacity(requests.len());
    let mut remaining: HashMap<Handle, Option<Handle>> = HashMap::new();

    for (entity, _) in requests.iter() {
        let leader = follow_edges.get(entity).copied();
        remaining.insert(*entity, leader);
    }

    // Also add entities that have no request but are follow targets.
    // (They won't be in `requests` but might be leaders.)

    let mut processed = std::collections::HashSet::new();

    // Iteratively extract nodes with no unprocessed dependencies.
    loop {
        let mut batch = Vec::new();

        for (entity, leader) in remaining.iter() {
            if processed.contains(entity) {
                continue;
            }
            match leader {
                None => batch.push(*entity),
                Some(l) => {
                    if processed.contains(l) || !remaining.contains_key(l) {
                        batch.push(*entity);
                    }
                }
            }
        }

        if batch.is_empty() {
            // Remaining unprocessed entities (shouldn't happen if cycles are broken).
            for (entity, _) in remaining.iter() {
                if !processed.contains(entity) {
                    sorted.push(*entity);
                    processed.insert(*entity);
                }
            }
            break;
        }

        for entity in &batch {
            sorted.push(*entity);
            processed.insert(*entity);
        }

        if processed.len() == remaining.len() {
            break;
        }
    }

    (sorted, broken_follows)
}

/// Resolves conflicts between creeps that want to move to the same tile.
///
/// # Algorithm
/// 1. Detect and resolve head-to-head swaps (A wants B's tile, B wants A's tile).
/// 2. Build an intent map (desired_pos -> list of entities) and a current-position
///    map (current_pos -> entity) for all unresolved creeps.
/// 3. For each contested tile, the highest priority creep wins. If the tile is
///    currently occupied by another creep (whether that creep is moving, idle, or
///    stationary), attempt to shove the occupant out of the way.
/// 4. Mark remaining unresolved creeps as staying in place.
pub(crate) fn resolve_conflicts<Handle: Hash + Eq + Copy + Ord>(
    creeps: &mut HashMap<Handle, ResolvedCreep<Handle>>,
    idle_creep_positions: &HashMap<Position, Handle>,
    is_tile_walkable: &dyn Fn(Position) -> bool,
) {
    // Step 1: Detect and resolve swaps first.
    resolve_swaps(creeps);

    // Step 2: Build intent map for non-resolved creeps that want to move somewhere.
    let mut intent_map: HashMap<Position, Vec<Handle>> = HashMap::new();

    for (entity, creep) in creeps.iter() {
        if creep.resolved {
            continue;
        }
        if let Some(desired) = creep.desired_pos {
            intent_map.entry(desired).or_default().push(*entity);
        }
    }

    // Build current-position map: position -> entity for all unresolved creeps
    // in `resolved_creeps`. This lets us find ANY creep blocking a tile, whether
    // it is stationary (desired_pos == None) or also trying to move somewhere.
    let current_pos_to_entity: HashMap<Position, Handle> = creeps
        .iter()
        .filter(|(_, c)| !c.resolved)
        .map(|(entity, c)| (c.current_pos, *entity))
        .collect();

    // Find the creep currently occupying a tile. Checks resolved_creeps first
    // (covers moving, idle, and stationary creeps), then idle_creep_positions
    // (creeps with no request at all).
    let find_occupant = |tile: &Position| -> Option<Handle> {
        if let Some(entity) = current_pos_to_entity.get(tile) {
            return Some(*entity);
        }
        if let Some(entity) = idle_creep_positions.get(tile) {
            return Some(*entity);
        }
        None
    };

    // Step 3: For each desired tile, resolve who gets to move there.
    // Process tiles in dependency order: if creep X wants tile T, and T is
    // occupied by creep Y who wants tile T2, then T2 should be processed
    // before T. This ensures convoy-style movement (A→B→C all moving the
    // same direction) resolves front-to-back: C moves first, freeing space
    // for B, which frees space for A.
    let tiles = {
        let all_tiles: Vec<Position> = intent_map.keys().copied().collect();

        // Build dependency graph: tile T depends on tile T2 if T is occupied
        // by a creep that wants T2. "T depends on T2" means T2 should be
        // processed first.
        let mut tile_deps: HashMap<Position, Vec<Position>> = HashMap::new();
        for &tile in &all_tiles {
            tile_deps.entry(tile).or_default();
        }

        for &tile in &all_tiles {
            // Who is currently sitting on `tile`?
            if let Some(occupant_handle) = current_pos_to_entity.get(&tile) {
                if let Some(occupant) = creeps.get(occupant_handle) {
                    if !occupant.resolved {
                        if let Some(occ_desired) = occupant.desired_pos {
                            // The occupant wants occ_desired. If occ_desired is
                            // also a contested tile, then `tile` depends on
                            // occ_desired being resolved first. We model this
                            // as an edge occ_desired → tile (occ_desired must
                            // come before tile in processing order).
                            if intent_map.contains_key(&occ_desired) && occ_desired != tile {
                                tile_deps.entry(occ_desired).or_default().push(tile);
                            }
                        }
                    }
                }
            }
        }

        // Topological sort of tiles (Kahn's algorithm). Tiles with in-degree 0
        // have no blockers and are processed first (front of convoy). Ties
        // broken by spatial order for determinism.
        let mut in_degree: HashMap<Position, usize> = HashMap::new();
        for &tile in &all_tiles {
            in_degree.entry(tile).or_insert(0);
        }
        for successors in tile_deps.values() {
            for successor in successors {
                if let Some(deg) = in_degree.get_mut(successor) {
                    *deg += 1;
                }
            }
        }

        let mut queue: std::collections::VecDeque<Position> = {
            let mut v: Vec<Position> = in_degree
                .iter()
                .filter(|(_, &deg)| deg == 0)
                .map(|(&pos, _)| pos)
                .collect();
            v.sort_by_key(|p| (p.room_name(), p.x().u8(), p.y().u8()));
            v.into()
        };

        let mut sorted_tiles: Vec<Position> = Vec::with_capacity(all_tiles.len());

        while let Some(tile) = queue.pop_front() {
            sorted_tiles.push(tile);

            if let Some(successors) = tile_deps.get(&tile) {
                let mut new_ready: Vec<Position> = Vec::new();
                for successor in successors {
                    if let Some(deg) = in_degree.get_mut(successor) {
                        *deg -= 1;
                        if *deg == 0 {
                            new_ready.push(*successor);
                        }
                    }
                }
                // Sort newly ready tiles and append to maintain deterministic order.
                new_ready.sort_by_key(|p| (p.room_name(), p.x().u8(), p.y().u8()));
                for tile in new_ready {
                    queue.push_back(tile);
                }
            }
        }

        // Any remaining tiles (cycles) appended in spatial order.
        if sorted_tiles.len() < all_tiles.len() {
            let mut remaining: Vec<Position> = all_tiles
                .iter()
                .filter(|t| !sorted_tiles.contains(t))
                .copied()
                .collect();
            remaining.sort_by_key(|p| (p.room_name(), p.x().u8(), p.y().u8()));
            sorted_tiles.extend(remaining);
        }

        sorted_tiles
    };

    for tile in &tiles {
        let candidates = &intent_map[tile];

        // Pick the best candidate (highest priority, then most stuck).
        let winner = candidates
            .iter()
            .filter(|h| !creeps[*h].resolved)
            .max_by(|a, b| {
                let ca = &creeps[*a];
                let cb = &creeps[*b];
                ca.priority
                    .cmp(&cb.priority)
                    .then_with(|| ca.stuck_ticks.cmp(&cb.stuck_ticks))
            })
            .copied();

        let winner_handle = match winner {
            Some(h) => h,
            None => continue,
        };

        // Check if the target tile is occupied by another creep.
        let mut winner_can_move = true;

        if let Some(occupant) = find_occupant(tile) {
            if occupant != winner_handle {
                // The tile is occupied. Try to shove the occupant away.
                let shoved = try_shove(
                    occupant,
                    creeps,
                    idle_creep_positions,
                    is_tile_walkable,
                    0,
                );
                if !shoved {
                    winner_can_move = false;
                }
            }
        }

        if winner_can_move {
            let winner_creep = creeps.get_mut(&winner_handle).unwrap();
            winner_creep.resolved = true;
            winner_creep.final_pos = *tile;
        }
    }

    // Step 4: Mark all remaining unresolved creeps as staying in place.
    for creep in creeps.values_mut() {
        if !creep.resolved {
            creep.resolved = true;
            creep.final_pos = creep.current_pos;
        }
    }
}

/// Detect and resolve head-to-head swaps: creep A wants B's tile and B wants A's tile.
fn resolve_swaps<Handle: Hash + Eq + Copy + Ord>(
    creeps: &mut HashMap<Handle, ResolvedCreep<Handle>>,
) {
    // Build position -> entity map for moving creeps.
    let mut pos_to_entity: HashMap<Position, Handle> = HashMap::new();
    for (entity, creep) in creeps.iter() {
        if creep.has_request && !creep.resolved {
            pos_to_entity.insert(creep.current_pos, *entity);
        }
    }

    let mut swap_pairs: Vec<(Handle, Handle)> = Vec::new();

    for (entity, creep) in creeps.iter() {
        if creep.resolved || !creep.allow_swap {
            continue;
        }
        if let Some(desired) = creep.desired_pos {
            // Is there a creep at the desired position?
            if let Some(&other_entity) = pos_to_entity.get(&desired) {
                if other_entity == *entity {
                    continue;
                }
                let other = &creeps[&other_entity];
                if other.resolved || !other.allow_swap {
                    continue;
                }
                // Does the other creep want our position?
                if other.desired_pos == Some(creep.current_pos) {
                    // It's a swap! Record it (avoid duplicates).
                    let pair = if *entity < other_entity {
                        (*entity, other_entity)
                    } else {
                        (other_entity, *entity)
                    };
                    if !swap_pairs.contains(&pair) {
                        swap_pairs.push(pair);
                    }
                }
            }
        }
    }

    // Execute swaps, respecting anchor constraints.
    for (a, b) in swap_pairs {
        let creep_a = &creeps[&a];
        let creep_b = &creeps[&b];
        let a_desired = creep_a.desired_pos;
        let b_desired = creep_b.desired_pos;

        if let (Some(a_dest), Some(b_dest)) = (a_desired, b_desired) {
            // Check anchor constraints: each creep's new position must be
            // within its anchor range (if it has one).
            let a_ok = creep_a
                .anchor
                .map(|ac| a_dest.get_range_to(ac.position) <= ac.range)
                .unwrap_or(true);
            let b_ok = creep_b
                .anchor
                .map(|ac| b_dest.get_range_to(ac.position) <= ac.range)
                .unwrap_or(true);

            if !a_ok || !b_ok {
                continue;
            }

            let creep_a = creeps.get_mut(&a).unwrap();
            creep_a.resolved = true;
            creep_a.final_pos = a_dest;

            let creep_b = creeps.get_mut(&b).unwrap();
            creep_b.resolved = true;
            creep_b.final_pos = b_dest;
        }
    }
}

/// Try to shove a creep out of the way. Returns true if successful.
///
/// Supports chain-shoving: if all adjacent tiles are occupied, it will
/// recursively attempt to shove occupants up to `MAX_SHOVE_DEPTH` levels deep.
fn try_shove<Handle: Hash + Eq + Copy + Ord>(
    entity: Handle,
    creeps: &mut HashMap<Handle, ResolvedCreep<Handle>>,
    idle_creep_positions: &HashMap<Position, Handle>,
    is_tile_walkable: &dyn Fn(Position) -> bool,
    depth: u32,
) -> bool {
    if depth >= MAX_SHOVE_DEPTH {
        return false;
    }

    let creep = match creeps.get(&entity) {
        Some(c) => c.clone(),
        None => return false,
    };

    if !creep.allow_shove {
        return false;
    }

    if creep.priority == MovementPriority::Immovable {
        return false;
    }

    // Already resolved (e.g. already being shoved or already moving).
    if creep.resolved && creep.final_pos != creep.current_pos {
        return true; // It's already leaving.
    }

    let pos = creep.current_pos;

    // Build set of positions that are definitively occupied (resolved creeps'
    // final positions). We don't include unresolved creeps' current_pos here
    // because those tiles may be freed if we chain-shove their occupants.
    let firmly_occupied: std::collections::HashSet<Position> = creeps
        .values()
        .filter(|c| c.resolved)
        .map(|c| c.final_pos)
        .collect();

    // Build a map of current_pos -> entity for unresolved creeps (and idle
    // creeps) so we can find chain-shove candidates.
    let mut unresolved_pos_to_entity: HashMap<Position, Handle> = creeps
        .iter()
        .filter(|(h, c)| !c.resolved && **h != entity)
        .map(|(h, c)| (c.current_pos, *h))
        .collect();
    for (pos, handle) in idle_creep_positions.iter() {
        unresolved_pos_to_entity.entry(*pos).or_insert(*handle);
    }

    // Prefer resolving the creep to its desired position before trying
    // arbitrary adjacent tiles. This lets convoy creeps advance along their
    // path instead of being shoved sideways, preventing oscillation when
    // multiple adjacent creeps are moving in the same direction.
    if let Some(desired) = creep.desired_pos {
        if desired != creep.current_pos
            && is_tile_walkable(desired)
            && !firmly_occupied.contains(&desired)
        {
            // Respect anchor constraint.
            let anchor_ok = creep
                .anchor
                .map(|ac| desired.get_range_to(ac.position) <= ac.range)
                .unwrap_or(true);

            if anchor_ok {
                // If an unresolved creep is at the desired tile, try to
                // chain-resolve it first (recursive, depth + 1).
                let tile_free =
                    if let Some(&neighbor_entity) = unresolved_pos_to_entity.get(&desired) {
                        try_shove(
                            neighbor_entity,
                            creeps,
                            idle_creep_positions,
                            is_tile_walkable,
                            depth + 1,
                        )
                    } else {
                        true
                    };

                if tile_free {
                    let creep = creeps.get_mut(&entity).unwrap();
                    creep.resolved = true;
                    creep.final_pos = desired;
                    return true;
                }
            }
        }
    }

    for direction in Direction::iter() {
        let offset = direction.into_offset();
        let nx = pos.x().u8() as i32 + offset.0;
        let ny = pos.y().u8() as i32 + offset.1;

        // Room boundary check.
        if !(0..=49).contains(&nx) || !(0..=49).contains(&ny) {
            continue;
        }

        let neighbor = Position::new(
            RoomCoordinate::new(nx as u8).unwrap(),
            RoomCoordinate::new(ny as u8).unwrap(),
            pos.room_name(),
        );

        if !is_tile_walkable(neighbor) {
            continue;
        }

        // Already firmly claimed by a resolved creep.
        if firmly_occupied.contains(&neighbor) {
            continue;
        }

        // Respect anchor constraint: only shove to tiles within anchor range.
        if let Some(anchor) = creep.anchor {
            if neighbor.get_range_to(anchor.position) > anchor.range {
                continue;
            }
        }

        // Check if an unresolved creep is sitting on this tile.
        if let Some(&neighbor_entity) = unresolved_pos_to_entity.get(&neighbor) {
            // Try to chain-shove the occupant to free this tile.
            let chain_shoved = try_shove(
                neighbor_entity,
                creeps,
                idle_creep_positions,
                is_tile_walkable,
                depth + 1,
            );
            if !chain_shoved {
                continue; // Can't free this tile, try next direction.
            }
        }

        // Tile is free (either empty or just freed by chain-shove). Shove here.
        let creep = creeps.get_mut(&entity).unwrap();
        creep.resolved = true;
        creep.final_pos = neighbor;
        return true;
    }

    false
}

/// Utility trait extension for Direction.
pub(crate) trait DirectionExt {
    fn into_offset(self) -> (i32, i32);
}

impl DirectionExt for Direction {
    fn into_offset(self) -> (i32, i32) {
        match self {
            Direction::Top => (0, -1),
            Direction::TopRight => (1, -1),
            Direction::Right => (1, 0),
            Direction::BottomRight => (1, 1),
            Direction::Bottom => (0, 1),
            Direction::BottomLeft => (-1, 1),
            Direction::Left => (-1, 0),
            Direction::TopLeft => (-1, -1),
        }
    }
}
