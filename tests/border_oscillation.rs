//! RED reproduction harness for creep BORDER / EXIT-TILE OSCILLATION.
//!
//! Drives ONE creep through the real `MovementSystem::process` pipeline (path
//! caching, path consumption, resolver, move execution) via a mock
//! `MovementSystemExternal`, across a multi-room `MoveTo`. Each tick we:
//!   1. build a `MoveTo` request and run `process`,
//!   2. capture the chosen `move_direction` the system emitted,
//!   3. apply that direction to the creep position with `Position::checked_add`
//!      — which is EXACTLY the game's border teleport: moving off an exit tile
//!      (x/y == 0/49) lands on the mirror entrance tile of the adjacent room.
//!
//! The per-tick trace prints (tick, room, x, y, chosen_next, repathed?, result)
//! and the test asserts NO position is ever revisited (an oscillation = the
//! creep steps back onto a tile it already occupied, in particular bouncing
//! across the border between two adjacent rooms).
//!
//! Run with:
//!   cargo test -p screeps-rover --test border_oscillation -- --nocapture

use std::collections::HashMap;

use screeps::local::{Position, RoomCoordinate, RoomName};
use screeps::Direction;

use screeps_rover::traits::CreepHandle;
use screeps_rover::{
    CostMatrixCache, CostMatrixDataSource, CostMatrixSystem, ConstructionSiteCostMatrixCache,
    CreepCostMatrixCache, CreepMovementData, LocalPathfinder, MovementData, MovementError,
    MovementSystem, MovementSystemExternal, StuctureCostMatrixCache,
};

// ---------------------------------------------------------------------------
// Mock world
// ---------------------------------------------------------------------------

type Handle = u32;

/// A creep handle backed by a shared, mutable position + captured move.
#[derive(Clone)]
struct MockCreep {
    pos: Position,
    /// Captured direction from the most recent `move_direction` call.
    last_move: std::rc::Rc<std::cell::RefCell<Option<Direction>>>,
}

impl CreepHandle for MockCreep {
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
        *self.last_move.borrow_mut() = Some(dir);
        Ok(())
    }
    fn pull(&self, _other: &Self) -> Result<(), String> {
        Ok(())
    }
    fn move_pulled_by(&self, _other: &Self) -> Result<(), String> {
        Ok(())
    }
}

/// An all-plain data source: no structures/sites/creeps anywhere, so every
/// cost matrix is empty (all terrain walkable at cost 1). This isolates the
/// pure path-following/border behaviour from any obstacle interaction.
struct PlainDataSource;

impl CostMatrixDataSource for PlainDataSource {
    fn get_structure_costs(&self, _room: RoomName) -> Option<StuctureCostMatrixCache> {
        None
    }
    fn get_construction_site_costs(&self, _room: RoomName) -> Option<ConstructionSiteCostMatrixCache> {
        None
    }
    fn get_creep_costs(&self, _room: RoomName) -> Option<CreepCostMatrixCache> {
        None
    }
}

struct MockExternal {
    creep: MockCreep,
    data: CreepMovementData,
}

impl MovementSystemExternal<Handle> for MockExternal {
    type Creep = MockCreep;

    fn get_creep(&self, _entity: Handle) -> Result<Self::Creep, MovementError> {
        Ok(self.creep.clone())
    }

    fn get_creep_movement_data(
        &mut self,
        _entity: Handle,
    ) -> Result<&mut CreepMovementData, MovementError> {
        Ok(&mut self.data)
    }

    fn get_entity_position(&self, _entity: Handle) -> Option<Position> {
        Some(self.creep.pos)
    }
}

fn pos(x: u8, y: u8, room: &str) -> Position {
    Position::new(
        RoomCoordinate::new(x).unwrap(),
        RoomCoordinate::new(y).unwrap(),
        room.parse::<RoomName>().unwrap(),
    )
}

/// Drive the movement system tick by tick, applying the game's border-teleport
/// move semantics, and return the per-tick trace.
fn run_trace(origin: Position, destination: Position, range: u32, max_ticks: usize) -> Vec<Position> {
    run_trace_opts(origin, destination, range, max_ticks, false)
}

/// As `run_trace`, but when `force_repath` is set the cached path is discarded
/// at the start of every tick, so the system pathfinds fresh from the creep's
/// current position each tick. This mimics the live "recompute every tick"
/// regime (short reuse_path_length, stuck escalation, or a job that re-issues
/// the request), the regime where border thrash was historically observed.
fn run_trace_opts(
    origin: Position,
    destination: Position,
    range: u32,
    max_ticks: usize,
    force_repath: bool,
) -> Vec<Position> {
    run_trace_full(origin, destination, range, max_ticks, force_repath, 0)
}

/// As `run_trace_opts`, with `teleport_y_skew`: when the creep crosses a
/// vertical (east/west) border, its landing y is nudged by this many tiles.
/// This models the game placing the creep on a mirror entrance tile that is
/// NOT the exact tile the rover path planned (which happens with diagonal exit
/// moves), stress-testing whether a path/landing mismatch at the border makes
/// the very next step re-cross.
fn run_trace_full(
    origin: Position,
    destination: Position,
    range: u32,
    max_ticks: usize,
    force_repath: bool,
    teleport_y_skew: i32,
) -> Vec<Position> {
    let last_move = std::rc::Rc::new(std::cell::RefCell::new(None));
    let mut cache = CostMatrixCache::default();
    let mut pathfinder = LocalPathfinder;

    let mut external = MockExternal {
        creep: MockCreep {
            pos: origin,
            last_move: last_move.clone(),
        },
        data: CreepMovementData::default(),
    };

    let mut trace: Vec<Position> = vec![origin];

    println!(
        "\n=== border oscillation trace: {:?} -> {:?} (range {}) ===",
        (origin.x().u8(), origin.y().u8(), origin.room_name().to_string()),
        (
            destination.x().u8(),
            destination.y().u8(),
            destination.room_name().to_string()
        ),
        range
    );
    println!("tick | room  |  x  y | chosen_next     | result");

    for tick in 0..max_ticks {
        // Snapshot the cached path length to detect repaths.
        // (A repath replaces path_data; we surface the MovementResult and the
        // chosen direction which together reveal stepping behaviour.)
        last_move.borrow_mut().take();

        if force_repath {
            // Discard any cached path so the system re-pathfinds this tick.
            external.data = CreepMovementData::default();
        }

        let mut cost_matrix_system = CostMatrixSystem::new(&mut cache, Box::new(PlainDataSource));
        let mut movement =
            MovementSystem::<Handle>::new(&mut cost_matrix_system, &mut pathfinder, None);

        let mut movement_data = MovementData::<Handle>::new();
        movement_data.move_to(1u32, destination).range(range);

        let results = movement.process(&mut external, movement_data);
        let result = results.get(&1u32).cloned();

        let before = external.creep.pos;
        let chosen = last_move.borrow().clone();

        // Apply the move exactly as the game does: applying a Direction offset
        // to an exit-tile Position crosses the border to the mirror entrance
        // tile of the adjacent room (Position::checked_add is world-coordinate
        // aware).
        let after = match chosen {
            Some(dir) => {
                let stepped = before.checked_add_direction(dir).unwrap_or(before);
                // If we crossed a vertical (E/W) border, optionally skew the
                // landing y to model a mirror-tile placement mismatch.
                if teleport_y_skew != 0 && stepped.room_name() != before.room_name() {
                    stepped
                        .checked_add((0, teleport_y_skew))
                        .unwrap_or(stepped)
                } else {
                    stepped
                }
            }
            None => before,
        };
        external.creep.pos = after;

        let chosen_str = chosen
            .map(|d| {
                let np = before.checked_add_direction(d).unwrap_or(before);
                format!(
                    "{:>2},{:<2} {:<6}",
                    np.x().u8(),
                    np.y().u8(),
                    np.room_name().to_string()
                )
            })
            .unwrap_or_else(|| "     (none)     ".to_string());

        println!(
            "{:>4} | {:<5} | {:>2} {:<2} | {} | {:?}",
            tick,
            before.room_name().to_string(),
            before.x().u8(),
            before.y().u8(),
            chosen_str,
            result
        );

        trace.push(after);

        if before.get_range_to(destination) <= range {
            println!("--- reached destination at tick {} ---", tick);
            break;
        }
    }

    trace
}

/// Detect a genuine OSCILLATION: the creep moves back onto a tile it already
/// occupied. Consecutive-duplicate positions (a stationary tick — stuck, or
/// idling at the destination after arrival) are collapsed first, so only real
/// back-steps count. Returns (index-in-collapsed-trace, position) if any.
fn first_revisit(trace: &[Position]) -> Option<(usize, Position)> {
    // Collapse consecutive duplicates (stationary ticks are not oscillation).
    let mut moves: Vec<Position> = Vec::new();
    for &p in trace {
        if moves.last().map(|&q| q != p).unwrap_or(true) {
            moves.push(p);
        }
    }
    let mut seen: HashMap<(u8, u8, RoomName), usize> = HashMap::new();
    for (i, p) in moves.iter().enumerate() {
        let key = (p.x().u8(), p.y().u8(), p.room_name());
        if seen.contains_key(&key) {
            return Some((i, *p));
        }
        seen.insert(key, i);
    }
    None
}

/// Straight west→east crossing: origin near the east edge of W1N1, destination
/// two rooms east (W3N1). The optimal path steps onto the x=49 exit tile of
/// W1N1, then teleports to x=0 of W2N1, etc.
#[test]
fn border_crossing_west_to_east_center_row() {
    // Short hop straight across ONE border so the A* search always completes.
    // Screeps world coords: higher W = further WEST, so W1N1 -> W2N1 crosses the
    // x=0 (west) border. Origin near x=0 of W1N1, destination near x=49 of W2N1.
    let origin = pos(3, 25, "W1N1");
    let destination = pos(46, 25, "W2N1");
    let trace = run_trace(origin, destination, 0, 200);

    let revisit = first_revisit(&trace);
    if let Some((i, p)) = revisit {
        panic!(
            "OSCILLATION: creep revisited tile ({}, {}) in {} at trace index {} — trace={:?}",
            p.x().u8(),
            p.y().u8(),
            p.room_name(),
            i,
            trace
                .iter()
                .map(|q| (q.x().u8(), q.y().u8(), q.room_name().to_string()))
                .collect::<Vec<_>>()
        );
    }
}

/// Same crossing but destination only ONE room east, exit exactly on the
/// border row — stresses the path-consumption window right at the transition.
#[test]
fn border_crossing_west_to_east_one_room() {
    // East crossing (short hop): W1N1 -> W0N1 crosses the x=49 (east) border
    // (west-to-east ordering is W2 < W1 < W0 < E0, so W0N1 is one room EAST).
    let origin = pos(46, 10, "W1N1");
    let destination = pos(3, 10, "W0N1");
    let trace = run_trace(origin, destination, 0, 120);

    if let Some((i, p)) = first_revisit(&trace) {
        panic!(
            "OSCILLATION: creep revisited tile ({}, {}) in {} at trace index {}",
            p.x().u8(),
            p.y().u8(),
            p.room_name(),
            i
        );
    }
}

/// MIRROR-MISMATCH regime: on each vertical-border crossing the creep's landing
/// y is skewed by 1, modelling the game placing it on a different entrance tile
/// than the rover path planned (as happens with diagonal exit moves). If a stale
/// cached path plus an off-path landing tile can produce a backward step, this
/// is where it surfaces.
#[test]
fn border_crossing_mirror_tile_mismatch() {
    let origin = pos(3, 25, "W1N1");
    let destination = pos(46, 25, "W2N1");
    let trace = run_trace_full(origin, destination, 0, 200, false, 1);

    if let Some((i, p)) = first_revisit(&trace) {
        panic!(
            "OSCILLATION (mirror-mismatch): revisited ({}, {}) in {} at index {} — trace={:?}",
            p.x().u8(),
            p.y().u8(),
            p.room_name(),
            i,
            trace
                .iter()
                .map(|q| (q.x().u8(), q.y().u8(), q.room_name().to_string()))
                .collect::<Vec<_>>()
        );
    }
}

/// FORCE-REPATH regime: the cached path is discarded every tick, so the system
/// pathfinds fresh from the creep's current position each tick. This is the
/// regime where live "border thrash" (aaac0f7) was reported — a creep sitting
/// on an entrance/exit tile and re-picking a first step that re-crosses.
#[test]
fn border_crossing_force_repath_every_tick() {
    let origin = pos(3, 25, "W1N1");
    let destination = pos(46, 25, "W2N1");
    let trace = run_trace_opts(origin, destination, 0, 200, true);

    if let Some((i, p)) = first_revisit(&trace) {
        panic!(
            "OSCILLATION (force-repath): creep revisited tile ({}, {}) in {} at index {} — trace={:?}",
            p.x().u8(),
            p.y().u8(),
            p.room_name(),
            i,
            trace
                .iter()
                .map(|q| (q.x().u8(), q.y().u8(), q.room_name().to_string()))
                .collect::<Vec<_>>()
        );
    }
}
