use screeps::constants::Direction;
use screeps::local::*;

use super::costmatrixsystem::*;

/// Abstraction over a creep game object. Provides the subset of the Screeps
/// `Creep` API that the movement system needs. The `screeps` feature provides
/// `impl CreepHandle for screeps::Creep`.
pub trait CreepHandle {
    fn pos(&self) -> Position;
    fn fatigue(&self) -> u32;
    fn spawning(&self) -> bool;
    fn move_direction(&self, dir: Direction) -> Result<(), String>;
    fn pull(&self, other: &Self) -> Result<(), String>;
    fn move_pulled_by(&self, other: &Self) -> Result<(), String>;
}

/// Abstraction over the Screeps pathfinder. The `screeps` feature provides
/// `ScreepsPathfinder` which delegates to `screeps::pathfinder::search`.
#[allow(clippy::too_many_arguments)]
pub trait PathfindingProvider {
    /// Single-target pathfinding (equivalent to `pathfinder::search`).
    fn search(
        &mut self,
        origin: Position,
        goal: Position,
        range: u32,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        swamp_cost: u8,
    ) -> PathfindingResult;

    /// Multi-target pathfinding with flee support (equivalent to `pathfinder::search_many`).
    fn search_many(
        &mut self,
        origin: Position,
        goals: &[(Position, u32)],
        flee: bool,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        swamp_cost: u8,
    ) -> PathfindingResult;

    /// Find a room-level route (equivalent to `game::map::find_route`).
    fn find_route(
        &self,
        from: RoomName,
        to: RoomName,
        room_callback: &dyn Fn(RoomName, RoomName) -> f64,
    ) -> Result<Vec<RouteStep>, String>;

    /// Get the linear distance between two rooms.
    fn get_room_linear_distance(&self, from: RoomName, to: RoomName) -> u32;

    /// Check if a tile is walkable (terrain check).
    fn is_tile_walkable(&self, pos: Position) -> bool;
}

/// Result of a pathfinding search.
pub struct PathfindingResult {
    pub path: Vec<Position>,
    pub incomplete: bool,
}

/// A step in a room-level route.
pub struct RouteStep {
    pub room: RoomName,
}

/// Abstraction over game state queries for building cost matrices.
/// Replaces the direct `game::rooms()` / `room.find()` calls in the old
/// `CostMatrixRoomAccessor`. Caching and expiration are the implementor's
/// concern.
pub trait CostMatrixDataSource {
    fn get_structure_costs(&self, room_name: RoomName) -> Option<StuctureCostMatrixCache>;
    fn get_construction_site_costs(&self, room_name: RoomName) -> Option<ConstructionSiteCostMatrixCache>;
    fn get_creep_costs(&self, room_name: RoomName) -> Option<CreepCostMatrixCache>;
}

/// Intent-based visualization callbacks for the movement system. Instead of
/// drawing primitives directly, the movement system reports *what* happened
/// and the implementor decides *how* to render it (or not).
///
/// The `screeps` feature provides `ScreepsMovementVisualizer` which renders
/// directly to `RoomVisual`. Consumers with their own rendering pipeline can
/// implement this trait to collect the intents and render them later.
pub trait MovementVisualizer {
    /// A creep is moving along a path. `path` contains the remaining
    /// waypoints in the current room (starting from the creep's next step).
    fn visualize_path(&mut self, creep_pos: Position, path: &[Position]);

    /// A creep has arrived and is anchored to a work position.
    fn visualize_anchor(&mut self, creep_pos: Position, anchor_pos: Position);

    /// A creep has arrived and is immovable (stationary guard, etc.).
    fn visualize_immovable(&mut self, creep_pos: Position);

    /// A creep is stuck and has not moved for `ticks` consecutive ticks.
    fn visualize_stuck(&mut self, creep_pos: Position, ticks: u16);

    /// A creep's movement failed (path not found, timeout, etc.).
    fn visualize_failed(&mut self, creep_pos: Position);
}
