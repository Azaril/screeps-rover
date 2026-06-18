mod anchor;
mod constants;
mod costmatrix;
mod costmatrixsystem;
mod error;
pub mod gridsearch;
mod local_pathfinder;
mod location;
mod movementrequest;
mod movementresult;
mod movementsystem;
mod resolver;
pub mod traits;
mod utility;

#[cfg(feature = "screeps")]
pub mod screeps_impl;

pub use costmatrix::*;
pub use costmatrixsystem::*;
pub use error::*;
pub use anchor::{AnchorOutcome, AnchorPath};
pub use gridsearch::{reaches_room_edge, room_grid_dijkstra};
pub use local_pathfinder::{moving_maximum, LocalPathfinder};
pub use location::*;
pub use movementrequest::*;
pub use movementresult::*;
pub use movementsystem::*;
// Re-export trait types selectively to avoid method ambiguity with screeps-game-api.
// `CreepHandle` is intentionally NOT glob-exported because its `pos()` method
// conflicts with `screeps::HasPosition::pos()`. Access it via `screeps_rover::traits::CreepHandle`
// or `screeps_rover::CreepHandle` when needed explicitly.
pub use traits::{
    CostMatrixDataSource, MovementVisualizer, PathfindingProvider, PathfindingResult, RouteStep,
};
pub use utility::*;
