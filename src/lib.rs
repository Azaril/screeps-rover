mod constants;
mod costmatrix;
mod costmatrixsystem;
mod error;
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
pub use location::*;
pub use movementrequest::*;
pub use movementresult::*;
pub use movementsystem::*;
// Re-export trait types selectively to avoid method ambiguity with screeps-game-api.
// `CreepHandle` is intentionally NOT glob-exported because its `pos()` method
// conflicts with `screeps::HasPosition::pos()`. Access it via `screeps_rover::traits::CreepHandle`
// or `screeps_rover::CreepHandle` when needed explicitly.
pub use traits::{
    CostMatrixDataSource,
    PathfindingProvider, PathfindingResult, RouteStep,
    MovementVisualizer,
};
pub use utility::*;
