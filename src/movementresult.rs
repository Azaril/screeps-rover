use std::collections::HashMap;
use std::hash::Hash;

/// Outcome of movement resolution for a single creep in a given tick.
#[derive(Clone, Debug)]
pub enum MovementResult {
    /// Creep moved successfully toward target.
    Moving,
    /// Creep arrived at target (within range).
    Arrived,
    /// Creep is stuck and recovery is in progress.
    Stuck { ticks: u16 },
    /// Movement failed: target unreachable, path not found, or stuck timeout.
    Failed(MovementFailure),
}

/// Reason why movement failed for a creep.
#[derive(Clone, Debug)]
pub enum MovementFailure {
    /// `pathfinder::search` returned an incomplete path.
    PathNotFound,
    /// Stuck for too long (exceeded threshold).
    StuckTimeout { ticks: u16 },
    /// Target is in a blocked/hostile room.
    RoomBlocked,
    /// Internal error (e.g. creep not found, no direction available).
    InternalError(String),
}

/// Per-tick collection of movement results, indexed by entity handle.
/// Written by `MovementSystem::process`, read by jobs on the next tick.
pub struct MovementResults<Handle>
where
    Handle: Hash + Eq,
{
    pub results: HashMap<Handle, MovementResult>,
}

impl<Handle> Default for MovementResults<Handle>
where
    Handle: Hash + Eq,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<Handle> MovementResults<Handle>
where
    Handle: Hash + Eq,
{
    pub fn new() -> Self {
        MovementResults {
            results: HashMap::new(),
        }
    }

    pub fn get(&self, handle: &Handle) -> Option<&MovementResult> {
        self.results.get(handle)
    }

    pub fn insert(&mut self, handle: Handle, result: MovementResult) {
        self.results.insert(handle, result);
    }
}
