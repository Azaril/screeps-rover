//! The squad **anchor mover** (P2.M2): a virtual anchor (the squad's shared coordinate frame) that
//! follows a **cached, footprint-aware path** instead of stepping in a straight line. This is the
//! shared mechanism behind the broken-anchor fix — the live bot's `advance_virtual_pos` walked a
//! straight `signum` line (clipping walls, so members pathing independently scattered); here the
//! anchor pathfinds once (over a [`moving_maximum`](crate::moving_maximum)-transformed cost matrix,
//! so the squad's W×H box routes as a unit), caches the path, follows it step by step, and re-paths
//! only on invalidation / being stuck (the rover `CreepPathData.path` discipline). If the pathfind
//! fails it degrades to the old straight-line step — never to a hang.
//!
//! It is generic over a [`PathfindingProvider`]: live supplies `ScreepsPathfinder` + the game cost
//! matrix; the combat sim supplies `LocalPathfinder` + a `CombatWorld` cost source. One mechanism,
//! both worlds. Serializable so it can live on the bot's persisted squad state.

use crate::local_pathfinder::moving_maximum;
use crate::traits::*;
use screeps::local::*;
use serde::{Deserialize, Serialize};

/// Re-path if stuck (no progress) for this many consecutive ticks.
const REPATH_STUCK_TICKS: u16 = 3;
/// Pathfinding op budget for an anchor search.
const MAX_OPS: u32 = 2000;

/// A virtual anchor following a cached footprint path toward a destination. Persisted on the
/// squad; `virtual_pos` is the anchor each member targets at `anchor + rotate(offset)`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AnchorPath {
    pub virtual_pos: Position,
    pub destination: Position,
    /// Remaining cached steps (origin-exclusive), `cached[index..]` not yet walked.
    cached: Vec<Position>,
    index: usize,
    /// Consecutive ticks with no progress (drives re-path + the caller's give-up logic).
    pub stuck_ticks: u16,
}

impl AnchorPath {
    pub fn new(virtual_pos: Position, destination: Position) -> Self {
        AnchorPath { virtual_pos, destination, cached: Vec::new(), index: 0, stuck_ticks: 0 }
    }

    /// Advance the anchor one step toward `destination`, footprint `(w,h)` (the squad's bounding
    /// box). Re-paths through `pathfinder` over the `room_callback` cost matrix (footprint-transformed)
    /// when the cache is empty/exhausted/stale or the anchor is stuck; otherwise follows the cache.
    /// Falls back to a straight-line step if no path is found (never hangs).
    pub fn advance(
        &mut self,
        destination: Position,
        footprint: (u8, u8),
        pathfinder: &mut dyn PathfindingProvider,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
    ) {
        if destination != self.destination {
            self.destination = destination;
            self.invalidate();
        }
        if self.virtual_pos == destination {
            self.stuck_ticks = 0;
            return;
        }
        if self.index >= self.cached.len() || self.stuck_ticks >= REPATH_STUCK_TICKS {
            self.repath(footprint, pathfinder, room_callback);
        }
        if self.index < self.cached.len() {
            let next = self.cached[self.index];
            self.index += 1;
            if next == self.virtual_pos {
                self.stuck_ticks += 1;
            } else {
                self.virtual_pos = next;
                self.stuck_ticks = 0;
            }
        } else {
            // No usable path — degrade to the straight-line step (the prior behavior).
            self.signum_step(destination);
        }
    }

    fn invalidate(&mut self) {
        self.cached.clear();
        self.index = 0;
    }

    fn repath(
        &mut self,
        footprint: (u8, u8),
        pathfinder: &mut dyn PathfindingProvider,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
    ) {
        let room = self.virtual_pos.room_name();
        // Footprint-transform the base matrix so the W×H box routes as a unit.
        let transformed = room_callback(room).map(|base| moving_maximum(&base, footprint.0, footprint.1));
        let mut inner = |r: RoomName| if r == room { transformed.clone() } else { None };
        let result = pathfinder.search(self.virtual_pos, self.destination, 0, &mut inner, MAX_OPS, 2, 10);
        if result.path.is_empty() {
            self.invalidate();
        } else {
            self.cached = result.path; // origin-exclusive steps
            self.index = 0;
            self.stuck_ticks = 0;
        }
    }

    /// The legacy straight-line step (world coords, so it crosses room boundaries) — the fallback.
    fn signum_step(&mut self, destination: Position) {
        let (cur_wx, cur_wy) = self.virtual_pos.world_coords();
        let (dst_wx, dst_wy) = destination.world_coords();
        let dx = (dst_wx - cur_wx).signum();
        let dy = (dst_wy - cur_wy).signum();
        match Position::checked_from_world_coords(cur_wx + dx, cur_wy + dy) {
            Ok(new_pos) if new_pos != self.virtual_pos => {
                self.virtual_pos = new_pos;
                self.stuck_ticks = 0;
            }
            _ => self.stuck_ticks += 1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::local_pathfinder::LocalPathfinder;

    fn room() -> RoomName {
        "W1N1".parse().unwrap()
    }
    fn pos(x: u8, y: u8) -> Position {
        Position::new(RoomCoordinate::new(x).unwrap(), RoomCoordinate::new(y).unwrap(), room())
    }
    fn empty_cb() -> impl FnMut(RoomName) -> Option<LocalCostMatrix> {
        |_r| Some(LocalCostMatrix::new())
    }

    #[test]
    fn follows_a_straight_open_path() {
        let mut a = AnchorPath::new(pos(5, 25), pos(10, 25));
        let mut pf = LocalPathfinder;
        let mut cb = empty_cb();
        for _ in 0..5 {
            a.advance(pos(10, 25), (1, 1), &mut pf, &mut cb);
        }
        assert_eq!(a.virtual_pos, pos(10, 25), "reaches the destination");
        assert_eq!(a.stuck_ticks, 0);
    }

    #[test]
    fn routes_a_footprint_around_a_wall_unlike_signum() {
        // Wall column at x=7, y=23..=27 between anchor (5,25) and goal (9,25). A straight signum
        // step would head into the wall and stall; the footprint path detours and arrives.
        let mut a = AnchorPath::new(pos(5, 25), pos(9, 25));
        let mut pf = LocalPathfinder;
        let mut cb = |_r| {
            let mut cm = LocalCostMatrix::new();
            for y in 23..=27u8 {
                cm.set(pos(7, y).xy(), u8::MAX);
            }
            Some(cm)
        };
        let mut reached = false;
        for _ in 0..20 {
            a.advance(pos(9, 25), (2, 2), &mut pf, &mut cb);
            // never let the 2x2 footprint anchor sit on a wall-adjacent clip
            assert_ne!(a.virtual_pos, pos(7, 25), "anchor never enters the wall line");
            if a.virtual_pos == pos(9, 25) {
                reached = true;
                break;
            }
        }
        assert!(reached, "the footprint path detours around the wall and arrives");
    }

    #[test]
    fn re_paths_when_destination_changes() {
        let mut a = AnchorPath::new(pos(5, 25), pos(10, 25));
        let mut pf = LocalPathfinder;
        let mut cb = empty_cb();
        a.advance(pos(10, 25), (1, 1), &mut pf, &mut cb);
        let after_first = a.virtual_pos;
        // Flip the destination to the opposite side; the anchor should head the other way.
        for _ in 0..6 {
            a.advance(pos(2, 25), (1, 1), &mut pf, &mut cb);
        }
        assert!(a.virtual_pos.x().u8() < after_first.x().u8(), "re-pathed toward the new destination");
    }
}
