//! The squad **anchor mover** (P2.M2): a virtual anchor (the squad's shared coordinate frame) that
//! follows a **cached, footprint-aware path** instead of stepping in a straight line. This is the
//! shared mechanism behind the broken-anchor fix — the live bot's `advance_virtual_pos` walked a
//! straight `signum` line (clipping walls, so members pathing independently scattered); here the
//! anchor pathfinds once (over a [`moving_maximum`](crate::moving_maximum)-transformed cost matrix,
//! so the squad's W×H box routes as a unit), caches the path, follows it step by step, and re-paths
//! only on invalidation / staleness (the rover `CreepPathData.path` discipline).
//!
//! **On path failure it does NOT degrade to a straight-line step** (that would walk the box into the
//! obstacle it can't route around — the very bug this fixes) — it **holds** and reports
//! [`AnchorOutcome::Blocked`] so the caller can *respond*: re-target, relax the footprint (M3
//! corridors), abort the objective, or signal the manager. Re-pathing is throttled while blocked so
//! a stuck anchor never hammers the pathfinder every tick.
//!
//! Generic over a [`PathfindingProvider`]: live supplies `ScreepsPathfinder` + the game cost
//! matrix; the combat sim supplies `LocalPathfinder` + a `CombatWorld` cost source. One mechanism,
//! both worlds. Serializable so it can live on the bot's persisted squad state.

use crate::local_pathfinder::moving_maximum;
use crate::traits::*;
use screeps::local::*;
use serde::{Deserialize, Serialize};

/// Re-path a cached path that has gone stale (no progress) after this many ticks.
const REPATH_STUCK_TICKS: u16 = 3;
/// After a failed re-path, wait this many ticks before searching again (throttle while blocked).
const BLOCKED_REPATH_COOLDOWN: u16 = 5;
/// Pathfinding op budget for an anchor search — comfortably covers a full 50×50 room (2500 tiles),
/// so within one room `incomplete` means *unreachable*, not *gave up early*.
const MAX_OPS: u32 = 4000;

/// The result of advancing the anchor — what the caller responds to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnchorOutcome {
    /// Stepped one tile along the path.
    Advanced,
    /// At the destination (no step needed / final step taken).
    Arrived,
    /// **No path to the destination for this footprint.** The anchor held position; the caller
    /// should respond (re-target, relax the footprint, abort, or signal the manager). Persisting
    /// `Blocked` (see [`AnchorPath::stuck_ticks`]) is the give-up signal.
    Blocked,
}

/// A virtual anchor following a cached footprint path toward a destination. Persisted on the
/// squad; `virtual_pos` is the anchor each member targets at `anchor + rotate(offset)`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AnchorPath {
    pub virtual_pos: Position,
    pub destination: Position,
    /// Cached steps (origin-exclusive); `cached[index..]` not yet walked.
    cached: Vec<Position>,
    index: usize,
    /// Consecutive ticks with no progress — drives stale re-path and is the caller's give-up signal.
    pub stuck_ticks: u16,
    /// Ticks remaining before re-searching after a failed path (throttle while blocked).
    repath_cooldown: u16,
}

impl AnchorPath {
    pub fn new(virtual_pos: Position, destination: Position) -> Self {
        AnchorPath {
            virtual_pos,
            destination,
            cached: Vec::new(),
            index: 0,
            stuck_ticks: 0,
            repath_cooldown: 0,
        }
    }

    /// Advance the anchor one step toward `destination`, footprint `(w,h)` (the squad's bounding
    /// box). Follows the cached path; (re)paths through `pathfinder` over the `room_callback` cost
    /// matrix (footprint-transformed) when the cache is exhausted or stale. Returns
    /// [`AnchorOutcome::Blocked`] (holding position) when no path exists — never a blind step.
    pub fn advance(
        &mut self,
        destination: Position,
        footprint: (u8, u8),
        pathfinder: &mut dyn PathfindingProvider,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
    ) -> AnchorOutcome {
        if self.repath_cooldown > 0 {
            self.repath_cooldown -= 1;
        }
        if destination != self.destination {
            self.destination = destination;
            self.invalidate();
            self.repath_cooldown = 0;
        }
        if self.virtual_pos == destination {
            self.stuck_ticks = 0;
            return AnchorOutcome::Arrived;
        }

        // (Re)path when the cache is empty/exhausted or the path has gone stale — unless we are
        // cooling down from a recent failed search.
        let needs_path = self.index >= self.cached.len() || self.stuck_ticks >= REPATH_STUCK_TICKS;
        if needs_path && self.repath_cooldown == 0 {
            self.repath(footprint, pathfinder, room_callback);
            if self.index >= self.cached.len() {
                self.repath_cooldown = BLOCKED_REPATH_COOLDOWN; // search found nothing — back off
            }
        }

        if self.index < self.cached.len() {
            let next = self.cached[self.index];
            self.index += 1;
            let moved = next != self.virtual_pos;
            self.virtual_pos = next;
            self.stuck_ticks = if moved { 0 } else { self.stuck_ticks.saturating_add(1) };
            return if self.virtual_pos == destination {
                AnchorOutcome::Arrived
            } else {
                AnchorOutcome::Advanced
            };
        }

        // No usable path — hold. The caller responds (re-target / relax footprint / abort).
        self.stuck_ticks = self.stuck_ticks.saturating_add(1);
        AnchorOutcome::Blocked
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
        // Cache only a COMPLETE path. An `incomplete` result is a best-effort partial toward an
        // unreachable goal (it ends at the obstacle) — following it would walk the box into the
        // wall, the exact failure we refuse. Treat it as no path → the caller gets `Blocked`.
        if result.incomplete || result.path.is_empty() {
            self.invalidate();
        } else {
            self.cached = result.path; // origin-exclusive steps
            self.index = 0;
            self.stuck_ticks = 0;
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
        let mut last = AnchorOutcome::Advanced;
        for _ in 0..6 {
            last = a.advance(pos(10, 25), (1, 1), &mut pf, &mut cb);
        }
        assert_eq!(a.virtual_pos, pos(10, 25));
        assert_eq!(last, AnchorOutcome::Arrived);
        assert_eq!(a.stuck_ticks, 0);
    }

    #[test]
    fn routes_a_footprint_around_a_wall() {
        // Wall column x=7, y=23..=27 between anchor (5,25) and goal (9,25). The 2×2 footprint path
        // detours around it; the anchor never sits on the wall line.
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
        for _ in 0..25 {
            assert_ne!(a.advance(pos(9, 25), (2, 2), &mut pf, &mut cb), AnchorOutcome::Blocked);
            assert_ne!(a.virtual_pos, pos(7, 25), "anchor never enters the wall line");
            if a.virtual_pos == pos(9, 25) {
                reached = true;
                break;
            }
        }
        assert!(reached, "the footprint path detours around the wall and arrives");
    }

    #[test]
    fn reports_blocked_and_holds_when_sealed_off_no_signum_step() {
        // Destination sealed behind a full wall column. The anchor must HOLD and report Blocked —
        // never blindly step toward (into) the wall.
        let start = pos(5, 25);
        let mut a = AnchorPath::new(start, pos(20, 25));
        let mut pf = LocalPathfinder;
        let mut cb = |_r| {
            let mut cm = LocalCostMatrix::new();
            for y in 0..=49u8 {
                cm.set(pos(10, y).xy(), u8::MAX); // full seal at x=10
            }
            Some(cm)
        };
        for _ in 0..12 {
            assert_eq!(a.advance(pos(20, 25), (1, 1), &mut pf, &mut cb), AnchorOutcome::Blocked);
            assert_eq!(a.virtual_pos, start, "held position — no straight-line degrade into the wall");
        }
        assert!(a.stuck_ticks >= 10, "stuck counter rises so the caller can give up");
    }

    #[test]
    fn re_paths_when_destination_changes() {
        let mut a = AnchorPath::new(pos(5, 25), pos(10, 25));
        let mut pf = LocalPathfinder;
        let mut cb = empty_cb();
        a.advance(pos(10, 25), (1, 1), &mut pf, &mut cb);
        let after_first = a.virtual_pos;
        for _ in 0..6 {
            a.advance(pos(2, 25), (1, 1), &mut pf, &mut cb);
        }
        assert!(a.virtual_pos.x().u8() < after_first.x().u8(), "re-pathed toward the new destination");
    }
}
