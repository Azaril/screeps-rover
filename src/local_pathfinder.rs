//! Pure-Rust, **headless** pathfinding — a [`PathfindingProvider`] that needs no game runtime and
//! no JS (the live impl, `ScreepsPathfinder`, is `screeps`-feature-gated and delegates to the
//! server's `PathFinder`). This is the irreducible search leaf for offline consumers — chiefly the
//! combat micro-sim, which routes movement through rover (so live and sim share one intent system)
//! while the engine port acts as the authoritative "server" that resolves the requested moves
//! (ADR 0006 §B.2, movement-architecture decision 2026-06-17). It lives here, in the pathfinding
//! system, rather than as a one-off in the harness.
//!
//! It is a weighted-grid search over the caller-supplied [`LocalCostMatrix`] for a **single room**
//! (the sim models one 50×50 room): A* for [`PathfindingProvider::search`], a cost-bounded Dijkstra
//! for [`PathfindingProvider::search_many`] (seek = reach any goal's range; flee = escape all goal
//! ranges, falling back to the best-effort tile). Tile cost: `255` ⇒ impassable, `0` ⇒
//! `plain_cost` (the terrain default — headless has no `Terrain`, so callers bake walls/swamp into
//! the matrix), else the matrix value. Returned paths **exclude the origin** (matching
//! `screeps::pathfinder::search`), so `path[0]` is the next step.

use crate::traits::*;
use screeps::local::*;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

const DIM: usize = 50;
const IMPASSABLE: u8 = u8::MAX;

/// 8-directional neighbour offsets.
const NEIGHBORS: [(i32, i32); 8] = [
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
];

/// A dense per-tile cost grid read once from a [`LocalCostMatrix`].
type Grid = [[u8; DIM]; DIM];
/// Per-tile predecessor table for path reconstruction.
type CameFrom = [[Option<(u8, u8)>; DIM]; DIM];

/// A headless [`PathfindingProvider`] (see module docs). Zero-sized; construct with `LocalPathfinder`.
pub struct LocalPathfinder;

fn cheby(ax: i32, ay: i32, bx: i32, by: i32) -> u32 {
    (ax - bx).abs().max((ay - by).abs()) as u32
}

fn to_pos(x: u8, y: u8, room: RoomName) -> Position {
    Position::new(RoomCoordinate::new(x).unwrap(), RoomCoordinate::new(y).unwrap(), room)
}

/// Room-agnostic `RoomXY` for indexing a (room-independent) cost matrix.
fn rxy(x: usize, y: usize) -> RoomXY {
    RoomXY::checked_new(x as u8, y as u8).expect("0..50 in bounds")
}

/// The **"moving-maximum" footprint transform** (P2.M1): given a per-tile cost matrix, produce one
/// where each tile's value is the MAX cost over the `w`×`h` block anchored at that tile (extending
/// toward +x/+y). A block cell that is impassable (255) — or a block that would run off the room —
/// makes the **anchor** impassable. Pathfinding the anchor over the transformed matrix routes a
/// `w`×`h` body (a squad's bounding box) as a unit: it never threads a gap narrower than its
/// footprint and never clips an obstacle. Generalizes the (dead) 2×2 `apply_quad_cost_overlay` to
/// arbitrary W×H. Pair with [`LocalPathfinder`] (search the anchor over the transformed matrix).
pub fn moving_maximum(cm: &LocalCostMatrix, w: u8, h: u8) -> LocalCostMatrix {
    let (w, h) = (w.max(1) as usize, h.max(1) as usize);
    let mut out = LocalCostMatrix::new();
    for ax in 0..DIM {
        for ay in 0..DIM {
            let val = if ax + w > DIM || ay + h > DIM {
                IMPASSABLE // the footprint would extend off the room
            } else {
                let mut m = 0u8;
                'block: for x in ax..ax + w {
                    for y in ay..ay + h {
                        let c = cm.get(rxy(x, y));
                        if c == IMPASSABLE {
                            m = IMPASSABLE;
                            break 'block;
                        }
                        m = m.max(c);
                    }
                }
                m
            };
            if val != 0 {
                out.set(rxy(ax, ay), val);
            }
        }
    }
    out
}

/// Read the cost matrix into a dense grid once (avoids per-expansion `RoomXY` construction).
fn snapshot(cm: &LocalCostMatrix, room: RoomName) -> Box<Grid> {
    let mut grid = Box::new([[0u8; DIM]; DIM]);
    for (x, col) in grid.iter_mut().enumerate() {
        for (y, cell) in col.iter_mut().enumerate() {
            *cell = cm.get(to_pos(x as u8, y as u8, room).xy());
        }
    }
    grid
}

/// Cost to ENTER tile `(x,y)`; `None` if impassable. `0` in the matrix means "terrain default" →
/// `plain_cost` (floored at 1 so search terminates), matching how a cost matrix overlays terrain.
fn enter_cost(grid: &Grid, x: usize, y: usize, plain_cost: u8) -> Option<u32> {
    match grid[x][y] {
        IMPASSABLE => None,
        0 => Some(plain_cost.max(1) as u32),
        c => Some(c as u32),
    }
}

/// Rebuild the forward path (origin-exclusive) from the came-from table.
fn reconstruct(came: &CameFrom, origin: (u8, u8), node: (u8, u8), room: RoomName) -> Vec<Position> {
    let mut rev = Vec::new();
    let mut cur = node;
    while cur != origin {
        rev.push(cur);
        match came[cur.0 as usize][cur.1 as usize] {
            Some(p) => cur = p,
            None => break,
        }
    }
    rev.reverse();
    rev.into_iter().map(|(x, y)| to_pos(x, y, room)).collect()
}

impl LocalPathfinder {
    /// Shared cost-bounded grid search. `satisfied(x,y)` is the goal predicate; `score(x,y)` ranks
    /// best-effort tiles (lower is better — distance-to-target for seek, negated-distance for flee,
    /// an arbitrary cost for scored). `dijkstra` forces a uniform-cost flood (priority = `g`,
    /// exploring outward by path cost and evaluating `score` at each tile) — used by the scored
    /// search where `score` is an arbitrary positive cost, not an admissible distance heuristic.
    /// Returns `(path, incomplete)`.
    #[allow(clippy::too_many_arguments)]
    fn run<S, B>(
        grid: &Grid,
        origin: (u8, u8),
        room: RoomName,
        max_ops: u32,
        plain_cost: u8,
        dijkstra: bool,
        satisfied: S,
        score: B,
    ) -> (Vec<Position>, bool)
    where
        S: Fn(i32, i32) -> bool,
        B: Fn(i32, i32) -> i64,
    {
        let priority = |x: u8, y: u8, g: u32| if dijkstra { g } else { score_priority(&score, x, y, g) };
        let (ox, oy) = origin;
        if satisfied(ox as i32, oy as i32) {
            return (Vec::new(), false); // already there
        }
        let mut g = Box::new([[u32::MAX; DIM]; DIM]);
        let mut came: Box<CameFrom> = Box::new([[None; DIM]; DIM]);
        // Min-heap on (priority, g, x, y). A* priority = g + h; Dijkstra priority = g.
        let mut heap: BinaryHeap<Reverse<(u32, u32, u8, u8)>> = BinaryHeap::new();
        g[ox as usize][oy as usize] = 0;
        heap.push(Reverse((priority(ox, oy, 0), 0, ox, oy)));
        let mut best = (score(ox as i32, oy as i32), ox, oy);
        let mut ops = 0u32;
        while let Some(Reverse((_pri, gc, x, y))) = heap.pop() {
            if gc > g[x as usize][y as usize] {
                continue; // stale heap entry
            }
            if satisfied(x as i32, y as i32) {
                return (reconstruct(&came, origin, (x, y), room), false);
            }
            let s = score(x as i32, y as i32);
            if s < best.0 {
                best = (s, x, y);
            }
            ops += 1;
            if ops >= max_ops {
                return (reconstruct(&came, origin, (best.1, best.2), room), true);
            }
            for (dx, dy) in NEIGHBORS {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;
                if !(0..DIM as i32).contains(&nx) || !(0..DIM as i32).contains(&ny) {
                    continue;
                }
                let (nx, ny) = (nx as usize, ny as usize);
                let step = match enter_cost(grid, nx, ny, plain_cost) {
                    Some(c) => c,
                    None => continue,
                };
                let ng = gc.saturating_add(step);
                if ng < g[nx][ny] {
                    g[nx][ny] = ng;
                    came[nx][ny] = Some((x, y));
                    heap.push(Reverse((priority(nx as u8, ny as u8, ng), ng, nx as u8, ny as u8)));
                }
            }
        }
        (reconstruct(&came, origin, (best.1, best.2), room), true)
    }
}

/// A* priority for seek (g + admissible h); for flee `score` is negative so the heap still explores
/// outward by cost. We approximate the priority as `g + max(0, score)` — for seek `score` is the
/// remaining Chebyshev distance (admissible), for flee `score` is ≤ 0 so priority collapses to `g`
/// (a uniform-cost Dijkstra outward), which is what flee wants.
fn score_priority<B: Fn(i32, i32) -> i64>(score: &B, x: u8, y: u8, g: u32) -> u32 {
    let h = score(x as i32, y as i32).max(0) as u32;
    g.saturating_add(h)
}

impl PathfindingProvider for LocalPathfinder {
    fn search(
        &mut self,
        origin: Position,
        goal: Position,
        range: u32,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        _swamp_cost: u8,
    ) -> PathfindingResult {
        let room = origin.room_name();
        if goal.room_name() != room {
            return PathfindingResult { path: Vec::new(), incomplete: true };
        }
        let grid = match room_callback(room) {
            Some(cm) => snapshot(&cm, room),
            None => return PathfindingResult { path: Vec::new(), incomplete: true },
        };
        let (gx, gy) = (goal.x().u8() as i32, goal.y().u8() as i32);
        let satisfied = |x: i32, y: i32| cheby(x, y, gx, gy) <= range;
        let score = |x: i32, y: i32| cheby(x, y, gx, gy) as i64; // minimize distance to goal
        let (path, incomplete) =
            Self::run(&grid, (origin.x().u8(), origin.y().u8()), room, max_ops, plain_cost, false, satisfied, score);
        PathfindingResult { path, incomplete }
    }

    fn search_many(
        &mut self,
        origin: Position,
        goals: &[(Position, u32)],
        flee: bool,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        _swamp_cost: u8,
    ) -> PathfindingResult {
        let room = origin.room_name();
        let local: Vec<(i32, i32, u32)> = goals
            .iter()
            .filter(|(p, _)| p.room_name() == room)
            .map(|(p, r)| (p.x().u8() as i32, p.y().u8() as i32, *r))
            .collect();
        if local.is_empty() {
            return PathfindingResult { path: Vec::new(), incomplete: true };
        }
        let grid = match room_callback(room) {
            Some(cm) => snapshot(&cm, room),
            None => return PathfindingResult { path: Vec::new(), incomplete: true },
        };
        let min_dist = |x: i32, y: i32| local.iter().map(|(gx, gy, _)| cheby(x, y, *gx, *gy)).min().unwrap();
        let (path, incomplete) = if flee {
            // Goal: outside EVERY flee range. Best-effort: maximize the min distance (score negated).
            let satisfied = |x: i32, y: i32| local.iter().all(|(gx, gy, r)| cheby(x, y, *gx, *gy) > *r);
            let score = |x: i32, y: i32| -(min_dist(x, y) as i64);
            Self::run(&grid, (origin.x().u8(), origin.y().u8()), room, max_ops, plain_cost, false, satisfied, score)
        } else {
            // Goal: within ANY goal's range. Best-effort: minimize the min distance.
            let satisfied = |x: i32, y: i32| local.iter().any(|(gx, gy, r)| cheby(x, y, *gx, *gy) <= *r);
            let score = |x: i32, y: i32| min_dist(x, y) as i64;
            Self::run(&grid, (origin.x().u8(), origin.y().u8()), room, max_ops, plain_cost, false, satisfied, score)
        };
        PathfindingResult { path, incomplete }
    }

    fn search_scored(
        &mut self,
        origin: Position,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        _swamp_cost: u8,
        cost: &dyn Fn(Position) -> i64,
    ) -> PathfindingResult {
        let room = origin.room_name();
        let grid = match room_callback(room) {
            Some(cm) => snapshot(&cm, room),
            None => return PathfindingResult { path: Vec::new(), incomplete: true },
        };
        // Flood the bounded region (never "satisfied" → run to max_ops or exhaustion) by path cost
        // (Dijkstra), evaluating the caller's tile cost at each; `run` returns the min-cost tile seen.
        let satisfied = |_x: i32, _y: i32| false;
        let score = |x: i32, y: i32| cost(to_pos(x as u8, y as u8, room));
        let (path, incomplete) =
            Self::run(&grid, (origin.x().u8(), origin.y().u8()), room, max_ops, plain_cost, true, satisfied, score);
        PathfindingResult { path, incomplete }
    }

    fn find_route(
        &self,
        from: RoomName,
        _to: RoomName,
        _room_callback: &dyn Fn(RoomName, RoomName) -> f64,
    ) -> Result<Vec<RouteStep>, String> {
        // Single-room sim: the trivial route. (Multi-room is out of the combat-sim scope.)
        Ok(vec![RouteStep { room: from }])
    }

    fn get_room_linear_distance(&self, from: RoomName, to: RoomName) -> u32 {
        if from == to {
            0
        } else {
            1
        }
    }

    fn is_tile_walkable(&self, _pos: Position) -> bool {
        // Headless: walkability is encoded in the cost matrix (255 = impassable), not queried from
        // a live `Terrain`. Callers that need a terrain check supply it through the matrix.
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn room() -> RoomName {
        "W1N1".parse().unwrap()
    }
    fn pos(x: u8, y: u8) -> Position {
        to_pos(x, y, room())
    }
    /// An empty (all-plain) matrix.
    fn empty_cm() -> LocalCostMatrix {
        LocalCostMatrix::new()
    }
    fn block(cm: &mut LocalCostMatrix, x: u8, y: u8) {
        cm.set(pos(x, y).xy(), IMPASSABLE);
    }

    /// Assert `path` is a contiguous chain of single-tile steps starting adjacent to `origin`
    /// (origin itself excluded) and ending at `goal`.
    fn assert_contiguous(path: &[Position], origin: Position, goal: Position) {
        assert!(!path.is_empty());
        assert_ne!(path[0], origin, "origin is excluded from the path");
        assert_eq!(path[0].get_range_to(origin), 1, "first step is adjacent to origin");
        for w in path.windows(2) {
            assert_eq!(w[0].get_range_to(w[1]), 1, "each step is single-tile");
        }
        assert_eq!(*path.last().unwrap(), goal, "ends at the goal");
    }

    #[test]
    fn straight_line_on_open_terrain() {
        let mut pf = LocalPathfinder;
        let mut cb = |_r| Some(empty_cm());
        let r = pf.search(pos(5, 5), pos(10, 5), 0, &mut cb, 2000, 1, 5);
        assert!(!r.incomplete);
        // Chebyshev-optimal: 5 single-tile steps (diagonals are equal-cost, so the exact tiles vary).
        assert_eq!(r.path.len(), 5, "five steps to span 5 tiles");
        assert_contiguous(&r.path, pos(5, 5), pos(10, 5));
    }

    #[test]
    fn already_in_range_is_empty_and_complete() {
        let mut pf = LocalPathfinder;
        let mut cb = |_r| Some(empty_cm());
        let r = pf.search(pos(5, 5), pos(7, 5), 3, &mut cb, 2000, 1, 5);
        assert!(!r.incomplete);
        assert!(r.path.is_empty(), "range 3 already satisfied at distance 2");
    }

    #[test]
    fn detours_around_a_wall() {
        // A wall column at x=8 spanning y=3..=7, with the goal behind it. The path must route
        // around (through y<3 or y>7), never through the wall.
        let mut pf = LocalPathfinder;
        let mut cb = || {
            let mut cm = empty_cm();
            for y in 3..=7 {
                block(&mut cm, 8, y);
            }
            cm
        };
        let mut cbf = |_r| Some(cb());
        let r = pf.search(pos(6, 5), pos(10, 5), 0, &mut cbf, 5000, 1, 5);
        assert!(!r.incomplete, "a route around exists");
        assert_eq!(*r.path.last().unwrap(), pos(10, 5));
        assert!(r.path.iter().all(|p| !(p.x().u8() == 8 && (3..=7).contains(&p.y().u8()))), "never steps into the wall");
    }

    #[test]
    fn no_route_through_a_sealed_wall_is_incomplete() {
        // A full wall column at x=8 (y=0..=49) seals the goal off entirely.
        let mut pf = LocalPathfinder;
        let mut cbf = |_r| {
            let mut cm = empty_cm();
            for y in 0..=49 {
                block(&mut cm, 8, y);
            }
            Some(cm)
        };
        let r = pf.search(pos(6, 5), pos(10, 5), 0, &mut cbf, 5000, 1, 5);
        assert!(r.incomplete, "sealed off → incomplete");
    }

    #[test]
    fn flee_increases_distance_from_the_threat() {
        // Origin at range 1 from a threat with flee-range 3 → flee to distance > 3.
        let mut pf = LocalPathfinder;
        let mut cbf = |_r| Some(empty_cm());
        let threat = pos(25, 25);
        let r = pf.search_many(pos(26, 25), &[(threat, 3)], true, &mut cbf, 3000, 1, 5);
        assert!(!r.incomplete, "open room → can flee out of range");
        let end = *r.path.last().unwrap();
        assert!(end.get_range_to(threat) > 3, "ends outside the flee range");
    }

    #[test]
    fn moving_maximum_blocks_anchors_that_would_clip_a_wall() {
        let mut cm = empty_cm();
        block(&mut cm, 10, 10);
        let mm = moving_maximum(&cm, 2, 2);
        // Every 2×2 block containing (10,10) — anchors (9,9)/(9,10)/(10,9)/(10,10) — is impassable.
        for &(ax, ay) in &[(9u8, 9u8), (9, 10), (10, 9), (10, 10)] {
            assert_eq!(mm.get(pos(ax, ay).xy()), IMPASSABLE, "2×2 anchor ({},{}) clips the wall", ax, ay);
        }
        assert_eq!(mm.get(pos(7, 7).xy()), 0, "a block clear of the wall stays passable");
    }

    #[test]
    fn moving_maximum_seals_a_one_wide_gap_for_a_2x2() {
        // Wall column x=10 with a single-tile gap at (10,25): a 2×2 body cannot thread it.
        let mut cm = empty_cm();
        for y in 0..=49u8 {
            if y != 25 {
                block(&mut cm, 10, y);
            }
        }
        let mm2 = moving_maximum(&cm, 2, 2);
        for &(ax, ay) in &[(9u8, 24u8), (9, 25), (10, 24), (10, 25)] {
            assert_eq!(mm2.get(pos(ax, ay).xy()), IMPASSABLE, "2×2 can't pass the 1-wide gap (anchor {},{})", ax, ay);
        }
        // A 1×1 body still fits the gap.
        assert_eq!(moving_maximum(&cm, 1, 1).get(pos(10, 25).xy()), 0, "1×1 fits the gap");
    }

    #[test]
    fn moving_maximum_blocks_off_room_footprints() {
        let mm = moving_maximum(&empty_cm(), 2, 2);
        assert_eq!(mm.get(pos(49, 25).xy()), IMPASSABLE, "2×2 runs off the +x edge");
        assert_eq!(mm.get(pos(25, 49).xy()), IMPASSABLE, "2×2 runs off the +y edge");
        assert_eq!(mm.get(pos(48, 48).xy()), 0, "fits just inside the corner");
    }

    #[test]
    fn search_scored_returns_min_cost_reachable_tile() {
        // Cost = distance to a goal tile (min, 0, at the goal). Open room → the search reaches it.
        let mut pf = LocalPathfinder;
        let mut cb = |_r| Some(empty_cm());
        let goal = pos(30, 25);
        let cost = |p: Position| p.get_range_to(goal) as i64;
        let r = pf.search_scored(pos(25, 25), &mut cb, 5000, 1, 5, &cost);
        assert!(!r.path.is_empty(), "moves toward the min-cost tile");
        assert_eq!(*r.path.last().unwrap(), goal, "ends at the min-cost (zero) tile");
    }

    #[test]
    fn search_scored_bounded_by_max_ops_returns_best_so_far() {
        // Tiny op budget: returns the best tile explored so far (toward lower cost), incomplete.
        let mut pf = LocalPathfinder;
        let mut cb = |_r| Some(empty_cm());
        let goal = pos(5, 5);
        let cost = |p: Position| p.get_range_to(goal) as i64;
        let origin = pos(25, 25);
        let r = pf.search_scored(origin, &mut cb, 10, 1, 5, &cost);
        assert!(r.incomplete, "bounded by max_ops");
        let end = *r.path.last().expect("a better tile than origin was explored");
        assert!(cost(end) < cost(origin), "moved toward lower cost (best so far)");
    }

    #[test]
    fn search_scored_never_returns_an_unreachable_tile() {
        // Origin fully walled in; the global min-cost tile is faraway + unreachable → it holds.
        let mut cbf = |_r| {
            let mut cm = empty_cm();
            for (x, y) in [(24, 24), (25, 24), (26, 24), (24, 25), (26, 25), (24, 26), (25, 26), (26, 26)] {
                block(&mut cm, x, y);
            }
            Some(cm)
        };
        let mut pf = LocalPathfinder;
        let goal = pos(45, 45);
        let cost = |p: Position| p.get_range_to(goal) as i64;
        let r = pf.search_scored(pos(25, 25), &mut cbf, 2000, 1, 5, &cost);
        assert!(r.path.is_empty(), "sealed origin → the unreachable low-cost tile is never returned (holds)");
    }

    #[test]
    fn prefers_cheaper_tiles() {
        // A swamp band (cost 10) at x=7 vs an open detour; with a wide-enough field the search
        // should avoid the expensive band when a cheap route of similar length exists. Here we just
        // assert the search completes and the costly tile is avoidable.
        let mut pf = LocalPathfinder;
        let mut cbf = |_r| {
            let mut cm = empty_cm();
            for y in 0..=49 {
                cm.set(pos(7, y).xy(), 10);
            }
            // a cheap gap at (7,5)
            cm.set(pos(7, 5).xy(), 1);
            Some(cm)
        };
        let r = pf.search(pos(5, 5), pos(9, 5), 0, &mut cbf, 5000, 1, 10);
        assert!(!r.incomplete);
        assert!(r.path.iter().any(|p| p.x().u8() == 7 && p.y().u8() == 5), "routes through the cheap gap");
    }
}
