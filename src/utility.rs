use screeps::game::map::RoomStatus;
use screeps::local::*;

/// Check room traversal based on room status values. This is pure logic with
/// no JS calls — the caller provides the status values.
///
/// When a status is `None` (e.g. on a private server where
/// `Game.map.getRoomStatus` is unavailable), it defaults to `Normal`
/// so traversal is not blocked.
pub fn can_traverse_between_room_status(from: Option<RoomStatus>, to: Option<RoomStatus>) -> bool {
    let from = from.unwrap_or(RoomStatus::Normal);
    let to = to.unwrap_or(RoomStatus::Normal);

    match to {
        RoomStatus::Normal => from == RoomStatus::Normal,
        RoomStatus::Closed => false,
        RoomStatus::Novice => from == RoomStatus::Novice,
        RoomStatus::Respawn => from == RoomStatus::Respawn,
        _ => false,
    }
}

/// Compute the linear (Chebyshev) distance between two rooms using their
/// coordinate values. This is a pure-Rust computation with no JS calls.
pub fn room_linear_distance(from: RoomName, to: RoomName) -> u32 {
    let dx = (from.x_coord() - to.x_coord()).unsigned_abs();
    let dy = (from.y_coord() - to.y_coord()).unsigned_abs();
    dx.max(dy)
}

/// Compute the minimum possible tile distance (Chebyshev) between any tile
/// in room `a` and any tile in room `b`.
///
/// Same room → 0. Adjacent rooms → 1 (tiles on shared border are range 1).
/// Rooms N apart → roughly `(N - 1) * 50 + 1` (closest border tiles).
///
/// This is a conservative lower bound used as a fast-path check: if the
/// minimum exceeds a threshold, no tile in the room can be within range.
pub fn min_tile_distance_between_rooms(a: RoomName, b: RoomName) -> u32 {
    if a == b {
        return 0;
    }
    let dx = (a.x_coord() - b.x_coord()).unsigned_abs();
    let dy = (a.y_coord() - b.y_coord()).unsigned_abs();
    // Adjacent rooms (dx=1 or dy=1) have border tiles at range 1.
    // Rooms further apart: each additional room gap adds ~50 tiles.
    let room_gap = dx.max(dy);
    (room_gap - 1) * 50 + 1
}

/// Check if two rooms can be traversed between using the live game API.
#[cfg(feature = "screeps")]
pub fn can_traverse_between_rooms(from: RoomName, to: RoomName) -> bool {
    let from_status = screeps::game::map::get_room_status(from).map(|r| r.status());
    let to_status = screeps::game::map::get_room_status(to).map(|r| r.status());
    can_traverse_between_room_status(from_status, to_status)
}

#[cfg(not(feature = "screeps"))]
pub fn can_traverse_between_rooms(_from: RoomName, _to: RoomName) -> bool {
    // Without the screeps feature, we cannot query room status.
    // Default to allowing traversal for offline testing.
    true
}
