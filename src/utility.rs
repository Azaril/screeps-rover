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
