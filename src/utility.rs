use screeps::game::map::*;
use screeps::*;

pub fn can_traverse_between_rooms(from: RoomName, to: RoomName) -> bool {
    let from_room_status = game::map::get_room_status(from).map(|r| r.status());
    let to_room_status = game::map::get_room_status(to).map(|r| r.status());

    can_traverse_between_room_status(from_room_status, to_room_status)
}

pub fn can_traverse_between_room_status(from: Option<RoomStatus>, to: Option<RoomStatus>) -> bool {
    match (from, to) {
        (Some(from), Some(to)) => match to {
            RoomStatus::Normal => from == RoomStatus::Normal,
            RoomStatus::Closed => false,
            RoomStatus::Novice => from == RoomStatus::Novice,
            RoomStatus::Respawn => from == RoomStatus::Respawn,
            _ => false,
        },
        _ => false,
    }
}
