use screeps::game::map::*;
use screeps::*;

pub fn can_traverse_between_rooms(from: RoomName, to: RoomName) -> bool {
    let from_room_status = game::map::get_room_status(from);
    let to_room_status = game::map::get_room_status(to);

    can_traverse_between_room_status(from_room_status.status(), to_room_status.status())
}

pub fn can_traverse_between_room_status(from: RoomStatus, to: RoomStatus) -> bool {
    match to {
        RoomStatus::Normal => from == RoomStatus::Normal,
        RoomStatus::Closed => false,
        RoomStatus::Novice => from == RoomStatus::Novice,
        RoomStatus::Respawn => from == RoomStatus::Respawn,
        _ => false,
    }
}