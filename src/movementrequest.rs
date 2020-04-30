use screeps::*;

pub struct RoomOptions {
    allow_hostile: bool,
}

impl RoomOptions {
    pub fn allow_hostile(&self) -> bool {
        self.allow_hostile
    }
}

pub struct MovementRequest {
    destination: RoomPosition,
    range: u32,
    room_options: RoomOptions,
}

impl MovementRequest {
    pub fn destination(&self) -> RoomPosition {
        self.destination
    }

    pub fn range(&self) -> u32 {
        self.range
    }

    pub fn room_options(&self) -> &RoomOptions {
        &self.room_options
    }
}

impl Default for RoomOptions {
    fn default() -> Self {
        RoomOptions {
            allow_hostile: false
        }
    }
}

impl MovementRequest {
    pub fn move_to(destination: RoomPosition) -> MovementRequest {
        MovementRequest {
            destination,
            range: 0,
            room_options: RoomOptions::default()
        }
    }

    pub fn move_to_with_options(destination: RoomPosition, room_options: RoomOptions) -> MovementRequest {
        MovementRequest {
            destination,
            range: 0,
            room_options
        }
    }

    pub fn move_to_range(destination: RoomPosition, range: u32) -> MovementRequest {
        MovementRequest { 
            destination, 
            range,
            room_options: RoomOptions::default()
        }
    }

    pub fn move_to_range_with_options(destination: RoomPosition, range: u32, room_options: RoomOptions) -> MovementRequest {
        MovementRequest {
            destination,
            range,
            room_options
        }
    }
}