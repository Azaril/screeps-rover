use screeps::*;
use screeps::pathfinder::*;
use serde::*;
use std::collections::HashMap;
use std::collections::HashSet;
use super::movementrequest::*;
use std::hash::Hash;
use super::error::*;
use super::costmatrixsystem::*;
use super::utility::*;

#[derive(Serialize, Deserialize)]
pub struct CachedMovementData {
    destination: RoomPosition,
    range: u32,
}

#[derive(Default)]
pub struct MovementData<Handle> where Handle: Hash + Eq {
    requests: HashMap<Handle, MovementRequest>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<Handle> MovementData<Handle> where Handle: Hash + Eq {
    pub fn new() -> MovementData<Handle> {
        MovementData {
            requests: HashMap::new()
        }
    }

    fn request(&mut self, entity: Handle, request: MovementRequest) {
        self.requests.insert(entity, request);
    }

    pub fn move_to(&mut self, entity: Handle, destination: RoomPosition) {
        self.request(entity, MovementRequest::move_to(destination));
    }

    pub fn move_to_range(&mut self, entity: Handle, destination: RoomPosition, range: u32) {
        self.request(entity, MovementRequest::move_to_range(destination, range));
    }
}

pub struct MovementSystem<Handle> {
    phantom: std::marker::PhantomData<Handle>
}

pub trait MovementSystemExternal<Handle> {
    fn get_creep(&self, entity: Handle) -> Result<Creep, MovementError>;

    fn get_room_weight(&self, from_room_name: RoomName, to_room_name: RoomName, _current_room_name: RoomName, _room_options: &RoomOptions) -> Option<f64> {
        if !can_traverse_between_rooms(from_room_name, to_room_name) {
            return Some(f64::INFINITY);
        }

        Some(1.0)
    }
}

impl<Handle> MovementSystem<Handle> where Handle: Hash + Eq + Copy {
    pub fn process_inbuilt<S>(external: &mut S, data: MovementData<Handle>) where S: MovementSystemExternal<Handle> {
        for (entity, request) in data.requests.iter() {
            match Self::process_request_inbuilt(external, *entity, &request) {
                Ok(()) => {}
                Err(_err) => {},
            }
        }
    }
    
    pub fn process<S>(external: &mut S, cost_matrix_system: &mut CostMatrixSystem, data: MovementData<Handle>) where S: MovementSystemExternal<Handle> {
        for (entity, request) in data.requests.iter() {
            match Self::process_request(external, cost_matrix_system, *entity, &request) {
                Ok(()) => {}
                Err(_err) => {},
            }
        }
    }

    fn process_request_inbuilt<S>(external: &mut S, entity: Handle, request: &MovementRequest) -> Result<(), MovementError> where S: MovementSystemExternal<Handle> {
        let creep = external.get_creep(entity)?;

        const REUSE_PATH_LENGTH: u32 = 10;

        let move_options = MoveToOptions::new()
            .range(request.range())
            .reuse_path(REUSE_PATH_LENGTH);

        match creep.move_to_with_options(&request.destination(), move_options) {
            ReturnCode::Ok => return Ok(()),
            err => return Err(format!("Move error: {:?}", err)),
        }
    }

    fn process_request<S>(external: &mut S, cost_matrix_system: &mut CostMatrixSystem, entity: Handle, request: &MovementRequest) -> Result<(), MovementError> where S: MovementSystemExternal<Handle> {
        let creep = external.get_creep(entity)?;

        const REUSE_PATH_LENGTH: u32 = 10;

        let move_options = MoveToOptions::new()
            .range(request.range())
            .reuse_path(REUSE_PATH_LENGTH)
            .no_path_finding(true);

        match creep.move_to_with_options(&request.destination(), move_options) {
            ReturnCode::Ok => return Ok(()),
            ReturnCode::NotFound => {},
            err => return Err(format!("Move error: {:?}", err)),
        }

        let creep_pos = creep.pos();
        let creep_room_name = creep_pos.room_name();

        let room_path = game::map::find_route_with_callback(
            creep_room_name, 
            request.destination().room_name(),
            |to_room_name, from_room_name| external.get_room_weight(from_room_name, to_room_name, creep_room_name, request.room_options()).unwrap_or(f64::INFINITY)
        ).map_err(|e| format!("Could not find path between rooms: {:?}", e))?;

        let room_names: HashSet<_> = room_path
            .iter()
            .map(|step| step.room)
            .collect();

        //TODO: Expose pathing configuration.
        let configration = CostMatrixConfiguration {
            structures: true,
            friendly_creeps: true,
            hostile_creeps: true
        };

        let move_options = MoveToOptions::new()
            .range(request.range())
            .reuse_path(REUSE_PATH_LENGTH)
            .cost_callback(|room_name: RoomName, mut cost_matrix: CostMatrix| -> MultiRoomCostResult {
                if room_names.contains(&room_name) {
                    match cost_matrix_system.apply_cost_matrix(room_name, &mut cost_matrix, &configration) {
                        Ok(()) => cost_matrix.into(),
                        Err(_err) => MultiRoomCostResult::Impassable
                    }
                } else {
                    MultiRoomCostResult::Impassable
                }
            });

        match creep.move_to_with_options(&request.destination(), move_options) {
            ReturnCode::Ok => Ok(()),
            //TODO: Replace with own pathfinding.
            ReturnCode::NoPath => Ok(()),
            //TODO: Don't run move to if tired?
            ReturnCode::Tired => Ok(()),
            err => Err(format!("Move error: {:?}", err)),
        }
    }
}