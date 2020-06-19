use super::costmatrixsystem::*;
use super::error::*;
use super::movementrequest::*;
use super::utility::*;
use screeps::pathfinder::*;
use screeps::*;
use serde::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::hash::Hash;

#[derive(Serialize, Deserialize)]
pub struct CachedMovementData {
    destination: RoomPosition,
    range: u32,
}

#[derive(Default)]
pub struct MovementData<Handle>
where
    Handle: Hash + Eq,
{
    requests: HashMap<Handle, MovementRequest>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<Handle> MovementData<Handle>
where
    Handle: Hash + Eq,
{
    pub fn new() -> MovementData<Handle> {
        MovementData {
            requests: HashMap::new(),
        }
    }

    pub fn move_to(&mut self, entity: Handle, destination: RoomPosition) -> MovementRequestBuilder {
        self.requests
            .entry(entity)
            .and_modify(|e| *e = MovementRequest::move_to(destination))
            .or_insert_with(|| MovementRequest::move_to(destination))
            .into()
    }
}

pub trait MovementSystemExternal<Handle> {
    fn get_creep(&self, entity: Handle) -> Result<Creep, MovementError>;

    fn get_room_cost(
        &self,
        from_room_name: RoomName,
        to_room_name: RoomName,
        _room_options: &RoomOptions,
    ) -> Option<f64> {
        if !can_traverse_between_rooms(from_room_name, to_room_name) {
            return Some(f64::INFINITY);
        }

        Some(1.0)
    }
}

pub struct MovementSystem<'a, Handle> {
    cost_matrix_system: &'a mut CostMatrixSystem,
    default_visualization_style: Option<PolyStyle>,
    reuse_path_length: u32,
    phantom: std::marker::PhantomData<Handle>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a, Handle> MovementSystem<'a, Handle>
where
    Handle: Hash + Eq + Copy,
{
    pub fn new(cost_matrix_system: &'a mut CostMatrixSystem) -> Self {
        Self {
            cost_matrix_system,
            default_visualization_style: None,
            reuse_path_length: 5,
            phantom: std::marker::PhantomData,
        }
    }

    pub fn set_default_visualization_style(&mut self, style: PolyStyle) {
        self.default_visualization_style = Some(style);
    }

    pub fn set_reuse_path_length(&mut self, length: u32) {
        self.reuse_path_length = length;
    }

    pub fn process_inbuilt<S>(&mut self, external: &mut S, data: MovementData<Handle>)
    where
        S: MovementSystemExternal<Handle>,
    {
        for (entity, request) in data.requests.into_iter() {
            match self.process_request_inbuilt(external, entity, request) {
                Ok(()) => {}
                Err(_err) => {}
            }
        }
    }

    pub fn process<S>(&mut self, external: &mut S, data: MovementData<Handle>)
    where
        S: MovementSystemExternal<Handle>,
    {
        for (entity, request) in data.requests.into_iter() {
            match self.process_request(external, entity, request) {
                Ok(()) => {}
                Err(_err) => {}
            }
        }
    }

    fn process_request_inbuilt<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        mut request: MovementRequest,
    ) -> Result<(), MovementError>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep = external.get_creep(entity)?;

        let move_options = MoveToOptions::new()
            .range(request.range)
            .reuse_path(self.reuse_path_length);

        let vis_move_options = if let Some(vis) = request.visualization.take() {
            move_options.visualize_path_style(vis)
        } else if let Some(vis) = self.default_visualization_style.clone() {
            move_options.visualize_path_style(vis)
        } else {
            move_options
        };

        match creep.move_to_with_options(&request.destination, vis_move_options) {
            ReturnCode::Ok => return Ok(()),
            err => return Err(format!("Move error: {:?}", err)),
        }
    }

    fn process_request<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        mut request: MovementRequest,
    ) -> Result<(), MovementError>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep = external.get_creep(entity)?;

        let move_options = MoveToOptions::new()
            .range(request.range)
            .reuse_path(self.reuse_path_length)
            .no_path_finding(true);

        let vis_move_options = if let Some(vis) = request.visualization.clone().take() {
            move_options.visualize_path_style(vis)
        } else if let Some(vis) = self.default_visualization_style.clone() {
            move_options.visualize_path_style(vis)
        } else {
            move_options
        };

        match creep.move_to_with_options(&request.destination, vis_move_options) {
            ReturnCode::Ok => return Ok(()),
            ReturnCode::NotFound => {}
            err => return Err(format!("Move error: {:?}", err)),
        }

        let creep_pos = creep.pos();
        let creep_room_name = creep_pos.room_name();
        let room_options = request.room_options.take().unwrap_or_default();

        let destination_room = request.destination.room_name();

        let room_path = game::map::find_route_with_callback(
            creep_room_name,
            request.destination.room_name(),
            |to_room_name, from_room_name| {
                if to_room_name == destination_room {
                    0.0
                } else {
                    external
                        .get_room_cost(from_room_name, to_room_name, &room_options)
                        .unwrap_or(f64::INFINITY)
                }
            },
        )
        .map_err(|e| format!("Could not find path between rooms: {:?}", e))?;

        let room_names: HashSet<_> = room_path
            .iter()
            .map(|step| step.room)
            .chain(std::iter::once(creep_room_name))
            .chain(std::iter::once(destination_room))
            .collect();

        //TODO: Expose pathing configuration.
        let configration = CostMatrixConfiguration {
            structures: true,
            friendly_creeps: true,
            hostile_creeps: true,
        };

        let cost_matrix_system = &mut self.cost_matrix_system;

        let max_ops = room_names.len() as u32 * 2000;

        let move_options = MoveToOptions::new()
            .range(request.range)
            .reuse_path(self.reuse_path_length)
            .max_ops(max_ops)
            .cost_callback(
                |room_name: RoomName, mut cost_matrix: CostMatrix| -> MultiRoomCostResult {
                    if room_names.contains(&room_name) {
                        match cost_matrix_system.apply_cost_matrix(
                            room_name,
                            &mut cost_matrix,
                            &configration,
                        ) {
                            Ok(()) => cost_matrix.into(),
                            Err(_err) => MultiRoomCostResult::Impassable
                        }
                    } else {
                        MultiRoomCostResult::Impassable
                    }
                },
            );

        let vis_move_options = if let Some(vis) = request.visualization.clone().take() {
            move_options.visualize_path_style(vis)
        } else if let Some(vis) = self.default_visualization_style.clone() {
            move_options.visualize_path_style(vis)
        } else {
            move_options
        };

        match creep.move_to_with_options(&request.destination, vis_move_options) {
            ReturnCode::Ok => Ok(()),
            //TODO: Replace with own pathfinding.
            ReturnCode::NoPath => Ok(()),
            //TODO: Don't run move to if tired?
            ReturnCode::Tired => Ok(()),
            err => Err(format!("Move error: {:?}", err)),
        }
    }
}
