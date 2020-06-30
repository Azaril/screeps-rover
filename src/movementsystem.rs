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
pub struct CreepPathData {
    destination: RoomPosition,
    range: u32,
    path: Vec<Position>,
    time: u32,
    stuck: u32,
}

#[derive(Serialize, Deserialize, Default)]
pub struct CreepMovementData {
    path_data: Option<CreepPathData>
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

    fn get_creep_movement_data(&mut self, entity: Handle) -> Result<&mut CreepMovementData, MovementError>;

    fn get_room_cost(
        &self,
        from_room_name: RoomName,
        to_room_name: RoomName,
        _room_options: &RoomOptions,
    ) -> Option<f64> {
        if !can_traverse_between_rooms(from_room_name, to_room_name) {
            return None;
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
                //TODO: Do something sensible with this error.
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
        let creep_pos = creep.pos();

        if creep.fatigue() > 0 {
            return Ok(());
        }

        if creep.spawning() {
            return Ok(());
        }

        //
        // Invalidate path if parameters have changed.
        //

        let generate_path = {
            let creep_data = external.get_creep_movement_data(entity)?;

            let path_invalid = if let Some(path_data) = &creep_data.path_data {
                if path_data.destination != request.destination || path_data.range != request.range || path_data.time > self.reuse_path_length {

                    //
                    // Ensure that creep is on start of path (didn't move) or on the next path point (did move).
                    //

                    path_data
                        .path
                        .iter()
                        .take(2)
                        .any(|p| *p == creep_pos)
                } else {
                    false
                }
            } else {
                true
            };

            if path_invalid {
                creep_data.path_data = None;
            }

            path_invalid
        };

        //
        // Don't move if parameters are already met.
        //

        if request.destination == creep_pos {
            return Ok(());
        }
        
        let creep_room_name = creep_pos.room_name();        

        if generate_path {
            let room_options = request.room_options.take().unwrap_or_default();
    
            let destination_room = request.destination.room_name();
    
            let room_path = game::map::find_route_with_callback(
                creep_room_name,
                request.destination.room_name(),
                |to_room_name, from_room_name| {
                        external
                            .get_room_cost(from_room_name, to_room_name, &room_options)
                            .unwrap_or(f64::INFINITY)
                },
            )
            .map_err(|e| format!("Could not find path between rooms: {:?}", e))?;
    
            let room_names: HashSet<_> = room_path
                .iter()
                .map(|step| step.room)
                .chain(std::iter::once(creep_room_name))
                .chain(std::iter::once(destination_room))
                .collect();
    
            let cost_matrix_options = request.cost_matrix_options.unwrap_or_default();
    
            let cost_matrix_system = &mut self.cost_matrix_system;
    
            let max_ops = room_names.len() as u32 * 2000;

            let search_options = SearchOptions::new()
            .max_ops(max_ops)
            .room_callback(|room_name: RoomName| -> MultiRoomCostResult {
                if room_names.contains(&room_name) {
                    let mut cost_matrix = CostMatrix::default();

                    match cost_matrix_system.apply_cost_matrix(
                        room_name,
                        &mut cost_matrix,
                        &cost_matrix_options,
                    ) {
                        Ok(()) => cost_matrix.into(),
                        Err(_err) => MultiRoomCostResult::Impassable
                    }
                } else {
                    MultiRoomCostResult::Impassable
                }
            });

            let search_result = pathfinder::search(&creep_pos, &request.destination, request.range, search_options);

            if search_result.incomplete {
                //TODO: Increment stuck, handle stuck?
                return Err("Unable to generate path".to_owned());
            }

            let path_points = search_result.load_local_path();

            let creep_data = external.get_creep_movement_data(entity)?;

            creep_data.path_data = Some(CreepPathData {
                destination: request.destination,
                range: request.range,
                path: path_points,
                time: 0,
                stuck: 0
            });
        }

        //
        // Path is generated at this point - run movement logic.
        //

        let creep_data = external.get_creep_movement_data(entity)?;

        let path_data = creep_data.path_data.as_mut().ok_or("Expected path data")?;
        let path = &mut path_data.path;

        //
        // Visualize
        //

        let visualization = request.visualization.or_else(|| self.default_visualization_style.clone());

        if let Some(visualization) = visualization {
            let visual = RoomVisual::new(Some(creep_room_name));

            let points = path
                .iter()
                .filter(|p| p.room_name() == creep_room_name)
                .map(|p| (p.x() as f32, p.y() as f32))
                .collect();

            visual.poly(points, Some(visualization));
        }

        //
        // Move!
        //

        let current_index = path
            .iter()
            .take(2)
            .enumerate()
            .find(|(_, p)| **p == creep_pos)
            .map(|(index, _)| index);

        let next_index = current_index.map(|i| i + 1).unwrap_or(0);

        let next_pos = path.get(next_index).cloned().ok_or("Expected path")?;

        if let Some(current_index) = current_index {
            path.drain(..=current_index);
        }

        //TODO: This should go at the start of the function once the path has been validated as good to prevent losing state on repath.
        if path_data.time > 0 && next_index == 0 {
            log::info!("Stuck!");

            path_data.stuck += 1;
        }

        path_data.time += 1;

        //TODO: This direction is reversed due to a bug in screeps-game-api which reverses the direction calculation.
        let direction = next_pos.get_direction_to(&creep_pos).ok_or("Expected movement direction")?;

        match creep.move_direction(direction) {
            ReturnCode::Ok => Ok(()),
            err => Err(format!("Movement error: {:?}", err))
        }
    }
}
