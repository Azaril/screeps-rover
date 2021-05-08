use super::costmatrixsystem::*;
use super::error::*;
use super::movementrequest::*;
use super::utility::*;
use map::FindRouteOptions;
use screeps::pathfinder::*;
use screeps::*;
use serde::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::hash::Hash;

#[derive(Clone, Serialize, Deserialize)]
pub struct CreepPathData {
    destination: Position,
    range: u32,
    path: Vec<Position>,
    time: u32,
    stuck: u32,
}

#[derive(Clone, Serialize, Deserialize, Default)]
pub struct CreepMovementData {
    path_data: Option<CreepPathData>,
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

    pub fn move_to(&mut self, entity: Handle, destination: Position) -> MovementRequestBuilder {
        self.requests
            .entry(entity)
            .and_modify(|e| *e = MovementRequest::move_to(destination))
            .or_insert_with(|| MovementRequest::move_to(destination))
            .into()
    }
}

pub trait MovementSystemExternal<Handle> {
    fn get_creep(&self, entity: Handle) -> Result<Creep, MovementError>;

    fn get_creep_movement_data(
        &mut self,
        entity: Handle,
    ) -> Result<&mut CreepMovementData, MovementError>;

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

        match creep.move_to_with_options(request.destination, Some(vis_move_options)) {
            ReturnCode::Ok => return Ok(()),
            err => return Err(format!("Move error: {:?}", err)),
        }
    }

    fn process_request<S>(
        &mut self,
        external: &mut S,
        entity: Handle,
        request: MovementRequest,
    ) -> Result<(), MovementError>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep = external.get_creep(entity)?;
        let creep_pos: Position = creep.pos();
        let creep_room_name = creep_pos.room_name();

        //
        // Don't move if parameters are already met.
        //

        if request.destination == creep_pos {
            return Ok(());
        }

        if creep.fatigue() == 0 && !creep.spawning() {
            //
            // Invalidate path if parameters have changed.
            //

            let has_path = {
                let creep_data = external.get_creep_movement_data(entity)?;

                if let Some(path_data) = &creep_data.path_data {
                    let path_valid = path_data.destination == request.destination
                        && path_data.range == request.range
                        && path_data.path.iter().take(2).any(|p| *p == creep_pos);

                    if !path_valid {
                        creep_data.path_data = None
                    }
                }

                creep_data.path_data.is_some()
            };

            //
            // Calculate if creep moved since last tick.
            //

            let move_result = {
                let creep_data = external.get_creep_movement_data(entity)?;
                
                if let Some(path_data) = creep_data.path_data.as_mut() {
                    path_data.time += 1;

                    let path = &mut path_data.path;

                    let current_index = path
                        .iter()
                        .take(2)
                        .enumerate()
                        .find(|(_, p)| **p == creep_pos)
                        .map(|(index, _)| index)
                        .ok_or("Expected current position in path")?;

                    let moved = current_index > 0;

                    path.drain(..current_index);

                    if path.len() == 1 {
                        return Ok(());
                    }

                    if moved {
                        path_data.stuck = 0;
                    } else {
                        path_data.stuck += 1;
                    }

                    Some((path_data.time, path_data.stuck))
                } else {
                    None
                }
            };

            let path_expired = move_result.map(|(path_time, _)| path_time >= self.reuse_path_length).unwrap_or(false);
            let stuck_count = move_result.map(|(_, stuck_count)| stuck_count).unwrap_or(0);

            //
            // Generate path if required.
            //

            let new_data = if !has_path || path_expired || stuck_count > 1 {
                let try_unstuck = stuck_count > 1 && stuck_count % 2 == 0;
                let path_points = self.generate_path(external, &request, &creep, try_unstuck)?;

                Some(CreepPathData {
                    destination: request.destination,
                    range: request.range,
                    path: path_points,
                    time: 0,
                    stuck: 0,
                })
            } else {
                None
            };

            //
            // Path is generated at this point - run movement logic.
            //

            let creep_data = external.get_creep_movement_data(entity)?;

            if new_data.is_some() {
                creep_data.path_data = new_data;
            }

            let path_data = creep_data.path_data.as_mut().ok_or("Expected path data")?;
            let path = &mut path_data.path;

            let next_pos = path.get(1).cloned().ok_or("Expected destination step")?;

            let direction = creep_pos
                .get_direction_to(next_pos)
                .ok_or("Expected movement direction")?;

            match creep.move_direction(direction) {
                ReturnCode::Ok => Ok(()),
                err => Err(format!("Movement error: {:?}", err)),
            }?;
        }

        {
            let creep_data = external.get_creep_movement_data(entity)?;
            let path_data = creep_data.path_data.as_mut().ok_or("Expected path data")?;
            let path = &mut path_data.path;

            //
            // Visualize
            //

            let visualization = request
                .visualization
                .or_else(|| self.default_visualization_style.clone());

            if let Some(visualization) = visualization {
                let visual = RoomVisual::new(Some(creep_room_name));

                let points = path
                    .iter()
                    .take_while(|p| p.room_name() == creep_room_name)
                    .map(|p| (p.x().u8() as f32, p.y().u8() as f32))
                    .collect::<Vec<_>>();

                visual.poly(points, Some(visualization));
            }
        }

        Ok(())
    }

    fn generate_path<S>(
        &mut self,
        external: &mut S,
        request: &MovementRequest,
        creep: &Creep,
        is_stuck: bool
    ) -> Result<Vec<Position>, MovementError>
    where
        S: MovementSystemExternal<Handle>,
    {
        let creep_pos: Position = creep.pos();
        let creep_room_name = creep_pos.room_name();

        let room_options = request.room_options.unwrap_or_default();

        let destination_room = request.destination.room_name();

        let options = FindRouteOptions::new()
            .room_callback(|to_room_name, from_room_name| {
                external
                    .get_room_cost(from_room_name, to_room_name, &room_options)
                    .unwrap_or(f64::INFINITY)
            });

        let room_path = game::map::find_route(creep_room_name, request.destination.room_name(), Some(options)).map_err(|e| format!("Could not find path between rooms: {:?}", e))?;

        let room_names: HashSet<_> = room_path
            .iter()
            .map(|step| step.room)
            .chain(std::iter::once(creep_room_name))
            .chain(std::iter::once(destination_room))
            .collect();

        let mut cost_matrix_options = request.cost_matrix_options.unwrap_or_default();

        if is_stuck {
            cost_matrix_options.friendly_creeps = true;
        }

        let cost_matrix_system = &mut self.cost_matrix_system;

        let max_ops = room_names.len() as u32 * 2000;

        let search_options = SearchOptions::new()
            .max_ops(max_ops)
            .plain_cost(cost_matrix_options.plains_cost)
            .swamp_cost(cost_matrix_options.swamp_cost)
            .room_callback(|room_name: RoomName| -> MultiRoomCostResult {
                if room_names.contains(&room_name) {
                    let mut cost_matrix = CostMatrix::new();

                    match cost_matrix_system.apply_cost_matrix(
                        room_name,
                        &mut cost_matrix,
                        &cost_matrix_options,
                    ) {
                        Ok(()) => {
                            MultiRoomCostResult::CostMatrix(cost_matrix)
                        },
                        Err(_err) => {
                            //TODO: Surface error?
                            MultiRoomCostResult::Impassable
                        }
                    }
                } else {
                    MultiRoomCostResult::Impassable
                }
            });

        let search_result = pathfinder::search(
            creep_pos,
            request.destination,
            request.range,
            Some(search_options),
        );

        if search_result.incomplete() {
            //TODO: Increment stuck, handle stuck? Increase number of ops?
            return Err("Unable to generate path".to_owned());
        }

        let mut path_points = search_result.path();

        path_points.insert(0, creep_pos);

        Ok(path_points)
    }
}
