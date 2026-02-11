//! Real Screeps game API implementations of the abstraction traits.
//! Only compiled when the `screeps` feature is enabled.

use screeps::game::map::FindRouteOptions;
use screeps::pathfinder;
use screeps::pathfinder::MultiRoomCostResult;
use screeps::*;

use super::constants::*;
use super::costmatrix::*;
use super::costmatrixsystem::*;
use super::traits::*;

// --- CreepHandle for screeps::Creep ---

impl CreepHandle for Creep {
    fn pos(&self) -> Position {
        HasPosition::pos(self)
    }

    fn fatigue(&self) -> u32 {
        Creep::fatigue(self)
    }

    fn spawning(&self) -> bool {
        Creep::spawning(self)
    }

    fn move_direction(&self, dir: Direction) -> Result<(), String> {
        Creep::move_direction(self, dir).map_err(|e| format!("{:?}", e))
    }

    fn pull(&self, other: &Self) -> Result<(), String> {
        Creep::pull(self, other).map_err(|e| format!("{:?}", e))
    }

    fn move_pulled_by(&self, other: &Self) -> Result<(), String> {
        Creep::move_pulled_by(self, other).map_err(|e| format!("{:?}", e))
    }
}

// --- ScreepsPathfinder ---

/// Pathfinding implementation that delegates to the Screeps `PathFinder` API.
pub struct ScreepsPathfinder;

impl PathfindingProvider for ScreepsPathfinder {
    fn search(
        &mut self,
        origin: Position,
        goal: Position,
        range: u32,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        swamp_cost: u8,
    ) -> PathfindingResult {
        let search_options =
            pathfinder::SearchOptions::new(|room_name: RoomName| -> MultiRoomCostResult {
                match room_callback(room_name) {
                    Some(lcm) => {
                        let js_cm: CostMatrix = lcm.into();
                        MultiRoomCostResult::CostMatrix(js_cm)
                    }
                    None => MultiRoomCostResult::Impassable,
                }
            })
            .max_ops(max_ops)
            .plain_cost(plain_cost)
            .swamp_cost(swamp_cost);

        let result = pathfinder::search(origin, goal, range, Some(search_options));

        PathfindingResult {
            path: result.path(),
            incomplete: result.incomplete(),
        }
    }

    fn search_many(
        &mut self,
        origin: Position,
        goals: &[(Position, u32)],
        flee: bool,
        room_callback: &mut dyn FnMut(RoomName) -> Option<LocalCostMatrix>,
        max_ops: u32,
        plain_cost: u8,
        swamp_cost: u8,
    ) -> PathfindingResult {
        let search_goals: Vec<pathfinder::SearchGoal> = goals
            .iter()
            .map(|(pos, range)| pathfinder::SearchGoal::new(*pos, *range))
            .collect();

        let search_options =
            pathfinder::SearchOptions::new(|room_name: RoomName| -> MultiRoomCostResult {
                match room_callback(room_name) {
                    Some(lcm) => {
                        let js_cm: CostMatrix = lcm.into();
                        MultiRoomCostResult::CostMatrix(js_cm)
                    }
                    None => MultiRoomCostResult::Impassable,
                }
            })
            .flee(flee)
            .max_ops(max_ops)
            .plain_cost(plain_cost)
            .swamp_cost(swamp_cost);

        let result =
            pathfinder::search_many(origin, search_goals.into_iter(), Some(search_options));

        PathfindingResult {
            path: result.path(),
            incomplete: result.incomplete(),
        }
    }

    fn find_route(
        &self,
        from: RoomName,
        to: RoomName,
        room_callback: &dyn Fn(RoomName, RoomName) -> f64,
    ) -> Result<Vec<RouteStep>, String> {
        let result = game::map::find_route(
            from,
            to,
            Some(
                FindRouteOptions::new()
                    .room_callback(room_callback),
            ),
        )
        .map_err(|e| format!("find_route error: {:?}", e))?;

        Ok(result
            .iter()
            .map(|step| RouteStep { room: step.room })
            .collect())
    }

    fn get_room_linear_distance(&self, from: RoomName, to: RoomName) -> u32 {
        game::map::get_room_linear_distance(from, to, false)
    }

    fn is_tile_walkable(&self, pos: Position) -> bool {
        let x = pos.x().u8();
        let y = pos.y().u8();
        if x == 0 || x == 49 || y == 0 || y == 49 {
            return true;
        }
        if let Some(terrain) = game::map::get_room_terrain(pos.room_name()) {
            let t = terrain.get(x, y);
            if t == Terrain::Wall {
                return false;
            }
        }
        true
    }
}

// --- ScreepsCostMatrixDataSource ---

/// Cost matrix data source that reads live game state via the Screeps API.
pub struct ScreepsCostMatrixDataSource;

impl CostMatrixDataSource for ScreepsCostMatrixDataSource {
    fn get_structure_costs(&self, room_name: RoomName) -> Option<StuctureCostMatrixCache> {
        let room = game::rooms().get(room_name)?;

        let mut roads = LinearCostMatrix::new();
        let mut other = LinearCostMatrix::new();

        let structures = room.find(find::STRUCTURES, None);

        for structure in structures.iter() {
            let res = match structure {
                StructureObject::StructureRampart(r) => {
                    if r.my() || r.is_public() {
                        None
                    } else {
                        Some((u8::MAX, &mut other))
                    }
                }
                StructureObject::StructureRoad(_) => Some((1, &mut roads)),
                StructureObject::StructureContainer(_) => Some((2, &mut other)),
                _ => Some((u8::MAX, &mut other)),
            };

            if let Some((cost, matrix)) = res {
                let pos = structure.pos();
                matrix.set(pos.x().u8(), pos.y().u8(), cost);
            }
        }

        Some(StuctureCostMatrixCache { roads, other })
    }

    fn get_construction_site_costs(
        &self,
        room_name: RoomName,
    ) -> Option<ConstructionSiteCostMatrixCache> {
        let room = game::rooms().get(room_name)?;

        let mut blocked_construction_sites = LinearCostMatrix::new();
        let mut friendly_inactive_construction_sites = LinearCostMatrix::new();
        let mut friendly_active_construction_sites = LinearCostMatrix::new();
        let mut hostile_inactive_construction_sites = LinearCostMatrix::new();
        let mut hostile_active_construction_sites = LinearCostMatrix::new();

        for construction_site in room.find(find::MY_CONSTRUCTION_SITES, None).iter() {
            let pos = construction_site.pos();
            let walkable = matches!(
                construction_site.structure_type(),
                StructureType::Container | StructureType::Road | StructureType::Rampart
            );
            if !walkable {
                blocked_construction_sites.set(pos.x().u8(), pos.y().u8(), u8::MAX);
            } else if construction_site.progress() > 0 {
                friendly_active_construction_sites.set(pos.x().u8(), pos.y().u8(), 1);
            } else {
                friendly_inactive_construction_sites.set(pos.x().u8(), pos.y().u8(), 1);
            }
        }

        let safe_mode = room
            .controller()
            .and_then(|c| c.safe_mode())
            .unwrap_or(0)
            > 0;

        for construction_site in room.find(find::HOSTILE_CONSTRUCTION_SITES, None).iter() {
            let pos = construction_site.pos();
            let walkable = !safe_mode;
            if !walkable {
                blocked_construction_sites.set(pos.x().u8(), pos.y().u8(), u8::MAX);
            } else if construction_site.progress() > 0 {
                hostile_active_construction_sites.set(pos.x().u8(), pos.y().u8(), 1);
            } else {
                hostile_inactive_construction_sites.set(pos.x().u8(), pos.y().u8(), 1);
            }
        }

        Some(ConstructionSiteCostMatrixCache {
            blocked_construction_sites,
            friendly_inactive_construction_sites,
            friendly_active_construction_sites,
            hostile_inactive_construction_sites,
            hostile_active_construction_sites,
        })
    }

    fn get_creep_costs(&self, room_name: RoomName) -> Option<CreepCostMatrixCache> {
        let room = game::rooms().get(room_name)?;

        let mut friendly_creeps = LinearCostMatrix::new();

        for creep in room.find(find::MY_CREEPS, None).iter() {
            let pos = HasPosition::pos(creep);
            friendly_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);
        }

        for power_creep in room.find(find::MY_POWER_CREEPS, None).iter() {
            let pos = HasPosition::pos(power_creep);
            friendly_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);
        }

        let mut hostile_creeps = LinearCostMatrix::new();
        let terrain = game::map::get_room_terrain(room_name)?;
        let mut source_keeper_agro = LinearCostMatrix::new();

        for creep in room.find(find::HOSTILE_CREEPS, None).iter() {
            let pos = HasPosition::pos(creep);
            hostile_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);

            if creep.owner().username() == SOURCE_KEEPER_NAME {
                let sk_pos = HasPosition::pos(creep);
                let x = sk_pos.x().u8() as i32;
                let y = sk_pos.y().u8() as i32;

                for x_offset in
                    x - SOURCE_KEEPER_AGRO_RADIUS as i32..=x + SOURCE_KEEPER_AGRO_RADIUS as i32
                {
                    for y_offset in
                        y - SOURCE_KEEPER_AGRO_RADIUS as i32..=y + SOURCE_KEEPER_AGRO_RADIUS as i32
                    {
                        if (0..50).contains(&x_offset) && (0..50).contains(&y_offset) {
                            let tile_terrain = terrain.get(x_offset as u8, y_offset as u8);
                            let is_wall = tile_terrain == Terrain::Wall;
                            if !is_wall {
                                source_keeper_agro.set(x_offset as u8, y_offset as u8, 1);
                            }
                        }
                    }
                }
            }
        }

        for power_creep in room.find(find::HOSTILE_POWER_CREEPS, None).iter() {
            let pos = HasPosition::pos(power_creep);
            hostile_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);
        }

        Some(CreepCostMatrixCache {
            friendly_creeps,
            hostile_creeps,
            source_keeper_agro,
        })
    }
}

// --- ScreepsMovementVisualizer ---

/// Default movement visualizer that renders directly to the Screeps
/// `RoomVisual` API. Suitable for quick integration; advanced users can
/// implement `MovementVisualizer` themselves for custom rendering pipelines.
pub struct ScreepsMovementVisualizer;

impl MovementVisualizer for ScreepsMovementVisualizer {
    fn visualize_path(&mut self, creep_pos: Position, path: &[Position]) {
        let room = creep_pos.room_name();
        let visual = RoomVisual::new(Some(room));
        let points: Vec<(f32, f32)> = path
            .iter()
            .map(|p| (p.x().u8() as f32, p.y().u8() as f32))
            .collect();
        let style = screeps::PolyStyle::default()
            .stroke("blue")
            .stroke_width(0.2)
            .opacity(0.5);
        visual.poly(points, Some(style));
    }

    fn visualize_anchor(&mut self, creep_pos: Position, anchor_pos: Position) {
        let room = creep_pos.room_name();
        let visual = RoomVisual::new(Some(room));
        let cx = creep_pos.x().u8() as f32;
        let cy = creep_pos.y().u8() as f32;

        let circle_style = screeps::CircleStyle::default()
            .fill("#ff8800")
            .radius(0.15)
            .opacity(0.5)
            .stroke("#ff8800")
            .stroke_width(0.02);
        visual.circle(cx, cy, Some(circle_style));

        let ax = anchor_pos.x().u8() as f32;
        let ay = anchor_pos.y().u8() as f32;
        if (ax - cx).abs() > 0.01 || (ay - cy).abs() > 0.01 {
            let line_style = screeps::LineStyle::default()
                .color("#ff8800")
                .opacity(0.25);
            visual.line((cx, cy), (ax, ay), Some(line_style));
        }
    }

    fn visualize_immovable(&mut self, creep_pos: Position) {
        let room = creep_pos.room_name();
        let visual = RoomVisual::new(Some(room));
        let cx = creep_pos.x().u8() as f32;
        let cy = creep_pos.y().u8() as f32;
        let d = 0.15;
        let style = screeps::LineStyle::default()
            .color("#ff4444")
            .opacity(0.6);
        visual.line((cx - d, cy - d), (cx + d, cy + d), Some(style.clone()));
        visual.line((cx - d, cy + d), (cx + d, cy - d), Some(style));
    }

    fn visualize_stuck(&mut self, creep_pos: Position, ticks: u16) {
        let room = creep_pos.room_name();
        let visual = RoomVisual::new(Some(room));
        let cx = creep_pos.x().u8() as f32;
        let cy = creep_pos.y().u8() as f32;

        let circle_style = screeps::CircleStyle::default()
            .fill("#ffcc00")
            .radius(0.2)
            .opacity(0.6)
            .stroke("#ffcc00")
            .stroke_width(0.03);
        visual.circle(cx, cy, Some(circle_style));

        let text_style = screeps::TextStyle::default()
            .color("#ffcc00")
            .font(0.4)
            .stroke("#000000")
            .stroke_width(0.03);
        visual.text(cx, cy + 0.55, format!("{}", ticks), Some(text_style));
    }

    fn visualize_failed(&mut self, creep_pos: Position) {
        let room = creep_pos.room_name();
        let visual = RoomVisual::new(Some(room));
        let cx = creep_pos.x().u8() as f32;
        let cy = creep_pos.y().u8() as f32;

        let circle_style = screeps::CircleStyle::default()
            .fill("#ff0000")
            .radius(0.2)
            .opacity(0.7)
            .stroke("#ff0000")
            .stroke_width(0.03);
        visual.circle(cx, cy, Some(circle_style));
    }
}

