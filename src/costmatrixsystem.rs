use super::costmatrix::*;
use super::constants::*;
use screeps::*;
use screeps_cache::*;
use serde::*;
use std::collections::HashMap;

#[derive(Serialize, Deserialize)]
pub struct CostMatrixTypeCache<T> {
    last_updated: u32,
    data: T,
}

#[derive(Serialize, Deserialize)]
pub struct StuctureCostMatrixCache {
    roads: LinearCostMatrix,
    other: LinearCostMatrix,
}

#[derive(Serialize, Deserialize)]
pub struct ConstructionSiteCostMatrixCache {
    blocked_construction_sites: LinearCostMatrix,
    friendly_inactive_construction_sites: LinearCostMatrix,
    friendly_active_construction_sites: LinearCostMatrix,
    hostile_inactive_construction_sites: LinearCostMatrix,    
    hostile_active_construction_sites: LinearCostMatrix,
}

#[derive(Serialize, Deserialize)]
pub struct CreepCostMatrixCache {
    friendly_creeps: LinearCostMatrix,
    hostile_creeps: LinearCostMatrix,
    source_keeper_agro: LinearCostMatrix,
}

#[derive(Serialize, Deserialize)]
pub struct CostMatrixRoomEntry {
    structures: Option<CostMatrixTypeCache<StuctureCostMatrixCache>>,
    #[serde(skip)]
    construction_sites: Option<CostMatrixTypeCache<ConstructionSiteCostMatrixCache>>,    
    #[serde(skip)]
    creeps: Option<CostMatrixTypeCache<CreepCostMatrixCache>>,
}

impl CostMatrixRoomEntry {
    pub fn new() -> CostMatrixRoomEntry {
        CostMatrixRoomEntry {
            structures: None,
            construction_sites: None,
            creeps: None,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct CostMatrixCache {
    rooms: HashMap<RoomName, CostMatrixRoomEntry>,
}

pub trait CostMatrixStorage {
    fn get_cache(&self, segment: u32) -> Result<CostMatrixCache, String>;

    fn set_cache(&mut self, segment: u32, data: &CostMatrixCache) -> Result<(), String>;
}

#[derive(Copy, Clone)]
pub struct CostMatrixOptions {
    pub structures: bool,
    pub friendly_creeps: bool,
    pub hostile_creeps: bool,
    pub construction_sites: bool,
    pub source_keeper_aggro: bool,
    pub road_cost: u8,
    pub plains_cost: u8,
    pub swamp_cost: u8,
    pub source_keeper_aggro_cost: u8,
    pub friendly_inactive_construction_site_cost: Option<u8>,
    pub friendly_active_construction_site_cost: Option<u8>,
    pub hostile_inactive_construction_site_cost: Option<u8>,    
    pub hostile_active_construction_site_cost: Option<u8>,
}

impl Default for CostMatrixOptions {
    fn default() -> Self {
        CostMatrixOptions {
            structures: true,
            friendly_creeps: false,
            hostile_creeps: true,
            construction_sites: true,
            source_keeper_aggro: true,
            road_cost: 1,
            plains_cost: 2,
            swamp_cost: 10,
            source_keeper_aggro_cost: 50,
            friendly_inactive_construction_site_cost: None,
            friendly_active_construction_site_cost: Some(3),
            hostile_inactive_construction_site_cost: Some(2),
            hostile_active_construction_site_cost: Some(1),
        }
    }
}

pub struct CostMatrixSystem {
    storage: Box<dyn CostMatrixStorage>,
    storage_segment: u32,
    cache: Option<CostMatrixCache>,
}

impl CostMatrixSystem {
    pub fn new(storage: Box<dyn CostMatrixStorage>, storage_segment: u32) -> CostMatrixSystem {
        CostMatrixSystem {
            storage,
            storage_segment,
            cache: None,
        }
    }

    pub fn flush_storage(&mut self) {
        let storage = &mut self.storage;
        let cache = &self.cache;
        let storage_segment = self.storage_segment;

        cache
            .as_ref()
            .map(|c| storage.set_cache(storage_segment, c));
    }

    pub fn apply_cost_matrix(
        &mut self,
        room_name: RoomName,
        cost_matrix: &mut CostMatrix,
        options: &CostMatrixOptions,
    ) -> Result<(), String> {
        let cache = self.get_cache();

        cache.apply_cost_matrix(room_name, cost_matrix, options)
    }

    fn get_cache(&mut self) -> &mut CostMatrixCache {
        let cache = &mut self.cache;
        let storage = &mut self.storage;
        let storage_segment = self.storage_segment;

        cache.get_or_insert_with(|| storage.get_cache(storage_segment).unwrap_or_default())
    }
}

impl Default for CostMatrixCache {
    fn default() -> CostMatrixCache {
        CostMatrixCache {
            rooms: HashMap::new(),
        }
    }
}

impl CostMatrixCache {
    fn get_room(&mut self, room_name: RoomName) -> CostMatrixRoomAccessor<'_> {
        let entry = self
            .rooms
            .entry(room_name)
            .or_insert_with(CostMatrixRoomEntry::new);

        CostMatrixRoomAccessor { room_name, entry }
    }

    pub fn apply_cost_matrix(
        &mut self,
        room_name: RoomName,
        cost_matrix: &mut CostMatrix,
        options: &CostMatrixOptions,
    ) -> Result<(), String> {
        let mut room = self.get_room(room_name);

        if options.structures {
            if let Some(structures) = room.get_structures() {
                structures
                    .roads
                    .apply_to_transformed(cost_matrix, |_| options.road_cost);

                structures.other.apply_to(cost_matrix);
            }
        }

        if options.construction_sites {
            if let Some(construction_sites) = room.get_construction_sites() {
                construction_sites.blocked_construction_sites.apply_to(cost_matrix);

                let applicators = [
                    (options.friendly_inactive_construction_site_cost, &construction_sites.friendly_inactive_construction_sites),
                    (options.friendly_active_construction_site_cost, &construction_sites.friendly_active_construction_sites),
                    (options.hostile_inactive_construction_site_cost, &construction_sites.hostile_inactive_construction_sites),
                    (options.hostile_active_construction_site_cost, &construction_sites.hostile_active_construction_sites),
                ];

                //TODO: Rework API to generate an iterator to batch the full set of cost matrix modifies.
                for (cost, source_matrix) in &applicators {
                    if let Some(cost) = cost {
                        source_matrix.apply_to_transformed(cost_matrix, |_| *cost);
                    }
                }        
            }
        }

        if options.friendly_creeps || options.hostile_creeps || options.source_keeper_aggro {
            if let Some(creeps) = room.get_creeps() {
                if options.source_keeper_aggro {
                    creeps.source_keeper_agro.apply_to_transformed(cost_matrix, |_| options.source_keeper_aggro_cost)
                }

                if options.friendly_creeps {
                    creeps.friendly_creeps.apply_to(cost_matrix);
                }

                if options.hostile_creeps {
                    creeps.hostile_creeps.apply_to(cost_matrix);
                }
            }
        }

        Ok(())
    }
}

pub struct CostMatrixRoomAccessor<'a> {
    room_name: RoomName,
    entry: &'a mut CostMatrixRoomEntry,
}

impl<'a> CostMatrixRoomAccessor<'a> {
    pub fn get_structures(&mut self) -> Option<&StuctureCostMatrixCache> {
        let room_name = self.room_name;

        let expiration = move |data: &CostMatrixTypeCache<_>| {
            game::time() - data.last_updated > 0 && game::rooms().get(room_name).is_some()
        };
        let filler = move || {
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

            let entry = CostMatrixTypeCache {
                last_updated: game::time(),
                data: StuctureCostMatrixCache { roads, other },
            };

            Some(entry)
        };

        self.entry
            .structures
            .maybe_access(expiration, filler)
            .get()
            .map(|d| &d.data)
    }

    pub fn get_construction_sites(&mut self) -> Option<&ConstructionSiteCostMatrixCache> {
        let room_name = self.room_name;
        let expiration = |data: &CostMatrixTypeCache<_>| game::time() - data.last_updated > 0 && game::rooms().get(room_name).is_some();
        let filler = move || {
            let room = game::rooms().get(room_name)?;

            let mut blocked_construction_sites = LinearCostMatrix::new();

            let mut friendly_inactive_construction_sites = LinearCostMatrix::new();
            let mut friendly_active_construction_sites = LinearCostMatrix::new();

            let mut hostile_inactive_construction_sites = LinearCostMatrix::new();
            let mut hostile_active_construction_sites = LinearCostMatrix::new();            

            for construction_site in room.find(find::MY_CONSTRUCTION_SITES, None).iter() {
                let pos = construction_site.pos();

                let walkable = match construction_site.structure_type() {
                    StructureType::Container => true,
                    StructureType::Road => true,
                    StructureType::Rampart => true,
                    _ => false
                };

                if !walkable {
                    blocked_construction_sites.set(pos.x().u8(), pos.y().u8(), u8::MAX);
                } else if construction_site.progress() > 0 {
                    friendly_active_construction_sites.set(pos.x().u8(), pos.y().u8(), 1);
                } else {
                    friendly_inactive_construction_sites.set(pos.x().u8(), pos.y().u8(), 1);
                }
            }

            let safe_mode = room.controller().and_then(|c| c.safe_mode()).unwrap_or(0) > 0;

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

            let entry = CostMatrixTypeCache {
                last_updated: game::time(),
                data: ConstructionSiteCostMatrixCache {
                    blocked_construction_sites,
                    friendly_inactive_construction_sites,
                    friendly_active_construction_sites,
                    hostile_inactive_construction_sites,
                    hostile_active_construction_sites
                },
            };

            Some(entry)
        };

        self.entry
            .construction_sites
            .maybe_access(expiration, filler)
            .get()
            .map(|d| &d.data)
    }

    pub fn get_creeps(&mut self) -> Option<&CreepCostMatrixCache> {
        let room_name = self.room_name;
        let expiration = |data: &CostMatrixTypeCache<_>| game::time() - data.last_updated > 0;
        let filler = move || {
            let room = game::rooms().get(room_name)?;

            let mut friendly_creeps = LinearCostMatrix::new();

            for creep in room.find(find::MY_CREEPS, None).iter() {
                let pos = creep.pos();

                friendly_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);
            }

            for power_creep in room.find(find::MY_POWER_CREEPS, None).iter() {
                let pos = power_creep.pos();

                friendly_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);
            }

            let mut hostile_creeps = LinearCostMatrix::new();

            let terrain = game::map::get_room_terrain(room_name).expect("Expected room terrain");

            let mut source_keeper_agro = LinearCostMatrix::new();            

            for creep in room.find(find::HOSTILE_CREEPS, None).iter() {
                let pos = creep.pos();

                hostile_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);

                if creep.owner().username() == SOURCE_KEEPER_NAME {
                    let pos = creep.pos();

                    let x = pos.x().u8() as i32;
                    let y = pos.y().u8() as i32;

                    //TODO: Add constants for room size? Use FastRoomTerrain?
                    
                    for x_offset in x-SOURCE_KEEPER_AGRO_RADIUS as i32..=x+SOURCE_KEEPER_AGRO_RADIUS as i32 {
                        for y_offset in y-SOURCE_KEEPER_AGRO_RADIUS as i32..=y+SOURCE_KEEPER_AGRO_RADIUS as i32 {
                            if x_offset >= 0 && x_offset < 50 && y_offset >= 0 && y_offset < 50 {
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
                let pos = power_creep.pos();

                hostile_creeps.set(pos.x().u8(), pos.y().u8(), u8::MAX);
            }

            let entry = CostMatrixTypeCache {
                last_updated: game::time(),
                data: CreepCostMatrixCache {
                    friendly_creeps,
                    hostile_creeps,
                    source_keeper_agro
                },
            };

            Some(entry)
        };

        self.entry
            .creeps
            .maybe_access(expiration, filler)
            .get()
            .map(|d| &d.data)
    }
}
