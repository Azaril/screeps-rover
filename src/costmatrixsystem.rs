use super::costmatrix::*;
use screeps::pathfinder::CostMatrix;
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
pub struct CostMatrixRoomEntry {
    structures: Option<CostMatrixTypeCache<StuctureCostMatrixCache>>,
    #[serde(skip)]
    friendly_creeps: Option<CostMatrixTypeCache<LinearCostMatrix>>,
    #[serde(skip)]
    hostile_creeps: Option<CostMatrixTypeCache<LinearCostMatrix>>,
}

impl CostMatrixRoomEntry {
    pub fn new() -> CostMatrixRoomEntry {
        CostMatrixRoomEntry {
            structures: None,
            friendly_creeps: None,
            hostile_creeps: None,
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
    pub road_cost: u8,
    pub plains_cost: u8,
    pub swamp_cost: u8,
}

impl Default for CostMatrixOptions {
    fn default() -> Self {
        CostMatrixOptions {
            structures: true,
            friendly_creeps: true,
            hostile_creeps: true,
            road_cost: 1,
            plains_cost: 2,
            swamp_cost: 10,
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
    fn get_room(&mut self, room_name: RoomName) -> CostMatrixRoomAccessor {
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

        if options.friendly_creeps {
            if let Some(friendly_creeps) = room.get_friendly_creeps() {
                friendly_creeps.apply_to(cost_matrix);
            }
        }

        if options.hostile_creeps {
            if let Some(hostile_creeps) = room.get_hostile_creeps() {
                hostile_creeps.apply_to(cost_matrix);
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
            game::time() - data.last_updated > 0 && game::rooms::get(room_name).is_some()
        };
        let filler = move || {
            let room = game::rooms::get(room_name)?;

            let mut roads = LinearCostMatrix::new();
            let mut other = LinearCostMatrix::new();

            let structures = room.find(find::STRUCTURES);

            for structure in structures.iter() {
                let res = match structure {
                    Structure::Rampart(r) => {
                        if r.my() {
                            None
                        } else {
                            Some((u8::MAX, &mut other))
                        }
                    }
                    Structure::Road(_) => Some((1, &mut roads)),
                    Structure::Container(_) => Some((2, &mut other)),
                    _ => Some((u8::MAX, &mut other)),
                };

                if let Some((cost, matrix)) = res {
                    let pos = structure.pos();

                    matrix.set(pos.x() as u8, pos.y() as u8, cost);
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

    pub fn get_friendly_creeps(&mut self) -> Option<&LinearCostMatrix> {
        let expiration = |data: &CostMatrixTypeCache<_>| game::time() - data.last_updated > 0;
        let room_name = self.room_name;
        let filler = move || {
            let room = game::rooms::get(room_name)?;

            let mut matrix = LinearCostMatrix::new();

            for creep in room.find(find::MY_CREEPS).iter() {
                let pos = creep.pos();

                matrix.set(pos.x() as u8, pos.y() as u8, u8::MAX);
            }

            for power_creep in room.find(find::MY_POWER_CREEPS).iter() {
                let pos = power_creep.pos();

                matrix.set(pos.x() as u8, pos.y() as u8, u8::MAX);
            }

            let entry = CostMatrixTypeCache {
                last_updated: game::time(),
                data: matrix,
            };

            Some(entry)
        };

        self.entry
            .friendly_creeps
            .maybe_access(expiration, filler)
            .get()
            .map(|d| &d.data)
    }

    pub fn get_hostile_creeps(&mut self) -> Option<&LinearCostMatrix> {
        let expiration = |data: &CostMatrixTypeCache<_>| game::time() - data.last_updated > 0;
        let room_name = self.room_name;
        let filler = move || {
            let room = game::rooms::get(room_name)?;

            let mut matrix = LinearCostMatrix::new();

            for creep in room.find(find::HOSTILE_CREEPS).iter() {
                let pos = creep.pos();

                matrix.set(pos.x() as u8, pos.y() as u8, u8::MAX);
            }

            for power_creep in room.find(find::HOSTILE_POWER_CREEPS).iter() {
                let pos = power_creep.pos();

                matrix.set(pos.x() as u8, pos.y() as u8, u8::MAX);
            }

            let entry = CostMatrixTypeCache {
                last_updated: game::time(),
                data: matrix,
            };

            Some(entry)
        };

        self.entry
            .hostile_creeps
            .maybe_access(expiration, filler)
            .get()
            .map(|d| &d.data)
    }
}
