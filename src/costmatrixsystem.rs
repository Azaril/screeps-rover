use super::costmatrix::*;
use super::traits::*;
use screeps::local::*;
use serde::*;
use std::collections::HashMap;

#[derive(Serialize, Deserialize)]
pub struct CostMatrixTypeCache<T> {
    pub(crate) data: T,
}

#[derive(Serialize, Deserialize)]
pub struct StuctureCostMatrixCache {
    pub roads: LinearCostMatrix,
    pub other: LinearCostMatrix,
}

#[derive(Serialize, Deserialize)]
pub struct ConstructionSiteCostMatrixCache {
    pub blocked_construction_sites: LinearCostMatrix,
    pub friendly_inactive_construction_sites: LinearCostMatrix,
    pub friendly_active_construction_sites: LinearCostMatrix,
    pub hostile_inactive_construction_sites: LinearCostMatrix,
    pub hostile_active_construction_sites: LinearCostMatrix,
}

#[derive(Serialize, Deserialize)]
pub struct CreepCostMatrixCache {
    pub friendly_creeps: LinearCostMatrix,
    pub hostile_creeps: LinearCostMatrix,
    pub source_keeper_agro: LinearCostMatrix,
}

#[derive(Serialize, Deserialize)]
pub struct CostMatrixRoomEntry {
    pub structures: Option<CostMatrixTypeCache<StuctureCostMatrixCache>>,
    #[serde(skip)]
    pub construction_sites: Option<CostMatrixTypeCache<ConstructionSiteCostMatrixCache>>,
    #[serde(skip)]
    pub creeps: Option<CostMatrixTypeCache<CreepCostMatrixCache>>,
}

impl Default for CostMatrixRoomEntry {
    fn default() -> Self {
        Self::new()
    }
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

#[derive(Default, Serialize, Deserialize)]
pub struct CostMatrixCache {
    rooms: HashMap<RoomName, CostMatrixRoomEntry>,
}

impl CostMatrixCache {
    /// Clear ephemeral per-tick data (structures, construction sites, creeps) from all
    /// rooms. Call this at the start of each tick so stale data is not reused.
    //TODO: Need to add cache eviction policy instead to save computation.
    pub fn clear_ephemeral(&mut self) {
        for entry in self.rooms.values_mut() {
            entry.structures = None;
            entry.construction_sites = None;
            entry.creeps = None;
        }
    }
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

pub struct CostMatrixSystem<'a> {
    cache: &'a mut CostMatrixCache,
    data_source: Box<dyn CostMatrixDataSource>,
}

impl<'a> CostMatrixSystem<'a> {
    pub fn new(
        cache: &'a mut CostMatrixCache,
        data_source: Box<dyn CostMatrixDataSource>,
    ) -> CostMatrixSystem<'a> {
        CostMatrixSystem { cache, data_source }
    }

    /// Build a `LocalCostMatrix` (Rust-native) for a room with the given options.
    pub fn build_local_cost_matrix(
        &mut self,
        room_name: RoomName,
        options: &CostMatrixOptions,
    ) -> Result<LocalCostMatrix, String> {
        // Ensure the cache has fresh data from the data source.
        self.refresh_room(room_name);

        self.cache.build_local_cost_matrix(room_name, options)
    }

    /// Refresh cached data for a room from the data source.
    fn refresh_room(&mut self, room_name: RoomName) {
        // Check what's missing first, then fetch from data source, then insert.
        // This avoids holding a mutable borrow on the cache while calling the data source.
        let entry = self.cache.rooms.entry(room_name).or_default();
        let needs_structures = entry.structures.is_none();
        let needs_construction_sites = entry.construction_sites.is_none();
        let needs_creeps = entry.creeps.is_none();

        // Fetch from data source (borrows self.data_source immutably).
        let structures = if needs_structures {
            self.data_source.get_structure_costs(room_name)
        } else {
            None
        };
        let construction_sites = if needs_construction_sites {
            self.data_source.get_construction_site_costs(room_name)
        } else {
            None
        };
        let creeps = if needs_creeps {
            self.data_source.get_creep_costs(room_name)
        } else {
            None
        };

        // Now insert into cache.
        let entry = self.cache.rooms.entry(room_name).or_default();
        if let Some(data) = structures {
            entry.structures = Some(CostMatrixTypeCache { data });
        }
        if let Some(data) = construction_sites {
            entry.construction_sites = Some(CostMatrixTypeCache { data });
        }
        if let Some(data) = creeps {
            entry.creeps = Some(CostMatrixTypeCache { data });
        }
    }
}

impl CostMatrixCache {
    /// Build a `LocalCostMatrix` (Rust-native `[u8; 2500]`) for a room.
    pub fn build_local_cost_matrix(
        &mut self,
        room_name: RoomName,
        options: &CostMatrixOptions,
    ) -> Result<LocalCostMatrix, String> {
        let mut lcm = LocalCostMatrix::new();
        let entry = self.rooms.entry(room_name).or_default();

        if options.structures {
            if let Some(ref structures) = entry.structures {
                structures
                    .data
                    .roads
                    .apply_to_transformed(&mut lcm, |_| options.road_cost);

                structures.data.other.apply_to(&mut lcm);
            }
        }

        if options.construction_sites {
            if let Some(ref construction_sites) = entry.construction_sites {
                construction_sites
                    .data
                    .blocked_construction_sites
                    .apply_to(&mut lcm);

                let applicators = [
                    (
                        options.friendly_inactive_construction_site_cost,
                        &construction_sites.data.friendly_inactive_construction_sites,
                    ),
                    (
                        options.friendly_active_construction_site_cost,
                        &construction_sites.data.friendly_active_construction_sites,
                    ),
                    (
                        options.hostile_inactive_construction_site_cost,
                        &construction_sites.data.hostile_inactive_construction_sites,
                    ),
                    (
                        options.hostile_active_construction_site_cost,
                        &construction_sites.data.hostile_active_construction_sites,
                    ),
                ];

                for (cost, source_matrix) in &applicators {
                    if let Some(cost) = cost {
                        source_matrix.apply_to_transformed(&mut lcm, |_| *cost);
                    }
                }
            }
        }

        if options.friendly_creeps || options.hostile_creeps || options.source_keeper_aggro {
            if let Some(ref creeps) = entry.creeps {
                if options.source_keeper_aggro {
                    creeps
                        .data
                        .source_keeper_agro
                        .apply_to_transformed(&mut lcm, |_| options.source_keeper_aggro_cost)
                }

                if options.friendly_creeps {
                    creeps.data.friendly_creeps.apply_to(&mut lcm);
                }

                if options.hostile_creeps {
                    creeps.data.hostile_creeps.apply_to(&mut lcm);
                }
            }
        }

        Ok(lcm)
    }
}
