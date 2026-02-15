use crate::location::*;
use screeps::*;
use serde::*;
use std::collections::HashMap;

pub trait CostMatrixApply {
    fn apply_to<T>(&self, target: &mut T)
    where
        T: CostMatrixSet;

    fn apply_to_transformed<T, TF>(&self, target: &mut T, transformer: TF)
    where
        T: CostMatrixSet,
        TF: Fn(u8) -> u8;

    /// Apply only entries where `filter` returns true for the location.
    fn apply_to_filtered<T, F>(&self, target: &mut T, filter: F)
    where
        T: CostMatrixSet,
        F: Fn(&Location) -> bool;
}

pub trait CostMatrixWrite {
    fn set(&mut self, x: u8, y: u8, val: u8);
}

pub trait CostMatrixRead {
    fn get(&self, x: u8, y: u8) -> u8;
}

#[derive(Serialize, Deserialize)]
pub struct SparseCostMatrix {
    data: HashMap<Location, u8>,
}

impl CostMatrixWrite for SparseCostMatrix {
    fn set(&mut self, x: u8, y: u8, val: u8) {
        self.data
            .insert(Location::from_coords(x as u32, y as u32), val);
    }
}

impl CostMatrixRead for SparseCostMatrix {
    fn get(&self, x: u8, y: u8) -> u8 {
        self.data
            .get(&Location::from_coords(x as u32, y as u32))
            .copied()
            .unwrap_or(0)
    }
}

impl CostMatrixApply for SparseCostMatrix {
    fn apply_to<T>(&self, target: &mut T)
    where
        T: CostMatrixSet,
    {
        for (location, cost) in self.data.iter() {
            target.set_xy(location.to_room_xy(), *cost);
        }
    }

    fn apply_to_transformed<T, TF>(&self, target: &mut T, transformer: TF)
    where
        T: CostMatrixSet,
        TF: Fn(u8) -> u8,
    {
        for (location, cost) in self.data.iter() {
            let new_cost = transformer(*cost);
            target.set_xy(location.to_room_xy(), new_cost);
        }
    }

    fn apply_to_filtered<T, F>(&self, target: &mut T, filter: F)
    where
        T: CostMatrixSet,
        F: Fn(&Location) -> bool,
    {
        for (location, cost) in self.data.iter() {
            if filter(location) {
                target.set_xy(location.to_room_xy(), *cost);
            }
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct LinearCostMatrix {
    data: Vec<(Location, u8)>,
}

impl Default for LinearCostMatrix {
    fn default() -> Self {
        Self::new()
    }
}

impl LinearCostMatrix {
    pub fn new() -> LinearCostMatrix {
        LinearCostMatrix { data: Vec::new() }
    }
}

impl CostMatrixWrite for LinearCostMatrix {
    fn set(&mut self, x: u8, y: u8, val: u8) {
        self.data
            .push((Location::from_coords(x as u32, y as u32), val));
    }
}

impl CostMatrixApply for LinearCostMatrix {
    fn apply_to<T>(&self, target: &mut T)
    where
        T: CostMatrixSet,
    {
        for (location, cost) in self.data.iter() {
            target.set_xy(location.to_room_xy(), *cost);
        }
    }

    fn apply_to_transformed<T, TF>(&self, target: &mut T, transformer: TF)
    where
        T: CostMatrixSet,
        TF: Fn(u8) -> u8,
    {
        for (location, cost) in self.data.iter() {
            let new_cost = transformer(*cost);
            target.set_xy(location.to_room_xy(), new_cost);
        }
    }

    fn apply_to_filtered<T, F>(&self, target: &mut T, filter: F)
    where
        T: CostMatrixSet,
        F: Fn(&Location) -> bool,
    {
        for (location, cost) in self.data.iter() {
            if filter(location) {
                target.set_xy(location.to_room_xy(), *cost);
            }
        }
    }
}
