# screeps-rover

A coordinated movement and pathfinding library for [Screeps](https://screeps.com/), written in Rust. screeps-rover manages multi-creep movement with global conflict resolution, path caching, stuck detection, and layered cost matrices — all designed for the tick-based, CPU-constrained Screeps runtime.

**Decoupled from the Screeps runtime:** The core library uses only pure-Rust data types from `screeps-game-api` (which compile on native targets). All JS interop calls (pathfinding, creep actions, room queries, visualization) are abstracted behind traits. Enable the `screeps` feature for real game API implementations, or provide your own for offline testing and simulation.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Trait Abstractions](#trait-abstractions)
- [Tick Lifecycle](#tick-lifecycle)
- [Movement Requests](#movement-requests)
- [Pathfinding & Path Caching](#pathfinding--path-caching)
- [Conflict Resolution](#conflict-resolution)
- [Stuck Detection & Escalation](#stuck-detection--escalation)
- [Cost Matrix System](#cost-matrix-system)
- [Module Reference](#module-reference)
- [Integration Guide](#integration-guide)
- [Dependencies](#dependencies)

## Overview

In Screeps, dozens of creeps navigate a shared grid world simultaneously. Naively issuing individual `move` intents leads to collisions, deadlocks, and wasted CPU. screeps-rover solves this by collecting all movement requests for a tick, resolving conflicts globally, and then issuing coordinated intents.

**Key features:**

- **Three intent types** — MoveTo (pathfind to a position), Follow (chain behind a leader with optional pull), Flee (run from threats)
- **Global conflict resolution** — priority-based tile assignment with swap detection and chain-shoving
- **Path caching** — reuse paths across ticks to save CPU; invalidate automatically on deviation
- **Tiered stuck detection** — escalating recovery strategies from creep avoidance to shoving to failure reporting
- **Layered cost matrices** — composable terrain, structure, creep, and construction site layers with per-room caching
- **Anchor constraints** — keep stationary workers within work range even when shoved
- **Pull mechanics** — support for the Screeps pull API to move fatigued creeps
- **Offline-capable** — core algorithms work without the Screeps runtime; all game API calls are behind traits

## Features

| Feature | Default | Description |
|---------|---------|-------------|
| *(none)* | ✓ | Core library with trait abstractions. Compiles on native targets for offline testing. |
| `screeps` | | Real game API implementations: `ScreepsPathfinder`, `ScreepsCostMatrixDataSource`, `ScreepsMovementVisualizer`, `impl CreepHandle for screeps::Creep`. Required when running in the Screeps game. |
| `profile` | | Profiling instrumentation via `screeps-timing`. |

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Consumer (bot logic)                        │
│   Operations → Missions → Jobs                                     │
│   Each job creates MovementRequests via MovementData                │
│   Consumer provides: PathfindingProvider, CostMatrixDataSource,     │
│                      MovementVisualizer (optional),                 │
│                      MovementSystemExternal                         │
└──────────────────────────────┬──────────────────────────────────────┘
                               │  MovementData<Handle>
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        MovementSystem                               │
│                                                                     │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────────────┐ │
│  │  Pass 0       │   │  Pass 1       │   │  Pass 2               │ │
│  │  Dependency   │──▶│  Pathfinding  │──▶│  Conflict Resolution  │ │
│  │  Sort         │   │  & Desires    │   │  (resolver.rs)        │ │
│  └───────────────┘   └───────┬───────┘   └───────────┬───────────┘ │
│                              │                       │             │
│                              ▼                       │             │
│                   ┌──────────────────┐               │             │
│                   │ CostMatrixSystem │               │             │
│                   │ (cached layers)  │               │             │
│                   └──────────────────┘               │             │
│                                                      ▼             │
│                                           ┌──────────────────────┐ │
│                                           │  Pass 3              │ │
│                                           │  Execute & Results   │ │
│                                           └──────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
                    MovementResults<Handle>
                    (Moving / Arrived / Stuck / Failed)
```

The library is generic over a `Handle` type (`Hash + Eq + Copy + Ord`) that identifies creeps. The consumer implements the `MovementSystemExternal<Handle>` trait to bridge between its own entity model and the creep abstraction.

## Trait Abstractions

All game API calls are abstracted behind traits, allowing the library to run offline or with custom implementations.

### `CreepHandle`

Abstracts the Screeps `Creep` game object. The `screeps` feature provides `impl CreepHandle for screeps::Creep`.

```rust
pub trait CreepHandle {
    fn pos(&self) -> Position;
    fn fatigue(&self) -> u32;
    fn spawning(&self) -> bool;
    fn move_direction(&self, dir: Direction) -> Result<(), String>;
    fn pull(&self, other: &Self) -> Result<(), String>;
    fn move_pulled_by(&self, other: &Self) -> Result<(), String>;
}
```

### `MovementSystemExternal<Handle>`

The consumer implements this to bridge between its entity model and the creep abstraction:

```rust
pub trait MovementSystemExternal<Handle> {
    type Creep: CreepHandle;

    fn get_creep(&self, entity: Handle) -> Result<Self::Creep, MovementError>;
    fn get_creep_movement_data(&mut self, entity: Handle)
        -> Result<&mut CreepMovementData, MovementError>;
    fn get_room_cost(&self, from: RoomName, to: RoomName,
                     options: &RoomOptions) -> Option<f64>;
    fn get_entity_position(&self, entity: Handle) -> Option<Position>;
}
```

### `PathfindingProvider`

Abstracts the Screeps pathfinder. The `screeps` feature provides `ScreepsPathfinder`.

```rust
pub trait PathfindingProvider {
    fn search(...) -> PathfindingResult;
    fn search_many(...) -> PathfindingResult;
    fn find_route(...) -> Result<Vec<RouteStep>, String>;
    fn get_room_linear_distance(&self, from: RoomName, to: RoomName) -> u32;
    fn is_tile_walkable(&self, pos: Position) -> bool;
}
```

### `CostMatrixDataSource`

Abstracts game state queries for building cost matrices. The `screeps` feature provides `ScreepsCostMatrixDataSource`. Caching/expiration is the implementor's concern.

```rust
pub trait CostMatrixDataSource {
    fn get_structure_costs(&self, room_name: RoomName) -> Option<StuctureCostMatrixCache>;
    fn get_construction_site_costs(&self, room_name: RoomName) -> Option<ConstructionSiteCostMatrixCache>;
    fn get_creep_costs(&self, room_name: RoomName) -> Option<CreepCostMatrixCache>;
}
```

### `MovementVisualizer`

Optional intent-based visualization callbacks. Instead of drawing primitives, the movement system reports *what* happened and the implementor decides *how* to render it. The `screeps` feature provides `ScreepsMovementVisualizer` which renders directly to `RoomVisual`. Pass `None` to disable visualization entirely.

```rust
pub trait MovementVisualizer {
    fn visualize_path(&mut self, creep_pos: Position, path: &[Position]);
    fn visualize_anchor(&mut self, creep_pos: Position, anchor_pos: Position);
    fn visualize_immovable(&mut self, creep_pos: Position);
    fn visualize_stuck(&mut self, creep_pos: Position, ticks: u16);
    fn visualize_failed(&mut self, creep_pos: Position);
}
```

Visualization is controlled at two levels: globally by whether a `MovementVisualizer` is provided to `MovementSystem::new` (pass `None` to disable all visualization), and per-request via the builder (`.visualize(false)` to suppress an individual request). Requests default to `visualize: true`.

## Tick Lifecycle

Each game tick, screeps-rover processes movement in four passes:

```
Tick N
├── 1. Jobs create MovementRequests
│       move_to(entity, destination)
│       follow(entity, leader)
│       flee(entity, threats)
│
├── 2. MovementSystem::process() is called
│   │
│   ├── Pass 0 ─ Dependency Sort
│   │   Topologically sort Follow chains so leaders resolve before
│   │   followers. Detect and break cycles (convert to MoveTo).
│   │
│   ├── Pass 1 ─ Compute Desired Positions
│   │   For each creep (in sorted order):
│   │   • MoveTo: validate/reuse cached path or generate new one;
│   │     extract next step as desired position
│   │   • Follow: derive desired position from leader's resolved
│   │     movement (step into the tile the leader vacated)
│   │   • Flee: run multi-target flee pathfinder
│   │   Track stuck state, apply escalation tiers.
│   │
│   ├── Pass 2 ─ Conflict Resolution
│   │   Resolve all creeps globally:
│   │   • Detect and execute head-to-head swaps
│   │   • For contested tiles, highest priority wins
│   │   • Shove displaced creeps to adjacent tiles (chain up to 3 deep)
│   │   • Respect anchor constraints and immovable flags
│   │
│   └── Pass 3 ─ Execute Movement
│       Issue intents via CreepHandle:
│       • move_direction() for normal movement
│       • pull() + move_pulled_by() for pull mechanics
│       Record per-creep results.
│
└── 3. Results returned to jobs
        Moving / Arrived / Stuck { ticks } / Failed(reason)
```

## Movement Requests

Requests are created through `MovementData<Handle>` and configured via the builder pattern:

```rust
// Simple move-to
movement_data.move_to(entity, destination);

// Move-to with configuration
movement_data
    .move_to(entity, destination)
    .range(3)
    .priority(MovementPriority::High)
    .allow_shove(true)
    .anchor(AnchorConstraint { position: work_pos, range: 3 });

// Follow a leader (with pull for fatigued creeps)
movement_data
    .follow(entity, leader_entity)
    .range(1)
    .pull(true);

// Flee from threats
movement_data
    .flee(entity, vec![FleeTarget { pos: enemy_pos, range: 5 }]);
```

### Intent Types

| Intent | Description | Pathfinding |
|--------|-------------|-------------|
| **MoveTo** | Navigate to a position within range | Cached multi-room A* via `PathfindingProvider::search()` |
| **Follow** | Trail behind another entity | Derived from leader's resolved movement; falls back to pathfinding if leader is distant |
| **Flee** | Move away from one or more threats | `PathfindingProvider::search_many()` with flee mode |

### Priority Levels

| Priority | Behavior |
|----------|----------|
| `Immovable` | Cannot be shoved or swapped; never moves from current tile |
| `Low` | Loses most conflicts; shoved first |
| `Normal` | Default for most creeps |
| `High` | Wins contested tiles; can shove lower-priority creeps |

## Pathfinding & Path Caching

### Path Generation

Paths are generated via the `PathfindingProvider` trait with custom cost matrices:

1. **Room routing** — `PathfindingProvider::find_route()` determines the sequence of rooms to traverse, using `get_room_cost()` from the external trait.
2. **Cost matrix construction** — For each room in the route, `CostMatrixSystem` builds a `LocalCostMatrix` from cached layers (structures, creeps, construction sites, source keeper aggro) via the `CostMatrixDataSource` trait.
3. **Search** — The pathfinder runs with configurable `max_ops`, `plains_cost`, and `swamp_cost`. Stuck escalation may modify these parameters.

### Path Caching

Paths are cached in `CreepPathData` and reused across ticks:

```
CreepPathData
├── destination: Position      # Target position
├── range: u32                 # Acceptable range
├── path: Vec<Position>        # Remaining waypoints
├── time: u32                  # Ticks since generation
└── stuck_state: StuckState    # Stuck tracking
```

**Validation rules:**
- Path is valid if destination and range match the current request
- Creep must be at or near the start of the remaining path
- Path expires after `reuse_path_length` ticks (default: 5)
- Invalidated immediately when stuck detection triggers a repath

## Conflict Resolution

The resolver (`resolver.rs`) processes all creeps in a single global pass to produce a conflict-free set of movement intents.

### Resolution Algorithm

```
                    ┌──────────────────────┐
                    │  All Resolved Creeps  │
                    │  (desired positions)  │
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │  1. Detect Swaps     │
                    │  A→B and B→A?        │──── Both allowed? ──▶ Execute swap
                    │  Check anchor        │                       (both resolved)
                    │  constraints         │
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │  2. Build Intent Map │
                    │  desired_pos → [entities]
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │  3. For each         │
                    │  contested tile:     │
                    │  • Pick winner       │──── Highest priority,
                    │    (priority, then   │     then most stuck_ticks
                    │     stuck_ticks)     │
                    │  • Tile occupied?    │
                    │    └─ try_shove()    │──── Recursive (max depth 3)
                    │  • Shove succeeded?  │
                    │    ├─ Yes: winner    │
                    │    │  moves in       │
                    │    └─ No: winner     │
                    │       stays put      │
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │  4. Remaining        │
                    │  unresolved creeps   │
                    │  stay in place       │
                    └──────────────────────┘
```

### Swap Detection

Two creeps wanting each other's tiles (head-to-head) are detected and resolved as a swap if:
- Both have `allow_swap` enabled
- Both satisfy their anchor constraints at the new position

### Chain Shoving

When a higher-priority creep needs a tile occupied by another creep, the resolver attempts to shove the occupant to an adjacent walkable tile:

```
Shove chain (max depth 3):

  [A wants tile of B]
       │
       ▼
  Can B move to an empty adjacent tile?
  ├── Yes → Shove B there; A takes B's old tile
  └── No  → Can B shove its neighbor C? (depth + 1)
            ├── Yes → C moves, B takes C's tile, A takes B's tile
            └── No  → Shove fails; A stays put
```

**Shove constraints:**
- `allow_shove` must be true on the target
- Target must not be `Immovable` priority
- Anchor constraints are respected (shoved position must be within work range)
- Maximum chain depth of 3 prevents unbounded recursion

## Stuck Detection & Escalation

screeps-rover tracks two dimensions of "stuckness" per creep:

| Metric | Definition |
|--------|-----------|
| **ticks_immobile** | Consecutive ticks where the creep's position did not change |
| **ticks_no_progress** | Consecutive ticks where distance to target did not decrease (even if position changed) |

These feed into a tiered escalation system with configurable thresholds:

```
Stuck Escalation Tiers (default thresholds)
────────────────────────────────────────────

Tick 0   Normal pathfinding
  │
  ▼
Tick 2   Tier 1: Avoid friendly creeps
  │       └─ Adds friendly_creeps to cost matrix (cost 255)
  ▼         so paths route around allies
Tick 4   Tier 2: Increase search ops
  │       └─ Doubles max_ops for pathfinder search
  ▼         to explore more of the grid
Tick 6   Tier 3: Enable shoving
  │       └─ Resolver will shove other creeps out of
  ▼         the way during conflict resolution
Tick 10  Tier 4: Report failure
  │       └─ Returns Failed(StuckTimeout) to the job
  ▼         so it can take alternative action
Tick 15  No-progress repath
          └─ Forces path regeneration even if creep
            is moving (but not getting closer)
```

Thresholds are configurable per-creep via `StuckThresholds`, allowing military creeps to escalate faster than economy creeps.

## Cost Matrix System

The `CostMatrixSystem` builds per-room cost matrices from composable layers with caching. It operates on a user-owned `CostMatrixCache` (passed as `&mut CostMatrixCache`), and fills it with data from the `CostMatrixDataSource` trait. The user is responsible for creating, persisting, and deserializing the cache — screeps-rover does not manage storage.

### Layer Architecture

```
┌─────────────────────────────────────────────────┐
│              Final LocalCostMatrix              │
│              (2500 bytes, per-room)              │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌─────────────┐  Applied based on              │
│  │ Terrain     │  CostMatrixOptions              │
│  │ (base)      │  flags and costs               │
│  └──────┬──────┘                                │
│         │                                       │
│  ┌──────▼──────┐  Persisted across ticks        │
│  │ Structures  │  • Roads (configurable cost)   │
│  │ (cached)    │  • Ramparts, containers        │
│  └──────┬──────┘  • Blocking structures (255)   │
│         │                                       │
│  ┌──────▼──────────────┐  Rebuilt each tick     │
│  │ Construction Sites  │  • Blocked (255)       │
│  │ (ephemeral)         │  • Active/inactive     │
│  └──────┬──────────────┘  • Friendly/hostile    │
│         │                                       │
│  ┌──────▼──────┐  Rebuilt each tick             │
│  │ Creeps      │  • Friendly (255)              │
│  │ (ephemeral) │  • Hostile (255)               │
│  └──────┬──────┘                                │
│         │                                       │
│  ┌──────▼──────────────┐  Rebuilt each tick     │
│  │ Source Keeper Aggro │  Tiles within radius 3 │
│  │ (ephemeral)         │  of Source Keepers      │
│  └─────────────────────┘  (configurable cost)   │
│                                                 │
└─────────────────────────────────────────────────┘
```

### CostMatrixOptions

Controls which layers are included and their costs:

```rust
CostMatrixOptions {
    structures: true,           // Include structure layer
    friendly_creeps: false,     // Include friendly creep positions
    hostile_creeps: true,       // Include hostile creep positions
    construction_sites: true,   // Include construction site layer
    source_keeper_aggro: true,  // Include SK aggro zones

    road_cost: 1,               // Cost for road tiles
    plains_cost: 2,             // Cost for plain terrain
    swamp_cost: 10,             // Cost for swamp terrain
    source_keeper_aggro_cost: 50, // Cost for SK aggro tiles
    // Optional per-type construction site costs...
}
```

### Caching Strategy

| Layer | Cache lifetime | Persistence |
|-------|---------------|-------------|
| **Structures** | Until room state changes | Serialized in `CostMatrixCache` (user-managed) |
| **Construction sites** | Single tick | `#[serde(skip)]` — not persisted |
| **Creeps** | Single tick | `#[serde(skip)]` — not persisted |
| **Source keeper aggro** | Single tick | `#[serde(skip)]` — not persisted |

### Persistence

`CostMatrixCache` implements `Serialize` and `Deserialize`. The user owns the cache and is responsible for loading it (e.g. from a RawMemory segment) before the tick and saving it afterwards. For offline or single-tick usage, simply create a default cache with `CostMatrixCache::default()` — no storage abstraction needed.

## Module Reference

| Module | File | Purpose |
|--------|------|---------|
| **traits** | `traits.rs` | Trait abstractions: `CreepHandle`, `PathfindingProvider`, `CostMatrixDataSource`, `MovementVisualizer` |
| **screeps_impl** | `screeps_impl.rs` | *(feature `screeps`)* Real game API implementations: `ScreepsPathfinder`, `ScreepsCostMatrixDataSource`, `ScreepsMovementVisualizer`, `impl CreepHandle for Creep` |
| **movementsystem** | `movementsystem.rs` | Core movement orchestration: path generation, stuck detection, movement execution, visualization |
| **resolver** | `resolver.rs` | Global conflict resolution: swap detection, priority assignment, chain shoving |
| **movementrequest** | `movementrequest.rs` | Request types: `MovementIntent`, `MovementRequest`, builder, priority, anchor constraints |
| **movementresult** | `movementresult.rs` | Result types: `MovementResult`, `MovementFailure`, `MovementResults` |
| **costmatrixsystem** | `costmatrixsystem.rs` | Cost matrix management: layer caching, building; operates on user-owned `CostMatrixCache`; uses `CostMatrixDataSource` |
| **costmatrix** | `costmatrix.rs` | Cost matrix data structures: `SparseCostMatrix`, `LinearCostMatrix`, read/write/apply traits |
| **location** | `location.rs` | Compact `Location` type: packed `u16` coordinates, distance calculations |
| **utility** | `utility.rs` | Room traversal rules: novice/respawn/closed zone checks, linear distance |
| **error** | `error.rs` | Error type alias (`MovementError = String`) |
| **constants** | `constants.rs` | Game constants: Source Keeper name and aggro radius |

## Integration Guide

### Add Dependency

```toml
[dependencies]
# For offline testing / simulation (no JS calls):
screeps-rover = { path = "../screeps-rover" }

# For use in the Screeps game (enables real API implementations):
screeps-rover = { path = "../screeps-rover", features = ["screeps"] }
```

### Simple Integration

The quickest way to get started. Uses `ObjectId<Creep>` directly as the handle type and a `HashMap` for movement data storage — no ECS required.

#### 1. Set Up State

```rust
use std::collections::HashMap;
use screeps::prelude::*;
use screeps::{Creep, ObjectId};
use screeps_rover::*;
use screeps_rover::screeps_impl::*;

/// Per-tick movement state. Persist `movement_data` and `cost_matrix_cache`
/// across ticks (e.g. in Memory or RawMemory segments) for path reuse and
/// cost matrix caching.
struct MovementState {
    movement_data_map: HashMap<ObjectId<Creep>, CreepMovementData>,
    cost_matrix_cache: CostMatrixCache,
}
```

#### 2. Implement the External Trait

```rust
struct SimpleExternal<'a> {
    movement_data_map: &'a mut HashMap<ObjectId<Creep>, CreepMovementData>,
}

impl<'a> MovementSystemExternal<ObjectId<Creep>> for SimpleExternal<'a> {
    type Creep = Creep;

    fn get_creep(&self, id: ObjectId<Creep>) -> Result<Creep, MovementError> {
        id.resolve().ok_or_else(|| "Creep not found".to_owned())
    }

    fn get_creep_movement_data(
        &mut self,
        id: ObjectId<Creep>,
    ) -> Result<&mut CreepMovementData, MovementError> {
        Ok(self.movement_data_map
            .entry(id)
            .or_default())
    }

    // get_room_cost has a default implementation that returns Some(1.0)
    // for all rooms. Override for smarter room routing.

    fn get_entity_position(&self, id: ObjectId<Creep>) -> Option<Position> {
        id.resolve().map(|c| c.pos())
    }
}
```

#### 3. Process Movement Each Tick

```rust
fn process_movement(state: &mut MovementState) {
    // Collect movement requests
    let mut requests = MovementData::new();

    for creep in screeps::game::creeps().values() {
        let id = creep.try_id().unwrap();
        // Example: move every creep to a target
        requests.move_to(id, target_pos).range(1);
    }

    // Build systems for this tick
    let mut cost_matrix_system = CostMatrixSystem::new(
        &mut state.cost_matrix_cache,
        Box::new(ScreepsCostMatrixDataSource),
    );
    let mut pathfinder = ScreepsPathfinder;
    let mut visualizer = ScreepsMovementVisualizer;

    let mut system = MovementSystem::new(
        &mut cost_matrix_system,
        &mut pathfinder,
        Some(&mut visualizer), // or None to disable visualization
    );
    system.set_reuse_path_length(5);

    // Process
    let mut external = SimpleExternal {
        movement_data_map: &mut state.movement_data_map,
    };
    let results = system.process(&mut external, requests);

    // Handle results
    for (id, result) in results.results.iter() {
        match result {
            MovementResult::Moving => { /* on the way */ }
            MovementResult::Arrived => { /* can do work */ }
            MovementResult::Stuck { .. } => { /* waiting */ }
            MovementResult::Failed(_) => { /* handle failure */ }
        }
    }
}
```

That's it. `ObjectId<Creep>` is `Copy + Hash + Eq + Ord + Serialize + Deserialize`, so it satisfies all handle requirements. Persist `movement_data_map` and `cost_matrix_cache` across ticks for path reuse and cost matrix caching.

### Advanced Integration (ECS)

For larger bots, you'll typically use an ECS (e.g. [specs](https://docs.rs/specs/)) with your own entity type as the handle. This gives you more control over room routing, hostile avoidance, and per-entity storage.

#### 1. Implement the External Trait

With an ECS, the external trait bridges between your entity model and the creep abstraction:

```rust
use screeps_rover::*;
use specs::prelude::*;

struct MyExternal<'a, 'b> {
    entities: &'b Entities<'a>,
    creep_owner: &'b ReadStorage<'a, CreepOwner>,
    creep_movement: &'b mut WriteStorage<'a, MyCreepMovement>,
    room_data: &'b ReadStorage<'a, RoomData>,
    mapping: &'b Read<'a, EntityMappingData>,
}

impl<'a, 'b> MovementSystemExternal<Entity> for MyExternal<'a, 'b> {
    type Creep = screeps::Creep;

    fn get_creep(&self, entity: Entity) -> Result<screeps::Creep, MovementError> {
        let owner = self.creep_owner.get(entity)
            .ok_or("Expected creep owner")?;
        owner.id().resolve().ok_or("Creep not found".to_owned())
    }

    fn get_creep_movement_data(
        &mut self,
        entity: Entity,
    ) -> Result<&mut CreepMovementData, MovementError> {
        // Insert default if missing, then return mutable reference
        if !self.creep_movement.contains(entity) {
            let _ = self.creep_movement.insert(entity, MyCreepMovement::default());
        }
        self.creep_movement
            .get_mut(entity)
            .map(|m| &mut m.0)
            .ok_or("Failed to get movement data".to_owned())
    }

    fn get_room_cost(
        &self,
        from: RoomName,
        to: RoomName,
        options: &RoomOptions,
    ) -> Option<f64> {
        // Use room visibility data for smarter routing:
        // - Return None for impassable/hostile rooms
        // - Return higher costs for dangerous rooms
        // - Return lower costs for owned rooms
        let room_data = self.mapping.get_room(&to)
            .and_then(|e| self.room_data.get(e));

        match room_data {
            Some(data) if data.is_hostile() => {
                match options.hostile_behavior() {
                    HostileBehavior::Allow => Some(5.0),
                    HostileBehavior::HighCost => Some(10.0),
                    HostileBehavior::Deny => None,
                }
            }
            Some(data) if data.is_owned() => Some(1.0),
            _ => Some(2.0),
        }
    }

    fn get_entity_position(&self, entity: Entity) -> Option<Position> {
        let owner = self.creep_owner.get(entity)?;
        let creep = owner.id().resolve()?;
        Some(creep.pos())
    }
}
```

#### 2. Create the Systems

```rust
use screeps_rover::*;
use screeps_rover::screeps_impl::*;

// The user owns the CostMatrixCache as an ECS resource.
// Load from a RawMemory segment on environment init, or start fresh.
let cache: CostMatrixCache = load_from_segment(55).unwrap_or_default();
world.insert(cache);
```

#### 3. Process Movement in an ECS System

```rust
fn run(&mut self, mut data: Self::SystemData) {
    let movement_data = std::mem::take(&mut *data.movement);

    let mut external = MyExternal {
        entities: &data.entities,
        creep_owner: &data.creep_owner,
        creep_movement: &mut data.creep_movement,
        room_data: &data.room_data,
        mapping: &data.mapping,
    };

    let mut cost_matrix_system = CostMatrixSystem::new(
        &mut data.cost_matrix_cache,
        Box::new(ScreepsCostMatrixDataSource),
    );
    let mut pathfinder = ScreepsPathfinder;
    let mut visualizer = ScreepsMovementVisualizer;

    let mut system = MovementSystem::new(
        &mut cost_matrix_system,
        &mut pathfinder,
        Some(&mut visualizer), // or None to disable
    );
    system.set_reuse_path_length(5);

    *data.movement_results = system.process(&mut external, movement_data);
}
```

#### 4. Persist Data

`CostMatrixCache` and `CreepMovementData` are both serializable. Persist them across ticks so paths and structure cost data survive VM reloads:

```rust
// Serialize the cost matrix cache to a RawMemory segment after the tick:
save_to_segment(55, &cost_matrix_cache);

// CreepMovementData is stored per-entity in your ECS and serialized
// alongside other components.
```

Ephemeral cost matrix layers (creeps, construction sites) are `#[serde(skip)]` and will be rebuilt automatically from the `CostMatrixDataSource` on the next tick.

## Dependencies

| Crate | Purpose |
|-------|---------|
| [screeps-game-api](https://github.com/rustyscreeps/screeps-game-api) | Pure-Rust data types (`Position`, `RoomName`, `Direction`, etc.). JS interop types are only used behind the `screeps` feature. |
| [screeps-cache](https://github.com/Azaril/screeps-cache) | Lazy evaluation and caching utilities |
| [serde](https://serde.rs/) | Serialization for path data, cost matrices, and stuck state |
| [log](https://docs.rs/log/) | Logging facade |
| screeps-timing *(optional, `profile` feature)* | Profiling and timing instrumentation |

## License

See repository root for license information.
