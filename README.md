# screeps-rover

A coordinated movement and pathfinding library for [Screeps](https://screeps.com/), written in Rust. screeps-rover manages multi-creep movement with global conflict resolution, path caching, stuck detection, and layered cost matrices — all designed for the tick-based, CPU-constrained Screeps runtime.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
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

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Consumer (bot logic)                        │
│   Operations → Missions → Jobs                                     │
│   Each job creates MovementRequests via MovementData                │
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

The library is generic over a `Handle` type (`Hash + Eq + Copy`) that identifies creeps. The consumer implements the `MovementSystemExternal<Handle>` trait to bridge between its own entity model and the Screeps API.

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
│       Issue Screeps intents:
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
    .flee(entity, &[FleeTarget { pos: enemy_pos, range: 5 }]);
```

### Intent Types

| Intent | Description | Pathfinding |
|--------|-------------|-------------|
| **MoveTo** | Navigate to a position within range | Cached multi-room A* via `pathfinder::search()` |
| **Follow** | Trail behind another entity | Derived from leader's resolved movement; falls back to pathfinding if leader is distant |
| **Flee** | Move away from one or more threats | `pathfinder::search_many()` with `flee: true` |

### Priority Levels

| Priority | Behavior |
|----------|----------|
| `Immovable` | Cannot be shoved or swapped; never moves from current tile |
| `Low` | Loses most conflicts; shoved first |
| `Normal` | Default for most creeps |
| `High` | Wins contested tiles; can shove lower-priority creeps |

## Pathfinding & Path Caching

### Path Generation

Paths are generated via the Screeps `pathfinder::search()` API with custom cost matrices:

1. **Room routing** — `game::map::find_route()` determines the sequence of rooms to traverse, using `get_room_cost()` from the external trait and room status checks (novice/respawn/closed zone rules).
2. **Cost matrix construction** — For each room in the route, `CostMatrixSystem` builds a `LocalCostMatrix` from cached layers (structures, creeps, construction sites, source keeper aggro).
3. **Search** — The pathfinder runs with configurable `max_ops`, `max_rooms`, `plains_cost`, and `swamp_cost`. Stuck escalation may modify these parameters.

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

Swaps use the Screeps `move_direction()` intent — the game engine handles the actual position exchange.

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

The `CostMatrixSystem` builds per-room cost matrices from composable layers with caching.

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
| **Structures** | Until room state changes (re-scanned when room is visible) | Serialized to RawMemory segment via `CostMatrixStorage` trait |
| **Construction sites** | Single tick | Not persisted (rebuilt from game state) |
| **Creeps** | Single tick | Not persisted (rebuilt from game state) |
| **Source keeper aggro** | Single tick | Not persisted (rebuilt from game state) |

Structure caches are persisted because structure layouts change rarely and scanning is expensive. Creep and construction site caches are ephemeral because they change every tick.

### Custom Storage

The `CostMatrixStorage` trait abstracts persistence:

```rust
pub trait CostMatrixStorage {
    fn get_cache(&self, segment: u32) -> Result<CostMatrixCache, String>;
    fn set_cache(&mut self, segment: u32, data: &CostMatrixCache) -> Result<(), String>;
}
```

This allows the consumer to plug in their own storage backend (e.g., RawMemory segments, in-memory cache, etc.).

## Module Reference

| Module | File | Purpose |
|--------|------|---------|
| **movementsystem** | `movementsystem.rs` | Core movement orchestration: path generation, stuck detection, movement execution, visualization |
| **resolver** | `resolver.rs` | Global conflict resolution: swap detection, priority assignment, chain shoving |
| **movementrequest** | `movementrequest.rs` | Request types: `MovementIntent`, `MovementRequest`, builder, priority, anchor constraints |
| **movementresult** | `movementresult.rs` | Result types: `MovementResult`, `MovementFailure`, `MovementResults` |
| **costmatrixsystem** | `costmatrixsystem.rs` | Cost matrix management: layer caching, building, lazy evaluation, storage |
| **costmatrix** | `costmatrix.rs` | Cost matrix data structures: `SparseCostMatrix`, `LinearCostMatrix`, read/write/apply traits |
| **location** | `location.rs` | Compact `Location` type: packed `u16` coordinates, distance calculations |
| **utility** | `utility.rs` | Room traversal rules: novice/respawn/closed zone checks |
| **error** | `error.rs` | Error type alias (`MovementError = String`) |
| **constants** | `constants.rs` | Game constants: Source Keeper name and aggro radius |

## Integration Guide

### 1. Implement the External Trait

The consumer must implement `MovementSystemExternal<Handle>` to bridge between its entity model and the Screeps API:

```rust
pub trait MovementSystemExternal<Handle> {
    /// Look up the Screeps Creep object for a given entity handle.
    fn get_creep(&self, entity: &Handle) -> Result<Creep, MovementError>;

    /// Get mutable access to the entity's cached movement data
    /// (path, stuck state). This is stored/serialized by the consumer.
    fn get_creep_movement_data(&self, entity: &Handle)
        -> Result<&mut CreepMovementData, MovementError>;

    /// Room traversal cost for pathfinding. Return None if the room
    /// is impassable; return Some(cost) otherwise (default: 1.0).
    fn get_room_cost(&self, from: RoomName, to: RoomName,
                     options: &RoomOptions) -> Option<f64>;

    /// Position lookup for Follow intents (resolve leader's position).
    fn get_entity_position(&self, entity: &Handle) -> Option<Position>;
}
```

### 2. Create the Systems

```rust
// Create cost matrix system with your storage backend
let mut cost_matrix_system = CostMatrixSystem::new(
    Box::new(my_storage),
    55, // RawMemory segment for structure cache
);

// Create movement system
let mut movement_system = MovementSystem::new(&mut cost_matrix_system);
movement_system.set_reuse_path_length(5);
```

### 3. Collect Requests

```rust
let mut movement_data = MovementData::new();

// Each job/creep adds its request
movement_data.move_to(harvester, source_pos).range(1);
movement_data.move_to(builder, construction_site)
    .range(3)
    .anchor(AnchorConstraint { position: work_pos, range: 3 });
movement_data.follow(carrier, harvester).pull(true);
movement_data.flee(scout, &flee_targets);
```

### 4. Process and Read Results

```rust
let results = movement_system.process(
    &mut movement_data,
    &mut external,  // your MovementSystemExternal impl
);

for (entity, result) in results.iter() {
    match result {
        MovementResult::Moving => { /* on the way */ }
        MovementResult::Arrived => { /* can do work */ }
        MovementResult::Stuck { ticks } => { /* waiting */ }
        MovementResult::Failed(failure) => {
            match failure {
                MovementFailure::PathNotFound => { /* reroute */ }
                MovementFailure::StuckTimeout { ticks } => { /* give up */ }
                MovementFailure::RoomBlocked => { /* avoid room */ }
                MovementFailure::InternalError(msg) => { /* log */ }
            }
        }
    }
}
```

### 5. Persist Movement Data

`CreepMovementData` is serializable. Store it alongside your ECS components so paths and stuck state survive VM reloads:

```rust
#[derive(Serialize, Deserialize)]
struct MyCreepComponent {
    movement_data: CreepMovementData,
    // ... other fields
}
```

## Dependencies

| Crate | Purpose |
|-------|---------|
| [screeps-game-api](https://github.com/rustyscreeps/screeps-game-api) | Typed Rust bindings for the Screeps JavaScript API |
| [screeps-cache](https://github.com/Azaril/screeps-cache) | Lazy evaluation and caching utilities |
| [serde](https://serde.rs/) | Serialization for path data, cost matrices, and stuck state |
| [log](https://docs.rs/log/) | Logging facade |
| screeps-timing *(optional, `profile` feature)* | Profiling and timing instrumentation |

## License

See repository root for license information.
