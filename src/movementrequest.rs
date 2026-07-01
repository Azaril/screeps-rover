use super::costmatrixsystem::*;
use super::movementsystem::StuckThresholds;
use screeps::local::*;
use std::hash::Hash;

#[derive(Copy, Clone)]
pub enum HostileBehavior {
    Allow,
    HighCost,
    Deny,
}

#[derive(Copy, Clone)]
pub struct RoomOptions {
    hostile_behavior: HostileBehavior,
}

impl RoomOptions {
    pub fn hostile_behavior(&self) -> HostileBehavior {
        self.hostile_behavior
    }
}

impl RoomOptions {
    pub fn new(hostile_behavior: HostileBehavior) -> RoomOptions {
        Self { hostile_behavior }
    }
}

impl Default for RoomOptions {
    fn default() -> Self {
        RoomOptions {
            hostile_behavior: HostileBehavior::Deny,
        }
    }
}

/// Priority level for movement requests. Higher priority creeps win
/// contested tiles and can shove lower priority creeps.
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MovementPriority {
    /// Cannot be shoved or swapped; does not move.
    Immovable,
    /// Lowest priority, will be shoved first.
    Low,
    /// Default priority for most creeps.
    #[default]
    Normal,
    /// High priority, wins most conflicts.
    High,
}

impl MovementPriority {
    /// The enum's NUMERIC ANCHOR on the shared i64 priority lane (ADR 0033 §D5.4 decision #9 —
    /// the w-as-priority substrate). The resolver compares `MovementRequest::priority_value`
    /// falling back to this anchor, so enum-only requests keep EXACTLY the derived `Ord` total
    /// order (anchors are strictly monotone in variant order), while a caller-supplied numeric
    /// priority (e.g. §D5.4's quantized `w(creep)` in milli-e/t) slots BETWEEN anchors: the
    /// 1_000_000 spacing leaves room for w up to 1000 e/t at milli-e/t quantization between any
    /// two adjacent tiers. `Immovable` anchors below `Low` (it never moves and loses every
    /// contest, matching the derived ordering); its no-shove/no-swap semantics stay ENUM-checked
    /// in the resolver, never value-checked.
    pub fn anchor_value(self) -> i64 {
        match self {
            MovementPriority::Immovable => -1_000_000,
            MovementPriority::Low => 0,
            MovementPriority::Normal => 1_000_000,
            MovementPriority::High => 2_000_000,
        }
    }
}

/// A target to flee from.
#[derive(Clone)]
pub struct FleeTarget {
    pub pos: Position,
    pub range: u32,
}

/// Constrains a creep to stay within `range` of `position` when shoved or
/// swapped. Used by stationary workers (upgraders, builders, etc.) so the
/// resolver can rearrange them without pushing them out of work range.
#[derive(Copy, Clone, Debug)]
pub struct AnchorConstraint {
    pub position: Position,
    pub range: u32,
}

/// Describes the semantic movement goal for a creep.
pub enum MovementIntent<Handle> {
    /// Move to a fixed position within range. Standard pathfinding.
    MoveTo { destination: Position, range: u32 },
    /// Follow another entity. The follower's desired next tile is derived
    /// from the leader's resolved movement during the same tick.
    /// If `pull` is true, the leader will issue pull and the follower will
    /// issue move_pulled_by, allowing fatigued creeps to move with the group.
    /// If `desired_offset` is set, the follower prefers the tile at that
    /// (dx, dy) offset from the leader's new position. This is computed
    /// *after* the leader's move is resolved (topological sort ensures this).
    /// If the offset tile is blocked, the follower falls back to any tile
    /// within `range` of the leader.
    Follow {
        target: Handle,
        range: u32,
        pull: bool,
        desired_offset: Option<(i32, i32)>,
    },
    /// Flee from one or more targets.
    Flee {
        targets: Vec<FleeTarget>,
        range: u32,
    },
}

pub struct MovementRequest<Handle> {
    pub(crate) intent: MovementIntent<Handle>,
    pub(crate) room_options: Option<RoomOptions>,
    pub(crate) cost_matrix_options: Option<CostMatrixOptions>,
    pub(crate) visualize: bool,
    pub(crate) priority: MovementPriority,
    /// Optional NUMERIC priority on the shared i64 lane (see
    /// [`MovementPriority::anchor_value`]). `None` (the default) = the enum's anchor, which is
    /// byte-identical to the historical enum-only ordering; `Some(v)` lets a caller bid a
    /// quantized value (§D5.4 `w(creep)` in milli-e/t) that slots between the enum tiers.
    /// Ties keep the exact historical tie-break (stuck_ticks, then Handle).
    pub(crate) priority_value: Option<i64>,
    pub(crate) allow_shove: bool,
    pub(crate) allow_swap: bool,
    pub(crate) anchor: Option<AnchorConstraint>,
    /// Per-request stuck-escalation ladder override (ADR 0033 M5 combat-adjudication end-state:
    /// SPLIT DEFAULTS — haul jobs pass a slow ladder like `ladder(8)` for the +H, military keeps
    /// the fast system default). `None` = the system-level `MovementSystem::stuck_thresholds`,
    /// which is byte-identical to the historical behavior.
    pub(crate) stuck_thresholds: Option<StuckThresholds>,
}

impl<Handle> MovementRequest<Handle> {
    pub fn move_to(destination: Position) -> MovementRequest<Handle> {
        MovementRequest {
            intent: MovementIntent::MoveTo {
                destination,
                range: 0,
            },
            room_options: None,
            cost_matrix_options: None,
            visualize: true,
            priority: MovementPriority::default(),
            priority_value: None,
            allow_shove: true,
            allow_swap: true,
            anchor: None,
            stuck_thresholds: None,
        }
    }

    pub fn follow(target: Handle) -> MovementRequest<Handle> {
        MovementRequest {
            intent: MovementIntent::Follow {
                target,
                range: 1,
                pull: false,
                desired_offset: None,
            },
            room_options: None,
            cost_matrix_options: None,
            visualize: true,
            priority: MovementPriority::default(),
            priority_value: None,
            allow_shove: true,
            allow_swap: true,
            anchor: None,
            stuck_thresholds: None,
        }
    }

    pub fn flee(targets: Vec<FleeTarget>) -> MovementRequest<Handle> {
        MovementRequest {
            intent: MovementIntent::Flee { targets, range: 5 },
            room_options: None,
            cost_matrix_options: None,
            visualize: true,
            priority: MovementPriority::High,
            priority_value: None,
            allow_shove: false,
            allow_swap: true,
            anchor: None,
            stuck_thresholds: None,
        }
    }

    /// The effective i64 contention priority: the caller's numeric bid if set, else the enum's
    /// anchor. This single value is what the resolver orders by (winner selection + shove gates).
    pub fn effective_priority(&self) -> i64 {
        self.priority_value.unwrap_or_else(|| self.priority.anchor_value())
    }

    /// Get the destination position for MoveTo intents. For Follow/Flee, returns None.
    pub fn destination(&self) -> Option<Position> {
        match &self.intent {
            MovementIntent::MoveTo { destination, .. } => Some(*destination),
            MovementIntent::Follow { .. } | MovementIntent::Flee { .. } => None,
        }
    }

    /// Get the range for the intent.
    pub fn range(&self) -> u32 {
        match &self.intent {
            MovementIntent::MoveTo { range, .. } => *range,
            MovementIntent::Follow { range, .. } => *range,
            MovementIntent::Flee { range, .. } => *range,
        }
    }
}

pub struct MovementRequestBuilder<'a, Handle> {
    request: &'a mut MovementRequest<Handle>,
}

impl<'a, Handle> From<&'a mut MovementRequest<Handle>> for MovementRequestBuilder<'a, Handle> {
    fn from(request: &'a mut MovementRequest<Handle>) -> MovementRequestBuilder<'a, Handle> {
        MovementRequestBuilder { request }
    }
}

impl<'a, Handle> MovementRequestBuilder<'a, Handle> {
    pub fn range(&mut self, range: u32) -> &mut Self {
        match &mut self.request.intent {
            MovementIntent::MoveTo {
                range: ref mut r, ..
            } => *r = range,
            MovementIntent::Follow {
                range: ref mut r, ..
            } => *r = range,
            MovementIntent::Flee {
                range: ref mut r, ..
            } => *r = range,
        }

        self
    }

    /// Enable pull mechanics for Follow intents.
    pub fn pull(&mut self, enable: bool) -> &mut Self {
        if let MovementIntent::Follow {
            pull: ref mut p, ..
        } = &mut self.request.intent
        {
            *p = enable;
        }

        self
    }

    /// Set a desired offset from the follow target's position.
    /// When the leader moves, the follower will prefer the tile at
    /// (leader_new_pos.x + dx, leader_new_pos.y + dy) rather than
    /// the leader's vacated tile. Falls back to any tile within range
    /// if the offset tile is blocked or out of bounds.
    pub fn desired_offset(&mut self, dx: i32, dy: i32) -> &mut Self {
        if let MovementIntent::Follow {
            desired_offset: ref mut o,
            ..
        } = &mut self.request.intent
        {
            *o = Some((dx, dy));
        }

        self
    }

    pub fn room_options(&mut self, options: RoomOptions) -> &mut Self {
        self.request.room_options = Some(options);

        self
    }

    pub fn cost_matrix_options(&mut self, options: CostMatrixOptions) -> &mut Self {
        self.request.cost_matrix_options = Some(options);

        self
    }

    pub fn visualize(&mut self, enable: bool) -> &mut Self {
        self.request.visualize = enable;

        self
    }

    pub fn priority(&mut self, priority: MovementPriority) -> &mut Self {
        self.request.priority = priority;

        self
    }

    /// Set a NUMERIC priority on the shared i64 lane (see [`MovementPriority::anchor_value`] for
    /// the enum anchors and spacing). Overrides the enum for resolver ordering; leave unset for
    /// the historical enum-only behavior.
    pub fn priority_value(&mut self, value: i64) -> &mut Self {
        self.request.priority_value = Some(value);

        self
    }

    pub fn allow_shove(&mut self, allow: bool) -> &mut Self {
        self.request.allow_shove = allow;

        self
    }

    pub fn allow_swap(&mut self, allow: bool) -> &mut Self {
        self.request.allow_swap = allow;

        self
    }

    /// Set an anchor constraint.
    pub fn anchor(&mut self, constraint: AnchorConstraint) -> &mut Self {
        self.request.anchor = Some(constraint);

        self
    }

    /// Override the stuck-escalation ladder for THIS request only (the ADR 0033 split-defaults
    /// end-state: per-job `StuckThresholds`, e.g. `ladder(8)` for haul, system default for
    /// military). Unset = the system-level thresholds.
    pub fn stuck_thresholds(&mut self, thresholds: StuckThresholds) -> &mut Self {
        self.request.stuck_thresholds = Some(thresholds);

        self
    }
}
