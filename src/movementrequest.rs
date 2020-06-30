use screeps::*;
use super::costmatrixsystem::*;

pub struct RoomOptions {
    allow_hostile: bool,
}

impl RoomOptions {
    pub fn allow_hostile(&self) -> bool {
        self.allow_hostile
    }
}

pub struct MovementRequest {
    pub(crate) destination: RoomPosition,
    pub(crate) range: u32,
    pub(crate) room_options: Option<RoomOptions>,
    pub(crate) cost_matrix_options: Option<CostMatrixOptions>,
    pub(crate) visualization: Option<PolyStyle>,
}

impl Default for RoomOptions {
    fn default() -> Self {
        RoomOptions {
            allow_hostile: false,
        }
    }
}

impl MovementRequest {
    pub fn move_to(destination: RoomPosition) -> MovementRequest {
        MovementRequest {
            destination,
            range: 0,
            room_options: None,
            cost_matrix_options: None,
            visualization: None,
        }
    }
}

pub struct MovementRequestBuilder<'a> {
    request: &'a mut MovementRequest,
}

impl<'a> Into<MovementRequestBuilder<'a>> for &'a mut MovementRequest {
    fn into(self) -> MovementRequestBuilder<'a> {
        MovementRequestBuilder { request: self }
    }
}

impl<'a> MovementRequestBuilder<'a> {
    pub fn range(&mut self, range: u32) -> &mut Self {
        self.request.range = range;

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

    pub fn visualization(&mut self, style: PolyStyle) -> &mut Self {
        self.request.visualization = Some(style);

        self
    }
}
