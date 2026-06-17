// Only consumed by the `screeps`-gated `ScreepsCostMatrixDataSource` (source-keeper aggro pricing),
// so gate them to keep the pure default-feature build (used headless by the combat sim) warning-free.
#[cfg(feature = "screeps")]
pub const SOURCE_KEEPER_NAME: &str = "Source Keeper";
#[cfg(feature = "screeps")]
pub const SOURCE_KEEPER_AGRO_RADIUS: u32 = 3;
