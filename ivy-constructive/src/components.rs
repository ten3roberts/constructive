use constructive::{
    brush::PositionedBrush,
    navmesh::{Navmesh, NavmeshSettings},
};
use ivy_engine::flax::component;

component! {
    /// A collection of brushes describing the navigable shape of an object
    pub brushes: Vec<PositionedBrush>,
    pub navmesh: Navmesh,
    pub navmesh_settings: NavmeshSettings,
}
