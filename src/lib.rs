//! A minimal vector-graphics data model: a planar arrangement of cubic bezier
//! curves with face-based fills. Curves never cross — intersections are
//! resolved by splitting curves and snapping nearby vertices together.

mod face;
mod fill;
mod geom;
mod query;
mod scene;
mod subgraph;

pub use geom::flatten_cubic;
pub use query::CurveHit;
pub use scene::Scene;

/// A 2D point in scene coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }
}

impl std::ops::Add for Point {
    type Output = Point;
    fn add(self, o: Point) -> Point {
        Point::new(self.x + o.x, self.y + o.y)
    }
}

impl std::ops::Sub for Point {
    type Output = Point;
    fn sub(self, o: Point) -> Point {
        Point::new(self.x - o.x, self.y - o.y)
    }
}

impl std::ops::Mul<f64> for Point {
    type Output = Point;
    fn mul(self, s: f64) -> Point {
        Point::new(self.x * s, self.y * s)
    }
}

// uvec is paint-agnostic: it stores no colors. A fill carries an opaque `u64`
// payload (see `Scene::add_fill`) that the caller maps to whatever paint model
// it likes — solid color, gradient, image, fill rule. This keeps rendering
// concerns entirely out of the topology library.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VertexId(pub(crate) u32);
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CurveId(pub(crate) u32);
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FillId(pub(crate) u32);

/// Failure to add a fill.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillError {
    /// The point is not enclosed by curves (its face is unbounded).
    NotEnclosed,
}

/// The given id does not refer to a live object.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BadId;
