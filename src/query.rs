//! Read-only spatial queries for interactive editing: nearest vertex, nearest
//! point on a curve, the fill under a point, and the face a point sits in
//! (paint-bucket hover preview) — all without mutating the scene.
//!
//! Everything works off the same rendered polylines the topology pipeline uses,
//! so a hit is pixel-consistent with what the user sees and with where a fill or
//! split would actually land.

use crate::face::point_in_polylines;
use crate::geom::dist;
use crate::{CurveId, FillId, Point, Scene, VertexId};

/// A nearest-point-on-a-curve result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveHit {
    pub curve: CurveId,
    /// Approximate curve parameter at the hit (interpolated from the rendered
    /// polyline's tags). Good enough to `eval`/snap; not an exact projection.
    pub t: f64,
    /// The nearest point on the rendered polyline.
    pub point: Point,
    pub dist: f64,
}

/// Project `p` onto segment `a`–`b`; returns (param in 0..=1, projected point,
/// distance).
fn project(p: Point, a: Point, b: Point) -> (f64, Point, f64) {
    let (vx, vy) = (b.x - a.x, b.y - a.y);
    let len2 = vx * vx + vy * vy;
    let u = if len2 <= 1e-12 {
        0.0
    } else {
        (((p.x - a.x) * vx + (p.y - a.y) * vy) / len2).clamp(0.0, 1.0)
    };
    let proj = Point::new(a.x + vx * u, a.y + vy * u);
    (u, proj, dist(p, proj))
}

impl Scene {
    /// Nearest live vertex to `p` and its distance, or `None` if the scene has
    /// no vertices.
    pub fn nearest_vertex(&self, p: Point) -> Option<(VertexId, f64)> {
        self.vertex_ids()
            .map(|v| (v, dist(p, self.vpos(v))))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
    }

    /// Nearest live vertex within `radius`.
    pub fn nearest_vertex_within(&self, p: Point, radius: f64) -> Option<(VertexId, f64)> {
        self.nearest_vertex(p).filter(|&(_, d)| d <= radius)
    }

    /// Nearest point on any curve's rendered polyline, or `None` if the scene
    /// has no curves.
    pub fn nearest_curve(&self, p: Point) -> Option<CurveHit> {
        let mut best: Option<CurveHit> = None;
        for c in self.curve_ids() {
            let poly = self.curve_polyline_tagged(c);
            for w in poly.windows(2) {
                let ((t0, a), (t1, b)) = (w[0], w[1]);
                let (u, proj, d) = project(p, a, b);
                if best.map_or(true, |h| d < h.dist) {
                    best = Some(CurveHit {
                        curve: c,
                        t: t0 + (t1 - t0) * u,
                        point: proj,
                        dist: d,
                    });
                }
            }
        }
        best
    }

    /// Nearest point on any curve within `radius`.
    pub fn nearest_curve_within(&self, p: Point, radius: f64) -> Option<CurveHit> {
        self.nearest_curve(p).filter(|h| h.dist <= radius)
    }

    /// The fill currently covering `p` (read-only; the same point-location test
    /// `add_fill` uses to decide a recolor). `None` if `p` is in no fill.
    pub fn fill_at(&self, p: Point) -> Option<FillId> {
        self.fill_ids().find(|&id| {
            let polys = self.fill_loops_flattened(id);
            point_in_polylines(p, &polys)
        })
    }

    /// The face loops a fill at `p` *would* occupy, flattened to polylines, WITHOUT
    /// creating a fill — for paint-bucket hover preview. `None` if `p` is not
    /// enclosed (its face is unbounded), matching `add_fill`'s `NotEnclosed`.
    pub fn face_at(&self, p: Point) -> Option<Vec<Vec<Point>>> {
        self.trace_face(p).ok().map(|loops| self.fill_polys(&loops))
    }
}
