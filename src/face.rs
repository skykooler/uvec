//! Point location and face walking over the arrangement.
//!
//! All geometry here operates on the flattened polylines of curves (shared
//! flattening with rendering), so topology decisions match what a renderer
//! sees. Orientation is algebraic: cross((x1,y1),(x2,y2)) = x1*y2 - y1*x2;
//! a face traced with the region on the algebraic left has positive shoelace
//! area iff it is bounded.

use std::collections::{HashMap, HashSet, VecDeque};

use crate::{CurveId, FillError, Point, Scene, VertexId};

pub(crate) type HalfEdge = (CurveId, bool); // bool: traverse reversed (v1 -> v0)

impl Scene {
    /// Rendered polyline of a half-edge, in traversal order.
    pub(crate) fn he_poly(&self, he: HalfEdge) -> Vec<Point> {
        let mut poly = self.curve_polyline(he.0);
        if he.1 {
            poly.reverse();
        }
        poly
    }

    fn he_end(&self, he: HalfEdge) -> VertexId {
        let c = self.curve(he.0);
        if he.1 { c.v0 } else { c.v1 }
    }

    /// Direction of the first polyline segment leaving the start vertex.
    /// Polylines are straightened within the snap disk of each endpoint, so
    /// this is the tangle-free exit direction from the vertex.
    fn he_out_dir(&self, he: HalfEdge) -> Point {
        let poly = self.he_poly(he);
        let a = poly[0];
        let b = poly.iter().copied().find(|&p| p != a).unwrap_or(a);
        b - a
    }

    fn outgoing(&self, v: VertexId) -> Vec<HalfEdge> {
        let mut out = Vec::new();
        for c in self.curve_ids() {
            let cd = self.curve(c);
            if cd.v0 == v {
                out.push((c, false));
            }
            if cd.v1 == v {
                out.push((c, true));
            }
        }
        out
    }

    /// Outgoing half-edges at `v` in cyclic CCW order by exit angle. At equal
    /// angles, (curve, true) sorts before (curve, false): a zero-width spike
    /// loop then embeds so that walks pass it like a spur and its zero-area
    /// interior becomes a self-cycle.
    fn ordered_outgoing(&self, v: VertexId) -> Vec<(f64, HalfEdge)> {
        let mut cands: Vec<(f64, HalfEdge)> = self
            .outgoing(v)
            .into_iter()
            .map(|he| {
                let d = self.he_out_dir(he);
                (d.y.atan2(d.x), he)
            })
            .collect();
        cands.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap()
                .then((a.1 .0).cmp(&b.1 .0))
                .then((b.1 .1).cmp(&a.1 .1)) // true before false
        });
        cands
    }

    /// Next half-edge of the face lying on the algebraic left after arriving
    /// at `v`: the CCW-predecessor of the twin half-edge in the cyclic order.
    fn next_half_edge(&self, v: VertexId, twin: HalfEdge) -> Option<HalfEdge> {
        let cands = self.ordered_outgoing(v);
        let idx = cands.iter().position(|&(_, he)| he == twin)?;
        let n = cands.len();
        Some(cands[(idx + n - 1) % n].1)
    }

    /// Like `next_half_edge` but for a virtual arrival from direction +x
    /// (used to start hole walks at a component's rightmost vertex).
    fn first_half_edge_from_plus_x(&self, v: VertexId) -> Option<HalfEdge> {
        let cands = self.ordered_outgoing(v);
        if cands.is_empty() {
            return None;
        }
        let pos = cands.iter().position(|&(a, _)| a >= 0.0).unwrap_or(0);
        let n = cands.len();
        Some(cands[(pos + n - 1) % n].1)
    }

    /// Walk the face boundary starting from `start` (face on the algebraic
    /// left), returning the closed sequence of half-edges.
    fn walk_face(&self, start: HalfEdge) -> Option<Vec<HalfEdge>> {
        let cap = 4 * self.curve_ids().count() + 8;
        let mut visited: HashSet<HalfEdge> = HashSet::from([start]);
        let mut hes = vec![start];
        let mut cur = start;
        let trace = {
            static FLAG: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
            *FLAG.get_or_init(|| std::env::var("UVEC_TRACE_WALK").is_ok())
        };
        loop {
            let v = self.he_end(cur);
            let next = self.next_half_edge(v, (cur.0, !cur.1))?;
            if trace {
                let vp = self.vpos(v);
                let cands: Vec<String> = self
                    .ordered_outgoing(v)
                    .iter()
                    .map(|&(a, he)| format!("{he:?}@{a:.3}"))
                    .collect();
                eprintln!(
                    "walk: at {:?}({:.2},{:.2}) cur={cur:?} cands={cands:?} -> {next:?}",
                    v, vp.x, vp.y
                );
            }
            if next == start {
                return Some(hes);
            }
            if !visited.insert(next) {
                return None; // inconsistent turn function (should not happen)
            }
            hes.push(next);
            if hes.len() > cap {
                return None;
            }
            cur = next;
        }
    }

    pub(crate) fn loop_polyline(&self, lp: &[(CurveId, bool)]) -> Vec<Point> {
        let mut out: Vec<Point> = Vec::new();
        for &he in lp {
            let poly = self.he_poly(he);
            let skip = usize::from(!out.is_empty());
            out.extend(poly.into_iter().skip(skip));
        }
        out
    }

    /// Trace the face containing `p`: outer loop plus hole loops.
    pub(crate) fn trace_face(&self, p: Point) -> Result<Vec<Vec<(CurveId, bool)>>, FillError> {
        // March the +x ray outward. A negative-area walk means we hit a hole
        // component (of p's face or a nested blob) — record it and keep going
        // past its other crossings. The first positive-area walk is the outer
        // boundary of p's face; exhausting all crossings means p is unbounded.
        let mut skip: HashSet<CurveId> = HashSet::new();
        let mut outer: Option<Vec<HalfEdge>> = None;
        for (_, he) in self.raycast_crossings(p) {
            if skip.contains(&he.0) {
                continue;
            }
            let Some(lp) = self.walk_face(he) else {
                if crate::scene::trace_enabled() {
                    eprintln!("march: he={he:?} WALK FAILED");
                }
                skip.insert(he.0); // pathological walk: ignore this crossing
                continue;
            };
            let poly = self.loop_polyline(&lp);
            if crate::scene::trace_enabled() {
                eprintln!(
                    "march: he={he:?} loop_len={} area={:.4} contains={}",
                    lp.len(),
                    signed_area(&poly),
                    point_in_polyline(p, &poly)
                );
            }
            // The outer boundary of p's face must have positive area (with
            // margin: an out-and-back walk over a dangling curve cancels to
            // ~0 up to float noise) and must actually contain p — near
            // vertices, residual polyline crossings can make a walk trace a
            // loop that excludes p (p sits in a sliver that is really part of
            // another face).
            if signed_area(&poly) > 1e-6 && point_in_polyline(p, &poly) {
                outer = Some(lp);
                break;
            }
            skip.extend(lp.iter().map(|&(c, _)| c));
        }
        let outer = outer.ok_or(FillError::NotEnclosed)?;
        let outer_poly = self.loop_polyline(&outer);

        // Vertices on the outer boundary.
        let mut outer_curves: HashSet<CurveId> = HashSet::new();
        let mut outer_verts: HashSet<VertexId> = HashSet::new();
        for &(c, _) in &outer {
            outer_curves.insert(c);
            let cd = self.curve(c);
            outer_verts.insert(cd.v0);
            outer_verts.insert(cd.v1);
        }

        // Connected components of the remaining curves; interior ones are holes.
        let comps = self.components(&outer_curves);
        let mut ordered: Vec<(Point, VertexId, Vec<CurveId>)> = Vec::new();
        for comp in comps {
            // Rightmost vertex of the component (walk start), plus an
            // inside-test vertex that is not on the outer boundary.
            let mut verts: HashSet<VertexId> = HashSet::new();
            for &c in &comp {
                verts.insert(self.curve(c).v0);
                verts.insert(self.curve(c).v1);
            }
            let rightmost = |ids: &mut dyn Iterator<Item = VertexId>| {
                ids.max_by(|a, b| {
                    let (pa, pb) = (self.vpos(*a), self.vpos(*b));
                    pa.x
                        .partial_cmp(&pb.x)
                        .unwrap()
                        .then(pa.y.partial_cmp(&pb.y).unwrap())
                        .then(a.cmp(b))
                })
            };
            let Some(test) = rightmost(&mut verts.iter().copied().filter(|v| !outer_verts.contains(v)))
            else {
                continue; // fully attached to the boundary => outside the face
            };
            let vr = rightmost(&mut verts.iter().copied()).unwrap();
            ordered.push((self.vpos(test), vr, comp));
        }
        // Outermost first so nested components are excluded by earlier holes.
        ordered.sort_by(|a, b| {
            let (ra, rb) = (self.vpos(a.1).x, self.vpos(b.1).x);
            rb.partial_cmp(&ra).unwrap()
        });

        let mut loops = vec![outer];
        let mut hole_polys: Vec<Vec<Point>> = Vec::new();
        for (test_pt, vr, _comp) in ordered {
            if !point_in_polyline(test_pt, &outer_poly) {
                continue;
            }
            if hole_polys.iter().any(|hp| point_in_polyline(test_pt, hp)) {
                continue;
            }
            // Walk as if arriving at the rightmost vertex from +x.
            let Some(first) = self.first_half_edge_from_plus_x(vr) else {
                continue;
            };
            let Some(hole) = self.walk_face(first) else {
                continue;
            };
            hole_polys.push(self.loop_polyline(&hole));
            loops.push(hole);
        }
        Ok(loops)
    }

    /// All crossings of the +x ray from `p`, sorted by x, each oriented so the
    /// side containing `p` locally is on the algebraic left.
    fn raycast_crossings(&self, p: Point) -> Vec<(f64, HalfEdge)> {
        let mut hits: Vec<(f64, HalfEdge)> = Vec::new();
        for c in self.curve_ids() {
            let poly = self.he_poly((c, false));
            for seg in poly.windows(2) {
                if let Some((x, down)) = ray_hit(p, seg[0], seg[1]) {
                    hits.push((x, (c, !down)));
                }
            }
        }
        hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        hits
    }

    /// Connected components (via shared vertices) of live curves not in `excl`.
    fn components(&self, excl: &HashSet<CurveId>) -> Vec<Vec<CurveId>> {
        let rest: Vec<CurveId> = self.curve_ids().filter(|c| !excl.contains(c)).collect();
        let mut by_vert: HashMap<VertexId, Vec<CurveId>> = HashMap::new();
        for &c in &rest {
            let cd = self.curve(c);
            by_vert.entry(cd.v0).or_default().push(c);
            by_vert.entry(cd.v1).or_default().push(c);
        }
        let mut seen: HashSet<CurveId> = HashSet::new();
        let mut comps = Vec::new();
        for &c in &rest {
            if seen.contains(&c) {
                continue;
            }
            let mut comp = Vec::new();
            let mut q = VecDeque::from([c]);
            seen.insert(c);
            while let Some(cur) = q.pop_front() {
                comp.push(cur);
                let cd = self.curve(cur);
                for v in [cd.v0, cd.v1] {
                    for &n in by_vert.get(&v).into_iter().flatten() {
                        if seen.insert(n) {
                            q.push_back(n);
                        }
                    }
                }
            }
            comps.push(comp);
        }
        comps
    }
}

/// Crossing of the +x ray from `p` with segment a->b, using the half-open rule
/// (a.y <= p.y < b.y counts as a downward crossing, the mirror as upward).
/// Returns (crossing_x, downward) for crossings strictly right of p.
fn ray_hit(p: Point, a: Point, b: Point) -> Option<(f64, bool)> {
    let down = a.y <= p.y && p.y < b.y;
    let up = b.y <= p.y && p.y < a.y;
    if !down && !up {
        return None;
    }
    let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
    if x > p.x + 1e-9 {
        Some((x, down))
    } else {
        None
    }
}

/// Even-odd point-in-polygon test over a closed polyline.
pub(crate) fn point_in_polyline(p: Point, poly: &[Point]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if ray_hit(p, a, b).is_some() {
            inside = !inside;
        }
    }
    inside
}

pub(crate) fn point_in_polylines(p: Point, polys: &[Vec<Point>]) -> bool {
    let mut crossings = 0usize;
    for poly in polys {
        let n = poly.len();
        for i in 0..n {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            if ray_hit(p, a, b).is_some() {
                crossings += 1;
            }
        }
    }
    crossings % 2 == 1
}

/// Shoelace area (algebraic; positive means the enclosed face is bounded when
/// walked with the face on the algebraic left).
pub(crate) fn signed_area(poly: &[Point]) -> f64 {
    let mut s = 0.0;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        s += a.x * b.y - b.x * a.y;
    }
    s * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    fn line(a: Point, b: Point) -> [Point; 4] {
        let m1 = Point::new(a.x + (b.x - a.x) / 3.0, a.y + (b.y - a.y) / 3.0);
        let m2 = Point::new(a.x + (b.x - a.x) * 2.0 / 3.0, a.y + (b.y - a.y) * 2.0 / 3.0);
        [a, m1, m2, b]
    }

    fn square(s: &mut Scene, x0: f64, y0: f64, x1: f64, y1: f64) {
        s.add_curve(line(pt(x0, y0), pt(x1, y0)));
        s.add_curve(line(pt(x1, y0), pt(x1, y1)));
        s.add_curve(line(pt(x1, y1), pt(x0, y1)));
        s.add_curve(line(pt(x0, y1), pt(x0, y0)));
    }

    #[test]
    fn square_face() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 10.0, 10.0, 110.0, 110.0);
        let loops = s.trace_face(pt(60.0, 60.0)).expect("interior should be enclosed");
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].len(), 4);
        assert!(s.trace_face(pt(5.0, 5.0)).is_err(), "outside is unbounded");
    }

    #[test]
    fn square_with_hole() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 0.0, 0.0, 100.0, 100.0);
        square(&mut s, 40.0, 40.0, 60.0, 60.0);
        let loops = s.trace_face(pt(10.0, 50.0)).expect("ring should be enclosed");
        assert_eq!(loops.len(), 2, "outer loop plus one hole");
        // Inside the inner square is a separate face with no hole.
        let inner = s.trace_face(pt(50.0, 50.0)).unwrap();
        assert_eq!(inner.len(), 1);
    }

    #[test]
    fn theta_chambers() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 0.0, 0.0, 100.0, 100.0);
        // Horizontal chord across the middle.
        s.add_curve(line(pt(0.0, 50.0), pt(100.0, 50.0)));
        let top = s.trace_face(pt(50.0, 25.0)).unwrap();
        let bottom = s.trace_face(pt(50.0, 75.0)).unwrap();
        assert_eq!(top.len(), 1);
        assert_eq!(bottom.len(), 1);
        assert_eq!(top[0].len(), 4);
        assert_eq!(bottom[0].len(), 4);
    }
}

#[cfg(test)]
mod dbgtests {
    use super::*;

    #[test]
    fn open_arc_not_enclosed() {
        let mut s = Scene::new();
        s.set_snap_distance(2.0);
        s.set_flatten_tolerance(0.1);
        s.add_curve([
            Point::new(65.64830340614216, 119.0),
            Point::new(52.93828336926242, 58.45697201306852),
            Point::new(92.0, 28.883502183107645),
            Point::new(57.359309409081575, 9.0),
        ]);
        let p = Point::new(61.8125, 38.3125);
        match s.trace_face(p) {
            Ok(loops) => {
                for lp in &loops {
                    let poly = s.loop_polyline(lp);
                    eprintln!("loop: {} hes, area {}", lp.len(), signed_area(&poly));
                    eprintln!("hes: {:?}", lp);
                }
                panic!("open arc must not enclose anything");
            }
            Err(e) => eprintln!("ok: {e:?}"),
        }
    }
}
