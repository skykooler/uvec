//! Scene storage and the split/snap integration pipeline.

use std::collections::{HashMap, VecDeque};

use crate::geom::{self, dist, poly_intersections, poly_self_intersections, polygon_len, self_intersection, subsegment};
use crate::{BadId, CurveId, Point, VertexId};

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Vertex {
    pub pos: Point,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct CurveData {
    pub v0: VertexId,
    pub c0: Point,
    pub c1: Point,
    pub v1: VertexId,
    /// Opaque caller payload. Rides through every split: when a curve is cut
    /// (self-intersection, crossing, or explicit split) all pieces inherit it.
    /// The library never interprets it — callers map it to stroke style/color
    /// or a tween-correspondence source id. 0 means "untagged".
    pub tag: u64,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct FillData {
    /// Opaque caller payload (paint id / gradient id / packed color — uvec never
    /// interprets it). Updated in place when a point inside the fill is
    /// re-filled.
    pub payload: u64,
    pub anchor: Point,
    pub loops: Vec<Vec<(CurveId, bool)>>,
}

/// A planar arrangement of cubic bezier curves with face fills.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Scene {
    pub(crate) verts: Vec<Option<Vertex>>,
    pub(crate) curves: Vec<Option<CurveData>>,
    pub(crate) fills: Vec<Option<FillData>>,
    pub(crate) snap: f64,
    pub(crate) tol: f64,
}

impl Default for Scene {
    fn default() -> Self {
        Self::new()
    }
}

impl Scene {
    pub fn new() -> Self {
        Scene { verts: Vec::new(), curves: Vec::new(), fills: Vec::new(), snap: 1e-3, tol: 0.1 }
    }

    /// Vertices closer than this are merged into one.
    pub fn set_snap_distance(&mut self, d: f64) {
        self.snap = d;
    }

    /// Tolerance used when flattening curves for topology queries
    /// (point-in-fill tests, face walking). Use the same value when rendering.
    pub fn set_flatten_tolerance(&mut self, tol: f64) {
        self.tol = tol;
    }

    pub fn snap_distance(&self) -> f64 {
        self.snap
    }

    pub fn flatten_tolerance(&self) -> f64 {
        self.tol
    }

    fn eps(&self) -> f64 {
        self.snap * 0.25
    }

    // ---- storage helpers ----

    pub(crate) fn alloc_vertex(&mut self, pos: Point) -> VertexId {
        self.verts.push(Some(Vertex { pos }));
        VertexId(self.verts.len() as u32 - 1)
    }

    pub(crate) fn alloc_curve(&mut self, v0: VertexId, c0: Point, c1: Point, v1: VertexId, tag: u64) -> CurveId {
        self.curves.push(Some(CurveData { v0, c0, c1, v1, tag }));
        CurveId(self.curves.len() as u32 - 1)
    }

    /// The opaque payload of a curve (see [`Scene::add_curve_tagged`]). 0 if
    /// untagged or the id is dead.
    pub fn curve_tag(&self, id: CurveId) -> u64 {
        self.curves
            .get(id.0 as usize)
            .and_then(|c| c.as_ref())
            .map_or(0, |c| c.tag)
    }

    pub(crate) fn curve(&self, id: CurveId) -> &CurveData {
        self.curves[id.0 as usize].as_ref().expect("dead curve id")
    }

    fn curve_live(&self, id: CurveId) -> bool {
        self.curves.get(id.0 as usize).map_or(false, Option::is_some)
    }

    pub(crate) fn vpos(&self, id: VertexId) -> Point {
        self.verts[id.0 as usize].as_ref().expect("dead vertex id").pos
    }

    /// Control points of a curve, endpoints resolved through its vertices.
    pub fn curve_points(&self, id: CurveId) -> [Point; 4] {
        let c = self.curve(id);
        [self.vpos(c.v0), c.c0, c.c1, self.vpos(c.v1)]
    }

    /// The curve's rendered polyline: flattened at the scene tolerance, with
    /// the portion inside a snap-radius disk around each endpoint replaced by
    /// a straight stub. Radial stubs from a shared vertex cannot cross each
    /// other, which keeps the arrangement's polylines crossing-free near
    /// junctions. All topology tests and any renderer must use this exact
    /// polyline.
    pub fn curve_polyline(&self, id: CurveId) -> Vec<Point> {
        self.curve_polyline_tagged(id).into_iter().map(|(_, p)| p).collect()
    }

    pub(crate) fn curve_polyline_tagged(&self, id: CurveId) -> Vec<(f64, Point)> {
        let mut poly = geom::flatten_cubic_tagged(&self.curve_points(id), self.tol);
        let r = self.snap * 1.5;
        // Straighten the start: drop interior points inside the disk.
        let start = poly[0].1;
        if let Some(exit) = poly.iter().position(|&(_, p)| dist(p, start) >= r) {
            if exit > 1 {
                poly.drain(1..exit);
            }
        } else {
            let last = *poly.last().unwrap();
            poly.clear();
            poly.push((0.0, start));
            poly.push(last);
            return poly;
        }
        // Straighten the end symmetrically.
        let end = poly.last().unwrap().1;
        if let Some(rev_exit) = poly.iter().rposition(|&(_, p)| dist(p, end) >= r) {
            if rev_exit + 2 < poly.len() {
                poly.drain(rev_exit + 1..poly.len() - 1);
            }
        }
        poly
    }

    pub fn vertex_pos(&self, id: VertexId) -> Point {
        self.vpos(id)
    }

    pub fn curve_ids(&self) -> impl Iterator<Item = CurveId> + '_ {
        self.curves.iter().enumerate().filter(|(_, c)| c.is_some()).map(|(i, _)| CurveId(i as u32))
    }

    pub fn vertex_ids(&self) -> impl Iterator<Item = VertexId> + '_ {
        self.verts.iter().enumerate().filter(|(_, v)| v.is_some()).map(|(i, _)| VertexId(i as u32))
    }

    pub(crate) fn incident_curves(&self, v: VertexId) -> Vec<CurveId> {
        self.curve_ids().filter(|&c| self.curve(c).v0 == v || self.curve(c).v1 == v).collect()
    }

    pub(crate) fn sweep_orphan_vertices(&mut self) {
        let mut used = vec![false; self.verts.len()];
        for c in self.curve_ids() {
            used[self.curve(c).v0 .0 as usize] = true;
            used[self.curve(c).v1 .0 as usize] = true;
        }
        for (i, v) in self.verts.iter_mut().enumerate() {
            if v.is_some() && !used[i] {
                *v = None;
            }
        }
    }

    // ---- public operations ----

    /// Op 1: add a curve. Returns the surviving pieces of the added curve
    /// (possibly empty if it was degenerate and dropped).
    pub fn add_curve(&mut self, pts: [Point; 4]) -> Vec<CurveId> {
        self.add_curve_tagged(pts, 0)
    }

    /// Op 1 with an opaque payload attached. Every surviving piece (after
    /// splitting/snapping) carries `tag`, so callers can recover which original
    /// curve a post-integration piece descends from — the basis for per-edge
    /// stroke attribution and index-free tween correspondence.
    pub fn add_curve_tagged(&mut self, pts: [Point; 4], tag: u64) -> Vec<CurveId> {
        let v0 = self.alloc_vertex(pts[0]);
        let v1 = self.alloc_vertex(pts[3]);
        let id = self.alloc_curve(v0, pts[1], pts[2], v1, tag);
        let pieces = self.integrate(vec![id], Some(id));
        self.sweep_orphan_vertices();
        self.repair_fills();
        pieces
    }

    /// Op 3: delete a curve; any fill whose boundary references it is deleted.
    pub fn delete_curve(&mut self, id: CurveId) -> Result<(), BadId> {
        if !self.curve_live(id) {
            return Err(BadId);
        }
        let doomed: Vec<usize> = self
            .fills
            .iter()
            .enumerate()
            .filter(|(_, f)| {
                f.as_ref().map_or(false, |f| {
                    f.loops.iter().flatten().any(|&(c, _)| c == id)
                })
            })
            .map(|(i, _)| i)
            .collect();
        for i in doomed {
            self.fills[i] = None;
        }
        self.curves[id.0 as usize] = None;
        self.sweep_orphan_vertices();
        self.repair_fills();
        Ok(())
    }

    /// Op 5: replace a curve with a new one. Returns the surviving pieces of
    /// the replacement.
    pub fn replace_curve(&mut self, id: CurveId, pts: [Point; 4]) -> Result<Vec<CurveId>, BadId> {
        if !self.curve_live(id) {
            return Err(BadId);
        }
        let tag = self.curve(id).tag;
        self.curves[id.0 as usize] = None;
        let v0 = self.alloc_vertex(pts[0]);
        let v1 = self.alloc_vertex(pts[3]);
        let nid = self.alloc_curve(v0, pts[1], pts[2], v1, tag);
        let pieces = self.integrate(vec![nid], Some(nid));
        self.sweep_orphan_vertices();
        self.repair_fills();
        Ok(pieces)
    }

    /// Op 6: move a vertex to new coordinates.
    pub fn move_vertex(&mut self, id: VertexId, to: Point) -> Result<(), BadId> {
        match self.verts.get_mut(id.0 as usize) {
            Some(Some(v)) => v.pos = to,
            _ => return Err(BadId),
        }
        let dirty = self.incident_curves(id);
        self.integrate(dirty, None);
        self.sweep_orphan_vertices();
        self.repair_fills();
        Ok(())
    }

    /// Move a vertex WITHOUT re-planarizing. Cheap, and leaves the arrangement
    /// temporarily non-planar (curves may cross, vertices may coincide).
    /// Intended for live drag preview: render the raw curves each frame, then
    /// call [`Scene::commit`] once on edit-commit (mouse-up) to restore the
    /// split/snap invariants. Topology queries (fills, face walking, point
    /// location) are NOT valid between a raw move and the next commit.
    pub fn set_vertex_position(&mut self, id: VertexId, to: Point) -> Result<(), BadId> {
        match self.verts.get_mut(id.0 as usize) {
            Some(Some(v)) => {
                v.pos = to;
                Ok(())
            }
            _ => Err(BadId),
        }
    }

    /// Re-planarize the whole scene: run the split / snap / dedup pipeline over
    /// every live curve, then re-derive fills. Call once after a batch of
    /// [`Scene::set_vertex_position`] edits (edit-commit) — not per drag frame.
    /// Payloads ([`Scene::curve_tag`]) survive: any curve split during commit
    /// leaves both pieces carrying the moved curve's tag.
    pub fn commit(&mut self) {
        let all: Vec<CurveId> = self.curve_ids().collect();
        self.integrate(all, None);
        self.sweep_orphan_vertices();
        self.repair_fills();
    }

    // ---- the rules pipeline ----

    /// Restore the no-self-intersection / no-crossing / snapped invariants for
    /// the given dirty curves (and everything they touch). Returns the live
    /// descendants of `track`.
    pub(crate) fn integrate(&mut self, dirty: Vec<CurveId>, track: Option<CurveId>) -> Vec<CurveId> {
        let mut work: VecDeque<CurveId> = dirty.into();
        // Maps every curve created by a split back to its original ancestor.
        let mut root: HashMap<CurveId, CurveId> = HashMap::new();
        let mut iters = 0usize;
        while let Some(cid) = work.pop_front() {
            iters += 1;
            assert!(iters < 100_000, "integrate did not terminate");
            if !self.curve_live(cid) {
                continue;
            }
            let cd = self.curve(cid);
            let (cv0, cv1) = (cd.v0, cd.v1);
            let pts = self.curve_points(cid);

            // Rule 0: drop degenerate loops — both endpoints merged and either
            // tiny, or rendering to nothing (a loop that never leaves the
            // straightening disk collapses to a dot).
            if cv0 == cv1
                && (polygon_len(&pts) < self.snap || self.curve_polyline_tagged(cid).len() < 3)
            {
                self.curves[cid.0 as usize] = None;
                continue;
            }

            // Rule 0.5: drop near-coincident duplicate edges (same endpoints,
            // polylines within eps of each other) — they arise from welding
            // near-parallel curves and would tie in angular ordering.
            if self.drop_duplicate_of(cid, &mut work) {
                continue;
            }

            // Rule 1: curves may not self-intersect. The exact double point
            // comes from the closed form; hairpin near-misses and stub-induced
            // crossings of the rendered polyline are caught by the polyline
            // check and welded to the nearest endpoint.
            let self_hit = self_intersection(&pts).map(|(t1, t2)| (t1, t2, geom::eval(&pts, t1)));
            let poly_hit = if self_hit.is_none() {
                poly_self_intersections(&self.curve_polyline_tagged(cid), self.eps())
                    .into_iter()
                    .next()
            } else {
                None
            };
            if let Some((t1, t2, p)) = self_hit.or(poly_hit) {
                let near_v0 = dist(p, self.vpos(cv0)) < self.snap;
                let near_v1 = dist(p, self.vpos(cv1)) < self.snap;
                if near_v0 || near_v1 {
                    // The loop closes at (essentially) an endpoint: split once
                    // so the loop shares that endpoint's vertex.
                    let (vend, t) = if near_v0 { (cv0, t2) } else { (cv1, t1) };
                    if t > 1e-4 && t < 1.0 - 1e-4 {
                        let (a, b) = self.split_curve_at(cid, t, vend, &mut root);
                        work.push_back(a);
                        work.push_back(b);
                        continue;
                    }
                    // Degenerate parameter: nothing sensible to split.
                } else {
                    let vm = self.alloc_vertex(p);
                    let tag = self.curve(cid).tag;
                    let segs = [
                        (cv0, subsegment(&pts, 0.0, t1), vm),
                        (vm, subsegment(&pts, t1, t2), vm),
                        (vm, subsegment(&pts, t2, 1.0), cv1),
                    ];
                    self.curves[cid.0 as usize] = None;
                    let r = *root.get(&cid).unwrap_or(&cid);
                    for (a, s, b) in segs {
                        let nid = self.alloc_curve(a, s[1], s[2], b, tag);
                        root.insert(nid, r);
                        work.push_back(nid);
                    }
                    continue;
                }
            }

            // Rule 2: curves may not cross other curves.
            if self.cross_split(cid, &mut root, &mut work) {
                continue;
            }

            // Rule 3: snapping.
            if self.snap_endpoints(cid, &mut work) {
                continue;
            }
        }
        match track {
            Some(t) => self
                .curve_ids()
                .filter(|&c| c == t || root.get(&c) == Some(&t))
                .collect(),
            None => Vec::new(),
        }
    }

    /// If `cid` duplicates another live curve (same endpoint vertices, whole
    /// polyline within eps both ways), delete the higher-id one. Returns true
    /// if `cid` itself was deleted.
    fn drop_duplicate_of(&mut self, cid: CurveId, work: &mut VecDeque<CurveId>) -> bool {
        let (cv0, cv1) = {
            let c = self.curve(cid);
            (c.v0, c.v1)
        };
        let eps = self.eps();
        let pa: Vec<Point> = self.curve_polyline(cid);
        let others: Vec<CurveId> = self
            .curve_ids()
            .filter(|&o| o != cid)
            .filter(|&o| {
                let od = self.curve(o);
                (od.v0 == cv0 && od.v1 == cv1) || (od.v0 == cv1 && od.v1 == cv0)
            })
            .collect();
        for oid in others {
            let pb = self.curve_polyline(oid);
            let near = |poly: &[Point], other: &[Point]| {
                poly.iter().all(|&p| {
                    other.windows(2).any(|s| geom::seg_dist(p, s[0], s[1]) < eps)
                })
            };
            if near(&pa, &pb) && near(&pb, &pa) {
                let doomed = if cid.0 > oid.0 { cid } else { oid };
                self.curves[doomed.0 as usize] = None;
                if doomed == cid {
                    return true;
                }
                work.push_back(cid);
            }
        }
        false
    }

    /// Find and resolve the first crossing (or sub-eps near-miss) between
    /// `cid`'s rendered polyline and any other curve's. Returns true if a
    /// split happened (in which case `cid` may be dead).
    fn cross_split(
        &mut self,
        cid: CurveId,
        root: &mut HashMap<CurveId, CurveId>,
        work: &mut VecDeque<CurveId>,
    ) -> bool {
        let others: Vec<CurveId> = self.curve_ids().filter(|&o| o != cid).collect();
        let (cv0, cv1) = {
            let c = self.curve(cid);
            (c.v0, c.v1)
        };
        let pa = self.curve_polyline_tagged(cid);
        for oid in others {
            let pb = self.curve_polyline_tagged(oid);
            let (ov0, ov1) = {
                let o = self.curve(oid);
                (o.v0, o.v1)
            };
            for (t, u, p) in poly_intersections(&pa, &pb, self.eps()) {
                // A crossing counts as "at an endpoint" only if it lies on
                // that curve's straightened stub segment AND within snap of
                // the vertex — a mid-body pass close to some vertex is a real
                // crossing that must be split.
                let ((cs0, cs1), (os0, os1)) = (stub_side(&pa, t), stub_side(&pb, u));
                let near_c0 = cs0 && dist(p, self.vpos(cv0)) < self.snap;
                let near_c1 = cs1 && dist(p, self.vpos(cv1)) < self.snap;
                let near_o0 = os0 && dist(p, self.vpos(ov0)) < self.snap;
                let near_o1 = os1 && dist(p, self.vpos(ov1)) < self.snap;
                let near_c = near_c0 || near_c1;
                let near_o = near_o0 || near_o1;
                if trace_enabled() {
                    eprintln!(
                        "split: c={cid:?} o={oid:?} t={t:.4} u={u:.4} p=({:.3},{:.3}) near_c={near_c} near_o={near_o}",
                        p.x, p.y
                    );
                }
                match (near_c, near_o) {
                    (true, true) => {
                        // Conflict inside the snap neighborhood of both
                        // curves' endpoints. Straight stubs from one shared
                        // vertex cannot truly cross (only graze within eps) —
                        // skip those. Stubs of two distinct nearby vertices
                        // in conflict: merge the vertices (strictly decreases
                        // the vertex count, so this always converges); the
                        // stubs then radiate from one vertex and cannot
                        // cross.
                        let vc = if near_c0 { cv0 } else { cv1 };
                        let vo = if near_o0 { ov0 } else { ov1 };
                        if vc == vo {
                            continue;
                        }
                        let keep = if dist(p, self.vpos(vc)) <= dist(p, self.vpos(vo)) { vc } else { vo };
                        let lose = if keep == vc { vo } else { vc };
                        self.merge_vertices(keep, lose, work);
                        work.push_back(cid);
                        return true;
                    }
                    (true, false) => {
                        // T-junction: c's endpoint lies on o. Split o there,
                        // welding it to c's endpoint vertex.
                        let vend = if near_c0 { cv0 } else { cv1 };
                        let (a, b) = self.split_curve_at(oid, u, vend, root);
                        work.push_back(a);
                        work.push_back(b);
                        work.push_back(cid);
                        return true;
                    }
                    (false, true) => {
                        let vend = if near_o0 { ov0 } else { ov1 };
                        let (a, b) = self.split_curve_at(cid, t, vend, root);
                        work.push_back(a);
                        work.push_back(b);
                        work.push_back(oid);
                        return true;
                    }
                    (false, false) => {
                        let vm = self.alloc_vertex(p);
                        let (a, b) = self.split_curve_at(cid, t, vm, root);
                        let (c, d) = self.split_curve_at(oid, u, vm, root);
                        for id in [a, b, c, d] {
                            work.push_back(id);
                        }
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Split a curve at parameter `t`, joining the two pieces at `vm`.
    fn split_curve_at(
        &mut self,
        id: CurveId,
        t: f64,
        vm: VertexId,
        root: &mut HashMap<CurveId, CurveId>,
    ) -> (CurveId, CurveId) {
        let pts = self.curve_points(id);
        let (v0, v1, tag) = {
            let c = self.curve(id);
            (c.v0, c.v1, c.tag)
        };
        let (l, r) = geom::split(&pts, t);
        self.curves[id.0 as usize] = None;
        let rt = *root.get(&id).unwrap_or(&id);
        let a = self.alloc_curve(v0, l[1], l[2], vm, tag);
        let b = self.alloc_curve(vm, r[1], r[2], v1, tag);
        root.insert(a, rt);
        root.insert(b, rt);
        (a, b)
    }

    /// Merge any vertex of `cid` with a nearby other vertex. Returns true if a
    /// merge happened (affected curves, including `cid`, are re-queued).
    fn snap_endpoints(&mut self, cid: CurveId, work: &mut VecDeque<CurveId>) -> bool {
        let (cv0, cv1) = {
            let c = self.curve(cid);
            (c.v0, c.v1)
        };
        for v in [cv0, cv1] {
            let vp = self.vpos(v);
            let hit = self
                .vertex_ids()
                .filter(|&w| w != v)
                .find(|&w| dist(vp, self.vpos(w)) < self.snap);
            if let Some(w) = hit {
                let (keep, lose) = if v.0 < w.0 { (v, w) } else { (w, v) };
                self.merge_vertices(keep, lose, work);
                return true;
            }
        }
        false
    }

    /// Merge `lose` into `keep` (keeping `keep`'s position), re-queueing every
    /// affected curve.
    fn merge_vertices(&mut self, keep: VertexId, lose: VertexId, work: &mut VecDeque<CurveId>) {
        for i in 0..self.curves.len() {
            if let Some(c) = self.curves[i].as_mut() {
                let mut touched = false;
                if c.v0 == lose {
                    c.v0 = keep;
                    touched = true;
                }
                if c.v1 == lose {
                    c.v1 = keep;
                    touched = true;
                }
                if touched {
                    work.push_back(CurveId(i as u32));
                }
            }
        }
        // Curves at `keep` also changed neighborhood; recheck them.
        for c in self.incident_curves(keep) {
            work.push_back(c);
        }
        self.verts[lose.0 as usize] = None;
    }
}

#[cfg(test)]
fn lmid(a: Point, b: Point) -> Point {
    Point::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5)
}

/// Whether a parameter lies on the straightened start/end stub segment of a
/// tagged polyline. Both can hold for short (fully straightened) curves.
fn stub_side(poly: &[(f64, Point)], t: f64) -> (bool, bool) {
    if poly.len() < 2 {
        return (true, true);
    }
    (t <= poly[1].0, t >= poly[poly.len() - 2].0)
}

/// Diagnostic tracing for the pipeline and face walk (UVEC_TRACE=1).
pub(crate) fn trace_enabled() -> bool {
    static FLAG: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *FLAG.get_or_init(|| std::env::var("UVEC_TRACE").is_ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    fn line(a: Point, b: Point) -> [Point; 4] {
        [a, lmid(a, lmid(a, b)), lmid(lmid(a, b), b), b]
    }

    fn scene() -> Scene {
        let mut s = Scene::new();
        s.set_snap_distance(2.0);
        s
    }

    #[test]
    fn x_crossing_splits_both() {
        let mut s = scene();
        let a = s.add_curve(line(pt(0.0, 0.0), pt(100.0, 100.0)));
        assert_eq!(a.len(), 1);
        let b = s.add_curve(line(pt(0.0, 100.0), pt(100.0, 0.0)));
        assert_eq!(b.len(), 2, "new curve should split into two pieces");
        assert_eq!(s.curve_ids().count(), 4);
        assert_eq!(s.vertex_ids().count(), 5);
    }

    #[test]
    fn self_loop_splits_into_three() {
        let mut s = scene();
        let pieces = s.add_curve([pt(0.0, 0.0), pt(150.0, 100.0), pt(-50.0, 100.0), pt(100.0, 0.0)]);
        assert_eq!(pieces.len(), 3);
        assert_eq!(s.curve_ids().count(), 3);
        // One of the pieces is a closed loop.
        assert_eq!(
            s.curve_ids().filter(|&c| s.curve(c).v0 == s.curve(c).v1).count(),
            1
        );
    }

    #[test]
    fn shared_endpoint_snaps() {
        let mut s = scene();
        s.add_curve(line(pt(0.0, 0.0), pt(50.0, 0.0)));
        s.add_curve(line(pt(50.5, 0.5), pt(100.0, 50.0)));
        assert_eq!(s.curve_ids().count(), 2);
        assert_eq!(s.vertex_ids().count(), 3, "close endpoints should merge");
    }

    #[test]
    fn t_junction_splits_crossed_curve() {
        let mut s = scene();
        s.add_curve(line(pt(0.0, 0.0), pt(100.0, 0.0)));
        // Endpoint lands exactly on the middle of the first curve.
        s.add_curve(line(pt(50.0, 0.0), pt(50.0, 80.0)));
        assert_eq!(s.curve_ids().count(), 3);
        assert_eq!(s.vertex_ids().count(), 4);
    }

    #[test]
    fn degenerate_curve_dropped() {
        let mut s = scene();
        let pieces = s.add_curve(line(pt(10.0, 10.0), pt(10.5, 10.2)));
        assert!(pieces.is_empty());
        assert_eq!(s.curve_ids().count(), 0);
        assert_eq!(s.vertex_ids().count(), 0);
    }
}
