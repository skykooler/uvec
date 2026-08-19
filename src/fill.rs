//! Fill operations: add-fill-at-point and anchor-based repair.

use crate::face::point_in_polylines;
use crate::scene::FillData;
use crate::{BadId, FillError, FillId, Point, Scene};

impl Scene {
    /// Op 2: attempt to add a fill at `p`, tagged with an opaque `payload`. If
    /// `p` is already inside an existing fill, that fill's payload is updated
    /// instead (the "re-paint the region" case). Fails when the face containing
    /// `p` is unbounded. uvec never interprets `payload` — callers map it to a
    /// color / gradient / image / fill rule of their own.
    pub fn add_fill(&mut self, p: Point, payload: u64) -> Result<FillId, FillError> {
        for i in 0..self.fills.len() {
            if let Some(f) = self.fills[i].as_ref() {
                let polys = self.fill_polys(&f.loops);
                if point_in_polylines(p, &polys) {
                    self.fills[i].as_mut().unwrap().payload = payload;
                    return Ok(FillId(i as u32));
                }
            }
        }
        let loops = self.trace_face(p)?;
        self.fills.push(Some(FillData { payload, anchor: p, loops }));
        Ok(FillId(self.fills.len() as u32 - 1))
    }

    /// Op 4: delete a fill.
    pub fn delete_fill(&mut self, id: FillId) -> Result<(), BadId> {
        match self.fills.get_mut(id.0 as usize) {
            Some(slot @ Some(_)) => {
                *slot = None;
                Ok(())
            }
            _ => Err(BadId),
        }
    }

    pub fn fill_ids(&self) -> impl Iterator<Item = FillId> + '_ {
        self.fills.iter().enumerate().filter(|(_, f)| f.is_some()).map(|(i, _)| FillId(i as u32))
    }

    /// The opaque payload of a fill (see [`Scene::add_fill`]).
    pub fn fill_payload(&self, id: FillId) -> u64 {
        self.fills[id.0 as usize].as_ref().expect("dead fill id").payload
    }

    /// The distinct curves that bound a fill (outer loop and any holes),
    /// deduplicated. Useful for hit-testing and for host-side selection that
    /// wants to grow a fill selection to include its walls.
    pub fn fill_boundary_curves(&self, id: FillId) -> Vec<crate::CurveId> {
        let f = self.fills[id.0 as usize].as_ref().expect("dead fill id");
        let mut v: Vec<crate::CurveId> = f.loops.iter().flatten().map(|&(c, _)| c).collect();
        v.sort();
        v.dedup();
        v
    }

    /// The fill's boundary loops as closed polylines (outer loop first, then
    /// holes), flattened with the scene tolerance. Render with the even-odd
    /// rule over all loops together.
    pub fn fill_loops_flattened(&self, id: FillId) -> Vec<Vec<Point>> {
        let f = self.fills[id.0 as usize].as_ref().expect("dead fill id");
        self.fill_polys(&f.loops)
    }

    pub(crate) fn fill_polys(&self, loops: &[Vec<(crate::CurveId, bool)>]) -> Vec<Vec<Point>> {
        loops.iter().map(|lp| self.loop_polyline(lp)).collect()
    }

    /// Re-derive every fill's loops as the arrangement face containing its
    /// anchor; fills whose anchor leaked into the unbounded face are deleted.
    /// Called after every geometry-changing operation.
    pub(crate) fn repair_fills(&mut self) {
        for i in 0..self.fills.len() {
            let Some(anchor) = self.fills[i].as_ref().map(|f| f.anchor) else {
                continue;
            };
            match self.trace_face(anchor) {
                Ok(loops) => self.fills[i].as_mut().unwrap().loops = loops,
                Err(_) => self.fills[i] = None,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{FillError, Point, Scene};

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

    // Opaque payloads — any distinct u64s stand in for "different paint".
    const RED: u64 = 1;
    const BLUE: u64 = 2;

    #[test]
    fn fill_recolor_delete() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 0.0, 0.0, 100.0, 100.0);
        assert_eq!(s.add_fill(pt(200.0, 200.0), RED), Err(FillError::NotEnclosed));
        let f = s.add_fill(pt(50.0, 50.0), RED).unwrap();
        // Same face: recolors instead of adding.
        let f2 = s.add_fill(pt(20.0, 80.0), BLUE).unwrap();
        assert_eq!(f, f2);
        assert_eq!(s.fill_payload(f), BLUE);
        assert_eq!(s.fill_ids().count(), 1);
        s.delete_fill(f).unwrap();
        assert_eq!(s.fill_ids().count(), 0);
    }

    #[test]
    fn fill_dies_when_wall_removed() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 0.0, 0.0, 100.0, 100.0);
        let f = s.add_fill(pt(50.0, 50.0), RED).unwrap();
        let wall = s.fill_loops_flattened(f); // just to exercise the accessor
        assert_eq!(wall.len(), 1);
        let any_curve = s.curve_ids().next().unwrap();
        s.delete_curve(any_curve).unwrap();
        assert_eq!(s.fill_ids().count(), 0, "fill referenced the wall");
    }

    #[test]
    fn fill_follows_anchor_on_bisection() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 0.0, 0.0, 100.0, 100.0);
        let f = s.add_fill(pt(50.0, 25.0), RED).unwrap();
        // Bisect the square; the fill keeps the chamber holding its anchor.
        s.add_curve(line(pt(0.0, 50.0), pt(100.0, 50.0)));
        assert_eq!(s.fill_ids().count(), 1);
        let loops = s.fill_loops_flattened(f);
        assert_eq!(loops.len(), 1);
        // All boundary points of the kept chamber are in the top half.
        assert!(loops[0].iter().all(|p| p.y <= 50.5));
    }

    #[test]
    fn hole_appears_when_island_added() {
        let mut s = Scene::new();
        s.set_snap_distance(1.0);
        square(&mut s, 0.0, 0.0, 100.0, 100.0);
        let f = s.add_fill(pt(10.0, 50.0), RED).unwrap();
        square(&mut s, 40.0, 40.0, 60.0, 60.0);
        let loops = s.fill_loops_flattened(f);
        assert_eq!(loops.len(), 2, "island becomes a hole");
    }
}
