//! Read-only editing queries, checked against independent ground truth: a
//! brute-force nearest scan for vertices/curves, and a raster point-in-face test
//! for fill_at / face_at.

mod common;

use common::*;
use rand::{rngs::StdRng, Rng, SeedableRng};
use uvec::{Point, Scene};

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

/// nearest_vertex agrees with a brute-force scan over every live vertex.
#[test]
fn nearest_vertex_matches_bruteforce() {
    let mut rng = StdRng::seed_from_u64(0x1111);
    let mut s = Scene::new();
    s.set_snap_distance(1.0);
    for _ in 0..12 {
        let a = pt(rng.gen_range(0.0..100.0), rng.gen_range(0.0..100.0));
        let b = pt(rng.gen_range(0.0..100.0), rng.gen_range(0.0..100.0));
        s.add_curve(line(a, b));
    }
    for _ in 0..200 {
        let p = pt(rng.gen_range(-20.0..120.0), rng.gen_range(-20.0..120.0));
        let got = s.nearest_vertex(p);
        let brute = s
            .vertex_ids()
            .map(|v| (v, s.vertex_pos(v)))
            .map(|(v, q)| (v, ((q.x - p.x).powi(2) + (q.y - p.y).powi(2)).sqrt()))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        match (got, brute) {
            (Some((_, dg)), Some((_, db))) => {
                assert!((dg - db).abs() < 1e-9, "nearest_vertex dist {dg} != brute {db}")
            }
            (None, None) => {}
            _ => panic!("nearest_vertex disagreed on presence"),
        }
    }
}

/// nearest_curve never reports a distance greater than the distance to the
/// nearest vertex (a vertex lies on a curve), and its reported point is actually
/// that far from `p` — i.e. the hit is self-consistent and no closer curve was
/// missed relative to the coarse vertex bound.
#[test]
fn nearest_curve_is_consistent_and_tight() {
    let mut rng = StdRng::seed_from_u64(0x2222);
    let mut s = Scene::new();
    s.set_snap_distance(1.0);
    for _ in 0..10 {
        let a = pt(rng.gen_range(10.0..90.0), rng.gen_range(10.0..90.0));
        let b = pt(rng.gen_range(10.0..90.0), rng.gen_range(10.0..90.0));
        s.add_curve(line(a, b));
    }
    for _ in 0..200 {
        let p = pt(rng.gen_range(0.0..100.0), rng.gen_range(0.0..100.0));
        let hit = s.nearest_curve(p).expect("curves exist");
        // reported point really is `dist` from p
        let d_pt = ((hit.point.x - p.x).powi(2) + (hit.point.y - p.y).powi(2)).sqrt();
        assert!((d_pt - hit.dist).abs() < 1e-9, "hit.point/dist inconsistent");
        // and no vertex is closer than the curve hit (vertices lie ON curves)
        if let Some((_, dv)) = s.nearest_vertex(p) {
            assert!(hit.dist <= dv + 1e-9, "a vertex ({dv}) beat the nearest curve ({})", hit.dist);
        }
        // brute-force min over all polyline segments must match
        let mut brute = f64::INFINITY;
        for c in s.curve_ids() {
            let poly = s.curve_polyline(c);
            for w in poly.windows(2) {
                brute = brute.min(seg_dist_pt(p, w[0], w[1]));
            }
        }
        assert!((hit.dist - brute).abs() < 1e-9, "nearest_curve {} != brute {brute}", hit.dist);
    }
}

/// fill_at / face_at agree with a raster point-in-face oracle on a fixed scene.
#[test]
fn fill_and_face_at_match_raster() {
    const RS: f64 = 6.0;
    const W: usize = 768;
    const H: usize = 768;

    let mut s = Scene::new();
    s.set_snap_distance(1.0);
    square(&mut s, 10.0, 10.0, 50.0, 50.0);
    let fid = s.add_fill(pt(30.0, 30.0), 0xABCD).unwrap();
    // an unfilled but enclosed box, and open space elsewhere
    square(&mut s, 70.0, 70.0, 110.0, 110.0);

    // Raster oracle: stroke all curves, then for a query point flood from it and
    // see if it reaches the border (unbounded => not enclosed).
    let oracle_enclosed = |p: Point| -> bool {
        let mut c = Canvas::new(W, H, WHITE);
        for cid in s.curve_ids() {
            let poly: Vec<Point> =
                s.curve_polyline(cid).iter().map(|q| Point::new(q.x * RS, q.y * RS)).collect();
            stroke_polyline(&mut c, &poly, BLACK);
        }
        let (px, py) = ((p.x * RS) as i64, (p.y * RS) as i64);
        if c.get(px, py) != Some(WHITE) {
            return true; // on a wall; treat as enclosed-ish, not tested below
        }
        !flood_fill(&mut c, px, py, WHITE, RED)
    };

    // Inside the filled box: fill_at finds it, face_at is Some.
    assert_eq!(s.fill_at(pt(30.0, 30.0)), Some(fid));
    assert!(s.face_at(pt(30.0, 30.0)).is_some());
    assert!(oracle_enclosed(pt(30.0, 30.0)));

    // Inside the unfilled-but-enclosed box: no fill, but face_at previews it.
    assert_eq!(s.fill_at(pt(90.0, 90.0)), None);
    assert!(s.face_at(pt(90.0, 90.0)).is_some());
    assert!(oracle_enclosed(pt(90.0, 90.0)));

    // Open space: no fill, no face, oracle agrees it escapes.
    assert_eq!(s.fill_at(pt(120.0, 10.0)), None);
    assert!(s.face_at(pt(120.0, 10.0)).is_none());
    assert!(!oracle_enclosed(pt(120.0, 10.0)));
}

fn seg_dist_pt(p: Point, a: Point, b: Point) -> f64 {
    let (vx, vy) = (b.x - a.x, b.y - a.y);
    let len2 = vx * vx + vy * vy;
    let u = if len2 <= 1e-12 {
        0.0
    } else {
        (((p.x - a.x) * vx + (p.y - a.y) * vy) / len2).clamp(0.0, 1.0)
    };
    let proj = Point::new(a.x + vx * u, a.y + vy * u);
    ((p.x - proj.x).powi(2) + (p.y - proj.y).powi(2)).sqrt()
}
