//! Subgraph extract/merge verified by raster comparison.
//!
//! The load-bearing methodology (per the design): when extract splits scene B
//! into extracted A and remainder C, the ground truth is NOT a re-render of A or
//! C after the fact — that would only prove extract is self-consistent, not
//! correct. Instead we rasterize the ORIGINAL B twice, each time masking out one
//! side (draw only A's elements / only C's elements), BEFORE extracting. Then:
//!     render(A) must equal  B-with-C-erased
//!     render(C) must equal  B-with-A-erased
//! So a bug that drops/moves/duplicates the wrong geometry shows up as a pixel
//! diff against an independent baseline.

mod common;

use common::*;
use rand::{rngs::StdRng, Rng, SeedableRng};
use std::collections::HashSet;
use uvec::{CurveId, FillId, Point, Scene};

const RS: f64 = 6.0; // scene -> raster scale
const W: usize = 512;
const H: usize = 256;

fn pt(x: f64, y: f64) -> Point {
    Point::new(x, y)
}
fn line(a: Point, b: Point) -> [Point; 4] {
    let m1 = Point::new(a.x + (b.x - a.x) / 3.0, a.y + (b.y - a.y) / 3.0);
    let m2 = Point::new(a.x + (b.x - a.x) * 2.0 / 3.0, a.y + (b.y - a.y) * 2.0 / 3.0);
    [a, m1, m2, b]
}
fn scaled(poly: &[Point]) -> Vec<Point> {
    poly.iter().map(|p| Point::new(p.x * RS, p.y * RS)).collect()
}
fn payload_color(payload: u64) -> u32 {
    0xFF00_0000 | (payload as u32 & 0x00FF_FFFF)
}

/// A closed square with a fill; returns the fill id.
fn filled_square(s: &mut Scene, x0: f64, y0: f64, x1: f64, y1: f64, payload: u64) -> FillId {
    s.add_curve(line(pt(x0, y0), pt(x1, y0)));
    s.add_curve(line(pt(x1, y0), pt(x1, y1)));
    s.add_curve(line(pt(x1, y1), pt(x0, y1)));
    s.add_curve(line(pt(x0, y1), pt(x0, y0)));
    s.add_fill(pt((x0 + x1) / 2.0, (y0 + y1) / 2.0), payload).unwrap()
}

/// Render exactly the given curves (as black walls) and fills of scene `s`.
fn render_selected(s: &Scene, curves: &[CurveId], fills: &[FillId]) -> Canvas {
    let mut c = Canvas::new(W, H, WHITE);
    for &fid in fills {
        let payload = s.fill_payload(fid);
        if payload == 0 {
            continue;
        }
        let polys: Vec<Vec<Point>> =
            s.fill_loops_flattened(fid).iter().map(|p| scaled(p)).collect();
        scanline_fill(&mut c, &polys, payload_color(payload));
    }
    for &cid in curves {
        stroke_polyline(&mut c, &scaled(&s.curve_polyline(cid)), BLACK);
    }
    c
}

fn render_all(s: &Scene) -> Canvas {
    let curves: Vec<_> = s.curve_ids().collect();
    let fills: Vec<_> = s.fill_ids().collect();
    render_selected(s, &curves, &fills)
}

/// Boundary curves of a set of fills (the natural "grow selection to walls").
fn boundary_of(s: &Scene, fills: &[FillId]) -> Vec<CurveId> {
    let mut set: HashSet<CurveId> = HashSet::new();
    for &f in fills {
        set.extend(s.fill_boundary_curves(f));
    }
    let mut v: Vec<_> = set.into_iter().collect();
    v.sort();
    v
}

/// Two squares sharing a wall: extracting one must DUPLICATE the shared wall
/// (kept in C, copied to A) so neither fill loses a boundary.
#[test]
fn extract_shared_wall_duplicates() {
    let mut b = Scene::new();
    b.set_snap_distance(1.0);
    let s1 = filled_square(&mut b, 10.0, 10.0, 40.0, 40.0, 0xFF_0000);
    let s2 = filled_square(&mut b, 40.0, 10.0, 70.0, 40.0, 0x00_00FF);
    assert_eq!(b.curve_ids().count(), 7, "shared wall deduped to one curve");

    let a_curves = boundary_of(&b, &[s1]);
    let c_curves = boundary_of(&b, &[s2]);
    // Ground truth from the ORIGINAL scene, one side masked out each way.
    let ga = render_selected(&b, &a_curves, &[s1]);
    let gc = render_selected(&b, &c_curves, &[s2]);

    let (a, _, _) = b.extract(&a_curves.iter().copied().collect(), &HashSet::from([s1]));

    assert_eq!(render_all(&a).px, ga.px, "A render != B with C erased");
    assert_eq!(render_all(&b).px, gc.px, "C render != B with A erased");
    assert_eq!(a.curve_ids().count(), 4, "A got all four walls (incl. a copy of the shared)");
    assert_eq!(b.curve_ids().count(), 4, "C keeps its four walls (incl. the shared)");
}

/// Fuzz: many disjoint squares, a random subset extracted. Each side's render
/// must match the original masked to that side.
#[test]
fn extract_partition_matches_masked_render_fuzz() {
    let mut rng = StdRng::seed_from_u64(0x5AB6_1234);
    for iter in 0..300 {
        let mut b = Scene::new();
        b.set_snap_distance(1.0);

        // 4x2 grid of well-separated squares (pitch 20, size 6..14, margin >=3).
        let mut fills = Vec::new();
        for gx in 0..4 {
            for gy in 0..2 {
                let x0 = gx as f64 * 20.0 + 3.0;
                let y0 = gy as f64 * 20.0 + 3.0;
                let sz = rng.gen_range(6.0..14.0);
                let payload = ((fills.len() as u64 + 1) * 0x27_35A1) & 0x00FF_FFFF | 0x01_0101;
                fills.push(filled_square(&mut b, x0, y0, x0 + sz, y0 + sz, payload));
            }
        }

        // Random subset -> A; rest -> C.
        let a_fills: Vec<FillId> = fills.iter().copied().filter(|_| rng.gen_bool(0.5)).collect();
        let a_set: HashSet<FillId> = a_fills.iter().copied().collect();
        let c_fills: Vec<FillId> = fills.iter().copied().filter(|f| !a_set.contains(f)).collect();

        let a_curves = boundary_of(&b, &a_fills);
        let c_curves = boundary_of(&b, &c_fills);

        // Ground truth from ORIGINAL b, before extraction.
        let ga = render_selected(&b, &a_curves, &a_fills);
        let gc = render_selected(&b, &c_curves, &c_fills);

        let (a, _, _) = b.extract(&a_curves.iter().copied().collect(), &a_set);

        assert_eq!(render_all(&a).px, ga.px, "iter {iter}: extracted A mismatch");
        assert_eq!(render_all(&b).px, gc.px, "iter {iter}: remainder C mismatch");
    }
}

/// Merge is the practical inverse: extracting a shape and merging it back (at
/// zero offset) re-planarizes the seam and renders identically to the original.
#[test]
fn extract_then_merge_back_round_trips() {
    let mut b = Scene::new();
    b.set_snap_distance(1.0);
    let s1 = filled_square(&mut b, 10.0, 10.0, 40.0, 40.0, 0xFF_0000);
    let _s2 = filled_square(&mut b, 50.0, 10.0, 80.0, 40.0, 0x00_00FF);
    let original = render_all(&b);

    let a_curves = boundary_of(&b, &[s1]);
    let (a, _, _) = b.extract(&a_curves.iter().copied().collect(), &HashSet::from([s1]));
    // b is now C (just the second square). Merge A back at its original place.
    b.merge(&a, Point::new(0.0, 0.0));

    assert_eq!(render_all(&b).px, original.px, "extract+merge-back changed the picture");
    assert_eq!(b.fill_ids().count(), 2, "both fills present after round trip");
}

/// Merge with an offset places a translated copy; the union renders as both.
#[test]
fn merge_with_offset_places_copy() {
    let mut a = Scene::new();
    a.set_snap_distance(1.0);
    filled_square(&mut a, 10.0, 10.0, 30.0, 30.0, 0xFF_0000);

    // Independent baseline: the two squares built directly where they should end up.
    let mut baseline = Scene::new();
    baseline.set_snap_distance(1.0);
    filled_square(&mut baseline, 10.0, 10.0, 30.0, 30.0, 0xFF_0000);
    filled_square(&mut baseline, 50.0, 50.0, 70.0, 70.0, 0xFF_0000);
    let expected = render_all(&baseline);

    // Actual: one square, then merge a translated copy of A on top.
    let mut target = Scene::new();
    target.set_snap_distance(1.0);
    filled_square(&mut target, 10.0, 10.0, 30.0, 30.0, 0xFF_0000);
    target.merge(&a, Point::new(40.0, 40.0));

    assert_eq!(render_all(&target).px, expected.px, "offset merge placed copy wrong");
    assert_eq!(target.fill_ids().count(), 2);
}
