//! Round-trips the whole scene through JSON (feature = "serde"). Ids are plain
//! indices and never reused, so serializing the slot vectors as-is preserves
//! every CurveId/VertexId/FillId — including fill boundary references and the
//! opaque curve/fill payloads.
#![cfg(feature = "serde")]

use uvec::{Point, Scene};

fn line(a: Point, b: Point) -> [Point; 4] {
    let m1 = Point::new(a.x + (b.x - a.x) / 3.0, a.y + (b.y - a.y) / 3.0);
    let m2 = Point::new(a.x + (b.x - a.x) * 2.0 / 3.0, a.y + (b.y - a.y) * 2.0 / 3.0);
    [a, m1, m2, b]
}

fn square(s: &mut Scene, x0: f64, y0: f64, x1: f64, y1: f64) {
    s.add_curve(line(Point::new(x0, y0), Point::new(x1, y0)));
    s.add_curve(line(Point::new(x1, y0), Point::new(x1, y1)));
    s.add_curve(line(Point::new(x1, y1), Point::new(x0, y1)));
    s.add_curve(line(Point::new(x0, y1), Point::new(x0, y0)));
}

#[test]
fn scene_round_trips_through_json() {
    let mut s = Scene::new();
    s.set_snap_distance(1.0);
    square(&mut s, 0.0, 0.0, 100.0, 100.0);
    // tagged crossing stroke (exercises curve payload + split provenance)
    s.add_curve_tagged(line(Point::new(-10.0, 50.0), Point::new(110.0, 50.0)), 7);
    let fill = s.add_fill(Point::new(25.0, 25.0), 0xABCD).unwrap();

    let json = serde_json::to_string(&s).unwrap();
    let s2: Scene = serde_json::from_str(&json).unwrap();

    // Structure preserved.
    assert_eq!(s2.curve_ids().count(), s.curve_ids().count());
    assert_eq!(s2.vertex_ids().count(), s.vertex_ids().count());
    assert_eq!(s2.fill_ids().count(), s.fill_ids().count());

    // Fill payload + geometry preserved (same FillId).
    assert_eq!(s2.fill_payload(fill), 0xABCD);
    assert_eq!(
        s2.fill_loops_flattened(fill).len(),
        s.fill_loops_flattened(fill).len()
    );

    // Curve payloads preserved: the tag-7 stroke split into pieces, all still 7.
    let tagged = s2.curve_ids().filter(|&c| s2.curve_tag(c) == 7).count();
    assert!(tagged >= 2, "tagged stroke survived split with its payload, got {tagged}");

    // The deserialized scene is still live: further ops work and keep invariants.
    let mut s3 = s2;
    s3.add_curve(line(Point::new(50.0, -10.0), Point::new(50.0, 110.0)));
    assert!(s3.fill_ids().count() >= 1);
}
