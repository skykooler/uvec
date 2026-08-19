//! Cubic bezier math: evaluation, splitting, flattening, intersections.

use crate::Point;

pub(crate) fn lerp(a: Point, b: Point, t: f64) -> Point {
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

pub(crate) fn dist(a: Point, b: Point) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

pub(crate) fn eval(p: &[Point; 4], t: f64) -> Point {
    let ab = lerp(p[0], p[1], t);
    let bc = lerp(p[1], p[2], t);
    let cd = lerp(p[2], p[3], t);
    lerp(lerp(ab, bc, t), lerp(bc, cd, t), t)
}

pub(crate) fn split(p: &[Point; 4], t: f64) -> ([Point; 4], [Point; 4]) {
    let ab = lerp(p[0], p[1], t);
    let bc = lerp(p[1], p[2], t);
    let cd = lerp(p[2], p[3], t);
    let abc = lerp(ab, bc, t);
    let bcd = lerp(bc, cd, t);
    let m = lerp(abc, bcd, t);
    ([p[0], ab, abc, m], [m, bcd, cd, p[3]])
}

/// The sub-curve of `p` over parameter range [t0, t1].
pub(crate) fn subsegment(p: &[Point; 4], t0: f64, t1: f64) -> [Point; 4] {
    let (_, right) = split(p, t0);
    let t = if 1.0 - t0 > 1e-12 { (t1 - t0) / (1.0 - t0) } else { 0.0 };
    split(&right, t.clamp(0.0, 1.0)).0
}

/// Total length of the control polygon (upper bound on arc length).
pub(crate) fn polygon_len(p: &[Point; 4]) -> f64 {
    dist(p[0], p[1]) + dist(p[1], p[2]) + dist(p[2], p[3])
}

pub(crate) fn seg_dist(p: Point, a: Point, b: Point) -> f64 {
    let d = b - a;
    let len2 = d.x * d.x + d.y * d.y;
    if len2 < 1e-24 {
        return dist(p, a);
    }
    let t = (((p - a).x * d.x + (p - a).y * d.y) / len2).clamp(0.0, 1.0);
    dist(p, a + d * t)
}

/// Flatten a cubic to a polyline (both endpoints included) by deterministic
/// adaptive subdivision. Shared by the library's internal topology tests and
/// any renderer that wants pixel-consistent results.
pub fn flatten_cubic(p: [Point; 4], tol: f64) -> Vec<Point> {
    let mut out = vec![p[0]];
    flatten_rec(&p, tol, 0, &mut out);
    out
}

fn flatten_rec(p: &[Point; 4], tol: f64, depth: u32, out: &mut Vec<Point>) {
    if depth >= 16 || (seg_dist(p[1], p[0], p[3]) <= tol && seg_dist(p[2], p[0], p[3]) <= tol) {
        out.push(p[3]);
        return;
    }
    let (l, r) = split(p, 0.5);
    flatten_rec(&l, tol, depth + 1, out);
    flatten_rec(&r, tol, depth + 1, out);
}

/// Flatten with parameter tags: (t, point) pairs, endpoints included.
pub(crate) fn flatten_cubic_tagged(p: &[Point; 4], tol: f64) -> Vec<(f64, Point)> {
    let mut out = vec![(0.0, p[0])];
    flatten_tag_rec(p, 0.0, 1.0, tol, 0, &mut out);
    out
}

fn flatten_tag_rec(p: &[Point; 4], t0: f64, t1: f64, tol: f64, depth: u32, out: &mut Vec<(f64, Point)>) {
    if depth >= 16 || (seg_dist(p[1], p[0], p[3]) <= tol && seg_dist(p[2], p[0], p[3]) <= tol) {
        out.push((t1, p[3]));
        return;
    }
    let (l, r) = split(p, 0.5);
    let mid = (t0 + t1) * 0.5;
    flatten_tag_rec(&l, t0, mid, tol, depth + 1, out);
    flatten_tag_rec(&r, mid, t1, tol, depth + 1, out);
}

/// Proper intersection of segments a0->a1 and b0->b1 as fractions (s, t).
fn seg_isect(a0: Point, a1: Point, b0: Point, b1: Point) -> Option<(f64, f64)> {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let denom = d1.x * d2.y - d1.y * d2.x;
    if denom.abs() < 1e-12 {
        return None; // parallel/collinear: near-miss detection covers these
    }
    let w = b0 - a0;
    let s = (w.x * d2.y - w.y * d2.x) / denom;
    let t = (w.x * d1.y - w.y * d1.x) / denom;
    if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
        Some((s, t))
    } else {
        None
    }
}

/// Minimum distance between two segments plus closest-point fractions.
fn seg_seg_dist(a0: Point, a1: Point, b0: Point, b1: Point) -> (f64, f64, f64) {
    if seg_isect(a0, a1, b0, b1).is_some() {
        return (0.0, 0.5, 0.5);
    }
    let frac = |p: Point, q0: Point, q1: Point| -> (f64, f64) {
        let d = q1 - q0;
        let len2 = d.x * d.x + d.y * d.y;
        if len2 < 1e-24 {
            return (dist(p, q0), 0.0);
        }
        let t = (((p - q0).x * d.x + (p - q0).y * d.y) / len2).clamp(0.0, 1.0);
        (dist(p, q0 + d * t), t)
    };
    let cands = [
        {
            let (d, t) = frac(a0, b0, b1);
            (d, 0.0, t)
        },
        {
            let (d, t) = frac(a1, b0, b1);
            (d, 1.0, t)
        },
        {
            let (d, s) = frac(b0, a0, a1);
            (d, s, 0.0)
        },
        {
            let (d, s) = frac(b1, a0, a1);
            (d, s, 1.0)
        },
    ];
    cands
        .into_iter()
        .min_by(|x, y| x.0.partial_cmp(&y.0).unwrap())
        .unwrap()
}

fn lerp_tag(seg: (&(f64, Point), &(f64, Point)), f: f64) -> (f64, Point) {
    let (a, b) = seg;
    (a.0 + (b.0 - a.0) * f, lerp(a.1, b.1, f))
}

/// Intersections (crossings and near-misses within `eps`) between two tagged
/// polylines, as (t_on_a, t_on_b, point) clustered so no two reported points
/// are within `2*eps`.
pub(crate) fn poly_intersections(
    a: &[(f64, Point)],
    b: &[(f64, Point)],
    eps: f64,
) -> Vec<(f64, f64, Point)> {
    let mut raw: Vec<(f64, f64, Point)> = Vec::new();
    for i in 0..a.len().saturating_sub(1) {
        let (a0, a1) = (&a[i], &a[i + 1]);
        for j in 0..b.len().saturating_sub(1) {
            let (b0, b1) = (&b[j], &b[j + 1]);
            // Cheap reject.
            let (minx, maxx) = (a0.1.x.min(a1.1.x) - eps, a0.1.x.max(a1.1.x) + eps);
            let (miny, maxy) = (a0.1.y.min(a1.1.y) - eps, a0.1.y.max(a1.1.y) + eps);
            if b0.1.x.max(b1.1.x) < minx
                || b0.1.x.min(b1.1.x) > maxx
                || b0.1.y.max(b1.1.y) < miny
                || b0.1.y.min(b1.1.y) > maxy
            {
                continue;
            }
            if let Some((s, t)) = seg_isect(a0.1, a1.1, b0.1, b1.1) {
                let (ta, pa) = lerp_tag((a0, a1), s);
                let (tb, _) = lerp_tag((b0, b1), t);
                raw.push((ta, tb, pa));
            } else {
                let (d, s, t) = seg_seg_dist(a0.1, a1.1, b0.1, b1.1);
                if d < eps {
                    let (ta, pa) = lerp_tag((a0, a1), s);
                    let (tb, pb) = lerp_tag((b0, b1), t);
                    raw.push((ta, tb, lerp(pa, pb, 0.5)));
                }
            }
        }
    }
    cluster(raw, eps)
}

/// Self-intersections (crossings and near-misses) of one tagged polyline.
/// Near-misses require clear parameter separation so flattening-adjacent
/// segments don't self-report.
pub(crate) fn poly_self_intersections(a: &[(f64, Point)], eps: f64) -> Vec<(f64, f64, Point)> {
    let mut raw: Vec<(f64, f64, Point)> = Vec::new();
    let n = a.len().saturating_sub(1);
    for i in 0..n {
        for j in (i + 2)..n {
            let (a0, a1) = (&a[i], &a[i + 1]);
            let (b0, b1) = (&a[j], &a[j + 1]);
            if let Some((s, t)) = seg_isect(a0.1, a1.1, b0.1, b1.1) {
                let (ta, pa) = lerp_tag((a0, a1), s);
                let (tb, _) = lerp_tag((b0, b1), t);
                if tb - ta > 1e-4 {
                    raw.push((ta, tb, pa));
                }
            } else {
                let (d, s, t) = seg_seg_dist(a0.1, a1.1, b0.1, b1.1);
                if d < eps {
                    let (ta, pa) = lerp_tag((a0, a1), s);
                    let (tb, pb) = lerp_tag((b0, b1), t);
                    if tb - ta > 0.1 {
                        raw.push((ta, tb, lerp(pa, pb, 0.5)));
                    }
                }
            }
        }
    }
    cluster(raw, eps)
}

fn cluster(mut raw: Vec<(f64, f64, Point)>, eps: f64) -> Vec<(f64, f64, Point)> {
    raw.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap());
    let mut out: Vec<(f64, f64, Point)> = Vec::new();
    for cand in raw {
        if !out.iter().any(|prev| dist(prev.2, cand.2) < eps * 2.0) {
            out.push(cand);
        }
    }
    out
}

/// Exact self-intersection of a cubic, if any, as parameter pair (t1, t2)
/// with t1 < t2. Solved in closed form: B(t1)=B(t2), t1!=t2 reduces to a
/// linear system in s=t1+t2 and u=t1^2+t1*t2+t2^2.
pub(crate) fn self_intersection(p: &[Point; 4]) -> Option<(f64, f64)> {
    let a = Point::new(
        -p[0].x + 3.0 * p[1].x - 3.0 * p[2].x + p[3].x,
        -p[0].y + 3.0 * p[1].y - 3.0 * p[2].y + p[3].y,
    );
    let b = Point::new(
        3.0 * p[0].x - 6.0 * p[1].x + 3.0 * p[2].x,
        3.0 * p[0].y - 6.0 * p[1].y + 3.0 * p[2].y,
    );
    let c = Point::new(-3.0 * p[0].x + 3.0 * p[1].x, -3.0 * p[0].y + 3.0 * p[1].y);
    let det = a.x * b.y - a.y * b.x;
    if det.abs() < 1e-12 {
        return None;
    }
    let u = (b.x * c.y - b.y * c.x) / det;
    let s = (a.y * c.x - a.x * c.y) / det;
    let q = s * s - u;
    let disc = s * s - 4.0 * q;
    if disc <= 0.0 {
        return None;
    }
    let r = disc.sqrt();
    let (t1, t2) = ((s - r) * 0.5, (s + r) * 0.5);
    const E: f64 = 1e-6;
    if t1 > E && t2 < 1.0 - E && t2 - t1 > E {
        Some((t1, t2))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn split_matches_eval() {
        let p = [pt(0.0, 0.0), pt(10.0, 20.0), pt(30.0, -5.0), pt(40.0, 10.0)];
        let (l, r) = split(&p, 0.3);
        assert!(dist(l[3], eval(&p, 0.3)) < 1e-12);
        assert!(dist(r[0], eval(&p, 0.3)) < 1e-12);
        assert!(dist(eval(&l, 0.5), eval(&p, 0.15)) < 1e-9);
        let ss = subsegment(&p, 0.2, 0.7);
        assert!(dist(ss[0], eval(&p, 0.2)) < 1e-9);
        assert!(dist(ss[3], eval(&p, 0.7)) < 1e-9);
    }

    #[test]
    fn flatten_deterministic_and_close() {
        let p = [pt(0.0, 0.0), pt(0.0, 50.0), pt(50.0, 50.0), pt(50.0, 0.0)];
        let f1 = flatten_cubic(p, 0.1);
        let f2 = flatten_cubic(p, 0.1);
        assert_eq!(f1, f2);
        assert_eq!(f1[0], p[0]);
        assert_eq!(*f1.last().unwrap(), p[3]);
        assert!(f1.len() > 4);
    }

    #[test]
    fn line_cross_intersection() {
        // Two straight-line cubics crossing at (5, 5).
        let a = flatten_cubic_tagged(&[pt(0.0, 0.0), pt(3.0, 3.0), pt(7.0, 7.0), pt(10.0, 10.0)], 0.1);
        let b = flatten_cubic_tagged(&[pt(0.0, 10.0), pt(3.0, 7.0), pt(7.0, 3.0), pt(10.0, 0.0)], 0.1);
        let hits = poly_intersections(&a, &b, 0.01);
        assert_eq!(hits.len(), 1);
        assert!(dist(hits[0].2, pt(5.0, 5.0)) < 0.01);
    }

    #[test]
    fn disjoint_no_intersection() {
        let a = flatten_cubic_tagged(&[pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(3.0, 0.0)], 0.1);
        let b = flatten_cubic_tagged(&[pt(0.0, 5.0), pt(1.0, 5.0), pt(2.0, 5.0), pt(3.0, 5.0)], 0.1);
        assert!(poly_intersections(&a, &b, 0.01).is_empty());
    }

    #[test]
    fn near_miss_reported() {
        let a = flatten_cubic_tagged(&[pt(0.0, 0.0), pt(3.0, 0.0), pt(7.0, 0.0), pt(10.0, 0.0)], 0.1);
        let b = flatten_cubic_tagged(&[pt(0.0, 0.3), pt(3.0, 0.3), pt(7.0, 0.3), pt(10.0, 0.3)], 0.1);
        assert!(!poly_intersections(&a, &b, 0.5).is_empty());
        assert!(poly_intersections(&a, &b, 0.2).is_empty());
    }

    #[test]
    fn self_intersection_loop() {
        let p = [pt(0.0, 0.0), pt(20.0, 10.0), pt(-10.0, 10.0), pt(10.0, 0.0)];
        let (t1, t2) = self_intersection(&p).expect("loop curve should self-intersect");
        assert!(t1 < t2);
        assert!(dist(eval(&p, t1), eval(&p, t2)) < 1e-9);
        // A plain arc does not.
        let arc = [pt(0.0, 0.0), pt(10.0, 10.0), pt(20.0, 10.0), pt(30.0, 0.0)];
        assert!(self_intersection(&arc).is_none());
    }
}
