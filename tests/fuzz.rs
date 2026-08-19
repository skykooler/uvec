//! Fuzz harness: random scenes + operations, then verify that a library fill
//! is pixel-identical to a raster flood fill from the same point.

mod common;

use common::*;
use rand::{rngs::StdRng, Rng, SeedableRng};
use uvec::{Point, Scene};

const BOX_MIN: f64 = 8.0;
const BOX_MAX: f64 = 120.0;
const RS: f64 = 8.0; // scene-to-raster scale
const W: usize = 1024;
const H: usize = 1024;
const MARGIN: i64 = 16; // px; keep seeds away from the border
const SNAP: f64 = 2.0;
const TOL: f64 = 0.1;
// Harness paint convention: payload 0 = transparent (not rendered); otherwise
// the low 24 bits are packed 0xRRGGBB, rendered opaque. uvec itself is agnostic.
const FILL_RED: u64 = 0x00FF_0000;

#[derive(Clone, Debug)]
enum Op {
    AddCurve([Point; 4]),
    ReplaceCurve(usize, [Point; 4]),
    MoveVertex(usize, Point),
    DeleteCurve(usize),
    AddFill(Point),
    DeleteFill(usize),
}

fn build_scene(ops: &[Op]) -> Scene {
    let mut s = Scene::new();
    s.set_snap_distance(SNAP);
    s.set_flatten_tolerance(TOL);
    for op in ops {
        match *op {
            Op::AddCurve(pts) => {
                s.add_curve(pts);
            }
            Op::ReplaceCurve(i, pts) => {
                let ids: Vec<_> = s.curve_ids().collect();
                if !ids.is_empty() {
                    let _ = s.replace_curve(ids[i % ids.len()], pts);
                }
            }
            Op::MoveVertex(i, p) => {
                let ids: Vec<_> = s.vertex_ids().collect();
                if !ids.is_empty() {
                    let _ = s.move_vertex(ids[i % ids.len()], p);
                }
            }
            Op::DeleteCurve(i) => {
                let ids: Vec<_> = s.curve_ids().collect();
                if !ids.is_empty() {
                    let _ = s.delete_curve(ids[i % ids.len()]);
                }
            }
            Op::AddFill(p) => {
                let _ = s.add_fill(p, 0); // transparent
            }
            Op::DeleteFill(i) => {
                let ids: Vec<_> = s.fill_ids().collect();
                if !ids.is_empty() {
                    let _ = s.delete_fill(ids[i % ids.len()]);
                }
            }
        }
    }
    s
}

fn scaled(poly: &[Point]) -> Vec<Point> {
    poly.iter().map(|p| Point::new(p.x * RS, p.y * RS)).collect()
}

fn stroke_scene(s: &Scene, canvas: &mut Canvas) {
    for c in s.curve_ids() {
        let poly = scaled(&s.curve_polyline(c));
        stroke_polyline(canvas, &poly, BLACK);
    }
}

fn render_scene(s: &Scene) -> Canvas {
    let mut canvas = Canvas::new(W, H, WHITE);
    for f in s.fill_ids() {
        let payload = s.fill_payload(f);
        if payload == 0 {
            continue; // transparent
        }
        let c = 0xFF00_0000 | (payload as u32 & 0x00FF_FFFF);
        let polys: Vec<Vec<Point>> =
            s.fill_loops_flattened(f).iter().map(|p| scaled(p)).collect();
        scanline_fill(&mut canvas, &polys, c);
    }
    stroke_scene(s, &mut canvas);
    canvas
}

/// Run one full comparison. Ok(()) means pass (or vacuous: seed unusable).
/// The optional dump dir gets a.ppm (raster truth) / b.ppm (library render).
fn check_case(ops: &[Op], seed: Point, dump: Option<&std::path::Path>) -> Result<(), String> {
    let mut scene = build_scene(ops);
    let mut a = Canvas::new(W, H, WHITE);
    stroke_scene(&scene, &mut a);
    let (px, py) = ((seed.x * RS).floor() as i64, (seed.y * RS).floor() as i64);
    if px < MARGIN || py < MARGIN || px >= W as i64 - MARGIN || py >= H as i64 - MARGIN {
        return Ok(());
    }
    if a.get(px, py) != Some(WHITE) {
        return Ok(()); // seed on a stroke: vacuous
    }
    let touched_border = flood_fill(&mut a, px, py, WHITE, RED);
    let result = scene.add_fill(seed, FILL_RED);
    if touched_border {
        return match result {
            Ok(_) => {
                if let Some(dir) = dump {
                    std::fs::create_dir_all(dir).unwrap();
                    a.write_ppm(&dir.join("a_raster.ppm"));
                    render_scene(&scene).write_ppm(&dir.join("b_library.ppm"));
                }
                Err(format!("flood reached border but library filled at {seed:?}"))
            }
            Err(_) => Ok(()),
        };
    }
    if result.is_err() {
        if let Some(dir) = dump {
            std::fs::create_dir_all(dir).unwrap();
            a.write_ppm(&dir.join("a_raster.ppm"));
        }
        return Err(format!("library says NotEnclosed but flood is bounded at {seed:?}"));
    }
    let b = render_scene(&scene);
    if a.px == b.px {
        return Ok(());
    }
    let diffs: Vec<(usize, usize, u32, u32)> = a
        .px
        .iter()
        .zip(&b.px)
        .enumerate()
        .filter(|(_, (x, y))| x != y)
        .map(|(i, (&x, &y))| (i % W, i / W, x, y))
        .collect();
    // Pixel-pinch tolerance: a pixel the library fills but the flood cannot
    // reach may be a wedge-tip pocket sealed off by strokes at this
    // resolution only. Accept iff every diff is fill-side-only and the flood
    // reaches it at a finer scale (a genuinely mis-filled face stays sealed
    // at every scale).
    if diffs.iter().all(|&(_, _, ca, cb)| ca == WHITE && cb == RED) {
        let px: Vec<(usize, usize)> = diffs.iter().map(|&(x, y, ..)| (x, y)).collect();
        if pockets_open_at_scale(&scene, seed, &px, 2) || pockets_open_at_scale(&scene, seed, &px, 4)
        {
            return Ok(());
        }
    }
    if let Some(dir) = dump {
        std::fs::create_dir_all(dir).unwrap();
        a.write_ppm(&dir.join("a_raster.ppm"));
        b.write_ppm(&dir.join("b_library.ppm"));
        for &(x, y, ca, cb) in diffs.iter().take(10) {
            eprintln!("diff px ({x},{y}): raster={ca:08X} library={cb:08X}");
            for ny in y.saturating_sub(3)..(y + 4).min(H) {
                let row: String = (x.saturating_sub(6)..(x + 7).min(W))
                    .map(|nx| {
                        let (pa, pb) =
                            (a.get(nx as i64, ny as i64).unwrap(), b.get(nx as i64, ny as i64).unwrap());
                        match (pa, pb) {
                            _ if (nx, ny) == (x, y) => 'X',
                            (BLACK, BLACK) => '#',
                            (RED, RED) => 'r',
                            (WHITE, WHITE) => '.',
                            _ => '?',
                        }
                    })
                    .collect();
                eprintln!("  {row}");
            }
        }
    }
    Err(format!("{} differing pixels at seed {seed:?}", diffs.len()))
}

/// Re-raster at `k`-times the base scale and flood from the seed; report
/// whether every listed base-scale diff pixel is reached by the finer flood.
fn pockets_open_at_scale(scene: &Scene, seed: Point, diffs: &[(usize, usize)], k: usize) -> bool {
    let rs = RS * k as f64;
    let mut c = Canvas::new(W * k, H * k, WHITE);
    for cid in scene.curve_ids() {
        let poly: Vec<Point> =
            scene.curve_polyline(cid).iter().map(|p| Point::new(p.x * rs, p.y * rs)).collect();
        stroke_polyline(&mut c, &poly, BLACK);
    }
    let (px, py) = ((seed.x * rs).floor() as i64, (seed.y * rs).floor() as i64);
    if c.get(px, py) != Some(WHITE) {
        return false;
    }
    flood_fill(&mut c, px, py, WHITE, RED);
    diffs.iter().all(|&(x, y)| {
        (0..k).any(|dy| {
            (0..k).any(|dx| c.get((x * k + dx) as i64, (y * k + dy) as i64) == Some(RED))
        })
    })
}

// ---- generation ----

fn gen_point(rng: &mut StdRng) -> Point {
    Point::new(rng.gen_range(BOX_MIN..BOX_MAX), rng.gen_range(BOX_MIN..BOX_MAX))
}

fn gen_curve(rng: &mut StdRng) -> [Point; 4] {
    [gen_point(rng), gen_point(rng), gen_point(rng), gen_point(rng)]
}

fn gen_ops(rng: &mut StdRng) -> Vec<Op> {
    let mut ops = Vec::new();
    for _ in 0..rng.gen_range(1..=8) {
        ops.push(Op::AddCurve(gen_curve(rng)));
    }
    for _ in 0..rng.gen_range(0..=12) {
        let roll = rng.gen_range(0..100);
        let idx = rng.gen_range(0..64usize);
        ops.push(match roll {
            0..=34 => Op::AddCurve(gen_curve(rng)),
            35..=49 => Op::ReplaceCurve(idx, gen_curve(rng)),
            50..=69 => Op::MoveVertex(idx, gen_point(rng)),
            70..=79 => Op::DeleteCurve(idx),
            80..=89 => Op::AddFill(gen_point(rng)),
            _ => Op::DeleteFill(idx),
        });
    }
    ops
}

/// Pick a seed pixel that is not on a stroke, mapped back to scene coords.
fn pick_seed(rng: &mut StdRng, scene: &Scene) -> Option<Point> {
    let mut canvas = Canvas::new(W, H, WHITE);
    stroke_scene(scene, &mut canvas);
    for _ in 0..300 {
        let px = rng.gen_range(MARGIN..W as i64 - MARGIN);
        let py = rng.gen_range(MARGIN..H as i64 - MARGIN);
        if canvas.get(px, py) == Some(WHITE) {
            return Some(Point::new((px as f64 + 0.5) / RS, (py as f64 + 0.5) / RS));
        }
    }
    None
}

// ---- minimization ----

fn coord_slots(op: &Op) -> usize {
    match op {
        Op::AddCurve(_) | Op::ReplaceCurve(..) => 8,
        Op::MoveVertex(..) | Op::AddFill(_) => 2,
        _ => 0,
    }
}

fn get_coord(op: &Op, k: usize) -> f64 {
    let pt = |p: &Point, k: usize| if k % 2 == 0 { p.x } else { p.y };
    match op {
        Op::AddCurve(c) | Op::ReplaceCurve(_, c) => pt(&c[k / 2], k),
        Op::MoveVertex(_, p) | Op::AddFill(p) => pt(p, k),
        _ => unreachable!(),
    }
}

fn set_coord(op: &mut Op, k: usize, v: f64) {
    let pt = |p: &mut Point, k: usize, v: f64| {
        if k % 2 == 0 {
            p.x = v
        } else {
            p.y = v
        }
    };
    match op {
        Op::AddCurve(c) | Op::ReplaceCurve(_, c) => pt(&mut c[k / 2], k, v),
        Op::MoveVertex(_, p) | Op::AddFill(p) => pt(p, k, v),
        _ => unreachable!(),
    }
}

fn minimize(mut ops: Vec<Op>, seed: Point) -> Vec<Op> {
    // Drop whole ops while the failure reproduces.
    loop {
        let mut changed = false;
        for i in 0..ops.len() {
            let mut t = ops.clone();
            t.remove(i);
            if check_case(&t, seed, None).is_err() {
                ops = t;
                changed = true;
                break;
            }
        }
        if !changed {
            break;
        }
    }
    // Round coordinates toward integers while the failure reproduces.
    for i in 0..ops.len() {
        for k in 0..coord_slots(&ops[i]) {
            let v = get_coord(&ops[i], k);
            let r = v.round();
            if r != v {
                let mut t = ops.clone();
                set_coord(&mut t[i], k, r);
                if check_case(&t, seed, None).is_err() {
                    ops = t;
                }
            }
        }
    }
    ops
}

// ---- runners ----

fn run_iters(n: usize, base: u64) {
    let verbose = std::env::var("FUZZ_VERBOSE").is_ok();
    let only: Option<usize> = std::env::var("FUZZ_ONLY").ok().and_then(|s| s.parse().ok());
    let mut vacuous = 0usize;
    for i in 0..n {
        if only.is_some_and(|o| o != i) {
            continue;
        }
        if verbose {
            eprintln!("iter {i}");
        }
        let mut rng = StdRng::seed_from_u64(base ^ (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let ops = gen_ops(&mut rng);
        if verbose {
            eprintln!("{ops:?}");
        }
        let scene = build_scene(&ops);
        if verbose {
            eprintln!("built: {} curves", scene.curve_ids().count());
        }
        let Some(seed) = pick_seed(&mut rng, &scene) else {
            vacuous += 1;
            continue;
        };
        if let Err(e) = check_case(&ops, seed, None) {
            eprintln!("FAILURE at iteration {i} (base {base}): {e}");
            let min = minimize(ops.clone(), seed);
            let dir = std::path::PathBuf::from("target/fuzz-failure");
            match check_case(&min, seed, Some(&dir)) {
                Ok(()) => {
                    eprintln!("warning: minimized case did not reproduce; dumping original");
                    let _ = check_case(&ops, seed, Some(&dir));
                }
                Err(em) => eprintln!("minimized case error: {em}"),
            }
            panic!(
                "fuzz failure at iteration {i}: {e}\nminimized seed point: {seed:?}\nminimized ops:\n{min:#?}\nimages dumped to {dir:?}"
            );
        }
        if (i + 1) % 500 == 0 {
            eprintln!("fuzz: {}/{} ok ({} vacuous)", i + 1, n, vacuous);
        }
    }
}

#[test]
fn repro1() {
    let ops = vec![Op::AddCurve([
        Point::new(65.64830340614216, 119.0),
        Point::new(52.93828336926242, 58.45697201306852),
        Point::new(92.0, 28.883502183107645),
        Point::new(57.359309409081575, 9.0),
    ])];
    let seed = Point::new(61.8125, 38.3125);
    check_case(&ops, seed, None).unwrap();
}

#[test]
fn repro2() {
    let ops = vec![
        Op::AddCurve([
            Point::new(85.0, 69.0),
            Point::new(105.0, 104.0),
            Point::new(57.0, 79.0),
            Point::new(81.0, 97.0),
        ]),
        Op::AddCurve([
            Point::new(82.0, 18.0),
            Point::new(31.0, 118.0),
            Point::new(17.0, 29.0),
            Point::new(71.0, 61.0),
        ]),
        Op::AddCurve([
            Point::new(85.0, 109.0),
            Point::new(112.0, 12.0),
            Point::new(112.0, 98.0),
            Point::new(84.0, 67.0),
        ]),
        Op::MoveVertex(56, Point::new(78.0, 16.0)),
        Op::AddCurve([
            Point::new(109.0, 82.0),
            Point::new(29.0, 101.0),
            Point::new(40.0, 108.0),
            Point::new(70.0, 56.0),
        ]),
        Op::AddCurve([
            Point::new(66.0, 119.0),
            Point::new(53.0, 58.0),
            Point::new(92.0, 29.0),
            Point::new(57.0, 9.0),
        ]),
    ];
    let seed = Point::new(61.8125, 38.3125);
    let scene = build_scene(&ops);
    for c in scene.curve_ids() {
        let pts = scene.curve_points(c);
        let poly = scene.curve_polyline(c);
        // even-odd of seed against this piece's polyline treated as closed
        let mut inside = false;
        let n = poly.len();
        for i in 0..n {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            let down = a.y <= seed.y && seed.y < b.y;
            let up = b.y <= seed.y && seed.y < a.y;
            if (down || up)
                && a.x + (seed.y - a.y) / (b.y - a.y) * (b.x - a.x) > seed.x
            {
                inside = !inside;
            }
        }
        eprintln!(
            "curve {c:?}: ends ({:.3},{:.3})->({:.3},{:.3}) len {} closed={} seed_inside={}",
            pts[0].x, pts[0].y, pts[3].x, pts[3].y, poly.len(),
            pts[0] == pts[3], inside
        );
    }
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

#[test]
fn repro3() {
    let ops = vec![
        Op::AddCurve([
            Point::new(71.0, 106.0),
            Point::new(11.0, 44.0),
            Point::new(34.0, 81.0),
            Point::new(98.0, 101.0),
        ]),
        Op::AddCurve([
            Point::new(114.0, 90.0),
            Point::new(37.0, 35.0),
            Point::new(88.0, 89.0),
            Point::new(115.0, 92.0),
        ]),
        Op::ReplaceCurve(15, [
            Point::new(86.0, 86.0),
            Point::new(15.0, 112.0),
            Point::new(28.0, 55.0),
            Point::new(66.0, 73.0),
        ]),
        Op::AddCurve([
            Point::new(60.0, 119.0),
            Point::new(95.0, 53.0),
            Point::new(93.0, 39.0),
            Point::new(52.0, 57.0),
        ]),
        Op::ReplaceCurve(59, [
            Point::new(106.0, 58.0),
            Point::new(71.0, 72.0),
            Point::new(65.0, 24.0),
            Point::new(99.0, 21.0),
        ]),
        Op::AddCurve([
            Point::new(29.0, 64.0),
            Point::new(16.0, 96.0),
            Point::new(59.0, 103.0),
            Point::new(84.0, 17.0),
        ]),
    ];
    let seed = Point::new(82.6875, 67.3125);
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

#[test]
fn repro4() {
    let ops = vec![
        Op::AddCurve([
            Point::new(29.0, 18.0),
            Point::new(45.0, 15.0),
            Point::new(92.0, 9.0),
            Point::new(8.0, 71.0),
        ]),
        Op::AddCurve([
            Point::new(93.0, 72.0),
            Point::new(91.0, 22.0),
            Point::new(62.0, 83.0),
            Point::new(104.0, 112.0),
        ]),
        Op::AddCurve([
            Point::new(52.0, 82.0),
            Point::new(58.0, 111.0),
            Point::new(105.0, 100.0),
            Point::new(88.0, 75.0),
        ]),
        Op::AddCurve([
            Point::new(83.51258505779585, 36.0),
            Point::new(104.0, 44.0),
            Point::new(70.0, 107.0),
            Point::new(86.0, 34.0),
        ]),
        Op::ReplaceCurve(63, [
            Point::new(59.0, 114.0),
            Point::new(52.0, 89.0),
            Point::new(85.0, 25.0),
            Point::new(42.0, 53.0),
        ]),
        Op::AddCurve([
            Point::new(91.0, 53.0),
            Point::new(25.0, 96.0),
            Point::new(72.0, 111.0),
            Point::new(87.0, 21.0),
        ]),
    ];
    let seed = Point::new(66.0625, 84.8125);
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

#[test]
fn repro5() {
    let ops = vec![
        Op::AddCurve([
            Point::new(43.0, 92.0),
            Point::new(23.0, 104.0),
            Point::new(25.0, 12.0),
            Point::new(103.0, 48.0),
        ]),
        Op::AddCurve([
            Point::new(102.0, 83.0),
            Point::new(9.0, 113.0),
            Point::new(35.0, 78.0),
            Point::new(36.0, 63.0),
        ]),
        Op::AddCurve([
            Point::new(30.0, 42.51450448292847),
            Point::new(111.45678470719959, 20.0),
            Point::new(59.0, 79.0),
            Point::new(87.42007899153623, 40.0),
        ]),
        Op::ReplaceCurve(46, [
            Point::new(74.0, 105.0),
            Point::new(103.0, 38.0),
            Point::new(86.0, 83.0),
            Point::new(100.0, 21.0),
        ]),
    ];
    let seed = Point::new(65.6875, 90.0625);
    let scene = build_scene(&ops);
    audit_crossings(&scene);
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

/// Diagnostic: report any true polyline crossings surviving in the scene.
fn audit_crossings(scene: &Scene) {
    let ids: Vec<_> = scene.curve_ids().collect();
    let vposs: Vec<Point> = scene.vertex_ids().map(|v| scene.vertex_pos(v)).collect();
    for (i, &a) in ids.iter().enumerate() {
        let pa = scene.curve_polyline(a);
        for &b in &ids[i + 1..] {
            let pb = scene.curve_polyline(b);
            for sa in pa.windows(2) {
                for sb in pb.windows(2) {
                    let d1 = (sa[1].x - sa[0].x, sa[1].y - sa[0].y);
                    let d2 = (sb[1].x - sb[0].x, sb[1].y - sb[0].y);
                    let denom = d1.0 * d2.1 - d1.1 * d2.0;
                    if denom.abs() < 1e-12 {
                        continue;
                    }
                    let w = (sb[0].x - sa[0].x, sb[0].y - sa[0].y);
                    let s = (w.0 * d2.1 - w.1 * d2.0) / denom;
                    let t = (w.0 * d1.1 - w.1 * d1.0) / denom;
                    if (1e-9..=1.0 - 1e-9).contains(&s) && (1e-9..=1.0 - 1e-9).contains(&t) {
                        let px = sa[0].x + d1.0 * s;
                        let py = sa[0].y + d1.1 * s;
                        let p = Point::new(px, py);
                        let dv = vposs
                            .iter()
                            .map(|&v| ((v.x - px).powi(2) + (v.y - py).powi(2)).sqrt())
                            .fold(f64::INFINITY, f64::min);
                        eprintln!(
                            "CROSSING {a:?} x {b:?} at ({px:.3},{py:.3}) nearest_vertex_dist={dv:.3}"
                        );
                        let _ = p;
                    }
                }
            }
        }
    }
}

#[test]
fn repro6() {
    let ops = vec![
        Op::AddCurve([
            Point::new(43.0, 92.0),
            Point::new(23.0, 104.0),
            Point::new(25.0, 11.668807898478086),
            Point::new(103.0, 48.0),
        ]),
        Op::AddCurve([
            Point::new(43.0, 81.0),
            Point::new(81.0, 42.0),
            Point::new(94.0, 11.0),
            Point::new(79.0, 64.0),
        ]),
        Op::AddCurve([
            Point::new(102.0, 83.0),
            Point::new(9.0, 113.0),
            Point::new(35.0, 78.44833013864337),
            Point::new(36.0, 63.0),
        ]),
        Op::AddCurve([
            Point::new(30.0, 43.0),
            Point::new(111.0, 20.0),
            Point::new(59.0, 79.0),
            Point::new(87.0, 40.0),
        ]),
        Op::ReplaceCurve(46, [
            Point::new(74.0, 105.0),
            Point::new(103.0, 38.0),
            Point::new(86.0, 83.0),
            Point::new(100.0, 21.0),
        ]),
    ];
    let seed = Point::new(65.6875, 90.0625);
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

#[test]
fn repro7() {
    let ops = vec![
        Op::AddCurve([
            Point::new(49.0, 79.0),
            Point::new(97.0, 86.0),
            Point::new(58.0, 86.0),
            Point::new(76.0, 33.0),
        ]),
        Op::AddCurve([
            Point::new(39.0, 72.0),
            Point::new(25.0, 42.0),
            Point::new(34.0, 82.0),
            Point::new(98.45663501904787, 19.41721936531993),
        ]),
        Op::AddCurve([
            Point::new(51.0, 43.0),
            Point::new(66.0, 47.0),
            Point::new(112.0, 40.0),
            Point::new(80.0, 43.0),
        ]),
        Op::AddCurve([
            Point::new(95.0, 14.0),
            Point::new(55.0, 70.27561542866727),
            Point::new(40.0, 33.0),
            Point::new(119.0, 40.0),
        ]),
        Op::DeleteCurve(50),
        Op::AddCurve([
            Point::new(12.0, 38.0),
            Point::new(71.43739326004234, 19.0),
            Point::new(109.0, 41.0),
            Point::new(98.0, 73.0),
        ]),
        Op::DeleteCurve(50),
        Op::ReplaceCurve(51, [
            Point::new(94.0, 82.0),
            Point::new(98.0, 113.0),
            Point::new(36.0, 106.0),
            Point::new(47.0, 31.0),
        ]),
    ];
    let seed = Point::new(70.3125, 62.8125);
    let scene = build_scene(&ops);
    audit_crossings(&scene);
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

#[test]
fn repro8() {
    let ops = vec![
        Op::AddCurve([
            Point::new(111.0, 29.0),
            Point::new(88.0, 29.0),
            Point::new(91.0, 61.0),
            Point::new(20.0, 42.0),
        ]),
        Op::AddCurve([
            Point::new(114.0, 50.0),
            Point::new(116.0, 31.0),
            Point::new(31.0, 86.0),
            Point::new(111.0, 90.0),
        ]),
        Op::ReplaceCurve(13, [
            Point::new(47.0, 14.0),
            Point::new(52.0, 65.0),
            Point::new(8.0, 50.287949054394744),
            Point::new(39.0, 46.0),
        ]),
    ];
    let seed = Point::new(35.6875, 49.4375);
    let scene = build_scene(&ops);
    audit_crossings(&scene);
    check_case(&ops, seed, Some(std::path::Path::new("target/fuzz-failure"))).unwrap();
}

#[test]
fn fuzz_smoke() {
    run_iters(100, 0xC0FFEE);
}

#[test]
#[ignore]
fn fuzz_10k() {
    run_iters(10_000, 1);
}
