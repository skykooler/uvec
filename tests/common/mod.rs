//! Raster test canvas: supercover line drawing, flood fill, scanline fill.

use uvec::Point;

pub const WHITE: u32 = 0xFFFF_FFFF;
pub const BLACK: u32 = 0xFF00_0000;
pub const RED: u32 = 0xFFFF_0000;

pub struct Canvas {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u32>,
}

impl Canvas {
    pub fn new(w: usize, h: usize, color: u32) -> Self {
        Canvas { w, h, px: vec![color; w * h] }
    }

    pub fn get(&self, x: i64, y: i64) -> Option<u32> {
        if x < 0 || y < 0 || x >= self.w as i64 || y >= self.h as i64 {
            None
        } else {
            Some(self.px[y as usize * self.w + x as usize])
        }
    }

    pub fn set(&mut self, x: i64, y: i64, c: u32) {
        if x >= 0 && y >= 0 && x < self.w as i64 && y < self.h as i64 {
            self.px[y as usize * self.w + x as usize] = c;
        }
    }

    pub fn write_ppm(&self, path: &std::path::Path) {
        let mut out = format!("P6\n{} {}\n255\n", self.w, self.h).into_bytes();
        for &p in &self.px {
            out.push((p >> 16) as u8);
            out.push((p >> 8) as u8);
            out.push(p as u8);
        }
        std::fs::write(path, out).unwrap();
    }
}

/// Visit every pixel the segment a->b passes through (grid traversal DDA).
pub fn supercover(a: Point, b: Point, mut plot: impl FnMut(i64, i64)) {
    let (mut x, mut y) = (a.x.floor() as i64, a.y.floor() as i64);
    let (xe, ye) = (b.x.floor() as i64, b.y.floor() as i64);
    plot(x, y);
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let sx: i64 = if dx > 0.0 { 1 } else { -1 };
    let sy: i64 = if dy > 0.0 { 1 } else { -1 };
    let mut tmx = if dx != 0.0 {
        let next = if dx > 0.0 { (x + 1) as f64 } else { x as f64 };
        (next - a.x) / dx
    } else {
        f64::INFINITY
    };
    let mut tmy = if dy != 0.0 {
        let next = if dy > 0.0 { (y + 1) as f64 } else { y as f64 };
        (next - a.y) / dy
    } else {
        f64::INFINITY
    };
    let tdx = if dx != 0.0 { 1.0 / dx.abs() } else { f64::INFINITY };
    let tdy = if dy != 0.0 { 1.0 / dy.abs() } else { f64::INFINITY };
    let limit = (xe - x).abs() + (ye - y).abs() + 4;
    let mut steps = 0;
    while (x != xe || y != ye) && steps < limit {
        if tmx < tmy {
            x += sx;
            tmx += tdx;
        } else {
            y += sy;
            tmy += tdy;
        }
        plot(x, y);
        steps += 1;
    }
}

pub fn stroke_polyline(canvas: &mut Canvas, poly: &[Point], color: u32) {
    for seg in poly.windows(2) {
        supercover(seg[0], seg[1], |x, y| canvas.set(x, y, color));
    }
}

/// 4-connected flood fill of `from`-colored pixels starting at (sx, sy).
/// Returns true if the filled region touches the canvas border.
pub fn flood_fill(canvas: &mut Canvas, sx: i64, sy: i64, from: u32, to: u32) -> bool {
    if canvas.get(sx, sy) != Some(from) {
        return false;
    }
    let mut touched = false;
    let mut stack = vec![(sx, sy)];
    canvas.set(sx, sy, to);
    while let Some((x, y)) = stack.pop() {
        if x == 0 || y == 0 || x == canvas.w as i64 - 1 || y == canvas.h as i64 - 1 {
            touched = true;
        }
        for (nx, ny) in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)] {
            if canvas.get(nx, ny) == Some(from) {
                canvas.set(nx, ny, to);
                stack.push((nx, ny));
            }
        }
    }
    touched
}

/// Even-odd scanline fill of closed polylines (sampled at pixel centers).
pub fn scanline_fill(canvas: &mut Canvas, polys: &[Vec<Point>], color: u32) {
    let (mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY);
    for poly in polys {
        for p in poly {
            ymin = ymin.min(p.y);
            ymax = ymax.max(p.y);
        }
    }
    let y0 = (ymin.floor().max(0.0)) as i64;
    let y1 = (ymax.ceil().min(canvas.h as f64)) as i64;
    let mut xs: Vec<f64> = Vec::new();
    for y in y0..y1 {
        let yc = y as f64 + 0.5;
        xs.clear();
        for poly in polys {
            let n = poly.len();
            for i in 0..n {
                let (a, b) = (poly[i], poly[(i + 1) % n]);
                let down = a.y <= yc && yc < b.y;
                let up = b.y <= yc && yc < a.y;
                if down || up {
                    xs.push(a.x + (yc - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut k = 0;
        while k + 1 < xs.len() {
            let (x0, x1) = (xs[k], xs[k + 1]);
            let mut px = (x0 - 0.5).ceil() as i64;
            while (px as f64) + 0.5 < x1 {
                canvas.set(px, y, color);
                px += 1;
            }
            k += 2;
        }
    }
}
