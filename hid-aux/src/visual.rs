//! Optical motion estimation from PMW3610 frame-grab pixels.
//!
//! The sensor streams raw 22x22 8-bit images of the navigation surface.
//! This module reconstructs motion purely from those pixels, bypassing the
//! chip's built-in DX/DY navigation. A single estimate is produced per frame:
//!
//! * relative: how far the surface image moved between the previous and the
//!   current frame (mouse-style).
//!

use crate::hid::{FRAME_H, FRAME_LEN, FRAME_W};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{RecvTimeoutError, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub type Frame = [u8; FRAME_LEN];

/// Fixed-point scaling: 1 unit = 1/64 pixel. Relative motion is reported in
/// these units; `dx_counts`/`dy_counts` are the float, gain-scaled versions.
pub const FP_SHIFT: i32 = 6;
pub const FP_ONE: i32 = 1 << FP_SHIFT;
pub const FP_MASK: i32 = FP_ONE - 1;

/// Tunables. Everything here is a host finetuning knob; on the MCU these map
/// to Kconfig or devicetree values.
#[derive(Debug, Clone, Copy)]
pub struct VisualConfig {
    /// Relative deadband in 1/64-pixel units; smaller motions are snapped to 0.
    pub deadband: i32,
    /// Minimum zero-mean pixel variance; below this the frame is "flat" and
    /// relative motion is rejected (confidence forced to 0).
    pub texture_min: u32,
    /// Lucas-Kanade refinement iterations (per pyramid level).
    pub lk_iter: u32,
    /// Image-pyramid levels the relative estimate uses (1 = single level,
    /// ≈2 px of capture range). Each extra level doubles the range; the
    /// 3-level default reaches ~8 px per frame. Clamped to the levels
    /// available for the frame size.
    pub pyramid_levels: u8,
    /// Seed the estimate from the previous frame's motion (fast flicks).
    /// The guess is validated against a null prediction at the coarsest
    /// level, so a bad prediction cannot hijack the result.
    pub predict: bool,
    /// Axis remap before gain scaling.
    pub swap_xy: bool,
    pub invert_x: bool,
    pub invert_y: bool,
    /// Pixel -> counts scale applied to the relative estimate.
    pub gain: f32,
    /// Exponential smoothing (0.0 = off, 1.0 = frozen) on the counts output.
    pub smoothing: f32,
    /// Minimum confidence (0..1000) for the relative estimate to be valid.
    pub conf_min: u32,
    /// 3x3 box pre-blur applied to every frame before estimation. Reduces
    /// sensor pixel noise, the dominant source of estimator jitter.
    pub prefilter: bool,
    /// Consecutive low-confidence frames tolerated before the relative
    /// estimate turns invalid (validity hysteresis, prevents flicker).
    pub hysteresis: u32,
    /// Consecutive frames at/below the deadband needed to snap the smoothed
    /// output back to exactly zero (rest detection).
    pub rest_frames: u32,
}

impl Default for VisualConfig {
    fn default() -> Self {
        Self {
            deadband: 6,
            texture_min: 12,
            lk_iter: 5,
            pyramid_levels: 3,
            predict: true,
            swap_xy: false,
            invert_x: false,
            invert_y: false,
            gain: 1.0,
            smoothing: 0.4,
            conf_min: 300,
            prefilter: true,
            hysteresis: 2,
            rest_frames: 2,
        }
    }
}

/// Per-frame result, suitable for display and tuning.
#[derive(Debug, Clone, Copy)]
pub struct VisualOutput {
    /// Relative motion in fixed-point 1/64-pixel units (axis-remapped).
    pub dx: i32,
    pub dy: i32,
    /// Relative motion after axis remap and gain, in counts.
    pub dx_counts: f32,
    pub dy_counts: f32,
    /// 0..=1000 reliability of the relative estimate.
    pub confidence: u32,
    pub valid: bool,
}

impl Default for VisualOutput {
    fn default() -> Self {
        Self {
            dx: 0,
            dy: 0,
            dx_counts: 0.0,
            dy_counts: 0.0,
            confidence: 0,
            valid: false,
        }
    }
}

#[inline]
fn variance(frame: &Frame) -> u32 {
    let mut s: i64 = 0;
    for &z in frame.iter() {
        s += z as i64;
    }
    let m0 = s / FRAME_LEN as i64;
    let mut v: i64 = 0;
    for &z in frame.iter() {
        let d = z as i64 - m0;
        v += d * d;
    }
    v as u32
}

// ---------------------------------------------------------------------------
// Relative motion estimators
// ---------------------------------------------------------------------------

/// Separable 3x3 box blur `(1,2,1) x (1,2,1) / 16` with border replication.
/// Damps sensor pixel noise before differentiation — the dominant source of
/// estimator jitter. Pure integer (u16 intermediate), `core`-only.
fn blur3x3(src: &Frame) -> Frame {
    let mut tmp = [0u16; FRAME_LEN];
    for y in 0..FRAME_H {
        let r = y * FRAME_W;
        for x in 0..FRAME_W {
            let nx0 = x.saturating_sub(1);
            let nx1 = (x + 1).min(FRAME_W - 1);
            let v = (src[r + nx0] as u16 + 2 * src[r + x] as u16 + src[r + nx1] as u16) >> 2;
            tmp[r + x] = v;
        }
    }
    let mut out = [0u8; FRAME_LEN];
    for y in 0..FRAME_H {
        let ny0 = y.saturating_sub(1);
        let ny1 = (y + 1).min(FRAME_H - 1);
        let r0 = ny0 * FRAME_W;
        let r1 = y * FRAME_W;
        let r2 = ny1 * FRAME_W;
        for x in 0..FRAME_W {
            let v = (tmp[r0 + x] + 2 * tmp[r1 + x] + tmp[r2 + x]) >> 2;
            out[r1 + x] = v as u8;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Heap-free image pyramid
// ---------------------------------------------------------------------------

/// Downsample factor between two pyramid levels.
const PYR_STEP: usize = 2;

/// Side a level must keep to stay usable as the search level. Far below this
/// the box average has removed the texture that tells displacements apart, and
/// an exhaustive search there starts ranking spurious matches above the true
/// shift — on a 22x22 frame the 6x6 level is already in that regime, so the
/// pyramid stops at 11x11.
const MIN_SEARCH_SIDE: usize = 8;



const fn level_count() -> usize {
    let mut n = 1;
    while half_up_dim(FRAME_W, n) >= MIN_SEARCH_SIDE && half_up_dim(FRAME_H, n) >= MIN_SEARCH_SIDE {
        n += 1;
    }
    n
}

const fn half_up_dim(d: usize, level: usize) -> usize {
    (d + (1 << level) - 1) >> level
}

/// Levels physically available; level 0 is the full frame.
const PYRAMID_MAX_LEVELS: usize = level_count();

const fn level_dims(base: usize) -> [usize; PYRAMID_MAX_LEVELS] {
    let mut out = [0usize; PYRAMID_MAX_LEVELS];
    let mut i = 0;
    while i < PYRAMID_MAX_LEVELS {
        out[i] = half_up_dim(base, i);
        i += 1;
    }
    out
}

const LEVEL_W: [usize; PYRAMID_MAX_LEVELS] = level_dims(FRAME_W);
const LEVEL_H: [usize; PYRAMID_MAX_LEVELS] = level_dims(FRAME_H);

const fn level_lens() -> [usize; PYRAMID_MAX_LEVELS] {
    let mut out = [0usize; PYRAMID_MAX_LEVELS];
    let mut i = 0;
    while i < PYRAMID_MAX_LEVELS {
        out[i] = LEVEL_W[i] * LEVEL_H[i];
        i += 1;
    }
    out
}

const LEVEL_LEN: [usize; PYRAMID_MAX_LEVELS] = level_lens();

const fn level_offsets() -> [usize; PYRAMID_MAX_LEVELS] {
    let mut out = [0usize; PYRAMID_MAX_LEVELS];
    let mut acc = 0;
    let mut i = 0;
    while i < PYRAMID_MAX_LEVELS {
        out[i] = acc;
        acc += LEVEL_LEN[i];
        i += 1;
    }
    out
}

const LEVEL_OFF: [usize; PYRAMID_MAX_LEVELS] = level_offsets();

const fn pyramid_len() -> usize {
    let mut total = 0;
    let mut i = 0;
    while i < PYRAMID_MAX_LEVELS {
        total += LEVEL_LEN[i];
        i += 1;
    }
    total
}

const PYRAMID_LEN: usize = pyramid_len();

// Lucas-Kanade needs at least a 3x3 level
const _: () = assert!(
    LEVEL_W[PYRAMID_MAX_LEVELS - 1] >= 3 && LEVEL_H[PYRAMID_MAX_LEVELS - 1] >= 3,
    "coarsest level must be at least 3x3"
);

/// A half-resolution pyramid in one flat buffer: level 0 is the frame, each
/// next level a 2x2 box average (edge-replicated for odd dimensions). Stored
/// inline so the estimator stays heap-free and portable to the MCU.
struct Pyramid {
    data: [u8; PYRAMID_LEN],
}

impl Pyramid {
    const fn empty() -> Self {
        Self { data: [0u8; PYRAMID_LEN] }
    }

    const fn dims(level: usize) -> (usize, usize) {
        (LEVEL_W[level], LEVEL_H[level])
    }

    fn level(&self, level: usize) -> &[u8] {
        &self.data[LEVEL_OFF[level]..LEVEL_OFF[level] + LEVEL_LEN[level]]
    }

    /// (Re)initialise from a full-resolution frame: level 0 is copied, the
    /// rest are 2x2 box averages.
    fn build(&mut self, frame: &Frame) {
        self.data[..LEVEL_LEN[0]].copy_from_slice(frame);
        for l in 1..PYRAMID_MAX_LEVELS {
            let (sw, sh) = Self::dims(l - 1);
            let (dw, dh) = Self::dims(l);
            let so = LEVEL_OFF[l - 1];
            let doff = LEVEL_OFF[l];
            for y in 0..dh {
                for x in 0..dw {
                    let x0 = (PYR_STEP * x).min(sw - 1);
                    let x1 = (PYR_STEP * x + 1).min(sw - 1);
                    let y0 = (PYR_STEP * y).min(sh - 1);
                    let y1 = (PYR_STEP * y + 1).min(sh - 1);
                    let sum = self.data[so + y0 * sw + x0] as u32
                        + self.data[so + y0 * sw + x1] as u32
                        + self.data[so + y1 * sw + x0] as u32
                        + self.data[so + y1 * sw + x1] as u32;
                    self.data[doff + y * dw + x] = ((sum + 2) >> 2) as u8;
                }
            }
        }
    }
}

/// Bilinear sample of a `w x h` image at fixed-point coordinates,
/// border-replicated at the edges.
fn sample_bilin(a: &[u8], w: usize, h: usize, xf: i32, yf: i32) -> i32 {
    let hi_x = w as i32 - 2;
    let hi_y = h as i32 - 2;
    let xi = (xf >> FP_SHIFT).clamp(0, hi_x);
    let yi = (yf >> FP_SHIFT).clamp(0, hi_y);
    let xr = xf & FP_MASK;
    let yr = yf & FP_MASK;
    let x0 = xi as usize;
    let y0 = yi as usize;
    let x1 = x0 + 1;
    let y1 = y0 + 1;
    let z00 = a[y0 * w + x0] as i64;
    let z01 = a[y0 * w + x1] as i64;
    let z10 = a[y1 * w + x0] as i64;
    let z11 = a[y1 * w + x1] as i64;
    let wx0 = (FP_ONE - xr) as i64;
    let wx1 = xr as i64;
    let wy0 = (FP_ONE - yr) as i64;
    let wy1 = yr as i64;
    let top = z00 * wx0 + z01 * wx1;
    let bot = z10 * wx0 + z11 * wx1;
    let v = (top * wy0 + bot * wy1) / (FP_ONE as i64 * FP_ONE as i64);
    v as i32
}

/// One Lucas-Kanade step on a `w x h` image. Returns (du, dv, det, sxx*syy)
/// in fixed point.
fn lk_step(a: &[u8], b: &[u8], w: usize, h: usize, u: i32, v: i32) -> (i32, i32, i64, i64) {
    let mut sxx: i64 = 0;
    let mut sxy: i64 = 0;
    let mut syy: i64 = 0;
    let mut sx: i64 = 0;
    let mut sy: i64 = 0;
    // Derivatives need one neighbour on each side, so the loop runs a pixel
    // inside the image edges.
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let i = y * w + x;
            let xs = (x as i32) * FP_ONE + u;
            let ys = (y as i32) * FP_ONE + v;
            let is_val = sample_bilin(a, w, h, xs, ys);
            // Gradients are evaluated at the current (warped) position, not at
            // the pixel grid: with the base-position gradients the update stops
            // descending once the remaining error is about a pixel, which is
            // exactly the case each pyramid level hands to the next.
            let ix = (sample_bilin(a, w, h, xs + FP_ONE, ys)
                - sample_bilin(a, w, h, xs - FP_ONE, ys)) as i64
                / 2;
            let iy = (sample_bilin(a, w, h, xs, ys + FP_ONE)
                - sample_bilin(a, w, h, xs, ys - FP_ONE)) as i64
                / 2;
            let dt = b[i] as i64 - is_val as i64;
            sxx += ix * ix;
            sxy += ix * iy;
            syy += iy * iy;
            sx += ix * dt;
            sy += iy * dt;
        }
    }
    let det = sxx * syy - sxy * sxy;
    let mut du = 0i32;
    let mut dv = 0i32;
    if det > 0 {
        // solution is in pixels; scale to 1/64-pixel fixed point
        let du64 = ((sx * syy - sxy * sy) * FP_ONE as i64) / det;
        let dv64 = ((sxx * sy - sxy * sx) * FP_ONE as i64) / det;
        du = du64 as i32;
        dv = dv64 as i32;
    }
    (du, dv, det, sxx * syy)
}

/// Iterated Lucas-Kanade on a `w x h` image, starting from the guess
/// `(u0, v0)` in 1/64-pixel units. Returns the absolute `(u, v, det, prod)`.
fn lk_at(
    a: &[u8],
    b: &[u8],
    w: usize,
    h: usize,
    u0: i32,
    v0: i32,
    iters: u32,
) -> (i32, i32, i64, i64) {
    let mut u = u0;
    let mut v = v0;
    let mut det = 0i64;
    let mut prod = 0i64;
    for _ in 0..iters.max(1) {
        if u.abs() > 32 * FP_ONE || v.abs() > 32 * FP_ONE {
            break;
        }
        let (du, dv, d, p) = lk_step(a, b, w, h, u, v);
        u += du;
        v += dv;
        det = d;
        prod = p;
        if du == 0 && dv == 0 {
            break;
        }
    }
    (u, v, det, prod)
}

/// Sum of squared differences between `a` and `b` shifted by `(u, v)`:
/// `b(x) - a(x + (u, v))`, the same convention as `lk_step`.
fn warp_cost(a: &[u8], b: &[u8], w: usize, h: usize, u: i32, v: i32) -> i64 {
    let mut cost = 0i64;
    for y in 0..h {
        for x in 0..w {
            let xs = (x as i32) * FP_ONE + u;
            let ys = (y as i32) * FP_ONE + v;
            let d = b[y * w + x] as i64 - sample_bilin(a, w, h, xs, ys) as i64;
            cost += d * d;
        }
    }
    cost
}

// ---------------------------------------------------------------------------
// Pipeline state (portable)
// ---------------------------------------------------------------------------

/// Pure validity-hysteresis decision. Returns `(ok, updated_bad_streak)`.
///
/// * A good frame always validates and resets the bad-frame counter.
/// * A bad frame while already valid is tolerated until `hysteresis`
///   consecutive bad frames pile up; then it turns invalid.
/// * A bad frame while invalid stays invalid — a signal that is persistently
///   bad never validates itself (unlike a naive "> N bad -> valid" rule).
pub fn apply_hysteresis(
    now_ok: bool,
    was_valid: bool,
    bad_streak: u32,
    hysteresis: u32,
) -> (bool, u32) {
    let streak = if now_ok { 0 } else { bad_streak.saturating_add(1) };
    let ok = now_ok || (was_valid && streak <= hysteresis);
    (ok, streak)
}

/// Frame-by-frame estimator state. Share this on the MCU and call `process`
/// from the frame-capture callback; no threads or heap involved.
pub struct VisualPipeline {
    cfg: VisualConfig,
    /// Previous and current frame as image pyramids, kept in place so the
    /// multi-scale estimate never touches the heap.
    prev: Pyramid,
    cur: Pyramid,
    have_prev: bool,
    /// Previous frame's raw estimate (level-0 1/64-pixel units), reused as
    /// the next frame's initial guess.
    last_est: (i32, i32),
    dx_sm: f32,
    dy_sm: f32,
    bad_streak: u32,
    rest_streak: u32,
    was_valid: bool,
}

impl VisualPipeline {
    pub fn new(cfg: VisualConfig) -> Self {
        Self {
            cfg,
            prev: Pyramid::empty(),
            cur: Pyramid::empty(),
            have_prev: false,
            last_est: (0, 0),
            dx_sm: 0.0,
            dy_sm: 0.0,
            bad_streak: 0,
            rest_streak: 0,
            was_valid: true,
        }
    }

    #[allow(dead_code)]
    pub fn config(&self) -> &VisualConfig {
        &self.cfg
    }

    pub fn set_config(&mut self, cfg: VisualConfig) {
        self.cfg = cfg;
    }

    pub fn process(&mut self, frame: &Frame) -> VisualOutput {
        let mut out = VisualOutput::default();
        let f = if self.cfg.prefilter { blur3x3(frame) } else { *frame };
        self.cur.build(&f);

        if self.have_prev {
            let tex = variance(&f);

            let (raw_x, raw_y, confidence, motion_ok) = if tex < self.cfg.texture_min {
                (0i32, 0i32, 0u32, false)
            } else {
                let pred = if self.cfg.predict { self.last_est } else { (0, 0) };
                let (u, v, det, prod) = self.estimate(pred);
                self.last_est = (u, v);
                let cond = if prod > 0 {
                    (det.abs() as f32 / prod as f32).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let conf = (cond * 1000.0) as u32;
                let (rx, ry) = (u, v);
                // Validity hysteresis: an already-valid signal tolerates up to
                // `hysteresis` consecutive bad frames before flipping invalid
                // (brief dips do not flicker `valid`); re-entering valid
                // requires a genuinely good frame again. A persistently bad
                // signal therefore STAYS invalid.
                let now_ok = conf >= self.cfg.conf_min;
                let (ok, streak) = apply_hysteresis(
                    now_ok,
                    self.was_valid,
                    self.bad_streak,
                    self.cfg.hysteresis,
                );
                self.bad_streak = streak;
                self.was_valid = ok;
                (rx, ry, conf, ok)
            };

            let mut dx = raw_x;
            let mut dy = raw_y;
            if self.cfg.deadband > 0 {
                if dx.abs() < self.cfg.deadband {
                    dx = 0;
                }
                if dy.abs() < self.cfg.deadband {
                    dy = 0;
                }
            }
            if self.cfg.swap_xy {
                std::mem::swap(&mut dx, &mut dy);
            }
            if self.cfg.invert_x {
                dx = -dx;
            }
            if self.cfg.invert_y {
                dy = -dy;
            }

            let mut cxx = dx as f32 / FP_ONE as f32 * self.cfg.gain;
            let mut cyy = dy as f32 / FP_ONE as f32 * self.cfg.gain;
            // Rest detection: after a few frames at/below the deadband, snap
            // the smoothed output (and its state) back to exactly zero.
            if dx == 0 && dy == 0 {
                self.rest_streak += 1;
            } else {
                self.rest_streak = 0;
            }
            if self.rest_streak >= self.cfg.rest_frames {
                cxx = 0.0;
                cyy = 0.0;
                self.dx_sm = 0.0;
                self.dy_sm = 0.0;
            } else if self.cfg.smoothing > 0.0 {
                let s = self.cfg.smoothing.clamp(0.0, 0.99);
                self.dx_sm = s * self.dx_sm + (1.0 - s) * cxx;
                self.dy_sm = s * self.dy_sm + (1.0 - s) * cyy;
                cxx = self.dx_sm;
                cyy = self.dy_sm;
            } else {
                self.dx_sm = cxx;
                self.dy_sm = cyy;
            }

            out.dx = dx;
            out.dy = dy;
            out.dx_counts = cxx;
            out.dy_counts = cyy;
            out.confidence = confidence;
            // A confidence-gated signal that genuinely reports motion (raw
            // estimate beyond the deadband) must still be delivered, even on
            // surfaces where the confidence metric stays low (e.g. SAD/Proj
            // on a smooth surface).
            out.valid = motion_ok || dx != 0 || dy != 0;
        } else {
            out.confidence = 1000;
            out.valid = true;
        }

        std::mem::swap(&mut self.prev, &mut self.cur);
        self.have_prev = true;
        out
    }

    /// Coarse-to-fine relative estimate between `prev` and `cur`, starting
    /// from `pred` (level-0 1/64-pixel units). Returns the finest level's
    /// `(u, v, det, sxx*syy)`.
    ///
    /// The coarsest level is searched exhaustively over integer shifts rather
    /// than started at zero: on the dot-pattern texture the gradient at the
    /// true displacement often points into a neighbouring valley, so a
    /// gradient-only start locks onto a wrong period. `pred` competes for the
    /// seed on equal terms, and each level keeps whichever of its seed and its
    /// own result matches better, which is also what drops a prediction that
    /// turned out wrong. The final shift is discarded when it no longer
    /// matches at level 0 at all.
    fn estimate(&self, pred: (i32, i32)) -> (i32, i32, i64, i64) {
        let levels = (self.cfg.pyramid_levels as usize).clamp(1, PYRAMID_MAX_LEVELS);
        let coarse = levels - 1;
        let (cw, ch) = Pyramid::dims(coarse);
        let a = self.prev.level(coarse);
        let b = self.cur.level(coarse);

        // level-0 seed: exhaustive integer search at the coarsest level
        let mut u;
        let mut v;
        if coarse > 0 {
            let sc = 1i32 << coarse;
            let r = cw as i32 / 2;
            let mut best = (0i32, 0i32);
            let mut best_cost = warp_cost(a, b, cw, ch, 0, 0);
            for vv in -r..=r {
                for uu in -r..=r {
                    if uu == 0 && vv == 0 {
                        continue;
                    }
                    let c = warp_cost(a, b, cw, ch, uu * FP_ONE, vv * FP_ONE);
                    if c < best_cost {
                        best_cost = c;
                        best = (uu * FP_ONE, vv * FP_ONE);
                    }
                }
            }
            if pred != (0, 0) {
                let p = (pred.0 / sc, pred.1 / sc);
                if warp_cost(a, b, cw, ch, p.0, p.1) < best_cost {
                    best = p;
                }
            }
            u = best.0 * sc;
            v = best.1 * sc;
        } else {
            u = pred.0;
            v = pred.1;
        }

        let mut det = 0i64;
        let mut prod = 0i64;
        for l in (0..levels).rev() {
            let (w, h) = Pyramid::dims(l);
            let sc = 1i32 << l;
            let la = self.prev.level(l);
            let lb = self.cur.level(l);
            let seed = (u / sc, v / sc);
            let (nu, nv, d, p) = lk_at(la, lb, w, h, seed.0, seed.1, self.cfg.lk_iter);
            if warp_cost(la, lb, w, h, nu, nv) < warp_cost(la, lb, w, h, seed.0, seed.1) {
                u = nu * sc;
                v = nv * sc;
            }
            if l == 0 {
                det = d;
                prod = p;
            }
        }

        if u != 0 || v != 0 {
            let (w, h) = Pyramid::dims(0);
            let a = self.prev.level(0);
            let b = self.cur.level(0);
            if warp_cost(a, b, w, h, u, v) > warp_cost(a, b, w, h, 0, 0) {
                u = 0;
                v = 0;
            }
        }

        (u, v, det, prod)
    }
}

// ---------------------------------------------------------------------------
// Host thread wrapper
// ---------------------------------------------------------------------------

/// Latest pipeline output plus a version bumped on every update, so callers
/// can tell "new data" from "same data" (mirrors `FrameReader::poll`).
struct SharedOutput {
    out: VisualOutput,
    version: u64,
}

/// Runs `VisualPipeline` on a dedicated thread, consuming frames as the HID
/// reader publishes them. Host-only.
pub struct Visual {
    output: Arc<Mutex<SharedOutput>>,
    handle: Option<thread::JoinHandle<()>>,
}

pub type FrameReceiver = Receiver<(u64, Frame)>;

impl Visual {
    /// Spawn the processing thread. `stop` is shared with the reader; setting
    /// it stops the thread. The config is shared so it can be retuned live.
    pub fn start(cfg: Arc<Mutex<VisualConfig>>, rx: FrameReceiver, stop: Arc<AtomicBool>) -> Self {
        let output = Arc::new(Mutex::new(SharedOutput { out: VisualOutput::default(), version: 0 }));
        let out_c = output.clone();
        let cfg_c = cfg.clone();
        let handle = thread::spawn(move || Self::run(cfg_c, rx, out_c, stop));
        Self { output, handle: Some(handle) }
    }

    fn run(
        cfg: Arc<Mutex<VisualConfig>>,
        rx: FrameReceiver,
        output: Arc<Mutex<SharedOutput>>,
        stop: Arc<AtomicBool>,
    ) {
        let mut pipe = VisualPipeline::new(*cfg.lock().unwrap());
        loop {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            match rx.recv_timeout(Duration::from_millis(30)) {
                Ok((_, frame)) => {
                    pipe.set_config(*cfg.lock().unwrap());
                    let out = pipe.process(&frame);
                    if let Ok(mut o) = output.lock() {
                        o.out = out;
                        o.version += 1;
                    }
                    // Drain any backlog so the latest frame wins and latency stays low.
                    while let Ok((_, f)) = rx.try_recv() {
                        if let Ok(mut o) = output.lock() {
                            o.out = pipe.process(&f);
                            o.version += 1;
                        }
                    }
                }
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            }
        }
    }

    pub fn snapshot(&self) -> VisualOutput {
        self.output.lock().unwrap().out
    }

    /// Like `FrameReader::poll`: returns the output only when the visual
    /// thread has produced a new one since the caller last saw it. `seen`
    /// holds the caller's last-seen version.
    pub fn poll_output(&self, seen: &mut u64) -> Option<VisualOutput> {
        let o = self.output.lock().unwrap();
        if o.version == *seen {
            return None;
        }
        *seen = o.version;
        Some(o.out)
    }

    pub fn stop(mut self) {
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic, aperiodic, band-limited texture: random values on a
    /// coarse grid, bilinearly upsampled, plus a finer octave — structure at
    /// every pyramid level without the periodicity of a sinusoid pattern.
    fn synth(seed: u32) -> Frame {
        let mut s = seed | 1;
        let mut next = move || {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            s
        };
        const COARSE: usize = 6;
        const FINE: usize = 12;
        let mut g = [0u8; COARSE * COARSE];
        for v in g.iter_mut() {
            *v = (next() >> 24) as u8;
        }
        let mut h = [0u8; FINE * FINE];
        for v in h.iter_mut() {
            *v = (next() >> 24) as u8;
        }
        let sample = |grid: &[u8], n: usize, x: usize, y: usize| -> f32 {
            let fx = x as f32 * (n - 1) as f32 / (FRAME_W - 1) as f32;
            let fy = y as f32 * (n - 1) as f32 / (FRAME_H - 1) as f32;
            let x0 = (fx as usize).min(n - 1);
            let y0 = (fy as usize).min(n - 1);
            let x1 = (x0 + 1).min(n - 1);
            let y1 = (y0 + 1).min(n - 1);
            let tx = fx - x0 as f32;
            let ty = fy - y0 as f32;
            let top = grid[y0 * n + x0] as f32 * (1.0 - tx) + grid[y0 * n + x1] as f32 * tx;
            let bot = grid[y1 * n + x0] as f32 * (1.0 - tx) + grid[y1 * n + x1] as f32 * tx;
            top * (1.0 - ty) + bot * ty
        };
        let mut f = [0u8; FRAME_LEN];
        for y in 0..FRAME_H {
            for x in 0..FRAME_W {
                let v = 0.75 * sample(&g, COARSE, x, y) + 0.25 * sample(&h, FINE, x, y);
                f[y * FRAME_W + x] = v.clamp(0.0, 255.0) as u8;
            }
        }
        f
    }

    /// `out(x, y) = frame(x + dx, y + dy)`: the shift the estimator reports.
    fn shift(frame: &Frame, dx: i32, dy: i32) -> Frame {
        let mut out = [0u8; FRAME_LEN];
        for y in 0..FRAME_H {
            for x in 0..FRAME_W {
                let sx = (x as i32 + dx).clamp(0, FRAME_W as i32 - 1) as usize;
                let sy = (y as i32 + dy).clamp(0, FRAME_H as i32 - 1) as usize;
                out[y * FRAME_W + x] = frame[sy * FRAME_W + sx];
            }
        }
        out
    }

    #[test]
    fn pyramid_tracks_large_shifts() {
        let base = synth(0x1234_5678);
        for (dx, dy) in [(5, 0), (0, -5), (6, 4), (-7, 3)] {
            let mut pipe = VisualPipeline::new(VisualConfig::default());
            let _ = pipe.process(&base);
            let out = pipe.process(&shift(&base, dx, dy));
            let px = out.dx as f32 / FP_ONE as f32;
            let py = out.dy as f32 / FP_ONE as f32;
            assert!(
                (px - dx as f32).abs() < 0.75 && (py - dy as f32).abs() < 0.75,
                "shift ({dx},{dy}) estimated ({px:.2},{py:.2})"
            );
        }
    }

    #[test]
    #[ignore = "debug aid"]
    fn dbg_l64() {
        let base = synth(0x1234_5678);
        let s = shift(&base, 6, 4);
        let a = blur3x3(&base);
        let b = blur3x3(&s);
        let mut ap = Pyramid::empty();
        ap.build(&a);
        let mut bp = Pyramid::empty();
        bp.build(&b);
        for l in (0..PYRAMID_MAX_LEVELS).rev() {
            let (w, h) = Pyramid::dims(l);
            eprintln!("--- L{l} {w}x{h} truth ({},{})", 6 >> l, 4 >> l);
            for r in -4i32..=4 {
                let mut row = String::new();
                for c in -4i32..=4 {
                    let cost = warp_cost(ap.level(l), bp.level(l), w, h, r * FP_ONE, c * FP_ONE);
                    row.push_str(&format!("{:>8}", cost / 1000));
                }
                eprintln!("{row}");
            }
            let (u, v, _, _) = lk_at(ap.level(l), bp.level(l), w, h, 0, 0, 5);
            eprintln!("lk from 0 -> ({:.2},{:.2})", u as f32 / FP_ONE as f32, v as f32 / FP_ONE as f32);
        }
    }

    #[test]
    #[ignore = "debug aid"]
    fn dbg_matrix() {
        let base = synth(0x1234_5678);
        for (dx, dy) in [(5, 0), (0, -5), (6, 4), (-7, 3), (8, 8), (2, -3)] {
            for levels in [1u8, 2, 3] {
                let mut cfg = VisualConfig::default();
                cfg.pyramid_levels = levels;
                cfg.predict = false;
                let mut pipe = VisualPipeline::new(cfg);
                let _ = pipe.process(&base);
                let o = pipe.process(&shift(&base, dx, dy));
                eprintln!(
                    "shift ({dx:>3},{dy:>3}) levels {levels}: -> ({:>6.2},{:>6.2}) conf {:>4} valid {}",
                    o.dx as f32 / FP_ONE as f32,
                    o.dy as f32 / FP_ONE as f32,
                    o.confidence,
                    o.valid
                );
            }
        }
    }

    #[test]
    #[ignore = "debug aid"]
    fn dbg_guard() {
        let base = synth(0x1234_5678);
        let s = shift(&base, 6, 4);
        let mut cfg = VisualConfig::default();
        cfg.lk_iter = 5;
        let mut pipe = VisualPipeline::new(cfg);
        let _ = pipe.process(&base);
        let o = pipe.process(&s);
        eprintln!("reported dx {:.2} dy {:.2}", o.dx as f32 / FP_ONE as f32, o.dy as f32 / FP_ONE as f32);
        // after process() the pyramids are swapped: prev = shifted, cur = base
        let c0 = warp_cost(pipe.cur.level(0), pipe.prev.level(0), FRAME_W, FRAME_H, 0, 0);
        eprintln!("cost at 0 = {c0}");
        for (u, v) in [(-13.42f32, -2.56f32), (6.0, 4.0), (-6.0, -4.0)] {
            let cost = warp_cost(
                pipe.cur.level(0),
                pipe.prev.level(0),
                FRAME_W,
                FRAME_H,
                (u * FP_ONE as f32) as i32,
                (v * FP_ONE as f32) as i32,
            );
            eprintln!("cost at ({u},{v}) = {cost}");
        }
        let (u, v, _, _) = pipe.estimate((0, 0));
        eprintln!("estimate (swapped) {:.2} {:.2}", u as f32 / FP_ONE as f32, v as f32 / FP_ONE as f32);
    }

}
