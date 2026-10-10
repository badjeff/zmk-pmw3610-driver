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

/// Number of 64-bit words needed to hold one bit per frame pixel.
const MASK_WORDS: usize = FRAME_LEN.div_ceil(64);

/// A set of ignored ("dead") frame pixels, one bit each. Bit set = dead.
///
/// Real PMW3610 frame-grabs carry defective pixels: dust, a dead photodiode or
/// dirt in the lens makes a pixel read a constant value (observed: a run of 20
/// pixels reading exactly 0 in the bottom margin). Such a pixel is a static
/// black hole in an otherwise moving image; window-search estimators can lock
/// onto its fixed edge and jump. `Mask` marks those pixels so the estimators
/// can exclude them from every gradient, sample and residual instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mask(pub [u64; MASK_WORDS]);

impl Mask {
    /// No dead pixels (all alive).
    pub const fn none() -> Self {
        Mask([0u64; MASK_WORDS])
    }

    /// Every real pixel dead (used to seed the candidate set before any frame
    /// has shown which pixels actually carry signal).
    pub const fn all() -> Self {
        let mut w = [u64::MAX; MASK_WORDS];
        // Clear padding bits past FRAME_LEN so they are never counted.
        let extra = MASK_WORDS * 64 - FRAME_LEN;
        if extra > 0 {
            w[MASK_WORDS - 1] = u64::MAX >> extra;
        }
        Mask(w)
    }

    #[inline]
    pub fn is_dead(&self, i: usize) -> bool {
        self.0[i >> 6] & (1u64 << (i & 63)) != 0
    }

    #[inline]
    pub fn set_alive(&mut self, i: usize) {
        self.0[i >> 6] &= !(1u64 << (i & 63));
    }
}

/// Tunables. Everything here is a host finetuning knob; on the MCU these map
/// to Kconfig or devicetree values.
#[derive(Debug, Clone, Copy)]
pub struct VisualConfig {
    /// Relative deadband in 1/64-pixel units; smaller motions are snapped to 0.
    pub deadband: i32,
    /// Minimum zero-mean pixel variance; below this the frame is "flat" and
    /// relative motion is rejected (confidence forced to 0).
    pub texture_min: u32,
    /// Lucas-Kanade refinement iterations.
    pub lk_iter: u32,
    /// Axis remap before gain scaling.
    pub swap_xy: bool,
    pub invert_x: bool,
    pub invert_y: bool,
    /// Pixel -> counts scale applied to the relative estimate.
    pub gain: f32,
    /// Detect dead pixels (those that never carry signal while their neighbours
    /// do). Detection is cheap and always runnable; whether the detected mask is
    /// applied to the estimators is controlled by `dead_mask_apply`.
    pub dead_pixels: bool,
    /// Apply the dead-pixel mask to the estimators (exclude each dead pixel and
    /// any pixel whose gradient stencil or warped sample touches one). Default
    /// ON: on the synthetic motion set built from real frames, excluding the
    /// dead row lifts 1-2px-motion accuracy from 16%/21% to 83%/88% (LK/HYB
    /// within 0.5px) — the static black-hole edge otherwise biases every
    /// estimate. On the at-rest capture it costs only ~0.09px on 3 of 27
    /// frames. The runtime mask flags the dead row but not the duplicated tail
    /// (those pixels carry signal), which is exactly the beneficial subset.
    pub dead_mask_apply: bool,
    /// A pixel counts as "carrying signal" once any frame value reaches this.
    /// Dead photodiodes/dust read a constant (near) zero.
    pub dead_level: u8,
    /// Frames observed before the dead-pixel mask is finalized and applied.
    pub dead_warmup: u32,
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
            lk_iter: 2,
            swap_xy: false,
            invert_x: false,
            invert_y: false,
            gain: 1.0,
            dead_pixels: true,
            dead_mask_apply: true,
            dead_level: 8,
            dead_warmup: 8,
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
    pub frames: u64,
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
            frames: 0,
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
fn variance(frame: &Frame, mask: &Mask) -> u32 {
    let mut s: i64 = 0;
    let mut n: i64 = 0;
    for (i, &z) in frame.iter().enumerate() {
        if mask.is_dead(i) {
            continue;
        }
        s += z as i64;
        n += 1;
    }
    if n == 0 {
        return 0;
    }
    let m0 = s / n;
    let mut v: i64 = 0;
    for (i, &z) in frame.iter().enumerate() {
        if mask.is_dead(i) {
            continue;
        }
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

/// Bilinear sample of a frame at fixed-point coordinates, border-replicated
/// at the frame edges. Returns `None` if any of the four bilinear corners is
/// dead, so the caller can drop that pixel from the fit instead of letting a
/// constant-zero photodiode corrupt the sampled value.
fn sample_bilin(a: &Frame, xf: i32, yf: i32, mask: &Mask) -> Option<i32> {
    let lo = 0i32;
    let hi_x = FRAME_W as i32 - 2;
    let hi_y = FRAME_H as i32 - 2;
    let xi = (xf >> FP_SHIFT).clamp(lo, hi_x);
    let yi = (yf >> FP_SHIFT).clamp(lo, hi_y);
    let xr = xf & FP_MASK;
    let yr = yf & FP_MASK;
    let x0 = xi as usize;
    let y0 = yi as usize;
    let x1 = x0 + 1;
    let y1 = y0 + 1;
    if mask.is_dead(y0 * FRAME_W + x0)
        || mask.is_dead(y0 * FRAME_W + x1)
        || mask.is_dead(y1 * FRAME_W + x0)
        || mask.is_dead(y1 * FRAME_W + x1)
    {
        return None;
    }
    let z00 = a[y0 * FRAME_W + x0] as i64;
    let z01 = a[y0 * FRAME_W + x1] as i64;
    let z10 = a[y1 * FRAME_W + x0] as i64;
    let z11 = a[y1 * FRAME_W + x1] as i64;
    let wx0 = (FP_ONE - xr) as i64;
    let wx1 = xr as i64;
    let wy0 = (FP_ONE - yr) as i64;
    let wy1 = yr as i64;
    let top = z00 * wx0 + z01 * wx1;
    let bot = z10 * wx0 + z11 * wx1;
    let v = (top * wy0 + bot * wy1) / (FP_ONE as i64 * FP_ONE as i64);
    Some(v as i32)
}

/// One Lucas-Kanade step. Returns (du, dv, det, sxx*syy) in fixed point.
fn lk_step(a: &Frame, b: &Frame, u: i32, v: i32, mask: &Mask) -> (i32, i32, i64, i64) {
    let mut sxx: i64 = 0;
    let mut sxy: i64 = 0;
    let mut syy: i64 = 0;
    let mut sx: i64 = 0;
    let mut sy: i64 = 0;
    // Derivatives need one neighbour on each side, so the loop runs a pixel
    // inside the frame edges.
    for y in 1..FRAME_H - 1 {
        for x in 1..FRAME_W - 1 {
            let i = y * FRAME_W + x;
            // Drop any pixel whose brightness-gradient stencil or warped sample
            // touches a dead photodiode (a static black hole would otherwise
            // inject a large, noise-amplified gradient term).
            if mask.is_dead(i)
                || mask.is_dead(i - 1)
                || mask.is_dead(i + 1)
                || mask.is_dead(i - FRAME_W)
                || mask.is_dead(i + FRAME_W)
            {
                continue;
            }
            let ix = (a[i + 1] as i64 - a[i - 1] as i64) / 2;
            let iy = (a[i + FRAME_W] as i64 - a[i - FRAME_W] as i64) / 2;
            let xs = (x as i32) * FP_ONE + u;
            let ys = (y as i32) * FP_ONE + v;
            let is_val = match sample_bilin(a, xs, ys, mask) {
                Some(v) => v,
                None => continue,
            };
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

/// Lucas-Kanade optical flow: returns (dx, dy, det, sxx*syy) fixed-point.
pub fn lk_motion(a: &Frame, b: &Frame, iters: u32, mask: &Mask) -> (i32, i32, i64, i64) {
    lk_motion_from(a, b, 0, 0, iters, mask)
}

/// Like `lk_motion`, but starts from an initial guess `(u0, v0)` in 1/64-pixel
/// fixed-point units. `lk_motion` is the `(0, 0)` specialization.
pub fn lk_motion_from(
    a: &Frame,
    b: &Frame,
    u0: i32,
    v0: i32,
    iters: u32,
    mask: &Mask,
) -> (i32, i32, i64, i64) {
    let mut u = u0;
    let mut v = v0;
    let mut det = 0i64;
    let mut prod = 0i64;
    for _ in 0..iters.max(1) {
        if u.abs() > 32 * FP_ONE || v.abs() > 32 * FP_ONE {
            break;
        }
        let (du, dv, d, p) = lk_step(a, b, u, v, mask);
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
    prev: Option<Frame>,
    frames: u64,
    dx_sm: f32,
    dy_sm: f32,
    bad_streak: u32,
    rest_streak: u32,
    was_valid: bool,
    /// Pixels that have not yet been seen carrying signal (candidate dead).
    dead_cand: Mask,
    /// Count of pixels seen carrying signal so far.
    dead_alive: u32,
    /// Finalized dead-pixel mask, computed once after warm-up.
    dead_final: Option<Mask>,
}

impl VisualPipeline {
    pub fn new(cfg: VisualConfig) -> Self {
        Self {
            cfg,
            prev: None,
            frames: 0,
            dx_sm: 0.0,
            dy_sm: 0.0,
            bad_streak: 0,
            rest_streak: 0,
            was_valid: true,
            dead_cand: Mask::all(),
            dead_alive: 0,
            dead_final: None,
        }
    }

    #[allow(dead_code)]
    pub fn config(&self) -> &VisualConfig {
        &self.cfg
    }

    pub fn set_config(&mut self, cfg: VisualConfig) {
        self.cfg = cfg;
    }

    #[allow(dead_code)]
    pub fn frame_count(&self) -> u64 {
        self.frames
    }

    /// Observe a raw (un-blurred) frame to learn which pixels carry signal.
    /// Runs every frame; the mask is finalized once after `dead_warmup`.
    fn learn_dead(&mut self, raw: &Frame) {
        if !self.cfg.dead_pixels || self.dead_final.is_some() {
            return;
        }
        for (i, &v) in raw.iter().enumerate() {
            if self.dead_cand.is_dead(i) && v >= self.cfg.dead_level {
                self.dead_cand.set_alive(i);
                self.dead_alive += 1;
            }
        }
        // Finalize only once warm-up elapsed AND most of the frame has been
        // seen carrying signal (a uniformly dark scene would otherwise mark
        // every pixel dead).
        if self.frames >= self.cfg.dead_warmup as u64
            && (self.dead_alive as usize) * 2 >= FRAME_LEN
        {
            self.dead_final = Some(self.dead_cand);
        }
    }

    /// The dead-pixel mask to apply, if detection is enabled and finalized.
    pub fn dead_mask(&self) -> Option<Mask> {
        if self.cfg.dead_pixels { self.dead_final } else { None }
    }

    pub fn process(&mut self, frame: &Frame) -> VisualOutput {
        self.frames += 1;
        let mut out = VisualOutput { frames: self.frames, ..VisualOutput::default() };
        self.learn_dead(frame);
        let mask = if self.cfg.dead_mask_apply {
            self.dead_mask().unwrap_or(Mask::none())
        } else {
            Mask::none()
        };
        let f = if self.cfg.prefilter { blur3x3(frame) } else { *frame };

        if let Some(prev) = self.prev.as_ref() {
            let tex = variance(&f, &mask);

            let (raw_x, raw_y, confidence, motion_ok) = if tex < self.cfg.texture_min {
                (0i32, 0i32, 0u32, false)
            } else {
                let (u, v, det, prod) = lk_motion(prev, &f, self.cfg.lk_iter, &mask);
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

        self.prev = Some(f);
        out
    }
}

// ---------------------------------------------------------------------------
// Host thread wrapper
// ---------------------------------------------------------------------------

/// Runs `VisualPipeline` on a dedicated thread, consuming frames as the HID
/// reader publishes them. Host-only.
pub struct Visual {
    output: Arc<Mutex<VisualOutput>>,
    handle: Option<thread::JoinHandle<()>>,
}

pub type FrameReceiver = Receiver<(u64, Frame)>;

impl Visual {
    /// Spawn the processing thread. `stop` is shared with the reader; setting
    /// it stops the thread. The config is shared so it can be retuned live.
    pub fn start(cfg: Arc<Mutex<VisualConfig>>, rx: FrameReceiver, stop: Arc<AtomicBool>) -> Self {
        let output = Arc::new(Mutex::new(VisualOutput::default()));
        let out_c = output.clone();
        let cfg_c = cfg.clone();
        let handle = thread::spawn(move || Self::run(cfg_c, rx, out_c, stop));
        Self { output, handle: Some(handle) }
    }

    fn run(
        cfg: Arc<Mutex<VisualConfig>>,
        rx: FrameReceiver,
        output: Arc<Mutex<VisualOutput>>,
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
                        *o = out;
                    }
                    // Drain any backlog so the latest frame wins and latency stays low.
                    while let Ok((_, f)) = rx.try_recv() {
                        if let Ok(mut o) = output.lock() {
                            *o = pipe.process(&f);
                        }
                    }
                }
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            }
        }
    }

    pub fn snapshot(&self) -> VisualOutput {
        *self.output.lock().unwrap()
    }

    pub fn stop(mut self) {
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}
