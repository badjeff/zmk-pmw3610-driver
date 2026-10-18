//! Headless pipeline: reads PMW3610 frames, runs the visual estimator, and
//! drives the OS cursor. All window/rendering code lives in `ui`; this module
//! only wires the sensor pipeline together.

mod hid;
mod ui;
mod visual;

use anyhow::Result;
use enigo::{Coordinate, Enigo, Mouse};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use hid::{FrameReader, FRAME_LEN};
use visual::{VisualConfig, VisualOutput};

// OS cursor follow (toggle with T). `v.dx_counts`/`v.dy_counts` are already in
// sensor pixel counts; scale them into OS pixels here.
const CURSOR_GAIN: f32 = 5.0;
const CURSOR_MAX_DELTA: i32 = 50;
const CURSOR_FLIP_X: bool = true;
const CURSOR_FLIP_Y: bool = true;

/// Convert one visual frame's relative counts into a synthetic OS cursor move.
/// Origin of the screen (and enigo's coordinate space) is top-left, y grows
/// downward. Callers gate on new output (`Visual::poll_output`) so each
/// snapshot moves the cursor once.
fn apply_cursor(enigo: &mut Enigo, v: &VisualOutput, acc: &mut (f32, f32), warned: &mut bool) {
    if !v.valid {
        return;
    }
    let mut dx = v.dx_counts * CURSOR_GAIN;
    let mut dy = v.dy_counts * CURSOR_GAIN;
    if CURSOR_FLIP_X {
        dx = -dx;
    }
    if CURSOR_FLIP_Y {
        dy = -dy;
    }
    // Accumulate sub-pixel remainders so slow drift still moves the cursor.
    acc.0 += dx;
    acc.1 += dy;
    let mx = acc.0.round();
    let my = acc.1.round();
    if mx == 0.0 && my == 0.0 {
        return;
    }
    let cx = (mx as i32).clamp(-CURSOR_MAX_DELTA, CURSOR_MAX_DELTA);
    let cy = (my as i32).clamp(-CURSOR_MAX_DELTA, CURSOR_MAX_DELTA);
    if let Err(e) = enigo.move_mouse(cx, cy, Coordinate::Rel)
        && !*warned
    {
        *warned = true;
        eprintln!("cursor move failed (grant accessibility/input monitoring?): {e}");
    }
    acc.0 -= mx;
    acc.1 -= my;
}

fn main() -> Result<()> {
    let mut reader = FrameReader::open()?;
    let rx = reader.subscribe();

    let vis_cfg = Arc::new(Mutex::new(VisualConfig::default()));
    let vis_stop = Arc::new(AtomicBool::new(false));
    let visual = visual::Visual::start(vis_cfg.clone(), rx, vis_stop.clone());

    let mut enigo =
        Enigo::new(&enigo::Settings::default()).map_err(|e| anyhow::anyhow!("enigo init failed: {e}"))?;
    let mut ui = ui::Ui::new()?;

    let mut frame_data = [0u8; FRAME_LEN];

    let mut fps_frames: u32 = 0;
    let mut fps_window = Instant::now();
    let mut cursor_enabled = false;
    let mut cursor_acc = (0.0f32, 0.0f32);
    let mut cursor_seen: u64 = 0;
    let mut cursor_warned = false;

    loop {
        let delta = reader.poll(&mut frame_data);
        fps_frames += delta as u32;

        if cursor_enabled
            && let Some(v) = visual.poll_output(&mut cursor_seen)
        {
            apply_cursor(&mut enigo, &v, &mut cursor_acc, &mut cursor_warned);
        }

        let v = visual.snapshot();

        if fps_window.elapsed() >= Duration::from_secs(1) {
            fps_frames = 0;
            fps_window = Instant::now();
        }

        let fps_now = (fps_frames as f64 / fps_window.elapsed().as_secs_f64()).floor();
        let cur_state = if cursor_enabled { "on" } else { "off" };
        let status = format!("cur:{:>3}  fps:{:>2}", cur_state, fps_now);
        let sub = format!("x:{:>7.1}     y:{:>7.1}", v.dx_counts, v.dy_counts);
        let help = "[T]cursor  [Q]quit";

        let inp = ui.poll();
        if inp.toggle_cursor {
            cursor_enabled = !cursor_enabled;
            cursor_acc = (0.0, 0.0);
            visual.poll_output(&mut cursor_seen); // skip the output current at enable time
        }

        ui.render(&frame_data, &v, &status, &sub, help)?;

        if inp.quit {
            break;
        }
    }

    reader.stop();
    vis_stop.store(true, Ordering::Relaxed);
    visual.stop();
    Ok(())
}