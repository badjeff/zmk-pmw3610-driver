//! Window, layout and rendering for the PMW3610 debug console.
//!
//! Everything the user sees is drawn here; the estimator logic in `visual` and
//! the headless pipeline in `main` stay independent of minifb.

use anyhow::Result;
use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Window, WindowOptions};

use crate::hid::{FRAME_H, FRAME_LEN, FRAME_W};
use crate::visual::VisualOutput;

const SCALE: usize = 20;

const FRAME_PX_W: usize = FRAME_W * SCALE;
const FRAME_PX_H: usize = FRAME_H * SCALE;

const MARGIN: usize = 16;
const PANEL_PAD: usize = 14;
const BAR_GAP: usize = 7;
const BAR_H: usize = 26;
const ARROW_GAP: usize = 3;
const ARROW_H: usize = 9;
const ARROW_BAND: usize = ARROW_H + 1;

const TRACK_H: usize = 12;
const TRACK_GAP: usize = 4;
const MOTION_GAP: usize = 12;
const NUM_TRACKS: usize = 2;
const TRACKS_BODY_H: usize = NUM_TRACKS * TRACK_H + (NUM_TRACKS - 1) * TRACK_GAP;

const TEXT_GAP: usize = 24;
const TEXT_LINE_H: usize = 8;
const TEXT_LINE_GAP: usize = 8;
const TEXT_LINES: usize = 3;
const TEXT_H: usize = TEXT_LINES * TEXT_LINE_H + (TEXT_LINES - 1) * TEXT_LINE_GAP;

const CONTENT_X: usize = MARGIN + PANEL_PAD;
const CONTENT_Y: usize = MARGIN + PANEL_PAD;
/// Left-hand label column for the gauge sliders / level bar.
const TRACK_LABEL_W: usize = 30;
const BAR_X: usize = CONTENT_X + TRACK_LABEL_W;
const BAR_W: usize = FRAME_PX_W - TRACK_LABEL_W;
const BAR_Y: usize = CONTENT_Y + FRAME_PX_H + BAR_GAP;
const ARROW_Y: usize = BAR_Y + BAR_H + ARROW_GAP;
const TRACK_Y0: usize = ARROW_Y + ARROW_BAND + MOTION_GAP;
const TEXT_Y: usize = TRACK_Y0 + TRACKS_BODY_H + TEXT_GAP;

const PANEL_W: usize = FRAME_PX_W + 2 * PANEL_PAD;
const PANEL_H: usize = PANEL_PAD * 2
    + FRAME_PX_H
    + BAR_GAP
    + BAR_H
    + ARROW_GAP
    + ARROW_BAND
    + MOTION_GAP
    + TRACKS_BODY_H
    + TEXT_GAP
    + TEXT_H;
const WIN_W: usize = MARGIN * 2 + PANEL_W;
const WIN_H: usize = MARGIN * 2 + PANEL_H;

const WINDOW_BG: u32 = 0x00181818;
const PANEL_BG: u32 = 0x00242424;
const PANEL_BORDER: u32 = 0x00323232;
const FRAME_BORDER: u32 = 0x003C3C3C;
const BAR_BORDER: u32 = 0x003C3C3C;
const TRACK_COLOR: u32 = 0x00505050;
const ARROW_COLOR: u32 = 0x00E0E0E0;
const ARROW_HOT: u32 = 0x00FFFFFF;

const TRACK_BG: u32 = 0x002A2A2A;
const TRACK_CENTER: u32 = 0x005E5E5E;
const REL_X_COLOR: u32 = 0x00CF9BFF;
const REL_Y_COLOR: u32 = 0x0055CCFF;
const TRACK_OFF: u32 = 0x00404040;
const TEXT_COLOR: u32 = 0x00D0D0D0;
const HELP_COLOR: u32 = 0x00FFFFFF;

const REL_FULL_SCALE: f32 = 24.0;

const FRAME_MAX: u8 = 0xff; // max 8-bit pixel value

// Level shift (in pixel counts) applied to the gradient projection at the ends
// of the arrow track. The track centre is 0 (no shift); moving the arrow shifts
// the linear gradient darker (left) or brighter (right).
const LEVEL_RANGE: i32 = 128;

/// Input decoded from the window for the current frame.
#[derive(Debug, Default)]
pub struct Input {
    pub toggle_cursor: bool,
    pub quit: bool,
}

/// Owns the minifb window, pixel buffer and level-slider interaction state.
pub struct Ui {
    window: Window,
    pixels: Vec<u32>,
    level: i32,
    dragging: bool,
}

impl Ui {
    pub fn new() -> Result<Self> {
        let mut window = Window::new(
            "PMW3610 frame",
            WIN_W,
            WIN_H,
            WindowOptions {
                resize: false,
                ..WindowOptions::default()
            },
        )?;
        window.set_target_fps(60);
        Ok(Self {
            window,
            pixels: vec![0u32; WIN_W * WIN_H],
            level: 0,
            dragging: false,
        })
    }

    /// Pump window events and decode keyboard/mouse input.
    pub fn poll(&mut self) -> Input {
        self.poll_level_drag();
        Input {
            toggle_cursor: self.window.is_key_pressed(Key::T, KeyRepeat::No),
            quit: self.window.is_key_pressed(Key::Q, KeyRepeat::No) || !self.window.is_open(),
        }
    }

    /// Drag the level arrow: press inside its band to grab it, drag horizontally
    /// to shift the gradient, release to drop.
    fn poll_level_drag(&mut self) {
        if let Some((mx, my)) = self.window.get_mouse_pos(MouseMode::Clamp) {
            let down = self.window.get_mouse_down(MouseButton::Left);
            if !down {
                self.dragging = false;
            } else if self.dragging {
                self.level = x_to_level(mx);
            } else {
                let mxi = mx.max(0.0) as usize;
                let myi = my.max(0.0) as usize;
                let in_band = (ARROW_Y..ARROW_Y + ARROW_BAND).contains(&myi);
                let near = mxi + ARROW_H >= BAR_X && mxi <= BAR_X + BAR_W + ARROW_H;
                if in_band && near {
                    self.dragging = true;
                    self.level = x_to_level(mx);
                }
            }
        }
    }

    pub fn set_title(&mut self, fps: f64) {
        self.window.set_title(&format!("PMW3610 | {:.1} fps", fps));
    }

    /// Render one frame's view and present it.
    pub fn render(
        &mut self,
        frame: &[u8; FRAME_LEN],
        v: &VisualOutput,
        status: &str,
        sub: &str,
        help: &str,
    ) -> Result<()> {
        render_ui(&mut self.pixels, frame, self.level, self.dragging, v, status, sub, help);
        self.window.update_with_buffer(&self.pixels, WIN_W, WIN_H)?;
        Ok(())
    }
}

/// 5x7 bitmap font, one `u8` per row, LSB = leftmost column.
fn glyph(ch: char) -> [u8; 7] {
    match ch.to_ascii_uppercase() {
        ' ' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        '0' => [0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e],
        '1' => [0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e],
        '2' => [0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f],
        '3' => [0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e],
        '4' => [0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02],
        '5' => [0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e],
        '6' => [0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e],
        '7' => [0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e],
        '9' => [0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c],
        'A' => [0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11],
        'B' => [0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e],
        'C' => [0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e],
        'D' => [0x1c, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1c],
        'E' => [0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f],
        'F' => [0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10],
        'G' => [0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f],
        'H' => [0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11],
        'I' => [0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e],
        'J' => [0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0c],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f],
        'M' => [0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e],
        'P' => [0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10],
        'Q' => [0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d],
        'R' => [0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11],
        'S' => [0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e],
        'T' => [0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a],
        'X' => [0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11],
        'Y' => [0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04],
        'Z' => [0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f],
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06],
        ',' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08],
        '-' => [0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00],
        '/' => [0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10],
        ':' => [0x00, 0x06, 0x06, 0x00, 0x06, 0x06, 0x00],
        '|' => [0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        '[' => [0x0e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0e],
        ']' => [0x0e, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0e],
        '%' => [0x19, 0x19, 0x02, 0x04, 0x08, 0x13, 0x13],
        _ => [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    }
}

fn draw_text(buf: &mut [u32], s: &str, x: usize, y: usize, color: u32) {
    let mut cx = x;
    for ch in s.chars() {
        let g = glyph(ch);
        for (r, &bits) in g.iter().enumerate() {
            for col in 0..5u8 {
                // bit 4 is the leftmost column of each row
                if bits & (1 << (4 - col)) != 0 {
                    let px = cx + col as usize;
                    let py = y + r;
                    if px < WIN_W && py < WIN_H {
                        buf[py * WIN_W + px] = color;
                    }
                }
            }
        }
        cx += 6;
    }
}

fn fill_rect(buf: &mut [u32], x: usize, y: usize, w: usize, h: usize, c: u32) {
    for yy in y..y + h {
        let row = yy * WIN_W;
        buf[row + x..row + x + w].fill(c);
    }
}

fn stroke_rect(buf: &mut [u32], x: usize, y: usize, w: usize, h: usize, c: u32) {
    fill_rect(buf, x, y, w, 1, c);
    fill_rect(buf, x, y + h - 1, w, 1, c);
    fill_rect(buf, x, y, 1, h, c);
    fill_rect(buf, x + w - 1, y, 1, h, c);
}

fn blit_frame(frame: &[u8; FRAME_LEN], out: &mut [u32], ox: usize, oy: usize, level: i32) {
    for y in 0..FRAME_H {
        for x in 0..FRAME_W {
            let v = (frame[y * FRAME_W + x] as i32 + level).clamp(0, FRAME_MAX as i32) as u32;
            let c = v * 0x010101;
            for sy in 0..SCALE {
                let row = (oy + y * SCALE + sy) * WIN_W + ox + x * SCALE;
                out[row..row + SCALE].fill(c);
            }
        }
    }
}

fn draw_gradient_bar(buf: &mut [u32], x: usize, y: usize, w: usize, h: usize) {
    for i in 0..w {
        let v = (i * FRAME_MAX as usize / (w - 1)) as u32;
        fill_rect(buf, x + i, y, 1, h, v * 0x010101);
    }
    stroke_rect(buf, x, y, w, h, BAR_BORDER);
}

fn x_to_level(cx: f32) -> i32 {
    let span = (BAR_W - 1) as f32;
    let t = ((cx - BAR_X as f32) / span).clamp(0.0, 1.0);
    ((t - 0.5) * 2.0 * LEVEL_RANGE as f32).round() as i32
}

fn level_to_x(level: i32) -> usize {
    let t = (level as f32 / LEVEL_RANGE as f32 * 0.5 + 0.5).clamp(0.0, 1.0);
    BAR_X + (t * (BAR_W - 1) as f32).round() as usize
}

fn draw_arrow(buf: &mut [u32], cx: usize, color: u32) {
    for i in 0..ARROW_H {
        let half = i;
        let x0 = cx.saturating_sub(half);
        fill_rect(buf, x0, ARROW_Y + i, half * 2 + 1, 1, color);
    }
}

/// Horizontal gauge: centered resting mark plus a needle whose travel is
/// `value / full_scale` of the half-span.
#[allow(clippy::too_many_arguments)]
fn draw_track(buf: &mut [u32], x: usize, y: usize, w: usize, value: f32, full_scale: f32, color: u32, valid: bool) {
    fill_rect(buf, x, y, w, TRACK_H, TRACK_BG);
    let cx = x + w / 2;
    fill_rect(buf, cx - 1, y, 2, TRACK_H, TRACK_CENTER);
    if !valid {
        fill_rect(buf, x + 2, y + TRACK_H / 2 - 1, w - 4, 2, TRACK_OFF);
        return;
    }
    let span = (w / 2 - 3) as f32;
    let v = (value / full_scale).clamp(-1.0, 1.0);
    let nx = (cx as f32 + v * span).round() as usize;
    fill_rect(buf, nx.saturating_sub(2), y + 2, 4, TRACK_H - 4, color);
}

/// Gauge label to the left of a bar/track, vertically centred on a TRACK_H
/// (or BAR_H) row.
fn draw_slider_label(buf: &mut [u32], label: &str, y: usize, h: usize, color: u32) {
    draw_text(buf, label, CONTENT_X, y + (h - TEXT_LINE_H) / 2, color);
}

#[allow(clippy::too_many_arguments)]
fn render_ui(
    buf: &mut [u32],
    frame: &[u8; FRAME_LEN],
    level: i32,
    dragging: bool,
    v: &VisualOutput,
    status: &str,
    sub: &str,
    help: &str,
) {
    buf.fill(WINDOW_BG);
    fill_rect(buf, MARGIN, MARGIN, PANEL_W, PANEL_H, PANEL_BG);
    stroke_rect(buf, MARGIN, MARGIN, PANEL_W, PANEL_H, PANEL_BORDER);

    stroke_rect(buf, CONTENT_X - 1, CONTENT_Y - 1, FRAME_PX_W + 2, FRAME_PX_H + 2, FRAME_BORDER);
    blit_frame(frame, buf, CONTENT_X, CONTENT_Y, level);

    draw_slider_label(buf, "lvl", BAR_Y, BAR_H, TEXT_COLOR);
    draw_gradient_bar(buf, BAR_X, BAR_Y, BAR_W, BAR_H);

    fill_rect(buf, BAR_X, ARROW_Y + ARROW_BAND - 1, BAR_W, 1, TRACK_COLOR);
    draw_arrow(buf, level_to_x(level), if dragging { ARROW_HOT } else { ARROW_COLOR });

    // Relative (mouse) gauges.
    draw_slider_label(buf, "r x", TRACK_Y0, TRACK_H, REL_X_COLOR);
    draw_track(buf, BAR_X, TRACK_Y0, BAR_W, v.dx_counts, REL_FULL_SCALE, REL_X_COLOR, v.valid);
    let y1 = TRACK_Y0 + TRACK_H + TRACK_GAP;
    draw_slider_label(buf, "r y", y1, TRACK_H, REL_Y_COLOR);
    draw_track(buf, BAR_X, y1, BAR_W, v.dy_counts, REL_FULL_SCALE, REL_Y_COLOR, v.valid);

    draw_text(buf, status, CONTENT_X, TEXT_Y, TEXT_COLOR);
    draw_text(buf, sub, CONTENT_X, TEXT_Y + TEXT_LINE_H + TEXT_LINE_GAP, TEXT_COLOR);
    draw_text(buf, help, CONTENT_X, TEXT_Y + 2 * (TEXT_LINE_H + TEXT_LINE_GAP), HELP_COLOR);
}