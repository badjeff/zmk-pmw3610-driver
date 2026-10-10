use anyhow::{bail, Result};
use hidapi::{HidApi, HidDevice};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub const FRAME_W: usize = 22;
pub const FRAME_H: usize = 22;
pub const FRAME_LEN: usize = FRAME_W * FRAME_H;

const VENDOR_USAGE_PAGE: u16 = 0xFF0C;
const VENDOR_USAGE: u16 = 0x01;

const MAGIC: u8 = 0xA5;
const CHUNK_PAYLOAD: usize = 56;
const CHUNKS: usize = FRAME_LEN.div_ceil(CHUNK_PAYLOAD);

const FRAME_REPORT_ID: u8 = 0x01;
const REQUEST_INTERVAL: Duration = Duration::from_millis(250);

fn request_frame(device: &HidDevice) {
    let mut buf = [0u8; 62];
    buf[0] = FRAME_REPORT_ID;
    let _ = device.get_input_report(&mut buf);
}

struct FrameAssembler {
    buf: [u8; FRAME_LEN],
    got: [bool; CHUNKS],
    seq: Option<u8>,
}

impl FrameAssembler {
    fn new() -> Self {
        Self {
            buf: [0; FRAME_LEN],
            got: [false; CHUNKS],
            seq: None,
        }
    }

    fn push(&mut self, seq: u8, chunk: u8, len: usize, payload: &[u8]) -> Option<[u8; FRAME_LEN]> {
        if self.seq != Some(seq) {
            self.seq = Some(seq);
            self.got = [false; CHUNKS];
        }

        if payload.len() < len {
            return None;
        }

        let off = chunk as usize * CHUNK_PAYLOAD;
        let n = len.min(FRAME_LEN - off);
        self.buf[off..off + n].copy_from_slice(&payload[..n]);
        self.got[chunk as usize] = true;

        if self.got.iter().all(|&g| g) {
            self.got = [false; CHUNKS];

            // let frame_hex = self.buf.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(" ");
            // println!("FRAME COMPLETE ({} bytes):\n  {}", self.buf.len(), frame_hex);

            Some(self.buf)
        } else {
            None
        }
    }
}

fn handle_report(data: &[u8], asm: &mut FrameAssembler) -> Option<[u8; FRAME_LEN]> {
    // report id may be stripped by the hidapi backend
    let m = if data.get(1) == Some(&MAGIC) {
        1
    } else if data.first() == Some(&MAGIC) {
        0
    } else {
        return None;
    };

    let seq = *data.get(m + 1)?;
    let chunk = *data.get(m + 2)?;
    let total = *data.get(m + 3)?;
    let len = *data.get(m + 4)? as usize;

    if total as usize != CHUNKS || chunk as usize >= CHUNKS {
        return None;
    }

    let end = m + 5 + len;
    let payload = data.get(m + 5..end)?;
    
    // let hex = payload.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(" ");
    // println!("CHUNK {} seq={} len={} payload_len={} off={} n={}", chunk, seq, len, payload.len(), chunk as usize * CHUNK_PAYLOAD, len.min(FRAME_LEN - chunk as usize * CHUNK_PAYLOAD));
    // println!("  PAYLOAD: {}", hex);
    
    asm.push(seq, chunk, len, payload)
}

pub type FrameMsg = (u64, [u8; FRAME_LEN]);

struct SharedFrame {
    data: [u8; FRAME_LEN],
    seq: u64,
}

/// All completed frames are fans out to the FIFO `FrameReader` state plus any
/// subscribed consumer channels (e.g. the visual processing thread).
struct Shared {
    frame: Mutex<SharedFrame>,
    subs: Mutex<Vec<Sender<FrameMsg>>>,
}

fn hid_loop(device: HidDevice, shared: Arc<Shared>, stop: Arc<AtomicBool>) {
    let mut asm = FrameAssembler::new();
    let mut read_buf = [0u8; 256];
    let mut last_request = Instant::now() - REQUEST_INTERVAL;
    while !stop.load(Ordering::Relaxed) {
        if last_request.elapsed() >= REQUEST_INTERVAL {
            request_frame(&device);
            last_request = Instant::now();
        }
        match device.read_timeout(&mut read_buf, 15) {
            Ok(n) if n > 0 => {
                if let Some(frame) = handle_report(&read_buf[..n], &mut asm)
                    && let Ok(mut s) = shared.frame.lock()
                {
                    s.data = frame;
                    s.seq = s.seq.wrapping_add(1);
                    let seq = s.seq;
                    if let Ok(subs) = shared.subs.lock() {
                        for tx in subs.iter() {
                            let _ = tx.send((seq, frame));
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(_) => thread::sleep(Duration::from_millis(20)),
        }
    }
}

pub struct FrameReader {
    shared: Arc<Shared>,
    stop: Arc<AtomicBool>,
    last_seq: u64,
}

impl FrameReader {
    pub fn open() -> Result<Self> {
        let api = HidApi::new()?;
        let mut target = None;
        for info in api.device_list() {
            if info.usage_page() == VENDOR_USAGE_PAGE && info.usage() == VENDOR_USAGE {
                target = Some(info);
            }
        }

        let device = match target {
            Some(info) => info.open_device(&api)?,
            None => bail!(
                "no device with usage page {:#06x} / usage {:#06x} found",
                VENDOR_USAGE_PAGE,
                VENDOR_USAGE
            ),
        };

        println!("\nopened {:?}, waiting for frames...", device.get_product_string()?);

        let shared = Arc::new(Shared {
            frame: Mutex::new(SharedFrame { data: [0u8; FRAME_LEN], seq: 0 }),
            subs: Mutex::new(Vec::new()),
        });
        let stop = Arc::new(AtomicBool::new(false));
        thread::spawn({
            let shared = shared.clone();
            let stop = stop.clone();
            move || hid_loop(device, shared, stop)
        });
        Ok(Self {
            shared,
            stop,
            last_seq: 0,
        })
    }

    /// Subscribe to the completed-frame stream (each frame delivered once).
    pub fn subscribe(&self) -> Receiver<FrameMsg> {
        let (tx, rx) = mpsc::channel::<FrameMsg>();
        if let Ok(mut subs) = self.shared.subs.lock() {
            subs.push(tx);
        }
        rx
    }

    pub fn poll(&mut self, out: &mut [u8; FRAME_LEN]) -> u64 {
        let mut delta = 0;
        if let Ok(s) = self.shared.frame.lock()
            && s.seq != self.last_seq
        {
            delta = s.seq - self.last_seq;
            *out = s.data;
            self.last_seq = s.seq;
        }
        delta
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}
