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

const FRAME_REPORT_ID: u8 = 0x01;
/// Report layout, mirrors `struct pmw3610_frame_report` in the firmware:
/// `report_id (1) | meta (1) | payload (CHUNK_PAYLOAD)`. The payload size is
/// static on both sides, so there is no len field.
const HEADER_LEN: usize = 2;
const CHUNK_PAYLOAD: usize = 44;
const CHUNKS: usize = FRAME_LEN.div_ceil(CHUNK_PAYLOAD);
const REQUEST_INTERVAL: Duration = Duration::from_millis(250);

// the frame must tile exactly: a short last chunk could not be expressed
// without a len field
const _: () = assert!(CHUNKS * CHUNK_PAYLOAD == FRAME_LEN);

/// The meta byte, mirrors `struct pmw3610_frame_meta`: `[recog:4 | chunk_left:4]`.
/// The fixed recognizer nibble lets the host find the header whether or not the
/// hidapi backend strips the report id (id 0x01 has a clear high nibble), and
/// chunk_left counts down `CHUNKS - 1 -> 0` (the first report of every frame).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Meta(u8);

impl Meta {
    const RECOG_MASK: u8 = 0xF0;
    const RECOG: u8 = 0xA0;
    const CHUNK_LEFT_MASK: u8 = 0x0F;

    fn is_valid(byte: u8) -> bool {
        byte & Self::RECOG_MASK == Self::RECOG
    }

    fn chunk_left(self) -> u8 {
        self.0 & Self::CHUNK_LEFT_MASK
    }
}

fn request_frame(device: &HidDevice) {
    let mut buf = [0u8; HEADER_LEN + CHUNK_PAYLOAD];
    buf[0] = FRAME_REPORT_ID;
    let _ = device.get_input_report(&mut buf);
}

struct FrameAssembler {
    buf: [u8; FRAME_LEN],
    got: [bool; CHUNKS],
}

impl FrameAssembler {
    fn new() -> Self {
        Self {
            buf: [0; FRAME_LEN],
            got: [false; CHUNKS],
        }
    }

    fn push(&mut self, chunk_left: u8, payload: &[u8]) -> Option<[u8; FRAME_LEN]> {
        // chunk_left counts down: the first report of a frame always carries
        // CHUNKS-1, which retires any partially received previous frame
        if chunk_left as usize == CHUNKS - 1 {
            self.got = [false; CHUNKS];
        }

        let idx = CHUNKS - 1 - chunk_left as usize;
        let off = idx * CHUNK_PAYLOAD;
        self.buf[off..off + CHUNK_PAYLOAD].copy_from_slice(payload);
        self.got[idx] = true;

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
    // the meta byte carries a fixed recognizer nibble, so the header can be
    // found whether or not the hidapi backend strips the report id; testing
    // index 0 first is safe because the report id (0x01) never matches it
    let m = if data.first().copied().is_some_and(Meta::is_valid) {
        0
    } else if data.first() == Some(&FRAME_REPORT_ID)
        && data.get(1).copied().is_some_and(Meta::is_valid)
    {
        1
    } else {
        return None;
    };

    let meta = Meta(*data.get(m)?);
    let chunk_left = meta.chunk_left();
    if chunk_left as usize >= CHUNKS {
        return None;
    }

    // payload size is static on both sides, no len field to consult
    let payload = data.get(m + 1..m + 1 + CHUNK_PAYLOAD)?;

    asm.push(chunk_left, payload)
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
