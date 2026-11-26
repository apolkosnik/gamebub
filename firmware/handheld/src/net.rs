//! Ethernet frames between an adapter on the dock and the running core's
//! network board (the Minimig core's A2065): the handheld's part.
//!
//! The dock sends a received frame as vendor bulk packets (`lightgun::net`:
//! pieces of at most 60 bytes); `on_packet` (the USB task) puts them
//! together and leaves the frame for this module's own thread, which writes
//! it into the core's receive buffer and tells the core its length. The
//! same thread asks the core every few milliseconds for a frame the Amiga
//! sends, reads it and sends it to the dock in pieces on the vendor bulk IN
//! endpoint.
//!
//! A thread of its own, started once at power-on with fixed buffers: a
//! frame is 26 packets on a full speed link, which is too long for the
//! worker (pad input waits there), and nothing may be allocated while a
//! core runs. It does nothing unless the core is one with the board
//! (`set_core`) and the dock has an adapter (`REQUEST_NET_LINK`).

use std::{
    sync::{
        atomic::{AtomicBool, AtomicU32, Ordering},
        Mutex,
    },
    time::{Duration, Instant},
};

use esp_idf_svc::sys;
use lightgun::net as proto;

use crate::device::{
    drivers::fpga::{FpgaSpiWordSize, SpiCommand, MAX_SPI_READ_CLOCK},
    Device,
};

/// The running core has the board.
static CORE: AtomicBool = AtomicBool::new(false);
/// The dock has an Ethernet adapter.
static LINK: AtomicBool = AtomicBool::new(false);
/// The core's board is always fitted (as its Toccata sound card is), docked
/// or not and before the machine starts, so that the Amiga finds it at any
/// reset; the dock's adapter and its cable only bring the link up. (A dock
/// takes seconds to set an adapter up after power on, and the handheld may
/// be put on the dock after the Amiga has started.)
const FITTED: bool = true;
/// The adapter has said that its cable is out.
static NO_CABLE: AtomicBool = AtomicBool::new(false);
/// What the core was last told (`STATUS`): 0 nothing yet, 1 no adapter,
/// 2 adapter and cable, 3 adapter without its cable.
static TOLD: AtomicU32 = AtomicU32::new(0);
/// What the dock last said of its adapter (`REQUEST_NET_LINK`): the link
/// flag in bit 0, then the first seven bytes of its report (stage, fault,
/// devices seen, the last one's vendor and product), and the next three
/// (its configurations, its first interface's class and subclass).
static REPORT: Mutex<(u64, [u8; REPORT_BYTES])> = Mutex::new((REPORT_NONE, [0; REPORT_BYTES]));
/// The dock's whole report (after the link flag): see the dock firmware's
/// `ecm::GetReport` (bytes 10 and up are its counts of frames).
const REPORT_BYTES: usize = 20;
const REPORT_NONE: u64 = u64::MAX;
/// Shown as notifications while this is a test build: what the dock says
/// of the adapter whenever that changes, and the board being fitted.
const SHOW_REPORTS: bool = false;
/// Frames from the dock lost: the one before was not yet in the core.
static RX_LOST: AtomicU32 = AtomicU32::new(0);

const BUFFER: usize = proto::MAX_FRAME + 2;

/// Frames from the dock: the pieces being put together, and one whole
/// frame waiting for the thread.
struct Inbox {
    reassembler: proto::Reassembler,
    frames: [[u8; BUFFER]; INBOX_FRAMES],
    lens: [usize; INBOX_FRAMES],
    head: usize,
    count: usize,
}

/// Frames that may wait for the thread (it can be a frame's time, 30 ms,
/// in sending one the other way).
const INBOX_FRAMES: usize = 4;

static INBOX: Mutex<Inbox> = Mutex::new(Inbox {
    reassembler: proto::Reassembler::new(),
    frames: [[0; BUFFER]; INBOX_FRAMES],
    lens: [0; INBOX_FRAMES],
    head: 0,
    count: 0,
});

/// Counts since the core started, shown when it ends (test builds):
/// frames into the core, frames from it to the dock, frames from it given
/// up (the dock did not take a piece), and RX_LOST.
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static TX_LOST: AtomicU32 = AtomicU32::new(0);
static RX_LATE: AtomicU32 = AtomicU32::new(0);
/// The core has ended: the thread shows the counts.
static ENDED: AtomicBool = AtomicBool::new(false);

/// The thread's own: the frame being carried either way.
static SCRATCH: Mutex<[u8; BUFFER]> = Mutex::new([0; BUFFER]);

/// How often the core is asked for a frame to send, with nothing else to do.
const IDLE_POLL: Duration = Duration::from_millis(4);
/// How long a piece may wait for the dock to take the one before.
const PIECE_TRIES: u32 = 40;

/// A vendor bulk packet from the dock with one of this module's requests
/// (the USB task: nothing here may wait).
pub fn on_packet(data: &[u8]) {
    match data.first().copied() {
        Some(proto::REQUEST_NET_LINK) => {
            let flags = data.get(proto::HEADER).copied().unwrap_or(0);
            let up = flags & proto::LINK_ADAPTER != 0;
            NO_CABLE.store(up && flags & proto::LINK_NO_CABLE != 0, Ordering::Release);
            let byte = |i: usize| data.get(proto::HEADER + 1 + i).copied().unwrap_or(0) as u64;
            let mut report = up as u64;
            for i in 0..7 {
                report |= byte(i) << (8 * (i + 1));
            }
            // (not waited for: the next link packet is a second away)
            if let Ok(mut r) = REPORT.try_lock() {
                r.0 = report;
                for i in 0..REPORT_BYTES {
                    r.1[i] = byte(i) as u8;
                }
            }
            if LINK.swap(up, Ordering::AcqRel) != up {
                log::info!(
                    "Network: the dock has {} Ethernet adapter",
                    if up { "an" } else { "no" }
                );
            }
        }
        Some(proto::REQUEST_NET) => {
            let Ok(mut inbox) = INBOX.try_lock() else {
                // (the thread is taking the frame before: this one is lost)
                RX_LOST.fetch_add(1, Ordering::Relaxed);
                return;
            };
            let inbox = &mut *inbox;
            if let Some(frame) = inbox.reassembler.push(data) {
                if inbox.count < INBOX_FRAMES {
                    let slot = (inbox.head + inbox.count) % INBOX_FRAMES;
                    inbox.frames[slot][..proto::buffer_len(frame.len())].fill(0);
                    inbox.frames[slot][..frame.len()].copy_from_slice(frame);
                    inbox.lens[slot] = frame.len();
                    inbox.count += 1;
                } else {
                    RX_LOST.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        _ => {}
    }
}

/// The handheld left the dock: its adapter went with it.
pub fn undocked() {
    LINK.store(false, Ordering::Release);
    NO_CABLE.store(false, Ordering::Release);
    if let Ok(mut r) = REPORT.try_lock() {
        r.0 = REPORT_NONE;
    }
}

/// A core is about to run (loaded and set up, the machine in it not yet
/// started): the board is fitted now when the dock has an adapter, so that
/// the Amiga finds it at its first reset. (`set_core` comes only when the
/// core already runs, and this module's thread up to 40 ms after that: a
/// fast CPU has looked through its expansion boards by then.)
pub fn before_run(core_id: &str) {
    if !lightgun::is_minimig(core_id) {
        return;
    }
    let fitted = FITTED;
    let link = LINK.load(Ordering::Acquire) && !NO_CABLE.load(Ordering::Acquire);
    let mut device = Device::lock();
    let serial = proto::serial_from(crate::hwinfo::get_serial_number().0);
    let ok = device.fpga.write_u32(proto::SERIAL, serial).is_ok()
        && device
            .fpga
            .write_u32(proto::STATUS, proto::status_link(fitted, link))
            .is_ok();
    log::info!(
        "Network: before the core runs, the board is {} ({})",
        if fitted { "fitted" } else { "not fitted" },
        if ok { "written" } else { "NOT written" }
    );
}

/// A core starts (`Some` with its ID) or ends.
pub fn set_core(core_id: Option<&str>) {
    let has = core_id.is_some_and(lightgun::is_minimig);
    if !has && CORE.load(Ordering::Acquire) {
        ENDED.store(true, Ordering::Release);
    }
    CORE.store(has, Ordering::Release);
    TOLD.store(0, Ordering::Release);
    if let Ok(mut inbox) = INBOX.lock() {
        inbox.count = 0;
    }
}

/// Start the thread (once, at power-on).
pub fn start() {
    std::thread::Builder::new()
        .name("net".into())
        .stack_size(4096)
        .spawn(run)
        .expect("net thread");
}

/// The dock's report as a notification, once it has stayed the same for a
/// second and a half (it changes several times while an adapter mounts).
struct Shown {
    shown: u64,
    last: u64,
    since: Instant,
}

fn show_report(state: &mut Shown) {
    let (report, all) = *REPORT.lock().unwrap();
    if report != state.last {
        state.last = report;
        state.since = Instant::now();
    }
    if report == REPORT_NONE {
        state.shown = REPORT_NONE;
        return;
    }
    if report == state.shown || state.since.elapsed() < Duration::from_millis(1500) {
        return;
    }
    state.shown = report;
    let b = |i: u32| (report >> (8 * i)) as u8;
    let device = format!(
        "{:02X}{:02X}:{:02X}{:02X}, {} cfg, class {:02X}/{:02X}",
        b(5),
        b(4),
        b(7),
        b(6),
        all[7],
        all[8],
        all[9]
    );
    let text = if b(0) != 0 && b(2) != 0 {
        format!("Dock network: adapter ready, fault {} ({device})", b(2))
    } else if b(0) != 0 {
        format!("Dock network: adapter ready ({device})")
    } else if b(1) == 0 {
        format!(
            "Dock network: no adapter. USB devices seen: {}, last {device}",
            b(3)
        )
    } else {
        format!(
            "Dock network: adapter at stage {}, fault {}. Devices: {}, last {device}",
            b(1),
            b(2),
            b(3)
        )
    };
    log::info!("{text}");
    crate::ui::send(crate::ui::Message::Notification(
        crate::ui::Notification::new_long(text),
    ));
}

/// The counts of the core that has ended, and the dock's, as a notification.
fn show_counts() {
    let all = REPORT.lock().unwrap().1;
    let w = |i: usize| u16::from_le_bytes([all[i], all[i + 1]]);
    let text = format!(
        "Net: core in {} out {}, lost in {}+{} out {}. Dock: in {} out {}, lost {}/{}/{}, fault {}",
        RX_FRAMES.swap(0, Ordering::Relaxed),
        TX_FRAMES.swap(0, Ordering::Relaxed),
        RX_LOST.swap(0, Ordering::Relaxed),
        RX_LATE.swap(0, Ordering::Relaxed),
        TX_LOST.swap(0, Ordering::Relaxed),
        w(10),
        w(12),
        w(16),
        w(14),
        w(18),
        all[1]
    );
    log::info!("{text}");
    // (the card is free again: the core has ended)
    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open("/sdcard/net-stats.txt")
    {
        use std::io::Write;
        let _ = writeln!(file, "{text}");
    }
    crate::ui::send(crate::ui::Message::Notification(crate::ui::Notification {
        text,
        duration: Duration::from_secs(20),
    }));
    // (a test build: where the time of the hard disks' reads went)
    if let Some(text) = crate::core::files_on_demand_stats_line() {
        log::info!("{text}");
        if let Ok(mut file) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open("/sdcard/net-stats.txt")
        {
            use std::io::Write;
            let _ = writeln!(file, "{text}");
        }
        crate::ui::send(crate::ui::Message::Notification(crate::ui::Notification {
            text,
            duration: Duration::from_secs(20),
        }));
    }
}

fn run() {
    let mut shown = Shown {
        shown: REPORT_NONE,
        last: REPORT_NONE,
        since: Instant::now(),
    };
    let mut refused = 0u32;
    loop {
        if SHOW_REPORTS {
            show_report(&mut shown);
            if ENDED.swap(false, Ordering::AcqRel) {
                show_counts();
            }
        }
        if !CORE.load(Ordering::Acquire) {
            refused = 0;
            // (short: the board must be fitted before the Amiga's first
            // reset ends, a few hundred milliseconds after the core starts)
            std::thread::sleep(Duration::from_millis(40));
            continue;
        }
        // (always fitted, the link up with the dock's adapter and its cable)
        let adapter = FITTED;
        let link = LINK.load(Ordering::Acquire) && !NO_CABLE.load(Ordering::Acquire);
        if tell_core(adapter, link) {
            refused = 0;
        } else {
            refused += 1;
            if SHOW_REPORTS && refused == 50 {
                crate::ui::send(crate::ui::Message::Notification(
                    crate::ui::Notification::new_long(
                        "Network: the core does not take the board's setting".to_string(),
                    ),
                ));
            }
        }
        if !link {
            std::thread::sleep(Duration::from_millis(40));
            continue;
        }
        let brought = bring();
        let sent = send();
        if !brought && !sent {
            std::thread::sleep(IDLE_POLL);
        }
    }
}

/// The link's state to the core, when it has changed (and at a core's
/// start: the station address, this handheld's own, first).
/// False if the core did not take it.
fn tell_core(adapter: bool, link: bool) -> bool {
    let want = if link {
        2
    } else if adapter {
        3
    } else {
        1
    };
    let told = TOLD.load(Ordering::Acquire);
    if told == want {
        return true;
    }
    let mut device = Device::lock();
    let serial = proto::serial_from(crate::hwinfo::get_serial_number().0);
    let ok = device.fpga.write_u32(proto::SERIAL, serial).is_ok()
        && device
            .fpga
            .write_u32(proto::STATUS, proto::status_link(adapter, link))
            .is_ok();
    if ok {
        TOLD.store(want, Ordering::Release);
        log::info!(
            "Network: the core's board is told the link is {}",
            if link { "up" } else { "down" }
        );
        // the cable taken out or put in while the core runs
        if (told == 2 && want == 3) || (told == 3 && want == 2) {
            drop(device);
            crate::ui::send(crate::ui::Message::Notification(
                crate::ui::Notification::new_long(
                    if link {
                        "Network connected"
                    } else {
                        "Network unplugged (adapter or cable)"
                    }
                    .to_string(),
                ),
            ));
        } else if SHOW_REPORTS && adapter {
            // (read back: the core says whether the board is fitted)
            let status = device.fpga.read_u32(proto::STATUS).unwrap_or(0);
            drop(device);
            crate::ui::send(crate::ui::Message::Notification(
                crate::ui::Notification::new_long(format!(
                    "Network: A2065 {} (status {:05X}). Reset the Amiga if it has booted",
                    if status & (1 << 17) != 0 {
                        "fitted"
                    } else {
                        "NOT fitted"
                    },
                    status & 0xF_FFFF
                )),
            ));
        }
    }
    ok
}

/// A frame from the dock into the core. True if there was one.
fn bring() -> bool {
    let mut scratch = SCRATCH.lock().unwrap();
    let len = {
        let mut inbox = INBOX.lock().unwrap();
        if inbox.count == 0 {
            return false;
        }
        let head = inbox.head;
        let len = inbox.lens[head];
        scratch[..proto::buffer_len(len)]
            .copy_from_slice(&inbox.frames[head][..proto::buffer_len(len)]);
        inbox.head = (head + 1) % INBOX_FRAMES;
        inbox.count -= 1;
        len
    };
    // the frame before must be out of the core's buffer (the LANCE takes
    // one in well under a millisecond)
    for _ in 0..8 {
        let status = Device::lock().fpga.read_u32(proto::STATUS);
        match status {
            Ok(s) if !proto::rx_busy(s) => {
                let mut device = Device::lock();
                let command = SpiCommand::new(FpgaSpiWordSize::Bits32);
                let _ = device
                    .fpga
                    .spi_write(
                        None,
                        command,
                        proto::RX_BUFFER,
                        &scratch[..proto::buffer_len(len)],
                    )
                    .and_then(|_| device.fpga.write_u32(proto::RX_COMMIT, len as u32));
                RX_FRAMES.fetch_add(1, Ordering::Relaxed);
                return true;
            }
            Ok(_) => std::thread::sleep(Duration::from_millis(1)),
            Err(_) => break,
        }
    }
    RX_LATE.fetch_add(1, Ordering::Relaxed);
    true
}

/// A frame the Amiga sends to the dock. True if there was one.
fn send() -> bool {
    let mut scratch = SCRATCH.lock().unwrap();
    let len = {
        let mut device = Device::lock();
        let Ok(status) = device.fpga.read_u32(proto::STATUS) else {
            return false;
        };
        let Some(len) = proto::tx_pending(status) else {
            return false;
        };
        let len = len.min(proto::MAX_FRAME);
        let command = SpiCommand::new(FpgaSpiWordSize::Bits32);
        let read = device.fpga.spi_read(
            Some(MAX_SPI_READ_CLOCK),
            command,
            proto::TX_BUFFER,
            &mut scratch[..proto::buffer_len(len)],
        );
        // (taken either way: a frame that cannot be read is not kept)
        let _ = device.fpga.write_u32(proto::STATUS, proto::STATUS_TX_TAKEN);
        if read.is_err() {
            return true;
        }
        len
    };
    TX_FRAMES.fetch_add(1, Ordering::Relaxed);
    for piece in proto::pieces(&scratch[..len]) {
        if !write_piece(piece.as_slice()) {
            // the dock is not taking them: the rest of the frame is given up
            TX_LOST.fetch_add(1, Ordering::Relaxed);
            break;
        }
    }
    true
}

/// One packet on the vendor bulk IN endpoint (it has no queue: a packet at
/// a time, when the one before has gone).
fn write_piece(packet: &[u8]) -> bool {
    for _ in 0..PIECE_TRIES {
        let written =
            unsafe { sys::tud_vendor_n_write(0, packet.as_ptr().cast(), packet.len() as u32) };
        if written as usize == packet.len() {
            unsafe { sys::tud_vendor_n_write_flush(0) };
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    false
}

/// Frames from the dock that were lost on the way to the core.
#[allow(unused)]
pub fn rx_lost() -> u32 {
    RX_LOST.load(Ordering::Relaxed)
}
