//! Worker threads to do background blocking work.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::{mpsc, OnceLock};

use crate::core::CoreManager;
use crate::device::drivers::fpga;
use crate::device::Device;

use crate::input::{GamepadId, InputManager, InputState};
use crate::{kvs, ui};

#[derive(Debug)]
pub enum Message {
    /// Interrupt requests from the FPGA: the bits are collected by
    /// `send_fpga_irq`.
    FpgaIrq,
    /// The headphone state has changed
    HeadphoneState(bool),
    /// The handheld has docked
    DockBegin {
        serial: u32,
        #[allow(unused)]
        hardware: u32,
        firmware: String,
    },
    /// The handheld has undocked
    DockEnd,
    /// A gamepad connected
    GamepadConnected { id: GamepadId, has_analog: bool },
    /// A gamepad disconnected
    GamepadDisconnected(GamepadId),
    /// Gamepad input event
    GamepadInput(GamepadId, InputState),

    /// Run a cartridge
    RunCartridge,
    /// The idle timer has expired
    IdleTimerExpired,

    /// Fetch the list of cores
    FetchCoreList,
    /// Start running a core (ID)
    RunCore(String),
    /// Exit the current core (or cancel loading)
    ExitCore,
    /// Reset the current core
    ResetCore,
    /// A file was selected for the core (could be a directory).
    CoreFileSelected(PathBuf),
    /// Core file selection was cancelled.
    CoreFileCancelled,
    /// Choose another disc for the running core
    CoreChangeDisc,
    /// Core focus changed
    CoreFocusChanged(bool),
    /// Load core settings
    CoreSettingsLoad,
    /// Core setting changed
    CoreSettingChanged { id: u16, value: u32 },
}

/// Room in the worker's queue. The queue is allocated once, at start: an
/// unbounded channel allocates a block for every 31 messages sent, even with
/// the queue empty, and while a core runs the heap is too fragmented for that
/// (a crash in the TinyUSB task, allocating a block for a dock packet, with
/// 31 KB free). The queue never fills: gamepad input takes at most
/// `MAX_PENDING_INPUT` of it, FPGA interrupts one (`send_fpga_irq`), and the
/// rest (docking, the UI) come at a human rate.
const QUEUE_LEN: usize = 64;

/// Gamepad input messages waiting at most: more are dropped while the worker
/// is busy (a save, a disc image being opened). A dock mouse sends hundreds a
/// second; the next report of a pad has its whole state again.
const MAX_PENDING_INPUT: usize = 32;

/// Gamepad input messages waiting in the queue.
static PENDING_INPUT: AtomicUsize = AtomicUsize::new(0);
/// Gamepad input messages dropped since the last one that was taken.
static DROPPED_INPUT: AtomicU32 = AtomicU32::new(0);

/// FPGA interrupts not yet handled (`send_fpga_irq`); one `FpgaIrq` message
/// in the queue stands for all of them.
static PENDING_FPGA_IRQ: AtomicU32 = AtomicU32::new(0);

/// Send a message to the worker threads.
///
/// Gamepad input is dropped while `MAX_PENDING_INPUT` wait. Anything else
/// waits for room if the queue is full, which it should never be.
pub fn send(message: Message) {
    let Some(sender) = SENDER.get() else {
        log::error!("Dropping worker message {:?}", message);
        return;
    };
    let input = matches!(message, Message::GamepadInput(..));
    if input {
        if PENDING_INPUT.fetch_add(1, Ordering::AcqRel) >= MAX_PENDING_INPUT {
            PENDING_INPUT.fetch_sub(1, Ordering::AcqRel);
            DROPPED_INPUT.fetch_add(1, Ordering::Relaxed);
            return;
        }
    }
    match sender.try_send(message) {
        Ok(()) => {}
        Err(mpsc::TrySendError::Full(message)) => {
            if input {
                PENDING_INPUT.fetch_sub(1, Ordering::AcqRel);
                DROPPED_INPUT.fetch_add(1, Ordering::Relaxed);
            } else {
                log::warn!("Worker queue full: waiting to send {:?}", message);
                let _ = sender.send(message);
            }
        }
        Err(mpsc::TrySendError::Disconnected(_)) => panic!("worker gone"),
    }
}

/// Send FPGA interrupts (`REG_CTRL_IRQ_PENDING` bits) to the worker: they are
/// collected, and one `FpgaIrq` message waits for all of them (the module's
/// vblank alone comes 60 times a second, also while the worker is busy).
pub fn send_fpga_irq(irq_mask: u32) {
    if SENDER.get().is_none() {
        // (not started: dropped, as `send` would, so that nothing is left
        // pending that no message will ever take)
        log::error!("Dropping FPGA IRQ {irq_mask:#x}");
        return;
    }
    if PENDING_FPGA_IRQ.fetch_or(irq_mask, Ordering::AcqRel) == 0 {
        send(Message::FpgaIrq);
    }
}

/// Start the worker threadpool. Called once during system init. Panics if called twice.
pub fn start() {
    let (sender, receiver) = mpsc::sync_channel::<Message>(QUEUE_LEN);
    SENDER.set(sender).expect("Worker already initialized");

    // TODO: look into reducing stack usage
    std::thread::Builder::new()
        .name("Worker".to_string())
        .stack_size(16 * 1024)
        .spawn(move || {
            while let Ok(message) = receiver.recv() {
                log::debug!("Dispatch {:?}", message);
                dispatch(message);
            }
        })
        .unwrap();
}

static SENDER: OnceLock<mpsc::SyncSender<Message>> = OnceLock::new();

fn dispatch(message: Message) {
    match message {
        Message::FpgaIrq => {
            let irq_mask = PENDING_FPGA_IRQ.swap(0, Ordering::AcqRel);
            if (irq_mask & fpga::Irq::ModuleVblank.as_flag()) != 0 {
                // Module vblank
                if let Some(bitstream) = CoreManager::lock().current_bitstream() {
                    bitstream.on_vblank_irq();
                }
            }
            if (irq_mask & fpga::Irq::CoreRequest.as_flag()) != 0 {
                CoreManager::lock().handle_core_request();
            }
        }
        Message::HeadphoneState(has_headphones) => {
            log::info!("Headphone detection: {}", has_headphones);
            let mut device = Device::lock();
            device.dac.set_headphones_enabled(has_headphones).unwrap();
            device.dac.set_speakers_enabled(!has_headphones).unwrap();
        }
        Message::RunCartridge => {
            let cart_type = {
                let mut device = Device::lock();
                device.get_cart_switch()
            };
            log::info!("Cart switch: {}", cart_type);

            let core_id = if cart_type {
                "Game-Bub.GB"
            } else {
                "Game-Bub.GBA"
            };

            CoreManager::lock().run_core(core_id, true);
        }
        Message::DockBegin {
            serial, firmware, ..
        } => {
            ui::send(ui::Message::DockBegin {
                serial: format!("{serial:08X}"),
                firmware,
            });

            InputManager::lock().remove_all_gamepads();
            Device::lock().set_docked(true).unwrap();
        }
        Message::DockEnd => {
            ui::send(ui::Message::DockEnd);
            InputManager::lock().remove_all_gamepads();
            Device::lock().set_docked(false).unwrap();
        }
        Message::GamepadConnected { id, has_analog } => {
            InputManager::lock().add_gamepad(id, has_analog);
        }
        Message::GamepadDisconnected(id) => {
            InputManager::lock().remove_gamepad(id);
        }
        Message::GamepadInput(id, state) => {
            PENDING_INPUT.fetch_sub(1, Ordering::AcqRel);
            let dropped = DROPPED_INPUT.swap(0, Ordering::Relaxed);
            if dropped != 0 {
                log::warn!("Worker busy: {dropped} gamepad input messages dropped");
            }
            InputManager::lock().update_gamepad(id, state);
        }
        Message::IdleTimerExpired => {
            // If the idle timer expires during setup, just power off.
            let setup_stage = kvs::keys::SETUP_STAGE.get().unwrap_or_default();
            if setup_stage == 0 {
                log::warn!("Idle during setup, powering off.");
                Device::lock().power_off();
            }
        }
        Message::FetchCoreList => CoreManager::lock().list_cores(),
        Message::RunCore(id) => CoreManager::lock().run_core(&id, false),
        Message::ExitCore => CoreManager::lock().exit_core(),
        Message::ResetCore => CoreManager::lock().reset_core(),
        Message::CoreFileSelected(file) => CoreManager::lock().handle_file_selected(file),
        Message::CoreFileCancelled => CoreManager::lock().cancel_file_select(),
        Message::CoreChangeDisc => CoreManager::lock().begin_disc_change(),
        Message::CoreFocusChanged(focused) => CoreManager::lock().focus_changed(focused),
        Message::CoreSettingsLoad => CoreManager::lock().ui_load_settings(),
        Message::CoreSettingChanged { id, value } => CoreManager::lock().setting_changed(id, value),
        #[allow(unreachable_patterns)]
        _ => {
            log::warn!("Unhandled message: {:?}", message);
        }
    }
}
