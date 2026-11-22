use std::sync::{LazyLock, Mutex, MutexGuard};
use std::time::Instant;

use lightgun::keyboard::{self, KeySet, Keyboard, KeyboardReport};
use lightgun::mouse::{self, Mouse, MouseInput, MouseReport};
use lightgun::{CalibrationEvent, GunReport};

use crate::device::drivers::fpga;
use crate::device::Device;
use crate::kvs::KvsKey;
use crate::ui::{self, ButtonMap};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GamepadId(pub u32);

struct Gamepad {
    /// ID of the gamepad
    pub id: GamepadId,
    /// Whether the gamepad has analog sticks
    pub has_analog: bool,
    /// Current input state.
    pub state: InputState,
}

#[derive(Debug, Clone, Default)]
pub struct InputState {
    pub btn_a: bool,
    pub btn_b: bool,
    pub btn_x: bool,
    pub btn_y: bool,

    pub btn_up: bool,
    pub btn_down: bool,
    pub btn_right: bool,
    pub btn_left: bool,

    pub btn_system: bool,
    pub btn_select: bool,
    pub btn_start: bool,
    pub btn_capture: bool,

    pub btn_vol_up: bool,
    pub btn_vol_down: bool,
    pub btn_power: bool,

    pub btn_l1: bool,
    pub btn_r1: bool,
    pub btn_l2: bool,
    pub btn_r2: bool,
    pub btn_l3: bool,
    pub btn_r3: bool,

    pub axis_lx: i16,
    pub axis_ly: i16,
    pub axis_lz: i16,
    pub axis_rx: i16,
    pub axis_ry: i16,
    pub axis_rz: i16,

    /// A dock gamepad's buttons word as received (0 for the handheld's own
    /// buttons): bit 31 marks a light gun, bit 30 a mouse, whose buttons
    /// are only here (our dock firmware, see the `lightgun` crate).
    pub raw_buttons: u32,
}

impl InputState {
    /// Whether this is a light gun on the dock (a GunCon 3): its left stick
    /// is its aim (all 16 bits), its left Z the distance to the emitters.
    pub fn is_light_gun(&self) -> bool {
        GunReport::is_light_gun(self.raw_buttons)
    }

    pub fn gun_report(&self) -> GunReport {
        GunReport {
            x: self.axis_lx,
            y: self.axis_ly,
            z: self.axis_lz as u16,
            buttons: self.raw_buttons,
        }
    }

    /// Whether this is a mouse on the dock: its left stick is its movement
    /// since the previous report, its left Z the wheel's. A mouse is never
    /// a gamepad (`InputManager::send_event`).
    pub fn is_mouse(&self) -> bool {
        MouseReport::is_mouse(self.raw_buttons)
    }

    pub fn mouse_report(&self) -> MouseReport {
        MouseReport {
            dx: self.axis_lx,
            dy: self.axis_ly,
            wheel: self.axis_lz,
            buttons: self.raw_buttons,
        }
    }

    /// Whether this is a keyboard on the dock: its buttons' low byte is its
    /// modifier keys, its left axes the keys held (`KeyboardReport`). A
    /// keyboard is never a gamepad.
    pub fn is_keyboard(&self) -> bool {
        KeyboardReport::is_keyboard(self.raw_buttons)
    }

    pub fn keyboard_report(&self) -> KeyboardReport {
        KeyboardReport::from_dock(self.raw_buttons, self.axis_lx, self.axis_ly, self.axis_lz)
    }

    /// The state as a gamepad: a light gun's aim and distance are not
    /// sticks or a trigger.
    fn as_pad(&self) -> InputState {
        let mut state = self.clone();
        if self.is_light_gun() {
            state.axis_lx = 0;
            state.axis_ly = 0;
            state.axis_lz = 0;
        }
        state
    }

    fn merge_axis(a: &mut i16, b: i16) {
        if b.unsigned_abs() > a.unsigned_abs() {
            *a = b;
        }
    }

    fn merge(&mut self, other: &InputState) {
        self.btn_a |= other.btn_a;
        self.btn_b |= other.btn_b;
        self.btn_x |= other.btn_x;
        self.btn_y |= other.btn_y;
        self.btn_up |= other.btn_up;
        self.btn_down |= other.btn_down;
        self.btn_right |= other.btn_right;
        self.btn_left |= other.btn_left;
        self.btn_system |= other.btn_system;
        self.btn_select |= other.btn_select;
        self.btn_start |= other.btn_start;
        self.btn_capture |= other.btn_capture;
        self.btn_vol_up |= other.btn_vol_up;
        self.btn_vol_down |= other.btn_vol_down;
        self.btn_power |= other.btn_power;
        self.btn_l1 |= other.btn_l1;
        self.btn_r1 |= other.btn_r1;
        self.btn_l2 |= other.btn_l2;
        self.btn_r2 |= other.btn_r2;
        self.btn_l3 |= other.btn_l3;
        self.btn_r3 |= other.btn_r3;
        Self::merge_axis(&mut self.axis_lx, other.axis_lx);
        Self::merge_axis(&mut self.axis_ly, other.axis_ly);
        Self::merge_axis(&mut self.axis_lz, other.axis_lz);
        Self::merge_axis(&mut self.axis_rx, other.axis_rx);
        Self::merge_axis(&mut self.axis_ry, other.axis_ry);
        Self::merge_axis(&mut self.axis_rz, other.axis_rz);
    }

    /// Whether this dock device is a pad: not a mouse (its movement is not
    /// a stick, its buttons are not a pad's), not a keyboard (its keys are
    /// in the axes), nor a light gun the running
    /// core takes as one (`gun_active`).
    fn is_pad(&self, gun_active: bool) -> bool {
        !(self.is_mouse() || self.is_keyboard() || (self.is_light_gun() && gun_active))
    }

    fn merge_gamepads(&mut self, gamepads: &[Gamepad], gun_active: bool) {
        for gamepad in gamepads.iter().filter(|g| g.state.is_pad(gun_active)) {
            let pad = gamepad.state.as_pad();
            self.merge(&pad);

            const THRESHOLD: i16 = i16::MAX / 2;
            self.btn_left |= pad.axis_lx <= -THRESHOLD;
            self.btn_right |= pad.axis_lx >= THRESHOLD;
            self.btn_up |= pad.axis_ly <= -THRESHOLD;
            self.btn_down |= pad.axis_ly >= THRESHOLD;
        }
    }
}

static INPUT_MANAGER: LazyLock<Mutex<InputManager>> =
    LazyLock::new(|| Mutex::new(InputManager::default()));

#[derive(Default)]
enum CoreInputMode {
    #[default]
    None,
    Basic {
        prev_update: u32,
    },
    Extended {
        num_slots: usize,
        prev_update: [[u32; 3]; 4],
    },
}

#[derive(Default)]
pub struct InputManager {
    /// The state of the internal buttons.
    internal_state: InputState,

    /// External gamepads
    gamepads: Vec<Gamepad>,

    /// Assigned gamepad slots
    slots: [Option<GamepadId>; 4],

    /// Current core input mode
    core_input: CoreInputMode,

    /// Last button map sent to UI
    last_button_map: ButtonMap,

    /// The light gun of the running core, if it takes one.
    light_gun: lightgun::Controller,

    /// The mouse of the running core, if it takes one: a dock mouse, or
    /// else a dock pad's right stick, while the core's Controller setting
    /// selects the mouse.
    mouse: Mouse,

    /// The core's second mouse, driven by the second dock mouse (the core
    /// chains it while that mouse is connected).
    mouse2: Mouse,

    /// The keyboard of the running core, if it takes one: the dock's
    /// keyboards.
    keyboard: Keyboard,

    /// Time base for the light gun and the mouse (its first use).
    epoch: Option<Instant>,
}

const INPUT_CAPS_NUM_SLOTS_MASK: u32 = 0xF;
const INPUT_CAPS_RUMBLE: u32 = 1 << 6;
const INPUT_CAPS_EXTENDED: u32 = 1 << 7;

const DEVICE_TYPE_DIGITAL_GAMEPAD: u32 = 2;
const DEVICE_TYPE_ANALOG_GAMEPAD: u32 = 3;

fn pack_buttons(state: &InputState) -> u32 {
    ((state.btn_a as u32) << 15)
        | ((state.btn_b as u32) << 14)
        | ((state.btn_x as u32) << 13)
        | ((state.btn_y as u32) << 12)
        | ((state.btn_up as u32) << 11)
        | ((state.btn_down as u32) << 10)
        | ((state.btn_left as u32) << 9)
        | ((state.btn_right as u32) << 8)
        | ((state.btn_l1 as u32) << 7)
        | ((state.btn_r1 as u32) << 6)
        | ((state.btn_l2 as u32) << 5)
        | ((state.btn_r2 as u32) << 4)
        | ((state.btn_l3 as u32) << 3)
        | ((state.btn_r3 as u32) << 2)
        | ((state.btn_start as u32) << 1)
        | (state.btn_select as u32)
}

fn pack_gamepad_slot(gamepad: &Gamepad) -> [u32; 3] {
    let state = gamepad.state.as_pad();
    let clamp_stick = |v: i16| -> u8 { (v / 256).clamp(-127, 127) as i8 as u8 };
    let device_type = if gamepad.has_analog {
        DEVICE_TYPE_ANALOG_GAMEPAD
    } else {
        DEVICE_TYPE_DIGITAL_GAMEPAD
    };
    let trigger_l = (state.axis_lz.max(0) >> 7) as u8;
    let trigger_r = (state.axis_rz.max(0) >> 7) as u8;

    let buttons = (device_type << 28) | pack_buttons(&gamepad.state);
    let sticks = u32::from_le_bytes([
        clamp_stick(state.axis_lx),
        clamp_stick(state.axis_ly),
        clamp_stick(state.axis_rx),
        clamp_stick(state.axis_ry),
    ]);
    let triggers = ((trigger_r as u32) << 8) | ((trigger_l as u32) << 0);
    [buttons, sticks, triggers]
}

impl InputManager {
    pub fn lock() -> MutexGuard<'static, Self> {
        // TODO: can we just only do this from the worker thread?
        INPUT_MANAGER.lock().unwrap()
    }

    /// Update the state of the internal buttons.
    pub fn update_state(&mut self, state: InputState) {
        self.internal_state = state;
        self.update_ui();
        // (the handheld's L and R are the mouse's buttons too)
        self.send_mouse();
    }

    fn find_gamepad(&mut self, id: GamepadId) -> Option<&mut Gamepad> {
        self.gamepads.iter_mut().find(|g| g.id == id)
    }

    pub fn add_gamepad(&mut self, id: GamepadId, has_analog: bool) {
        if self.find_gamepad(id).is_some() {
            log::warn!("Ignoring duplicate gamepad id={}", id.0);
            return;
        }
        log::info!("Adding gamepad id={}", id.0);
        if let Some(slot) = self.slots.iter_mut().find(|s| s.is_none()) {
            *slot = Some(id);
        }
        self.gamepads.push(Gamepad {
            id,
            has_analog,
            state: InputState::default(),
        });
        ui::send(ui::Message::Notification(ui::Notification::new_short(
            "Gamepad connected".to_string(),
        )));
        self.update_ui();
        self.sync_fpga();
    }

    pub fn remove_gamepad(&mut self, id: GamepadId) {
        let index = match self.gamepads.iter().position(|g| g.id == id) {
            Some(index) => index,
            None => {
                log::warn!("Not removing unknown gamepad");
                return;
            }
        };
        log::info!("Removing gamepad id={}", id.0);
        self.gamepads.remove(index);
        if let Some(slot) = self.slots.iter_mut().find(|s| **s == Some(id)) {
            *slot = None;
        }
        ui::send(ui::Message::Notification(ui::Notification::new_short(
            "Gamepad disconnected".to_string(),
        )));
        self.update_ui();
        self.sync_fpga();
        // (a keyboard that went: its keys are released)
        self.send_keyboard();
    }

    pub fn remove_all_gamepads(&mut self) {
        // (the handheld has left the dock: its Ethernet adapter went too)
        crate::net::undocked();
        log::info!("Removing all gamepads");
        self.slots = [None; 4];
        self.gamepads.clear();
        self.update_ui();
        self.sync_fpga();
        self.send_keyboard();
    }

    pub fn update_gamepad(&mut self, id: GamepadId, state: InputState) {
        if let Some(gamepad) = self.find_gamepad(id) {
            gamepad.state = state;

            if crate::kvs::keys::DOCK_SWAP_ABXY.get().unwrap() {
                std::mem::swap(&mut gamepad.state.btn_a, &mut gamepad.state.btn_b);
                std::mem::swap(&mut gamepad.state.btn_x, &mut gamepad.state.btn_y);
            }

            // A mouse's report is a movement since the previous one: it is
            // counted here, once, as it arrives (the state kept above is the
            // same report until the next, so `send_mouse` reads only its
            // buttons from it). Each dock mouse drives its own 3DO mouse (in
            // the order the dock connected them), as two mice chained on a
            // console.
            let mouse_report = gamepad
                .state
                .is_mouse()
                .then(|| gamepad.state.mouse_report());
            if let Some(report) = mouse_report {
                let index = self
                    .gamepads
                    .iter()
                    .filter(|gamepad| gamepad.state.is_mouse())
                    .position(|gamepad| gamepad.id == id)
                    .unwrap_or(0);
                match mouse::mouse_for(index) {
                    Some(0) => self.mouse.report(&report),
                    Some(_) => self.mouse2.report(&report),
                    None => {}
                }
            }

            self.update_ui();
            self.sync_fpga();
            self.send_light_gun();
            self.send_mouse();
            self.send_keyboard();
        }
    }

    pub fn set_core_input_caps(&mut self, caps: u32) {
        let num_slots = ((caps & INPUT_CAPS_NUM_SLOTS_MASK) as usize).min(4);
        let has_rumble = (caps & INPUT_CAPS_RUMBLE) != 0;
        let is_extended = (caps & INPUT_CAPS_EXTENDED) != 0;
        log::info!(
            "Core input caps: 0x{caps:08X} (num_slots={num_slots}, ext={is_extended}, rumble={has_rumble})"
        );
        self.core_input = if num_slots == 0 {
            CoreInputMode::None
        } else if is_extended {
            CoreInputMode::Extended {
                num_slots,
                prev_update: [[0; 3]; 4],
            }
        } else {
            CoreInputMode::Basic { prev_update: 0 }
        };
        self.sync_fpga();
    }

    pub fn clear_core_input_caps(&mut self) {
        self.core_input = CoreInputMode::None;
    }

    fn sync_fpga(&mut self) {
        match &mut self.core_input {
            CoreInputMode::None => {}
            CoreInputMode::Basic { prev_update } => {
                let mut state = InputState::default();
                state.merge_gamepads(&self.gamepads, self.light_gun.active());
                let update = pack_buttons(&state);
                if update != *prev_update {
                    *prev_update = update;
                    let _ = Device::lock()
                        .fpga
                        .write_u32(fpga::REG_CTRL_INPUT_SLOT_BASE, update);
                }
            }
            CoreInputMode::Extended {
                num_slots,
                prev_update,
            } => {
                // The pads in the order they were connected: a dock mouse,
                // or a light gun the core takes as one, takes no slot.
                let gun_active = self.light_gun.active();
                let mut pads = self.gamepads.iter().filter(|g| g.state.is_pad(gun_active));
                for (index, prev) in prev_update[..*num_slots].iter_mut().enumerate() {
                    let words = match pads.next() {
                        Some(gamepad) => pack_gamepad_slot(gamepad),
                        None => [0; 3],
                    };
                    if words != *prev {
                        *prev = words;
                        let _ = Device::lock().fpga.write_input_slot(index, words);
                    }
                }
            }
        }
    }

    /// Send an input event to the UI thread, if needed.
    fn update_ui(&mut self) {
        // TODO: handle button remapping before merge
        let mut state = self.internal_state.clone();
        state.merge_gamepads(&self.gamepads, self.light_gun.active());

        use ui::buttons::Button;
        let buttons = enum_map::enum_map! {
            Button::A => state.btn_a,
            Button::B => state.btn_b,
            Button::X => state.btn_x,
            Button::Y => state.btn_y,
            Button::Up => state.btn_up,
            Button::Down => state.btn_down,
            Button::Left => state.btn_left,
            Button::Right => state.btn_right,
            Button::Start => state.btn_start,
            Button::Select => state.btn_select,
            Button::L => state.btn_l1,
            Button::R => state.btn_r1,
            Button::Home => state.btn_system,
            Button::VolUp => state.btn_vol_up,
            Button::VolDown => state.btn_vol_down,
            Button::Power => state.btn_power,
        };

        if self.last_button_map != buttons {
            ui::send(ui::Message::Button(buttons));
            self.last_button_map = buttons;
        }
    }
    /// The core running with focus now (None: none), with the value of its
    /// Controller setting (`lightgun::mouse::CONTROLLER_*`; 0 without one),
    /// for the light gun and the mouse it takes, if any
    /// (`lightgun::target_for_core`, `lightgun::mouse::target_for_core`):
    /// their registers are all written again. The gun's calibration comes
    /// from the KVS. The input tick (`ui::Message::MouseTick`) runs while
    /// the core takes the light gun, or the mouse and the setting selects it.
    pub fn set_core(&mut self, core_id: Option<&str>, controller: u32) {
        // (the network board, if the core has one: its own module)
        crate::net::set_core(core_id);
        let was_ticking = self.ticking();
        let target = core_id.and_then(lightgun::target_for_core);
        let calibration = target
            .and_then(calibration_key)
            .and_then(|key| key.get())
            .and_then(lightgun::Calibration::from_words)
            .unwrap_or_default();
        if target.is_some() {
            log::info!(
                "Light gun: core takes one (Controller setting {controller}), calibration {:?}",
                calibration
            );
        }
        self.light_gun.set_target(target, calibration, controller);

        let had_mouse = self.mouse.enabled();
        self.mouse
            .set_target(core_id.and_then(mouse::target_for_core), controller);
        self.mouse2
            .set_target(core_id.and_then(mouse::second_target_for_core), controller);
        if self.mouse.enabled() != had_mouse {
            log::info!(
                "Mouse: {}",
                if self.mouse.enabled() {
                    "a dock mouse, else the first dock pad's right stick"
                } else {
                    "off"
                }
            );
        }
        if self.ticking() != was_ticking {
            ui::send(ui::Message::MouseTick(self.ticking()));
        }
        let had_keyboard = self.keyboard.enabled();
        self.keyboard
            .set_target(core_id.and_then(keyboard::target_for_core));
        if self.keyboard.enabled() != had_keyboard {
            log::info!(
                "Keyboard: {}",
                if self.keyboard.enabled() {
                    "the dock's keyboards"
                } else {
                    "off"
                }
            );
        }
        // The gun stops (or starts) being a gamepad, and the core's
        // registers are written.
        self.update_ui();
        self.sync_fpga();
        self.send_light_gun();
        self.send_mouse();
        self.send_keyboard();
    }

    /// Milliseconds since the light gun's or the mouse's first use.
    fn now_ms(&mut self) -> u64 {
        self.epoch
            .get_or_insert_with(Instant::now)
            .elapsed()
            .as_millis() as u64
    }

    /// Whether a light gun is on the dock (the core's, while it takes one).
    fn gun_connected(&self) -> bool {
        self.gamepads
            .iter()
            .any(|gamepad| gamepad.state.is_light_gun())
    }

    /// Write the light gun (the first one on the dock) to the core that
    /// takes it, where it changed, and tell the user how calibration goes.
    ///
    /// The core takes these as setting writes on the host memory interface,
    /// at any time: they go through the FPGA's SPI port under the `Device`
    /// lock, as a whole between the disc sectors and file data the core
    /// asks for.
    fn send_light_gun(&mut self) {
        if !self.light_gun.active() {
            return;
        }
        let report = self
            .gamepads
            .iter()
            .find(|gamepad| gamepad.state.is_light_gun())
            .map(|gamepad| gamepad.state.gun_report());
        let now = self.now_ms();
        let (writes, event) = self.light_gun.update(now, report.as_ref());
        if !writes.is_empty() {
            let mut device = Device::lock();
            for &(address, value) in writes.as_slice() {
                let _ = device.fpga.write_u32(address, value);
            }
        }

        if let Some(notice) = self.light_gun.take_notice() {
            let text = match notice {
                lightgun::GunNotice::NoScreen => {
                    log::warn!("Light gun: off screen for a while, no IR emitters seen");
                    "Light gun: cannot see the screen. Check the IR emitters"
                }
                lightgun::GunNotice::ScreenFound => {
                    log::info!("Light gun: sees the screen");
                    "Light gun: sees the screen"
                }
            };
            ui::send(ui::Message::Notification(ui::Notification::new_long(
                text.to_string(),
            )));
        }

        let Some(event) = event else {
            return;
        };
        let text = match event {
            CalibrationEvent::Started => {
                "Gun calibration: aim at the top-left corner and pull the trigger"
            }
            CalibrationEvent::TopLeftTaken => {
                "Now aim at the bottom-right corner and pull the trigger"
            }
            CalibrationEvent::OffScreen => "Off screen: aim at the corner and pull again",
            CalibrationEvent::Failed => "Corners too close: calibration not changed",
            CalibrationEvent::Cancelled => "Gun calibration cancelled",
            CalibrationEvent::Finished(calibration) => {
                log::info!("Light gun: calibrated {:?}", calibration);
                if let Some(key) = self.light_gun.target().and_then(calibration_key) {
                    key.set(&calibration.to_words());
                    key.flush();
                }
                "Gun calibrated"
            }
        };
        ui::send(ui::Message::Notification(ui::Notification::new_long(
            text.to_string(),
        )));
    }

    /// What drives the mouse: the dock mice as one (their latest reports,
    /// for their buttons together; their movement went in through
    /// `update_gamepad`), else the first dock gamepad that is neither a
    /// light gun nor a mouse (its right stick, R1, L1 and R3; nothing
    /// without one); and the handheld's own L and R. The pad's buttons also
    /// reach the core as the pad's, as they always do.
    /// The first dock mouse drives the first 3DO mouse, the second the second
    /// (which has no stick or handheld buttons standing in).
    fn mouse_inputs(&self) -> [MouseInput; 2] {
        let states = || self.gamepads.iter().map(|gamepad| &gamepad.state);
        // (No heap: nothing may allocate while a core runs.)
        let mut mice = arrayvec::ArrayVec::<MouseReport, 8>::new();
        for state in states().filter(|state| state.is_mouse()) {
            if mice.try_push(state.mouse_report()).is_err() {
                break;
            }
        }
        let [first, second] = mouse::route_mice(&mice);
        let pad = states().find(|state| !state.is_light_gun() && !state.is_mouse());
        let one = MouseInput {
            mouse: first,
            stick_x: pad.map_or(0, |pad| pad.axis_rx),
            stick_y: pad.map_or(0, |pad| pad.axis_ry),
            pad_r1: pad.is_some_and(|pad| pad.btn_r1),
            pad_l1: pad.is_some_and(|pad| pad.btn_l1),
            pad_r3: pad.is_some_and(|pad| pad.btn_r3),
            handheld_l: self.internal_state.btn_l1,
            handheld_r: self.internal_state.btn_r1,
        };
        let two = MouseInput {
            mouse: second,
            ..MouseInput::default()
        };
        [one, two]
    }

    /// Write the mouse (movement since the last packet, buttons) to the
    /// core that takes it, where there is any: on every input event, and on
    /// the mouse tick in between, since a pad only reports when something
    /// changes and a stick held still must keep the mouse moving (and a
    /// dock mouse's last reports before it stops are written by the tick
    /// when the packet is due). A connected light gun is the core's instead
    /// (`lightgun::mouse`).
    fn send_mouse(&mut self) {
        if !self.mouse.enabled() {
            return;
        }
        let [input, input2] = self.mouse_inputs();
        let gun_connected = self.gun_connected();
        let now = self.now_ms();
        let writes = self.mouse.update(now, gun_connected, &input);
        let writes2 = if self.mouse2.enabled() {
            self.mouse2.update(now, gun_connected, &input2)
        } else {
            Default::default()
        };
        if !writes.is_empty() || !writes2.is_empty() {
            let mut device = Device::lock();
            for &(address, value) in writes.as_slice().iter().chain(writes2.as_slice()) {
                let _ = device.fpga.write_u32(address, value);
            }
        }
    }

    /// Write the key events that bring the core's keyboard to the keys held
    /// on the dock's keyboards now (all of them as one), to the core that
    /// takes a keyboard: on every report of a keyboard, when one goes, and
    /// when the core gets the focus back (the keys let go meanwhile).
    fn send_keyboard(&mut self) {
        if !self.keyboard.enabled() {
            return;
        }
        let mut down = KeySet::new();
        for gamepad in self.gamepads.iter().filter(|g| g.state.is_keyboard()) {
            gamepad.state.keyboard_report().add_to(&mut down);
        }
        // (an update gives at most `keyboard::MAX_EVENTS`)
        while self.keyboard.pending(&down) {
            let events = self.keyboard.update(&down);
            if events.is_empty() {
                break;
            }
            let mut device = Device::lock();
            for &(address, value) in events.as_slice() {
                let _ = device.fpga.write_u32(address, value);
            }
        }
    }

    /// The mouse tick (`ui::Message::MouseTick`, every
    /// `lightgun::mouse::MIN_MOTION_INTERVAL_MS` while the mouse is on).
    ///
    /// It runs while the core takes the light gun too: the dock sends a
    /// gun's report only when it changes, so a gun held still (C1 + C2 held
    /// to calibrate, an aim held back by `MIN_AIM_INTERVAL_MS`, a gun that
    /// keeps reporting off screen) is fed its last report again here.
    pub fn mouse_tick(&mut self) {
        self.send_light_gun();
        self.send_mouse();
    }

    /// Whether the tick (`mouse_tick`) runs: a mouse, or a light gun.
    fn ticking(&self) -> bool {
        self.mouse.enabled() || self.light_gun.active()
    }
}

/// Where a core's light gun calibration is kept.
fn calibration_key(target: lightgun::Target) -> Option<&'static KvsKey<[i32; 4]>> {
    (target == lightgun::TARGET_3DO).then_some(&crate::kvs::keys::GUN_CAL_3DO)
}
