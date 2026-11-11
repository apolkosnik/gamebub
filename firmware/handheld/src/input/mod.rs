use std::sync::{LazyLock, Mutex, MutexGuard};
use std::time::Instant;

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
    /// are only here.
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

    /// A gamepad's state as the FPGA's gamepad registers take it
    /// (`fpga::REG_CTRL_GAMEPADS`): bit 16 connected, 15-0 the buttons (L2
    /// R2 L3 R3, then as `REG_CTRL_BUTTON_FORCE`); the sticks, -128 to 127
    /// (left x 31-24, left y 23-16, right x 15-8, right y 7-0). A trigger
    /// pulled more than half way (its Z axis) counts as L2 / R2.
    fn gamepad_registers(&self) -> [u32; 2] {
        let buttons = (1 << 16)
            | (((self.btn_l2 || self.axis_lz > 16384) as u32) << 15)
            | (((self.btn_r2 || self.axis_rz > 16384) as u32) << 14)
            | ((self.btn_l3 as u32) << 13)
            | ((self.btn_r3 as u32) << 12)
            | ((self.btn_a as u32) << 11)
            | ((self.btn_b as u32) << 10)
            | ((self.btn_x as u32) << 9)
            | ((self.btn_y as u32) << 8)
            | ((self.btn_up as u32) << 7)
            | ((self.btn_down as u32) << 6)
            | ((self.btn_left as u32) << 5)
            | ((self.btn_right as u32) << 4)
            | ((self.btn_l1 as u32) << 3)
            | ((self.btn_r1 as u32) << 2)
            | ((self.btn_start as u32) << 1)
            | (self.btn_select as u32);
        let stick = |axis: i16| ((axis >> 8) as u8) as u32;
        let sticks = (stick(self.axis_lx) << 24)
            | (stick(self.axis_ly) << 16)
            | (stick(self.axis_rx) << 8)
            | stick(self.axis_ry);
        [buttons, sticks]
    }
}

static INPUT_MANAGER: LazyLock<Mutex<InputManager>> =
    LazyLock::new(|| Mutex::new(InputManager::default()));

#[derive(Default)]
pub struct InputManager {
    /// The state of the internal buttons.
    internal_state: InputState,

    /// External gamepads
    gamepads: Vec<Gamepad>,

    /// Last button map sent to UI
    last_button_map: ButtonMap,

    /// Last gamepad registers written to the FPGA (None: to be written).
    last_gamepad_registers: [Option<[u32; 2]>; fpga::REG_CTRL_GAMEPAD_COUNT],

    /// The light gun of the running core, if it takes one.
    light_gun: lightgun::Controller,

    /// The mouse of the running core, if it takes one: a dock mouse, or
    /// else a dock pad's right stick, while the core's Controller setting
    /// selects the mouse.
    mouse: Mouse,

    /// Time base for the light gun and the mouse (its first use).
    epoch: Option<Instant>,
}

impl InputManager {
    pub fn lock() -> MutexGuard<'static, Self> {
        // TODO: can we just only do this from the worker thread?
        INPUT_MANAGER.lock().unwrap()
    }

    /// Update the state of the internal buttons.
    pub fn update_state(&mut self, state: InputState) {
        self.internal_state = state;
        self.send_event();
    }

    fn find_gamepad(&mut self, id: GamepadId) -> Option<&mut Gamepad> {
        self.gamepads.iter_mut().find(|g| g.id == id)
    }

    pub fn add_gamepad(&mut self, id: GamepadId) {
        if self.find_gamepad(id).is_some() {
            log::warn!("Ignoring duplicate gamepad");
            return;
        }
        log::info!("Adding gamepad id={}", id.0);
        self.gamepads.push(Gamepad {
            id,
            state: InputState::default(),
        });
        self.send_event();
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
        self.send_event();
    }

    pub fn remove_all_gamepads(&mut self) {
        log::info!("Removing all gamepads");
        self.gamepads.clear();
        self.send_event();
    }

    pub fn update_gamepad(&mut self, id: GamepadId, state: InputState) {
        let Some(gamepad) = self.find_gamepad(id) else {
            return;
        };
        gamepad.state = state;

        if crate::kvs::keys::DOCK_SWAP_ABXY.get().unwrap() {
            std::mem::swap(&mut gamepad.state.btn_a, &mut gamepad.state.btn_b);
            std::mem::swap(&mut gamepad.state.btn_x, &mut gamepad.state.btn_y);
        }

        // A mouse's report is a movement since the previous one: it is
        // counted here, once, as it arrives (the state kept above is the
        // same report until the next, so `send_mouse` reads only its
        // buttons from it).
        let mouse_report = gamepad
            .state
            .is_mouse()
            .then(|| gamepad.state.mouse_report());
        if let Some(report) = mouse_report {
            self.mouse.report(&report);
        }

        self.send_event();
    }

    /// Send an input event to the UI thread, if needed.
    ///
    /// Also update the FPGA, if needed?
    fn send_event(&mut self) {
        // TODO: handle button remapping before merge
        let mut state = self.internal_state.clone();
        for gamepad in &self.gamepads {
            // A light gun the core takes as one is not a gamepad.
            if gamepad.state.is_light_gun() && self.light_gun.active() {
                continue;
            }
            // A mouse never is: its movement is not a stick (it would press
            // the d-pad), its buttons are not the pad's (`lightgun::mouse`).
            if gamepad.state.is_mouse() {
                continue;
            }
            let pad = gamepad.state.as_pad();
            state.merge(&pad);

            // Temporary: map left analog stick to DPAD
            if pad.axis_lx <= -16384 {
                state.btn_left = true;
            }
            if pad.axis_lx >= 16384 {
                state.btn_right = true;
            }
            if pad.axis_ly <= -16384 {
                state.btn_up = true;
            }
            if pad.axis_ly >= 16384 {
                state.btn_down = true;
            }
        }

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

            // Send new button state to FPGA
            let fpga_buttons = if !self.gamepads.is_empty() {
                0u32 | ((state.btn_a as u32) << 11)
                    | ((state.btn_b as u32) << 10)
                    | ((state.btn_x as u32) << 9)
                    | ((state.btn_y as u32) << 8)
                    | ((state.btn_up as u32) << 7)
                    | ((state.btn_down as u32) << 6)
                    | ((state.btn_left as u32) << 5)
                    | ((state.btn_right as u32) << 4)
                    | ((state.btn_l1 as u32) << 3)
                    | ((state.btn_r1 as u32) << 2)
                    | ((state.btn_start as u32) << 1)
                    | ((state.btn_select as u32) << 0)
            } else {
                0
            };
            let _ = Device::lock()
                .fpga
                .write_u32(fpga::REG_CTRL_BUTTON_FORCE, fpga_buttons);
        }
        self.last_button_map = buttons;

        self.send_gamepad_registers();
        self.send_light_gun();
        self.send_mouse();
    }

    /// Write the gamepads, in the order they were connected, to the FPGA's
    /// gamepad registers (for a core taking them apart from the handheld's
    /// buttons; other bitstreams ignore the writes), where they changed.
    fn send_gamepad_registers(&mut self) {
        for (i, last) in self.last_gamepad_registers.iter_mut().enumerate() {
            let registers = match self.gamepads.get(i) {
                None => [0, 0],
                Some(gamepad) if gamepad.state.is_light_gun() && self.light_gun.active() => [0, 0],
                Some(gamepad) if gamepad.state.is_mouse() => [0, 0],
                Some(gamepad) => gamepad.state.as_pad().gamepad_registers(),
            };
            if *last != Some(registers) {
                let address = fpga::REG_CTRL_GAMEPADS + 8 * i as u32;
                let mut device = Device::lock();
                let _ = device.fpga.write_u32(address, registers[0]);
                let _ = device.fpga.write_u32(address + 4, registers[1]);
                *last = Some(registers);
            }
        }
    }

    /// Write the gamepad registers again: a newly loaded bitstream starts
    /// with them cleared.
    pub fn resend_gamepad_registers(&mut self) {
        self.last_gamepad_registers = Default::default();
        self.send_gamepad_registers();
    }

    /// The core running with focus now (None: none), with the value of its
    /// Controller setting (`lightgun::mouse::CONTROLLER_*`; 0 without one),
    /// for the light gun and the mouse it takes, if any
    /// (`lightgun::target_for_core`, `lightgun::mouse::target_for_core`):
    /// their registers are all written again. The gun's calibration comes
    /// from the KVS. The mouse tick (`ui::Message::MouseTick`) runs while
    /// the core takes the mouse and the setting selects it.
    pub fn set_core(&mut self, core_id: Option<&str>, controller: u32) {
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

        let was_ticking = self.mouse.enabled();
        self.mouse
            .set_target(core_id.and_then(mouse::target_for_core), controller);
        if self.mouse.enabled() != was_ticking {
            log::info!(
                "Mouse: {}",
                if self.mouse.enabled() {
                    "a dock mouse, else the first dock pad's right stick"
                } else {
                    "off"
                }
            );
            ui::send(ui::Message::MouseTick(self.mouse.enabled()));
        }
        // The gun stops (or starts) being a gamepad, and the core's
        // registers are written.
        self.send_event();
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
    fn mouse_input(&self) -> MouseInput {
        let states = || self.gamepads.iter().map(|gamepad| &gamepad.state);
        let mice = states()
            .filter(|state| state.is_mouse())
            .map(|state| state.mouse_report());
        let pad = states().find(|state| !state.is_light_gun() && !state.is_mouse());
        MouseInput {
            mouse: MouseReport::merged(mice),
            stick_x: pad.map_or(0, |pad| pad.axis_rx),
            stick_y: pad.map_or(0, |pad| pad.axis_ry),
            pad_r1: pad.is_some_and(|pad| pad.btn_r1),
            pad_l1: pad.is_some_and(|pad| pad.btn_l1),
            pad_r3: pad.is_some_and(|pad| pad.btn_r3),
            handheld_l: self.internal_state.btn_l1,
            handheld_r: self.internal_state.btn_r1,
        }
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
        let input = self.mouse_input();
        let gun_connected = self.gun_connected();
        let now = self.now_ms();
        let writes = self.mouse.update(now, gun_connected, &input);
        if !writes.is_empty() {
            let mut device = Device::lock();
            for &(address, value) in writes.as_slice() {
                let _ = device.fpga.write_u32(address, value);
            }
        }
    }

    /// The mouse tick (`ui::Message::MouseTick`, every
    /// `lightgun::mouse::MIN_MOTION_INTERVAL_MS` while the mouse is on).
    pub fn mouse_tick(&mut self) {
        self.send_mouse();
    }
}

/// Where a core's light gun calibration is kept.
fn calibration_key(target: lightgun::Target) -> Option<&'static KvsKey<[i32; 4]>> {
    (target == lightgun::TARGET_3DO).then_some(&crate::kvs::keys::GUN_CAL_3DO)
}
