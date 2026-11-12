//! A 3DO mouse from a mouse on the dock, or from a gamepad's right stick.
//!
//! The 3DO core takes a mouse on its control port beside the pad while its
//! Controller register (`crate::Target::enable`, 0x1014) holds
//! `CONTROLLER_MOUSE`. The register is also a setting of the core
//! (settings.json: 0 pad only, 1 pad + light gun, 2 pad + mouse), so the
//! user picks the mouse in the core's settings menu; the firmware passes the
//! setting's value to `Mouse::set_target`, and the mouse works only while it
//! is `CONTROLLER_MOUSE`.
//!
//! What drives it is one of two things, in this order:
//!
//!  * A physical mouse on the dock. The dock reports one as an ordinary
//!    gamepad (`GamepadData`) with bit 30 of its buttons set
//!    (`button::MOUSE`; a light gun has bit 31): the left stick is the
//!    movement since the previous report (lx right, ly down, in the mouse's
//!    counts), the left Z axis the wheel, bits 0-4 its buttons
//!    (`MouseReport`). Every report is one movement, counted exactly once
//!    (`Mouse::report`), summed across the reports since the previous
//!    packet (`Motion`) and scaled by `SENSITIVITY_NUMERATOR` /
//!    `SENSITIVITY_DENOMINATOR` (1: one count of the mouse is one count of
//!    the 3DO mouse), any fraction of a unit carried to the next packet.
//!    Every report is written to the core as it arrives: paced, a 125 Hz
//!    mouse's 8 ms reports fell into the 3DO's 16.7 ms fields one or three
//!    at a time, and the cursor stuttered. While a mouse is on the dock the stick does
//!    nothing; when it goes the stick goes on, from where it is then. Two
//!    mice at once are one: their movements add up, a button is held while
//!    either holds it (`MouseReport::merged`).
//!
//!  * The first dock gamepad's right stick as a velocity cursor (`Cursor`):
//!    the stick's deflection past `DEAD_ZONE` is a speed, linear up to
//!    `MAX_SPEED` units per second at full deflection, integrated over time.
//!    What is below one unit stays accumulated, so a slow deflection still
//!    moves.
//!
//! The core gets, over the host memory interface (`Target`):
//!
//!  * `Target::motion`: `{dy[31:16], dx[15:0]}`, two signed 16-bit units
//!    moved since the previous write (x right, y down), see `pack_motion`.
//!    Each write is one movement packet (the core moves the 3DO mouse one
//!    count per unit and clamps a packet to +-255 per axis), so packets are
//!    small and frequent: written only when there is movement, the physical
//!    mouse's with every report, the stick's at most every
//!    `MIN_MOTION_INTERVAL_MS`, clamped to `MAX_PACKET` per axis (a
//!    physical mouse's excess waits for the next packet; the stick's is
//!    dropped);
//!  * `Target::buttons`: bit 0 left, bit 1 right, bit 2 middle (held = 1),
//!    see `pack_buttons`, written on a change. A physical mouse's left,
//!    right and middle buttons are those (its others do nothing); without
//!    one the pad's R1 is left, L1 right, R3 (the right stick's click)
//!    middle. The handheld's own L and R are left and right either way, so
//!    the mouse clicks without a dock pad (`MouseInput::buttons`).
//!
//! A light gun on the dock wins: while one is connected the gun code holds
//! the Controller register at 1 (`crate::Controller`) and the mouse stands
//! still with its buttons released (`Mouse::update`'s `gun_connected`).
//! When the gun is unplugged the gun code writes the setting's value back,
//! and the mouse goes on.
//!
//! A physical mouse is never a gamepad: the firmware keeps it out of the
//! merged pad (its movement is not a stick, its buttons are not the pad's).

use crate::Writes;

/// The dock's mouse buttons (bits of `GamepadData`'s buttons word).
pub mod button {
    /// Set on every report of a mouse.
    pub const MOUSE: u32 = 1 << 30;
    pub const LEFT: u32 = 1 << 0;
    pub const RIGHT: u32 = 1 << 1;
    pub const MIDDLE: u32 = 1 << 2;
    /// Not the 3DO mouse's.
    pub const BACK: u32 = 1 << 3;
    pub const FORWARD: u32 = 1 << 4;
}

/// One report of a physical mouse from the dock, as received.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct MouseReport {
    /// Movement since the previous report, in the mouse's counts: x
    /// positive right (the dock's lx), y positive down (ly).
    pub dx: i16,
    pub dy: i16,
    /// The wheel's movement (the dock's lz, two's complement); not the 3DO
    /// mouse's.
    pub wheel: i16,
    /// The dock's buttons word, `button::*`.
    pub buttons: u32,
}

impl MouseReport {
    /// Whether a dock gamepad's buttons word is a mouse's.
    pub const fn is_mouse(buttons: u32) -> bool {
        (buttons & button::MOUSE) != 0
    }

    pub const fn pressed(&self, mask: u32) -> bool {
        (self.buttons & mask) == mask
    }

    pub const fn left(&self) -> bool {
        self.pressed(button::LEFT)
    }

    pub const fn right(&self) -> bool {
        self.pressed(button::RIGHT)
    }

    pub const fn middle(&self) -> bool {
        self.pressed(button::MIDDLE)
    }

    /// The mice on the dock as one, from their latest reports: a button is
    /// held while any mouse holds it; no movement (each report's counts
    /// once, through `Mouse::report`). None without a mouse, so a mouse is
    /// connected while any is.
    pub fn merged(reports: impl IntoIterator<Item = MouseReport>) -> Option<MouseReport> {
        reports.into_iter().fold(None, |merged, report| {
            Some(MouseReport {
                dx: 0,
                dy: 0,
                wheel: 0,
                buttons: merged.map_or(0, |merged: MouseReport| merged.buttons) | report.buttons,
            })
        })
    }
}

/// A physical mouse's sensitivity: the units written to the core per count
/// of the mouse, as the fraction `SENSITIVITY_NUMERATOR` /
/// `SENSITIVITY_DENOMINATOR`. 1: the core moves the 3DO mouse one count per
/// unit, so one count of the mouse moves the 3DO mouse one count. (It was
/// 1/2 while the core doubled, as MiSTer's does for raw PS/2 deltas: the
/// cursor moved in steps of two.) Any fraction of a unit left over after a
/// packet is carried to the next, so nothing is lost.
pub const SENSITIVITY_NUMERATOR: i64 = 1;
pub const SENSITIVITY_DENOMINATOR: i64 = 1;

/// A physical mouse's movement: the reports since the previous packet
/// summed, in `SENSITIVITY_DENOMINATOR`ths of a unit, with what a packet
/// did not take (the fraction of a unit, and anything beyond `MAX_PACKET`)
/// kept for the next.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Motion {
    x: i64,
    y: i64,
}

impl Motion {
    pub const fn new() -> Self {
        Motion { x: 0, y: 0 }
    }

    /// Forget what was not taken (the mouse stopped being the core's).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// One report's movement, in the mouse's counts.
    pub fn add(&mut self, dx: i16, dy: i16) {
        self.x += dx as i64 * SENSITIVITY_NUMERATOR;
        self.y += dy as i64 * SENSITIVITY_NUMERATOR;
    }

    /// The whole units moved since the last take, at most `MAX_PACKET` per
    /// axis (toward zero), taken out: the fraction below one unit and the
    /// excess beyond the clamp stay for the next take.
    pub fn take(&mut self) -> (i16, i16) {
        fn take_axis(accumulated: &mut i64) -> i16 {
            let units = (*accumulated / SENSITIVITY_DENOMINATOR)
                .clamp(-(MAX_PACKET as i64), MAX_PACKET as i64);
            *accumulated -= units * SENSITIVITY_DENOMINATOR;
            units as i16
        }
        (take_axis(&mut self.x), take_axis(&mut self.y))
    }

    /// Whether there is movement not yet taken (a whole unit or a fraction).
    pub fn pending(&self) -> bool {
        self.x != 0 || self.y != 0
    }
}

/// The Controller register's / setting's values.
pub const CONTROLLER_PAD: u32 = 0;
pub const CONTROLLER_LIGHT_GUN: u32 = 1;
pub const CONTROLLER_MOUSE: u32 = 2;

/// Where a core takes a mouse: addresses on the host memory interface.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Target {
    /// A movement packet, `pack_motion`.
    pub motion: u32,
    /// The buttons, `pack_buttons`.
    pub buttons: u32,
}

/// The 3DO core (apolkosnik/3DO_GameBub, `rtl/gamebub/gb_host.sv`).
pub const TARGET_3DO: Target = Target {
    motion: 0x0000_1030,
    buttons: 0x0000_1034,
};

/// The mouse registers of a core, by its ID (core.json), if it takes one.
pub fn target_for_core(core_id: &str) -> Option<Target> {
    match core_id {
        crate::CORE_3DO => Some(TARGET_3DO),
        _ => None,
    }
}

/// The motion register: `{dy[31:16], dx[15:0]}`, each signed.
pub const fn pack_motion(dx: i16, dy: i16) -> u32 {
    ((dy as u16 as u32) << 16) | dx as u16 as u32
}

pub const BUTTON_LEFT: u32 = 1 << 0;
pub const BUTTON_RIGHT: u32 = 1 << 1;
pub const BUTTON_MIDDLE: u32 = 1 << 2;

/// The buttons register: bit 0 left, bit 1 right, bit 2 middle.
pub const fn pack_buttons(left: bool, right: bool, middle: bool) -> u32 {
    (left as u32) * BUTTON_LEFT | (right as u32) * BUTTON_RIGHT | (middle as u32) * BUTTON_MIDDLE
}

/// The stick's deflection (either sign) that does nothing: a quarter of the
/// way, well clear of a worn stick's rest position, since any drift would
/// move the mouse without end.
pub const DEAD_ZONE: i32 = 8192;

/// The deflection past the dead zone at the stick's full travel.
pub const SPAN: i32 = i16::MAX as i32 - DEAD_ZONE;

/// The mouse's speed at full deflection, in units written to the core per
/// second (the core moves the 3DO mouse one count per unit). The speed is
/// linear in the deflection past the dead zone: half way out of it moves
/// half as fast.
pub const MAX_SPEED: i32 = 300;

/// Least time between two of the stick cursor's movement packets (about
/// 120 Hz, as the gun's aim). A physical mouse's reports are not paced: a
/// report is written as it arrives (the core takes a packet at a time, at
/// any rate). Paced to this interval with a millisecond clock, a 125 Hz
/// mouse's 8 ms reports were held back now and then and fell into the 3DO's
/// 16.7 ms fields one or three at a time: the cursor stuttered.
pub const MIN_MOTION_INTERVAL_MS: u64 = 8;

/// The most time one update of the cursor integrates: a long gap between
/// updates (the firmware was busy) moves the mouse this much, not by a jump.
pub const MAX_STEP_MS: u64 = 100;

/// The most movement in one packet per axis (the core's clamp). A physical
/// mouse's movement beyond it waits for the next packet (`Motion`); the
/// stick's is dropped (`Cursor`): at `MAX_SPEED` with `MAX_STEP_MS` that
/// takes more than eight updates without a packet, which the controller
/// does not do.
pub const MAX_PACKET: i32 = 255;

/// The stick's deflection past the dead zone: 0 inside it, then linear to
/// +-`SPAN` at full travel (positive right / down, as the dock's axes).
pub const fn deflection(axis: i16) -> i32 {
    let axis = axis as i32;
    if axis > DEAD_ZONE {
        let past = axis - DEAD_ZONE;
        if past > SPAN {
            SPAN
        } else {
            past
        }
    } else if axis < -DEAD_ZONE {
        let past = axis + DEAD_ZONE;
        if past < -SPAN {
            -SPAN
        } else {
            past
        }
    } else {
        0
    }
}

/// One unit of movement in the accumulators' fixed point: a deflection of
/// `SPAN` held for 1000 ms adds `MAX_SPEED` units.
const UNIT: i64 = SPAN as i64 * 1000;

/// The velocity cursor: the right stick integrated over time, with the
/// fraction of a unit kept between packets.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Cursor {
    /// Movement not yet taken, in `UNIT`s.
    x: i64,
    y: i64,
    /// The stick's deflection since the last update, and when that was.
    held: (i32, i32),
    last_ms: Option<u64>,
}

impl Cursor {
    pub const fn new() -> Self {
        Cursor {
            x: 0,
            y: 0,
            held: (0, 0),
            last_ms: None,
        }
    }

    /// Forget the fraction, the deflection and the time (the mouse stopped
    /// being driven): the next update starts over, without a catch-up.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// The stick at `now_ms`: the deflection held since the previous update
    /// moves the mouse for the time in between (at most `MAX_STEP_MS`), and
    /// the stick's position now is held from here on.
    pub fn update(&mut self, now_ms: u64, stick_x: i16, stick_y: i16) {
        if let Some(at) = self.last_ms {
            let dt = now_ms.saturating_sub(at).min(MAX_STEP_MS) as i64;
            self.x += self.held.0 as i64 * MAX_SPEED as i64 * dt;
            self.y += self.held.1 as i64 * MAX_SPEED as i64 * dt;
        }
        self.last_ms = Some(now_ms);
        self.held = (deflection(stick_x), deflection(stick_y));
    }

    /// The whole units moved since the last take, taken out: the fraction
    /// below one unit stays for the next; beyond `MAX_PACKET` per axis is
    /// dropped.
    pub fn take(&mut self) -> (i16, i16) {
        fn take_axis(accumulated: &mut i64) -> i16 {
            let units = *accumulated / UNIT;
            *accumulated -= units * UNIT;
            units.clamp(-(MAX_PACKET as i64), MAX_PACKET as i64) as i16
        }
        (take_axis(&mut self.x), take_axis(&mut self.y))
    }

    /// Whether the stick is out of the dead zone (the mouse is moving).
    pub fn moving(&self) -> bool {
        self.held != (0, 0)
    }
}

/// What drives the mouse: a physical mouse on the dock if there is one,
/// else the first dock gamepad (a light gun is not one); and the handheld's
/// own buttons either way.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct MouseInput {
    /// The physical mice on the dock as one, their latest reports' buttons
    /// together (`MouseReport::merged`; None: no mouse on the dock). Only
    /// its buttons are read here: a report's movement counts once, through
    /// `Mouse::report`, since the latest report is the same one until the
    /// next arrives. While it is Some the stick and the pad's buttons below
    /// do nothing.
    pub mouse: Option<MouseReport>,
    /// The pad's right stick (the dock's rx, ry: 0 centre, positive right /
    /// down); 0 without a pad.
    pub stick_x: i16,
    pub stick_y: i16,
    /// The pad's shoulder buttons and right stick click.
    pub pad_r1: bool,
    pub pad_l1: bool,
    pub pad_r3: bool,
    /// The handheld's own L and R.
    pub handheld_l: bool,
    pub handheld_r: bool,
}

impl MouseInput {
    /// The left button: the physical mouse's, or without one the pad's R1
    /// (under the index finger of the hand on the stick); or the handheld's
    /// L.
    pub const fn left(&self) -> bool {
        let dock = match self.mouse {
            Some(mouse) => mouse.left(),
            None => self.pad_r1,
        };
        dock || self.handheld_l
    }

    /// The right button: the physical mouse's, or without one the pad's L1;
    /// or the handheld's R.
    pub const fn right(&self) -> bool {
        let dock = match self.mouse {
            Some(mouse) => mouse.right(),
            None => self.pad_l1,
        };
        dock || self.handheld_r
    }

    /// The middle button: the physical mouse's, or without one the right
    /// stick's click.
    pub const fn middle(&self) -> bool {
        match self.mouse {
            Some(mouse) => mouse.middle(),
            None => self.pad_r3,
        }
    }

    /// The buttons register, `pack_buttons`.
    pub const fn buttons(&self) -> u32 {
        pack_buttons(self.left(), self.right(), self.middle())
    }
}

/// The mouse of the running core: what to write, and when.
#[derive(Clone, Debug)]
pub struct Mouse {
    target: Option<Target>,
    /// The core's Controller setting (`CONTROLLER_*`).
    controller: u32,
    /// A physical mouse's movement since the previous packet.
    motion: Motion,
    /// The stick's.
    cursor: Cursor,
    sent_buttons: Option<u32>,
    last_motion_write: Option<u64>,
}

impl Default for Mouse {
    fn default() -> Self {
        Self::new()
    }
}

impl Mouse {
    pub const fn new() -> Self {
        Mouse {
            target: None,
            controller: CONTROLLER_PAD,
            motion: Motion::new(),
            cursor: Cursor::new(),
            sent_buttons: None,
            last_motion_write: None,
        }
    }

    /// The core taking a mouse now (None: no core, or one without a mouse,
    /// or not running with focus), with the value of its Controller setting.
    /// The buttons are written again on the next `update`.
    pub fn set_target(&mut self, target: Option<Target>, controller: u32) {
        *self = Mouse {
            target,
            controller,
            ..Self::new()
        };
    }

    /// Whether the running core takes the mouse and its setting selects it
    /// (so the firmware should update it regularly, a stick held still
    /// moving it between reports).
    pub fn enabled(&self) -> bool {
        self.target.is_some() && self.controller == CONTROLLER_MOUSE
    }

    pub fn target(&self) -> Option<Target> {
        self.target
    }

    pub fn controller(&self) -> u32 {
        self.controller
    }

    /// A report of the physical mouse, as it arrives (each one once): its
    /// movement is added to the next packet's, while the core takes the
    /// mouse and the setting selects it (else it is dropped: nothing is
    /// saved up for later). The buttons are the next `update`'s, through
    /// `MouseInput::mouse`.
    pub fn report(&mut self, report: &MouseReport) {
        if self.enabled() {
            self.motion.add(report.dx, report.dy);
        }
    }

    /// The inputs at `now_ms`: the writes for the core. `gun_connected`: a
    /// light gun is on the dock, and the core's (the gun code holds the
    /// Controller register at 1); the mouse then stands still, its buttons
    /// released, as it does while the setting is not `CONTROLLER_MOUSE`.
    ///
    /// A movement packet is written when there are whole units to move:
    /// the physical mouse's reports since the previous update at once, the
    /// stick's integral at most every `MIN_MOTION_INTERVAL_MS`; a button
    /// change at once.
    pub fn update(&mut self, now_ms: u64, gun_connected: bool, input: &MouseInput) -> Writes {
        let mut writes = Writes::default();
        let Some(target) = self.target else {
            return writes;
        };

        if !self.enabled() || gun_connected {
            self.motion.reset();
            self.cursor.reset();
            self.last_motion_write = None;
            if self.sent_buttons.is_some_and(|buttons| buttons != 0) {
                writes.push(target.buttons, 0);
                self.sent_buttons = Some(0);
            }
            return writes;
        }

        // A physical mouse on the dock drives it; the stick waits, and
        // starts over (no catch-up) when the mouse goes. Without one the
        // stick drives it, and the mouse's leftovers go.
        let physical = input.mouse.is_some();
        if physical {
            self.cursor.reset();
        } else {
            self.motion.reset();
            self.cursor.update(now_ms, input.stick_x, input.stick_y);
        }
        // The mouse's reports go out as they come (the firmware updates on
        // every report); the stick's cursor is paced.
        let due = physical
            || self
                .last_motion_write
                .is_none_or(|at| now_ms.saturating_sub(at) >= MIN_MOTION_INTERVAL_MS);
        if due {
            let (dx, dy) = if physical {
                self.motion.take()
            } else {
                self.cursor.take()
            };
            if (dx, dy) != (0, 0) {
                writes.push(target.motion, pack_motion(dx, dy));
                self.last_motion_write = Some(now_ms);
            }
        }

        let buttons = input.buttons();
        if self.sent_buttons != Some(buttons) {
            writes.push(target.buttons, buttons);
            self.sent_buttons = Some(buttons);
        }
        writes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FULL: i16 = i16::MAX;

    fn stick(x: i16, y: i16) -> MouseInput {
        MouseInput {
            stick_x: x,
            stick_y: y,
            ..MouseInput::default()
        }
    }

    fn writes(w: &Writes) -> Vec<(u32, u32)> {
        w.as_slice().to_vec()
    }

    /// A dock mouse's report with these buttons and no movement.
    fn dock_mouse(buttons: u32) -> MouseReport {
        MouseReport {
            dx: 0,
            dy: 0,
            wheel: 0,
            buttons: button::MOUSE | buttons,
        }
    }

    /// A dock mouse's report of this movement, no buttons.
    fn moved(dx: i16, dy: i16) -> MouseReport {
        MouseReport {
            dx,
            dy,
            ..dock_mouse(0)
        }
    }

    /// A dock mouse with these buttons, nothing else.
    fn physical(buttons: u32) -> MouseInput {
        MouseInput {
            mouse: Some(dock_mouse(buttons)),
            ..MouseInput::default()
        }
    }

    #[test]
    fn target_only_for_the_3do_core() {
        assert_eq!(
            target_for_core("apolkosnik.3DO"),
            Some(Target {
                motion: 0x1030,
                buttons: 0x1034
            })
        );
        assert_eq!(target_for_core("Game-Bub.GBA"), None);
        assert_eq!(target_for_core("apolkosnik.3do"), None);
        assert_eq!(target_for_core(""), None);
    }

    #[test]
    fn controller_values() {
        assert_eq!(CONTROLLER_PAD, 0);
        assert_eq!(CONTROLLER_LIGHT_GUN, 1);
        assert_eq!(CONTROLLER_MOUSE, 2);
    }

    #[test]
    fn motion_packing() {
        assert_eq!(pack_motion(0, 0), 0);
        assert_eq!(pack_motion(1, 0), 0x0000_0001);
        assert_eq!(pack_motion(0, 1), 0x0001_0000);
        assert_eq!(pack_motion(-1, 0), 0x0000_FFFF);
        assert_eq!(pack_motion(0, -1), 0xFFFF_0000);
        assert_eq!(pack_motion(255, -255), 0xFF01_00FF);
        assert_eq!(pack_motion(-2, 3), 0x0003_FFFE);
        assert_eq!(pack_motion(0x1234, 0x5678u16 as i16), 0x5678_1234);
    }

    #[test]
    fn button_packing() {
        assert_eq!(pack_buttons(false, false, false), 0b000);
        assert_eq!(pack_buttons(true, false, false), 0b001);
        assert_eq!(pack_buttons(false, true, false), 0b010);
        assert_eq!(pack_buttons(false, false, true), 0b100);
        assert_eq!(pack_buttons(true, true, true), 0b111);
    }

    #[test]
    fn button_mapping() {
        let none = MouseInput::default();
        assert_eq!(none.buttons(), 0);
        let r1 = MouseInput {
            pad_r1: true,
            ..none
        };
        assert_eq!(r1.buttons(), 0b001, "the pad's R1 is the left button");
        let l1 = MouseInput {
            pad_l1: true,
            ..none
        };
        assert_eq!(l1.buttons(), 0b010, "the pad's L1 is the right button");
        let r3 = MouseInput {
            pad_r3: true,
            ..none
        };
        assert_eq!(
            r3.buttons(),
            0b100,
            "the stick's click is the middle button"
        );
        let l = MouseInput {
            handheld_l: true,
            ..none
        };
        assert_eq!(l.buttons(), 0b001, "the handheld's L is the left button");
        let r = MouseInput {
            handheld_r: true,
            ..none
        };
        assert_eq!(r.buttons(), 0b010, "the handheld's R is the right button");
        let all = MouseInput {
            pad_r1: true,
            pad_l1: true,
            pad_r3: true,
            handheld_l: true,
            handheld_r: true,
            ..none
        };
        assert_eq!(all.buttons(), 0b111);
        // Both sources of a button: still one button.
        let both_left = MouseInput {
            pad_r1: true,
            handheld_l: true,
            ..none
        };
        assert_eq!(both_left.buttons(), 0b001);
        // The stick does not press anything.
        assert_eq!(stick(FULL, i16::MIN).buttons(), 0);

        // With a mouse on the dock: its buttons one to one, and the
        // handheld's; the pad's are the pad's.
        let pad_held = MouseInput {
            mouse: Some(dock_mouse(0)),
            pad_r1: true,
            pad_l1: true,
            pad_r3: true,
            ..none
        };
        assert_eq!(
            pad_held.buttons(),
            0,
            "the pad's buttons while a mouse is there"
        );
        for (bits, expected) in [
            (button::LEFT, 0b001),
            (button::RIGHT, 0b010),
            (button::MIDDLE, 0b100),
            (button::LEFT | button::MIDDLE, 0b101),
            (button::LEFT | button::RIGHT | button::MIDDLE, 0b111),
            (button::BACK | button::FORWARD, 0),
            (0x3FFF_FFF8, 0),
        ] {
            let input = MouseInput {
                mouse: Some(dock_mouse(bits)),
                ..pad_held
            };
            assert_eq!(input.buttons(), expected, "{bits:#x}");
        }
        let l = MouseInput {
            handheld_l: true,
            ..physical(0)
        };
        assert_eq!(l.buttons(), 0b001, "the handheld's L with a mouse");
        let r = MouseInput {
            handheld_r: true,
            ..physical(button::LEFT)
        };
        assert_eq!(r.buttons(), 0b011, "the handheld's R with a mouse");
        // A report's movement does not press anything.
        let moving = MouseInput {
            mouse: Some(moved(i16::MIN, i16::MAX)),
            ..none
        };
        assert_eq!(moving.buttons(), 0);
    }

    #[test]
    fn mouse_marker_is_bit_30() {
        assert!(MouseReport::is_mouse(0x4000_0000));
        assert!(MouseReport::is_mouse(0xC000_0007));
        assert!(
            !MouseReport::is_mouse(0x8000_0000),
            "a light gun is not a mouse"
        );
        assert!(!MouseReport::is_mouse(0xBFFF_FFFF));
        assert!(!MouseReport::is_mouse(0));
        assert!(
            !crate::GunReport::is_light_gun(0x4000_0000),
            "a mouse is not a light gun"
        );
    }

    #[test]
    fn dock_mouse_button_bits() {
        let r = dock_mouse(1 << 0);
        assert!(r.left() && !r.right() && !r.middle());
        let r = dock_mouse(1 << 1);
        assert!(r.right() && !r.left() && !r.middle());
        let r = dock_mouse(1 << 2);
        assert!(r.middle() && !r.left() && !r.right());
        // Back, forward and anything else are not the 3DO's.
        let r = dock_mouse((1 << 3) | (1 << 4) | 0x3FFF_FFE0);
        assert!(!r.left() && !r.right() && !r.middle());
        assert_eq!(button::BACK, 1 << 3);
        assert_eq!(button::FORWARD, 1 << 4);
        // The movement is not a button.
        let r = moved(-1, -1);
        assert!(!r.left() && !r.right() && !r.middle());
    }

    #[test]
    fn sensitivity_is_one() {
        // One count of the mouse is one unit: one count of the 3DO mouse.
        assert_eq!((SENSITIVITY_NUMERATOR, SENSITIVITY_DENOMINATOR), (1, 1));
        let mut m = Motion::new();
        m.add(1, -1);
        assert_eq!(m.take(), (1, -1));
        m.add(100, -200);
        assert_eq!(m.take(), (100, -200));
        assert!(!m.pending());
    }

    #[test]
    fn motion_sums_reports_and_loses_none() {
        let mut m = Motion::new();
        assert!(!m.pending());
        assert_eq!(m.take(), (0, 0));
        // The reports between two packets add up: 3 + 4 + 2 = 9 counts
        // right, 1 + 1 + 1 = 3 down.
        m.add(3, 1);
        m.add(4, 1);
        m.add(2, 1);
        assert!(m.pending());
        assert_eq!(m.take(), (9, 3));
        assert!(!m.pending(), "a count is a whole unit: nothing carried");
        assert_eq!(m.take(), (0, 0));
        // One count at a time, both ways: every take moves, none is lost.
        let mut steps = Vec::new();
        for _ in 0..10 {
            m.add(1, -1);
            steps.push(m.take());
        }
        assert_eq!(steps, [(1, -1)].repeat(10));
        // Toward zero too.
        m.add(-3, 3);
        assert_eq!(m.take(), (-3, 3));
        assert!(!m.pending());
        // A report back cancels.
        m.add(1, 0);
        m.add(-1, 0);
        assert_eq!(m.take(), (0, 0));
        assert!(!m.pending());
        // Reset forgets what was not taken.
        m.add(1, 1);
        m.reset();
        assert!(!m.pending());
        assert_eq!(m.take(), (0, 0));
    }

    #[test]
    fn motion_is_clamped_and_the_excess_carried() {
        let mut m = Motion::new();
        // A jerk of 1000 counts right and 700 up: 255 a packet.
        m.add(1000, -700);
        assert_eq!(m.take(), (255, -255));
        assert!(m.pending());
        assert_eq!(m.take(), (255, -255), "the rest in the next packets");
        assert_eq!(m.take(), (255, -190));
        assert_eq!(m.take(), (235, 0), "nothing lost");
        assert_eq!(m.take(), (0, 0));
        assert!(!m.pending());
        // The biggest reports there are, many of them: taken 255 at a
        // time, the sum conserved.
        let n = 101;
        for _ in 0..n {
            m.add(i16::MAX, i16::MIN);
        }
        let expected = (n as i64 * i16::MAX as i64, n as i64 * i16::MIN as i64);
        let mut total = (0i64, 0i64);
        let mut packets = 0;
        loop {
            let (dx, dy) = m.take();
            if (dx, dy) == (0, 0) {
                break;
            }
            assert!(dx.abs() <= 255 && dy.abs() <= 255, "{dx} {dy}");
            total.0 += dx as i64;
            total.1 += dy as i64;
            packets += 1;
        }
        assert_eq!(total, expected);
        assert_eq!(packets, (expected.1.abs() + 254) / 255);
        assert_eq!(m, Motion { x: 0, y: 0 }, "nothing left: a count is a unit");
    }

    #[test]
    fn physical_mouse_stream() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        // Taken with a mouse on the dock: the buttons are written (released).
        assert_eq!(writes(&m.update(0, false, &physical(0))), [(0x1034, 0)]);
        // A report: written at once, count for count.
        m.report(&moved(3, 0));
        assert_eq!(
            writes(&m.update(1, false, &physical(0))),
            [(0x1030, pack_motion(3, 0))]
        );
        // The next ones too, however soon: a report is not held back for an
        // interval (held back, a 125 Hz mouse's reports fell into the 3DO's
        // fields one or three at a time).
        m.report(&moved(3, 0));
        assert_eq!(
            writes(&m.update(3, false, &physical(0))),
            [(0x1030, pack_motion(3, 0))]
        );
        m.report(&moved(2, -5));
        assert_eq!(
            writes(&m.update(5, false, &physical(0))),
            [(0x1030, pack_motion(2, -5))]
        );
        for t in 6..=30u64 {
            m.report(&moved(1, 1));
            assert_eq!(
                writes(&m.update(t, false, &physical(0))),
                [(0x1030, pack_motion(1, 1))],
                "at {t}"
            );
        }
        // Reports that came between two updates add up into one packet.
        m.report(&moved(1, 0));
        m.report(&moved(2, -1));
        assert_eq!(
            writes(&m.update(31, false, &physical(0))),
            [(0x1030, pack_motion(3, -1))]
        );
        // Nothing new: the ticks write nothing.
        assert!(m.update(39, false, &physical(0)).is_empty());
        assert!(m.update(47, false, &physical(0)).is_empty());
        // The latest report's movement is not counted by an update, however
        // many: it was counted when it arrived.
        let same = MouseInput {
            mouse: Some(moved(100, 100)),
            ..MouseInput::default()
        };
        assert!(m.update(55, false, &same).is_empty());
        assert!(m.update(63, false, &same).is_empty());
        // Left pressed, no movement: the button at once, alone.
        m.report(&dock_mouse(button::LEFT));
        assert_eq!(
            writes(&m.update(64, false, &physical(button::LEFT))),
            [(0x1034, 0b001)]
        );
        // Held: nothing.
        m.report(&dock_mouse(button::LEFT));
        assert!(m.update(65, false, &physical(button::LEFT)).is_empty());
        // Moved as it is released and the middle one pressed: motion, then
        // the buttons.
        m.report(&MouseReport {
            dx: -2,
            dy: 4,
            ..dock_mouse(button::MIDDLE)
        });
        assert_eq!(
            writes(&m.update(66, false, &physical(button::MIDDLE))),
            [(0x1030, pack_motion(-2, 4)), (0x1034, 0b100)]
        );
        // All three, and the handheld's R too: one right button.
        let all = MouseInput {
            handheld_r: true,
            ..physical(button::LEFT | button::RIGHT | button::MIDDLE)
        };
        assert_eq!(writes(&m.update(67, false, &all)), [(0x1034, 0b111)]);
        // Back, forward and the wheel do nothing.
        let other = button::BACK | button::FORWARD;
        m.report(&MouseReport {
            wheel: -3,
            ..dock_mouse(other)
        });
        assert_eq!(
            writes(&m.update(75, false, &physical(other))),
            [(0x1034, 0)]
        );
        assert!(m.update(83, false, &physical(other)).is_empty());
    }

    #[test]
    fn physical_mouse_clamps_a_packet_and_carries_the_rest() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        m.update(0, false, &physical(0));
        // 300 reports of 4 counts right and 1 up before the next update
        // (a fast mouse, a fast hand): 1200 counts right, 300 up. The packet
        // is clamped; the rest follows with the next updates (the next
        // reports, or the ticks), nothing lost.
        for _ in 0..300 {
            m.report(&moved(4, -1));
        }
        assert_eq!(
            writes(&m.update(1, false, &physical(0))),
            [(0x1030, pack_motion(255, -255))]
        );
        assert_eq!(
            writes(&m.update(2, false, &physical(0))),
            [(0x1030, pack_motion(255, -45))]
        );
        assert_eq!(
            writes(&m.update(3, false, &physical(0))),
            [(0x1030, pack_motion(255, 0))]
        );
        assert_eq!(
            writes(&m.update(4, false, &physical(0))),
            [(0x1030, pack_motion(255, 0))]
        );
        assert_eq!(
            writes(&m.update(12, false, &physical(0))),
            [(0x1030, pack_motion(180, 0))]
        );
        assert!(m.update(20, false, &physical(0)).is_empty());
    }

    #[test]
    fn physical_mouse_wins_over_the_stick() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        // A pad with its stick full right-down and R1, L1, R3 held, and a
        // mouse: the mouse's buttons (none), not the pad's.
        let both = MouseInput {
            stick_x: FULL,
            stick_y: FULL,
            pad_r1: true,
            pad_l1: true,
            pad_r3: true,
            ..physical(0)
        };
        assert_eq!(writes(&m.update(0, false, &both)), [(0x1034, 0)]);
        // The stick moves nothing, however long.
        for t in 1..=100u64 {
            assert!(m.update(8 * t, false, &both).is_empty(), "at {}", 8 * t);
        }
        // The mouse's buttons do...
        let both_left = MouseInput {
            mouse: Some(dock_mouse(button::LEFT)),
            ..both
        };
        assert_eq!(writes(&m.update(900, false, &both_left)), [(0x1034, 0b001)]);
        // ...as the handheld's.
        let both_left_r = MouseInput {
            handheld_r: true,
            ..both_left
        };
        assert_eq!(
            writes(&m.update(901, false, &both_left_r)),
            [(0x1034, 0b011)]
        );
        // The mouse's movement does (11 counts: 11 units).
        m.report(&moved(11, 0));
        assert_eq!(
            writes(&m.update(910, false, &both_left_r)),
            [(0x1030, pack_motion(11, 0))]
        );
        // The mouse goes (unplugged), the stick still full with the pad's
        // buttons held: those are the mouse's now...
        let stick_only = MouseInput {
            mouse: None,
            ..both
        };
        assert_eq!(
            writes(&m.update(920, false, &stick_only)),
            [(0x1034, 0b111)]
        );
        // ...and the stick moves it from here on, not for the time it was
        // held while the mouse was there.
        assert!(m.update(921, false, &stick_only).is_empty());
        assert_eq!(
            writes(&m.update(928, false, &stick_only)),
            [(0x1030, pack_motion(2, 2))]
        );
        // The mouse back: the stick stops at once and the pad's buttons go
        // (the mouse's are released); the mouse's report is written.
        m.report(&moved(1, 2));
        assert_eq!(
            writes(&m.update(936, false, &both)),
            [(0x1030, pack_motion(1, 2)), (0x1034, 0)]
        );
        assert!(m.update(944, false, &both).is_empty());
        assert!(m.update(1000, false, &both).is_empty());
        // Gone again after a long while: the stick starts over too, with no
        // catch-up for the time the mouse was there (not even a step).
        assert_eq!(
            writes(&m.update(1500, false, &stick_only)),
            [(0x1034, 0b111)]
        );
        assert_eq!(
            writes(&m.update(1508, false, &stick_only)),
            [(0x1030, pack_motion(2, 2))]
        );
    }

    #[test]
    fn physical_mouse_yields_to_a_connected_gun() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        assert_eq!(
            writes(&m.update(0, false, &physical(button::LEFT))),
            [(0x1034, 0b001)]
        );
        m.report(&moved(6, 6));
        assert_eq!(
            writes(&m.update(8, false, &physical(button::LEFT))),
            [(0x1030, pack_motion(6, 6))]
        );
        // A gun is plugged in: the button is released; the mouse's reports
        // go nowhere.
        m.report(&moved(6, 6));
        assert_eq!(
            writes(&m.update(16, true, &physical(button::LEFT))),
            [(0x1034, 0)]
        );
        for t in 3..100u64 {
            m.report(&moved(6, 6));
            assert!(m.update(8 * t, true, &physical(button::LEFT)).is_empty());
        }
        assert!(m.enabled(), "still the core's mouse, waiting");
        // Gone: the button is written again; what moved meanwhile is not.
        assert_eq!(
            writes(&m.update(1000, false, &physical(button::LEFT))),
            [(0x1034, 0b001)]
        );
        assert!(m.update(1008, false, &physical(button::LEFT)).is_empty());
        m.report(&moved(-2, 0));
        assert_eq!(
            writes(&m.update(1016, false, &physical(button::LEFT))),
            [(0x1030, pack_motion(-2, 0))]
        );
    }

    #[test]
    fn physical_mouse_reports_are_dropped_while_not_selected() {
        let mut m = Mouse::new();
        // No core: a report is not kept.
        m.report(&moved(100, 100));
        assert!(!m.motion.pending());
        assert!(m.update(0, false, &physical(0)).is_empty());
        // The 3DO with the pad only: not kept either...
        m.set_target(Some(TARGET_3DO), CONTROLLER_PAD);
        assert!(!m.enabled());
        m.report(&moved(100, 100));
        assert!(!m.motion.pending());
        assert!(m.update(10, false, &physical(0)).is_empty());
        // ...for when the mouse is selected (the setting changed in the
        // menu: the target is set again).
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        assert_eq!(writes(&m.update(20, false, &physical(0))), [(0x1034, 0)]);
        assert!(m.update(28, false, &physical(0)).is_empty());
        // Now they count.
        m.report(&moved(2, 2));
        assert!(m.motion.pending());
        assert_eq!(
            writes(&m.update(36, false, &physical(0))),
            [(0x1030, pack_motion(2, 2))]
        );
        // The core lost the focus: dropped again.
        m.set_target(None, CONTROLLER_MOUSE);
        m.report(&moved(100, 100));
        assert!(!m.motion.pending());
        assert!(m.update(44, false, &physical(0)).is_empty());
    }

    #[test]
    fn two_mice_are_one() {
        assert_eq!(MouseReport::merged([]), None, "no mouse");
        // One mouse: its buttons, and no movement (counted per report).
        assert_eq!(
            MouseReport::merged([MouseReport {
                dx: 5,
                dy: -5,
                wheel: 1,
                ..dock_mouse(button::LEFT)
            }]),
            Some(dock_mouse(button::LEFT))
        );
        // Two: a button held on either is held, whichever comes first.
        let buttons = |reports: &[MouseReport]| {
            let merged = MouseReport::merged(reports.iter().copied());
            assert!(merged.is_some(), "a mouse is connected while any is");
            MouseInput {
                mouse: merged,
                ..MouseInput::default()
            }
            .buttons()
        };
        assert_eq!(buttons(&[dock_mouse(0), dock_mouse(0)]), 0);
        assert_eq!(buttons(&[dock_mouse(button::LEFT), dock_mouse(0)]), 0b001);
        assert_eq!(buttons(&[dock_mouse(0), dock_mouse(button::LEFT)]), 0b001);
        assert_eq!(buttons(&[moved(3, 3), dock_mouse(button::RIGHT)]), 0b010);
        assert_eq!(
            buttons(&[dock_mouse(button::LEFT), dock_mouse(button::MIDDLE)]),
            0b101
        );
        assert_eq!(
            buttons(&[dock_mouse(button::MIDDLE), dock_mouse(button::LEFT)]),
            0b101
        );
        assert_eq!(
            buttons(&[
                dock_mouse(button::LEFT),
                dock_mouse(button::LEFT),
                dock_mouse(button::RIGHT)
            ]),
            0b011
        );
        assert_eq!(
            buttons(&[
                dock_mouse(button::BACK),
                dock_mouse(button::FORWARD),
                moved(1, 1)
            ]),
            0,
            "nothing the 3DO mouse has"
        );

        // In the stream: both mice's movement adds up, either's buttons
        // count, and one left is still a mouse (the stick does nothing).
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        let both = |a: MouseReport, b: MouseReport| MouseInput {
            mouse: MouseReport::merged([a, b]),
            stick_x: FULL,
            pad_r1: true,
            ..MouseInput::default()
        };
        assert_eq!(
            writes(&m.update(0, false, &both(dock_mouse(0), dock_mouse(0)))),
            [(0x1034, 0)]
        );
        m.report(&moved(2, 0));
        m.report(&moved(0, 2));
        assert_eq!(
            writes(&m.update(8, false, &both(moved(2, 0), moved(0, 2)))),
            [(0x1030, pack_motion(2, 2))]
        );
        // The second mouse presses left.
        assert_eq!(
            writes(&m.update(9, false, &both(moved(2, 0), dock_mouse(button::LEFT)))),
            [(0x1034, 0b001)]
        );
        // The first presses right while the second holds left.
        assert_eq!(
            writes(&m.update(
                10,
                false,
                &both(dock_mouse(button::RIGHT), dock_mouse(button::LEFT))
            )),
            [(0x1034, 0b011)]
        );
        // The second releases: the first's right stays.
        assert_eq!(
            writes(&m.update(11, false, &both(dock_mouse(button::RIGHT), dock_mouse(0)))),
            [(0x1034, 0b010)]
        );
        // The second is unplugged: still a mouse, its right still held,
        // and the stick (full right all along) still does nothing.
        let one = MouseInput {
            mouse: MouseReport::merged([dock_mouse(button::RIGHT)]),
            ..both(dock_mouse(0), dock_mouse(0))
        };
        assert!(m.update(20, false, &one).is_empty());
        assert!(m.update(28, false, &one).is_empty());
        assert!(m.update(100, false, &one).is_empty());
    }

    #[test]
    fn dead_zone() {
        assert_eq!(deflection(0), 0);
        assert_eq!(deflection(8192), 0);
        assert_eq!(deflection(-8192), 0);
        assert_eq!(deflection(8193), 1);
        assert_eq!(deflection(-8193), -1);
        assert_eq!(deflection(FULL), SPAN);
        assert_eq!(deflection(i16::MIN), -SPAN, "symmetric at full travel");
        assert_eq!(deflection(8192 + SPAN as i16 / 2), SPAN / 2);
        assert_eq!(SPAN, 24575);
    }

    #[test]
    fn full_deflection_moves_300_units_per_second() {
        let mut c = Cursor::new();
        c.update(0, FULL, 0);
        assert_eq!(c.take(), (0, 0), "nothing yet: no time has passed");
        assert!(c.moving());
        // A second, in steps of the most one update counts: 30 each.
        for i in 1..=10u64 {
            c.update(100 * i, FULL, 0);
            assert_eq!(c.take(), (30, 0), "step {i}");
        }
        // Y, the other way.
        let mut c = Cursor::new();
        c.update(0, 0, i16::MIN);
        for i in 1..=10u64 {
            c.update(100 * i, 0, i16::MIN);
            assert_eq!(c.take(), (0, -30), "step {i}");
        }
        // In 8 ms steps: the same over a second, the fraction carried.
        let mut c = Cursor::new();
        c.update(0, -FULL, FULL);
        let mut steps = Vec::new();
        for i in 1..=125u64 {
            c.update(8 * i, -FULL, FULL);
            steps.push(c.take());
        }
        assert!(
            steps.iter().all(|&(x, y)| (2..=3).contains(&y) && x == -y),
            "{steps:?}"
        );
        let total: (i32, i32) = steps
            .iter()
            .fold((0, 0), |(x, y), &(dx, dy)| (x + dx as i32, y + dy as i32));
        assert_eq!(total, (-300, 300));
    }

    #[test]
    fn half_deflection_moves_half_as_fast() {
        let mut c = Cursor::new();
        let half = (DEAD_ZONE + SPAN / 2) as i16;
        c.update(0, half, -half);
        for i in 1..=10u64 {
            c.update(100 * i, half, -half);
        }
        let (x, y) = c.take();
        assert!((149..=150).contains(&x) && y == -x, "{x} {y}");
    }

    #[test]
    fn slow_deflection_accumulates() {
        // A tenth of the way out of the dead zone: 30 units per second,
        // less than one per 8 ms step.
        let tenth = (DEAD_ZONE + SPAN / 10) as i16;
        let mut c = Cursor::new();
        c.update(0, tenth, 0);
        let mut steps = Vec::new();
        for i in 1..=125u64 {
            c.update(8 * i, tenth, 0);
            steps.push(c.take().0);
        }
        assert!(steps.iter().any(|&dx| dx == 0), "{steps:?}");
        assert!(steps.iter().all(|&dx| (0..=1).contains(&dx)), "{steps:?}");
        let total: i32 = steps.iter().map(|&dx| dx as i32).sum();
        assert!((29..=30).contains(&total), "{total}");
        // Just out of the dead zone (a unit every 82 s): it still gets
        // there, slowly.
        let mut c = Cursor::new();
        c.update(0, 8193, 0);
        for i in 1..=900u64 {
            c.update(100 * i, 8193, 0);
            match c.take() {
                (0, 0) => continue,
                (1, 0) => {
                    assert_eq!(i, 820, "at {i} s / 10");
                    return;
                }
                other => panic!("{other:?} at {i}"),
            }
        }
        panic!("never moved");
    }

    #[test]
    fn inside_the_dead_zone_nothing_moves() {
        let mut c = Cursor::new();
        c.update(0, 8192, -8192);
        assert!(!c.moving());
        c.update(60_000, 8192, -8192);
        assert_eq!(c.take(), (0, 0));
        assert_eq!(c, {
            let mut fresh = Cursor::new();
            fresh.update(60_000, 0, 0);
            fresh
        });
    }

    #[test]
    fn the_deflection_held_counts_not_the_new_one() {
        let mut c = Cursor::new();
        c.update(0, 0, 0);
        // The stick was centred during these 50 ms.
        c.update(50, FULL, 0);
        assert_eq!(c.take(), (0, 0));
        // And full during these.
        c.update(100, 0, 0);
        assert_eq!(c.take(), (15, 0));
    }

    #[test]
    fn a_long_gap_is_a_step_not_a_jump() {
        let mut c = Cursor::new();
        c.update(0, FULL, FULL);
        c.update(5000, FULL, FULL);
        assert_eq!(c.take(), (30, 30));
        // Time going backwards (it does not) is no time.
        c.update(4000, FULL, FULL);
        assert_eq!(c.take(), (0, 0));
    }

    #[test]
    fn a_packet_is_clamped_and_the_excess_dropped() {
        let mut c = Cursor::new();
        c.update(0, FULL, -FULL);
        for i in 1..=10u64 {
            c.update(100 * i, FULL, -FULL);
        }
        assert_eq!(c.take(), (255, -255));
        assert_eq!(c.take(), (0, 0));
    }

    #[test]
    fn reset_forgets_everything() {
        let mut c = Cursor::new();
        c.update(0, FULL, 0);
        c.update(50, FULL, 0);
        c.reset();
        assert!(!c.moving());
        c.update(1000, 0, 0);
        assert_eq!(c.take(), (0, 0));
    }

    #[test]
    fn mouse_inactive_writes_nothing() {
        let mut m = Mouse::new();
        let held = MouseInput {
            pad_r1: true,
            ..stick(FULL, FULL)
        };
        assert!(!m.enabled());
        assert!(m.update(0, false, &held).is_empty());
        assert!(m.update(1000, false, &held).is_empty());
        // Another core.
        m.set_target(target_for_core("Game-Bub.GB"), CONTROLLER_MOUSE);
        assert!(!m.enabled());
        assert!(m.update(0, false, &held).is_empty());
        assert!(m.update(1000, false, &held).is_empty());
        // The 3DO, but the setting is not the mouse.
        for controller in [CONTROLLER_PAD, CONTROLLER_LIGHT_GUN, 3] {
            m.set_target(Some(TARGET_3DO), controller);
            assert!(!m.enabled());
            assert_eq!(m.controller(), controller);
            assert!(m.update(0, false, &held).is_empty());
            assert!(m.update(1000, false, &held).is_empty());
        }
    }

    #[test]
    fn mouse_stream() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        assert!(m.enabled());
        assert_eq!(m.target(), Some(TARGET_3DO));
        // Taken: the buttons are written (released), nothing moves.
        assert_eq!(writes(&m.update(0, false, &stick(0, 0))), [(0x1034, 0)]);
        assert!(m.update(4, false, &stick(0, 0)).is_empty());
        // The stick goes full right at 8 ms: held from there.
        assert!(m.update(8, false, &stick(FULL, 0)).is_empty());
        // 8 ms later: 2.4 units, so 2, with 0.4 carried.
        assert_eq!(
            writes(&m.update(16, false, &stick(FULL, 0))),
            [(0x1030, pack_motion(2, 0))]
        );
        // 4 ms later: 1.6 units, but not due.
        assert!(m.update(20, false, &stick(FULL, 0)).is_empty());
        // Due: 2.8 units, so 2, 0.8 carried.
        assert_eq!(
            writes(&m.update(24, false, &stick(FULL, 0))),
            [(0x1030, pack_motion(2, 0))]
        );
        // 3.2: 3, 0.2 carried.
        assert_eq!(
            writes(&m.update(32, false, &stick(FULL, 0))),
            [(0x1030, pack_motion(3, 0))]
        );
        // A button pressed between packets is written at once, alone.
        let r1 = MouseInput {
            pad_r1: true,
            ..stick(FULL, 0)
        };
        assert_eq!(writes(&m.update(33, false, &r1)), [(0x1034, 0b001)]);
        // The stick back to the centre, button still held: 0.8 unit
        // pending (0.2, then 2 ms of full), nothing whole to move.
        let r1_centred = MouseInput {
            pad_r1: true,
            ..stick(0, 0)
        };
        assert!(m.update(34, false, &r1_centred).is_empty());
        assert!(m.update(50, false, &r1_centred).is_empty());
        // Released, diagonally up-left, with the handheld's R.
        let r_up_left = MouseInput {
            handheld_r: true,
            ..stick(i16::MIN, i16::MIN)
        };
        assert_eq!(writes(&m.update(60, false, &r_up_left)), [(0x1034, 0b010)]);
        // 8 ms of full up-left from 60: -2.4 each, with the carried +0.8
        // in x: x -1.6 -> -1, y -2.4 -> -2 (toward zero).
        assert_eq!(
            writes(&m.update(68, false, &r_up_left)),
            [(0x1030, pack_motion(-1, -2))]
        );
        // Motion and a button change together: motion first.
        assert_eq!(
            writes(&m.update(76, false, &stick(i16::MIN, i16::MIN))),
            [(0x1030, pack_motion(-3, -2)), (0x1034, 0)]
        );
    }

    #[test]
    fn mouse_clicks_from_the_handheld_without_a_pad() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        m.update(0, false, &MouseInput::default());
        let l = MouseInput {
            handheld_l: true,
            ..MouseInput::default()
        };
        assert_eq!(writes(&m.update(10, false, &l)), [(0x1034, 0b001)]);
        let lr = MouseInput {
            handheld_r: true,
            ..l
        };
        assert_eq!(writes(&m.update(20, false, &lr)), [(0x1034, 0b011)]);
        assert!(m.update(1000, false, &lr).is_empty());
        assert_eq!(
            writes(&m.update(1010, false, &MouseInput::default())),
            [(0x1034, 0)]
        );
    }

    #[test]
    fn mouse_yields_to_a_connected_gun() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        let held = MouseInput {
            pad_r1: true,
            ..stick(FULL, 0)
        };
        assert_eq!(writes(&m.update(0, false, &held)), [(0x1034, 0b001)]);
        assert_eq!(
            writes(&m.update(8, false, &held)),
            [(0x1030, pack_motion(2, 0))]
        );
        // A gun is plugged in: the button is released, nothing else.
        assert_eq!(writes(&m.update(16, true, &held)), [(0x1034, 0)]);
        assert!(m.update(24, true, &held).is_empty());
        assert!(m.update(1000, true, &held).is_empty());
        assert!(m.enabled(), "still the core's mouse, waiting");
        // Gone again: the mouse goes on, without a catch-up for the time
        // the gun had, and the button is written again.
        assert_eq!(writes(&m.update(2000, false, &held)), [(0x1034, 0b001)]);
        assert_eq!(
            writes(&m.update(2008, false, &held)),
            [(0x1030, pack_motion(2, 0))]
        );
    }

    #[test]
    fn mouse_rewrites_buttons_on_new_target() {
        let mut m = Mouse::new();
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        let held = MouseInput {
            pad_r3: true,
            ..MouseInput::default()
        };
        assert_eq!(writes(&m.update(0, false, &held)), [(0x1034, 0b100)]);
        assert!(m.update(100, false, &held).is_empty());
        // Refocused with the mouse still selected: written again.
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        assert_eq!(writes(&m.update(200, false, &held)), [(0x1034, 0b100)]);
        // Refocused with the pad only: nothing (the setting write took
        // the register; a held button is not the mouse's any more).
        m.set_target(Some(TARGET_3DO), CONTROLLER_PAD);
        assert!(m.update(300, false, &held).is_empty());
        // Deselected while held: released once.
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        m.update(400, false, &held);
        m.set_target(Some(TARGET_3DO), CONTROLLER_MOUSE);
        m.update(500, false, &held);
        assert_eq!(writes(&m.update(600, true, &held)), [(0x1034, 0)]);
        assert!(m.update(700, true, &held).is_empty());
    }
}
