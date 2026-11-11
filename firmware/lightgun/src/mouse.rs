//! A 3DO mouse from a gamepad's right stick.
//!
//! The 3DO core takes a mouse on its control port beside the pad while its
//! Controller register (`crate::Target::enable`, 0x1014) holds
//! `CONTROLLER_MOUSE`. The register is also a setting of the core
//! (settings.json: 0 pad only, 1 pad + light gun, 2 pad + mouse), so the
//! user picks the mouse in the core's settings menu; the firmware passes the
//! setting's value to `Mouse::set_target`, and the mouse works only while it
//! is `CONTROLLER_MOUSE`.
//!
//! The mouse is a velocity cursor on the first dock gamepad's right stick
//! (`Cursor`): the stick's deflection past `DEAD_ZONE` is a speed, linear up
//! to `MAX_SPEED` units per second at full deflection, integrated over time.
//! What is below one unit stays accumulated, so a slow deflection still
//! moves. The core gets, over the host memory interface (`Target`):
//!
//!  * `Target::motion`: `{dy[31:16], dx[15:0]}`, two signed 16-bit units
//!    moved since the previous write (x right, y down), see `pack_motion`.
//!    Each write is one movement packet (the core moves the 3DO mouse 2
//!    counts per unit and clamps a packet to +-255 per axis), so packets are
//!    small and frequent: written only when there is movement, at most
//!    every `MIN_MOTION_INTERVAL_MS`;
//!  * `Target::buttons`: bit 0 left, bit 1 right, bit 2 middle (held = 1),
//!    see `pack_buttons`, written on a change. The pad's R1 is left, L1
//!    right, R3 (the right stick's click) middle; the handheld's own L and
//!    R are left and right too, so the mouse clicks without a dock pad
//!    (`MouseInput::buttons`).
//!
//! A light gun on the dock wins: while one is connected the gun code holds
//! the Controller register at 1 (`crate::Controller`) and the mouse stands
//! still with its buttons released (`Mouse::update`'s `gun_connected`).
//! When the gun is unplugged the gun code writes the setting's value back,
//! and the mouse goes on.

use crate::Writes;

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
/// second (the core moves the 3DO mouse 2 counts per unit). The speed is
/// linear in the deflection past the dead zone: half way out of it moves
/// half as fast.
pub const MAX_SPEED: i32 = 300;

/// Least time between two movement packets (about 120 Hz, as the gun's
/// aim): the core takes a packet at a time.
pub const MIN_MOTION_INTERVAL_MS: u64 = 8;

/// The most time one update of the cursor integrates: a long gap between
/// updates (the firmware was busy) moves the mouse this much, not by a jump.
pub const MAX_STEP_MS: u64 = 100;

/// The most movement in one packet per axis (the core's clamp). Beyond it
/// movement is dropped; at `MAX_SPEED` with `MAX_STEP_MS` that takes more
/// than eight updates without a packet, which the controller does not do.
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

/// What drives the mouse: the first dock gamepad (a light gun is not one)
/// and the handheld's own buttons.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct MouseInput {
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
    /// The left button: the pad's R1 (under the index finger of the hand
    /// on the stick), or the handheld's L.
    pub const fn left(&self) -> bool {
        self.pad_r1 || self.handheld_l
    }

    /// The right button: the pad's L1, or the handheld's R.
    pub const fn right(&self) -> bool {
        self.pad_l1 || self.handheld_r
    }

    /// The middle button: the right stick's click.
    pub const fn middle(&self) -> bool {
        self.pad_r3
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

    /// The inputs at `now_ms`: the writes for the core. `gun_connected`: a
    /// light gun is on the dock, and the core's (the gun code holds the
    /// Controller register at 1); the mouse then stands still, its buttons
    /// released, as it does while the setting is not `CONTROLLER_MOUSE`.
    ///
    /// A movement packet is written when there are whole units to move, at
    /// most every `MIN_MOTION_INTERVAL_MS`; a button change at once.
    pub fn update(&mut self, now_ms: u64, gun_connected: bool, input: &MouseInput) -> Writes {
        let mut writes = Writes::default();
        let Some(target) = self.target else {
            return writes;
        };

        if !self.enabled() || gun_connected {
            self.cursor.reset();
            self.last_motion_write = None;
            if self.sent_buttons.is_some_and(|buttons| buttons != 0) {
                writes.push(target.buttons, 0);
                self.sent_buttons = Some(0);
            }
            return writes;
        }

        self.cursor.update(now_ms, input.stick_x, input.stick_y);
        let due = self
            .last_motion_write
            .is_none_or(|at| now_ms.saturating_sub(at) >= MIN_MOTION_INTERVAL_MS);
        if due {
            let (dx, dy) = self.cursor.take();
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
