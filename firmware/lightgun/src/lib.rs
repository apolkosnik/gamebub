//! Light gun support: a GunCon 3 on the dock, aimed into a core.
//!
//! The dock reports a light gun as an ordinary gamepad (`GamepadData`) with
//! bit 31 of its buttons set: the left stick is the aim (X, Y: signed, 0 in
//! the centre, positive right / down), the left Z axis the distance to the
//! emitters, the right stick the gun's stick A. A core that takes a light gun
//! (see `target_for_core`) gets, over the host memory interface:
//!
//!  * `Target::enable`: 1 while a gun is connected, 0 otherwise;
//!  * `Target::aim`: `{y[31:16], x[15:0]}`, each 0..65535 across the picture
//!    (0 the left / top edge), see `pack_aim`;
//!  * `Target::buttons`: bit 0 trigger, bit 1 option, bit 2 off screen, see
//!    `pack_buttons`.
//!
//! The aim is the gun's raw X / Y through a linear calibration per axis
//! (`Axis`), by default `raw + 32768`. The user calibrates without a screen
//! of its own (`Calibrator`): hold C1 + C2 for two seconds, then aim at the
//! picture's top-left corner and pull the trigger, then at its bottom-right
//! corner and pull the trigger.
//!
//! Everything here is pure logic, without the firmware's dependencies, so it
//! runs on the host: `cargo test` in this directory.

#![cfg_attr(not(test), no_std)]

/// The dock's light gun buttons (bits of `GamepadData`'s buttons word).
pub mod button {
    /// Set on every report of a light gun.
    pub const LIGHT_GUN: u32 = 1 << 31;
    pub const TRIGGER: u32 = 1 << 0;
    pub const A1: u32 = 1 << 1;
    pub const A2: u32 = 1 << 2;
    pub const B1: u32 = 1 << 3;
    pub const C2: u32 = 1 << 9;
    pub const C1: u32 = 1 << 10;
    pub const B2: u32 = 1 << 12;
    pub const STICK_A: u32 = 1 << 16;
    pub const STICK_B: u32 = 1 << 17;
    /// The gun does not see the screen (no emitter).
    pub const OFF_SCREEN: u32 = 1 << 18;
    /// The gun sees only one of the two emitters (aim less accurate).
    pub const ONE_EMITTER: u32 = 1 << 19;
}

/// One report of a light gun from the dock, as received.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct GunReport {
    /// Aim X: 0 centre, positive right (the dock's lx, all 16 bits).
    pub x: i16,
    /// Aim Y: 0 centre, positive down (the dock's ly, all 16 bits).
    pub y: i16,
    /// Distance to the emitters (the dock's lz, unsigned).
    pub z: u16,
    /// The dock's buttons word, `button::*`.
    pub buttons: u32,
}

impl GunReport {
    /// Whether a dock gamepad's buttons word is a light gun's.
    pub const fn is_light_gun(buttons: u32) -> bool {
        (buttons & button::LIGHT_GUN) != 0
    }

    pub const fn pressed(&self, mask: u32) -> bool {
        (self.buttons & mask) == mask
    }

    pub const fn trigger(&self) -> bool {
        self.pressed(button::TRIGGER)
    }

    /// The 3DO gun's option button: the GunCon 3's A1.
    pub const fn option(&self) -> bool {
        self.pressed(button::A1)
    }

    pub const fn off_screen(&self) -> bool {
        self.pressed(button::OFF_SCREEN)
    }

    /// C1 and C2 together: held for `CALIBRATION_HOLD_MS` to calibrate.
    pub const fn calibration_combo(&self) -> bool {
        self.pressed(button::C1 | button::C2)
    }
}

/// Where a core takes a light gun: addresses on the host memory interface.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Target {
    /// 1: a light gun on the control port (with the pad), 0: pad only.
    pub enable: u32,
    /// The aim, `pack_aim`.
    pub aim: u32,
    /// The buttons, `pack_buttons`.
    pub buttons: u32,
}

/// The 3DO core (apolkosnik/3DO_GameBub, `rtl/gamebub/gb_host.sv`).
pub const CORE_3DO: &str = "apolkosnik.3DO";
pub const TARGET_3DO: Target = Target {
    enable: 0x0000_1014,
    aim: 0x0000_1018,
    buttons: 0x0000_101C,
};

/// The light gun registers of a core, by its ID (core.json), if it takes one.
pub fn target_for_core(core_id: &str) -> Option<Target> {
    match core_id {
        CORE_3DO => Some(TARGET_3DO),
        _ => None,
    }
}

/// The aim register: `{y[31:16], x[15:0]}`.
pub const fn pack_aim(x: u16, y: u16) -> u32 {
    ((y as u32) << 16) | x as u32
}

pub const BUTTON_TRIGGER: u32 = 1 << 0;
pub const BUTTON_OPTION: u32 = 1 << 1;
pub const BUTTON_OFF_SCREEN: u32 = 1 << 2;

/// The buttons register: bit 0 trigger, bit 1 option, bit 2 off screen.
pub const fn pack_buttons(trigger: bool, option: bool, off_screen: bool) -> u32 {
    (trigger as u32) * BUTTON_TRIGGER
        | (option as u32) * BUTTON_OPTION
        | (off_screen as u32) * BUTTON_OFF_SCREEN
}

/// The buttons register with nothing pressed, aimed off screen (also the
/// core's reset value): what the core gets while the trigger is held back.
pub const BUTTONS_RELEASED: u32 = pack_buttons(false, false, true);

/// One axis of the calibration: `out = (raw * gain >> 16) + offset`, clamped
/// to 0..65535. `gain` is 16.16 fixed point (65536 = 1.0).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Axis {
    pub gain: i32,
    pub offset: i32,
}

/// A calibration point pair closer than this (raw units, per axis) is
/// rejected: the user did not aim at two corners.
pub const MIN_CALIBRATION_SPAN: i32 = 1024;

impl Axis {
    /// The default: `raw + 32768` (the full signed range across the picture).
    pub const DEFAULT: Axis = Axis {
        gain: 1 << 16,
        offset: 32768,
    };

    pub fn map(&self, raw: i16) -> u16 {
        let value = ((raw as i64 * self.gain as i64) >> 16) + self.offset as i64;
        value.clamp(0, 65535) as u16
    }

    /// The axis taking `at_0` to 0 and `at_65535` to 65535 (within one or
    /// two units), or None if they are less than `MIN_CALIBRATION_SPAN`
    /// apart. A gun reading the other way round gets a negative gain.
    pub fn from_points(at_0: i16, at_65535: i16) -> Option<Axis> {
        let span = at_65535 as i64 - at_0 as i64;
        if span.abs() < MIN_CALIBRATION_SPAN as i64 {
            return None;
        }
        // 65535 * 65536 / span, rounded to nearest (away from zero on a tie).
        let numerator = 65535i64 << 16;
        let magnitude = (2 * numerator + span.abs()) / (2 * span.abs());
        let gain = if span > 0 { magnitude } else { -magnitude };
        let offset = -((at_0 as i64 * gain) >> 16);
        Some(Axis {
            gain: i32::try_from(gain).ok()?,
            offset: i32::try_from(offset).ok()?,
        })
    }

    pub fn is_valid(&self) -> bool {
        self.gain != 0
    }
}

/// The calibration of both axes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Calibration {
    pub x: Axis,
    pub y: Axis,
}

impl Default for Calibration {
    fn default() -> Self {
        Calibration {
            x: Axis::DEFAULT,
            y: Axis::DEFAULT,
        }
    }
}

impl Calibration {
    /// The aim, 0..65535 per axis.
    pub fn map(&self, report: &GunReport) -> (u16, u16) {
        (self.x.map(report.x), self.y.map(report.y))
    }

    /// From the aim at the picture's top-left and bottom-right corners.
    pub fn from_corners(top_left: (i16, i16), bottom_right: (i16, i16)) -> Option<Calibration> {
        Some(Calibration {
            x: Axis::from_points(top_left.0, bottom_right.0)?,
            y: Axis::from_points(top_left.1, bottom_right.1)?,
        })
    }

    /// As stored: x gain, x offset, y gain, y offset.
    pub fn to_words(&self) -> [i32; 4] {
        [self.x.gain, self.x.offset, self.y.gain, self.y.offset]
    }

    /// From `to_words`; None if it is not a usable calibration.
    pub fn from_words(words: [i32; 4]) -> Option<Calibration> {
        let calibration = Calibration {
            x: Axis {
                gain: words[0],
                offset: words[1],
            },
            y: Axis {
                gain: words[2],
                offset: words[3],
            },
        };
        (calibration.x.is_valid() && calibration.y.is_valid()).then_some(calibration)
    }
}

/// How long C1 + C2 are held to start calibrating.
pub const CALIBRATION_HOLD_MS: u64 = 2000;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CalibrationState {
    Idle,
    /// Waiting for the trigger aimed at the top-left corner.
    AwaitTopLeft,
    /// Waiting for the trigger aimed at the bottom-right corner.
    AwaitBottomRight {
        top_left: (i16, i16),
    },
}

/// What the user is told as calibration goes along.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CalibrationEvent {
    /// C1 + C2 held: aim at the top-left corner and pull the trigger.
    Started,
    /// Top-left taken: aim at the bottom-right corner and pull the trigger.
    TopLeftTaken,
    /// The trigger was pulled off screen: the point was not taken.
    OffScreen,
    /// Done: the new calibration (to store).
    Finished(Calibration),
    /// The corners were too close together; the old calibration stays.
    Failed,
    /// C1 + C2 pressed again: stopped, the old calibration stays.
    Cancelled,
}

/// The two-point calibration, fed every report of the gun.
#[derive(Clone, Debug)]
pub struct Calibrator {
    state: CalibrationState,
    /// Since when C1 + C2 have been held (while idle).
    combo_since: Option<u64>,
    /// C1 + C2 already acted on: they do nothing until released.
    combo_used: bool,
    trigger_was: bool,
}

impl Default for Calibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl Calibrator {
    pub const fn new() -> Self {
        Calibrator {
            state: CalibrationState::Idle,
            combo_since: None,
            combo_used: false,
            trigger_was: false,
        }
    }

    pub fn state(&self) -> CalibrationState {
        self.state
    }

    /// Whether calibrating (the trigger is the calibration's, not the game's).
    pub fn active(&self) -> bool {
        self.state != CalibrationState::Idle
    }

    /// Forget everything (gun gone).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// A report of the gun at `now_ms` (any monotonic millisecond clock).
    pub fn update(&mut self, now_ms: u64, report: &GunReport) -> Option<CalibrationEvent> {
        let combo = report.calibration_combo();
        let trigger = report.trigger();
        let trigger_pulled = trigger && !self.trigger_was;
        self.trigger_was = trigger;
        if !combo {
            self.combo_since = None;
            self.combo_used = false;
        }

        match self.state {
            CalibrationState::Idle => {
                if combo && !self.combo_used {
                    let since = *self.combo_since.get_or_insert(now_ms);
                    if now_ms.saturating_sub(since) >= CALIBRATION_HOLD_MS {
                        self.combo_used = true;
                        self.combo_since = None;
                        self.state = CalibrationState::AwaitTopLeft;
                        return Some(CalibrationEvent::Started);
                    }
                }
                None
            }
            CalibrationState::AwaitTopLeft | CalibrationState::AwaitBottomRight { .. } => {
                if combo && !self.combo_used {
                    self.combo_used = true;
                    self.state = CalibrationState::Idle;
                    return Some(CalibrationEvent::Cancelled);
                }
                if !trigger_pulled {
                    return None;
                }
                if report.off_screen() {
                    return Some(CalibrationEvent::OffScreen);
                }
                let point = (report.x, report.y);
                match self.state {
                    CalibrationState::AwaitTopLeft => {
                        self.state = CalibrationState::AwaitBottomRight { top_left: point };
                        Some(CalibrationEvent::TopLeftTaken)
                    }
                    CalibrationState::AwaitBottomRight { top_left } => {
                        self.state = CalibrationState::Idle;
                        Some(match Calibration::from_corners(top_left, point) {
                            Some(calibration) => CalibrationEvent::Finished(calibration),
                            None => CalibrationEvent::Failed,
                        })
                    }
                    CalibrationState::Idle => unreachable!(),
                }
            }
        }
    }
}

/// Least time between two writes of a moving aim (about 120 Hz).
pub const MIN_AIM_INTERVAL_MS: u64 = 8;

/// Register writes for the core, in order: (address, value).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Writes {
    buf: [(u32, u32); 3],
    len: usize,
}

impl Writes {
    fn push(&mut self, address: u32, value: u32) {
        self.buf[self.len] = (address, value);
        self.len += 1;
    }

    pub fn as_slice(&self) -> &[(u32, u32)] {
        &self.buf[..self.len]
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// The light gun of the running core: what to write, and when.
#[derive(Clone, Debug)]
pub struct Controller {
    target: Option<Target>,
    calibration: Calibration,
    calibrator: Calibrator,
    /// Held back from the core until the trigger is released (calibrating,
    /// or just finished calibrating with the trigger still pulled).
    hold_back: bool,
    sent_enable: Option<bool>,
    sent_aim: Option<u32>,
    sent_buttons: Option<u32>,
    last_aim_write: Option<u64>,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    pub const fn new() -> Self {
        Controller {
            target: None,
            calibration: Calibration {
                x: Axis::DEFAULT,
                y: Axis::DEFAULT,
            },
            calibrator: Calibrator::new(),
            hold_back: false,
            sent_enable: None,
            sent_aim: None,
            sent_buttons: None,
            last_aim_write: None,
        }
    }

    /// The core taking the gun now (None: no core, or one without a gun, or
    /// not running), with its calibration. Everything is written again on
    /// the next `update`.
    pub fn set_target(&mut self, target: Option<Target>, calibration: Calibration) {
        *self = Controller {
            target,
            calibration,
            ..Self::new()
        };
    }

    /// Whether a core takes the gun now (the gun is then the core's, not a
    /// gamepad).
    pub fn active(&self) -> bool {
        self.target.is_some()
    }

    pub fn target(&self) -> Option<Target> {
        self.target
    }

    pub fn calibration(&self) -> Calibration {
        self.calibration
    }

    pub fn calibrating(&self) -> bool {
        self.calibrator.active()
    }

    /// The gun's state at `now_ms` (its latest report; None: no gun): the
    /// writes for the core, and what calibration has to say.
    ///
    /// Connecting writes aim and buttons, then enable = 1; disconnecting
    /// releases the buttons, then writes enable = 0. A button change is
    /// written at once; a moving aim at most every `MIN_AIM_INTERVAL_MS`.
    /// While calibrating the core sees the trigger and option released and
    /// the gun off screen.
    pub fn update(
        &mut self,
        now_ms: u64,
        gun: Option<&GunReport>,
    ) -> (Writes, Option<CalibrationEvent>) {
        let mut writes = Writes::default();
        let Some(target) = self.target else {
            return (writes, None);
        };

        let Some(report) = gun else {
            self.calibrator.reset();
            self.hold_back = false;
            if self.sent_buttons != Some(BUTTONS_RELEASED) {
                writes.push(target.buttons, BUTTONS_RELEASED);
                self.sent_buttons = Some(BUTTONS_RELEASED);
            }
            if self.sent_enable != Some(false) {
                writes.push(target.enable, 0);
                self.sent_enable = Some(false);
            }
            return (writes, None);
        };

        let event = self.calibrator.update(now_ms, report);
        if let Some(CalibrationEvent::Finished(calibration)) = event {
            self.calibration = calibration;
        }
        if self.calibrator.active() || event.is_some() {
            self.hold_back = true;
        } else if !report.trigger() && !report.option() {
            self.hold_back = false;
        }

        let (x, y) = self.calibration.map(report);
        let aim = pack_aim(x, y);
        let buttons = if self.hold_back {
            BUTTONS_RELEASED
        } else {
            pack_buttons(report.trigger(), report.option(), report.off_screen())
        };

        let aim_due = self
            .last_aim_write
            .is_none_or(|at| now_ms.saturating_sub(at) >= MIN_AIM_INTERVAL_MS);
        if self.sent_buttons != Some(buttons) || (self.sent_aim != Some(aim) && aim_due) {
            writes.push(target.aim, aim);
            writes.push(target.buttons, buttons);
            self.sent_aim = Some(aim);
            self.sent_buttons = Some(buttons);
            self.last_aim_write = Some(now_ms);
        }
        if self.sent_enable != Some(true) {
            writes.push(target.enable, 1);
            self.sent_enable = Some(true);
        }
        (writes, event)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gun(x: i16, y: i16, buttons: u32) -> GunReport {
        GunReport {
            x,
            y,
            z: 1000,
            buttons: button::LIGHT_GUN | buttons,
        }
    }

    #[test]
    fn target_only_for_the_3do_core() {
        assert_eq!(
            target_for_core("apolkosnik.3DO"),
            Some(Target {
                enable: 0x1014,
                aim: 0x1018,
                buttons: 0x101C
            })
        );
        assert_eq!(target_for_core("Game-Bub.GBA"), None);
        assert_eq!(target_for_core("apolkosnik.3do"), None);
        assert_eq!(target_for_core(""), None);
    }

    #[test]
    fn light_gun_marker_is_bit_31() {
        assert!(GunReport::is_light_gun(0x8000_0000));
        assert!(!GunReport::is_light_gun(0x7FFF_FFFF));
    }

    #[test]
    fn dock_button_bits() {
        let r = gun(0, 0, 1 << 0);
        assert!(r.trigger() && !r.option() && !r.off_screen());
        let r = gun(0, 0, 1 << 1);
        assert!(r.option() && !r.trigger());
        let r = gun(0, 0, 1 << 2);
        assert!(!r.option(), "A2 is not the option button");
        let r = gun(0, 0, 1 << 18);
        assert!(r.off_screen());
        let r = gun(0, 0, 1 << 19);
        assert!(!r.off_screen(), "one emitter seen is still on screen");
        assert!(gun(0, 0, (1 << 9) | (1 << 10)).calibration_combo());
        assert!(!gun(0, 0, 1 << 9).calibration_combo());
        assert!(!gun(0, 0, 1 << 10).calibration_combo());
    }

    #[test]
    fn aim_packing() {
        assert_eq!(pack_aim(0, 0), 0);
        assert_eq!(pack_aim(0x1234, 0xABCD), 0xABCD_1234);
        assert_eq!(pack_aim(65535, 0), 0x0000_FFFF);
        assert_eq!(pack_aim(0, 65535), 0xFFFF_0000);
        assert_eq!(pack_aim(32768, 32768), 0x8000_8000);
    }

    #[test]
    fn button_packing() {
        assert_eq!(pack_buttons(false, false, false), 0b000);
        assert_eq!(pack_buttons(true, false, false), 0b001);
        assert_eq!(pack_buttons(false, true, false), 0b010);
        assert_eq!(pack_buttons(false, false, true), 0b100);
        assert_eq!(pack_buttons(true, true, true), 0b111);
        assert_eq!(BUTTONS_RELEASED, 0b100);
    }

    #[test]
    fn default_mapping_is_offset_by_half() {
        let c = Calibration::default();
        assert_eq!(c.map(&gun(0, 0, 0)), (32768, 32768));
        assert_eq!(c.map(&gun(-32768, -32768, 0)), (0, 0));
        assert_eq!(c.map(&gun(32767, 32767, 0)), (65535, 65535));
        assert_eq!(c.map(&gun(-1000, 2000, 0)), (31768, 34768));
        // X and Y are not swapped.
        assert_eq!(c.map(&gun(-32768, 32767, 0)), (0, 65535));
    }

    #[test]
    fn axis_clamps() {
        let a = Axis {
            gain: 2 << 16,
            offset: 32768,
        };
        assert_eq!(a.map(20000), 65535);
        assert_eq!(a.map(-20000), 0);
        assert_eq!(a.map(1000), 34768);
        let a = Axis {
            gain: 1 << 16,
            offset: -100,
        };
        assert_eq!(a.map(50), 0);
        assert_eq!(a.map(150), 50);
    }

    fn near(value: u16, expected: u16) -> bool {
        (value as i32 - expected as i32).abs() <= 2
    }

    #[test]
    fn two_point_calibration_maps_the_corners() {
        for &(lo, hi) in &[
            (-20000i16, 18000i16),
            (-32768, 32767),
            (-1500, -400),
            (100, 30000),
            (-25000, -1000),
        ] {
            let a = Axis::from_points(lo, hi).unwrap();
            assert_eq!(a.map(lo), 0, "{lo}..{hi}");
            assert!(near(a.map(hi), 65535), "{lo}..{hi}: {}", a.map(hi));
            let mid = ((lo as i32 + hi as i32) / 2) as i16;
            assert!(near(a.map(mid), 32768), "{lo}..{hi}: mid {}", a.map(mid));
            // Beyond the corners: clamped.
            assert_eq!(a.map(lo.saturating_sub(500)), 0);
            assert_eq!(
                a.map(hi.saturating_add(500)),
                if hi > 32266 { a.map(hi) } else { 65535 }
            );
        }
    }

    #[test]
    fn two_point_calibration_reversed_axis() {
        let a = Axis::from_points(10000, -10000).unwrap();
        assert!(a.gain < 0);
        assert_eq!(a.map(10000), 0);
        assert!(near(a.map(-10000), 65535));
        assert!(near(a.map(0), 32768));
    }

    #[test]
    fn two_point_calibration_rejects_close_points() {
        assert_eq!(Axis::from_points(100, 100), None);
        assert_eq!(Axis::from_points(100, 100 + 1023), None);
        assert!(Axis::from_points(100, 100 + 1024).is_some());
        assert_eq!(
            Calibration::from_corners((-9000, -9000), (9000, -8500)),
            None
        );
    }

    #[test]
    fn calibration_storage_round_trip() {
        let c = Calibration::from_corners((-20000, -15000), (21000, 16000)).unwrap();
        assert_eq!(Calibration::from_words(c.to_words()), Some(c));
        assert_eq!(
            Calibration::default().to_words(),
            [65536, 32768, 65536, 32768]
        );
        assert_eq!(Calibration::from_words([0, 0, 65536, 0]), None);
    }

    const C1C2: u32 = button::C1 | button::C2;
    const TRIGGER: u32 = button::TRIGGER;

    /// Run the calibrator from idle to waiting for the top-left corner.
    fn start(cal: &mut Calibrator, t: &mut u64) {
        assert_eq!(cal.update(*t, &gun(0, 0, C1C2)), None);
        *t += 1000;
        assert_eq!(cal.update(*t, &gun(0, 0, C1C2)), None);
        *t += 1000;
        assert_eq!(
            cal.update(*t, &gun(0, 0, C1C2)),
            Some(CalibrationEvent::Started)
        );
        assert_eq!(cal.state(), CalibrationState::AwaitTopLeft);
        *t += 10;
        assert_eq!(cal.update(*t, &gun(0, 0, 0)), None);
    }

    #[test]
    fn calibration_needs_a_two_second_hold() {
        let mut cal = Calibrator::new();
        assert_eq!(cal.update(0, &gun(0, 0, C1C2)), None);
        assert_eq!(cal.update(1999, &gun(0, 0, C1C2)), None);
        assert!(!cal.active());
        // Released before two seconds: starts over.
        assert_eq!(cal.update(2000, &gun(0, 0, button::C1)), None);
        assert_eq!(cal.update(2100, &gun(0, 0, C1C2)), None);
        assert_eq!(cal.update(4099, &gun(0, 0, C1C2)), None);
        assert_eq!(
            cal.update(4100, &gun(0, 0, C1C2)),
            Some(CalibrationEvent::Started)
        );
        // Still held: nothing more.
        assert_eq!(cal.update(9000, &gun(0, 0, C1C2)), None);
        assert!(cal.active());
    }

    #[test]
    fn calibration_full_run() {
        let mut cal = Calibrator::new();
        let mut t = 0;
        start(&mut cal, &mut t);
        // Trigger held from before does not count; a new pull does.
        assert_eq!(
            cal.update(t + 10, &gun(-20000, -15000, TRIGGER)),
            Some(CalibrationEvent::TopLeftTaken)
        );
        assert_eq!(cal.update(t + 20, &gun(-20000, -15000, TRIGGER)), None);
        assert_eq!(
            cal.update(t + 30, &gun(21000, 16000, TRIGGER)),
            None,
            "still held"
        );
        assert_eq!(cal.update(t + 40, &gun(21000, 16000, 0)), None);
        let event = cal.update(t + 50, &gun(21000, 16000, TRIGGER));
        let Some(CalibrationEvent::Finished(c)) = event else {
            panic!("{event:?}");
        };
        assert!(!cal.active());
        assert_eq!(c.map(&gun(-20000, -15000, 0)), (0, 0));
        let (x, y) = c.map(&gun(21000, 16000, 0));
        assert!(near(x, 65535) && near(y, 65535), "{x} {y}");
        let (x, y) = c.map(&gun(500, 500, 0));
        assert!(near(x, 32768) && near(y, 32768), "{x} {y}");
    }

    #[test]
    fn calibration_rejects_off_screen() {
        let mut cal = Calibrator::new();
        let mut t = 0;
        start(&mut cal, &mut t);
        let off = button::OFF_SCREEN;
        assert_eq!(
            cal.update(t + 1, &gun(-20000, -15000, TRIGGER | off)),
            Some(CalibrationEvent::OffScreen)
        );
        assert_eq!(cal.state(), CalibrationState::AwaitTopLeft);
        assert_eq!(cal.update(t + 2, &gun(-20000, -15000, 0)), None);
        assert_eq!(
            cal.update(t + 3, &gun(-21000, -16000, TRIGGER)),
            Some(CalibrationEvent::TopLeftTaken)
        );
        assert_eq!(cal.update(t + 4, &gun(0, 0, 0)), None);
        assert_eq!(
            cal.update(t + 5, &gun(20000, 15000, TRIGGER | off)),
            Some(CalibrationEvent::OffScreen)
        );
        assert_eq!(
            cal.state(),
            CalibrationState::AwaitBottomRight {
                top_left: (-21000, -16000)
            }
        );
        assert_eq!(cal.update(t + 6, &gun(20000, 15000, 0)), None);
        let event = cal.update(t + 7, &gun(19000, 14000, TRIGGER));
        let Some(CalibrationEvent::Finished(c)) = event else {
            panic!("{event:?}");
        };
        assert_eq!(
            c,
            Calibration::from_corners((-21000, -16000), (19000, 14000)).unwrap()
        );
    }

    #[test]
    fn calibration_fails_on_close_corners() {
        let mut cal = Calibrator::new();
        let mut t = 0;
        start(&mut cal, &mut t);
        assert_eq!(
            cal.update(t + 1, &gun(100, 100, TRIGGER)),
            Some(CalibrationEvent::TopLeftTaken)
        );
        assert_eq!(cal.update(t + 2, &gun(100, 100, 0)), None);
        assert_eq!(
            cal.update(t + 3, &gun(300, 9000, TRIGGER)),
            Some(CalibrationEvent::Failed)
        );
        assert!(!cal.active());
    }

    #[test]
    fn calibration_cancel() {
        let mut cal = Calibrator::new();
        let mut t = 0;
        start(&mut cal, &mut t);
        assert_eq!(
            cal.update(t + 1, &gun(0, 0, C1C2)),
            Some(CalibrationEvent::Cancelled)
        );
        assert!(!cal.active());
        // Still held after cancelling: does not start again at once.
        assert_eq!(cal.update(t + 1500, &gun(0, 0, C1C2)), None);
        assert_eq!(cal.update(t + 2500, &gun(0, 0, C1C2)), None);
        assert!(!cal.active());
    }

    fn writes(w: &Writes) -> Vec<(u32, u32)> {
        w.as_slice().to_vec()
    }

    #[test]
    fn controller_inactive_writes_nothing() {
        let mut c = Controller::new();
        let (w, e) = c.update(0, Some(&gun(0, 0, TRIGGER)));
        assert!(w.is_empty() && e.is_none());
        let (w, _) = c.update(10, None);
        assert!(w.is_empty());
        c.set_target(target_for_core("Game-Bub.GB"), Calibration::default());
        assert!(!c.active());
        assert!(c.update(20, Some(&gun(0, 0, TRIGGER))).0.is_empty());
    }

    #[test]
    fn controller_connect_stream_disconnect() {
        let mut c = Controller::new();
        c.set_target(Some(TARGET_3DO), Calibration::default());
        // Activated with no gun: pad only.
        assert_eq!(writes(&c.update(0, None).0), [(0x101C, 0b100), (0x1014, 0)]);
        assert!(c.update(5, None).0.is_empty());

        // Gun connects: aim, buttons, then enable.
        let (w, _) = c.update(10, Some(&gun(-1000, 2000, 0)));
        assert_eq!(
            writes(&w),
            [
                (0x1018, pack_aim(31768, 34768)),
                (0x101C, 0b000),
                (0x1014, 1)
            ]
        );
        // Same state: nothing.
        assert!(c.update(12, Some(&gun(-1000, 2000, 0))).0.is_empty());
        // Aim moves 4 ms later: held back (120 Hz at most)...
        assert!(c.update(14, Some(&gun(-900, 2000, 0))).0.is_empty());
        // ...and written once 8 ms have passed.
        assert_eq!(
            writes(&c.update(18, Some(&gun(-900, 2000, 0))).0),
            [(0x1018, pack_aim(31868, 34768)), (0x101C, 0b000)]
        );
        // A button change is written at once, with the latest aim.
        assert_eq!(
            writes(&c.update(19, Some(&gun(-800, 2000, TRIGGER))).0),
            [(0x1018, pack_aim(31968, 34768)), (0x101C, 0b001)]
        );
        assert_eq!(
            writes(
                &c.update(
                    20,
                    Some(&gun(-800, 2000, TRIGGER | button::A1 | button::OFF_SCREEN))
                )
                .0
            ),
            [(0x1018, pack_aim(31968, 34768)), (0x101C, 0b111)]
        );
        // Other buttons are not the core's.
        assert!(c
            .update(
                40,
                Some(&gun(
                    -800,
                    2000,
                    TRIGGER | button::A1 | button::OFF_SCREEN | button::B1 | button::ONE_EMITTER
                ))
            )
            .0
            .is_empty());
        // Gone: buttons released, then pad only.
        assert_eq!(
            writes(&c.update(50, None).0),
            [(0x101C, 0b100), (0x1014, 0)]
        );
        assert!(c.update(60, None).0.is_empty());
    }

    #[test]
    fn controller_rewrites_everything_on_new_target() {
        let mut c = Controller::new();
        c.set_target(Some(TARGET_3DO), Calibration::default());
        let report = gun(0, 0, 0);
        assert_eq!(c.update(0, Some(&report)).0.as_slice().len(), 3);
        assert!(c.update(100, Some(&report)).0.is_empty());
        // Refocused (or a new core): written again.
        c.set_target(Some(TARGET_3DO), Calibration::default());
        assert_eq!(
            writes(&c.update(101, Some(&report)).0),
            [(0x1018, 0x8000_8000), (0x101C, 0), (0x1014, 1)]
        );
    }

    #[test]
    fn controller_uses_calibration() {
        let cal = Calibration::from_corners((-20000, -10000), (20000, 10000)).unwrap();
        let mut c = Controller::new();
        c.set_target(Some(TARGET_3DO), cal);
        let (w, _) = c.update(0, Some(&gun(-20000, -10000, 0)));
        assert_eq!(w.as_slice()[0], (0x1018, 0));
        let (w, _) = c.update(100, Some(&gun(-30000, 30000, 0)));
        assert_eq!(
            w.as_slice()[0],
            (0x1018, 0xFFFF_0000),
            "clamped to the edges"
        );
    }

    #[test]
    fn controller_calibration_holds_back_the_trigger() {
        let mut c = Controller::new();
        c.set_target(Some(TARGET_3DO), Calibration::default());
        let mut t = 0;
        c.update(t, Some(&gun(0, 0, 0)));
        for _ in 0..3 {
            t += 1000;
            c.update(t, Some(&gun(0, 0, C1C2)));
        }
        assert!(c.calibrating());
        t += 100;
        c.update(t, Some(&gun(0, 0, 0)));
        // Trigger pulled while calibrating: the core sees nothing pressed.
        t += 100;
        let (w, e) = c.update(t, Some(&gun(-20000, -15000, TRIGGER | button::A1)));
        assert_eq!(e, Some(CalibrationEvent::TopLeftTaken));
        assert_eq!(w.as_slice().last(), Some(&(0x101C, 0b100)));
        t += 100;
        c.update(t, Some(&gun(0, 0, 0)));
        t += 100;
        let (w, e) = c.update(t, Some(&gun(21000, 16000, TRIGGER)));
        let Some(CalibrationEvent::Finished(cal)) = e else {
            panic!("{e:?}");
        };
        assert_eq!(c.calibration(), cal);
        // The aim already uses the new calibration; the trigger that took
        // the corner is not the game's...
        let (x, y) = cal.map(&gun(21000, 16000, 0));
        assert_eq!(writes(&w), [(0x1018, pack_aim(x, y)), (0x101C, 0b100)]);
        // ...nor is it while it stays pulled...
        t += 100;
        assert!(c.update(t, Some(&gun(21000, 16000, TRIGGER))).0.is_empty());
        // ...until released; then the next pull is.
        t += 100;
        let (w, _) = c.update(t, Some(&gun(21000, 16000, 0)));
        assert_eq!(writes(&w), [(0x1018, pack_aim(x, y)), (0x101C, 0b000)]);
        t += 100;
        let (w, _) = c.update(t, Some(&gun(21000, 16000, TRIGGER)));
        assert_eq!(writes(&w), [(0x1018, pack_aim(x, y)), (0x101C, 0b001)]);
    }
}
