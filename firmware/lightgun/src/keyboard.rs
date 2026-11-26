//! A core's keyboard from a USB keyboard on the dock.
//!
//! The dock reports a keyboard as an ordinary gamepad (`GamepadData`) with
//! bit 29 of its buttons set (`button::KEYBOARD`; a mouse has bit 30, a
//! light gun bit 31): bits 0-7 of the buttons word are the modifier keys
//! (the USB HID boot report's first byte: left Ctrl, Shift, Alt, GUI, then
//! the right ones), and the three left axes carry the up to six keys held,
//! as their HID usages, two to an axis (lx: keys 0 and 1, ly: 2 and 3, lz:
//! 4 and 5; low byte first; 0 where no key is). Every report is the whole
//! state of that keyboard (`KeyboardReport`).
//!
//! A core that takes a keyboard (`target_for_core`) gets key events over
//! the host memory interface: one write to `Target::event` per key going
//! down or up, `pack_event`: bits 7:0 the key's HID usage (0xE0..0xE7 the
//! modifiers), bit 8 set when it went down. The core translates the usages
//! to its machine's key codes.
//!
//! `Keyboard` keeps the keys the core holds (`KeySet`) and turns the state
//! of the dock's keyboards (all of them together: a key is down while any
//! holds it) into events, releases first. While the core does not take the
//! keyboard (no focus: the menu is open) nothing is written; when it takes
//! it again the keys released meanwhile are released in the core, so none
//! stays stuck.
//!
//! A keyboard is never a gamepad: the firmware keeps it out of the merged
//! pad and out of the cores' controller slots.

/// The dock's keyboard marker (a bit of `GamepadData`'s buttons word).
pub mod button {
    /// Set on every report of a keyboard.
    pub const KEYBOARD: u32 = 1 << 29;
}

/// The first modifier's usage (left Ctrl); the eight follow the bits of the
/// boot report's modifier byte.
pub const USAGE_MODIFIER_FIRST: u8 = 0xE0;

/// One report of a keyboard from the dock, as received.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct KeyboardReport {
    pub modifiers: u8,
    /// HID usages of the keys held (0: none; 1..3 are the boot protocol's
    /// error codes, not keys).
    pub keys: [u8; 6],
}

impl KeyboardReport {
    /// Whether a dock gamepad's buttons word is a keyboard's.
    pub const fn is_keyboard(buttons: u32) -> bool {
        (buttons & button::KEYBOARD) != 0
    }

    /// From the dock's buttons word and left axes.
    pub const fn from_dock(buttons: u32, lx: i16, ly: i16, lz: i16) -> Self {
        let (lx, ly, lz) = (lx as u16, ly as u16, lz as u16);
        KeyboardReport {
            modifiers: buttons as u8,
            keys: [
                lx as u8,
                (lx >> 8) as u8,
                ly as u8,
                (ly >> 8) as u8,
                lz as u8,
                (lz >> 8) as u8,
            ],
        }
    }

    /// The keys of this report into a set.
    pub fn add_to(&self, set: &mut KeySet) {
        for bit in 0..8 {
            if self.modifiers & (1 << bit) != 0 {
                set.insert(USAGE_MODIFIER_FIRST + bit);
            }
        }
        for &key in &self.keys {
            if key > 3 {
                set.insert(key);
            }
        }
    }
}

/// A set of keys, by HID usage.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct KeySet([u32; 8]);

impl KeySet {
    pub const fn new() -> Self {
        KeySet([0; 8])
    }

    pub fn insert(&mut self, usage: u8) {
        self.0[(usage >> 5) as usize] |= 1 << (usage & 31);
    }

    pub fn remove(&mut self, usage: u8) {
        self.0[(usage >> 5) as usize] &= !(1 << (usage & 31));
    }

    pub const fn contains(&self, usage: u8) -> bool {
        self.0[(usage >> 5) as usize] & (1 << (usage & 31)) != 0
    }

    pub fn is_empty(&self) -> bool {
        self.0 == [0; 8]
    }
}

/// Where a core takes a keyboard: an address on the host memory interface.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Target {
    /// A key event, `pack_event`.
    pub event: u32,
}

/// The Minimig core (apolkosnik/Minimig_GameBub, `rtl/gamebub/gb_host.sv`)
/// and its build variants (`crate::is_minimig`).
pub const TARGET_MINIMIG: Target = Target { event: 0x0000_1038 };

/// The keyboard register of a core, by its ID (core.json), if it takes one.
pub fn target_for_core(core_id: &str) -> Option<Target> {
    match core_id {
        id if crate::is_minimig(id) => Some(TARGET_MINIMIG),
        _ => None,
    }
}

/// The event register: bits 7:0 the HID usage, bit 8 set when pressed.
pub const fn pack_event(usage: u8, pressed: bool) -> u32 {
    ((pressed as u32) << 8) | usage as u32
}

/// The most events one update gives; what is left comes with the next.
pub const MAX_EVENTS: usize = 16;

/// Key events for the core, in order: (address, value).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Events {
    buf: [(u32, u32); MAX_EVENTS],
    len: usize,
}

impl Default for Events {
    fn default() -> Self {
        Events {
            buf: [(0, 0); MAX_EVENTS],
            len: 0,
        }
    }
}

impl Events {
    fn push(&mut self, address: u32, value: u32) {
        self.buf[self.len] = (address, value);
        self.len += 1;
    }

    fn is_full(&self) -> bool {
        self.len == MAX_EVENTS
    }

    pub fn as_slice(&self) -> &[(u32, u32)] {
        &self.buf[..self.len]
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// The keyboard of the running core: what to write.
#[derive(Clone, Debug, Default)]
pub struct Keyboard {
    target: Option<Target>,
    /// The core `held` is about (the last one that took the keyboard).
    held_for: Option<Target>,
    /// The keys the core holds: every press written and not yet released.
    held: KeySet,
}

impl Keyboard {
    pub const fn new() -> Self {
        Keyboard {
            target: None,
            held_for: None,
            held: KeySet::new(),
        }
    }

    /// The core taking the keyboard now (None: no core, or one without a
    /// keyboard, or not running with focus). The keys a core holds are
    /// remembered while it does not take the keyboard, so the next `update`
    /// releases those let go meanwhile; another core starts with none.
    pub fn set_target(&mut self, target: Option<Target>) {
        if target.is_some() && target != self.held_for {
            self.held = KeySet::new();
            self.held_for = target;
        }
        self.target = target;
    }

    pub fn enabled(&self) -> bool {
        self.target.is_some()
    }

    /// The keys down now on the dock's keyboards: the events that bring the
    /// core's keys to them, releases first, then presses, each in the order
    /// of the usages. At most `MAX_EVENTS`; `pending` says whether more are
    /// left for another update.
    pub fn update(&mut self, down: &KeySet) -> Events {
        let mut events = Events::default();
        let Some(target) = self.target else {
            return events;
        };
        for pressed in [false, true] {
            for usage in 0..=255u8 {
                if events.is_full() {
                    return events;
                }
                if self.held.contains(usage) != down.contains(usage)
                    && down.contains(usage) == pressed
                {
                    events.push(target.event, pack_event(usage, pressed));
                    if pressed {
                        self.held.insert(usage);
                    } else {
                        self.held.remove(usage);
                    }
                }
            }
        }
        events
    }

    /// Whether the core's keys differ from `down` (an update has more to do).
    pub fn pending(&self, down: &KeySet) -> bool {
        self.target.is_some() && self.held != *down
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(reports: &[KeyboardReport]) -> KeySet {
        let mut s = KeySet::new();
        for r in reports {
            r.add_to(&mut s);
        }
        s
    }

    fn keys(modifiers: u8, keys: &[u8]) -> KeyboardReport {
        let mut k = [0u8; 6];
        k[..keys.len()].copy_from_slice(keys);
        KeyboardReport { modifiers, keys: k }
    }

    fn events(e: &Events) -> Vec<(u32, u32)> {
        e.as_slice().to_vec()
    }

    fn minimig() -> Keyboard {
        let mut k = Keyboard::new();
        k.set_target(target_for_core("apolkosnik.Minimig"));
        k
    }

    #[test]
    fn targets() {
        assert_eq!(
            target_for_core("apolkosnik.Minimig"),
            Some(Target { event: 0x1038 })
        );
        assert_eq!(target_for_core("apolkosnik.3DO"), None);
        assert_eq!(target_for_core(""), None);
    }

    #[test]
    fn a_variant_of_the_minimig_core_takes_the_keyboard() {
        // the 68040 build (variants/ap68040 there): the same register
        assert_eq!(
            target_for_core("apolkosnik.Minimig040"),
            Some(TARGET_MINIMIG)
        );
        assert_eq!(target_for_core("apolkosnik.Minimig.040"), None);
        assert_eq!(target_for_core("apolkosnik.minimig040"), None);
        let mut k = Keyboard::new();
        k.set_target(target_for_core("apolkosnik.Minimig040"));
        assert!(k.enabled());
        assert_eq!(
            events(&k.update(&set(&[keys(0x02, &[0x04])]))),
            [(0x1038, 0x104), (0x1038, 0x1E1)]
        );
        assert_eq!(
            events(&k.update(&set(&[keys(0, &[])]))),
            [(0x1038, 0x004), (0x1038, 0x0E1)]
        );
    }

    #[test]
    fn report_from_dock_words() {
        assert!(KeyboardReport::is_keyboard(button::KEYBOARD | 2));
        assert!(!KeyboardReport::is_keyboard(1 << 30));
        let r =
            KeyboardReport::from_dock(button::KEYBOARD | 0x22, 0x0504, 0x0706, 0x0908u16 as i16);
        assert_eq!(r, keys(0x22, &[4, 5, 6, 7, 8, 9]));
        // usages above 0x7F in the high byte of an axis
        let r = KeyboardReport::from_dock(button::KEYBOARD, 0xE400u16 as i16, 0, 0);
        assert_eq!(r.keys[1], 0xE4);
    }

    #[test]
    fn set_holds_modifiers_and_keys_not_error_codes() {
        let s = set(&[keys(0x81, &[0x04, 0x01, 0x00, 0x52])]);
        assert!(s.contains(0xE0) && s.contains(0xE7) && s.contains(0x04) && s.contains(0x52));
        assert!(!s.contains(0x01) && !s.contains(0x00) && !s.contains(0xE1));
        // the rollover error report holds no key
        assert!(set(&[keys(0, &[1, 1, 1, 1, 1, 1])]).is_empty());
    }

    #[test]
    fn press_and_release() {
        let mut k = minimig();
        assert_eq!(
            events(&k.update(&set(&[keys(0, &[0x04])]))),
            [(0x1038, 0x104)]
        );
        assert_eq!(events(&k.update(&set(&[keys(0, &[0x04])]))), []);
        assert_eq!(events(&k.update(&set(&[keys(0, &[])]))), [(0x1038, 0x004)]);
    }

    #[test]
    fn releases_come_before_presses() {
        let mut k = minimig();
        k.update(&set(&[keys(0x02, &[0x10])]));
        // Shift and M go, A comes
        assert_eq!(
            events(&k.update(&set(&[keys(0, &[0x04])]))),
            [(0x1038, 0x010), (0x1038, 0x0E1), (0x1038, 0x104)]
        );
    }

    #[test]
    fn two_keyboards_are_one() {
        let mut k = minimig();
        let a = keys(0, &[0x04]);
        let b = keys(0, &[0x04, 0x05]);
        assert_eq!(
            events(&k.update(&set(&[a, b]))),
            [(0x1038, 0x104), (0x1038, 0x105)]
        );
        // one lets A go, the other still holds it
        assert_eq!(events(&k.update(&set(&[keys(0, &[]), b]))), []);
        assert_eq!(
            events(&k.update(&set(&[keys(0, &[])]))),
            [(0x1038, 0x004), (0x1038, 0x005)]
        );
    }

    #[test]
    fn nothing_without_a_core() {
        let mut k = Keyboard::new();
        assert!(!k.enabled());
        assert_eq!(events(&k.update(&set(&[keys(0, &[0x04])]))), []);
        k.set_target(target_for_core("apolkosnik.3DO"));
        assert_eq!(events(&k.update(&set(&[keys(0, &[0x04])]))), []);
    }

    #[test]
    fn keys_let_go_without_focus_are_released_after() {
        let mut k = minimig();
        k.update(&set(&[keys(0x01, &[0x04])]));
        // the menu opens: nothing is written
        k.set_target(None);
        assert_eq!(events(&k.update(&set(&[keys(0, &[])]))), []);
        // back in the core: Ctrl and A were let go, B is down now
        k.set_target(Some(TARGET_MINIMIG));
        assert_eq!(
            events(&k.update(&set(&[keys(0, &[0x05])]))),
            [(0x1038, 0x004), (0x1038, 0x0E0), (0x1038, 0x105)]
        );
    }

    #[test]
    fn another_core_starts_with_no_keys() {
        let mut k = minimig();
        k.update(&set(&[keys(0, &[0x04])]));
        k.set_target(Some(Target { event: 0x2000 }));
        assert_eq!(events(&k.update(&set(&[keys(0, &[])]))), []);
        assert_eq!(
            events(&k.update(&set(&[keys(0, &[0x04])]))),
            [(0x2000, 0x104)]
        );
    }

    #[test]
    fn more_events_than_fit_come_with_the_next_update() {
        let mut k = minimig();
        let mut s = KeySet::new();
        for usage in 4..4 + 20 {
            s.insert(usage);
        }
        let first = k.update(&s);
        assert_eq!(first.as_slice().len(), MAX_EVENTS);
        assert!(k.pending(&s));
        let second = k.update(&s);
        assert_eq!(second.as_slice().len(), 20 - MAX_EVENTS);
        assert_eq!(
            second.as_slice()[0],
            (0x1038, 0x100 | (4 + MAX_EVENTS as u32))
        );
        assert!(!k.pending(&s));
    }
}
