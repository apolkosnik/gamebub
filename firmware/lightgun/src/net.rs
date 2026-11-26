//! Ethernet frames between the dock and a core's network board (the Minimig
//! core's A2065): a USB Ethernet adapter on the dock (CDC Ethernet) is the
//! Amiga's link to a network.
//!
//! The dock and the handheld exchange vendor bulk packets of at most 64
//! bytes (a 4 byte header and up to 60 bytes of payload; the gamepad
//! reports' transport). A frame of up to 1518 bytes travels as the pieces
//! `pieces` cuts it into, in either direction:
//!
//! ```text
//!   byte 0   REQUEST_NET
//!   byte 1   bit 0: the frame's first piece, bit 1: its last
//!   byte 2   the piece's number in the frame, from 0
//!   byte 3   bytes of payload (1..60)
//! ```
//!
//! `Reassembler` puts them together again in a buffer of its own: nothing
//! is allocated (while a core runs the handheld's heap has no room). A
//! piece that is not the next one drops the frame; the next frame's first
//! piece starts afresh.
//!
//! The dock tells the handheld whether it has an adapter with a
//! `REQUEST_NET_LINK` packet (payload: 1 byte, 0 or 1), when the adapter
//! comes or goes and when the handheld is docked.
//!
//! The core's side (gb_host.sv of the Minimig core) is a few registers and
//! two buffers over the host memory interface, see `STATUS` and below.

/// A piece of an Ethernet frame.
pub const REQUEST_NET: u8 = 9;
/// The dock has (payload byte 1) or has no longer (0) an Ethernet adapter.
pub const REQUEST_NET_LINK: u8 = 10;

/// The longest frame: 1500 bytes of payload, the 14 of the header and a
/// VLAN tag (no CRC: the adapter and the core's LANCE make their own).
pub const MAX_FRAME: usize = 1518;
/// The shortest that is worth carrying: an Ethernet header.
pub const MIN_FRAME: usize = 14;
/// A packet: the header and the payload.
pub const PACKET: usize = 64;
pub const HEADER: usize = 4;
pub const PAYLOAD: usize = PACKET - HEADER;

const FLAG_FIRST: u8 = 1;
const FLAG_LAST: u8 = 2;

/// The core's network registers and buffers (host memory addresses).
/// `STATUS` read: bits 10:0 the length of the frame the Amiga sends, bit 11
/// there is one, bit 15 the frame brought is still being taken, bit 16 the
/// MCU brings frames, bit 17 the board is fitted. `STATUS` write: bit 0 the
/// frame to send was taken; bit 17 sets "brings frames" from bit 16, bit 19
/// "fitted" from bit 18.
pub const STATUS: u32 = 0x0000_0050;
/// The station address's last three bytes (00:80:10:xx:xx:xx).
pub const SERIAL: u32 = 0x0000_0054;
/// Write: a frame of this many bytes is in the receive buffer.
pub const RX_COMMIT: u32 = 0x0000_0058;
/// The frame the Amiga sends (read), byte 0 first.
pub const TX_BUFFER: u32 = 0x4000_0000;
/// The frame brought to the Amiga (write), byte 0 first.
pub const RX_BUFFER: u32 = 0x4000_1000;

/// `STATUS`: the length of the frame the Amiga sends, if there is one.
pub fn tx_pending(status: u32) -> Option<usize> {
    if status & (1 << 11) != 0 {
        Some((status & 0x7FF) as usize)
    } else {
        None
    }
}

/// `STATUS`: the frame brought before is still being taken.
pub fn rx_busy(status: u32) -> bool {
    status & (1 << 15) != 0
}

/// The value to write to `STATUS` when the frame to send has been read.
pub const STATUS_TX_TAKEN: u32 = 1;

/// The link packet's flags (its first byte of data): the dock has an
/// adapter, and the adapter has said that its cable is out.
pub const LINK_ADAPTER: u8 = 1;
pub const LINK_NO_CABLE: u8 = 2;

/// The value to write to `STATUS` for the link: with an adapter on the dock
/// the board is fitted (the Amiga sees it from its next reset, and it stays
/// until the core is left), and frames are brought while the adapter's
/// cable is in; without an adapter, or without its cable, they are not.
pub fn status_link(adapter: bool, cable: bool) -> u32 {
    let fitted = if adapter { (1 << 19) | (1 << 18) } else { 0 };
    let up = if adapter && cable { 1 << 16 } else { 0 };
    fitted | (1 << 17) | up
}

/// The bytes a frame takes in the core's buffer: whole 32 bit words.
pub fn buffer_len(frame_len: usize) -> usize {
    (frame_len + 3) & !3
}

/// The pieces of a frame, each ready to send (`PACKET` bytes at most).
/// Nothing for a frame that is too short or too long.
pub fn pieces(frame: &[u8]) -> Pieces<'_> {
    let ok = frame.len() >= MIN_FRAME && frame.len() <= MAX_FRAME;
    Pieces {
        frame: if ok { frame } else { &[] },
        offset: 0,
        number: 0,
    }
}

pub struct Pieces<'a> {
    frame: &'a [u8],
    offset: usize,
    number: u8,
}

/// One packet: its bytes and how many of them count.
#[derive(Clone, Copy)]
pub struct Piece {
    pub bytes: [u8; PACKET],
    pub len: usize,
}

impl Piece {
    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Iterator for Pieces<'_> {
    type Item = Piece;

    fn next(&mut self) -> Option<Piece> {
        if self.offset >= self.frame.len() {
            return None;
        }
        let n = (self.frame.len() - self.offset).min(PAYLOAD);
        let mut bytes = [0u8; PACKET];
        let mut flags = 0;
        if self.offset == 0 {
            flags |= FLAG_FIRST;
        }
        if self.offset + n == self.frame.len() {
            flags |= FLAG_LAST;
        }
        bytes[0] = REQUEST_NET;
        bytes[1] = flags;
        bytes[2] = self.number;
        bytes[3] = n as u8;
        bytes[HEADER..HEADER + n].copy_from_slice(&self.frame[self.offset..self.offset + n]);
        self.offset += n;
        self.number = self.number.wrapping_add(1);
        Some(Piece {
            bytes,
            len: HEADER + n,
        })
    }
}

/// Frames from their pieces.
pub struct Reassembler {
    buffer: [u8; MAX_FRAME + 2],
    len: usize,
    next: u8,
    open: bool,
    /// Frames given up: a piece missing, out of turn or too many.
    pub dropped: u32,
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

impl Reassembler {
    pub const fn new() -> Self {
        Reassembler {
            buffer: [0; MAX_FRAME + 2],
            len: 0,
            next: 0,
            open: false,
            dropped: 0,
        }
    }

    fn give_up(&mut self) {
        if self.open {
            self.dropped = self.dropped.wrapping_add(1);
        }
        self.open = false;
        self.len = 0;
    }

    /// A packet (header and payload). The frame, when this was its last
    /// piece; it stays valid until the next call. The bytes after it up to
    /// `buffer_len` are zero.
    pub fn push(&mut self, packet: &[u8]) -> Option<&[u8]> {
        if packet.len() < HEADER + 1 || packet[0] != REQUEST_NET {
            return None;
        }
        let (flags, number, n) = (packet[1], packet[2], packet[3] as usize);
        if n == 0 || n > PAYLOAD || packet.len() < HEADER + n {
            self.give_up();
            return None;
        }
        if flags & FLAG_FIRST != 0 {
            self.give_up();
            if number != 0 {
                return None;
            }
            self.open = true;
            self.next = 0;
        }
        if !self.open || number != self.next || self.len + n > MAX_FRAME {
            self.give_up();
            return None;
        }
        self.buffer[self.len..self.len + n].copy_from_slice(&packet[HEADER..HEADER + n]);
        self.len += n;
        self.next = self.next.wrapping_add(1);
        if flags & FLAG_LAST == 0 {
            return None;
        }
        self.open = false;
        let len = self.len;
        self.len = 0;
        if len < MIN_FRAME {
            self.dropped = self.dropped.wrapping_add(1);
            return None;
        }
        // (whole words for the core's buffer)
        for b in &mut self.buffer[len..buffer_len(len).min(MAX_FRAME + 2)] {
            *b = 0;
        }
        Some(&self.buffer[..len])
    }
}

/// The station address's last three bytes from the handheld's serial
/// number: every handheld its own.
pub fn serial_from(handheld_serial: u32) -> u32 {
    // (a multiplicative hash: neighbouring serial numbers far apart)
    (handheld_serial.wrapping_mul(0x9E37_79B1) >> 8) & 0x00FF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(len: usize, seed: u8) -> Vec<u8> {
        (0..len)
            .map(|i| (i as u8).wrapping_mul(7).wrapping_add(seed))
            .collect()
    }

    fn carry(r: &mut Reassembler, f: &[u8]) -> Option<Vec<u8>> {
        let mut out = None;
        let all: Vec<Piece> = pieces(f).collect();
        for (i, p) in all.iter().enumerate() {
            let got = r.push(p.as_slice()).map(|s| s.to_vec());
            assert_eq!(
                got.is_some(),
                i + 1 == all.len(),
                "only the last piece gives the frame"
            );
            if got.is_some() {
                out = got;
            }
        }
        out
    }

    #[test]
    fn every_length_arrives_as_it_was_sent() {
        let mut r = Reassembler::new();
        for len in MIN_FRAME..=MAX_FRAME {
            let f = frame(len, len as u8);
            assert_eq!(carry(&mut r, &f).as_deref(), Some(&f[..]), "length {len}");
        }
        assert_eq!(r.dropped, 0);
    }

    #[test]
    fn pieces_are_at_most_a_packet_and_numbered() {
        let f = frame(1518, 3);
        let all: Vec<Piece> = pieces(&f).collect();
        assert_eq!(all.len(), 26); // 25 of 60 bytes and one of 18
        for (i, p) in all.iter().enumerate() {
            assert!(p.len <= PACKET);
            assert_eq!(p.bytes[0], REQUEST_NET);
            assert_eq!(p.bytes[2] as usize, i);
            assert_eq!(p.bytes[1] & FLAG_FIRST != 0, i == 0);
            assert_eq!(p.bytes[1] & FLAG_LAST != 0, i == 25);
        }
        assert_eq!(all[25].len, HEADER + 18);
        // a frame of one piece is both first and last
        let one: Vec<Piece> = pieces(&frame(60, 0)).collect();
        assert_eq!(one.len(), 1);
        assert_eq!(one[0].bytes[1], FLAG_FIRST | FLAG_LAST);
    }

    #[test]
    fn frames_out_of_range_are_not_sent() {
        assert_eq!(pieces(&frame(MIN_FRAME - 1, 0)).count(), 0);
        assert_eq!(pieces(&frame(MAX_FRAME + 1, 0)).count(), 0);
    }

    #[test]
    fn a_lost_piece_drops_the_frame_and_the_next_arrives() {
        let mut r = Reassembler::new();
        let a = frame(300, 1);
        let b = frame(200, 2);
        let pa: Vec<Piece> = pieces(&a).collect();
        for (i, p) in pa.iter().enumerate() {
            if i == 2 {
                continue; // lost
            }
            assert!(r.push(p.as_slice()).is_none());
        }
        assert_eq!(r.dropped, 1);
        assert_eq!(carry(&mut r, &b).as_deref(), Some(&b[..]));
        // the last piece lost: the next frame's first piece gives the old one up
        let pb: Vec<Piece> = pieces(&b).collect();
        for p in &pb[..pb.len() - 1] {
            assert!(r.push(p.as_slice()).is_none());
        }
        assert_eq!(carry(&mut r, &a).as_deref(), Some(&a[..]));
        assert_eq!(r.dropped, 2);
    }

    #[test]
    fn a_piece_twice_or_without_a_start_gives_nothing() {
        let mut r = Reassembler::new();
        let a = frame(200, 5);
        let pa: Vec<Piece> = pieces(&a).collect();
        // from the middle, with no first piece
        assert!(r.push(pa[1].as_slice()).is_none());
        assert!(r.push(pa[2].as_slice()).is_none());
        assert_eq!(r.dropped, 0);
        // a piece twice
        assert!(r.push(pa[0].as_slice()).is_none());
        assert!(r.push(pa[1].as_slice()).is_none());
        assert!(r.push(pa[1].as_slice()).is_none());
        assert!(r.push(pa[2].as_slice()).is_none());
        assert!(r.push(pa[3].as_slice()).is_none());
        assert_eq!(r.dropped, 1);
        assert_eq!(carry(&mut r, &a).as_deref(), Some(&a[..]));
    }

    #[test]
    fn packets_that_are_not_pieces_are_left_alone() {
        let mut r = Reassembler::new();
        let a = frame(100, 9);
        let pa: Vec<Piece> = pieces(&a).collect();
        assert!(r.push(pa[0].as_slice()).is_none());
        // a gamepad report in between is not this module's
        assert!(r.push(&[6, 0, 0, 0, 1, 2, 3, 4]).is_none());
        assert_eq!(r.push(pa[1].as_slice()), Some(&a[..]));
        // a piece that claims more payload than it has, or none
        let mut bad = pa[0];
        bad.bytes[3] = 61;
        assert!(r.push(bad.as_slice()).is_none());
        bad.bytes[3] = 0;
        assert!(r.push(bad.as_slice()).is_none());
        assert!(r.push(&pa[0].bytes[..10]).is_none());
        assert_eq!(carry(&mut r, &a).as_deref(), Some(&a[..]));
    }

    #[test]
    fn more_pieces_than_a_frame_holds_are_dropped() {
        let mut r = Reassembler::new();
        let f = frame(MAX_FRAME, 0);
        let mut all: Vec<Piece> = pieces(&f).collect();
        let last = all.len() - 1;
        all[last].bytes[1] = 0; // it does not end
        for p in &all {
            assert!(r.push(p.as_slice()).is_none());
        }
        let mut more = all[last];
        more.bytes[2] = 26;
        more.bytes[1] = FLAG_LAST;
        assert!(r.push(more.as_slice()).is_none());
        assert_eq!(r.dropped, 1);
        assert_eq!(carry(&mut r, &f).as_deref(), Some(&f[..]));
    }

    #[test]
    fn the_core_registers_as_gb_host_has_them() {
        assert_eq!(tx_pending(0x0000_0800 | 60), Some(60));
        assert_eq!(tx_pending(0x0003_0000 | 1514), None);
        assert_eq!(tx_pending(0x0000_0800 | 1514), Some(1514));
        assert!(rx_busy(0x8000));
        assert!(!rx_busy(0x0003_0800));
        assert_eq!(status_link(true, true), 0x000F_0000);
        assert_eq!(status_link(true, false), 0x000E_0000);
        assert_eq!(status_link(false, true), 0x0002_0000);
        assert_eq!(status_link(false, false), 0x0002_0000);
        assert_eq!(buffer_len(60), 60);
        assert_eq!(buffer_len(61), 64);
        assert_eq!(buffer_len(1518), 1520);
    }

    #[test]
    fn the_padding_after_a_frame_is_zero() {
        let mut r = Reassembler::new();
        let long = vec![0xFFu8; 64];
        carry(&mut r, &long);
        let short = frame(61, 1);
        let all: Vec<Piece> = pieces(&short).collect();
        assert!(r.push(all[0].as_slice()).is_none());
        let got = r.push(all[1].as_slice()).unwrap();
        assert_eq!(got.len(), 61);
        // (the bytes behind the slice, to the word's end)
        let p = got.as_ptr();
        for i in 61..64 {
            assert_eq!(unsafe { *p.add(i) }, 0, "byte {i}");
        }
    }

    #[test]
    fn serial_numbers_give_different_addresses() {
        let a = serial_from(1);
        let b = serial_from(2);
        assert_ne!(a, b);
        assert!(a <= 0x00FF_FFFF && b <= 0x00FF_FFFF);
        assert_ne!(serial_from(0x1234_5678), serial_from(0x1234_5679));
    }
}
