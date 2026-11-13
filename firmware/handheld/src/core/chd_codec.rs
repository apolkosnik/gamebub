//! The decoders of the CHD codecs, written so that they allocate nothing:
//! the handheld's heap is too fragmented while a core runs for the buffers
//! that general-purpose decoders allocate per call (an LZMA decoder's 32 KiB
//! window failed with 40 KB free in blocks of at most 7.5 KB). Each decoder
//! writes straight into the output (which is the whole hunk, so it is also
//! the LZ window) and keeps its tables on the stack or in memory the caller
//! lends it from the disc-read scratch buffer.
//!
//! * [`inflate`]: raw deflate (RFC 1951), after zlib's `puff.c`.
//! * [`lzma`]: LZMA without a header, after the LZMA SDK's `LzmaSpec.cpp`.
//! * [`flac`]: FLAC frames without a stream header (the FLAC format
//!   specification), as MAME's CD FLAC codec writes them.

pub type Result<T> = core::result::Result<T, String>;

// ---------------------------------------------------------------- deflate

const MAX_BITS: usize = 15;
const MAX_LCODES: usize = 286;
const MAX_DCODES: usize = 30;
const FIX_LCODES: usize = 288;

/// Bits of a deflate stream, least significant first.
struct DeflateBits<'a> {
    src: &'a [u8],
    pos: usize,
    buf: u32,
    cnt: u32,
}

impl DeflateBits<'_> {
    #[inline]
    fn bits(&mut self, need: u32) -> Result<u32> {
        let mut val = self.buf;
        while self.cnt < need {
            let Some(&b) = self.src.get(self.pos) else {
                return Err("deflate: data ends early".into());
            };
            self.pos += 1;
            val |= (b as u32) << self.cnt;
            self.cnt += 8;
        }
        self.buf = if need == 32 { 0 } else { val >> need };
        self.cnt -= need;
        Ok(if need == 32 { val } else { val & ((1 << need) - 1) })
    }
}

/// A canonical Huffman code: the number of codes of each length and the
/// symbols in code order.
struct Huffman {
    count: [u16; MAX_BITS + 1],
    symbol: [u16; FIX_LCODES],
}

impl Huffman {
    const fn new() -> Self {
        Huffman { count: [0; MAX_BITS + 1], symbol: [0; FIX_LCODES] }
    }

    /// Build the code from code lengths. Returns 0 for a complete code, a
    /// positive number for an incomplete one, a negative one for an
    /// over-subscribed one.
    fn construct(&mut self, length: &[u16]) -> i32 {
        self.count = [0; MAX_BITS + 1];
        for &l in length {
            self.count[l as usize] += 1;
        }
        if self.count[0] as usize == length.len() {
            return 0;
        }
        let mut left = 1i32;
        for len in 1..=MAX_BITS {
            left <<= 1;
            left -= self.count[len] as i32;
            if left < 0 {
                return left;
            }
        }
        let mut offs = [0u16; MAX_BITS + 1];
        for len in 1..MAX_BITS {
            offs[len + 1] = offs[len] + self.count[len];
        }
        for (symbol, &l) in length.iter().enumerate() {
            if l != 0 {
                self.symbol[offs[l as usize] as usize] = symbol as u16;
                offs[l as usize] += 1;
            }
        }
        left
    }
}

impl DeflateBits<'_> {
    fn decode(&mut self, h: &Huffman) -> Result<usize> {
        let (mut code, mut first, mut index) = (0i32, 0i32, 0i32);
        let mut buf = self.buf;
        let mut left = self.cnt;
        let mut len = 1;
        loop {
            while left > 0 {
                code |= (buf & 1) as i32;
                buf >>= 1;
                let count = h.count[len] as i32;
                if code - count < first {
                    self.buf = buf;
                    self.cnt = self.cnt.wrapping_sub(len as u32) & 7;
                    return Ok(h.symbol[(index + (code - first)) as usize] as usize);
                }
                index += count;
                first += count;
                first <<= 1;
                code <<= 1;
                len += 1;
                left -= 1;
            }
            left = (MAX_BITS + 1 - len) as u32;
            if left == 0 {
                return Err("deflate: bad code".into());
            }
            let Some(&b) = self.src.get(self.pos) else {
                return Err("deflate: data ends early".into());
            };
            self.pos += 1;
            buf = b as u32;
            left = left.min(8);
        }
    }
}

const LBASE: [u16; 29] = [
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258,
];
const LEXT: [u8; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
const DBASE: [u16; 30] = [
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145,
    8193, 12289, 16385, 24577,
];
const DEXT: [u8; 30] = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];

fn codes(s: &mut DeflateBits, out: &mut [u8], pos: &mut usize, lencode: &Huffman, distcode: &Huffman) -> Result<()> {
    loop {
        let symbol = s.decode(lencode)?;
        if symbol < 256 {
            if *pos >= out.len() {
                return Err("deflate: more data than the hunk holds".into());
            }
            out[*pos] = symbol as u8;
            *pos += 1;
        } else if symbol == 256 {
            return Ok(());
        } else {
            let symbol = symbol - 257;
            if symbol >= 29 {
                return Err("deflate: bad length code".into());
            }
            let len = LBASE[symbol] as usize + s.bits(LEXT[symbol] as u32)? as usize;
            let symbol = s.decode(distcode)?;
            if symbol >= 30 {
                return Err("deflate: bad distance code".into());
            }
            let dist = DBASE[symbol] as usize + s.bits(DEXT[symbol] as u32)? as usize;
            if dist > *pos {
                return Err("deflate: distance too far back".into());
            }
            if *pos + len > out.len() {
                return Err("deflate: more data than the hunk holds".into());
            }
            for i in *pos..*pos + len {
                out[i] = out[i - dist];
            }
            *pos += len;
        }
    }
}

/// Raw deflate (no zlib header), filling `out` exactly.
pub fn inflate(src: &[u8], out: &mut [u8]) -> Result<()> {
    let mut s = DeflateBits { src, pos: 0, buf: 0, cnt: 0 };
    let mut pos = 0usize;
    let mut lencode = Huffman::new();
    let mut distcode = Huffman::new();
    loop {
        let last = s.bits(1)?;
        match s.bits(2)? {
            0 => {
                s.buf = 0;
                s.cnt = 0;
                if s.pos + 4 > src.len() {
                    return Err("deflate: data ends early".into());
                }
                let len = u16::from_le_bytes([src[s.pos], src[s.pos + 1]]) as usize;
                let nlen = u16::from_le_bytes([src[s.pos + 2], src[s.pos + 3]]) as usize;
                if len != !nlen & 0xffff {
                    return Err("deflate: bad stored block".into());
                }
                s.pos += 4;
                if s.pos + len > src.len() {
                    return Err("deflate: data ends early".into());
                }
                if pos + len > out.len() {
                    return Err("deflate: more data than the hunk holds".into());
                }
                out[pos..pos + len].copy_from_slice(&src[s.pos..s.pos + len]);
                s.pos += len;
                pos += len;
            }
            1 => {
                let mut lengths = [0u16; FIX_LCODES];
                lengths[..144].fill(8);
                lengths[144..256].fill(9);
                lengths[256..280].fill(7);
                lengths[280..].fill(8);
                lencode.construct(&lengths);
                distcode.construct(&[5u16; MAX_DCODES]);
                codes(&mut s, out, &mut pos, &lencode, &distcode)?;
            }
            2 => {
                const ORDER: [usize; 19] = [16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15];
                let nlen = s.bits(5)? as usize + 257;
                let ndist = s.bits(5)? as usize + 1;
                let ncode = s.bits(4)? as usize + 4;
                if nlen > MAX_LCODES || ndist > MAX_DCODES {
                    return Err("deflate: bad counts".into());
                }
                let mut lengths = [0u16; MAX_LCODES + MAX_DCODES];
                for &o in &ORDER[..ncode] {
                    lengths[o] = s.bits(3)? as u16;
                }
                if lencode.construct(&lengths[..19]) != 0 {
                    return Err("deflate: incomplete code-length code".into());
                }
                let mut index = 0;
                while index < nlen + ndist {
                    let symbol = s.decode(&lencode)?;
                    if symbol < 16 {
                        lengths[index] = symbol as u16;
                        index += 1;
                    } else {
                        let (len, repeat) = match symbol {
                            16 => {
                                if index == 0 {
                                    return Err("deflate: repeat with no first length".into());
                                }
                                (lengths[index - 1], 3 + s.bits(2)? as usize)
                            }
                            17 => (0, 3 + s.bits(3)? as usize),
                            _ => (0, 11 + s.bits(7)? as usize),
                        };
                        if index + repeat > nlen + ndist {
                            return Err("deflate: too many lengths".into());
                        }
                        lengths[index..index + repeat].fill(len);
                        index += repeat;
                    }
                }
                if lengths[256] == 0 {
                    return Err("deflate: no end-of-block code".into());
                }
                let err = lencode.construct(&lengths[..nlen]);
                if err < 0 || (err > 0 && nlen - lencode.count[0] as usize != 1) {
                    return Err("deflate: bad literal/length code".into());
                }
                let err = distcode.construct(&lengths[nlen..nlen + ndist]);
                if err < 0 || (err > 0 && ndist - distcode.count[0] as usize != 1) {
                    return Err("deflate: bad distance code".into());
                }
                codes(&mut s, out, &mut pos, &lencode, &distcode)?;
            }
            _ => return Err("deflate: bad block type".into()),
        }
        if last == 1 {
            break;
        }
    }
    if pos != out.len() {
        return Err(format!("deflate gave {pos} of {} bytes", out.len()));
    }
    Ok(())
}

// ---------------------------------------------------------------- LZMA

/// chdman's LZMA properties (level 8): lc 3, lp 0, pb 2.
const LC: u32 = 3;
const LP: u32 = 0;
const PB: u32 = 2;
const STATES: usize = 12;
const POS_STATES_MAX: usize = 1 << 4;
const LEN_PROBS: usize = 2 + 2 * POS_STATES_MAX * 8 + 256;
const LITERAL: usize = 0;
const IS_MATCH: usize = LITERAL + (0x300 << (LC + LP));
const IS_REP: usize = IS_MATCH + (STATES << 4);
const IS_REP_G0: usize = IS_REP + STATES;
const IS_REP_G1: usize = IS_REP_G0 + STATES;
const IS_REP_G2: usize = IS_REP_G1 + STATES;
const IS_REP0_LONG: usize = IS_REP_G2 + STATES;
const POS_SLOT: usize = IS_REP0_LONG + (STATES << 4);
const POS_DECODERS: usize = POS_SLOT + 4 * 64;
const ALIGN: usize = POS_DECODERS + 1 + 128 - 14;
const LEN_DECODER: usize = ALIGN + 16;
const REP_LEN_DECODER: usize = LEN_DECODER + LEN_PROBS;
/// The probabilities [`lzma`] needs (16-bit each).
pub const LZMA_PROBS: usize = REP_LEN_DECODER + LEN_PROBS;

struct RangeDecoder<'a> {
    src: &'a [u8],
    pos: usize,
    range: u32,
    code: u32,
}

impl RangeDecoder<'_> {
    #[inline]
    fn normalize(&mut self) -> Result<()> {
        if self.range < 1 << 24 {
            let Some(&b) = self.src.get(self.pos) else {
                return Err("LZMA: data ends early".into());
            };
            self.pos += 1;
            self.range <<= 8;
            self.code = (self.code << 8) | b as u32;
        }
        Ok(())
    }

    #[inline]
    fn bit(&mut self, prob: &mut u16) -> Result<u32> {
        let p = *prob as u32;
        let bound = (self.range >> 11) * p;
        let bit = if self.code < bound {
            self.range = bound;
            *prob = (p + ((2048 - p) >> 5)) as u16;
            0
        } else {
            self.range -= bound;
            self.code -= bound;
            *prob = (p - (p >> 5)) as u16;
            1
        };
        self.normalize()?;
        Ok(bit)
    }

    fn direct(&mut self, n: u32) -> Result<u32> {
        let mut res = 0u32;
        for _ in 0..n {
            self.range >>= 1;
            self.code = self.code.wrapping_sub(self.range);
            let t = 0u32.wrapping_sub(self.code >> 31);
            self.code = self.code.wrapping_add(self.range & t);
            if self.code == self.range {
                return Err("LZMA: corrupt data".into());
            }
            self.normalize()?;
            res = (res << 1).wrapping_add(t.wrapping_add(1));
        }
        Ok(res)
    }

    fn tree(&mut self, probs: &mut [u16], bits: u32) -> Result<u32> {
        let mut m = 1usize;
        for _ in 0..bits {
            m = (m << 1) + self.bit(&mut probs[m])? as usize;
        }
        Ok(m as u32 - (1 << bits))
    }

    fn reverse(&mut self, probs: &mut [u16], bits: u32) -> Result<u32> {
        let mut m = 1usize;
        let mut symbol = 0u32;
        for i in 0..bits {
            let bit = self.bit(&mut probs[m])?;
            m = (m << 1) + bit as usize;
            symbol |= bit << i;
        }
        Ok(symbol)
    }

    fn len(&mut self, probs: &mut [u16], pos_state: usize) -> Result<usize> {
        let (choice, rest) = probs.split_at_mut(1);
        let (choice2, rest) = rest.split_at_mut(1);
        let (low, rest) = rest.split_at_mut(POS_STATES_MAX * 8);
        let (mid, high) = rest.split_at_mut(POS_STATES_MAX * 8);
        if self.bit(&mut choice[0])? == 0 {
            return Ok(self.tree(&mut low[pos_state * 8..pos_state * 8 + 8], 3)? as usize);
        }
        if self.bit(&mut choice2[0])? == 0 {
            return Ok(8 + self.tree(&mut mid[pos_state * 8..pos_state * 8 + 8], 3)? as usize);
        }
        Ok(16 + self.tree(high, 8)? as usize)
    }
}

/// LZMA without a header (chdman's properties), filling `out` exactly;
/// `probs` holds at least [`LZMA_PROBS`] probabilities. The output is the LZ
/// window: chdman's dictionary is never smaller than the hunk.
pub fn lzma(src: &[u8], out: &mut [u8], probs: &mut [u16]) -> Result<()> {
    let p = &mut probs[..LZMA_PROBS];
    p.fill(1024);
    if src.len() < 5 || src[0] != 0 {
        return Err("LZMA: bad stream start".into());
    }
    let mut rc = RangeDecoder {
        src,
        pos: 5,
        range: 0xffff_ffff,
        code: u32::from_be_bytes(src[1..5].try_into().unwrap()),
    };
    if rc.code == rc.range {
        return Err("LZMA: bad stream start".into());
    }
    let size = out.len();
    let mut pos = 0usize;
    let mut state = 0usize;
    let (mut rep0, mut rep1, mut rep2, mut rep3) = (0usize, 0usize, 0usize, 0usize);
    while pos < size {
        let pos_state = pos & ((1 << PB) - 1);
        if rc.bit(&mut p[IS_MATCH + (state << 4) + pos_state])? == 0 {
            let prev = if pos > 0 { out[pos - 1] as usize } else { 0 };
            let lit_state = ((pos & ((1 << LP) - 1)) << LC) + (prev >> (8 - LC));
            let lit = &mut p[LITERAL + 0x300 * lit_state..LITERAL + 0x300 * (lit_state + 1)];
            let mut symbol = 1usize;
            if state >= 7 {
                if rep0 >= pos {
                    return Err("LZMA: distance too far back".into());
                }
                let mut match_byte = out[pos - rep0 - 1] as usize;
                while symbol < 0x100 {
                    let match_bit = (match_byte >> 7) & 1;
                    match_byte <<= 1;
                    let bit = rc.bit(&mut lit[((1 + match_bit) << 8) + symbol])? as usize;
                    symbol = (symbol << 1) | bit;
                    if match_bit != bit {
                        break;
                    }
                }
            }
            while symbol < 0x100 {
                symbol = (symbol << 1) | rc.bit(&mut lit[symbol])? as usize;
            }
            out[pos] = symbol as u8;
            pos += 1;
            state = if state < 4 {
                0
            } else if state < 10 {
                state - 3
            } else {
                state - 6
            };
            continue;
        }
        let len;
        if rc.bit(&mut p[IS_REP + state])? != 0 {
            if pos == 0 {
                return Err("LZMA: repeat at the start".into());
            }
            if rc.bit(&mut p[IS_REP_G0 + state])? == 0 {
                if rc.bit(&mut p[IS_REP0_LONG + (state << 4) + pos_state])? == 0 {
                    state = if state < 7 { 9 } else { 11 };
                    if rep0 >= pos {
                        return Err("LZMA: distance too far back".into());
                    }
                    out[pos] = out[pos - rep0 - 1];
                    pos += 1;
                    continue;
                }
            } else {
                let dist;
                if rc.bit(&mut p[IS_REP_G1 + state])? == 0 {
                    dist = rep1;
                } else {
                    if rc.bit(&mut p[IS_REP_G2 + state])? == 0 {
                        dist = rep2;
                    } else {
                        dist = rep3;
                        rep3 = rep2;
                    }
                    rep2 = rep1;
                }
                rep1 = rep0;
                rep0 = dist;
            }
            len = rc.len(&mut p[REP_LEN_DECODER..REP_LEN_DECODER + LEN_PROBS], pos_state)?;
            state = if state < 7 { 8 } else { 11 };
        } else {
            rep3 = rep2;
            rep2 = rep1;
            rep1 = rep0;
            len = rc.len(&mut p[LEN_DECODER..LEN_DECODER + LEN_PROBS], pos_state)?;
            state = if state < 7 { 7 } else { 10 };
            // The distance.
            let len_state = len.min(3);
            let slot = rc.tree(&mut p[POS_SLOT + (len_state << 6)..POS_SLOT + ((len_state + 1) << 6)], 6)?;
            rep0 = if slot < 4 {
                slot as usize
            } else {
                let direct = (slot >> 1) - 1;
                let mut dist = (2 | (slot & 1)) << direct;
                if slot < 14 {
                    let base = POS_DECODERS + dist as usize - slot as usize;
                    dist += rc.reverse(&mut p[base..base + (1 << direct)], direct)?;
                } else {
                    dist = dist.wrapping_add(rc.direct(direct - 4)? << 4);
                    dist = dist.wrapping_add(rc.reverse(&mut p[ALIGN..ALIGN + 16], 4)?);
                    if dist == 0xffff_ffff {
                        // The end marker.
                        break;
                    }
                }
                dist as usize
            };
            if rep0 >= pos {
                return Err("LZMA: distance too far back".into());
            }
        }
        let len = len + 2;
        if pos + len > size {
            return Err("LZMA: more data than the hunk holds".into());
        }
        for i in pos..pos + len {
            out[i] = out[i - rep0 - 1];
        }
        pos += len;
    }
    if pos != size {
        return Err(format!("LZMA gave {pos} of {size} bytes"));
    }
    Ok(())
}

// ---------------------------------------------------------------- FLAC

/// The largest FLAC block [`flac`] decodes (MAME's CD FLAC codec halves the
/// hunk's samples until they are at most a sector's bytes, 2352).
pub const FLAC_MAX_BLOCK: usize = 2352;
/// The samples [`flac`] needs to decode into (both channels of a block).
pub const FLAC_SAMPLES: usize = 2 * FLAC_MAX_BLOCK;

/// Bits of a FLAC stream, most significant first.
struct FlacBits<'a> {
    src: &'a [u8],
    pos: usize,
    acc: u64,
    n: u32,
}

impl FlacBits<'_> {
    #[inline]
    fn refill(&mut self) {
        while self.n <= 56 {
            let b = self.src.get(self.pos).copied().unwrap_or(0);
            self.pos += 1;
            self.acc |= (b as u64) << (56 - self.n);
            self.n += 8;
        }
    }

    /// The bytes taken so far.
    fn consumed(&self) -> usize {
        self.pos - (self.n / 8) as usize
    }

    fn check(&self) -> Result<()> {
        if self.consumed() > self.src.len() {
            return Err("FLAC: data ends early".into());
        }
        Ok(())
    }

    #[inline]
    fn read(&mut self, bits: u32) -> u32 {
        if bits == 0 {
            return 0;
        }
        if self.n < bits {
            self.refill();
        }
        let v = (self.acc >> (64 - bits)) as u32;
        self.acc <<= bits;
        self.n -= bits;
        v
    }

    #[inline]
    fn signed(&mut self, bits: u32) -> i32 {
        if bits == 0 {
            return 0;
        }
        let v = self.read(bits);
        ((v << (32 - bits)) as i32) >> (32 - bits)
    }

    /// Zeros up to the next one (which is taken too).
    #[inline]
    fn unary(&mut self) -> Result<u32> {
        let mut q = 0u32;
        loop {
            if self.n == 0 || self.acc == 0 {
                q += self.n;
                self.acc = 0;
                self.n = 0;
                if self.pos > self.src.len() {
                    return Err("FLAC: data ends early".into());
                }
                self.refill();
                continue;
            }
            let z = self.acc.leading_zeros();
            q += z;
            self.acc = self.acc.checked_shl(z + 1).unwrap_or(0);
            self.n -= z + 1;
            return Ok(q);
        }
    }

    fn align(&mut self) {
        let drop = self.n % 8;
        self.acc <<= drop;
        self.n -= drop;
    }
}

fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        }
    }
    crc
}

fn residual(s: &mut FlacBits, out: &mut [i32], order: usize) -> Result<()> {
    let n = out.len();
    let (param_bits, escape) = match s.read(2) {
        0 => (4, 15),
        1 => (5, 31),
        _ => return Err("FLAC: bad residual coding".into()),
    };
    let partition_order = s.read(4);
    let partitions = 1usize << partition_order;
    if n % partitions != 0 || n / partitions < order {
        return Err("FLAC: bad partition order".into());
    }
    let mut i = order;
    for part in 0..partitions {
        let end = (part + 1) * (n / partitions);
        let k = s.read(param_bits);
        if k == escape {
            let bits = s.read(5);
            for v in &mut out[i..end] {
                *v = s.signed(bits);
            }
        } else {
            for v in &mut out[i..end] {
                let q = s.unary()?;
                let u = (q << k) | s.read(k);
                *v = ((u >> 1) as i32) ^ -((u & 1) as i32);
            }
        }
        i = end;
    }
    s.check()
}

fn subframe(s: &mut FlacBits, out: &mut [i32], bps: u32) -> Result<()> {
    if s.read(1) != 0 {
        return Err("FLAC: bad subframe padding".into());
    }
    let kind = s.read(6);
    let wasted = if s.read(1) != 0 { s.unary()? + 1 } else { 0 };
    if wasted >= bps {
        return Err("FLAC: bad wasted bits".into());
    }
    let bps = bps - wasted;
    let n = out.len();
    match kind {
        0 => {
            let v = s.signed(bps);
            out.fill(v);
        }
        1 => {
            for v in out.iter_mut() {
                *v = s.signed(bps);
            }
        }
        8..=12 => {
            let order = (kind - 8) as usize;
            if order > n {
                return Err("FLAC: order beyond the block".into());
            }
            for v in &mut out[..order] {
                *v = s.signed(bps);
            }
            residual(s, out, order)?;
            for i in order..n {
                let r = out[i];
                out[i] = match order {
                    0 => r,
                    1 => r.wrapping_add(out[i - 1]),
                    2 => r.wrapping_add(2i32.wrapping_mul(out[i - 1])).wrapping_sub(out[i - 2]),
                    3 => r
                        .wrapping_add(3i32.wrapping_mul(out[i - 1].wrapping_sub(out[i - 2])))
                        .wrapping_add(out[i - 3]),
                    _ => r
                        .wrapping_add(4i32.wrapping_mul(out[i - 1].wrapping_add(out[i - 3])))
                        .wrapping_sub(6i32.wrapping_mul(out[i - 2]))
                        .wrapping_sub(out[i - 4]),
                };
            }
        }
        32..=63 => {
            let order = (kind - 31) as usize;
            if order > n {
                return Err("FLAC: order beyond the block".into());
            }
            for v in &mut out[..order] {
                *v = s.signed(bps);
            }
            let precision = s.read(4);
            if precision == 15 {
                return Err("FLAC: bad coefficient precision".into());
            }
            let precision = precision + 1;
            let shift = s.signed(5);
            if shift < 0 {
                return Err("FLAC: negative shift".into());
            }
            let mut coefs = [0i32; 32];
            for c in &mut coefs[..order] {
                *c = s.signed(precision);
            }
            residual(s, out, order)?;
            for i in order..n {
                let mut sum = 0i64;
                for (j, &c) in coefs[..order].iter().enumerate() {
                    sum += c as i64 * out[i - j - 1] as i64;
                }
                out[i] = out[i].wrapping_add((sum >> shift) as i32);
            }
        }
        _ => return Err(format!("FLAC: subframe type {kind}")),
    }
    if wasted > 0 {
        for v in out.iter_mut() {
            *v <<= wasted;
        }
    }
    s.check()
}

/// FLAC frames (no stream header) of 16-bit stereo, as MAME's CD FLAC codec
/// writes them, decoded into `out` (filled exactly) as big-endian samples;
/// `samples` holds at least [`FLAC_SAMPLES`]. Returns the bytes of `src` the
/// frames took (the subcode follows them).
pub fn flac(src: &[u8], out: &mut [u8], samples: &mut [i32]) -> Result<usize> {
    let total = out.len() / 4;
    let mut done = 0usize;
    let mut s = FlacBits { src, pos: 0, acc: 0, n: 0 };
    while done < total {
        let start = s.consumed();
        if s.read(15) != 0x7ffc {
            return Err("FLAC: no frame sync".into());
        }
        s.read(1);
        let block_code = s.read(4);
        let rate_code = s.read(4);
        let channels = s.read(4);
        let size_code = s.read(3);
        if s.read(1) != 0 {
            return Err("FLAC: bad frame header".into());
        }
        // The frame or sample number (UTF-8 style).
        let first = s.read(8);
        let extra = (first as u8).leading_ones();
        if extra == 1 || extra > 7 {
            return Err("FLAC: bad frame number".into());
        }
        for _ in 1..extra {
            if s.read(8) & 0xc0 != 0x80 {
                return Err("FLAC: bad frame number".into());
            }
        }
        let block = match block_code {
            1 => 192,
            2..=5 => 576 << (block_code - 2),
            6 => s.read(8) as usize + 1,
            7 => s.read(16) as usize + 1,
            8..=15 => 256 << (block_code - 8),
            _ => return Err("FLAC: bad block size".into()),
        };
        match rate_code {
            12 => {
                s.read(8);
            }
            13 | 14 => {
                s.read(16);
            }
            15 => return Err("FLAC: bad sample rate".into()),
            _ => {}
        }
        let header_end = s.consumed();
        let crc = s.read(8) as u8;
        s.check()?;
        if crc8(&src[start..header_end]) != crc {
            return Err("FLAC: frame header CRC mismatch".into());
        }
        if size_code != 0 && size_code != 4 {
            return Err("FLAC: not 16-bit".into());
        }
        if block > FLAC_MAX_BLOCK || done + block > total {
            return Err(format!("FLAC: block of {block} samples"));
        }
        let (left, rest) = samples[..2 * FLAC_MAX_BLOCK].split_at_mut(FLAC_MAX_BLOCK);
        let (left, right) = (&mut left[..block], &mut rest[..block]);
        match channels {
            1 => {
                subframe(&mut s, left, 16)?;
                subframe(&mut s, right, 16)?;
            }
            8 => {
                subframe(&mut s, left, 16)?;
                subframe(&mut s, right, 17)?;
                for i in 0..block {
                    right[i] = left[i].wrapping_sub(right[i]);
                }
            }
            9 => {
                subframe(&mut s, left, 17)?;
                subframe(&mut s, right, 16)?;
                for i in 0..block {
                    left[i] = left[i].wrapping_add(right[i]);
                }
            }
            10 => {
                subframe(&mut s, left, 16)?;
                subframe(&mut s, right, 17)?;
                for i in 0..block {
                    let side = right[i];
                    let mid = (left[i] << 1) | (side & 1);
                    left[i] = (mid + side) >> 1;
                    right[i] = (mid - side) >> 1;
                }
            }
            _ => return Err("FLAC: not stereo".into()),
        }
        s.align();
        s.read(16); // The frame's CRC-16 (the hunk's CRC covers the samples).
        s.check()?;
        for i in 0..block {
            let at = (done + i) * 4;
            out[at..at + 2].copy_from_slice(&(left[i] as i16).to_be_bytes());
            out[at + 2..at + 4].copy_from_slice(&(right[i] as i16).to_be_bytes());
        }
        done += block;
    }
    Ok(s.consumed())
}
