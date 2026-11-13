//! CD images in MAME's CHD format (version 5, as `chdman createcd` makes
//! them): the hunks of CD frames (2352 bytes of sector data and 96 of
//! subcode each), compressed with the CD codecs (`cdlz`: LZMA, `cdzl`:
//! deflate, `cdfl`: FLAC; the subcode with deflate) or the generic ones
//! (`zlib`, `lzma`), or stored; and the track list from the `CHT2`/`CHTR`
//! metadata. The zstd codecs (`cdzs`, `zstd`) are refused: a zstd decoder
//! needs more memory than the handheld has free while a core runs.
//!
//! Nothing is allocated after [`Chd::open`]: the decoders
//! ([`super::chd_codec`]) work in the caller's scratch space.
//!
//! There is no PSRAM, so the hunk map is not decoded into memory (12 bytes per
//! hunk: 500 KiB for a full CD). The compressed map stays in the file, and a
//! checkpoint of its decoder's state every [`CHECKPOINT`] hunks (24 bytes)
//! lets the entry of any hunk be decoded from the one before it. The map is
//! decoded once when the file is opened, to place the checkpoints and check
//! its CRC.
//!
//! One hunk (8 frames as `chdman` makes them) is kept decompressed: the core
//! reads sectors in order, so most reads are copies from it. Only the sector
//! data is kept; the subcode is decompressed to check the hunk's CRC.
//!
//! The reference is MAME's `src/lib/util/chd.cpp`, `chdcodec.cpp`,
//! `huffman.cpp` and `cdrom.cpp`.

use super::chd_codec;
use std::{
    fs::File,
    io::{Read, Seek, SeekFrom},
    path::Path,
};

/// A CD frame in a CHD: the raw sector and its subcode.
pub const FRAME_SIZE: usize = 2448;
pub const SECTOR_DATA: usize = 2352;
const SUBCODE_DATA: usize = 96;
/// The largest hunk supported, in frames (`chdman createcd` makes 8).
pub const MAX_HUNK_FRAMES: u32 = 8;
/// `chdman` pads each track to a multiple of 4 frames.
const TRACK_PADDING: u32 = 4;
/// The decoders' share of the work space (LZMA probabilities or FLAC
/// samples), with room to align it.
const DECODER_SPACE: usize = 4 * chd_codec::FLAC_SAMPLES + 8;
/// The work space [`Chd::read_frame`] needs: the compressed hunk, the whole
/// hunk for the generic codecs, and the decoders' tables.
pub const WORK_SIZE: usize = 2 * MAX_HUNK_FRAMES as usize * FRAME_SIZE + DECODER_SPACE;
const _: () = assert!(DECODER_SPACE >= 2 * chd_codec::LZMA_PROBS + 2);
/// Hunks between two checkpoints of the map decoder.
const CHECKPOINT: u32 = 128;
const MAX_TRACKS: usize = 99;
const SYNC: [u8; 12] = [0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00];

const fn tag(s: &[u8; 4]) -> u32 {
    u32::from_be_bytes(*s)
}
const CODEC_NONE: u32 = 0;
const CODEC_ZLIB: u32 = tag(b"zlib");
const CODEC_LZMA: u32 = tag(b"lzma");
const CODEC_ZSTD: u32 = tag(b"zstd");
const CODEC_CD_ZLIB: u32 = tag(b"cdzl");
const CODEC_CD_LZMA: u32 = tag(b"cdlz");
const CODEC_CD_FLAC: u32 = tag(b"cdfl");
const CODEC_CD_ZSTD: u32 = tag(b"cdzs");
const META_TRACK: u32 = tag(b"CHTR");
const META_TRACK2: u32 = tag(b"CHT2");
const META_GDROM: u32 = tag(b"CHGD");
const META_GDROM_OLD: u32 = tag(b"CHGT");

// Hunk types in the map (and the pseudo-types of its compressed form).
const COMPRESSION_TYPE_0: u8 = 0;
const COMPRESSION_TYPE_3: u8 = 3;
const COMPRESSION_NONE: u8 = 4;
const COMPRESSION_SELF: u8 = 5;
const COMPRESSION_PARENT: u8 = 6;
const COMPRESSION_RLE_SMALL: u8 = 7;
const COMPRESSION_RLE_LARGE: u8 = 8;
const COMPRESSION_SELF_0: u8 = 9;
const COMPRESSION_SELF_1: u8 = 10;
const COMPRESSION_PARENT_SELF: u8 = 11;
const COMPRESSION_PARENT_0: u8 = 12;
const COMPRESSION_PARENT_1: u8 = 13;

/// A track as the CHD's metadata gives it.
#[derive(Debug, Clone)]
pub struct ChdTrack {
    pub number: u8,
    pub audio: bool,
    /// 2048 bytes of MODE1 user data per frame (`MODE1`), not raw sectors.
    pub cooked: bool,
    /// Frames of the track in the CHD (its pregap included if `pregap_in_data`).
    pub frames: u32,
    pub pregap: u32,
    /// The pregap's sectors are in the CHD (a cue sheet's `INDEX 00`), before index 1.
    pub pregap_in_data: bool,
    pub postgap: u32,
    /// The track's first frame in the CHD.
    pub chd_frame: u32,
}

/// CRC-16/CCITT-FALSE, as MAME's `crc16_creator`.
struct Crc16(u16);
impl Crc16 {
    fn new() -> Self {
        Crc16(0xffff)
    }
    fn update(&mut self, data: &[u8]) {
        for &b in data {
            let mut x = (self.0 >> 8) as u8 ^ b;
            x ^= x >> 4;
            self.0 = (self.0 << 8) ^ ((x as u16) << 12) ^ ((x as u16) << 5) ^ (x as u16);
        }
    }
}

/// MAME's Huffman decoder for the map's hunk types: 16 codes of at most 8 bits.
struct Huffman {
    /// Indexed by the next 8 bits: the code (bits 15-5) and its length (4-0).
    lookup: [u16; 256],
}

impl Huffman {
    fn import_tree_rle(bits: &mut MapBits, file: &mut File) -> Result<Self, String> {
        const NUM_CODES: usize = 16;
        const MAX_BITS: u32 = 8;
        let mut lengths = [0u8; NUM_CODES];
        let mut node = 0;
        while node < NUM_CODES {
            let n = bits.read(file, 4)? as u8;
            if n != 1 {
                lengths[node] = n;
                node += 1;
            } else {
                let n = bits.read(file, 4)? as u8;
                if n == 1 {
                    lengths[node] = 1;
                    node += 1;
                } else {
                    let repeat = bits.read(file, 4)? as usize + 3;
                    if node + repeat > NUM_CODES {
                        return Err("bad map code table".into());
                    }
                    lengths[node..node + repeat].fill(n);
                    node += repeat;
                }
            }
        }
        // Canonical codes, longest first.
        let mut histogram = [0u32; 33];
        for &l in &lengths {
            if l as u32 > MAX_BITS {
                return Err("bad map code table".into());
            }
            histogram[l as usize] += 1;
        }
        let mut start = 0u32;
        for length in (1..=32).rev() {
            let next = (start + histogram[length]) >> 1;
            if length != 1 && next * 2 != start + histogram[length] {
                return Err("bad map code table".into());
            }
            histogram[length] = start;
            start = next;
        }
        let mut lookup = [0u16; 256];
        for (code, &l) in lengths.iter().enumerate() {
            if l == 0 {
                continue;
            }
            let bits = histogram[l as usize];
            histogram[l as usize] += 1;
            let shift = MAX_BITS - l as u32;
            let first = (bits << shift) as usize;
            let last = (((bits + 1) << shift) - 1) as usize;
            if last >= lookup.len() {
                return Err("bad map code table".into());
            }
            lookup[first..=last].fill(((code as u16) << 5) | l as u16);
        }
        Ok(Huffman { lookup })
    }

    fn decode(&self, bits: &mut MapBits, file: &mut File) -> Result<u8, String> {
        let entry = self.lookup[bits.peek(file, 8)? as usize];
        bits.skip((entry & 0x1f) as u32);
        Ok((entry >> 5) as u8)
    }
}

/// An MSB-first bit reader over the compressed map in the file, through a
/// small window. Bits past the map's end read as zeros (as MAME's).
struct MapBits {
    /// The map's bytes in the file.
    start: u64,
    len: u32,
    /// The position, in bits from the map's start.
    pos: u64,
    window: Vec<u8>,
    /// The map offset of the window's first byte, and its valid length.
    window_at: u32,
    window_len: usize,
}

impl MapBits {
    const WINDOW: usize = 1024;

    fn new(start: u64, len: u32) -> Self {
        MapBits { start, len, pos: 0, window: vec![0; Self::WINDOW], window_at: 0, window_len: 0 }
    }

    fn byte(&mut self, file: &mut File, at: u32) -> Result<u8, String> {
        if at >= self.len {
            return Ok(0);
        }
        if at < self.window_at || at >= self.window_at + self.window_len as u32 {
            let len = (self.len - at).min(Self::WINDOW as u32) as usize;
            file.seek(SeekFrom::Start(self.start + at as u64))
                .and_then(|_| file.read_exact(&mut self.window[..len]))
                .map_err(|e| format!("map: {e}"))?;
            self.window_at = at;
            self.window_len = len;
        }
        Ok(self.window[(at - self.window_at) as usize])
    }

    /// The next `n` bits (at most 32), not consumed.
    fn peek(&mut self, file: &mut File, n: u32) -> Result<u32, String> {
        if n == 0 {
            return Ok(0);
        }
        let first = (self.pos / 8) as u32;
        let skip = (self.pos % 8) as u32;
        let mut acc = 0u64;
        let bytes = (skip + n + 7) / 8;
        for i in 0..bytes {
            acc = (acc << 8) | self.byte(file, first.saturating_add(i))? as u64;
        }
        Ok(((acc >> (bytes * 8 - skip - n)) & ((1u64 << n) - 1)) as u32)
    }

    fn skip(&mut self, n: u32) {
        self.pos += n as u64;
    }

    fn read(&mut self, file: &mut File, n: u32) -> Result<u32, String> {
        let v = self.peek(file, n)?;
        self.skip(n);
        Ok(v)
    }
}

/// The map decoder's state before a hunk: the hunk-type stream (its bit
/// position, the rest of a run and the type it repeats) and the entry stream
/// (its bit position, the next hunk's file offset, the last self reference).
#[derive(Clone, Copy, Default)]
struct MapState {
    type_pos: u64,
    repeat: u32,
    last_type: u8,
    data_pos: u64,
    offset: u64,
    last_self: u32,
}

/// A hunk's map entry.
#[derive(Clone, Copy, Debug)]
struct MapEntry {
    kind: u8,
    length: u32,
    /// The file offset, or for a self reference the hunk it repeats.
    offset: u64,
    crc: u16,
}

struct CompressedMap {
    huffman: Huffman,
    length_bits: u32,
    self_bits: u32,
    parent_bits: u32,
    types: MapBits,
    entries: MapBits,
    /// The state before hunk `CHECKPOINT * i`.
    checkpoints: Vec<MapState>,
}

impl CompressedMap {
    /// The next hunk's type, from the type stream.
    fn next_type(&mut self, state: &mut MapState, file: &mut File) -> Result<u8, String> {
        self.types.pos = state.type_pos;
        let kind = if state.repeat > 0 {
            state.repeat -= 1;
            state.last_type
        } else {
            let v = self.huffman.decode(&mut self.types, file)?;
            if v == COMPRESSION_RLE_SMALL {
                state.repeat = 2 + self.huffman.decode(&mut self.types, file)? as u32;
            } else if v == COMPRESSION_RLE_LARGE {
                let high = self.huffman.decode(&mut self.types, file)? as u32;
                let low = self.huffman.decode(&mut self.types, file)? as u32;
                state.repeat = 2 + 16 + (high << 4) + low;
            } else {
                state.last_type = v;
            }
            // A run code gives this hunk the last type, and `repeat` more after it.
            state.last_type
        };
        state.type_pos = self.types.pos;
        Ok(kind)
    }

    /// The next hunk's entry, of type `kind`, from the entry stream.
    fn next_entry(
        &mut self,
        kind: u8,
        hunk: u32,
        hunk_bytes: u32,
        unit_bytes: u32,
        state: &mut MapState,
        file: &mut File,
    ) -> Result<MapEntry, String> {
        self.entries.pos = state.data_pos;
        let mut entry = MapEntry { kind, length: 0, offset: state.offset, crc: 0 };
        match kind {
            COMPRESSION_TYPE_0..=COMPRESSION_TYPE_3 => {
                entry.length = self.entries.read(file, self.length_bits)?;
                state.offset += entry.length as u64;
                entry.crc = self.entries.read(file, 16)? as u16;
            }
            COMPRESSION_NONE => {
                entry.length = hunk_bytes;
                state.offset += hunk_bytes as u64;
                entry.crc = self.entries.read(file, 16)? as u16;
            }
            COMPRESSION_SELF => {
                state.last_self = self.entries.read(file, self.self_bits)?;
                entry.offset = state.last_self as u64;
            }
            COMPRESSION_SELF_0 | COMPRESSION_SELF_1 => {
                if kind == COMPRESSION_SELF_1 {
                    state.last_self = state.last_self.wrapping_add(1);
                }
                entry.kind = COMPRESSION_SELF;
                entry.offset = state.last_self as u64;
            }
            COMPRESSION_PARENT => {
                entry.offset = self.entries.read(file, self.parent_bits)? as u64;
            }
            COMPRESSION_PARENT_SELF | COMPRESSION_PARENT_0 | COMPRESSION_PARENT_1 => {
                entry.kind = COMPRESSION_PARENT;
                entry.offset = hunk as u64 * hunk_bytes as u64 / unit_bytes as u64;
            }
            other => return Err(format!("hunk {hunk}: bad map entry type {other}")),
        }
        state.data_pos = self.entries.pos;
        Ok(entry)
    }
}

enum Map {
    /// An uncompressed CHD: a 4-byte hunk number per hunk at this offset (0: all zeros).
    Raw(u64),
    Compressed(CompressedMap),
}

pub struct Chd {
    file: File,
    hunk_bytes: u32,
    unit_bytes: u32,
    hunk_count: u32,
    codecs: [u32; 4],
    map: Map,
    /// The sector data of hunk `cached` (2352 bytes per frame), as in the CHD
    /// (audio samples big-endian).
    cache: Vec<u8>,
    cached: Option<u32>,
    /// A hunk's subcode, for its CRC.
    subcode: Vec<u8>,
    pub tracks: Vec<ChdTrack>,
}

fn be16(b: &[u8]) -> u32 {
    u16::from_be_bytes([b[0], b[1]]) as u32
}
fn be24(b: &[u8]) -> u32 {
    (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32
}
fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes(b[..4].try_into().unwrap())
}
fn be48(b: &[u8]) -> u64 {
    (be16(b) as u64) << 32 | be32(&b[2..]) as u64
}
fn be64(b: &[u8]) -> u64 {
    u64::from_be_bytes(b[..8].try_into().unwrap())
}

fn codec_name(codec: u32) -> String {
    if codec == CODEC_NONE {
        return "none".into();
    }
    codec.to_be_bytes().iter().map(|&b| if b.is_ascii_graphic() { b as char } else { '?' }).collect()
}

impl Chd {
    pub fn open(path: &Path) -> Result<Self, String> {
        let mut file = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut header = [0u8; 124];
        let n = file.read(&mut header).map_err(|e| format!("{}: {e}", path.display()))?;
        if n < 16 || &header[..8] != b"MComprHD" {
            return Err(format!("{}: not a CHD file", path.display()));
        }
        let version = be32(&header[12..]);
        if version != 5 {
            return Err(format!(
                "{}: CHD version {version} (version 5 is supported: convert it with a newer chdman)",
                path.display()
            ));
        }
        if n < header.len() || be32(&header[8..]) < 124 {
            return Err(format!("{}: CHD header too short", path.display()));
        }
        let codecs = [be32(&header[16..]), be32(&header[20..]), be32(&header[24..]), be32(&header[28..])];
        let logical_bytes = be64(&header[32..]);
        let map_offset = be64(&header[40..]);
        let meta_offset = be64(&header[48..]);
        let hunk_bytes = be32(&header[56..]);
        let unit_bytes = be32(&header[60..]);
        if header[104..124].iter().any(|&b| b != 0) {
            return Err(format!("{}: a CHD that needs a parent CHD (not supported)", path.display()));
        }
        if unit_bytes != FRAME_SIZE as u32 || hunk_bytes == 0 || hunk_bytes % FRAME_SIZE as u32 != 0 {
            return Err(format!(
                "{}: not a CD image (unit {unit_bytes} bytes, hunk {hunk_bytes})",
                path.display()
            ));
        }
        if hunk_bytes > MAX_HUNK_FRAMES * FRAME_SIZE as u32 {
            return Err(format!(
                "{}: hunks of {} frames (at most {MAX_HUNK_FRAMES} are supported, chdman's default)",
                path.display(),
                hunk_bytes / FRAME_SIZE as u32
            ));
        }
        for &c in &codecs {
            if c == CODEC_ZSTD || c == CODEC_CD_ZSTD {
                return Err(format!(
                    "{}: zstd compression (not supported: recreate it with chdman's default codecs)",
                    path.display()
                ));
            }
            if !matches!(c, CODEC_NONE | CODEC_ZLIB | CODEC_LZMA | CODEC_CD_ZLIB | CODEC_CD_LZMA | CODEC_CD_FLAC) {
                return Err(format!("{}: unsupported codec {}", path.display(), codec_name(c)));
            }
        }
        let hunk_count = logical_bytes.div_ceil(hunk_bytes as u64);
        if hunk_count > u32::MAX as u64 / 2 {
            return Err(format!("{}: too many hunks", path.display()));
        }
        let hunk_count = hunk_count as u32;
        let frames_per_hunk = hunk_bytes as usize / FRAME_SIZE;

        let tracks = Self::read_tracks(&mut file, meta_offset).map_err(|e| format!("{}: {e}", path.display()))?;
        let map = if codecs[0] == CODEC_NONE {
            Map::Raw(map_offset)
        } else {
            Map::Compressed(
                Self::scan_map(&mut file, map_offset, hunk_count, hunk_bytes, unit_bytes)
                    .map_err(|e| format!("{}: {e}", path.display()))?,
            )
        };
        let chd = Chd {
            file,
            hunk_bytes,
            unit_bytes,
            hunk_count,
            codecs,
            map,
            cache: vec![0; frames_per_hunk * SECTOR_DATA],
            cached: None,
            subcode: vec![0; frames_per_hunk * SUBCODE_DATA],
            tracks,
        };
        let frames: u64 = chd.tracks.iter().map(|t| (t.chd_frame + t.frames) as u64).max().unwrap_or(0);
        if frames > hunk_count as u64 * frames_per_hunk as u64 {
            return Err(format!("{}: the tracks need more frames than the CHD has", path.display()));
        }
        log::info!(
            "CHD: {} hunks of {} frames, codecs {}",
            hunk_count,
            frames_per_hunk,
            codecs.iter().map(|&c| codec_name(c)).collect::<Vec<_>>().join(",")
        );
        Ok(chd)
    }

    /// The tracks, from the `CHT2` (or older `CHTR`) metadata entries.
    fn read_tracks(file: &mut File, mut offset: u64) -> Result<Vec<ChdTrack>, String> {
        let mut found: Vec<Option<ChdTrack>> = vec![None; MAX_TRACKS];
        let mut entries = 0;
        while offset != 0 {
            entries += 1;
            if entries > 10_000 {
                return Err("metadata list does not end".into());
            }
            let mut header = [0u8; 16];
            file.seek(SeekFrom::Start(offset))
                .and_then(|_| file.read_exact(&mut header))
                .map_err(|e| format!("metadata: {e}"))?;
            let meta_tag = be32(&header);
            let length = be24(&header[5..]) as usize;
            let next = be64(&header[8..]);
            if meta_tag == META_GDROM || meta_tag == META_GDROM_OLD {
                return Err("a GD-ROM image".into());
            }
            if meta_tag == META_TRACK || meta_tag == META_TRACK2 {
                if length > 256 {
                    return Err("track metadata too long".into());
                }
                let mut text = vec![0u8; length];
                file.read_exact(&mut text).map_err(|e| format!("metadata: {e}"))?;
                let text = String::from_utf8_lossy(&text);
                let track = Self::parse_track(text.trim_end_matches('\0'))?;
                let slot = &mut found[track.number as usize - 1];
                if slot.is_some() {
                    return Err(format!("track {} twice", track.number));
                }
                *slot = Some(track);
            }
            offset = next;
        }
        let count = found.iter().take_while(|t| t.is_some()).count();
        if count == 0 || found[count..].iter().any(|t| t.is_some()) {
            return Err("no track list, or a track missing from it".into());
        }
        let mut tracks: Vec<ChdTrack> = found.into_iter().take(count).map(Option::unwrap).collect();
        let mut chd_frame = 0u32;
        for t in &mut tracks {
            t.chd_frame = chd_frame;
            chd_frame += t.frames.div_ceil(TRACK_PADDING) * TRACK_PADDING;
        }
        Ok(tracks)
    }

    /// `TRACK:1 TYPE:MODE1_RAW SUBTYPE:NONE FRAMES:330030 PREGAP:0 PGTYPE:MODE1 PGSUB:RW POSTGAP:0`
    fn parse_track(text: &str) -> Result<ChdTrack, String> {
        let field = |key: &str| {
            text.split_ascii_whitespace()
                .find_map(|f| f.strip_prefix(key).and_then(|v| v.strip_prefix(':')))
        };
        let number_of = |key: &str| -> Result<u32, String> {
            field(key).unwrap_or("0").parse().map_err(|_| format!("bad track metadata: {text}"))
        };
        let number = number_of("TRACK")?;
        if number == 0 || number as usize > MAX_TRACKS {
            return Err(format!("bad track metadata: {text}"));
        }
        let kind = field("TYPE").ok_or_else(|| format!("bad track metadata: {text}"))?;
        let (audio, cooked) = match kind {
            "AUDIO" => (true, false),
            "MODE1_RAW" | "MODE2_RAW" => (false, false),
            "MODE1" => (false, true),
            other => {
                return Err(format!(
                    "track {number}: unsupported type {other} (AUDIO, MODE1_RAW, MODE2_RAW and MODE1 are)"
                ))
            }
        };
        let frames = number_of("FRAMES")?;
        let pregap = number_of("PREGAP")?;
        let pregap_in_data = pregap > 0 && field("PGTYPE").map_or(false, |t| t.starts_with('V'));
        if frames == 0 || (pregap_in_data && pregap >= frames) {
            return Err(format!("bad track metadata: {text}"));
        }
        Ok(ChdTrack {
            number: number as u8,
            audio,
            cooked,
            frames,
            pregap,
            pregap_in_data,
            postgap: number_of("POSTGAP")?,
            chd_frame: 0,
        })
    }

    /// Decode the whole map once: place the checkpoints, check the CRC.
    fn scan_map(
        file: &mut File,
        map_offset: u64,
        hunk_count: u32,
        hunk_bytes: u32,
        unit_bytes: u32,
    ) -> Result<CompressedMap, String> {
        let mut header = [0u8; 16];
        file.seek(SeekFrom::Start(map_offset))
            .and_then(|_| file.read_exact(&mut header))
            .map_err(|e| format!("map: {e}"))?;
        let map_bytes = be32(&header);
        let first_offset = be48(&header[4..]);
        let map_crc = be16(&header[10..]) as u16;
        let (length_bits, self_bits, parent_bits) = (header[12] as u32, header[13] as u32, header[14] as u32);
        if length_bits > 32 || self_bits > 32 || parent_bits > 32 {
            return Err("bad map header".into());
        }
        let start = map_offset + 16;
        let mut types = MapBits::new(start, map_bytes);
        let huffman = Huffman::import_tree_rle(&mut types, file)?;
        let mut map = CompressedMap {
            huffman,
            length_bits,
            self_bits,
            parent_bits,
            types,
            entries: MapBits::new(start, map_bytes),
            checkpoints: Vec::with_capacity(hunk_count.div_ceil(CHECKPOINT) as usize),
        };

        // The type stream first: the entries follow its end.
        let mut state = MapState { type_pos: map.types.pos, ..Default::default() };
        let types_start = state;
        for _ in 0..hunk_count {
            map.next_type(&mut state, file)?;
        }
        if state.type_pos > map_bytes as u64 * 8 {
            return Err("map too short".into());
        }

        // Both streams, hunk by hunk.
        let mut state = MapState {
            data_pos: state.type_pos,
            offset: first_offset,
            ..types_start
        };
        let mut crc = Crc16::new();
        for hunk in 0..hunk_count {
            if hunk % CHECKPOINT == 0 {
                map.checkpoints.push(state);
            }
            let kind = map.next_type(&mut state, file)?;
            let e = map.next_entry(kind, hunk, hunk_bytes, unit_bytes, &mut state, file)?;
            if e.kind == COMPRESSION_PARENT {
                return Err("a CHD that needs a parent CHD (not supported)".into());
            }
            if e.kind == COMPRESSION_SELF && e.offset >= hunk as u64 {
                return Err(format!("hunk {hunk}: bad self reference"));
            }
            let mut raw = [0u8; 12];
            raw[0] = e.kind;
            raw[1..4].copy_from_slice(&e.length.to_be_bytes()[1..]);
            raw[4..10].copy_from_slice(&e.offset.to_be_bytes()[2..]);
            raw[10..12].copy_from_slice(&e.crc.to_be_bytes());
            crc.update(&raw);
        }
        if state.data_pos > map_bytes as u64 * 8 {
            return Err("map too short".into());
        }
        if crc.0 != map_crc {
            return Err("map CRC mismatch".into());
        }
        Ok(map)
    }

    fn entry(&mut self, hunk: u32) -> Result<MapEntry, String> {
        match &mut self.map {
            Map::Raw(offset) => {
                let mut b = [0u8; 4];
                self.file
                    .seek(SeekFrom::Start(*offset + 4 * hunk as u64))
                    .and_then(|_| self.file.read_exact(&mut b))
                    .map_err(|e| format!("map: {e}"))?;
                let block = u32::from_be_bytes(b) as u64;
                Ok(MapEntry {
                    kind: COMPRESSION_NONE,
                    length: if block == 0 { 0 } else { self.hunk_bytes },
                    offset: block * self.hunk_bytes as u64,
                    crc: 0,
                })
            }
            Map::Compressed(map) => {
                let mut state = map.checkpoints[(hunk / CHECKPOINT) as usize];
                let first = hunk - hunk % CHECKPOINT;
                for h in first..=hunk {
                    let kind = map.next_type(&mut state, &mut self.file)?;
                    let e = map.next_entry(kind, h, self.hunk_bytes, self.unit_bytes, &mut state, &mut self.file)?;
                    if h == hunk {
                        return Ok(e);
                    }
                }
                unreachable!()
            }
        }
    }

    /// The frames per hunk.
    fn frames(&self) -> usize {
        self.hunk_bytes as usize / FRAME_SIZE
    }

    /// Read frame `frame`'s sector data (2352 bytes, as in the CHD: audio
    /// samples big-endian) into `out`. `work` is scratch space of at least
    /// [`WORK_SIZE`] bytes.
    pub fn read_frame(&mut self, frame: u32, out: &mut [u8], work: &mut [u8]) -> Result<(), String> {
        let frames = self.frames() as u32;
        let hunk = frame / frames;
        if hunk >= self.hunk_count {
            out.fill(0);
            return Ok(());
        }
        if self.cached != Some(hunk) {
            self.cached = None;
            self.load_hunk(hunk, work).map_err(|e| format!("hunk {hunk}: {e}"))?;
            self.cached = Some(hunk);
        }
        let at = (frame % frames) as usize * SECTOR_DATA;
        out.copy_from_slice(&self.cache[at..at + SECTOR_DATA]);
        Ok(())
    }

    fn load_hunk(&mut self, hunk: u32, work: &mut [u8]) -> Result<(), String> {
        let hunk_bytes = self.hunk_bytes as usize;
        if work.len() < 2 * hunk_bytes + DECODER_SPACE {
            return Err("work buffer too small".into());
        }
        let mut e = self.entry(hunk)?;
        // A self reference repeats an earlier hunk (which is never one itself
        // as chdman writes them, but follow a chain all the same).
        let mut hops = 0;
        while e.kind == COMPRESSION_SELF {
            hops += 1;
            if hops > 16 {
                return Err("self references do not end".into());
            }
            e = self.entry(e.offset as u32)?;
        }
        let (compressed, rest) = work.split_at_mut(hunk_bytes);
        let (rest, decoder) = rest.split_at_mut(hunk_bytes);
        match e.kind {
            COMPRESSION_NONE => {
                let full = &mut rest[..hunk_bytes];
                if e.length == 0 {
                    full.fill(0);
                } else {
                    self.read_at(e.offset, full)?;
                }
                self.deinterleave(full);
            }
            COMPRESSION_TYPE_0..=COMPRESSION_TYPE_3 => {
                let len = e.length as usize;
                if len > hunk_bytes {
                    return Err(format!("compressed length {len}"));
                }
                self.read_at(e.offset, &mut compressed[..len])?;
                let codec = self.codecs[e.kind as usize];
                self.decompress(codec, &compressed[..len], rest, decoder)
                    .map_err(|err| format!("{}: {err}", codec_name(codec)))?;
            }
            other => return Err(format!("map entry type {other}")),
        }
        if self.codecs[0] != CODEC_NONE {
            let mut crc = Crc16::new();
            for f in 0..self.frames() {
                crc.update(&self.cache[f * SECTOR_DATA..(f + 1) * SECTOR_DATA]);
                crc.update(&self.subcode[f * SUBCODE_DATA..(f + 1) * SUBCODE_DATA]);
            }
            if crc.0 != e.crc {
                return Err("CRC mismatch".into());
            }
        }
        Ok(())
    }

    fn read_at(&mut self, offset: u64, buf: &mut [u8]) -> Result<(), String> {
        self.file
            .seek(SeekFrom::Start(offset))
            .and_then(|_| self.file.read_exact(buf))
            .map_err(|e| format!("read at {offset}: {e}"))
    }

    /// Split a hunk of whole frames into the cache and the subcode.
    fn deinterleave(&mut self, full: &[u8]) {
        for f in 0..self.frames() {
            let frame = &full[f * FRAME_SIZE..(f + 1) * FRAME_SIZE];
            self.cache[f * SECTOR_DATA..(f + 1) * SECTOR_DATA].copy_from_slice(&frame[..SECTOR_DATA]);
            self.subcode[f * SUBCODE_DATA..(f + 1) * SUBCODE_DATA].copy_from_slice(&frame[SECTOR_DATA..]);
        }
    }

    /// Decompress a hunk into the cache and the subcode; `full` is scratch
    /// space of one hunk, `decoder` of [`DECODER_SPACE`] bytes.
    fn decompress(&mut self, codec: u32, src: &[u8], full: &mut [u8], decoder: &mut [u8]) -> Result<(), String> {
        let frames = self.frames();
        match codec {
            CODEC_ZLIB => {
                chd_codec::inflate(src, full)?;
                self.deinterleave(full);
                Ok(())
            }
            CODEC_LZMA => {
                chd_codec::lzma(src, full, lzma_probs(decoder))?;
                self.deinterleave(full);
                Ok(())
            }
            CODEC_CD_FLAC => {
                // SAFETY: any bit pattern is a valid i32.
                let (_, samples, _) = unsafe { decoder.align_to_mut::<i32>() };
                let consumed = chd_codec::flac(src, &mut self.cache, samples)?;
                chd_codec::inflate(&src[consumed..], &mut self.subcode)
            }
            _ => {
                // cdlz, cdzl: a bitmap of the frames whose sync and ECC
                // were dropped, the base's compressed length, the base (the
                // sector data), then the subcode.
                let ecc_bytes = frames.div_ceil(8);
                let length_bytes = if self.hunk_bytes < 65536 { 2 } else { 3 };
                let header = ecc_bytes + length_bytes;
                if src.len() < header {
                    return Err("short hunk".into());
                }
                let base_len =
                    if length_bytes == 2 { be16(&src[ecc_bytes..]) } else { be24(&src[ecc_bytes..]) } as usize;
                if header + base_len > src.len() {
                    return Err("bad base length".into());
                }
                let base = &src[header..header + base_len];
                let sub = &src[header + base_len..];
                if codec == CODEC_CD_LZMA {
                    chd_codec::lzma(base, &mut self.cache, lzma_probs(decoder))?;
                } else {
                    chd_codec::inflate(base, &mut self.cache)?;
                }
                chd_codec::inflate(sub, &mut self.subcode)?;
                for f in 0..frames {
                    if src[f / 8] & (1 << (f % 8)) != 0 {
                        let sector = &mut self.cache[f * SECTOR_DATA..(f + 1) * SECTOR_DATA];
                        sector[..12].copy_from_slice(&SYNC);
                        super::disc::ecc_generate(sector.try_into().unwrap());
                    }
                }
                Ok(())
            }
        }
    }
}

/// The LZMA decoder's probabilities, in the decoder's work space.
fn lzma_probs(decoder: &mut [u8]) -> &mut [u16] {
    // SAFETY: any bit pattern is a valid u16.
    let (_, probs, _) = unsafe { decoder.align_to_mut::<u16>() };
    probs
}
