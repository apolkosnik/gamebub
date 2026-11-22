use std::{
    fs::File,
    io::{Read, Seek, SeekFrom, Write},
};

use esp_idf_svc::hal::units::Hertz;

use crate::device::{
    drivers::fpga::{SpiCommand, MAX_SPI_READ_CLOCK},
    Device,
};

use super::{command, CoreManager};

/// Where the time of the on-demand reads goes (a test build's counters:
/// `crate::net` shows them and writes them to the card when a Minimig core
/// ends): the requests and their bytes, the microseconds in the card (seek
/// and read), in the SPI writes to the core, and between one request's end
/// and the next one's start when that is under 200 ms (the core taking the
/// data and asking again, and the way here).
pub mod stats {
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Mutex;
    use std::time::Instant;

    pub static READS: AtomicU32 = AtomicU32::new(0);
    pub static BYTES: Mutex<u64> = Mutex::new(0);
    pub static CARD_US: Mutex<u64> = Mutex::new(0);
    pub static SPI_US: Mutex<u64> = Mutex::new(0);
    pub static GAP_US: Mutex<u64> = Mutex::new(0);
    pub static GAPS: AtomicU32 = AtomicU32::new(0);
    pub static LAST_END: Mutex<Option<Instant>> = Mutex::new(None);

    pub fn add(counter: &Mutex<u64>, since: Instant) {
        *counter.lock().unwrap() += since.elapsed().as_micros() as u64;
    }

    /// The counters as a line, and all of them cleared; None without a read.
    pub fn take_line() -> Option<String> {
        let reads = READS.swap(0, Ordering::Relaxed) as u64;
        let bytes = std::mem::take(&mut *BYTES.lock().unwrap());
        let gaps = GAPS.swap(0, Ordering::Relaxed) as u64;
        let card = std::mem::take(&mut *CARD_US.lock().unwrap());
        let spi = std::mem::take(&mut *SPI_US.lock().unwrap());
        let gap = std::mem::take(&mut *GAP_US.lock().unwrap());
        *LAST_END.lock().unwrap() = None;
        if reads == 0 {
            return None;
        }
        Some(format!(
            "Disk: {} reads, {} KB; per read: card {} us, SPI {} us, between reads {} us ({} of them)",
            reads,
            bytes / 1024,
            card / reads,
            spi / reads,
            if gaps != 0 { gap / gaps } else { 0 },
            gaps
        ))
    }
}

/// The bytes of one SPI write of an on-demand read: the lock on the device
/// is given up between two, so that what else goes to the core (a dock
/// mouse's movement) does not wait for a whole request.
const SPI_PIECE: usize = 4 * 1024;

pub struct OnDemandFile {
    pub id: u16,
    pub file: File,

    // Cached info about the file
    read_only: bool,
    exact_size: u32,
    max_size: u32,
    max_transfer_speed: u32,
}

impl CoreManager {
    fn ensure_file_open(&mut self, file_id: u16) -> Result<&mut OnDemandFile, u32> {
        if self
            .open_on_demand_file
            .as_ref()
            .is_some_and(|f| f.id == file_id)
        {
            return Ok(self.open_on_demand_file.as_mut().unwrap());
        }

        // Look up the file
        let info = self
            .core_info
            .as_ref()
            .unwrap()
            .files
            .iter()
            .enumerate()
            .find(|f| f.1.id == file_id);
        let Some((i, info)) = info else {
            log::warn!("Core accessed invalid file id {file_id}");
            return Err(command::FILE_STATUS_ERR_INVALID_FILE);
        };
        if !info.on_demand {
            return Err(command::FILE_STATUS_ERR_INVALID_FILE);
        }
        let Some(path) = self.selected_files[i].as_ref() else {
            // No file selected.
            return Err(command::FILE_STATUS_ERR_INVALID_FILE);
        };

        // Open the file.
        self.open_on_demand_file = None;

        let file = if info.read_only {
            File::open(path)
        } else {
            std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(false)
                .open(path)
        };

        let file = file.map_err(|e| {
            log::error!("Failed to open file for core: id={}: {}", file_id, e);
            command::FILE_STATUS_ERR_IO
        })?;

        Ok(self.open_on_demand_file.insert(OnDemandFile {
            id: file_id,
            file,
            read_only: info.read_only,
            exact_size: info.exact_size,
            max_size: info.max_size,
            max_transfer_speed: info.max_transfer_speed,
        }))
    }

    pub(super) fn handle_file_get_size(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        if self.is_disc_file(file_id) {
            return self.handle_disc_get_size(file_id, result);
        }
        let file = match self.ensure_file_open(file_id) {
            Ok(file) => file,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };

        match file.file.metadata() {
            Ok(metadata) => {
                let size = metadata.len();
                result[0] = command::FILE_STATUS_OK;
                result[1] = size as u32;
                result[2] = (size >> 32) as u32;
                true
            }
            Err(_) => {
                result[0] = command::FILE_STATUS_ERR_IO;
                false
            }
        }
    }

    pub(super) fn handle_file_set_size(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        let new_size = ((params[2] as u64) << 32) | (params[1] as u64);

        let file = match self.ensure_file_open(file_id) {
            Ok(file) => file,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };

        if file.read_only {
            result[0] = command::FILE_STATUS_ERR_INVALID_FILE;
            return false;
        }
        if file.max_size != 0 && new_size > file.max_size as u64 {
            result[0] = command::FILE_STATUS_ERR_BOUNDS;
            return false;
        }
        if file.exact_size != 0 && new_size != file.exact_size as u64 {
            result[0] = command::FILE_STATUS_ERR_BOUNDS;
            return false;
        }

        match file.file.set_len(new_size) {
            Ok(_) => {
                result[0] = command::FILE_STATUS_OK;
                true
            }
            Err(_) => {
                result[0] = command::FILE_STATUS_ERR_IO;
                false
            }
        }
    }

    pub(super) fn handle_file_read(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        if self.is_disc_file(file_id) {
            return self.handle_disc_read(params, result);
        }
        let address = params[1];
        let offset = ((params[3] as u64) << 32) | (params[2] as u64);
        let length = params[4] as usize;

        result[0] = command::FILE_STATUS_OK;
        result[1] = 0;

        let file = match self.ensure_file_open(file_id) {
            Ok(file) => file,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };

        let started = std::time::Instant::now();
        if let Some(end) = *stats::LAST_END.lock().unwrap() {
            let gap = started.duration_since(end).as_micros() as u64;
            if gap < 200_000 {
                *stats::GAP_US.lock().unwrap() += gap;
                stats::GAPS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
        }
        if file.file.seek(SeekFrom::Start(offset)).is_err() {
            result[0] = command::FILE_STATUS_ERR_IO;
            return false;
        }

        let mut scratch = crate::bitstream::SCRATCH.take().expect("scratch buffer");
        let spi_max_clock = Some(Hertz(file.max_transfer_speed * 1000 * 2));
        let spi_command = SpiCommand::new(crate::fpga::FpgaSpiWordSize::Bits32);

        let mut transferred: usize = 0;
        while transferred < length {
            let chunk = (length - transferred).min(scratch.len());
            let card = std::time::Instant::now();
            let read = file.file.read(&mut scratch[..chunk]);
            stats::add(&stats::CARD_US, card);
            let n = match read {
                Ok(0) => break, // Reached EOF naturally
                Ok(n) => n,
                Err(_) => {
                    result[0] = command::FILE_STATUS_ERR_IO;
                    result[1] = transferred as u32;
                    return false;
                }
            };

            let spi = std::time::Instant::now();
            let mut done = 0;
            for piece in scratch[..n].chunks(SPI_PIECE) {
                Device::lock()
                    .fpga
                    .spi_write(
                        spi_max_clock,
                        spi_command,
                        address + (transferred + done) as u32,
                        piece,
                    )
                    .unwrap();
                done += piece.len();
            }
            stats::add(&stats::SPI_US, spi);
            transferred += n;
        }

        stats::READS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        *stats::BYTES.lock().unwrap() += transferred as u64;
        *stats::LAST_END.lock().unwrap() = Some(std::time::Instant::now());
        result[1] = transferred as u32;
        true
    }

    pub(super) fn handle_file_write(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        let address = params[1];
        let offset = ((params[3] as u64) << 32) | (params[2] as u64);
        let length = params[4] as usize;

        result[0] = command::FILE_STATUS_OK;
        result[1] = 0;

        let file = match self.ensure_file_open(file_id) {
            Ok(file) => file,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };

        if file.read_only {
            result[0] = command::FILE_STATUS_ERR_INVALID_FILE;
            return false;
        }

        let end = offset + length as u64;
        if (file.max_size != 0 && end > file.max_size as u64)
            || (file.exact_size != 0 && end > file.exact_size as u64)
        {
            result[0] = command::FILE_STATUS_ERR_BOUNDS;
            return false;
        }

        if file.file.seek(SeekFrom::Start(offset)).is_err() {
            result[0] = command::FILE_STATUS_ERR_IO;
            return false;
        }

        let mut scratch = crate::bitstream::SCRATCH.take().expect("scratch buffer");
        let spi_max_clock = Some(Hertz(file.max_transfer_speed * 1000 * 2).min(MAX_SPI_READ_CLOCK));
        let spi_command = SpiCommand::new(crate::fpga::FpgaSpiWordSize::Bits32);

        let mut transferred: usize = 0;
        while transferred < length {
            let chunk = (length - transferred).min(scratch.len());
            let _ = Device::lock().fpga.spi_read(
                spi_max_clock,
                spi_command,
                address + transferred as u32,
                &mut scratch[..chunk],
            );

            if file.file.write_all(&scratch[..chunk]).is_err() {
                result[0] = command::FILE_STATUS_ERR_IO;
                result[1] = transferred as u32;
                return false;
            }

            transferred += chunk;
        }

        result[1] = transferred as u32;
        true
    }

    pub(super) fn handle_file_flush(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        let file = match self.ensure_file_open(file_id) {
            Ok(file) => file,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };

        match file.file.sync_all() {
            Ok(_) => {
                result[0] = command::FILE_STATUS_OK;
                true
            }
            Err(e) => {
                log::error!("Error syncing file id={file_id}: {e}");
                result[0] = command::FILE_STATUS_ERR_IO;
                false
            }
        }
    }
}
