//! On-demand CD images (`files.json`: `"on_demand": true, "disc": true`).
//!
//! The image (see [`super::disc`]: a `.cue` with its `.bin` files, a lone
//! `.bin`, an `.iso` or a `.chd`) is served to the core as one raw image of
//! 2352-byte sectors, the sector at absolute LBA L (track 1 at 150) at byte
//! offset (L - 150) x 2352, through the framework's `FILE_GET_SIZE` and
//! `FILE_READ`; `DISC_GET_TOC` writes its table of contents
//! ([`super::disc::DiscImage::toc`]) to the core's memory. A core that only
//! knows the official requests reads a single-track image as it is.

use std::ops::DerefMut;

use esp_idf_svc::hal::units::Hertz;

use crate::device::{
    drivers::fpga::{self, SpiCommand, MAX_SPI_READ_CLOCK},
    Device,
};

use super::{command, disc, CoreManager};

/// The absolute LBA of the image's first sector.
const FIRST_LBA: u64 = 150;

// A disc read takes the scratch buffer for a sector (a CHD's work space is
// the image's own).
const _: () = assert!(disc::SECTOR_SIZE <= crate::bitstream::SCRATCH_SIZE);

impl CoreManager {
    /// Whether the core's file `file_id` is an on-demand disc.
    pub(super) fn is_disc_file(&self, file_id: u16) -> bool {
        self.core_info
            .as_ref()
            .is_some_and(|core| core.files.iter().any(|f| f.id == file_id && f.on_demand && f.disc))
    }

    /// The disc in file `file_id`, opened on its first use.
    fn disc_image(&mut self, file_id: u16) -> Result<&mut disc::DiscImage, u32> {
        if self.open_disc.as_ref().is_some_and(|(id, _)| *id == file_id) {
            return Ok(&mut self.open_disc.as_mut().unwrap().1);
        }
        let core = self.core_info.as_ref().unwrap();
        let Some(i) = core.files.iter().position(|f| f.id == file_id) else {
            return Err(command::FILE_STATUS_ERR_INVALID_FILE);
        };
        let Some(path) = self.selected_files[i].as_ref() else {
            return Err(command::FILE_STATUS_ERR_INVALID_FILE);
        };
        let image = disc::DiscImage::open(path).map_err(|e| {
            log::error!("Disc {}: {e}", path.display());
            crate::ui::send(crate::ui::Message::Notification(crate::ui::Notification::new_long(
                "Cannot open the disc image".to_string(),
            )));
            command::FILE_STATUS_ERR_IO
        })?;
        image.log();
        Ok(&mut self.open_disc.insert((file_id, image)).1)
    }

    fn disc_transfer_speed(&self, file_id: u16) -> u32 {
        self.core_info
            .as_ref()
            .and_then(|core| core.files.iter().find(|f| f.id == file_id))
            .map_or(5000, |f| f.max_transfer_speed)
    }

    /// `FILE_GET_SIZE` of a disc: its raw image's size.
    pub(super) fn handle_disc_get_size(&mut self, file_id: u16, result: &mut [u32]) -> bool {
        match self.disc_image(file_id) {
            Ok(image) => {
                let size = (image.lead_out() as u64).saturating_sub(FIRST_LBA) * disc::SECTOR_SIZE as u64;
                result[0] = command::FILE_STATUS_OK;
                result[1] = size as u32;
                result[2] = (size >> 32) as u32;
                true
            }
            Err(status) => {
                result[0] = status;
                false
            }
        }
    }

    /// `FILE_READ` of a disc: bytes of its raw image, sector by sector.
    pub(super) fn handle_disc_read(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        let address = params[1];
        let offset = ((params[3] as u64) << 32) | (params[2] as u64);
        let length = params[4] as usize;
        result[0] = command::FILE_STATUS_OK;
        result[1] = 0;

        let write_clock = Hertz(self.disc_transfer_speed(file_id) * 1000 * 2);
        let image = match self.disc_image(file_id) {
            Ok(image) => image,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };
        let size = (image.lead_out() as u64).saturating_sub(FIRST_LBA) * disc::SECTOR_SIZE as u64;
        let end = (offset + length as u64).min(size);

        // The scratch buffer (internal RAM): a sector.
        let Some(mut scratch) = crate::bitstream::SCRATCH.take() else {
            log::warn!("Disc read: scratch buffer busy");
            result[0] = command::FILE_STATUS_ERR_IO;
            return false;
        };
        let sector: &mut [u8; disc::SECTOR_SIZE] =
            (&mut scratch.deref_mut()[..disc::SECTOR_SIZE]).try_into().unwrap();
        let command = SpiCommand::new(fpga::FpgaSpiWordSize::Bits32);

        let mut pos = offset;
        while pos < end {
            let lba = FIRST_LBA + pos / disc::SECTOR_SIZE as u64;
            let within = (pos % disc::SECTOR_SIZE as u64) as usize;
            let n = (disc::SECTOR_SIZE - within).min((end - pos) as usize);
            if let Err(e) = image.read_sector(lba as u32, sector) {
                log::error!("Disc read at {lba}: {e}");
                result[0] = command::FILE_STATUS_ERR_IO;
                result[1] = (pos - offset) as u32;
                return false;
            }
            let at = address + (pos - offset) as u32;
            if Device::lock()
                .fpga
                .spi_write(Some(write_clock), command, at, &sector[within..within + n])
                .is_err()
            {
                result[0] = command::FILE_STATUS_ERR_IO;
                result[1] = (pos - offset) as u32;
                return false;
            }
            pos += n as u64;
        }
        result[1] = (pos - offset) as u32;
        true
    }

    /// `DISC_GET_TOC` (file, address): the disc's table of contents
    /// ([`disc::DiscImage::toc`]) to the core's memory.
    pub(super) fn handle_disc_get_toc(&mut self, params: &[u32], result: &mut [u32]) -> bool {
        let file_id = params[0] as u16;
        let address = params[1];
        if !self.is_disc_file(file_id) {
            result[0] = command::FILE_STATUS_ERR_INVALID_FILE;
            return false;
        }
        let image = match self.disc_image(file_id) {
            Ok(image) => image,
            Err(status) => {
                result[0] = status;
                return false;
            }
        };
        let toc = image.toc();
        let ok = Device::lock()
            .fpga
            .spi_write(
                Some(MAX_SPI_READ_CLOCK),
                SpiCommand::new(fpga::FpgaSpiWordSize::Bits32),
                address,
                &toc,
            )
            .is_ok();
        result[0] = if ok { command::FILE_STATUS_OK } else { command::FILE_STATUS_ERR_IO };
        ok
    }
}
