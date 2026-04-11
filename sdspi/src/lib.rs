//! A crate for interfacing with SD cards over SPI.

#![no_std]

use aligned::Aligned;
use core::fmt::Debug;
use core::future::Future;
use core::marker::PhantomData;
use embassy_futures::select::{select, Either};
use embassy_futures::yield_now;
use sdio_host::sd::{CardCapacity, CID, CSD, OCR, SD};
use sdio_host::{common_cmd::*, sd_cmd::*};

// MUST be the first module listed
mod fmt;

/// Status for card in the ready state
pub const R1_READY_STATE: u8 = 0x00;
/// Status for card in the idle state
pub const R1_IDLE_STATE: u8 = 0x01;
/// Status bit for illegal command
pub const R1_ILLEGAL_COMMAND: u8 = 0x04;
/// Start data token for read or write single block*/
pub const DATA_START_BLOCK: u8 = 0xFE;
/// Stop token for write multiple blocks*/
pub const STOP_TRAN_TOKEN: u8 = 0xFD;
/// Start data token for write multiple blocks*/
pub const WRITE_MULTIPLE_TOKEN: u8 = 0xFC;
/// Mask for data response tokens after a write block operation
pub const DATA_RES_MASK: u8 = 0x1F;
/// Write data accepted token
pub const DATA_RES_ACCEPTED: u8 = 0x05;

#[derive(Clone, Copy, Debug, Default)]
/// SD Card
pub struct Card {
    /// The type of this card
    pub card_type: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Relative Card Address
    pub rca: u32,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
}

impl Card {
    /// Size in bytes
    pub fn size(&self) -> u64 {
        // SDHC / SDXC / SDUC
        u64::from(self.csd.block_count()) * 512
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    ChipSelect,
    SpiError,
    Timeout,
    UnsupportedCard,
    Cmd58Error,
    Cmd59Error,
    RegisterError(u8),
    CrcMismatch(u16, u16),
    NotInitialized,
    WriteError,
}

/// Must be called between powerup and [SdSpi::init] to ensure the sdcard is properly initialized.
pub async fn sd_init<SPI, CS, BE>(spi: &mut SPI, cs: &mut CS) -> Result<(), Error>
where
    SPI: embedded_hal_async::spi::SpiBus<Error = BE>,
    CS: embedded_hal::digital::OutputPin,
{
    // Supply minimum of 74 clock cycles without CS asserted.
    cs.set_high().map_err(|_| Error::ChipSelect)?;
    // Try flushing the card as done here: https://github.com/greiman/SdFat/blob/master/src/SdCard/SdSpiCard.cpp#L170,
    // https://github.com/rust-embedded-community/embedded-sdmmc-rs/pull/65#issuecomment-1270709448
    spi.write(&[0xFF; 256]).await.map_err(|_| Error::SpiError)?;

    Ok(())
}

pub struct SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs,
    ALIGN: aligned::Alignment,
{
    spi: SPI,
    delay: D,
    card: Option<Card>,
    _align: PhantomData<ALIGN>,
}

impl<SPI, D, ALIGN> SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    pub fn new(spi: SPI, delay: D) -> Self {
        Self {
            spi,
            delay,
            card: None,
            _align: PhantomData,
        }
    }

    /// To comply with the SD card spec, [sd_init] must be called between powerup and calling this function.
    pub async fn init(&mut self) -> Result<(), Error> {
        let r = async {
            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(idle()).await?;
                    if r == R1_IDLE_STATE {
                        return Ok(());
                    }
                }
            })
            .await??;

            // "The SPI interface is initialized in the CRC OFF mode in default"
            // -- SD Part 1 Physical Layer Specification v9.00, Section 7.2.2 Bus Transfer Protection
            if self.cmd(cmd::<R1>(0x3B, 1)).await? != R1_IDLE_STATE {
                return Err(Error::Cmd59Error);
            }

            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(send_if_cond(0x1, 0xAA)).await?;
                    if r == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                        return Err(Error::UnsupportedCard);
                    }
                    let mut buffer = [0xFFu8; 4];
                    self.spi
                        .transfer_in_place(&mut buffer[..])
                        .await
                        .map_err(|_| Error::SpiError)?;
                    if buffer[3] == 0xAA {
                        return Ok(());
                    }
                }
            })
            .await??;

            trace!("Valid card detected!");

            // If we get here we're at least a v2 card
            let mut card = Card::default();

            // send ACMD41
            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.acmd(sd_send_op_cond(true, false, true, 0x20)).await?;
                    if r == R1_READY_STATE {
                        return Ok(());
                    }
                }
            })
            .await??;

            trace!("send_ocr");
            card.ocr = with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(cmd::<R3>(0x3A, 0)).await?;
                    if r != R1_READY_STATE {
                        return Err(Error::Cmd58Error);
                    }
                    let mut buffer = [0xFFu8; 4];
                    self.spi
                        .transfer_in_place(&mut buffer[..])
                        .await
                        .map_err(|_| Error::SpiError)?;
                    let ocr: OCR<SD> = u32::from_be_bytes(buffer).into();
                    if !ocr.is_busy() {
                        return Ok(ocr);
                    }
                }
            })
            .await??;

            trace!("send_csd");
            let r = self.cmd(send_csd(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut csd = [0xFFu8; 16];
            self.read_data(&mut csd).await?;
            card.csd = u128::from_be_bytes(csd).into();

            trace!("all_send_cid");
            let r = self.cmd(send_cid(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut cid = [0xFFu8; 16];
            self.read_data(&mut cid).await?;
            card.cid = u128::from_be_bytes(cid).into();

            debug!("Found card with size: {}bytes", card.size());

            self.card = Some(card);

            Ok(())
        }
        .await;

        r
    }

    pub async fn read<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        let r = async {
            if data.len() == 1 {
                self.cmd(read_single_block(block_address)).await?;
                self.read_data(&mut data[0][..]).await?;
            } else {
                self.cmd(read_multiple_blocks(block_address)).await?;
                for block in data {
                    self.read_data(&mut block[..]).await?;
                }
                self.cmd(stop_transmission()).await?;
            }
            Ok(())
        }
        .await;

        r?;

        Ok(())
    }

    pub async fn write<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        let n = data.len();
        let r = async {
            if n == 1 {
                self.cmd(write_single_block(block_address)).await.map_err(|e| {
                    error!("sdspi::write[single] CMD24 @ {}: {:?}", block_address, e);
                    e
                })?;
                self.write_data(DATA_START_BLOCK, &data[0][..]).await.map_err(|e| {
                    error!("sdspi::write[single] write_data @ {}: {:?}", block_address, e);
                    e
                })?;
                self.wait_idle().await.map_err(|e| {
                    error!("sdspi::write[single] wait_idle @ {}: {:?}", block_address, e);
                    e
                })?;
                // check status, in SD SPI mode, the status is two bytes
                let s1 = self.cmd(sd_status()).await.map_err(|e| {
                    error!("sdspi::write[single] sd_status cmd @ {}: {:?}", block_address, e);
                    e
                })?;
                if s1 != 0 {
                    error!("sdspi::write[single] sd_status R1 nonzero @ {}: 0x{:02x}", block_address, s1);
                    return Err(Error::WriteError);
                }
                let s2 = self.read_byte().await.map_err(|e| {
                    error!("sdspi::write[single] sd_status byte2 @ {}: {:?}", block_address, e);
                    e
                })?;
                if s2 != 0 {
                    error!("sdspi::write[single] sd_status byte2 nonzero @ {}: 0x{:02x}", block_address, s2);
                    return Err(Error::WriteError);
                }
            } else {
                // Try sending ACMD23 _before_ write.
                // This will pre-erase blocks to improve write performance.
                // We ignore the return value, because whether its accepted
                // or not doesn't matter we will still proceed with the write
                self.acmd(cmd::<R1>(0x17, n as u32)).await.map_err(|e| {
                    error!("sdspi::write[multi] ACMD23 @ {} n={}: {:?}", block_address, n, e);
                    e
                })?;
                self.wait_idle().await.map_err(|e| {
                    error!("sdspi::write[multi] wait_idle post-ACMD23 @ {}: {:?}", block_address, e);
                    e
                })?;

                let r1 = self.cmd(write_multiple_blocks(block_address)).await.map_err(|e| {
                    error!("sdspi::write[multi] CMD25 @ {} n={}: {:?}", block_address, n, e);
                    e
                })?;
                if r1 != 0 {
                    error!("sdspi::write[multi] CMD25 R1 nonzero @ {} n={}: 0x{:02x}", block_address, n, r1);
                    return Err(Error::RegisterError(r1));
                }
                for (i, block) in data.iter().enumerate() {
                    self.wait_idle().await.map_err(|e| {
                        error!("sdspi::write[multi] wait_idle pre-block {} @ {}: {:?}", i, block_address, e);
                        e
                    })?;
                    self.write_data(WRITE_MULTIPLE_TOKEN, &block[..]).await.map_err(|e| {
                        error!("sdspi::write[multi] write_data block {} @ {}: {:?}", i, block_address, e);
                        e
                    })?;
                }
                // stop the write
                self.wait_idle().await.map_err(|e| {
                    error!("sdspi::write[multi] wait_idle pre-STOP @ {}: {:?}", block_address, e);
                    e
                })?;
                self.spi.write(&[STOP_TRAN_TOKEN]).await.map_err(|_| {
                    error!("sdspi::write[multi] STOP_TRAN spi error @ {}", block_address);
                    Error::SpiError
                })?;
            }
            Ok(())
        }
        .await;

        r?;

        Ok(())
    }

    pub async fn size(&mut self) -> Result<u64, Error> {
        Ok(self.card.ok_or(Error::NotInitialized)?.size())
    }

    async fn read_data(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        // Same rationale as `wait_idle` — yield between polls while
        // waiting for the data-start token so other async tasks on
        // the same executor are not starved.
        let outer = with_timeout(self.delay.clone(), 1000, async {
            let mut byte = self.read_byte().await?;
            while byte == 0xFF {
                yield_now().await;
                byte = self.read_byte().await?;
            }
            Ok(byte)
        })
        .await;
        if let Err(Error::Timeout) = outer {
            error!("sdspi: read_data data-start token wait timed out after 1000 ms");
        }
        let r = outer??;

        if r != DATA_START_BLOCK {
            return Err(Error::RegisterError(r));
        }

        buffer.fill(0xFF);
        self.spi
            .transfer_in_place(buffer)
            .await
            .map_err(|_| Error::SpiError)?;

        let mut crc_bytes = [0xFF; 2];
        self.spi
            .transfer_in_place(&mut crc_bytes)
            .await
            .map_err(|_| Error::SpiError)?;
        let crc = u16::from_be_bytes(crc_bytes);
        let calc_crc = crc16(buffer);
        if crc != calc_crc {
            return Err(Error::CrcMismatch(crc, calc_crc));
        }

        Ok(())
    }

    async fn write_data(&mut self, token: u8, buffer: &[u8]) -> Result<(), Error> {
        self.spi
            .write(&[token])
            .await
            .map_err(|_| Error::SpiError)?;
        self.spi.write(buffer).await.map_err(|_| Error::SpiError)?;
        let crc_bytes = crc16(buffer).to_be_bytes();
        self.spi
            .write(&crc_bytes)
            .await
            .map_err(|_| Error::SpiError)?;

        let status = self.read_byte().await?;
        if (status & DATA_RES_MASK) != DATA_RES_ACCEPTED {
            return Err(Error::WriteError);
        }

        Ok(())
    }

    pub fn spi(&mut self) -> &mut SPI {
        &mut self.spi
    }

    async fn cmd<R: Resp>(&mut self, cmd: Cmd<R>) -> Result<u8, Error> {
        if cmd.cmd != idle().cmd {
            self.wait_idle().await?;
        }

        let mut buf = [
            0x40 | cmd.cmd,
            (cmd.arg >> 24) as u8,
            (cmd.arg >> 16) as u8,
            (cmd.arg >> 8) as u8,
            cmd.arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        self.spi.write(&buf).await.map_err(|_| Error::SpiError)?;

        // skip stuff byte for stop read
        if cmd.cmd == stop_transmission().cmd {
            self.spi
                .transfer_in_place(&mut [0xFF])
                .await
                .map_err(|_| Error::SpiError)?;
        }

        // Use a mutable counter so a timeout report tells us whether
        // the poll loop actually ran for the full window or was
        // scheduler-starved. For the card to legitimately not
        // respond, `polls` should be large.
        let mut polls: u32 = 0;
        // 10 s: the SD spec allows the card to defer command
        // acceptance during background work (wear-leveling, GC).
        // 1 s proved too tight under sustained write pressure during
        // `format_volume` — CMD24 would time out after ~200 s of
        // continuous writes even though the card was still alive.
        let outer = with_timeout(self.delay.clone(), 10_000, async {
            loop {
                let byte = self.read_byte().await?;
                polls += 1;
                if byte & 0x80 == 0 {
                    return Ok(byte);
                }
            }
        })
        .await;
        if let Err(Error::Timeout) = outer {
            error!(
                "sdspi: cmd {} response wait timed out after 10 s ({} polls)",
                cmd.cmd, polls
            );
        }
        let byte = outer??;

        Ok(byte)
    }

    async fn acmd<R: Resp>(&mut self, cmd: Cmd<R>) -> Result<u8, Error> {
        self.cmd(app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16))
            .await?;
        self.cmd(cmd).await
    }

    async fn wait_idle(&mut self) -> Result<(), Error> {
        // The card can hold the bus busy for 100+ ms during a
        // write-programming cycle. Polling without `yield_now().await`
        // starves every other async task on the same executor and, on
        // shared-bus configurations, continuously reacquires the SPI
        // bus mutex, blocking display/radio work entirely. Yielding
        // once per poll lets the scheduler service other tasks between
        // checks at the cost of one scheduler round-trip per iteration.
        let outer = with_timeout(self.delay.clone(), 5000, async {
            while self.read_byte().await? != 0xFF {
                yield_now().await;
            }
            Ok(())
        })
        .await;
        if let Err(Error::Timeout) = outer {
            error!("sdspi: wait_idle card-busy wait timed out after 5000 ms");
        }
        outer?
    }

    async fn read_byte(&mut self) -> Result<u8, Error> {
        let mut buf = [0xFFu8; 1];
        self.spi
            .transfer_in_place(&mut buf[..])
            .await
            .map_err(|_| Error::SpiError)?;

        Ok(buf[0])
    }
}

impl<SPI, D, ALIGN, const SIZE: usize> block_device_driver::BlockDevice<SIZE>
    for SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    type Error = Error;
    type Align = ALIGN;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Self::Error> {
        self.read(block_address, data).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Self::Error> {
        self.write(block_address, data).await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.size().await
    }
}

async fn with_timeout<D: embedded_hal_async::delay::DelayNs, F: Future>(
    mut delay: D,
    timeout: u32,
    fut: F,
) -> Result<F::Output, Error> {
    match select(fut, delay.delay_ms(timeout)).await {
        Either::First(r) => Ok(r),
        Either::Second(_) => Err(Error::Timeout),
    }
}

/// Perform the 7-bit CRC used on the SD card
fn crc7(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut d in data.iter().cloned() {
        for _bit in 0..8 {
            crc <<= 1;
            if ((d & 0x80) ^ (crc & 0x80)) != 0 {
                crc ^= 0x09;
            }
            d <<= 1;
        }
    }
    (crc << 1) | 1
}

/// Perform the X25 CRC calculation, as used for data blocks.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in data {
        crc = ((crc >> 8) & 0xFF) | (crc << 8);
        crc ^= u16::from(byte);
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    crc
}
