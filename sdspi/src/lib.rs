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
    EraseError,
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
        let n = data.len();
        let r = async {
            if n == 1 {
                self.cmd(read_single_block(block_address)).await.map_err(|e| {
                    error!("sdspi::read[single] CMD17 @ {}: {:?}", block_address, e);
                    e
                })?;
                self.read_data(&mut data[0][..]).await.map_err(|e| {
                    error!("sdspi::read[single] read_data @ {}: {:?}", block_address, e);
                    e
                })?;
            } else {
                self.cmd(read_multiple_blocks(block_address)).await.map_err(|e| {
                    error!("sdspi::read[multi] CMD18 @ {} n={}: {:?}", block_address, n, e);
                    e
                })?;
                for (i, block) in data.iter_mut().enumerate() {
                    self.read_data(&mut block[..]).await.map_err(|e| {
                        error!("sdspi::read[multi] read_data block {} @ {}: {:?}", i, block_address, e);
                        e
                    })?;
                }
                self.cmd(stop_transmission()).await.map_err(|e| {
                    error!("sdspi::read[multi] CMD12 @ {}: {:?}", block_address, e);
                    e
                })?;
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
                // NOTE: write[multi] has no analogous CMD13 (sd_status)
                // post-check, and a CMD13 here on fast SPI hosts
                // (ESP32-S3 GDMA observed) intermittently times out
                // with R1 not appearing in the NCR window after the
                // card's programming cycle — symptom is identical to
                // the CMD24-after-CMD38 issue but per-write. We rely
                // on `write_data`'s DATA_RES_ACCEPTED response (host-
                // side ACK) to detect write rejection; the additional
                // card-side status check this CMD13 provided was
                // never propagated meaningfully to upper layers.
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

    /// Erase blocks in the range `[start_block, end_block]` (inclusive).
    ///
    /// Sends CMD32 (ERASE_WR_BLK_START), CMD33 (ERASE_WR_BLK_END),
    /// CMD38 (ERASE). The card performs the erase internally — orders
    /// of magnitude faster than writing zeros over SPI. After erase,
    /// blocks contain 0x00 or 0xFF depending on the card.
    pub async fn erase(&mut self, start_block: u32, end_block: u32) -> Result<(), Error> {
        self.card.ok_or(Error::NotInitialized)?;

        let r = self.cmd(cmd::<R1>(32, start_block)).await?;
        if r != R1_READY_STATE {
            error!("sdspi::erase CMD32 R1=0x{:02x}", r);
            return Err(Error::EraseError);
        }

        let r = self.cmd(cmd::<R1>(33, end_block)).await?;
        if r != R1_READY_STATE {
            error!("sdspi::erase CMD33 R1=0x{:02x}", r);
            return Err(Error::EraseError);
        }

        let r = self.cmd(cmd::<R1>(38, 0)).await?;
        if r != R1_READY_STATE {
            error!("sdspi::erase CMD38 R1=0x{:02x}", r);
            return Err(Error::EraseError);
        }

        // CMD38 returns R1b — card holds busy until erase completes.
        // 60 s sanity bound is large because the SD spec puts no fixed
        // upper bound on full-card CMD38 (function of capacity × per-AU
        // erase time).
        self.wait_idle_with_timeout_ms(60_000).await?;

        // Post-CMD38 sustained-idle. Some host/card combinations show a
        // brief false-idle right after CMD38 (MISO goes 0xFF transiently
        // while the card is still doing post-erase housekeeping). Without
        // a sustained check, the next CMD24 lands on a busy card whose
        // R1 never arrives, and the format times out.
        //
        // FEATURE GATING: the cores3 + 16 GB SDHC card (ESP32-S3 GDMA)
        // *requires* this guard — 0/10 format failure without it. fire27
        // (ESP32 PDMA + shared-bus display + BLE controller) cannot
        // tolerate the cumulative SPI-bus-mutex hold across thousands
        // of probe transactions: the level-1 RWBLE interrupt gets
        // masked for ms-scale windows, the BLE controller blob de-syncs,
        // and the embassy executor eventually stops polling tasks
        // (visible as LVGL FPS dropping to zero mid-format and the
        // chip becoming unresponsive to all serial input). fire27's
        // 8-byte NCR fast-path in `cmd()` is sufficient on its own.
        //
        // Targets opt in by enabling the `post-erase-sustained-idle`
        // feature on the sdspi dependency. fire27 leaves it off; cores3
        // enables it.
        //
        // sustained_count = 2 000 (≈ 2 s of confirmed continuous idle):
        // empirically required for cores3 + 8 GB SDHC card combo
        // (200 ms is insufficient — sustained exits OK but the next
        // CMD24 still finds the card busy at the protocol level and
        // times out at 10 s). Conservative; small fast cards exit
        // promptly because any 0x00 byte in a probe resets the counter
        // and they reach 2 s of idle quickly.
        #[cfg(feature = "post-erase-sustained-idle")]
        self.wait_idle_sustained_ms(60_000, 2_000).await?;

        Ok(())
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

        // Read data block + 2 CRC bytes in one SpiDevice transaction
        // so CS stays asserted for the entire data phase.
        buffer.fill(0xFF);
        let mut crc_bytes = [0xFFu8; 2];
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::TransferInPlace(buffer),
                Operation::TransferInPlace(&mut crc_bytes),
            ])
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
        // Send token + data + CRC + read data-response window as one
        // SpiDevice transaction. CS must stay asserted across the
        // whole write-data phase per the SD SPI spec.
        //
        // The data-response token is supposed to arrive immediately
        // after CRC ("with no delay" per spec) but on fast SPI hosts
        // (cores3 GDMA observed) and during sustained multi-block
        // writes, the card occasionally needs a few additional byte
        // clocks before driving the response — symptom: 1-byte read
        // returns 0xFF and the host rejects the (actually OK) write
        // with WriteError. Mirror the cmd() fast-path: clock 8 bytes
        // in the same transaction and scan for the first non-0xFF byte
        // (the response token).
        let crc_bytes = crc16(buffer).to_be_bytes();
        let token_buf = [token];
        let mut status_buf = [0xFFu8; 8];
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&token_buf),
                Operation::Write(buffer),
                Operation::Write(&crc_bytes),
                Operation::TransferInPlace(&mut status_buf),
            ])
            .await
            .map_err(|_| Error::SpiError)?;

        for &b in &status_buf {
            if b != 0xFF {
                if (b & DATA_RES_MASK) != DATA_RES_ACCEPTED {
                    error!(
                        "sdspi: write_data rejected, status=0x{:02x} ({})",
                        b,
                        match b & DATA_RES_MASK {
                            0x0B => "CRC error",
                            0x0D => "write/program error",
                            _ => "unknown",
                        }
                    );
                    return Err(Error::WriteError);
                }
                return Ok(());
            }
        }
        // No response in 8 bytes — card violated spec / lost sync.
        error!("sdspi: write_data no data-response token in 8-byte window");
        Err(Error::WriteError)
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

        // Send command + read the full SD-spec NCR window in a single
        // transaction so CS stays asserted across the command-to-response
        // boundary. The SD spec puts NCR (host-clock-to-R1 latency) at
        // 1-8 bytes; a spec-compliant card MUST respond within that
        // window, so an 8-byte read here catches R1 in a single
        // CS-asserted transaction for any compliant card.
        //
        // CRITICAL on fast SPI hosts (ESP32-S3 GDMA): a previous
        // implementation read only 1 byte in the fast path and fell
        // back to per-byte slow-path polls (each its own SpiDevice
        // transaction → CS toggled between bytes). On fast hosts the
        // MISO pull-up wins for several µs after CS reasserts, so the
        // card's R1 — sent during the gap when MISO was high-Z — was
        // silently lost. CMD24-after-CMD38 reliably hit this on cores3
        // + 16 GB card (NCR>1 post-erase), timing out at 10 s while
        // the card was actually fine.
        //
        // Commands with bytes the caller must consume AFTER R1:
        //   - R3/R7/R2 trailing response bytes: CMD8, CMD13, CMD58 send
        //     4 (or 1) data bytes in the same response stream right
        //     after R1.
        //   - Block-read commands: CMD9/10 (CSD/CID) and CMD17/18 (data
        //     blocks) follow R1 with a card-controlled gap, then a
        //     0xFE data-start token; the gap is normally many bytes
        //     long but is card-dependent and could in principle land
        //     within our 8-byte fast-path window.
        // For all of these we keep the original 1-byte fast-path read
        // so trailing bytes / data tokens stay queued in the card's
        // SPI pipeline for the caller's follow-up read. Init has retry
        // loops around the R-type readers so an occasional NCR>1 on
        // these commands is recoverable.
        let has_trailing_bytes = matches!(cmd.cmd, 8 | 9 | 10 | 13 | 17 | 18 | 58);
        let mut response = [0xFFu8; 8];
        let mut stuff = [0xFFu8; 1];

        use embedded_hal_async::spi::Operation;
        if cmd.cmd == stop_transmission().cmd {
            // CMD12 has a mandatory stuff byte before R1 (SPI-mode
            // erratum). Keep the original two-slot read.
            self.spi
                .transaction(&mut [
                    Operation::Write(&buf),
                    Operation::TransferInPlace(&mut stuff),
                    Operation::TransferInPlace(&mut response[..1]),
                ])
                .await
                .map_err(|_| Error::SpiError)?;
        } else {
            let resp_len = if has_trailing_bytes { 1 } else { 8 };
            self.spi
                .transaction(&mut [
                    Operation::Write(&buf),
                    Operation::TransferInPlace(&mut response[..resp_len]),
                ])
                .await
                .map_err(|_| Error::SpiError)?;
        }

        // Scan the response window for the first non-0xFF byte (R1).
        let scan_len = if cmd.cmd == stop_transmission().cmd || has_trailing_bytes { 1 } else { 8 };
        for &b in &response[..scan_len] {
            if b & 0x80 == 0 {
                return Ok(b);
            }
        }

        // Slow path: card violated SD spec (NCR > 8 bytes). Per-byte
        // poll with CS toggle is unreliable on fast hosts but a
        // compliant card cannot reach here, so the fallback is just
        // for graceful degradation on misbehaving cards.
        let mut polls: u32 = 1;
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
        self.wait_idle_with_timeout_ms(10_000).await
    }

    /// Default `wait_idle` semantics: return at the first all-0xFF
    /// 8-byte probe. Used for fast-path per-write idle confirmation.
    async fn wait_idle_with_timeout_ms(&mut self, timeout_ms: u32) -> Result<(), Error> {
        self.wait_idle_sustained_ms(timeout_ms, 1).await
    }

    /// Poll busy state with a caller-chosen timeout and a sustained-idle
    /// requirement: return only when `sustained_count` consecutive 8-byte
    /// probes have all read 0xFF. Any 0x00 byte in a probe resets the
    /// counter — a real polled signal that catches cards transiently
    /// flicking MISO high during post-CMD38 housekeeping.
    ///
    /// `sustained_count == 1` (used by the default `wait_idle`) gives
    /// the legacy "exit on first idle probe" behavior — fine for per-
    /// write idle waits where the card is in TRAN state when busy
    /// clears. The erase path passes a larger value to absorb post-
    /// erase housekeeping that the per-probe view alone can miss.
    ///
    /// Each loop iteration is one 8-byte SpiDevice transaction
    /// (CS held across all 8 bytes — single-byte polls are unreliable
    /// on fast SPI hosts like ESP32-S3 GDMA) plus a 1 ms delay so
    /// shared-bus consumers (display flush) aren't starved.
    async fn wait_idle_sustained_ms(
        &mut self,
        timeout_ms: u32,
        sustained_count: u32,
    ) -> Result<(), Error> {
        use embedded_hal_async::spi::Operation;
        let target = sustained_count.max(1);
        let outer = with_timeout(self.delay.clone(), timeout_ms, async {
            let mut consec: u32 = 0;
            loop {
                let mut probe = [0xFFu8; 8];
                self.spi
                    .transaction(&mut [Operation::TransferInPlace(&mut probe)])
                    .await
                    .map_err(|_| Error::SpiError)?;
                if probe.iter().all(|&b| b == 0xFF) {
                    consec += 1;
                    if consec >= target {
                        return Ok(());
                    }
                } else {
                    consec = 0;
                }
                self.delay.delay_ms(1).await;
            }
        })
        .await;
        if let Err(Error::Timeout) = outer {
            error!(
                "sdspi: wait_idle timed out after {} ms (sustained target {})",
                timeout_ms, sustained_count
            );
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

impl<SPI, D, ALIGN> block_device_driver::Erase for SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    type Error = Error;

    async fn erase_blocks(&mut self, start_block: u32, end_block: u32) -> Result<(), Self::Error> {
        self.erase(start_block, end_block).await
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
