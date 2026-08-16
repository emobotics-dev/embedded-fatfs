//! A crate for interfacing with SD cards over SPI.

#![no_std]

use block_device_driver::DmaBlock;
use core::fmt::Debug;
use core::future::Future;
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
    /// The arbiter would not hand over the bus. The command fails rather than
    /// running unlocked — an unlocked command is how a display flush clocks the
    /// card's payload away.
    BusUnavailable,
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


/// The only way to reach the SPI bus.
///
/// `SdSpi` holds one of these instead of a device, so a command that forgets to
/// take the lock has nothing to talk to and does not compile. That is the whole
/// point: the previous arrangement was an app-level `acquire_bus()` call which
/// any path could simply not make, and the erase path deliberately did not.
///
/// Granularity is per COMMAND, not per operation and not per session: a public
/// method acquires once and passes the guard down to its helpers, so CS-visible
/// sequences (command, token wait, payload) cannot be interleaved by another
/// bus master. Between commands — and inside `wait_idle_sustained_ms`, between
/// busy probes — the guard is dropped, which is what lets a display flush share
/// the bus during a multi-second erase.
use embedded_hal_async::spi::SpiBus as _;
use embedded_hal::digital::OutputPin as _;

pub trait BusAccess {
    /// The raw bus. NOT a `SpiDevice`: a `SpiDevice` asserts CS on entry to
    /// every call and deasserts on exit, so a command built from several calls
    /// drops CS in the middle of itself. Taking the bus and the chip-select
    /// separately is what lets CS stay low for a whole command, including the
    /// unbounded `0xFE` data-token wait that cannot fit in one transaction.
    type Bus: embedded_hal_async::spi::SpiBus;
    /// The card's chip-select, driven by this driver rather than by a device
    /// wrapper.
    type Cs: embedded_hal::digital::OutputPin;
    /// Held for one command; dropping it releases the bus.
    type Guard<'a>: BusAndCs<Bus = Self::Bus, Cs = Self::Cs>
    where
        Self: 'a;

    /// Take the bus. `None` means the arbiter gave up waiting — the caller
    /// fails the command rather than proceeding unlocked.
    fn acquire(&self) -> impl core::future::Future<Output = Option<Self::Guard<'_>>>;
}

/// A guard lending the bus and the chip-select together.
///
/// Together is the point: CS asserted while another master can drive the bus
/// would clock that master's bytes into a selected card, which is worse than
/// the CS drops this replaces. Whatever provides this must hold both.
///
/// # Contract
///
/// **Dropping the guard MUST deassert CS.** The driver asserts it once per
/// command and then uses `?` freely; every early return therefore releases the
/// card by dropping the guard. Releasing the bus and deselecting the card are
/// the same event, so the deselect belongs with the release — an implementation
/// that leaves CS low after drop leaves the card selected while another master
/// owns the bus.
pub trait BusAndCs {
    type Bus: embedded_hal_async::spi::SpiBus;
    type Cs: embedded_hal::digital::OutputPin;
    fn split(&mut self) -> (&mut Self::Bus, &mut Self::Cs);
}

pub struct SdSpi<A, D>
where
    A: BusAccess,
    D: embedded_hal_async::delay::DelayNs,
{
    bus: A,
    delay: D,
    card: Option<Card>,
}

impl<A, D> SdSpi<A, D>
where
    A: BusAccess,
    D: embedded_hal_async::delay::DelayNs + Clone,
{
    pub fn new(bus: A, delay: D) -> Self {
        Self {
            bus,
            delay,
            card: None,
        }
    }

    /// Take the bus for one command, or fail the command.
    async fn lock(&self) -> Result<A::Guard<'_>, Error> {
        self.bus.acquire().await.ok_or(Error::BusUnavailable)
    }

    /// To comply with the SD card spec, [sd_init] must be called between powerup and calling this function.
    pub async fn init(&mut self) -> Result<(), Error> {
        // One lock for the WHOLE command: the card sees command, token wait and
        // payload with no other master able to interleave. Released on return,
        // so the next command and the display both get their turn.
        let mut _guard = self.lock().await?;
        // CS low for the WHOLE command, released after the last operation. This
        // is the guard: the individual transfers below no longer touch CS, so
        // it cannot rise mid-command — not even across the unbounded 0xFE
        // token wait, which is what no fixed `transaction()` could express.
        let (spi, cs) = _guard.split();
        cs.set_low().map_err(|_| Error::ChipSelect)?;
        let r = async {
            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(spi, idle()).await?;
                    if r == R1_IDLE_STATE {
                        return Ok(());
                    }
                    // PARK between polls — do NOT spin, and do NOT use
                    // `yield_now` here. On fire27 this runs on the PRO-core
                    // level-1 InterruptExecutor; a self-wake (spin or
                    // yield_now) just re-pends the level-1 SWI, which re-fires
                    // immediately on handler return and never lets the level-0
                    // thread-mode executor run. That executor hosts the BLE
                    // controller blob's scheduler — starve it and the blob
                    // desyncs and the whole PRO executor wedges (HIL: 100% boot
                    // freeze with no card, where this CMD0 loop spins the full
                    // 1 s timeout 5× via the caller's retry loop). A real timer
                    // park idles the interrupt-exec so level-0 (and the blob)
                    // gets to run.
                    self.delay.clone().delay_ms(1).await;
                }
            })
            .await??;

            // "The SPI interface is initialized in the CRC OFF mode in default"
            // -- SD Part 1 Physical Layer Specification v9.00, Section 7.2.2 Bus Transfer Protection
            if self.cmd(spi, cmd::<R1>(0x3B, 1)).await? != R1_IDLE_STATE {
                return Err(Error::Cmd59Error);
            }

            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(spi, send_if_cond(0x1, 0xAA)).await?;
                    if r == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                        return Err(Error::UnsupportedCard);
                    }
                    let mut buffer = [0xFFu8; 4];
                    spi.transfer_in_place(&mut buffer[..]).await.map_err(|_| Error::SpiError)?;
                    if buffer[3] == 0xAA {
                        return Ok(());
                    }
                    self.delay.clone().delay_ms(1).await; // park — see CMD0 loop above
                }
            })
            .await??;

            trace!("Valid card detected!");

            // If we get here we're at least a v2 card
            let mut card = Card::default();

            // send ACMD41
            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.acmd(spi, sd_send_op_cond(true, false, true, 0x20)).await?;
                    if r == R1_READY_STATE {
                        return Ok(());
                    }
                    // park — ACMD41 loops many times on a slow card mid-init;
                    // a bare spin here also starves the level-0 blob (see CMD0).
                    self.delay.clone().delay_ms(1).await;
                }
            })
            .await??;

            trace!("send_ocr");
            card.ocr = with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(spi, cmd::<R3>(0x3A, 0)).await?;
                    if r != R1_READY_STATE {
                        return Err(Error::Cmd58Error);
                    }
                    let mut buffer = [0xFFu8; 4];
                    spi.transfer_in_place(&mut buffer[..]).await.map_err(|_| Error::SpiError)?;
                    let ocr: OCR<SD> = u32::from_be_bytes(buffer).into();
                    if !ocr.is_busy() {
                        return Ok(ocr);
                    }
                    self.delay.clone().delay_ms(1).await; // park — see CMD0 loop above
                }
            })
            .await??;

            trace!("send_csd");
            let r = self.cmd(spi, send_csd(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut csd = [0xFFu8; 16];
            self.read_data(spi, &mut csd).await?;
            card.csd = u128::from_be_bytes(csd).into();

            trace!("all_send_cid");
            let r = self.cmd(spi, send_cid(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut cid = [0xFFu8; 16];
            self.read_data(spi, &mut cid).await?;
            card.cid = u128::from_be_bytes(cid).into();

            debug!("Found card with size: {}bytes", card.size());

            // Returned rather than stored here: assigning `self.card` inside
            // this block would capture `self` MUTABLY, and the guard above
            // already borrows `self` for the whole command. Store it after the
            // guard is gone.
            Ok(card)
        }
        .await;

        let card = r?;
        drop(_guard);
        self.card = Some(card);
        Ok(())
    }

    pub async fn read<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &mut [DmaBlock<SIZE>],
    ) -> Result<(), Error> {
        // One lock for the WHOLE command: the card sees command, token wait and
        // payload with no other master able to interleave. Released on return,
        // so the next command and the display both get their turn.
        let mut _guard = self.lock().await?;
        // CS low for the WHOLE command, released after the last operation. This
        // is the guard: the individual transfers below no longer touch CS, so
        // it cannot rise mid-command — not even across the unbounded 0xFE
        // token wait, which is what no fixed `transaction()` could express.
        let (spi, cs) = _guard.split();
        cs.set_low().map_err(|_| Error::ChipSelect)?;
        let n = data.len();
        let r = async {
            if n == 1 {
                self.cmd(spi, read_single_block(block_address)).await.map_err(|e| {
                    error!("sdspi::read[single] CMD17 @ {}: {:?}", block_address, e);
                    e
                })?;
                self.read_data(spi, &mut data[0][..]).await.map_err(|e| {
                    error!("sdspi::read[single] read_data @ {}: {:?}", block_address, e);
                    e
                })?;
            } else {
                self.cmd(spi, read_multiple_blocks(block_address)).await.map_err(|e| {
                    error!("sdspi::read[multi] CMD18 @ {} n={}: {:?}", block_address, n, e);
                    e
                })?;
                for (i, block) in data.iter_mut().enumerate() {
                    self.read_data(spi, &mut block[..]).await.map_err(|e| {
                        error!("sdspi::read[multi] read_data block {} @ {}: {:?}", i, block_address, e);
                        e
                    })?;
                }
                self.cmd(spi, stop_transmission()).await.map_err(|e| {
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
        data: &[DmaBlock<SIZE>],
    ) -> Result<(), Error> {
        // One lock for the WHOLE command: the card sees command, token wait and
        // payload with no other master able to interleave. Released on return,
        // so the next command and the display both get their turn.
        let mut _guard = self.lock().await?;
        // CS low for the WHOLE command, released after the last operation. This
        // is the guard: the individual transfers below no longer touch CS, so
        // it cannot rise mid-command — not even across the unbounded 0xFE
        // token wait, which is what no fixed `transaction()` could express.
        let (spi, cs) = _guard.split();
        cs.set_low().map_err(|_| Error::ChipSelect)?;
        let n = data.len();
        let r = async {
            if n == 1 {
                self.cmd(spi, write_single_block(block_address)).await.map_err(|e| {
                    error!("sdspi::write[single] CMD24 @ {}: {:?}", block_address, e);
                    e
                })?;
                self.write_data(spi, DATA_START_BLOCK, &data[0][..]).await.map_err(|e| {
                    error!("sdspi::write[single] write_data @ {}: {:?}", block_address, e);
                    e
                })?;
                self.wait_idle(spi).await.map_err(|e| {
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
                self.acmd(spi, cmd::<R1>(0x17, n as u32)).await.map_err(|e| {
                    error!("sdspi::write[multi] ACMD23 @ {} n={}: {:?}", block_address, n, e);
                    e
                })?;
                self.wait_idle(spi).await.map_err(|e| {
                    error!("sdspi::write[multi] wait_idle post-ACMD23 @ {}: {:?}", block_address, e);
                    e
                })?;

                let r1 = self.cmd(spi, write_multiple_blocks(block_address)).await.map_err(|e| {
                    error!("sdspi::write[multi] CMD25 @ {} n={}: {:?}", block_address, n, e);
                    e
                })?;
                if r1 != 0 {
                    error!("sdspi::write[multi] CMD25 R1 nonzero @ {} n={}: 0x{:02x}", block_address, n, r1);
                    return Err(Error::RegisterError(r1));
                }
                for (i, block) in data.iter().enumerate() {
                    self.wait_idle(spi).await.map_err(|e| {
                        error!("sdspi::write[multi] wait_idle pre-block {} @ {}: {:?}", i, block_address, e);
                        e
                    })?;
                    self.write_data(spi, WRITE_MULTIPLE_TOKEN, &block[..]).await.map_err(|e| {
                        error!("sdspi::write[multi] write_data block {} @ {}: {:?}", i, block_address, e);
                        e
                    })?;
                }
                // stop the write
                self.wait_idle(spi).await.map_err(|e| {
                    error!("sdspi::write[multi] wait_idle pre-STOP @ {}: {:?}", block_address, e);
                    e
                })?;
                spi.write(&[STOP_TRAN_TOKEN]).await.map_err(|_| {
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
        // No lock: this reads the CSD cached at init and never touches the bus.
        // Taking one anyway would be cargo-culting the pattern.
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

        // The three erase commands go under ONE lock — nothing may interleave
        // between CMD32/33/38. The guard is then dropped BEFORE the busy wait:
        // a full-card erase takes seconds, and holding across it would freeze
        // the display for the whole format. That wait re-acquires per probe.
        {
        let mut _guard = self.lock().await?;
        // CS low for the WHOLE command, released after the last operation. This
        // is the guard: the individual transfers below no longer touch CS, so
        // it cannot rise mid-command — not even across the unbounded 0xFE
        // token wait, which is what no fixed `transaction()` could express.
        let (spi, cs) = _guard.split();
        cs.set_low().map_err(|_| Error::ChipSelect)?;

        let r = self.cmd(spi, cmd::<R1>(32, start_block)).await?;
        if r != R1_READY_STATE {
            error!("sdspi::erase CMD32 R1=0x{:02x}", r);
            return Err(Error::EraseError);
        }

        let r = self.cmd(spi, cmd::<R1>(33, end_block)).await?;
        if r != R1_READY_STATE {
            error!("sdspi::erase CMD33 R1=0x{:02x}", r);
            return Err(Error::EraseError);
        }

        let r = self.cmd(spi, cmd::<R1>(38, 0)).await?;
        if r != R1_READY_STATE {
            error!("sdspi::erase CMD38 R1=0x{:02x}", r);
            return Err(Error::EraseError);
        }

        }
        // Guard dropped above. CMD38 returns R1b — the card holds busy until
        // the erase completes. 60 s sanity bound is large because the SD spec
        // puts no fixed upper bound on full-card CMD38 (a function of capacity
        // x per-AU erase time). Re-acquires per probe, so the display renders
        // format progress throughout.
        self.wait_idle_reacquiring(60_000, 1).await?;

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
        self.wait_idle_reacquiring(60_000, 2_000).await?;

        Ok(())
    }

    async fn read_data(&self, spi: &mut A::Bus, buffer: &mut [u8]) -> Result<(), Error> {
        // Same rationale as `wait_idle` — yield between polls while
        // waiting for the data-start token so other async tasks on
        // the same executor are not starved.
        let outer = with_timeout(self.delay.clone(), 1000, async {
            let mut byte = self.read_byte(spi).await?;
            while byte == 0xFF {
                yield_now().await;
                byte = self.read_byte(spi).await?;
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
        spi
            .transfer_in_place(buffer)
            .await
            .map_err(|_| Error::SpiError)?;
        spi.transfer_in_place(&mut crc_bytes).await.map_err(|_| Error::SpiError)?;
        let crc = u16::from_be_bytes(crc_bytes);
        let calc_crc = crc16(buffer);
        if crc != calc_crc {
            return Err(Error::CrcMismatch(crc, calc_crc));
        }

        Ok(())
    }

    async fn write_data(&self, spi: &mut A::Bus, token: u8, buffer: &[u8]) -> Result<(), Error> {
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
        // Word-aligned: ESP32 PDMA needs 4-byte aligned DMA buffers;
        // a bare `[u8; 8]` on stack is alignment 1. See wait_idle probe
        // comment for the failure mode. `[u32; 2]` forces 4-aligned.
        let mut status_word = [0xFFFFFFFFu32; 2];
        let status_buf: &mut [u8; 8] = unsafe {
            &mut *(status_word.as_mut_ptr() as *mut [u8; 8])
        };
        spi
            .write(&token_buf)
            .await
            .map_err(|_| Error::SpiError)?;
        spi.write(buffer).await.map_err(|_| Error::SpiError)?;
        spi.write(&crc_bytes).await.map_err(|_| Error::SpiError)?;
        spi.transfer_in_place(status_buf).await.map_err(|_| Error::SpiError)?;

        for &b in &*status_buf {
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

    // `pub fn spi(&mut self) -> &mut SPI` used to live here, handing the raw
    // device to anyone who asked. It is deleted, not fixed: an escape hatch is
    // precisely what makes bus access representable without the lock, and one
    // caller using it would reintroduce the whole class silently.

    async fn cmd<R: Resp>(&self, spi: &mut A::Bus, cmd: Cmd<R>) -> Result<u8, Error> {
        if cmd.cmd != idle().cmd {
            self.wait_idle(spi).await?;
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
        // Word-aligned 8-byte response. See wait_idle probe comment for the
        // ESP32 PDMA alignment hazard. `stuff` is only one byte so DMA
        // alignment is moot (1-byte transfers don't need word alignment).
        let mut response_word = [0xFFFFFFFFu32; 2];
        let response: &mut [u8; 8] = unsafe {
            &mut *(response_word.as_mut_ptr() as *mut [u8; 8])
        };
        let mut stuff = [0xFFu8; 1];

        if cmd.cmd == stop_transmission().cmd {
            // CMD12 has a mandatory stuff byte before R1 (SPI-mode
            // erratum). Keep the original two-slot read.
            spi
                .write(&buf)
                .await
                .map_err(|_| Error::SpiError)?;
            spi.transfer_in_place(&mut stuff).await.map_err(|_| Error::SpiError)?;
            spi.transfer_in_place(&mut response[..1]).await.map_err(|_| Error::SpiError)?;
        } else {
            let resp_len = if has_trailing_bytes { 1 } else { 8 };
            spi
                .write(&buf)
                .await
                .map_err(|_| Error::SpiError)?;
            spi.transfer_in_place(&mut response[..resp_len])
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
                let byte = self.read_byte(spi).await?;
                polls += 1;
                if byte & 0x80 == 0 {
                    return Ok(byte);
                }
                // PARK between polls — THIS is the no-card hog. A missing card
                // never sends R1, so every CMD0 falls through the 8-byte scan
                // into this loop and spins it the full timeout. On fire27 cmd()
                // runs on the PRO-core level-1 InterruptExecutor; a bare spin of
                // single-byte transfers here pins level-1 and starves the
                // level-0 thread-mode BLE blob (HIL 2026-05-30: BLE init dragged
                // 2.8s→5.5s, then wedge). The outer init retry loops never reach
                // their own park because cmd() never returns on a missing card —
                // it lives here until the caller's with_timeout cancels it. A
                // real timer park idles the interrupt-exec so level-0 runs.
                self.delay.clone().delay_ms(1).await;
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

    async fn acmd<R: Resp>(&self, spi: &mut A::Bus, cmd: Cmd<R>) -> Result<u8, Error> {
        self.cmd(spi, app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16))
            .await?;
        self.cmd(spi, cmd).await
    }

    async fn wait_idle(&self, spi: &mut A::Bus) -> Result<(), Error> {
        self.wait_idle_with_timeout_ms(spi, 10_000).await
    }

    /// Default `wait_idle` semantics: return at the first all-0xFF
    /// 8-byte probe. Used for fast-path per-write idle confirmation.
    async fn wait_idle_with_timeout_ms(&self, spi: &mut A::Bus, timeout_ms: u32) -> Result<(), Error> {
        self.wait_idle_sustained_ms(spi, timeout_ms, 1).await
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
    /// Interleaving variant: takes the bus for ONE probe at a time.
    ///
    /// For waits measured in seconds — post-CMD38 above all. Holding across
    /// those would freeze the display for the whole format, which is the thing
    /// a format must never do. Acquiring per probe means the display is
    /// guaranteed the bus between any two probes, and the fair arbiter bounds
    /// how long the probe then waits to get it back.
    ///
    /// The caller must NOT hold a guard when calling this.
    async fn wait_idle_reacquiring(&self, timeout_ms: u32, sustained_count: u32) -> Result<(), Error> {
        let target = sustained_count.max(1);
        let outer = with_timeout(self.delay.clone(), timeout_ms, async {
            let mut consec: u32 = 0;
            loop {
                let idle = {
                    let mut guard = self.bus.acquire().await.ok_or(Error::BusUnavailable)?;
                    let (bus, cs) = guard.split();
                    cs.set_low().map_err(|_| Error::ChipSelect)?;
                    // 4-aligned backing: ESP32 PDMA needs it (see the locked
                    // variant's note).
                    let mut probe_word = [0xFFFF_FFFFu32; 2];
                    let probe: &mut [u8; 8] =
                        unsafe { &mut *(probe_word.as_mut_ptr() as *mut [u8; 8]) };
                    bus
                        .transfer_in_place(probe)
                        .await
                        .map_err(|_| Error::SpiError)?;
                    probe.iter().all(|&b| b == 0xFF)
                    // guard dropped here — the display's turn
                };
                if idle {
                    consec += 1;
                    if consec >= target {
                        return Ok(());
                    }
                } else {
                    consec = 0;
                }
                self.delay.clone().delay_ms(1).await;
            }
        })
        .await;
        outer?
    }

    /// Locked variant: probes on a guard the CALLER holds.
    ///
    /// Must not acquire — it runs inside a command that already holds the bus,
    /// and the arbiter has one permit, so re-acquiring here would deadlock
    /// against ourselves. The interleaving variant is
    /// [`Self::wait_idle_reacquiring`].
    async fn wait_idle_sustained_ms(
        &self,
        spi: &mut A::Bus,
        timeout_ms: u32,
        sustained_count: u32,
    ) -> Result<(), Error> {
        let target = sustained_count.max(1);
        // Atomic counter so we can tell on timeout whether polling ran
        // normally (≈ timeout_ms polls — card stayed busy) or was starved
        // (only a handful of polls — SPI bus mutex held by another path).
        // The AtomicU32 is ALSO load-bearing for reliability beyond the
        // diagnostic: removing it (replacing with local u32) on top of the
        // [u32; 2] alignment fix re-triggers the silent fire27 wedge after
        // `sd: erasing N blocks` (HIL 2026-05-26 — see docs/spi-dma-and-
        // wakeup.md §7). LLVM lowers Relaxed fetch_add on Xtensa LX6 to
        // `s32c1i`, whose AHB bus arbitration / write-buffer flush has
        // hardware side effects that are not promised by Rust's Relaxed
        // ordering but are empirically required here. Do not remove.
        use core::sync::atomic::{AtomicU32, Ordering};
        static POLLS: AtomicU32 = AtomicU32::new(0);
        let start_polls = POLLS.load(Ordering::Relaxed);
        // Word-aligned backing storage. ESP32 PDMA REQUIRES the DMA
        // source/dest address to be 4-byte aligned; a bare `[u8; 8]` on
        // stack has alignment 1 and can land at any byte address, and
        // when it lands non-aligned the PDMA TransferDone IRQ never
        // fires (TransferInPlace = duplex, both TX and RX paths must be
        // aligned). Without explicit alignment the outcome is layout-
        // sensitive: a single info!() elsewhere shifts this stack frame
        // and can flip a working format path into an indefinite spin
        // inside the bus driver, which then never yields and traps the
        // executor itself (no timeout fires, no log appears). Use
        // [u32; 2] backing and cast on each probe.
        let outer = with_timeout(self.delay.clone(), timeout_ms, async {
            let mut consec: u32 = 0;
            let mut probe_word: [u32; 2];
            loop {
                probe_word = [0xFFFFFFFFu32; 2];
                let probe: &mut [u8; 8] = unsafe {
                    &mut *(probe_word.as_mut_ptr() as *mut [u8; 8])
                };
                spi.transfer_in_place(probe).await.map_err(|_| Error::SpiError)?;
                POLLS.fetch_add(1, Ordering::Relaxed);
                if probe.iter().all(|&b| b == 0xFF) {
                    consec += 1;
                    if consec >= target {
                        return Ok(());
                    }
                } else {
                    consec = 0;
                }
                self.delay.clone().delay_ms(1).await;
            }
        })
        .await;
        if let Err(Error::Timeout) = outer {
            let polls = POLLS.load(Ordering::Relaxed).wrapping_sub(start_polls);
            error!(
                "sdspi: wait_idle timed out after {} ms (sustained target {}) polls_in_window={}",
                timeout_ms, sustained_count, polls
            );
        }
        outer?
    }

    async fn read_byte(&self, spi: &mut A::Bus) -> Result<u8, Error> {
        let mut buf = [0xFFu8; 1];
        spi.transfer_in_place(&mut buf[..]).await.map_err(|_| Error::SpiError)?;

        Ok(buf[0])
    }
}

impl<A, D, const SIZE: usize> block_device_driver::BlockDevice<SIZE>
    for SdSpi<A, D>
where
    A: BusAccess,
    D: embedded_hal_async::delay::DelayNs + Clone,
{
    type Error = Error;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [DmaBlock<SIZE>],
    ) -> Result<(), Self::Error> {
        self.read(block_address, data).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[DmaBlock<SIZE>],
    ) -> Result<(), Self::Error> {
        self.write(block_address, data).await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.size().await
    }
}

impl<A, D> block_device_driver::Erase for SdSpi<A, D>
where
    A: BusAccess,
    D: embedded_hal_async::delay::DelayNs + Clone,
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
