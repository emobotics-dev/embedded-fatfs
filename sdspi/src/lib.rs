//! A crate for interfacing with SD cards over SPI.

// `no_std` on target; std under `cargo test` so the response decoders below
// can be unit-tested on the host without hardware.
#![cfg_attr(not(test), no_std)]

use block_device_driver::DmaBlock;
use core::fmt::Debug;
use core::future::Future;
use embassy_futures::select::{select, Either};
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

/// R1 status byte, decoded (SD Physical Layer spec v9.00, 7.3.2.1).
///
/// Bit 7 is always 0 on a valid R1; the rest are sticky error flags that the
/// card clears on the next command. Every one of them is checked -- an R1 that
/// is merely "not the value we hoped for" hides which fault occurred.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct R1Status(pub u8);

impl R1Status {
    pub const IDLE: u8 = 0x01;
    pub const ERASE_RESET: u8 = 0x02;
    pub const ILLEGAL_COMMAND: u8 = 0x04;
    pub const COM_CRC_ERROR: u8 = 0x08;
    pub const ERASE_SEQUENCE_ERROR: u8 = 0x10;
    pub const ADDRESS_ERROR: u8 = 0x20;
    pub const PARAMETER_ERROR: u8 = 0x40;
    /// Every bit that reports a fault (i.e. all but `IDLE`).
    pub const ERROR_MASK: u8 = 0x7E;

    pub fn is_valid(self) -> bool { self.0 & 0x80 == 0 }
    pub fn idle(self) -> bool { self.0 & Self::IDLE != 0 }
    pub fn ready(self) -> bool { self.0 == 0 }
    pub fn errors(self) -> u8 { self.0 & Self::ERROR_MASK }

    /// Faults that are wrong in EVERY card state, so `cmd()` can reject them
    /// without knowing what the caller was probing for.
    ///
    /// `ILLEGAL_COMMAND` and `ERASE_RESET` are deliberately excluded: they are
    /// legitimate answers, not failures. CMD8 identifies a v1 card *by* being
    /// refused, and some cards refuse CMD59. Treating them as hard errors makes
    /// a normal negotiation look like a broken card.
    pub const ALWAYS_FAULT_MASK: u8 =
        Self::COM_CRC_ERROR | Self::PARAMETER_ERROR | Self::ADDRESS_ERROR
        | Self::ERASE_SEQUENCE_ERROR;

    /// The unconditional fault this R1 reports, if any. Used by `cmd()`.
    pub fn to_hard_error(self) -> Option<Error> {
        if !self.is_valid() {
            return Some(Error::NoResponse);
        }
        if self.0 & Self::ALWAYS_FAULT_MASK == 0 {
            return None;
        }
        self.to_error()
    }

    /// The first fault the card reports, most severe first, or `None`.
    ///
    /// Order matters: a CRC error means the command never took effect, so it is
    /// reported ahead of consequences like ADDRESS_ERROR.
    pub fn to_error(self) -> Option<Error> {
        if !self.is_valid() {
            return Some(Error::NoResponse);
        }
        if self.0 & Self::COM_CRC_ERROR != 0 { return Some(Error::CommandCrcError); }
        if self.0 & Self::ILLEGAL_COMMAND != 0 { return Some(Error::IllegalCommand); }
        if self.0 & Self::PARAMETER_ERROR != 0 { return Some(Error::ParameterError); }
        if self.0 & Self::ADDRESS_ERROR != 0 { return Some(Error::AddressError); }
        if self.0 & Self::ERASE_SEQUENCE_ERROR != 0 { return Some(Error::EraseSequenceError); }
        if self.0 & Self::ERASE_RESET != 0 { return Some(Error::EraseReset); }
        None
    }
}

/// Data-response token returned after a write block (spec 7.3.3.1).
/// Format `xxx0sss1`; only the three `sss` bits carry meaning.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataResponse(pub u8);

impl DataResponse {
    pub fn status(self) -> u8 { self.0 & DATA_RES_MASK }
    pub fn accepted(self) -> bool { self.status() == DATA_RES_ACCEPTED }

    pub fn to_error(self) -> Option<Error> {
        match self.status() {
            DATA_RES_ACCEPTED => None,
            0x0B => Some(Error::DataCrcError),
            0x0D => Some(Error::DataWriteError),
            other => Some(Error::UnknownDataResponse(other)),
        }
    }
}

/// Data-error token, sent instead of a data-start token when the card cannot
/// deliver a block (spec 7.3.3.3). High nibble is zero; low bits are flags.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataErrorToken(pub u8);

impl DataErrorToken {
    /// A token is an error token only if the top four bits are clear.
    pub fn is_error_token(self) -> bool { self.0 & 0xF0 == 0 && self.0 != 0 }

    pub fn to_error(self) -> Option<Error> {
        if !self.is_error_token() { return None; }
        if self.0 & 0x08 != 0 { return Some(Error::OutOfRange); }
        if self.0 & 0x04 != 0 { return Some(Error::CardEccFailed); }
        if self.0 & 0x02 != 0 { return Some(Error::CardControllerError); }
        Some(Error::ReadError)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// No R1 arrived inside the spec's NCR window and the fallback poll.
    /// Carries the command index, because "some command went unanswered" is
    /// not actionable — CMD0 silent means no card, ACMD41 silent means the
    /// card stopped answering partway through init.
    NoResponseTo(u8),
    /// An R1 was read but bit 7 was set, so it was not an R1 at all.
    NoResponse,
    /// R1: the command's CRC was wrong, so the card ignored it.
    CommandCrcError,
    /// R1: command not legal for the current card state.
    IllegalCommand,
    /// R1: argument out of the allowed range.
    ParameterError,
    /// R1: misaligned address for the block length.
    AddressError,
    /// R1: erase sequence broken (e.g. CMD32/33/38 out of order).
    EraseSequenceError,
    /// R1: an erase sequence was cleared before it ran.
    EraseReset,
    /// Write rejected: the card saw a CRC error in the data block.
    DataCrcError,
    /// Write rejected: an internal write error.
    DataWriteError,
    /// Data response token outside the values the spec defines.
    UnknownDataResponse(u8),
    /// Read failed: generic error token.
    ReadError,
    /// Read failed: internal card controller error.
    CardControllerError,
    /// Read failed: ECC could not correct the data.
    CardEccFailed,
    /// Read failed: address out of range.
    OutOfRange,
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

/// Stretch a wall-clock bound when the program is being interpreted.
///
/// Every timeout in this driver measures real time, which stops being a
/// progress measure under Miri (~100x slower): a perfectly healthy wait
/// overruns and the operation is reported as a fault. The block layer already
/// scales its backstops for this reason; these are the driver-side twins.
///
/// Most visible on the post-CMD38 sustained-idle wait, which needs 2000 probes
/// at 1 ms each -- 2 s natively, ~200 s interpreted, against a 60 s bound.
#[inline]
const fn wall(ms: u32) -> u32 {
    if cfg!(miri) { ms.saturating_mul(100) } else { ms }
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
    // DRAM, not `&[0xFF; 256]`: a flash literal picks the copy path (§14).
    let mut flush_words = [0xFFFF_FFFFu32; 64];
    let flush: &mut [u8; 256] = unsafe { &mut *(flush_words.as_mut_ptr() as *mut [u8; 256]) };
    spi.write(&flush[..]).await.map_err(|_| Error::SpiError)?;

    Ok(())
}


use embedded_hal_async::spi::SpiBus as _;

use embedded_hal::digital::OutputPin as _;

/// The only way to reach the SPI bus.
///
/// `SdSpi` holds one of these instead of a device, so a command that forgets to
/// lock has nothing to talk to and does not compile.
///
/// Granularity is per COMMAND: acquire once, pass the guard to the helpers, so
/// command/token-wait/payload cannot be interleaved. Released between commands
/// and between busy probes, which is what lets the display share the bus.
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

/// A guard lending bus and chip-select together. Together, because CS asserted
/// while another master can drive the bus is worse than the CS drops this
/// replaces.
///
/// # Contract
///
/// **Dropping the guard MUST deassert CS** — the driver asserts once per
/// command and then uses `?` freely, so every early return releases the card.
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

    /// Wait for the card to go idle WITHOUT holding the bus. Called before
    /// acquiring: a card still programming a previous write must not pin the
    /// bus, or the block layer's backstop is spent waiting for it.
    async fn settle(&self) -> Result<(), Error> {
        self.wait_idle_reacquiring(wall(10_000), 1).await
    }

    /// Take the bus for one command, or fail the command.
    async fn lock(&self) -> Result<A::Guard<'_>, Error> {
        let g = self.bus.acquire().await.ok_or(Error::BusUnavailable);
        g
    }

    /// The arbiter, NOT the bus: callers can reach board-level settings (the
    /// post-init clock) without gaining a way to talk to the card unlocked.
    pub fn bus(&self) -> &A {
        &self.bus
    }

    /// Run `f` against bus + CS with the bus held. For non-command work (the
    /// post-init clock raise). References cannot outlive the guard, so this
    /// does not reopen the hole that deleting `spi()` closed.
    pub async fn with_bus<R>(
        &self,
        f: impl FnOnce(&mut A::Bus, &mut A::Cs) -> R,
    ) -> Result<R, Error> {
        let mut guard = self.lock().await?;
        let (bus, cs) = guard.split();
        Ok(f(bus, cs))
    }

    /// To comply with the SD card spec, [sd_init] must be called between powerup and calling this function.
    pub async fn init(&mut self) -> Result<(), Error> {
        // One lock for the WHOLE command: the card sees command, token wait and
        // payload with no other master able to interleave. Released on return,
        // so the next command and the display both get their turn.
        self.settle().await?;
        let mut _guard = self.lock().await?;
        // CS low for the WHOLE command, released after the last operation. This
        // is the guard: the individual transfers below no longer touch CS, so
        // it cannot rise mid-command — not even across the unbounded 0xFE
        // token wait, which is what no fixed `transaction()` could express.
        let (spi, cs) = _guard.split();
        cs.set_low().map_err(|_| Error::ChipSelect)?;
        let r = async {
            with_timeout(self.delay.clone(), wall(1000), async {
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

            with_timeout(self.delay.clone(), wall(1000), async {
                loop {
                    // CMD8 is the version probe: a v1 card ILLEGALLY-COMMANDs
                    // it, which identifies the card rather than being a fault.
                    // `cmd()` now surfaces that bit as an error, so catch it
                    // here instead of comparing raw bytes.
                    let r = self.cmd(spi, send_if_cond(0x1, 0xAA)).await?;
                    if R1Status(r).0 & R1Status::ILLEGAL_COMMAND != 0 {
                        // v1 card: it refuses CMD8. That identifies it.
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
            with_timeout(self.delay.clone(), wall(1000), async {
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
            card.ocr = with_timeout(self.delay.clone(), wall(1000), async {
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
            // TRAN_SPEED: 0x32 = 25 MHz default speed, 0x5a = 50 MHz high speed.
            debug!("sdspi: CSD TRAN_SPEED 0x{:02x}", card.csd.transfer_rate());

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

        // Say WHAT the card answered when init fails. 0xFF every poll means
        // MISO never went low -- the card is not driving the line at all
        // (unselected, unpowered, or the pad is not routed to MISO), which is
        // a different fault from a card answering with an unexpected R1. A
        // bare `Timeout` cannot tell those apart.
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
        // ONE BLOCK PER LOCK, never CMD18.
        //
        // CS has to stay low for a whole command, so a multi-block read holds
        // the bus across EVERY block's data-token wait — up to 16 of them, each
        // bounded at 1 s. That is seconds of exclusive bus on a card that is
        // slow to answer, with the display starved behind it (measured: 282 of
        // 282 ms of a panel transfer spent waiting for the permit) and the
        // caller's backstop firing on work that was going to succeed.
        //
        // Single-block commands bound the hold to one block and free the bus
        // between them. Costs one command per block; the alternative is an
        // unbounded shared-bus hold, which no outer timeout can fix (#46).
        // Settle ONCE. Between two reads the card is never busy (no program
        // cycle), so a per-block idle probe is redundant work on the critical
        // path -- and a FAT scan is thousands of blocks, where that redundancy
        // showed up as a logger that started ~39 s late.
        self.settle().await?;
        for (i, block) in data.iter_mut().enumerate() {
            let addr = block_address + i as u32;
            let mut guard = self.lock().await?;
            // CS low for the whole command: the transfers below never touch it,
            // so it cannot rise mid-command — not across the token wait either,
            // which is what no fixed `transaction()` could express.
            let (spi, cs) = guard.split();
            cs.set_low().map_err(|_| Error::ChipSelect)?;
            let r = async {
                self.cmd(spi, read_single_block(addr)).await.map_err(|e| {
                    error!("sdspi::read CMD17 @ {}: {:?}", addr, e);
                    e
                })?;
                self.read_data(spi, &mut block[..]).await.map_err(|e| {
                    error!("sdspi::read read_data @ {}: {:?}", addr, e);
                    e
                })?;
                Ok::<(), Error>(())
            }
            .await;
            r?;
        }

        Ok(())
    }

    pub async fn write<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &[DmaBlock<SIZE>],
    ) -> Result<(), Error> {
        // ONE BLOCK PER LOCK, never CMD25 — same reasoning as `read`, and worse
        // here: a multi-block write holds the bus across every block's busy
        // wait, each bounded at 10 s, so up to 16 program cycles back to back
        // with CS pinned low. That is the hold no outer backstop can bound
        // (#46). ACMD23 pre-erase goes with it; the format path already writes
        // pre-erased.
        // Settle ONCE: after each block the program-cycle wait below already
        // leaves the card idle, so re-probing before the next command adds a
        // round trip and proves nothing new.
        self.settle().await?;
        for (i, block) in data.iter().enumerate() {
            let addr = block_address + i as u32;
            {
                let mut guard = self.lock().await?;
                let (spi, cs) = guard.split();
                cs.set_low().map_err(|_| Error::ChipSelect)?;
                let r = async {
                    self.cmd(spi, write_single_block(addr)).await.map_err(|e| {
                        error!("sdspi::write CMD24 @ {}: {:?}", addr, e);
                        e
                    })?;
                    self.write_data(spi, DATA_START_BLOCK, &block[..]).await.map_err(|e| {
                        error!("sdspi::write write_data @ {}: {:?}", addr, e);
                        e
                    })?;
                    Ok::<(), Error>(())
                }
                .await;
                r?;
            }
            // Guard dropped: the card ACKed with DATA_RES_ACCEPTED and is now
            // running its program cycle. Wait it out with the bus RELEASED,
            // re-acquiring per probe, so the display is served throughout.
            self.wait_idle_reacquiring(wall(10_000), 1).await?;
        }

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
        self.settle().await?;
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
        self.wait_idle_reacquiring(wall(60_000), 1).await?;

        // Post-CMD38 sustained-idle. A card can show a brief FALSE idle right
        // after CMD38 — MISO goes 0xFF transiently while it is still doing
        // post-erase housekeeping. A single idle probe accepts that, and the
        // next command then starts on a busy card, holds the bus for its whole
        // duration waiting for a response that never comes, and dies on the
        // caller's backstop.
        //
        // This was gated per target, on because cores3 failed 0/10 without it
        // and off on fire27 because the wait used to HOLD the bus across
        // thousands of probes (starving RWBLE). It re-acquires per probe now,
        // so that objection is gone — and fire27 needed the guard all along:
        // without it, format failed intermittently with the display starved
        // (287 of 287 ms waiting for the bus) behind a stuck command. Ungated.
        //
        // sustained_count = 2 000 (~2 s of continuous idle): 200 ms exits OK
        // but the next command still finds the card busy. Fast cards reach it
        // promptly — any 0x00 byte resets the counter.
        self.wait_idle_reacquiring(wall(60_000), 2_000).await?;

        Ok(())
    }

    async fn read_data(&self, spi: &mut A::Bus, buffer: &mut [u8]) -> Result<(), Error> {
        // Poll for the 0xFE data-start token EIGHT bytes at a time.
        //
        // The polling itself is the protocol -- SD SPI has no ready signal, the
        // card returns 0xFF until it emits the token, and the host only sees it
        // by clocking. What is NOT required is clocking one byte per poll: the
        // card holds the token until read, so a wider window costs nothing and
        // cuts the number of transfers, and of yields, by 8x. `write_data`
        // already uses this trick for its data-response window.
        //
        // Bytes AFTER the token in the winning window are the first payload
        // bytes -- the card is already streaming. They are carried into the
        // buffer rather than discarded, which is what makes this correct rather
        // than merely faster.
        const WINDOW: usize = 8;
        // 4-aligned backing: ESP32 PDMA needs it (see the wait_idle probe).
        let mut window_word = [0xFFFF_FFFFu32; 2];
        let window: &mut [u8; WINDOW] =
            unsafe { &mut *(window_word.as_mut_ptr() as *mut [u8; WINDOW]) };

        let found = with_timeout(self.delay.clone(), wall(1000), async {
            loop {
                window.fill(0xFF);
                spi.transfer_in_place(&mut window[..]).await.map_err(|_| Error::SpiError)?;
                if let Some(i) = window.iter().position(|&b| b != 0xFF) {
                    return Ok::<usize, Error>(i);
                }
                // Short park, not `yield_now`: on fire27 a self-wake re-pends
                // the level-1 SWI and the level-0 thread-mode executor -- which
                // hosts the BLE blob -- never runs. 1 ms was measurably too
                // coarse (3/3 format failures against the block layer's 3 s
                // backstop); 100 us keeps block ops well inside it while still
                // handing the core back.
                self.delay.clone().delay_us(100).await;
            }
        })
        .await;
        if let Err(Error::Timeout) = found {
            error!("sdspi: read_data data-start token wait timed out after 1000 ms");
        }
        let i = found??;

        let token = window[i];
        if token != DATA_START_BLOCK {
            // A card that cannot deliver the block sends a data ERROR token
            // (high nibble clear) in place of the start token. Decode it --
            // reporting the raw byte as `RegisterError` threw away which of
            // out-of-range / ECC / controller error the card actually reported.
            if let Some(e) = DataErrorToken(token).to_error() {
                error!("sdspi::read_data error token 0x{:02x}: {:?}", token, e);
                return Err(e);
            }
            error!("sdspi::read_data unexpected token 0x{:02x}", token);
            return Err(Error::RegisterError(token));
        }

        // Carry the payload bytes that shared the token's window.
        let carry = core::cmp::min(WINDOW - (i + 1), buffer.len());
        buffer[..carry].copy_from_slice(&window[i + 1..i + 1 + carry]);

        let mut crc_bytes = [0xFFu8; 2];
        if carry < buffer.len() {
            buffer[carry..].fill(0xFF);
            spi.transfer_in_place(&mut buffer[carry..])
                .await
                .map_err(|_| Error::SpiError)?;
        }
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
                let response = DataResponse(b);
                if let Some(e) = response.to_error() {
                    // Return WHICH rejection, not a blanket WriteError: a CRC
                    // error means the data never landed and the block is
                    // retryable, a write error means the card failed to program
                    // it. Callers cannot tell those apart from one variant.
                    error!("sdspi::write_data rejected, status=0x{:02x}: {:?}", b, e);
                    return Err(e);
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

    /// Clock the N_RC gap: the spec's >=8 idle cycles between a response and
    /// the next command. Commands that read exactly their payload leave none,
    /// and a card that enforces it answers the next command with silence.
    ///
    /// The buffer must be DRAM-resident and 4-aligned: a flash literal selects
    /// esp-hal's copy path, and PDMA requires the alignment. Four bytes gives
    /// both, and 32 idle clocks where the spec asks for 8.
    async fn clock_n_rc_gap(spi: &mut A::Bus) -> Result<(), Error> {
        let mut gap = [0xFFFF_FFFFu32; 1];
        let bytes: &mut [u8; 4] = unsafe { &mut *(gap.as_mut_ptr() as *mut [u8; 4]) };
        spi.transfer_in_place(&mut bytes[..]).await.map_err(|_| Error::SpiError)
    }

    async fn cmd<R: Resp>(&self, spi: &mut A::Bus, cmd: Cmd<R>) -> Result<u8, Error> {
        // No idle wait here. `settle()` did it before the lock was taken, and
        // repeating it inside means holding the bus for the card's programming
        // time -- which is what spent the block layer's 3 s backstop and
        // reported as "Write STALL, card state unknown".
        //
        // CS continuity is needed for the DATA phase (command -> token ->
        // payload), not for busy polling: a busy wait has no data phase to
        // corrupt. That is why the old device-per-probe wait was safe, and why
        // moving it outside the lock loses nothing.

        Self::clock_n_rc_gap(spi).await?;

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
        // Grace for a card that missed the NCR window, bounded WELL below the
        // callers' budgets. The spec puts NCR at 0-8 bytes -- 160 us at 400 kHz
        // -- so a card still silent after 200 ms is not "slow", it is not
        // answering. At 10 s this bound could never fire: every init loop wraps
        // cmd() in 1 s, so the outer timeout always won and the failure arrived
        // as an opaque `Timeout` with no idea which command or why. Same
        // ordering rule as the block layer's backstop: an inner bound that
        // exceeds its caller's budget is unreachable, and unreachable bounds
        // report nothing.
        const NCR_GRACE_MS: u32 = 200;
        let mut polls: u32 = 1;
        let outer = with_timeout(self.delay.clone(), wall(NCR_GRACE_MS), async {
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
                "sdspi: cmd {} got no R1 within {} ms ({} polls)",
                cmd.cmd, NCR_GRACE_MS, polls
            );
            // NOT `Timeout`: the card never answered at all, which is a
            // different fault from an operation that ran too long.
            return Err(Error::NoResponseTo(cmd.cmd));
        }
        let byte = outer??;

        // Every R1 error bit is a fault the card is reporting; returning the
        // raw byte let callers compare against the one value they expected and
        // silently ignore the rest.
        // Only the always-wrong bits. Illegal-command and erase-reset are
        // answers a caller may be probing for (CMD8 on a v1 card, CMD59 on a
        // card that refuses CRC mode) -- rejecting those here turned a normal
        // negotiation into an init failure.
        if let Some(e) = R1Status(byte).to_hard_error() {
            error!("sdspi: cmd {} R1=0x{:02x}: {:?}", cmd.cmd, byte, e);
            return Err(e);
        }

        Ok(byte)
    }

    async fn acmd<R: Resp>(&self, spi: &mut A::Bus, cmd: Cmd<R>) -> Result<u8, Error> {
        self.cmd(spi, app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16))
            .await?;
        self.cmd(spi, cmd).await
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
    /// Interleaving variant: one probe per acquire, for second-scale waits
    /// (post-CMD38). Holding across those would freeze the display for a whole
    /// format. Caller must NOT hold a guard.
    async fn wait_idle_reacquiring(&self, timeout_ms: u32, sustained_count: u32) -> Result<(), Error> {
        let target = sustained_count.max(1);
        // Probe counter, reported ONCE after the wait. The loop parks 1 ms, so
        // probes ~= elapsed ms when the card is simply busy; far fewer probes
        // than elapsed ms means the wait is starving on bus acquisition
        // instead. Logging per probe would change what it measures.
        //
        // Atomic, not `Cell`: a `Cell` is not `Sync`, which makes this future
        // non-`Send` and stops the block-device handler being spawnable on a
        // thread — the host integration tests do exactly that.
        let probes = core::sync::atomic::AtomicU32::new(0);
        let outer = with_timeout(self.delay.clone(), timeout_ms, async {
            let mut consec: u32 = 0;
            loop {
                let idle = {
                    probes.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
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
        let probes = probes.load(core::sync::atomic::Ordering::Relaxed);
        if probes > 1_000 {
            debug!(
                "sdspi: idle wait ended after {} probes (bound {} ms, target {})",
                probes,
                timeout_ms,
                target
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


#[cfg(test)]
mod response_tests {
    use super::*;

    // ---- R1 ---------------------------------------------------------------

    #[test]
    fn r1_ready_and_idle_are_not_errors() {
        assert!(R1Status(0x00).ready());
        assert_eq!(R1Status(0x00).to_error(), None);
        assert!(R1Status(0x01).idle());
        assert!(!R1Status(0x01).ready());
        assert_eq!(R1Status(0x01).to_error(), None, "idle alone is a state, not a fault");
    }

    #[test]
    fn r1_with_bit7_set_is_not_a_response_at_all() {
        assert!(!R1Status(0xFF).is_valid());
        assert_eq!(R1Status(0xFF).to_error(), Some(Error::NoResponse));
        assert!(!R1Status(0x80).is_valid());
    }

    #[test]
    fn every_r1_error_bit_decodes_to_its_own_error() {
        let cases = [
            (R1Status::ERASE_RESET, Error::EraseReset),
            (R1Status::ILLEGAL_COMMAND, Error::IllegalCommand),
            (R1Status::COM_CRC_ERROR, Error::CommandCrcError),
            (R1Status::ERASE_SEQUENCE_ERROR, Error::EraseSequenceError),
            (R1Status::ADDRESS_ERROR, Error::AddressError),
            (R1Status::PARAMETER_ERROR, Error::ParameterError),
        ];
        for (bit, want) in cases {
            assert_eq!(R1Status(bit).to_error(), Some(want), "bit {bit:#04x}");
            // Also set alongside IDLE, which is how a card reports during init.
            assert_eq!(R1Status(bit | R1Status::IDLE).to_error(), Some(want),
                       "bit {bit:#04x} with IDLE");
        }
    }

    #[test]
    fn no_error_bit_is_silently_ignored() {
        // Every settable bit except IDLE must produce SOME error. A bit that
        // decodes to None is a fault the driver would swallow.
        for bit in 1..7 {
            let raw = 1u8 << bit;
            assert!(R1Status(raw).to_error().is_some(),
                    "R1 bit {bit} ({raw:#04x}) decodes to no error");
        }
    }

    #[test]
    fn r1_reports_crc_ahead_of_its_consequences() {
        // A bad CRC means the command never ran, so it outranks ADDRESS_ERROR.
        let both = R1Status(R1Status::COM_CRC_ERROR | R1Status::ADDRESS_ERROR);
        assert_eq!(both.to_error(), Some(Error::CommandCrcError));
    }

    #[test]
    fn illegal_command_is_an_answer_not_a_hard_error() {
        // CMD8 identifies a v1 card BY being refused, and some cards refuse
        // CMD59. cmd() must pass these through so the caller can interpret
        // them; rejecting them turned a normal negotiation into an init
        // failure that looked like a dead card.
        let r = R1Status(R1Status::ILLEGAL_COMMAND | R1Status::IDLE);
        assert_eq!(r.to_hard_error(), None);
        assert_eq!(r.to_error(), Some(Error::IllegalCommand), "still decodable on demand");
        assert_eq!(R1Status(R1Status::ERASE_RESET).to_hard_error(), None);
    }

    #[test]
    fn unconditional_faults_are_always_rejected() {
        for (bit, want) in [
            (R1Status::COM_CRC_ERROR, Error::CommandCrcError),
            (R1Status::PARAMETER_ERROR, Error::ParameterError),
            (R1Status::ADDRESS_ERROR, Error::AddressError),
            (R1Status::ERASE_SEQUENCE_ERROR, Error::EraseSequenceError),
        ] {
            assert_eq!(R1Status(bit).to_hard_error(), Some(want), "bit {bit:#04x}");
            assert_eq!(R1Status(bit | R1Status::IDLE).to_hard_error(), Some(want));
        }
        assert_eq!(R1Status(0x00).to_hard_error(), None);
        assert_eq!(R1Status(R1Status::IDLE).to_hard_error(), None);
        assert_eq!(R1Status(0xFF).to_hard_error(), Some(Error::NoResponse));
    }

    #[test]
    fn r1_errors_mask_excludes_idle() {
        assert_eq!(R1Status(0x01).errors(), 0);
        assert_eq!(R1Status(0x09).errors(), R1Status::COM_CRC_ERROR);
    }

    // ---- data response token (write) --------------------------------------

    #[test]
    fn data_response_accepted() {
        // The spec fixes only bits 3..1; the surrounding bits are undefined.
        for pad in [0x00u8, 0xE0, 0x20] {
            let t = DataResponse(pad | DATA_RES_ACCEPTED);
            assert!(t.accepted(), "pad {pad:#04x}");
            assert_eq!(t.to_error(), None);
        }
    }

    #[test]
    fn data_response_crc_and_write_errors_decode() {
        assert_eq!(DataResponse(0x0B).to_error(), Some(Error::DataCrcError));
        assert_eq!(DataResponse(0x0D).to_error(), Some(Error::DataWriteError));
        assert!(!DataResponse(0x0B).accepted());
        assert!(!DataResponse(0x0D).accepted());
    }

    #[test]
    fn undefined_data_response_is_reported_not_swallowed() {
        match DataResponse(0x07).to_error() {
            Some(Error::UnknownDataResponse(0x07)) => {}
            other => panic!("undefined token must surface, got {other:?}"),
        }
    }

    // ---- data error token (read) ------------------------------------------

    // ---- CRC7 (command integrity) -----------------------------------------
    //
    // Init sends CMD59 with arg 1, which turns CRC CHECKING ON. From that point
    // a command whose CRC7 is wrong is IGNORED BY THE CARD -- no R1, no error,
    // just silence. So a CRC bug is indistinguishable from a dead card, and
    // only shows up on cards that actually enforce it.

    fn cmd_bytes(cmd: u8, arg: u32) -> [u8; 5] {
        [0x40 | cmd, (arg >> 24) as u8, (arg >> 16) as u8, (arg >> 8) as u8, arg as u8]
    }

    #[test]
    fn crc7_matches_the_spec_reference_vectors() {
        // Values fixed by the SD Physical Layer spec / universally published
        // reference frames; the trailing bit is the always-1 stop bit.
        assert_eq!(crc7(&cmd_bytes(0, 0x0000_0000)), 0x95, "CMD0");
        assert_eq!(crc7(&cmd_bytes(8, 0x0000_01AA)), 0x87, "CMD8");
        assert_eq!(crc7(&cmd_bytes(55, 0x0000_0000)), 0x65, "CMD55");
        assert_eq!(crc7(&cmd_bytes(41, 0x4000_0000)), 0x77, "ACMD41 HCS=1");
        assert_eq!(crc7(&cmd_bytes(58, 0x0000_0000)), 0xFD, "CMD58");
    }

    #[test]
    fn crc7_always_sets_the_stop_bit() {
        for cmd in 0..=63u8 {
            assert_eq!(crc7(&cmd_bytes(cmd, 0)) & 1, 1, "CMD{cmd} stop bit");
        }
    }

    #[test]
    fn crc7_changes_with_the_argument() {
        assert_ne!(crc7(&cmd_bytes(17, 0)), crc7(&cmd_bytes(17, 1)));
    }

    #[test]
    fn data_error_token_bits_decode() {
        assert_eq!(DataErrorToken(0x01).to_error(), Some(Error::ReadError));
        assert_eq!(DataErrorToken(0x02).to_error(), Some(Error::CardControllerError));
        assert_eq!(DataErrorToken(0x04).to_error(), Some(Error::CardEccFailed));
        assert_eq!(DataErrorToken(0x08).to_error(), Some(Error::OutOfRange));
    }

    #[test]
    fn data_error_token_reports_the_most_specific_cause() {
        // Bit 0 is set alongside the specific cause on real cards; the specific
        // one must win, or every read failure looks identical.
        assert_eq!(DataErrorToken(0x09).to_error(), Some(Error::OutOfRange));
        assert_eq!(DataErrorToken(0x05).to_error(), Some(Error::CardEccFailed));
        assert_eq!(DataErrorToken(0x03).to_error(), Some(Error::CardControllerError));
    }

    #[test]
    fn data_start_token_is_not_mistaken_for_an_error_token() {
        // 0xFE starts a data block and 0xFF is idle -- neither has a clear high
        // nibble, so neither may decode as an error.
        assert!(!DataErrorToken(DATA_START_BLOCK).is_error_token());
        assert_eq!(DataErrorToken(DATA_START_BLOCK).to_error(), None);
        assert!(!DataErrorToken(0xFF).is_error_token());
        assert_eq!(DataErrorToken(0xFF).to_error(), None);
        assert!(!DataErrorToken(0x00).is_error_token(), "all-zero is not a token");
    }

    #[test]
    fn no_data_error_bit_is_silently_ignored() {
        for bit in 0..4 {
            let raw = 1u8 << bit;
            assert!(DataErrorToken(raw).to_error().is_some(),
                    "data error bit {bit} ({raw:#04x}) decodes to no error");
        }
    }
}
