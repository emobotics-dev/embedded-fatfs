//! An abstraction of block devices.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]
#![allow(async_fn_in_trait)]

use aligned::Aligned;

/// The buffer-alignment invariant for all `BlockDevice` implementations in
/// this fork: 4 bytes (word). ESP32 PDMA requires word-aligned source/dest
/// addresses for SPI DMA; ARM Cortex-M and RISC-V DMA engines that do
/// word-burst transfers have the same requirement. Host simulators that
/// don't care about alignment receive over-aligned buffers — harmless.
///
/// Upstream `block-device-driver` exposes this as `BlockDevice::Align` for
/// per-impl customisation. We hard-code it instead: in practice everyone
/// wants ≥A4, and the per-impl escape hatch was a footgun (host test
/// helpers were silently A1, leading to UB when those types were fed into
/// DMA-bound consumers via generics).
pub type DmaAlign = aligned::A4;

/// A DMA-safe block buffer: `Aligned<DmaAlign, [u8; SIZE]>`.
///
/// Use this as the element type in `[DmaBlock<SIZE>; N]` arrays passed to
/// `BlockDevice::read`/`write`. See [`DmaAlign`] for why A4.
pub type DmaBlock<const SIZE: usize> = Aligned<DmaAlign, [u8; SIZE]>;

/// A trait for a block devices
///
/// [`BlockDevice<const SIZE: usize>`](BlockDevice) is parameterised on:
///
/// - `const SIZE`: The size of the block in the block device, in bytes.
/// - `type Error`: The error type for the implementation.
///
/// All block buffers are `[DmaBlock<SIZE>]` — see [`DmaAlign`] for the
/// fork's alignment rationale.
///
/// All addresses are zero indexed, and the unit is blocks. For example to read bytes
/// from 1024 to 1536 on a 512 byte block device, the supplied block address would be 2.
///
/// This trait can be implemented multiple times to support various different block sizes.
pub trait BlockDevice<const SIZE: usize> {
    /// The error type for the BlockDevice implementation.
    type Error: core::fmt::Debug;

    /// Read one or more blocks at the given block address.
    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [DmaBlock<SIZE>],
    ) -> Result<(), Self::Error>;

    /// Write one or more blocks at the given block address.
    async fn write(
        &mut self,
        block_address: u32,
        data: &[DmaBlock<SIZE>],
    ) -> Result<(), Self::Error>;

    /// Report the size of the block device in bytes.
    async fn size(&mut self) -> Result<u64, Self::Error>;
}

impl<T: BlockDevice<SIZE>, const SIZE: usize> BlockDevice<SIZE> for &mut T {
    type Error = T::Error;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [DmaBlock<SIZE>],
    ) -> Result<(), Self::Error> {
        (*self).read(block_address, data).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[DmaBlock<SIZE>],
    ) -> Result<(), Self::Error> {
        (*self).write(block_address, data).await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        (*self).size().await
    }
}

/// Stream-level erase — implemented by adapters (e.g. `BufStream`)
/// that wrap a [`BlockDevice`] supporting erase.
///
/// Separate from `BlockDevice` so it can be used as a bound on
/// byte-stream consumers (`Read + Write + Seek + Erase`) without
/// pulling in the block-level generics.
pub trait Erase {
    /// Error type for erase operations.
    type Error: core::fmt::Debug;

    /// Erase the block range `[start_block, end_block]` (inclusive).
    ///
    /// After erase, block content is device-dependent (0x00 or 0xFF).
    /// No-op if the underlying device does not support erase.
    async fn erase_blocks(&mut self, start_block: u32, end_block: u32) -> Result<(), Self::Error>;
}

/// Cast a byte slice to an aligned slice of blocks.
///
/// This function panics if
///
/// * ALIGNment is not a multiple of SIZE
/// * The input slice is not a multiple of SIZE
/// * The input slice does not have the correct alignment.
pub fn slice_to_blocks<ALIGN, const SIZE: usize>(slice: &[u8]) -> &[Aligned<ALIGN, [u8; SIZE]>]
where
    ALIGN: aligned::Alignment,
{
    let align: usize = core::mem::align_of::<Aligned<ALIGN, ()>>();
    assert!(slice.len() % SIZE == 0);
    assert!(slice.len() % align == 0);
    assert!(slice.as_ptr().cast::<u8>() as usize % align == 0);
    // Note unsafe: we check the buf has the correct SIZE and ALIGNment before casting
    unsafe {
        core::slice::from_raw_parts(
            slice.as_ptr() as *const Aligned<ALIGN, [u8; SIZE]>,
            slice.len() / SIZE,
        )
    }
}

/// Cast a mutable byte slice to an aligned mutable slice of blocks.
///
/// This function panics if
///
/// * ALIGNment is not a multiple of SIZE
/// * The input slice is not a multiple of SIZE
/// * The input slice does not have the correct alignment.
pub fn slice_to_blocks_mut<ALIGN, const SIZE: usize>(
    slice: &mut [u8],
) -> &mut [Aligned<ALIGN, [u8; SIZE]>]
where
    ALIGN: aligned::Alignment,
{
    let align: usize = core::mem::align_of::<Aligned<ALIGN, [u8; SIZE]>>();
    assert!(slice.len() % SIZE == 0);
    assert!(slice.len() % align == 0);
    assert!(slice.as_ptr().cast::<u8>() as usize % align == 0);
    // Note unsafe: we check the buf has the correct SIZE and ALIGNment before casting
    unsafe {
        core::slice::from_raw_parts_mut(
            slice.as_mut_ptr() as *mut Aligned<ALIGN, [u8; SIZE]>,
            slice.len() / SIZE,
        )
    }
}

/// Cast a slice of aligned blocks to a byte slice
///
/// This function panics if
///
/// * ALIGNment is not a multiple of SIZE
pub fn blocks_to_slice<ALIGN, const SIZE: usize>(buf: &[Aligned<ALIGN, [u8; SIZE]>]) -> &[u8]
where
    ALIGN: aligned::Alignment,
{
    // We only need to assert that ALIGN is a multiple of SIZE, the other invariants are checked via the type system.
    // This relationship must be true to avoid padding bytes which will introduce UB when casting.
    let align: usize = core::mem::align_of::<Aligned<ALIGN, ()>>();
    assert!(SIZE % align == 0);
    // Note unsafe: we check the buf has the correct SIZE and ALIGNment before casting
    unsafe { core::slice::from_raw_parts(buf.as_ptr() as *const u8, buf.len() * SIZE) }
}

/// Cast a mutable slice of aligned blocks to a mutable byte slice
///
/// This function panics if
///
/// * ALIGNment is not a multiple of SIZE
pub fn blocks_to_slice_mut<ALIGN, const SIZE: usize>(
    buf: &mut [Aligned<ALIGN, [u8; SIZE]>],
) -> &mut [u8]
where
    ALIGN: aligned::Alignment,
{
    // We only need to assert that ALIGN is a multiple of SIZE, the other invariants are checked via the type system.
    // This relationship must be true to avoid padding bytes which will introduce UB when casting.
    let align: usize = core::mem::align_of::<Aligned<ALIGN, ()>>();
    assert!(SIZE % align == 0);
    // Note unsafe: we check the buf has the correct SIZE and ALIGNment before casting
    unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, buf.len() * SIZE) }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_conversion_round_trip() {
        let blocks = &mut [
            Aligned::<aligned::A4, _>([0; 512]),
            Aligned::<aligned::A4, _>([0; 512]),
        ];
        let slice = blocks_to_slice_mut(blocks);
        assert!(slice.len() == 1024);
        let blocks: &mut [Aligned<aligned::A4, [u8; 512]>] = slice_to_blocks_mut(slice);
        assert!(blocks.len() == 2);
    }
}
