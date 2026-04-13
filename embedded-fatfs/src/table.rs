use core::borrow::BorrowMut;
use core::cmp;
use core::marker::PhantomData;

use crate::error::{Error, IoError, ReadExactError};
use crate::fs::{FatType, FsStatusFlags};
use crate::io::{self, IoBase, Read, ReadLeExt, Seek, SeekFrom, Write, WriteLeExt};

struct Fat<S> {
    phantom: PhantomData<S>,
}

type Fat12 = Fat<u8>;
type Fat16 = Fat<u16>;
type Fat32 = Fat<u32>;

pub const RESERVED_FAT_ENTRIES: u32 = 2;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum FatValue {
    Free,
    Data(u32),
    Bad,
    EndOfChain,
}

trait FatTrait {
    async fn get_raw<S, E>(fat: &mut S, cluster: u32) -> Result<u32, Error<E>>
    where
        S: Read + Seek + IoBase,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>;

    async fn get<S, E>(fat: &mut S, cluster: u32) -> Result<FatValue, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>;

    async fn set_raw<S, E>(fat: &mut S, cluster: u32, raw_value: u32) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>;

    async fn set<S, E>(fat: &mut S, cluster: u32, value: FatValue) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>;

    async fn find_free<S, E>(fat: &mut S, start_cluster: u32, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>;

    async fn count_free<S, E>(fat: &mut S, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>;
}

async fn read_fat<S, E>(fat: &mut S, fat_type: FatType, cluster: u32) -> Result<FatValue, Error<E>>
where
    S: Read + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    match fat_type {
        FatType::Fat12 => Fat12::get(fat, cluster).await,
        FatType::Fat16 => Fat16::get(fat, cluster).await,
        FatType::Fat32 => Fat32::get(fat, cluster).await,
    }
}

async fn write_fat<S, E>(fat: &mut S, fat_type: FatType, cluster: u32, value: FatValue) -> Result<(), Error<E>>
where
    S: Read + Write + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    trace!("write FAT - cluster {} value {:?}", cluster, value);
    match fat_type {
        FatType::Fat12 => Fat12::set(fat, cluster, value).await,
        FatType::Fat16 => Fat16::set(fat, cluster, value).await,
        FatType::Fat32 => Fat32::set(fat, cluster, value).await,
    }
}

async fn get_next_cluster<S, E>(fat: &mut S, fat_type: FatType, cluster: u32) -> Result<Option<u32>, Error<E>>
where
    S: Read + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    let val = read_fat(fat, fat_type, cluster).await?;
    match val {
        FatValue::Data(n) => {
            // Clusters 0 and 1 are reserved in the FAT specification and must never
            // appear as data cluster references. Treat them as filesystem corruption.
            if n < 2 {
                return Err(Error::CorruptedFileSystem);
            }
            Ok(Some(n))
        }
        _ => Ok(None),
    }
}

/// Check if a FAT entry value represents a free cluster.
/// When `erased_byte` is 0xFF, the all-ones value for each FAT type
/// (0xFFF / 0xFFFF / 0x0FFFFFFF) is also treated as free — these are
/// unwritten entries on a device that erases to 0xFF.
fn is_free_fat12(val: u16, erased_byte: u8) -> bool {
    val == 0 || (erased_byte == 0xFF && val == 0x0FFF)
}
fn is_free_fat16(val: u16, erased_byte: u8) -> bool {
    val == 0 || (erased_byte == 0xFF && val == 0xFFFF)
}
fn is_free_fat32(val: u32, erased_byte: u8) -> bool {
    val == 0 || (erased_byte == 0xFF && val == 0x0FFF_FFFF)
}

async fn find_free_cluster<S, E>(
    fat: &mut S,
    fat_type: FatType,
    start_cluster: u32,
    end_cluster: u32,
    erased_byte: u8,
) -> Result<u32, Error<E>>
where
    S: Read + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    match fat_type {
        FatType::Fat12 => Fat12::find_free(fat, start_cluster, end_cluster, erased_byte).await,
        FatType::Fat16 => Fat16::find_free(fat, start_cluster, end_cluster, erased_byte).await,
        FatType::Fat32 => Fat32::find_free(fat, start_cluster, end_cluster, erased_byte).await,
    }
}

pub(crate) async fn alloc_cluster<S, E>(
    fat: &mut S,
    fat_type: FatType,
    prev_cluster: Option<u32>,
    hint: Option<u32>,
    total_clusters: u32,
    erased_byte: u8,
) -> Result<u32, Error<E>>
where
    S: Read + Write + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    let end_cluster = total_clusters + RESERVED_FAT_ENTRIES;
    let start_cluster = match hint {
        Some(n) if n < end_cluster => n,
        _ => RESERVED_FAT_ENTRIES,
    };
    let new_cluster = match find_free_cluster(fat, fat_type, start_cluster, end_cluster, erased_byte).await {
        Ok(n) => n,
        Err(_) if start_cluster > RESERVED_FAT_ENTRIES => {
            find_free_cluster(fat, fat_type, RESERVED_FAT_ENTRIES, start_cluster, erased_byte).await?
        }
        Err(e) => return Err(e),
    };
    write_fat(fat, fat_type, new_cluster, FatValue::EndOfChain).await?;
    if let Some(n) = prev_cluster {
        write_fat(fat, fat_type, n, FatValue::Data(new_cluster)).await?;
    }
    trace!("allocated cluster {}", new_cluster);
    Ok(new_cluster)
}

pub(crate) async fn read_fat_flags<S, E>(fat: &mut S, fat_type: FatType) -> Result<FsStatusFlags, Error<E>>
where
    S: Read + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    // check MSB (except in FAT12)
    let val = match fat_type {
        FatType::Fat12 => 0xFFF,
        FatType::Fat16 => Fat16::get_raw(fat, 1).await?,
        FatType::Fat32 => Fat32::get_raw(fat, 1).await?,
    };
    let dirty = match fat_type {
        FatType::Fat12 => false,
        FatType::Fat16 => val & (1 << 15) == 0,
        FatType::Fat32 => val & (1 << 27) == 0,
    };
    let io_error = match fat_type {
        FatType::Fat12 => false,
        FatType::Fat16 => val & (1 << 14) == 0,
        FatType::Fat32 => val & (1 << 26) == 0,
    };
    Ok(FsStatusFlags { dirty, io_error })
}

pub(crate) async fn count_free_clusters<S, E>(
    fat: &mut S,
    fat_type: FatType,
    total_clusters: u32,
    erased_byte: u8,
) -> Result<u32, Error<E>>
where
    S: Read + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    let end_cluster = total_clusters + RESERVED_FAT_ENTRIES;
    match fat_type {
        FatType::Fat12 => Fat12::count_free(fat, end_cluster, erased_byte).await,
        FatType::Fat16 => Fat16::count_free(fat, end_cluster, erased_byte).await,
        FatType::Fat32 => Fat32::count_free(fat, end_cluster, erased_byte).await,
    }
}

pub(crate) async fn format_fat<S, E, F>(
    fat: &mut S,
    fat_type: FatType,
    media: u8,
    bytes_per_fat: u64,
    _total_clusters: u32,
    pre_erased: bool,
    mut progress: F,
) -> Result<(), Error<E>>
where
    S: Read + Write + Seek,
    E: IoError,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    F: FnMut(u64, u64),
{
    // Write the first 512-byte block as a complete unit: reserved
    // entries at the front, zero-padded to a full block. This keeps
    // the stream block-aligned so every subsequent write_all hits
    // BufStream's fast path (direct CMD25 multi-block, no
    // read-before-write). Previously writing 8 bytes of reserved
    // entries first left the stream at offset 8, which forced EVERY
    // subsequent 8 KiB chunk through the slow path (read + modify +
    // flush per 512-byte block) — turning a 12-second operation into
    // 6 minutes.
    let mut first_block = [0u8; 512];
    match fat_type {
        FatType::Fat12 => {
            first_block[0] = media;
            first_block[1] = 0xFF;
            first_block[2] = 0xFF;
        }
        FatType::Fat16 => {
            let e0 = (u16::from(media) | 0xFF00).to_le_bytes();
            let e1 = 0xFFFFu16.to_le_bytes();
            first_block[0..2].copy_from_slice(&e0);
            first_block[2..4].copy_from_slice(&e1);
        }
        FatType::Fat32 => {
            let e0 = (u32::from(media) | 0x0FFF_FF00).to_le_bytes();
            let e1 = 0x0FFF_FFFFu32.to_le_bytes();
            first_block[0..4].copy_from_slice(&e0);
            first_block[4..8].copy_from_slice(&e1);
        }
    };
    fat.write_all(&first_block).await?;

    let zero_total = bytes_per_fat - 512;

    if pre_erased {
        // Device was erased before format — skip zero-fill entirely.
        // Seek past the FAT body so the stream position is correct
        // for the caller.
        fat.seek(SeekFrom::Current(zero_total as i64)).await?;
        trace!("fmt: fat_fill skipped (pre_erased), {} bytes", zero_total);
        progress(zero_total, zero_total);
    } else {
        // Fill the rest of the FAT with zeros in 8 KiB chunks.
        // Stream is now block-aligned → every chunk hits BufStream's
        // fast path → single CMD25 multi-block write per chunk.
        const ZEROS_CHUNK: [u8; 8192] = [0_u8; 8192];
        let mut to_write = zero_total;
        let mut last_log = 0_u64;
        const LOG_STEP: u64 = 512 * 1024;
        progress(0, zero_total);
        while to_write > 0 {
            let chunk = cmp::min(to_write, ZEROS_CHUNK.len() as u64) as usize;
            fat.write_all(&ZEROS_CHUNK[..chunk]).await?;
            to_write -= chunk as u64;
            let done = zero_total - to_write;
            if done - last_log >= LOG_STEP {
                trace!("fmt: fat_fill {} / {}", done, zero_total);
                last_log = done;
            }
            progress(done, zero_total);
        }
        trace!("fmt: fat_fill done {} bytes", zero_total + 512);
    }
    // Tail-padding entries (start_cluster..end_cluster) and
    // BAD-range markers are intentionally skipped. These entries
    // sit beyond total_clusters + RESERVED_FAT_ENTRIES and are
    // never reached by any alloc/free/lookup path. The fat_fill
    // loop above already zeroed them, which means they read as
    // "free" rather than "EndOfChain" — functionally equivalent
    // because the allocator stops at total_clusters.
    //
    // The per-entry write_fat loop that used to live here caused
    // mirror-write thrashing (fat_slice seeks between FAT1 and
    // FAT2 for every 4-byte entry), which after 12 min of
    // sustained I/O pushed the SD card past its programming
    // timeout and triggered a BD Handler: Write error: Timeout.
    Ok(())
}

impl FatTrait for Fat12 {
    async fn get_raw<S, E>(fat: &mut S, cluster: u32) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let fat_offset = cluster + (cluster / 2);
        fat.seek(io::SeekFrom::Start(u64::from(fat_offset))).await?;
        let packed_val = fat.read_u16_le().await?;
        Ok(u32::from(match cluster & 1 {
            0 => packed_val & 0x0FFF,
            _ => packed_val >> 4,
        }))
    }

    async fn get<S, E>(fat: &mut S, cluster: u32) -> Result<FatValue, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let val = Self::get_raw(fat, cluster).await?;
        Ok(match val {
            0 => FatValue::Free,
            0xFF7 => FatValue::Bad,
            0xFF8..=0xFFF => FatValue::EndOfChain,
            n => FatValue::Data(n),
        })
    }

    async fn set<S, E>(fat: &mut S, cluster: u32, value: FatValue) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let raw_val = match value {
            FatValue::Free => 0,
            FatValue::Bad => 0xFF7,
            FatValue::EndOfChain => 0xFFF,
            FatValue::Data(n) => n,
        };
        Self::set_raw(fat, cluster, raw_val).await
    }

    async fn set_raw<S, E>(fat: &mut S, cluster: u32, raw_val: u32) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek + IoBase,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let fat_offset = cluster + (cluster / 2);
        fat.seek(io::SeekFrom::Start(u64::from(fat_offset))).await?;
        let old_packed = fat.read_u16_le().await?;
        fat.seek(io::SeekFrom::Start(u64::from(fat_offset))).await?;
        let new_packed = match cluster & 1 {
            0 => (old_packed & 0xF000) | raw_val as u16,
            _ => (old_packed & 0x000F) | ((raw_val as u16) << 4),
        };
        fat.write_u16_le(new_packed).await?;
        Ok(())
    }

    async fn find_free<S, E>(fat: &mut S, start_cluster: u32, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let mut cluster = start_cluster;
        let fat_offset = cluster + (cluster / 2);
        fat.seek(io::SeekFrom::Start(u64::from(fat_offset))).await?;
        let mut packed_val = fat.read_u16_le().await?;
        loop {
            let val = match cluster & 1 {
                0 => packed_val & 0x0FFF,
                _ => packed_val >> 4,
            };
            if is_free_fat12(val, erased_byte) {
                return Ok(cluster);
            }
            cluster += 1;
            if cluster == end_cluster {
                return Err(Error::NotEnoughSpace);
            }
            packed_val = if cluster & 1 == 0 {
                fat.read_u16_le().await?
            } else {
                let next_byte = fat.read_u8().await?;
                (packed_val >> 8) | (u16::from(next_byte) << 8)
            };
        }
    }

    async fn count_free<S, E>(fat: &mut S, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let mut count = 0;
        let mut cluster = RESERVED_FAT_ENTRIES;
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 3 / 2))).await?;
        let mut prev_packed_val = 0_u16;
        while cluster < end_cluster {
            let res = match cluster & 1 {
                0 => fat.read_u16_le().await,
                _ => fat.read_u8().await.map(u16::from),
            };
            let packed_val = match res {
                Err(err) => return Err(err.into()),
                Ok(n) => n,
            };
            let val = match cluster & 1 {
                0 => packed_val & 0x0FFF,
                _ => (packed_val << 8) | (prev_packed_val >> 12),
            };
            prev_packed_val = packed_val;
            if is_free_fat12(val, erased_byte) {
                count += 1;
            }
            cluster += 1;
        }
        Ok(count)
    }
}

impl FatTrait for Fat16 {
    async fn get_raw<S, E>(fat: &mut S, cluster: u32) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 2))).await?;
        Ok(u32::from(fat.read_u16_le().await?))
    }

    async fn get<S, E>(fat: &mut S, cluster: u32) -> Result<FatValue, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let val = Self::get_raw(fat, cluster).await?;
        Ok(match val {
            0 => FatValue::Free,
            0xFFF7 => FatValue::Bad,
            0xFFF8..=0xFFFF => FatValue::EndOfChain,
            n => FatValue::Data(n),
        })
    }

    async fn set<S, E>(fat: &mut S, cluster: u32, value: FatValue) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let raw_value = match value {
            FatValue::Free => 0,
            FatValue::Bad => 0xFFF7,
            FatValue::EndOfChain => 0xFFFF,
            FatValue::Data(n) => n,
        };
        Self::set_raw(fat, cluster, raw_value).await
    }

    async fn find_free<S, E>(fat: &mut S, start_cluster: u32, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let mut cluster = start_cluster;
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 2))).await?;
        while cluster < end_cluster {
            let val = fat.read_u16_le().await?;
            if is_free_fat16(val, erased_byte) {
                return Ok(cluster);
            }
            cluster += 1;
        }
        Err(Error::NotEnoughSpace)
    }

    async fn count_free<S, E>(fat: &mut S, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let mut count = 0;
        let mut cluster = RESERVED_FAT_ENTRIES;
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 2))).await?;
        while cluster < end_cluster {
            let val = fat.read_u16_le().await?;
            if is_free_fat16(val, erased_byte) {
                count += 1;
            }
            cluster += 1;
        }
        Ok(count)
    }

    async fn set_raw<S, E>(fat: &mut S, cluster: u32, raw_value: u32) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 2))).await?;
        fat.write_u16_le(raw_value as u16).await?;
        Ok(())
    }
}

impl FatTrait for Fat32 {
    async fn get_raw<S, E>(fat: &mut S, cluster: u32) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 4))).await?;
        Ok(fat.read_u32_le().await?)
    }

    async fn get<S, E>(fat: &mut S, cluster: u32) -> Result<FatValue, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let val = Self::get_raw(fat, cluster).await? & 0x0FFF_FFFF;
        Ok(match val {
            0 if (0x0FFF_FFF7..=0x0FFF_FFFF).contains(&cluster) => {
                let tmp = if cluster == 0x0FFF_FFF7 {
                    "BAD_CLUSTER"
                } else {
                    "end-of-chain"
                };
                warn!(
                    "cluster number {} is a special value in FAT to indicate {}; it should never be seen as free",
                    cluster, tmp
                );
                FatValue::Bad // avoid accidental use or allocation into a FAT chain
            }
            0 => FatValue::Free,
            0x0FFF_FFF7 => FatValue::Bad,
            0x0FFF_FFF8..=0x0FFF_FFFF => FatValue::EndOfChain,
            n if (0x0FFF_FFF7..=0x0FFF_FFFF).contains(&cluster) => {
                let tmp = if cluster == 0x0FFF_FFF7 {
                    "BAD_CLUSTER"
                } else {
                    "end-of-chain"
                };
                warn!("cluster number {} is a special value in FAT to indicate {}; hiding potential FAT chain value {} and instead reporting as a bad sector", cluster, tmp, n);
                FatValue::Bad // avoid accidental use or allocation into a FAT chain
            }
            n => FatValue::Data(n),
        })
    }

    async fn set<S, E>(fat: &mut S, cluster: u32, value: FatValue) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let old_reserved_bits = Self::get_raw(fat, cluster).await? & 0xF000_0000;

        if value == FatValue::Free && (0x0FFF_FFF7..=0x0FFF_FFFF).contains(&cluster) {
            // NOTE: it is technically allowed for them to store FAT chain loops,
            //       or even have them all store value '4' as their next cluster.
            //       Some believe only FatValue::Bad should be allowed for this edge case.
            let tmp = if cluster == 0x0FFF_FFF7 {
                "BAD_CLUSTER"
            } else {
                "end-of-chain"
            };
            panic!(
                "cluster number {} is a special value in FAT to indicate {}; it should never be set as free",
                cluster, tmp
            );
        };
        let raw_val = match value {
            FatValue::Free => 0,
            FatValue::Bad => 0x0FFF_FFF7,
            FatValue::EndOfChain => 0x0FFF_FFFF,
            FatValue::Data(n) => n,
        };
        let raw_val = raw_val | old_reserved_bits; // must preserve original reserved values
        Self::set_raw(fat, cluster, raw_val).await
    }

    async fn find_free<S, E>(fat: &mut S, start_cluster: u32, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let mut cluster = start_cluster;
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 4))).await?;
        while cluster < end_cluster {
            let val = fat.read_u32_le().await? & 0x0FFF_FFFF;
            if is_free_fat32(val, erased_byte) {
                return Ok(cluster);
            }
            cluster += 1;
        }
        Err(Error::NotEnoughSpace)
    }

    async fn count_free<S, E>(fat: &mut S, end_cluster: u32, erased_byte: u8) -> Result<u32, Error<E>>
    where
        S: Read + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        let mut count = 0;
        let mut cluster = RESERVED_FAT_ENTRIES;
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 4))).await?;
        while cluster < end_cluster {
            let val = fat.read_u32_le().await? & 0x0FFF_FFFF;
            if is_free_fat32(val, erased_byte) {
                count += 1;
            }
            cluster += 1;
        }
        Ok(count)
    }

    async fn set_raw<S, E>(fat: &mut S, cluster: u32, raw_value: u32) -> Result<(), Error<E>>
    where
        S: Read + Write + Seek,
        E: IoError,
        Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
    {
        fat.seek(io::SeekFrom::Start(u64::from(cluster * 4))).await?;
        fat.write_u32_le(raw_value).await?;
        Ok(())
    }
}

pub(crate) struct ClusterIterator<B, E, S = B> {
    fat: B,
    fat_type: FatType,
    cluster: Option<u32>,
    err: bool,
    // phantom is needed to add type bounds on the storage type
    phantom_s: PhantomData<S>,
    phantom_e: PhantomData<E>,
}

impl<B, E, S> ClusterIterator<B, E, S>
where
    B: BorrowMut<S>,
    E: IoError,
    S: Read + Write + Seek,
    Error<E>: From<S::Error> + From<ReadExactError<S::Error>>,
{
    pub(crate) fn new(fat: B, fat_type: FatType, cluster: u32) -> Self {
        Self {
            fat,
            fat_type,
            cluster: Some(cluster),
            err: false,
            phantom_s: PhantomData,
            phantom_e: PhantomData,
        }
    }

    pub(crate) async fn truncate(&mut self) -> Result<u32, Error<E>> {
        if let Some(n) = self.cluster {
            // Move to the next cluster
            self.next().await;
            // Mark previous cluster as end of chain
            write_fat(self.fat.borrow_mut(), self.fat_type, n, FatValue::EndOfChain).await?;
            // Free rest of chain
            self.free().await
        } else {
            Ok(0)
        }
    }

    pub(crate) async fn free(&mut self) -> Result<u32, Error<E>> {
        let mut num_free = 0;
        while let Some(n) = self.cluster {
            self.next().await;
            write_fat(self.fat.borrow_mut(), self.fat_type, n, FatValue::Free).await?;
            num_free += 1;
        }
        Ok(num_free)
    }

    pub async fn next(&mut self) -> Option<Result<u32, Error<E>>> {
        if self.err {
            return None;
        }
        if let Some(current_cluster) = self.cluster {
            self.cluster = match get_next_cluster(self.fat.borrow_mut(), self.fat_type, current_cluster).await {
                Ok(next_cluster) => next_cluster,
                Err(err) => {
                    self.err = true;
                    return Some(Err(err));
                }
            }
        }
        self.cluster.map(Ok)
    }
}

#[cfg(test)]
mod tests {
    use embedded_io_adapters::tokio_1::FromTokio;

    use super::*;
    use std::io::Cursor;

    async fn test_fat<S: Read + Write + Seek + IoBase>(fat_type: FatType, mut cur: S) {
        // based on cluster maps from Wikipedia:
        // https://en.wikipedia.org/wiki/Design_of_the_FAT_file_system#Cluster_map
        assert_eq!(read_fat(&mut cur, fat_type, 1).await.ok(), Some(FatValue::EndOfChain));
        assert_eq!(read_fat(&mut cur, fat_type, 4).await.ok(), Some(FatValue::Data(5)));
        assert_eq!(read_fat(&mut cur, fat_type, 5).await.ok(), Some(FatValue::Data(6)));
        assert_eq!(read_fat(&mut cur, fat_type, 8).await.ok(), Some(FatValue::EndOfChain));
        assert_eq!(read_fat(&mut cur, fat_type, 9).await.ok(), Some(FatValue::Data(0xA)));
        assert_eq!(read_fat(&mut cur, fat_type, 0xA).await.ok(), Some(FatValue::Data(0x14)));
        assert_eq!(read_fat(&mut cur, fat_type, 0x12).await.ok(), Some(FatValue::Free));
        assert_eq!(read_fat(&mut cur, fat_type, 0x17).await.ok(), Some(FatValue::Bad));
        assert_eq!(read_fat(&mut cur, fat_type, 0x18).await.ok(), Some(FatValue::Bad));
        assert_eq!(read_fat(&mut cur, fat_type, 0x1B).await.ok(), Some(FatValue::Free));

        assert_eq!(find_free_cluster(&mut cur, fat_type, 2, 0x20, 0).await.ok(), Some(0x12));
        assert_eq!(find_free_cluster(&mut cur, fat_type, 0x12, 0x20, 0).await.ok(), Some(0x12));
        assert_eq!(find_free_cluster(&mut cur, fat_type, 0x13, 0x20, 0).await.ok(), Some(0x1B));
        assert!(find_free_cluster(&mut cur, fat_type, 0x13, 0x14, 0).await.is_err());

        assert_eq!(count_free_clusters(&mut cur, fat_type, 0x1E, 0).await.ok(), Some(5));

        // test allocation
        assert_eq!(
            alloc_cluster(&mut cur, fat_type, None, Some(0x13), 0x1E, 0).await.ok(),
            Some(0x1B)
        );
        assert_eq!(
            read_fat(&mut cur, fat_type, 0x1B).await.ok(),
            Some(FatValue::EndOfChain)
        );
        assert_eq!(
            alloc_cluster(&mut cur, fat_type, Some(0x1B), None, 0x1E, 0).await.ok(),
            Some(0x12)
        );
        assert_eq!(
            read_fat(&mut cur, fat_type, 0x1B).await.ok(),
            Some(FatValue::Data(0x12))
        );
        assert_eq!(
            read_fat(&mut cur, fat_type, 0x12).await.ok(),
            Some(FatValue::EndOfChain)
        );
        assert_eq!(count_free_clusters(&mut cur, fat_type, 0x1E, 0).await.ok(), Some(3));
        // test reading from iterator
        {
            let mut iter = ClusterIterator::<&mut S, S::Error, S>::new(&mut cur, fat_type, 0x9);
            let actual_cluster_numbers = {
                let mut v = Vec::new();
                while let Some(i) = iter.next().await {
                    v.push(i.ok())
                }

                v
            };
            let expected_cluster_numbers = [0xA_u32, 0x14_u32, 0x15_u32, 0x16_u32, 0x19_u32, 0x1A_u32]
                .iter()
                .cloned()
                .map(Some)
                .collect::<Vec<_>>();
            assert_eq!(actual_cluster_numbers, expected_cluster_numbers);
        }
        // test truncating a chain
        {
            let mut iter = ClusterIterator::<&mut S, S::Error, S>::new(&mut cur, fat_type, 0x9);
            iter.next().await;
            iter.next().await;
            iter.next().await;
            let value = iter.next().await.unwrap().ok();
            assert_eq!(value, Some(0x16));
            assert!(iter.truncate().await.is_ok());
        }
        assert_eq!(
            read_fat(&mut cur, fat_type, 0x16).await.ok(),
            Some(FatValue::EndOfChain)
        );
        assert_eq!(read_fat(&mut cur, fat_type, 0x19).await.ok(), Some(FatValue::Free));
        assert_eq!(read_fat(&mut cur, fat_type, 0x1A).await.ok(), Some(FatValue::Free));
        // test freeing a chain
        {
            let mut iter = ClusterIterator::<&mut S, S::Error, S>::new(&mut cur, fat_type, 0x9);
            assert!(iter.free().await.is_ok());
        }
        assert_eq!(read_fat(&mut cur, fat_type, 0x9).await.ok(), Some(FatValue::Free));
        assert_eq!(read_fat(&mut cur, fat_type, 0xA).await.ok(), Some(FatValue::Free));
        assert_eq!(read_fat(&mut cur, fat_type, 0x14).await.ok(), Some(FatValue::Free));
        assert_eq!(read_fat(&mut cur, fat_type, 0x15).await.ok(), Some(FatValue::Free));
        assert_eq!(read_fat(&mut cur, fat_type, 0x16).await.ok(), Some(FatValue::Free));
    }

    #[tokio::test]
    async fn test_fat12() {
        let fat: Vec<u8> = vec![
            0xF0, 0xFF, 0xFF, 0x03, 0x40, 0x00, 0x05, 0x60, 0x00, 0x07, 0x80, 0x00, 0xFF, 0xAF, 0x00, 0x14, 0xC0, 0x00,
            0x0D, 0xE0, 0x00, 0x0F, 0x00, 0x01, 0x11, 0xF0, 0xFF, 0x00, 0xF0, 0xFF, 0x15, 0x60, 0x01, 0x19, 0x70, 0xFF,
            0xF7, 0xAF, 0x01, 0xFF, 0x0F, 0x00, 0x00, 0x70, 0xFF, 0x00, 0x00, 0x00,
        ];
        test_fat(FatType::Fat12, FromTokio::new(Cursor::<Vec<u8>>::new(fat))).await;
    }

    #[tokio::test]
    async fn test_fat16() {
        let fat: Vec<u8> = vec![
            0xF0, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0xFF, 0xFF,
            0x0A, 0x00, 0x14, 0x00, 0x0C, 0x00, 0x0D, 0x00, 0x0E, 0x00, 0x0F, 0x00, 0x10, 0x00, 0x11, 0x00, 0xFF, 0xFF,
            0x00, 0x00, 0xFF, 0xFF, 0x15, 0x00, 0x16, 0x00, 0x19, 0x00, 0xF7, 0xFF, 0xF7, 0xFF, 0x1A, 0x00, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0xF7, 0xFF, 0x00, 0x00, 0x00, 0x00,
        ];
        test_fat(FatType::Fat16, FromTokio::new(Cursor::<Vec<u8>>::new(fat))).await;
    }

    #[tokio::test]
    async fn test_fat32() {
        let fat: Vec<u8> = vec![
            0xF0, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0x0F, 0x04, 0x00, 0x00, 0x00, 0x05, 0x00,
            0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F,
            0x0A, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x0E, 0x00,
            0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F,
            0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F, 0x15, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x19, 0x00,
            0x00, 0x00, 0xF7, 0xFF, 0xFF, 0x0F, 0xF7, 0xFF, 0xFF, 0x0F, 0x1A, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        test_fat(FatType::Fat32, FromTokio::new(Cursor::<Vec<u8>>::new(fat))).await;
    }
}
