#![allow(dead_code)]
// Meteor-M N2-4 LRPT Signal Parameters

// QPSK Modulation
pub const SYMBOL_RATE: u32 = 72_000; // 72 ksps QPSK
pub const SAMPLES_PER_SYMBOL: u32 = 4; // Oversampling factor
pub const SAMPLE_RATE: u32 = SYMBOL_RATE * SAMPLES_PER_SYMBOL; // 288000 Hz

// Image Parameters (MSU-MR instrument)
pub const IMAGE_WIDTH: usize = 1568; // Pixels per line
pub const MCU_SIZE: usize = 8; // JPEG MCU block size
pub const MCUS_PER_ROW: usize = IMAGE_WIDTH / MCU_SIZE; // 196 MCUs per row

// CCSDS Transfer Frame
pub const VCDU_HEADER_SIZE: usize = 6;
pub const VCDU_INSERT_ZONE_SIZE: usize = 2; // AOS insert zone before MPDU header
pub const MPDU_HEADER_SIZE: usize = 2;
pub const VCDU_DATA_SIZE: usize = 892; // Total VCDU size (header + data)
pub const MPDU_DATA_SIZE: usize =
    VCDU_DATA_SIZE - VCDU_HEADER_SIZE - VCDU_INSERT_ZONE_SIZE - MPDU_HEADER_SIZE; // 882
pub const CCSDS_ASM: [u8; 4] = [0x1A, 0xCF, 0xFC, 0x1D]; // Attached Sync Marker

// Reed-Solomon (255,223) CCSDS Standard
pub const RS_N: usize = 255;
pub const RS_K: usize = 223;
pub const RS_2T: usize = RS_N - RS_K; // 32 parity symbols
pub const RS_INTERLEAVE: usize = 4;
pub const RS_PARITY_TOTAL: usize = RS_2T * RS_INTERLEAVE; // 128 bytes
pub const RS_PRIM_POLY: u16 = 0x187; // x^8 + x^7 + x^2 + x + 1
pub const RS_FIRST_ROOT: u8 = 112; // First consecutive root index
pub const RS_PRIM: u8 = 11; // Root step (CCSDS convention)
pub const CODED_FRAME_SIZE: usize = VCDU_DATA_SIZE + RS_PARITY_TOTAL; // 1020
pub const CADU_SIZE: usize = CODED_FRAME_SIZE + CCSDS_ASM.len(); // 1024

// Convolutional Code (Rate 1/2, K=7)
pub const CONV_G1: u8 = 0x79; // 171 octal = 1111001 binary
pub const CONV_G2: u8 = 0x5B; // 133 octal = 1011011 binary

// QPSK Root Raised Cosine Filter
pub const RRC_ROLLOFF: f32 = 0.5;
pub const RRC_SPAN: usize = 33; // Filter span in symbols

// Meteor-M N2-4 Defaults
pub const DEFAULT_SCID: u8 = 57; // Spacecraft ID (configurable)
pub const DEFAULT_VCID: u8 = 5; // Virtual Channel ID for LRPT

// MSU-MR APIDs (channels 1-6)
pub const APID_CHANNEL_1: u16 = 64;
pub const APID_CHANNEL_2: u16 = 65;
pub const APID_CHANNEL_3: u16 = 66;

// Default SDR settings
pub const DEFAULT_FREQ_HZ: u64 = 433_000_000;

// JPEG Compression
pub const JPEG_DEFAULT_QUALITY: u8 = 80;

// Standard JPEG luminance quantization table
pub const JPEG_LUMA_QT: [u8; 64] = [
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55, 14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62, 18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113,
    92, 49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99,
];

// LRPT coefficient index mapping used by SatDump's MSU-MR decoder.
// This maps row-major DCT coefficient index -> entropy-coded sequence index.
pub const ZIGZAG: [usize; 64] = [
    0, 1, 5, 6, 14, 15, 27, 28, 2, 4, 7, 13, 16, 26, 29, 42, 3, 8, 12, 17, 25, 30, 41, 43, 9, 11,
    18, 24, 31, 40, 44, 53, 10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60, 21, 34,
    37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63,
];
