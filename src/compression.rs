/// Meteor LRPT JPEG-like image compression
///
/// Implements the compression chain used by Meteor-M satellites:
/// 8x8 MCU → Level shift → Forward DCT → Quantization → Zigzag → Huffman coding
///
/// Uses standard JPEG luminance quantization and Huffman tables.
/// DC coefficients use differential coding (reset per MCU row/segment).
use crate::constants::{JPEG_LUMA_QT, MCU_SIZE, ZIGZAG};
use std::f32::consts::PI;

/// Bit-level writer for Huffman-coded output
pub struct BitWriter {
    buffer: Vec<u8>,
    current_byte: u8,
    bit_pos: u8, // bits written in current byte (0-7)
}

impl BitWriter {
    pub fn new() -> Self {
        BitWriter {
            buffer: Vec::new(),
            current_byte: 0,
            bit_pos: 0,
        }
    }

    /// Write `count` bits from `value` (MSB-first)
    pub fn write_bits(&mut self, value: u32, count: u8) {
        for i in (0..count).rev() {
            let bit = (value >> i) & 1;
            self.current_byte = (self.current_byte << 1) | bit as u8;
            self.bit_pos += 1;
            if self.bit_pos == 8 {
                self.buffer.push(self.current_byte);
                self.current_byte = 0;
                self.bit_pos = 0;
            }
        }
    }

    /// Flush remaining bits (pad with zeros)
    pub fn flush(mut self) -> Vec<u8> {
        if self.bit_pos > 0 {
            self.current_byte <<= 8 - self.bit_pos;
            self.buffer.push(self.current_byte);
        }
        self.buffer
    }
}

/// Scaled quantization table based on quality factor
fn build_quant_table(quality: u8) -> [u16; 64] {
    let quality = quality.clamp(1, 100) as u32;
    let scale = if quality < 50 {
        5000 / quality
    } else {
        200 - 2 * quality
    };

    let mut qt = [0u16; 64];
    for i in 0..64 {
        let val = ((JPEG_LUMA_QT[i] as u32 * scale + 50) / 100).clamp(1, 255);
        qt[i] = val as u16;
    }
    qt
}

/// Forward 2D DCT on an 8x8 block (Type-II)
fn forward_dct(block: &[[i16; MCU_SIZE]; MCU_SIZE]) -> [[f32; MCU_SIZE]; MCU_SIZE] {
    let mut result = [[0.0f32; MCU_SIZE]; MCU_SIZE];

    // Precompute cosine table
    let mut cos_table = [[0.0f32; MCU_SIZE]; MCU_SIZE];
    for k in 0..MCU_SIZE {
        for n in 0..MCU_SIZE {
            cos_table[k][n] = ((2 * n + 1) as f32 * k as f32 * PI / 16.0).cos();
        }
    }

    for u in 0..MCU_SIZE {
        for v in 0..MCU_SIZE {
            let cu = if u == 0 { 1.0 / 2.0f32.sqrt() } else { 1.0 };
            let cv = if v == 0 { 1.0 / 2.0f32.sqrt() } else { 1.0 };

            let mut sum = 0.0f32;
            for x in 0..MCU_SIZE {
                for y in 0..MCU_SIZE {
                    sum += block[x][y] as f32 * cos_table[u][x] * cos_table[v][y];
                }
            }
            result[u][v] = 0.25 * cu * cv * sum;
        }
    }

    result
}

/// Quantize DCT coefficients and apply zigzag scan
fn quantize_zigzag(dct: &[[f32; MCU_SIZE]; MCU_SIZE], qt: &[u16; 64]) -> [i16; 64] {
    // `flat` is in entropy-coded coefficient order expected by the LRPT decoder.
    let mut flat = [0i16; 64];

    // Flatten 2D to 1D in row-major order
    let mut linear = [0.0f32; 64];
    for i in 0..MCU_SIZE {
        for j in 0..MCU_SIZE {
            linear[i * MCU_SIZE + j] = dct[i][j];
        }
    }

    // Quantize in row-major coefficient space, then place each coefficient in
    // its LRPT entropy index slot.
    for x in 0..64 {
        let entropy_idx = ZIGZAG[x];
        flat[entropy_idx] = (linear[x] / qt[x] as f32).round() as i16;
    }

    flat
}

/// Determine JPEG category (number of bits) for a value
fn jpeg_category(value: i16) -> (u8, u16) {
    if value == 0 {
        return (0, 0);
    }
    let abs_val = value.unsigned_abs();
    let category = 16 - abs_val.leading_zeros() as u8;
    let bits = if value > 0 {
        value as u16
    } else {
        // One's complement for negative values
        (value + (1 << category) - 1) as u16
    };
    (category, bits)
}

/// DC luminance Huffman table (standard JPEG)
/// Returns (code, code_length) for a given category
fn dc_huffman_encode(category: u8) -> (u16, u8) {
    match category {
        0 => (0b00, 2),
        1 => (0b010, 3),
        2 => (0b011, 3),
        3 => (0b100, 3),
        4 => (0b101, 3),
        5 => (0b110, 3),
        6 => (0b1110, 4),
        7 => (0b11110, 5),
        8 => (0b111110, 6),
        9 => (0b1111110, 7),
        10 => (0b11111110, 8),
        11 => (0b111111110, 9),
        _ => (0b111111110, 9), // Clamp to max
    }
}

/// AC luminance Huffman table (standard JPEG)
/// Input: (run_length, category) where run_length = 0-15 zeros before this coeff
/// Returns (code, code_length)
fn ac_huffman_encode(run: u8, category: u8) -> (u32, u8) {
    // LRPT AC Huffman mapping used by SatDump's MSU-MR decoder.
    // Encoded as (run << 4 | size) -> (code, length).
    let symbol = (run << 4) | category;
    match symbol {
        0x00 => (0b1010, 4),
        0x01 => (0b00, 2),
        0x02 => (0b01, 2),
        0x03 => (0b100, 3),
        0x04 => (0b1011, 4),
        0x05 => (0b11010, 5),
        0x06 => (0b1111000, 7),
        0x07 => (0b11111000, 8),
        0x08 => (0b1111110110, 10),
        0x09 => (0b1111111110000010, 16),
        0x0A => (0b1111111110000011, 16),
        0x11 => (0b1100, 4),
        0x12 => (0b11011, 5),
        0x13 => (0b1111001, 7),
        0x14 => (0b111110110, 9),
        0x15 => (0b11111110110, 11),
        0x16 => (0b1111111110000100, 16),
        0x17 => (0b1111111110000101, 16),
        0x18 => (0b1111111110000110, 16),
        0x19 => (0b1111111110000111, 16),
        0x1A => (0b1111111110001000, 16),
        0x21 => (0b11100, 5),
        0x22 => (0b11111001, 8),
        0x23 => (0b1111110111, 10),
        0x24 => (0b111111110100, 12),
        0x25 => (0b1111111110001001, 16),
        0x26 => (0b1111111110001010, 16),
        0x27 => (0b1111111110001011, 16),
        0x28 => (0b1111111110001100, 16),
        0x29 => (0b1111111110001101, 16),
        0x2A => (0b1111111110001110, 16),
        0x31 => (0b111010, 6),
        0x32 => (0b111110111, 9),
        0x33 => (0b111111110101, 12),
        0x34 => (0b1111111110001111, 16),
        0x35 => (0b1111111110010000, 16),
        0x36 => (0b1111111110010001, 16),
        0x37 => (0b1111111110010010, 16),
        0x38 => (0b1111111110010011, 16),
        0x39 => (0b1111111110010100, 16),
        0x3A => (0b1111111110010101, 16),
        0x41 => (0b111011, 6),
        0x42 => (0b1111111000, 10),
        0x43 => (0b1111111110010110, 16),
        0x44 => (0b1111111110010111, 16),
        0x45 => (0b1111111110011000, 16),
        0x46 => (0b1111111110011001, 16),
        0x47 => (0b1111111110011010, 16),
        0x48 => (0b1111111110011011, 16),
        0x49 => (0b1111111110011100, 16),
        0x4A => (0b1111111110011101, 16),
        0x51 => (0b1111010, 7),
        0x52 => (0b11111110111, 11),
        0x53 => (0b1111111110011110, 16),
        0x54 => (0b1111111110011111, 16),
        0x55 => (0b1111111110100000, 16),
        0x56 => (0b1111111110100001, 16),
        0x57 => (0b1111111110100010, 16),
        0x58 => (0b1111111110100011, 16),
        0x59 => (0b1111111110100100, 16),
        0x5A => (0b1111111110100101, 16),
        0x61 => (0b1111011, 7),
        0x62 => (0b111111110110, 12),
        0x63 => (0b1111111110100110, 16),
        0x64 => (0b1111111110100111, 16),
        0x65 => (0b1111111110101000, 16),
        0x66 => (0b1111111110101001, 16),
        0x67 => (0b1111111110101010, 16),
        0x68 => (0b1111111110101011, 16),
        0x69 => (0b1111111110101100, 16),
        0x6A => (0b1111111110101101, 16),
        0x71 => (0b11111010, 8),
        0x72 => (0b111111110111, 12),
        0x73 => (0b1111111110101110, 16),
        0x74 => (0b1111111110101111, 16),
        0x75 => (0b1111111110110000, 16),
        0x76 => (0b1111111110110001, 16),
        0x77 => (0b1111111110110010, 16),
        0x78 => (0b1111111110110011, 16),
        0x79 => (0b1111111110110100, 16),
        0x7A => (0b1111111110110101, 16),
        0x81 => (0b111111000, 9),
        0x82 => (0b111111111000000, 15),
        0x83 => (0b1111111110110110, 16),
        0x84 => (0b1111111110110111, 16),
        0x85 => (0b1111111110111000, 16),
        0x86 => (0b1111111110111001, 16),
        0x87 => (0b1111111110111010, 16),
        0x88 => (0b1111111110111011, 16),
        0x89 => (0b1111111110111100, 16),
        0x8A => (0b1111111110111101, 16),
        0x91 => (0b111111001, 9),
        0x92 => (0b1111111110111110, 16),
        0x93 => (0b1111111110111111, 16),
        0x94 => (0b1111111111000000, 16),
        0x95 => (0b1111111111000001, 16),
        0x96 => (0b1111111111000010, 16),
        0x97 => (0b1111111111000011, 16),
        0x98 => (0b1111111111000100, 16),
        0x99 => (0b1111111111000101, 16),
        0x9A => (0b1111111111000110, 16),
        0xA1 => (0b111111010, 9),
        0xA2 => (0b1111111111000111, 16),
        0xA3 => (0b1111111111001000, 16),
        0xA4 => (0b1111111111001001, 16),
        0xA5 => (0b1111111111001010, 16),
        0xA6 => (0b1111111111001011, 16),
        0xA7 => (0b1111111111001100, 16),
        0xA8 => (0b1111111111001101, 16),
        0xA9 => (0b1111111111001110, 16),
        0xAA => (0b1111111111001111, 16),
        0xB1 => (0b1111111001, 10),
        0xB2 => (0b1111111111010000, 16),
        0xB3 => (0b1111111111010001, 16),
        0xB4 => (0b1111111111010010, 16),
        0xB5 => (0b1111111111010011, 16),
        0xB6 => (0b1111111111010100, 16),
        0xB7 => (0b1111111111010101, 16),
        0xB8 => (0b1111111111010110, 16),
        0xB9 => (0b1111111111010111, 16),
        0xBA => (0b1111111111011000, 16),
        0xC1 => (0b1111111010, 10),
        0xC2 => (0b1111111111011001, 16),
        0xC3 => (0b1111111111011010, 16),
        0xC4 => (0b1111111111011011, 16),
        0xC5 => (0b1111111111011100, 16),
        0xC6 => (0b1111111111011101, 16),
        0xC7 => (0b1111111111011110, 16),
        0xC8 => (0b1111111111011111, 16),
        0xC9 => (0b1111111111100000, 16),
        0xCA => (0b1111111111100001, 16),
        0xD1 => (0b11111111000, 11),
        0xD2 => (0b1111111111100010, 16),
        0xD3 => (0b1111111111100011, 16),
        0xD4 => (0b1111111111100100, 16),
        0xD5 => (0b1111111111100101, 16),
        0xD6 => (0b1111111111100110, 16),
        0xD7 => (0b1111111111100111, 16),
        0xD8 => (0b1111111111101000, 16),
        0xD9 => (0b1111111111101001, 16),
        0xDA => (0b1111111111101010, 16),
        0xE1 => (0b1111111111101011, 16),
        0xE2 => (0b1111111111101100, 16),
        0xE3 => (0b1111111111101101, 16),
        0xE4 => (0b1111111111101110, 16),
        0xE5 => (0b1111111111101111, 16),
        0xE6 => (0b1111111111110000, 16),
        0xE7 => (0b1111111111110001, 16),
        0xE8 => (0b1111111111110010, 16),
        0xE9 => (0b1111111111110011, 16),
        0xEA => (0b1111111111110100, 16),
        0xF0 => (0b11111111001, 11),
        0xF1 => (0b1111111111110101, 16),
        0xF2 => (0b1111111111110110, 16),
        0xF3 => (0b1111111111110111, 16),
        0xF4 => (0b1111111111111000, 16),
        0xF5 => (0b1111111111111001, 16),
        0xF6 => (0b1111111111111010, 16),
        0xF7 => (0b1111111111111011, 16),
        0xF8 => (0b1111111111111100, 16),
        0xF9 => (0b1111111111111101, 16),
        0xFA => (0b1111111111111110, 16),
        _ => (0b1010, 4), // Default to EOB for unknown symbols
    }
}

/// Huffman-encode one 8x8 MCU block
fn encode_mcu(
    block: &[[u8; MCU_SIZE]; MCU_SIZE],
    qt: &[u16; 64],
    prev_dc: &mut i16,
    writer: &mut BitWriter,
) {
    // Level shift: subtract 128
    let mut shifted = [[0i16; MCU_SIZE]; MCU_SIZE];
    for i in 0..MCU_SIZE {
        for j in 0..MCU_SIZE {
            shifted[i][j] = block[i][j] as i16 - 128;
        }
    }

    // Forward DCT
    let dct = forward_dct(&shifted);

    // Quantize and zigzag
    let coeffs = quantize_zigzag(&dct, qt);

    // DC coefficient (differential coding)
    let dc_diff = coeffs[0] - *prev_dc;
    *prev_dc = coeffs[0];

    let (dc_cat, dc_bits) = jpeg_category(dc_diff);
    let (dc_code, dc_code_len) = dc_huffman_encode(dc_cat);
    writer.write_bits(dc_code as u32, dc_code_len);
    if dc_cat > 0 {
        writer.write_bits(dc_bits as u32, dc_cat);
    }

    // AC coefficients
    let mut zero_run: u8 = 0;
    for i in 1..64 {
        if coeffs[i] == 0 {
            zero_run += 1;
        } else {
            // Emit ZRL for runs > 15
            while zero_run >= 16 {
                let (zrl_code, zrl_len) = ac_huffman_encode(15, 0);
                writer.write_bits(zrl_code, zrl_len);
                zero_run -= 16;
            }

            let (ac_cat, ac_bits) = jpeg_category(coeffs[i]);
            let (ac_code, ac_code_len) = ac_huffman_encode(zero_run, ac_cat);
            writer.write_bits(ac_code, ac_code_len);
            writer.write_bits(ac_bits as u32, ac_cat);
            zero_run = 0;
        }
    }

    // EOB if we didn't end exactly at coefficient 63
    if zero_run > 0 {
        let (eob_code, eob_len) = ac_huffman_encode(0, 0);
        writer.write_bits(eob_code, eob_len);
    }
}

/// Compress a row of MCUs (196 blocks of 8x8 pixels)
/// Returns Huffman-coded compressed data bytes
/// DC predictor is reset at the start of each row (segment)
pub fn compress_mcu_row(mcus: &[[[u8; MCU_SIZE]; MCU_SIZE]], quality: u8) -> Vec<u8> {
    let qt = build_quant_table(quality);
    let mut writer = BitWriter::new();
    let mut prev_dc: i16 = 0;

    for mcu in mcus {
        encode_mcu(mcu, &qt, &mut prev_dc, &mut writer);
    }

    writer.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quant_table_quality_50() {
        let qt = build_quant_table(50);
        // At quality 50, table should be unchanged
        assert_eq!(qt[0], 16);
        assert_eq!(qt[1], 11);
    }

    #[test]
    fn test_quant_table_quality_100() {
        let qt = build_quant_table(100);
        // At quality 100, all values should be 1
        assert!(qt.iter().all(|&v| v == 1));
    }

    #[test]
    fn test_jpeg_category() {
        assert_eq!(jpeg_category(0), (0, 0));
        assert_eq!(jpeg_category(1), (1, 1));
        assert_eq!(jpeg_category(-1), (1, 0));
        assert_eq!(jpeg_category(5), (3, 5));
        assert_eq!(jpeg_category(-5), (3, 2));
    }

    #[test]
    fn test_compress_uniform_block() {
        // Uniform blocks should compress very small (only DC)
        let block = [[128u8; MCU_SIZE]; MCU_SIZE];
        let mcus = vec![block; 4];
        let compressed = compress_mcu_row(&mcus, 80);
        assert!(!compressed.is_empty());
        // Should be small since all blocks are identical gray
        assert!(compressed.len() < 20);
    }

    #[test]
    fn test_bitwriter() {
        let mut w = BitWriter::new();
        w.write_bits(0b11, 2);
        w.write_bits(0b000000, 6);
        let result = w.flush();
        assert_eq!(result, vec![0xC0]);
    }
}
