/// CCSDS Convolutional Encoder
///
/// Rate 1/2, Constraint length K=7
/// G1 = 171 octal (0b1111001) = 0x79
/// G2 = 133 octal (0b1011011) = 0x5B
///
/// For each input bit, produces 2 output bits (G1 first, then G2).
/// Output is packed MSB-first into bytes.
use crate::constants::{CONV_G1, CONV_G2};

/// Encode data while preserving the 6-bit shift-register state between calls.
///
/// This is required for streaming CADUs continuously: restarting the encoder
/// state on each frame can corrupt boundary bits and break ASM lock downstream.
pub fn encode_with_state(data: &[u8], state: &mut u8) -> Vec<u8> {
    let input_bits = data.len() * 8;
    let output_bits = input_bits * 2;
    let output_bytes = (output_bits + 7) / 8;

    let mut output = vec![0u8; output_bytes];
    let mut conv_state = *state & 0x3F; // 6-bit shift register
    let mut out_bit_idx: usize = 0;

    for &byte in data {
        for bit_pos in (0..8).rev() {
            let input_bit = (byte >> bit_pos) & 1;

            // Combine input with state (7 bits total)
            let combined = (input_bit << 6) | conv_state;

            // G1 output
            let g1 = parity(combined & CONV_G1);
            set_bit(&mut output, out_bit_idx, g1);
            out_bit_idx += 1;

            // G2 output
            let g2 = parity(combined & CONV_G2);
            set_bit(&mut output, out_bit_idx, g2);
            out_bit_idx += 1;

            // Shift state: oldest bit drops, input becomes newest
            conv_state = ((conv_state >> 1) | (input_bit << 5)) & 0x3F;
        }
    }

    *state = conv_state;
    output
}

/// Compute parity (number of set bits mod 2)
#[inline]
fn parity(x: u8) -> u8 {
    (x.count_ones() & 1) as u8
}

/// Set a single bit in a byte array (MSB-first ordering)
#[inline]
fn set_bit(data: &mut [u8], bit_index: usize, value: u8) {
    if value != 0 {
        data[bit_index / 8] |= 1 << (7 - (bit_index % 8));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_zeros() {
        let input = vec![0x00];
        let mut state: u8 = 0;
        let output = encode_with_state(&input, &mut state);
        // All-zero input with zero state should produce all-zero output
        assert_eq!(output.len(), 2);
        assert_eq!(output, vec![0x00, 0x00]);
    }

    #[test]
    fn test_encode_output_length() {
        let input = vec![0xFF; 10];
        let mut state: u8 = 0;
        let output = encode_with_state(&input, &mut state);
        // 10 bytes * 8 bits * 2 = 160 bits = 20 bytes
        assert_eq!(output.len(), 20);
    }

    #[test]
    fn test_encode_single_one() {
        // Input: 0x80 = 10000000
        // First bit is 1, rest are 0
        let input = vec![0x80];
        let mut state: u8 = 0;
        let output = encode_with_state(&input, &mut state);
        // First input bit=1, state=000000
        // combined = 1000000 = 0x40
        // G1: 0x40 & 0x79 = 0x40, parity(0x40) = 1
        // G2: 0x40 & 0x5B = 0x40, parity(0x40) = 1
        // Output starts with 11...
        assert_eq!(output[0] & 0xC0, 0xC0);
    }

    #[test]
    fn test_encode_with_state_matches_single_pass() {
        let part1 = vec![0x12, 0x34, 0x56];
        let part2 = vec![0xAB, 0xCD];

        let mut full = part1.clone();
        full.extend_from_slice(&part2);
        let mut single_state: u8 = 0;
        let one_shot = encode_with_state(&full, &mut single_state);

        let mut state: u8 = 0;
        let mut split = encode_with_state(&part1, &mut state);
        split.extend_from_slice(&encode_with_state(&part2, &mut state));

        assert_eq!(split, one_shot);
    }
}
