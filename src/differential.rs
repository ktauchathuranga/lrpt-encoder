/// NRZ-M differential encoding helpers for LRPT.
///
/// SatDump's Meteor M2-x 72k chain applies NRZ-M decoding on Viterbi output
/// bits before deframing. To interoperate, we encode CADU bits with NRZ-M
/// before convolutional coding.

/// NRZ-M encode a packed bitstream (MSB-first) while preserving continuity.
///
/// `last_bit` is the previous encoded bit and must be kept across consecutive
/// CADUs in a continuous transmission.
pub fn nrzm_encode_bits(input: &[u8], last_bit: &mut u8) -> Vec<u8> {
    let mut output = vec![0u8; input.len()];
    let total_bits = input.len() * 8;

    for bit_idx in 0..total_bits {
        let bit = get_bit(input, bit_idx);
        let encoded = bit ^ (*last_bit & 1);
        *last_bit = encoded;
        set_bit(&mut output, bit_idx, encoded);
    }

    output
}

#[inline]
fn get_bit(data: &[u8], bit_idx: usize) -> u8 {
    (data[bit_idx / 8] >> (7 - (bit_idx % 8))) & 1
}

#[inline]
fn set_bit(data: &mut [u8], bit_idx: usize, bit: u8) {
    if bit != 0 {
        data[bit_idx / 8] |= 1 << (7 - (bit_idx % 8));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_bits_like_satdump(bits: &mut [u8], last_bit: &mut u8) {
        for b in bits {
            let current = *b;
            *b = current ^ *last_bit;
            *last_bit = current;
        }
    }

    fn unpack_bits_msb(data: &[u8]) -> Vec<u8> {
        let mut bits = Vec::with_capacity(data.len() * 8);
        for &byte in data {
            for shift in (0..8).rev() {
                bits.push((byte >> shift) & 1);
            }
        }
        bits
    }

    fn pack_bits_msb(bits: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; (bits.len() + 7) / 8];
        for (idx, &bit) in bits.iter().enumerate() {
            if bit != 0 {
                out[idx / 8] |= 1 << (7 - (idx % 8));
            }
        }
        out
    }

    #[test]
    fn test_nrzm_roundtrip_against_satdump_decode_bits() {
        let input = vec![0x1A, 0xCF, 0xFC, 0x1D, 0xAA, 0x55, 0x00, 0xFF];

        let mut tx_state = 0u8;
        let encoded = nrzm_encode_bits(&input, &mut tx_state);

        let mut bits = unpack_bits_msb(&encoded);
        let mut rx_state = 0u8;
        decode_bits_like_satdump(&mut bits, &mut rx_state);
        let decoded = pack_bits_msb(&bits);

        assert_eq!(decoded, input);
    }

    #[test]
    fn test_nrzm_continuity_matches_single_pass() {
        let part1 = vec![0x12, 0x34, 0x56];
        let part2 = vec![0xAB, 0xCD, 0xEF];

        let mut full = part1.clone();
        full.extend_from_slice(&part2);

        let mut s1 = 0u8;
        let one_shot = nrzm_encode_bits(&full, &mut s1);

        let mut s2 = 0u8;
        let mut split = nrzm_encode_bits(&part1, &mut s2);
        split.extend_from_slice(&nrzm_encode_bits(&part2, &mut s2));

        assert_eq!(split, one_shot);
    }
}
