/// CCSDS pseudo-random scrambler
///
/// Polynomial: x^8 + x^7 + x^5 + x^3 + 1
/// Initial state: 0xFF (all ones)
/// Applied to coded frame data (after RS, excluding ASM)
/// PN sequence period: 255 bytes
///
/// Register uses left-shift with output = register state (byte-at-a-time).
/// Feedback taps at bits 0, 2, 4, 7 corresponding to the recurrence:
///   s_n = s_{n-1} + s_{n-3} + s_{n-5} + s_{n-8}

/// Apply CCSDS scrambling to data in-place
pub fn ccsds_scramble(data: &mut [u8]) {
    let mut sr: u8 = 0xFF;

    for byte in data.iter_mut() {
        // Output current register state as PN byte
        *byte ^= sr;

        // Clock the LFSR 8 times
        for _ in 0..8 {
            let feedback = (sr ^ (sr >> 2) ^ (sr >> 4) ^ (sr >> 7)) & 1;
            sr = (sr << 1) | feedback;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pn_sequence_first_bytes() {
        // Verify the CCSDS PN sequence starts with known values
        let mut data = vec![0u8; 4];
        ccsds_scramble(&mut data);
        // XOR with 0x00 gives the raw PN sequence
        assert_eq!(data[0], 0xFF, "First PN byte should be 0xFF");
        assert_eq!(data[1], 0x48, "Second PN byte should be 0x48");
        assert_eq!(data[2], 0x0E, "Third PN byte should be 0x0E");
        assert_eq!(data[3], 0xC0, "Fourth PN byte should be 0xC0");
    }

    #[test]
    fn test_scramble_descramble() {
        let original = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x42];
        let mut data = original.clone();
        ccsds_scramble(&mut data);
        assert_ne!(data, original);
        ccsds_scramble(&mut data);
        assert_eq!(data, original);
    }
}
