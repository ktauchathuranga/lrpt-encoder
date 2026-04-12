/// OQPSK Modulation with Root Raised Cosine pulse shaping
///
/// Meteor-M N2-4 LRPT parameters:
/// - Symbol rate: 72 ksps
/// - Modulation: OQPSK (Offset QPSK) — Q delayed by T_symbol/2 vs. I
/// - 4 samples/symbol → Q offset = 2 samples at 288 kHz
/// - Pulse shaping: Root Raised Cosine (roll-off 0.5)
///
/// Bit ordering:
/// - Convolutional encoder outputs pairs (G1, G2) for each info bit.
/// - Bytes are packed MSB-first with alternating G1, G2, G1, G2, ...
/// - Even-indexed output bits (G1) → I channel (BPSK: 0→+1, 1→-1)
/// - Odd-indexed output bits  (G2) → Q channel, delayed by T/2
///
/// Reference: J.M. Friedt, "Decoding LRPT from Meteor-M2" (glmf_meteor_eng, 2019);
/// SatDump Meteor-M2-4 pipeline uses OQPSK demodulator at 72 ksym/s.

use crate::constants::*;
use std::f32::consts::PI;

/// BPSK map a single bit: 0 → +1.0, 1 → -1.0
#[inline]
fn bpsk(bit: u8) -> f32 {
    if bit & 1 == 0 { 1.0 } else { -1.0 }
}

/// Generate Root Raised Cosine filter coefficients
fn rrc_filter(span_symbols: usize, samples_per_symbol: usize, rolloff: f32) -> Vec<f32> {
    let num_taps = span_symbols * samples_per_symbol + 1;
    let half = (num_taps / 2) as f32;
    let t_s = 1.0;
    let alpha = rolloff;

    let mut taps = Vec::with_capacity(num_taps);

    for i in 0..num_taps {
        let t = (i as f32 - half) / samples_per_symbol as f32;

        let val = if t.abs() < 1e-10 {
            (1.0 - alpha + 4.0 * alpha / PI) / t_s
        } else if (t.abs() - t_s / (4.0 * alpha)).abs() < 1e-10 {
            alpha / (t_s * 2.0f32.sqrt())
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
        } else {
            let pi_t = PI * t / t_s;
            let four_alpha_t = 4.0 * alpha * t / t_s;
            (((1.0 - alpha) * pi_t).sin()
                + four_alpha_t * ((1.0 + alpha) * pi_t).cos())
                / (pi_t * (1.0 - four_alpha_t * four_alpha_t))
                / t_s
        };

        taps.push(val);
    }

    let energy: f32 = taps.iter().map(|x| x * x).sum();
    let norm = 1.0 / energy.sqrt();
    for tap in &mut taps {
        *tap *= norm;
    }

    taps
}

/// Convert convolutionally-encoded bytes to OQPSK I/Q samples.
///
/// Input: encoded bytes (bit-packed, MSB-first). The convolutional encoder
/// emits pairs (G1, G2) per info bit, packed sequentially, so even bit index
/// = I stream, odd bit index = Q stream.
///
/// Output: Vec of (I, Q) sample pairs at SAMPLE_RATE (288 kHz), with Q
/// delayed by SAMPLES_PER_SYMBOL/2 samples relative to I.
pub fn oqpsk_modulate(encoded_bytes: &[u8]) -> Vec<(f32, f32)> {
    // Split the encoded bit stream into I (even) and Q (odd) BPSK symbols.
    let total_bits = encoded_bytes.len() * 8;
    debug_assert!(total_bits % 2 == 0, "encoded bit stream must have even length for QPSK");

    let num_symbols = total_bits / 2;
    let mut i_syms: Vec<f32> = Vec::with_capacity(num_symbols);
    let mut q_syms: Vec<f32> = Vec::with_capacity(num_symbols);

    for &byte in encoded_bytes {
        // Bits MSB-first: b7 b6 b5 b4 b3 b2 b1 b0
        // Pair them: (b7,b6) (b5,b4) (b3,b2) (b1,b0)
        for shift in (0..8).step_by(2).rev() {
            let b_i = (byte >> (shift + 1)) & 1;
            let b_q = (byte >> shift) & 1;
            i_syms.push(bpsk(b_i));
            q_syms.push(bpsk(b_q));
        }
    }

    let sps = SAMPLES_PER_SYMBOL as usize;
    let q_offset = sps / 2; // T/2 delay for OQPSK

    // Normalize symbol amplitude so average constellation magnitude is ~1/√2 per arm
    let scale = 1.0 / 2.0f32.sqrt();

    // RRC pulse shaping
    let filter = rrc_filter(RRC_SPAN, sps, RRC_ROLLOFF);
    let filter_len = filter.len();
    let half_filter = filter_len / 2;

    // Allocate upsampled impulse trains. Q is offset forward by q_offset samples.
    let total_samples = num_symbols * sps + filter_len + q_offset;
    let mut i_up = vec![0.0f32; total_samples];
    let mut q_up = vec![0.0f32; total_samples];

    for k in 0..num_symbols {
        let base = k * sps + half_filter;
        if base < total_samples {
            i_up[base] = i_syms[k] * scale;
        }
        let qpos = base + q_offset;
        if qpos < total_samples {
            q_up[qpos] = q_syms[k] * scale;
        }
    }

    // Convolve each arm with the RRC filter.
    let output_len = num_symbols * sps;
    let mut output = Vec::with_capacity(output_len);

    for n in 0..output_len {
        let mut iv = 0.0f32;
        let mut qv = 0.0f32;
        for (k, &tap) in filter.iter().enumerate() {
            let idx = n + k;
            if idx < total_samples {
                iv += i_up[idx] * tap;
                qv += q_up[idx] * tap;
            }
        }
        output.push((iv, qv));
    }

    // Normalize to [-1, 1] with small headroom.
    let max_val = output
        .iter()
        .map(|(i, q)| i.abs().max(q.abs()))
        .fold(0.0f32, f32::max);

    if max_val > 0.0 {
        let scale = 0.95 / max_val;
        for sample in &mut output {
            sample.0 *= scale;
            sample.1 *= scale;
        }
    }

    output
}

/// Modulate a stream of encoded CADUs into continuous I/Q samples.
pub fn modulate_cadu_stream(encoded_cadus: &[Vec<u8>]) -> Vec<(f32, f32)> {
    let mut all_bytes = Vec::new();
    for cadu in encoded_cadus {
        all_bytes.extend_from_slice(cadu);
    }
    oqpsk_modulate(&all_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_map() {
        assert_eq!(bpsk(0), 1.0);
        assert_eq!(bpsk(1), -1.0);
    }

    #[test]
    fn test_rrc_filter_symmetric() {
        let filter = rrc_filter(RRC_SPAN, SAMPLES_PER_SYMBOL as usize, RRC_ROLLOFF);
        let len = filter.len();
        for i in 0..len / 2 {
            assert!(
                (filter[i] - filter[len - 1 - i]).abs() < 1e-6,
                "RRC filter should be symmetric"
            );
        }
    }

    #[test]
    fn test_oqpsk_output_length() {
        // 4 bytes = 32 bits → 16 QPSK symbols
        let data = vec![0xFF; 4];
        let samples = oqpsk_modulate(&data);
        assert_eq!(samples.len(), 16 * SAMPLES_PER_SYMBOL as usize);
    }

    #[test]
    fn test_oqpsk_output_bounded() {
        let data = vec![0xAA; 100];
        let samples = oqpsk_modulate(&data);
        for (i, q) in &samples {
            assert!(i.abs() <= 1.0, "I sample out of range: {}", i);
            assert!(q.abs() <= 1.0, "Q sample out of range: {}", q);
        }
    }
}
