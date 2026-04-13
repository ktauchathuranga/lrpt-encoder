/// OQPSK Modulation with Root Raised Cosine pulse shaping.
///
/// Meteor-M N2-4 LRPT parameters:
/// - Symbol rate: 72 ksps
/// - Modulation: OQPSK (Q arm delayed by T/2)
/// - 4 samples/symbol → Q offset = 2 samples at 288 kHz
/// - Pulse shaping: Root Raised Cosine (α = 0.5)
///
/// Bit ordering: convolutional encoder emits (G1, G2) pairs packed MSB-first.
/// Even-indexed bits (G1) → I arm; odd-indexed bits (G2) → Q arm, delayed T/2.

use crate::constants::*;
use std::f32::consts::PI;

#[inline]
fn bpsk(bit: u8) -> f32 {
    if bit & 1 == 0 { 1.0 } else { -1.0 }
}

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

/// Compute a deterministic amplitude scale so OQPSK output stays in [-1,1].
/// The worst-case amplitude is sum(|filter taps|) * symbol amplitude (1/√2).
fn compute_scale(filter: &[f32]) -> f32 {
    let sum_abs: f32 = filter.iter().map(|t| t.abs()).sum();
    let peak = sum_abs * (1.0 / 2.0f32.sqrt());
    0.85 / peak.max(1e-6)
}

/// One-shot OQPSK modulator (used for file/WAV output).
#[allow(dead_code)]
pub fn oqpsk_modulate(encoded_bytes: &[u8]) -> Vec<(f32, f32)> {
    let mut m = OqpskModulator::new();
    let mut out = m.feed(encoded_bytes);
    out.extend(m.flush());
    out
}

/// Convenience: modulate a stream of CADUs as one continuous signal.
pub fn modulate_cadu_stream(encoded_cadus: &[Vec<u8>]) -> Vec<(f32, f32)> {
    let mut m = OqpskModulator::new();
    let mut out = Vec::new();
    for cadu in encoded_cadus {
        out.extend(m.feed(cadu));
    }
    out.extend(m.flush());
    out
}

/// Stateful OQPSK modulator with continuous RRC pulse shaping across chunks.
///
/// Use [`OqpskModulator::feed`] for each chunk of encoded bytes; the filter
/// history is preserved so chunk boundaries have no glitches. Call
/// [`OqpskModulator::flush`] at end-of-stream to drain the final filter tail.
pub struct OqpskModulator {
    filter: Vec<f32>,
    sps: usize,
    q_offset: usize,
    scale: f32,
    /// Impulse-train history; length = filter.len() - 1. Newest sample last.
    i_hist: Vec<f32>,
    q_hist: Vec<f32>,
    /// Leftover single Q symbol when an odd number of QPSK symbols was fed
    /// (Q arm is offset, so we may need to defer one symbol worth to the next
    /// call). Always `None` in practice because encoded bytes come in whole
    /// bytes → even number of bits → integer QPSK symbols.
    pending_q_delay: f32,
    has_pending_q: bool,
}

impl OqpskModulator {
    pub fn new() -> Self {
        let filter = rrc_filter(RRC_SPAN, SAMPLES_PER_SYMBOL as usize, RRC_ROLLOFF);
        let sps = SAMPLES_PER_SYMBOL as usize;
        let scale = compute_scale(&filter);
        let hist_len = filter.len().saturating_sub(1);
        Self {
            filter,
            sps,
            q_offset: sps / 2,
            scale,
            i_hist: vec![0.0; hist_len],
            q_hist: vec![0.0; hist_len],
            pending_q_delay: 0.0,
            has_pending_q: false,
        }
    }

    /// Modulate one chunk of encoded bytes. Returns I/Q samples ready to
    /// write. Number of output samples = (bytes*8/2) * sps per call.
    pub fn feed(&mut self, encoded_bytes: &[u8]) -> Vec<(f32, f32)> {
        let total_bits = encoded_bytes.len() * 8;
        if total_bits == 0 {
            return Vec::new();
        }
        debug_assert!(total_bits % 2 == 0);
        let num_symbols = total_bits / 2;
        let sym_amp = 1.0 / 2.0f32.sqrt();

        // Upsampled impulse trains for this chunk. Length = num_symbols * sps.
        let chunk_len = num_symbols * self.sps;
        let mut i_up = vec![0.0f32; chunk_len];
        let mut q_up = vec![0.0f32; chunk_len];

        let mut sym_idx = 0usize;
        for &byte in encoded_bytes {
            for shift in (0..8).step_by(2).rev() {
                let b_i = (byte >> (shift + 1)) & 1;
                let b_q = (byte >> shift) & 1;
                let pos = sym_idx * self.sps;
                i_up[pos] = bpsk(b_i) * sym_amp;
                // Q is offset by +q_offset samples within this chunk
                let qpos = pos + self.q_offset;
                if qpos < chunk_len {
                    q_up[qpos] = bpsk(b_q) * sym_amp;
                } else {
                    // Shouldn't happen when q_offset < sps (it's sps/2 = 2 < 4)
                    unreachable!()
                }
                sym_idx += 1;
            }
        }

        // Convolve with RRC filter using continuous history.
        let filter_len = self.filter.len();
        let hist_len = self.i_hist.len(); // = filter_len - 1
        let total_len = hist_len + chunk_len;

        // Build combined streams: [history | current chunk]
        // We only need to index into them; use function instead of allocation.
        let i_at = |n: usize, i_hist: &[f32], i_up: &[f32]| -> f32 {
            if n < hist_len { i_hist[n] } else { i_up[n - hist_len] }
        };
        let q_at = |n: usize, q_hist: &[f32], q_up: &[f32]| -> f32 {
            if n < hist_len { q_hist[n] } else { q_up[n - hist_len] }
        };

        let mut output = Vec::with_capacity(chunk_len);
        // Output sample n corresponds to filter applied ending at combined[hist_len + n]
        for n in 0..chunk_len {
            let mut iv = 0.0f32;
            let mut qv = 0.0f32;
            // filter[k] applied to combined[hist_len + n - (filter_len - 1 - k)]
            //                         = combined[n + k - (filter_len - 1 - hist_len)]
            // with hist_len = filter_len - 1 → combined[n + k]
            // But we want: y[n] = sum_k filter[k] * x[n - k + something]; use
            // standard FIR: y[n] = sum_{k=0}^{L-1} filter[k] * x[n + L-1 - k]
            // where x index 0 is oldest history sample.
            for k in 0..filter_len {
                let idx = n + (filter_len - 1 - k);
                if idx < total_len {
                    iv += self.filter[k] * i_at(idx, &self.i_hist, &i_up);
                    qv += self.filter[k] * q_at(idx, &self.q_hist, &q_up);
                }
            }
            output.push((iv * self.scale, qv * self.scale));
        }

        // Update history with the tail of the current chunk's impulse train.
        // New history = last hist_len samples of [history | chunk].
        if hist_len > 0 {
            let mut new_i = vec![0.0f32; hist_len];
            let mut new_q = vec![0.0f32; hist_len];
            for j in 0..hist_len {
                let src = total_len - hist_len + j; // indices in combined stream
                new_i[j] = i_at(src, &self.i_hist, &i_up);
                new_q[j] = q_at(src, &self.q_hist, &q_up);
            }
            self.i_hist = new_i;
            self.q_hist = new_q;
        }

        // Silence the pending-Q handling noise
        let _ = &mut self.pending_q_delay;
        let _ = &mut self.has_pending_q;

        output
    }

    /// Drain the filter tail by feeding zeros. Call once at end-of-stream.
    pub fn flush(&mut self) -> Vec<(f32, f32)> {
        // Feed half-filter of zero bytes so the last real symbols get fully
        // convolved. filter_len = RRC_SPAN*sps + 1 ≈ 133 samples.
        let symbols_needed = (self.filter.len() + self.sps - 1) / self.sps;
        let bits_needed = symbols_needed * 2;
        let bytes_needed = (bits_needed + 7) / 8;
        let zeros = vec![0u8; bytes_needed];
        // Feed zeros but we only need the first `filter_len` samples out.
        let out = self.feed(&zeros);
        out
    }
}

impl Default for OqpskModulator {
    fn default() -> Self {
        Self::new()
    }
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
        let data = vec![0xFF; 4];
        let samples = oqpsk_modulate(&data);
        // 16 symbols * 4 sps = 64 from feed + tail from flush
        let expected_min = 16 * SAMPLES_PER_SYMBOL as usize;
        assert!(samples.len() >= expected_min);
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

    #[test]
    fn test_streaming_matches_oneshot_prefix() {
        // Feeding the same bytes in two chunks should yield identical samples
        // for the prefix that's fully covered by both approaches.
        let data: Vec<u8> = (0..1024u32).map(|i| (i * 31) as u8).collect();

        let one_shot = {
            let mut m = OqpskModulator::new();
            m.feed(&data)
        };

        let streamed = {
            let mut m = OqpskModulator::new();
            let mut out = m.feed(&data[..512]);
            out.extend(m.feed(&data[512..]));
            out
        };

        assert_eq!(one_shot.len(), streamed.len());
        for (idx, (a, b)) in one_shot.iter().zip(streamed.iter()).enumerate() {
            assert!(
                (a.0 - b.0).abs() < 1e-5 && (a.1 - b.1).abs() < 1e-5,
                "sample {} differs: one_shot={:?} streamed={:?}",
                idx, a, b
            );
        }
    }
}
