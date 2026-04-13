/// Output module for LRPT encoder
///
/// Supports:
/// - 2-channel WAV file (I/Q interleaved, 16-bit, 288 kHz)
/// - Raw I/Q binary (interleaved float32 or int16)
/// - Stdout streaming (raw PCM for piping to SDR tools)
use crate::constants::SAMPLE_RATE;
use hound::{SampleFormat, WavSpec, WavWriter};
use std::io::{self, Write};
use std::path::Path;

/// Write I/Q samples to a 2-channel WAV file
pub fn write_iq_wav(path: &Path, samples: &[(f32, f32)]) -> Result<(), Box<dyn std::error::Error>> {
    let spec = WavSpec {
        channels: 2,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: 16,
        sample_format: SampleFormat::Int,
    };

    let mut writer = WavWriter::create(path, spec)?;

    for &(i_val, q_val) in samples {
        let i_sample = (i_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
        let q_sample = (q_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
        writer.write_sample(i_sample)?;
        writer.write_sample(q_sample)?;
    }

    writer.finalize()?;
    eprintln!(
        "Wrote {} I/Q samples to {:?} ({:.2}s at {} Hz)",
        samples.len(),
        path,
        samples.len() as f32 / SAMPLE_RATE as f32,
        SAMPLE_RATE
    );

    Ok(())
}

/// Write I/Q samples as raw interleaved int16 to stdout
/// Format: [I0_lo, I0_hi, Q0_lo, Q0_hi, I1_lo, I1_hi, Q1_lo, Q1_hi, ...]
#[allow(dead_code)]
pub fn write_iq_stdout(samples: &[(f32, f32)]) -> Result<(), Box<dyn std::error::Error>> {
    let stdout = io::stdout();
    let mut out = stdout.lock();

    for &(i_val, q_val) in samples {
        let i_sample = (i_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
        let q_sample = (q_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
        out.write_all(&i_sample.to_le_bytes())?;
        out.write_all(&q_sample.to_le_bytes())?;
    }

    out.flush()?;
    Ok(())
}

/// Write I/Q samples as raw interleaved float32 binary file
pub fn write_iq_raw(path: &Path, samples: &[(f32, f32)]) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;

    let mut file = File::create(path)?;

    for &(i_val, q_val) in samples {
        file.write_all(&i_val.to_le_bytes())?;
        file.write_all(&q_val.to_le_bytes())?;
    }

    eprintln!(
        "Wrote {} I/Q samples to {:?} (raw float32, {:.2}s at {} Hz)",
        samples.len(),
        path,
        samples.len() as f32 / SAMPLE_RATE as f32,
        SAMPLE_RATE
    );

    Ok(())
}

/// Incrementally write I/Q samples to a WAV file (for camera/continuous mode)
#[allow(dead_code)]
pub struct StreamingWavWriter {
    writer: WavWriter<io::BufWriter<std::fs::File>>,
    sample_count: usize,
}

#[allow(dead_code)]
impl StreamingWavWriter {
    pub fn new(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let spec = WavSpec {
            channels: 2,
            sample_rate: SAMPLE_RATE,
            bits_per_sample: 16,
            sample_format: SampleFormat::Int,
        };

        let writer = WavWriter::create(path, spec)?;
        Ok(StreamingWavWriter {
            writer,
            sample_count: 0,
        })
    }

    pub fn write_samples(
        &mut self,
        samples: &[(f32, f32)],
    ) -> Result<(), Box<dyn std::error::Error>> {
        for &(i_val, q_val) in samples {
            let i_sample = (i_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
            let q_sample = (q_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
            self.writer.write_sample(i_sample)?;
            self.writer.write_sample(q_sample)?;
            self.sample_count += 1;
        }
        Ok(())
    }

    pub fn finalize(self) -> Result<usize, Box<dyn std::error::Error>> {
        self.writer.finalize()?;
        Ok(self.sample_count)
    }
}
