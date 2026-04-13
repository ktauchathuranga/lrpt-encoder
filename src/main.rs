use clap::Parser;
#[cfg(feature = "input-v4l")]
use std::io::Write;
use std::path::PathBuf;
#[cfg(feature = "input-v4l")]
use std::sync::Arc;
#[cfg(feature = "input-v4l")]
use std::sync::atomic::{AtomicBool, Ordering};

#[cfg(feature = "input-v4l")]
use nokhwa::{
    Camera,
    pixel_format::RgbFormat,
    utils::{CameraIndex, RequestedFormat, RequestedFormatType},
};

mod ccsds;
mod compression;
mod constants;
mod convolutional;
mod differential;
mod imaging;
mod modulation;
mod output;
mod reed_solomon;
mod scrambler;

use constants::*;

fn parse_hex_bytes(input: &str) -> Result<Vec<u8>, String> {
    let trimmed = input.trim();
    if trimmed.is_empty() {
        return Err("telemetry payload cannot be empty".to_string());
    }

    let has_separators = trimmed
        .chars()
        .any(|c| c == ',' || c == ':' || c == '-' || c.is_whitespace());

    let mut bytes = Vec::new();

    if has_separators {
        for (idx, token) in trimmed
            .split(|c: char| c == ',' || c == ':' || c == '-' || c.is_whitespace())
            .filter(|t| !t.is_empty())
            .enumerate()
        {
            let token = token
                .strip_prefix("0x")
                .or_else(|| token.strip_prefix("0X"))
                .unwrap_or(token);

            if token.is_empty() || token.len() > 2 {
                return Err(format!(
                    "invalid byte '{}' at position {} (expected 1-2 hex digits)",
                    token,
                    idx + 1
                ));
            }

            let byte = u8::from_str_radix(token, 16)
                .map_err(|_| format!("invalid hex byte '{}' at position {}", token, idx + 1))?;
            bytes.push(byte);
        }
    } else {
        let raw = trimmed
            .strip_prefix("0x")
            .or_else(|| trimmed.strip_prefix("0X"))
            .unwrap_or(trimmed);

        if raw.len() % 2 != 0 {
            return Err(
                "hex payload without separators must have an even number of digits".to_string(),
            );
        }

        for i in (0..raw.len()).step_by(2) {
            let token = &raw[i..i + 2];
            let byte = u8::from_str_radix(token, 16)
                .map_err(|_| format!("invalid hex byte '{}'", token))?;
            bytes.push(byte);
        }
    }

    if bytes.is_empty() {
        return Err("telemetry payload cannot be empty".to_string());
    }

    if bytes.len() > (u16::MAX as usize + 1) {
        return Err("telemetry payload too large for CCSDS source packet".to_string());
    }

    Ok(bytes)
}

#[derive(Parser, Debug)]
#[command(name = "lrpt-encoder")]
#[command(author, version, about = "Encode images into Meteor-M N2-4 LRPT digital signals", long_about = None)]
struct Args {
    /// Input image file (JPEG/PNG) - used for all 3 channels if no separate inputs given
    #[arg(value_name = "INPUT_IMAGE")]
    input: Option<PathBuf>,

    /// Output file (WAV for I/Q, or .raw/.bin for raw float32 I/Q)
    #[arg(short = 'o', long = "output", value_name = "OUTPUT_FILE")]
    output: Option<PathBuf>,

    /// Input image for channel 2 (green/near-IR)
    #[arg(long, value_name = "IMAGE_CH2")]
    input_ch2: Option<PathBuf>,

    /// Input image for channel 3 (thermal IR)
    #[arg(long, value_name = "IMAGE_CH3")]
    input_ch3: Option<PathBuf>,

    /// Use RGB channels from a color image (R=ch1, G=ch2, B=ch3)
    #[arg(long)]
    rgb: bool,

    /// Camera capture mode
    #[arg(long)]
    capture: bool,

    /// Output raw I/Q to stdout (for piping to SDR tools)
    #[arg(long)]
    stdout: bool,

    /// Continuous capture mode (Ctrl+C to stop)
    #[arg(long)]
    continuous: bool,

    /// Camera index for all channels in capture mode
    #[arg(long, default_value = "0")]
    cam_idx: u32,

    /// Number of times to repeat the image
    #[arg(short, long, default_value = "1")]
    repeat: usize,

    /// JPEG compression quality (1-100)
    #[arg(long, default_value = "80")]
    quality: u8,

    /// Spacecraft ID (Meteor-M satellite identifier)
    #[arg(long, default_value = "57")]
    scid: u8,

    /// Virtual Channel ID
    #[arg(long, default_value = "5")]
    vcid: u8,

    /// APID for channel 1
    #[arg(long, default_value = "64")]
    apid_ch1: u16,

    /// APID for channel 2
    #[arg(long, default_value = "65")]
    apid_ch2: u16,

    /// APID for channel 3
    #[arg(long, default_value = "66")]
    apid_ch3: u16,

    /// MSU-MR instrument ID in telemetry (0-15)
    #[arg(long = "msumr-id", default_value = "4", value_parser = clap::value_parser!(u8).range(0..=15))]
    telemetry_msumr_id: u8,

    /// CCSDS day (days since 1958-01-01)
    #[arg(long = "ccsds-day", default_value = "24938")]
    ccsds_day: u16,

    /// CCSDS timestamp in milliseconds of day
    #[arg(long = "ccsds-ms", default_value = "43200000", value_parser = clap::value_parser!(u32).range(0..=86_399_999))]
    ccsds_ms: u32,

    /// APID 70 telemetry payload bytes in hex (AABBCC, AA BB CC, or 0xAA,0xBB)
    #[arg(long = "telemetry-payload", value_name = "HEX_BYTES", value_parser = parse_hex_bytes)]
    telemetry_payload: Option<Vec<u8>>,

    /// Optional debug output: write first raw CADU (1024 bytes) before NRZ-M/Conv
    #[arg(long, value_name = "CADU_FILE")]
    dump_first_cadu: Option<PathBuf>,

    /// Optional debug output: write full raw CADU stream before NRZ-M/Conv
    #[arg(long, value_name = "CADU_STREAM_FILE")]
    dump_cadu_stream: Option<PathBuf>,
}

/// Encode image channels into LRPT I/Q samples
fn encode_channels(channels: &[Vec<Vec<u8>>; 3], args: &Args) -> Vec<(f32, f32)> {
    let apids = [args.apid_ch1, args.apid_ch2, args.apid_ch3];
    let height = channels[0].len();
    let num_mcu_rows = imaging::mcu_row_count(height);

    eprintln!(
        "Encoding {} MCU rows across 3 channels ({}x{} pixels)",
        num_mcu_rows, IMAGE_WIDTH, height
    );

    let day: u16 = args.ccsds_day;
    let ms_of_day: u32 = args.ccsds_ms;
    let telemetry_payload = args.telemetry_payload.as_deref();

    let mut all_source_packets: Vec<Vec<u8>> = Vec::new();
    let mut seq_count: u16 = 0;

    // M2-x cadence: 14 segments per channel + 1 telemetry packet = 43 packets/line.
    let segments_per_line: usize = 14;
    let mcus_per_segment: usize = MCUS_PER_ROW / segments_per_line;
    debug_assert_eq!(mcus_per_segment, 14);

    if let Some(payload) = args.telemetry_payload.as_ref() {
        eprintln!(
            "Using custom APID 70 telemetry payload ({} bytes); day/time and MSU-MR ID fields will be updated from CLI options",
            payload.len()
        );
    }

    for mcu_row_idx in 0..num_mcu_rows {
        let timestamp = ms_of_day.wrapping_add((mcu_row_idx as u32).wrapping_mul(500)) % 86_400_000;

        for ch in 0..3 {
            let mcus = imaging::extract_mcu_row(&channels[ch], mcu_row_idx);

            for seg_idx in 0..segments_per_line {
                let start = seg_idx * mcus_per_segment;
                let end = start + mcus_per_segment;

                let compressed = compression::compress_mcu_row(&mcus[start..end], args.quality);

                let packet = ccsds::build_msumr_segment_packet(
                    apids[ch],
                    seq_count,
                    day,
                    timestamp,
                    start as u8,
                    args.quality,
                    &compressed,
                );

                all_source_packets.push(packet);
                seq_count = (seq_count + 1) & 0x3FFF;
            }
        }

        // Keep the 43-packet cadence expected by the M2-x reader.
        let telemetry = ccsds::build_msumr_telemetry_packet(
            seq_count,
            day,
            timestamp,
            args.telemetry_msumr_id,
            telemetry_payload,
        );
        all_source_packets.push(telemetry);
        seq_count = (seq_count + 1) & 0x3FFF;

        if (mcu_row_idx + 1) % 25 == 0 || mcu_row_idx == num_mcu_rows - 1 {
            eprintln!("  Compressed MCU row {}/{}", mcu_row_idx + 1, num_mcu_rows);
        }
    }

    eprintln!(
        "Generated {} source packets ({} bytes total)",
        all_source_packets.len(),
        all_source_packets.iter().map(|p| p.len()).sum::<usize>()
    );

    // Multiplex into VCDUs
    let mut vcdu_builder = ccsds::VcduBuilder::new(args.scid, args.vcid);
    let vcdus = vcdu_builder.multiplex(&all_source_packets);
    eprintln!("Multiplexed into {} VCDUs", vcdus.len());

    // Encode each VCDU through the channel coding chain
    eprintln!("Applying channel coding (RS + Scramble + NRZ-M + Conv)...");
    let mut encoded_cadus: Vec<Vec<u8>> = Vec::with_capacity(vcdus.len());
    let mut raw_cadu_stream: Option<Vec<u8>> = args
        .dump_cadu_stream
        .as_ref()
        .map(|_| Vec::with_capacity(vcdus.len() * CADU_SIZE));
    let mut conv_state: u8 = 0;
    let mut nrzm_last_bit: u8 = 0;
    for (idx, vcdu) in vcdus.iter().enumerate() {
        let raw_cadu = ccsds::build_cadu(vcdu);

        if idx == 0 {
            if let Some(path) = args.dump_first_cadu.as_ref() {
                match std::fs::write(path, &raw_cadu) {
                    Ok(()) => eprintln!("Wrote first raw CADU to {:?}", path),
                    Err(e) => eprintln!("Warning: failed to write CADU dump to {:?}: {}", path, e),
                }
            }
        }

        if let Some(stream) = raw_cadu_stream.as_mut() {
            stream.extend_from_slice(&raw_cadu);
        }

        let diff_cadu = differential::nrzm_encode_bits(&raw_cadu, &mut nrzm_last_bit);
        encoded_cadus.push(convolutional::encode_with_state(
            &diff_cadu,
            &mut conv_state,
        ));
    }

    if let Some(path) = args.dump_cadu_stream.as_ref() {
        if let Some(stream) = raw_cadu_stream.as_ref() {
            match std::fs::write(path, stream) {
                Ok(()) => eprintln!(
                    "Wrote raw CADU stream ({} frames) to {:?}",
                    vcdus.len(),
                    path
                ),
                Err(e) => eprintln!("Warning: failed to write CADU stream to {:?}: {}", path, e),
            }
        }
    }

    // Calculate transmission duration
    let total_symbols: usize = encoded_cadus.iter().map(|c| c.len() * 4).sum(); // 4 dibits per byte
    let duration_s = total_symbols as f32 / SYMBOL_RATE as f32;
    eprintln!(
        "Total: {} symbols, {:.2}s transmission at {} ksps",
        total_symbols,
        duration_s,
        SYMBOL_RATE / 1000
    );

    // QPSK modulate
    eprintln!("QPSK modulating with RRC pulse shaping...");
    modulation::modulate_cadu_stream(&encoded_cadus)
}

/// Load channel data from file arguments
fn load_channels(args: &Args) -> Result<[Vec<Vec<u8>>; 3], Box<dyn std::error::Error>> {
    let input = args
        .input
        .as_ref()
        .ok_or("Input image is required for file mode")?;

    if args.rgb {
        // Split color image into R, G, B channels
        let (r, g, b) = imaging::load_image_rgb(input.as_path())?;
        return Ok([r, g, b]);
    }

    // Load channel 1
    let ch1 = imaging::load_image(input.as_path())?;
    let height = ch1.len() as u32;

    // Load channel 2 (or duplicate ch1)
    let ch2 = if let Some(ref path) = args.input_ch2 {
        imaging::load_image_to_height(path.as_path(), height)?
    } else {
        ch1.clone()
    };

    // Load channel 3 (or duplicate ch1)
    let ch3 = if let Some(ref path) = args.input_ch3 {
        imaging::load_image_to_height(path.as_path(), height)?
    } else {
        ch1.clone()
    };

    Ok([ch1, ch2, ch3])
}

/// Streaming encode: produce and emit I/Q samples per MCU row so that downstream
/// consumers (HackRF via stdout pipe) never starve.
fn encode_channels_streaming(
    channels: &[Vec<Vec<u8>>; 3],
    args: &Args,
    mut write: impl FnMut(&[(f32, f32)]) -> Result<(), Box<dyn std::error::Error>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let apids = [args.apid_ch1, args.apid_ch2, args.apid_ch3];
    let height = channels[0].len();
    let num_mcu_rows = imaging::mcu_row_count(height);

    eprintln!(
        "Encoding {} MCU rows across 3 channels ({}x{} pixels) [streaming]",
        num_mcu_rows, IMAGE_WIDTH, height
    );

    let day: u16 = args.ccsds_day;
    let ms_of_day: u32 = args.ccsds_ms;
    let telemetry_payload = args.telemetry_payload.as_deref();

    let segments_per_line: usize = 14;
    let mcus_per_segment: usize = MCUS_PER_ROW / segments_per_line;
    debug_assert_eq!(mcus_per_segment, 14);

    let mut vcdu_builder = ccsds::VcduBuilder::new(args.scid, args.vcid);
    let mut modulator = modulation::OqpskModulator::new();
    let mut conv_state: u8 = 0;
    let mut nrzm_last_bit: u8 = 0;
    let mut seq_count: u16 = 0;
    let mut total_vcdus: usize = 0;

    for mcu_row_idx in 0..num_mcu_rows {
        let timestamp = ms_of_day.wrapping_add((mcu_row_idx as u32).wrapping_mul(500)) % 86_400_000;

        for ch in 0..3 {
            let mcus = imaging::extract_mcu_row(&channels[ch], mcu_row_idx);
            for seg_idx in 0..segments_per_line {
                let start = seg_idx * mcus_per_segment;
                let end = start + mcus_per_segment;
                let compressed = compression::compress_mcu_row(&mcus[start..end], args.quality);
                let packet = ccsds::build_msumr_segment_packet(
                    apids[ch],
                    seq_count,
                    day,
                    timestamp,
                    start as u8,
                    args.quality,
                    &compressed,
                );
                vcdu_builder.push_packet(&packet);
                seq_count = (seq_count + 1) & 0x3FFF;
            }
        }

        let telemetry = ccsds::build_msumr_telemetry_packet(
            seq_count,
            day,
            timestamp,
            args.telemetry_msumr_id,
            telemetry_payload,
        );
        vcdu_builder.push_packet(&telemetry);
        seq_count = (seq_count + 1) & 0x3FFF;

        // Drain any complete VCDUs produced by this MCU row and stream them out.
        for vcdu in vcdu_builder.drain_vcdus() {
            let raw_cadu = ccsds::build_cadu(&vcdu);
            let diff_cadu = differential::nrzm_encode_bits(&raw_cadu, &mut nrzm_last_bit);
            let coded = convolutional::encode_with_state(&diff_cadu, &mut conv_state);
            let samples = modulator.feed(&coded);
            write(&samples)?;
            total_vcdus += 1;
        }

        if (mcu_row_idx + 1) % 25 == 0 || mcu_row_idx == num_mcu_rows - 1 {
            eprintln!(
                "  Row {}/{} ({} VCDUs sent)",
                mcu_row_idx + 1,
                num_mcu_rows,
                total_vcdus
            );
        }
    }

    // Flush final partial VCDU if any.
    for vcdu in vcdu_builder.finalize() {
        let raw_cadu = ccsds::build_cadu(&vcdu);
        let diff_cadu = differential::nrzm_encode_bits(&raw_cadu, &mut nrzm_last_bit);
        let coded = convolutional::encode_with_state(&diff_cadu, &mut conv_state);
        let samples = modulator.feed(&coded);
        write(&samples)?;
        total_vcdus += 1;
    }

    // Drain RRC filter tail so the very last symbols are fully shaped.
    let tail = modulator.flush();
    write(&tail)?;

    eprintln!("Streamed {} VCDUs total.", total_vcdus);
    Ok(())
}

/// File mode: encode image(s) to output file
fn run_file_mode(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let channels = load_channels(args)?;

    // Handle repeat
    let channels = if args.repeat > 1 {
        eprintln!("Repeating image {} times", args.repeat);
        let mut repeated = [Vec::new(), Vec::new(), Vec::new()];
        for _ in 0..args.repeat {
            for ch in 0..3 {
                repeated[ch].extend_from_slice(&channels[ch]);
            }
        }
        repeated
    } else {
        channels
    };

    if args.stdout {
        // Streaming path: emit samples incrementally so downstream SDR never starves.
        use std::io::{self, Write};
        let stdout = io::stdout();
        let handle = stdout.lock();
        let mut buffered = io::BufWriter::with_capacity(64 * 1024, handle);
        encode_channels_streaming(&channels, args, |samples| {
            for &(i_val, q_val) in samples {
                let is = (i_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let qs = (q_val * 32767.0).clamp(-32768.0, 32767.0) as i16;
                buffered.write_all(&is.to_le_bytes())?;
                buffered.write_all(&qs.to_le_bytes())?;
            }
            Ok(())
        })?;
        buffered.flush()?;
        return Ok(());
    }

    let samples = encode_channels(&channels, args);

    if let Some(ref out_path) = args.output {
        let ext = out_path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("wav");

        match ext {
            "raw" | "bin" | "iq" => output::write_iq_raw(out_path, &samples)?,
            _ => output::write_iq_wav(out_path, &samples)?,
        }
    } else {
        return Err("Either --output or --stdout is required".into());
    }

    Ok(())
}

/// Camera capture mode
#[cfg(feature = "input-v4l")]
fn run_capture_mode(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        eprintln!("\nReceived Ctrl+C, finishing current frame...");
        r.store(false, Ordering::SeqCst);
    })?;

    let cam_idx = CameraIndex::Index(args.cam_idx);
    let format = RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
    let mut camera = Camera::new(cam_idx, format)?;
    camera.open_stream()?;
    eprintln!("Camera {} opened", args.cam_idx);

    if args.stdout {
        // Stream mode: output raw I/Q to stdout
        loop {
            if !running.load(Ordering::SeqCst) {
                break;
            }

            let frame = camera.frame()?;
            let rgb_image = frame.decode_image::<RgbFormat>()?;
            eprintln!(
                "Captured frame {}x{}",
                rgb_image.width(),
                rgb_image.height()
            );

            let rows = imaging::camera_frame_to_rows(&rgb_image);
            let channels = [rows.clone(), rows.clone(), rows];
            let samples = encode_channels(&channels, args);
            output::write_iq_stdout(&samples)?;

            if !args.continuous {
                break;
            }
        }
    } else if let Some(ref out_path) = args.output {
        let mut wav_writer = output::StreamingWavWriter::new(out_path)?;

        loop {
            if !running.load(Ordering::SeqCst) {
                break;
            }

            let frame = camera.frame()?;
            let rgb_image = frame.decode_image::<RgbFormat>()?;
            eprintln!(
                "Captured frame {}x{}",
                rgb_image.width(),
                rgb_image.height()
            );

            let rows = imaging::camera_frame_to_rows(&rgb_image);
            let channels = [rows.clone(), rows.clone(), rows];
            let samples = encode_channels(&channels, args);
            wav_writer.write_samples(&samples)?;

            if !args.continuous {
                break;
            }
        }

        let count = wav_writer.finalize()?;
        eprintln!("Wrote {} I/Q samples to {:?}", count, out_path);
    } else {
        return Err("Either --output or --stdout is required for capture mode".into());
    }

    Ok(())
}

#[cfg(not(feature = "input-v4l"))]
fn run_capture_mode(_args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    Err("Camera capture requires building with --features input-v4l".into())
}

fn main() {
    let args = Args::parse();

    let result = if args.capture {
        run_capture_mode(&args)
    } else {
        run_file_mode(&args)
    };

    if let Err(e) = result {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}
