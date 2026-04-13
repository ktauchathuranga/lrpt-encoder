# lrpt-encoder

Encode images into Meteor-M N2-4 LRPT (Low Rate Picture Transmission) digital signals in Rust.

Takes an image (or camera feed), runs it through the full CCSDS digital transmission chain used by the Meteor-M2-4 weather satellite, and emits baseband I/Q samples ready for SDR transmission. Output is decodable by [SatDump](https://www.satdump.org/) and any standards-compliant LRPT decoder.

## Features

- **Full satellite-faithful digital chain**: JPEG-like DCT/Huffman compression → CCSDS Space Packets → VCDU multiplexing → Reed-Solomon(255,223) interleave-4 → CCSDS scrambler → NRZ-M differential encoding → rate-1/2 K=7 convolutional coding → OQPSK with RRC pulse shaping
- **3-channel MSU-MR emulation** (APIDs 64/65/66) with telemetry packet (APID 70) cadence
- **Output formats**: 2-channel WAV (16-bit I/Q at 288 kHz), raw float32 I/Q, raw int16 stdout streaming
- **Camera capture mode** (optional, V4L2 via `nokhwa`)
- **Configurable**: SCID, VCID, APIDs, JPEG quality, CCSDS timestamp, telemetry payload
- **Debug dumps** for raw pre-modulation CADU inspection

## Build

```sh
cargo build --release
```

With camera support:

```sh
cargo build --release --features input-v4l
```

The binary is produced at `target/release/lrpt-encoder`.

## Quick start

Encode a single image to a WAV file containing baseband I/Q:

```sh
./target/release/lrpt-encoder a.png -o output.wav
```

Open the resulting `output.wav` in SatDump:
1. **Offline processing** → **METEOR M2-x LRPT 72k**
2. Input format: `cs16` (or `wav`), sample rate **288000**
3. Satellite: **M2-4**

## Usage

```
lrpt-encoder [OPTIONS] [INPUT_IMAGE]
```

### Common options

| Flag | Default | Description |
|------|---------|-------------|
| `-o`, `--output <FILE>` | — | Output file (`.wav`, `.raw`, `.bin`, `.iq`) |
| `--stdout` | off | Stream raw int16 I/Q to stdout (for piping to SDR tools) |
| `-r`, `--repeat <N>` | 1 | Repeat the image N times |
| `--quality <1-100>` | 80 | JPEG compression quality |
| `--rgb` | off | Split a color image: R → ch1, G → ch2, B → ch3 |
| `--input-ch2 <FILE>` | — | Separate image for channel 2 |
| `--input-ch3 <FILE>` | — | Separate image for channel 3 |

### CCSDS / telemetry options

| Flag | Default | Description |
|------|---------|-------------|
| `--scid <N>` | 57 | Spacecraft ID |
| `--vcid <N>` | 5 | Virtual Channel ID |
| `--apid-ch1/2/3 <N>` | 64/65/66 | APID per MSU-MR channel |
| `--msumr-id <0-15>` | 4 | MSU-MR instrument ID in telemetry |
| `--ccsds-day <N>` | 24938 | CCSDS day (days since 1958-01-01) |
| `--ccsds-ms <N>` | 43200000 | Time-of-day in milliseconds |
| `--telemetry-payload <HEX>` | — | Custom APID-70 payload (`AABBCC`, `AA BB CC`, `0xAA,0xBB`) |

### Camera capture (requires `--features input-v4l`)

| Flag | Description |
|------|-------------|
| `--capture` | Enable camera capture |
| `--continuous` | Keep capturing until Ctrl+C |
| `--cam-idx <N>` | V4L2 device index (default 0) |

### Debug

| Flag | Description |
|------|-------------|
| `--dump-first-cadu <FILE>` | Write first 1024-byte raw CADU before NRZ-M/Conv |
| `--dump-cadu-stream <FILE>` | Write entire raw CADU stream before NRZ-M/Conv |

## Examples

Single-image encode (same image used for all 3 MSU-MR channels):

```sh
./target/release/lrpt-encoder a.png -o output.wav
```

Three separate channel images (ch1, ch2, ch3), repeated 10 times:

```sh
./target/release/lrpt-encoder a.png --input-ch2 b.png --input-ch3 c.png -r 10 -o output.wav
```

Color image split into RGB channels (R → ch1, G → ch2, B → ch3):

```sh
./target/release/lrpt-encoder a.png --rgb -o output.wav
```

Stream to HackRF via GNU Radio companion script:

```sh
./target/release/lrpt-encoder a.png --stdout | python3 qpsk_stream.py
```

Custom CCSDS timestamp, SCID, and MSU-MR instrument ID:

```sh
./target/release/lrpt-encoder a.png \
    --scid 57 --vcid 5 \
    --ccsds-day 24938 --ccsds-ms 43200000 \
    --msumr-id 4 \
    -o output.wav
```

Custom APID-70 telemetry payload (raw hex):

```sh
# 62-byte payload; day/ms/MSU-MR ID fields are overwritten from CLI flags
./target/release/lrpt-encoder a.png \
    --telemetry-payload "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 40 DE AD BE EF" \
    --msumr-id 4 --ccsds-day 24938 --ccsds-ms 43200000 \
    -o output.wav
```

Three channels with custom APIDs and quality:

```sh
./target/release/lrpt-encoder a.png --input-ch2 b.png --input-ch3 c.png \
    --apid-ch1 64 --apid-ch2 65 --apid-ch3 66 \
    --quality 90 \
    -o output.wav
```

Debug dump of raw CADUs before NRZ-M/convolutional coding:

```sh
./target/release/lrpt-encoder a.png \
    --dump-first-cadu first_cadu.bin \
    --dump-cadu-stream all_cadus.bin \
    -o output.wav
```

## Signal parameters (Meteor-M2-4)

| Parameter | Value |
|-----------|-------|
| Symbol rate | 72 ksps |
| Samples/symbol | 4 |
| Sample rate | 288 kHz |
| Modulation | OQPSK |
| Pulse shaping | Root Raised Cosine, α = 0.5, span 33 symbols |
| Frame sync (ASM) | `0x1ACFFC1D` |
| VCDU size | 892 bytes |
| Reed-Solomon | (255,223) GF(2⁸), interleave 4, first root α^112 |
| Scrambler | CCSDS PN, x⁸+x⁷+x⁵+x³+1, init 0xFF |
| Differential | NRZ-M (continuous across CADUs) |
| Convolutional | rate 1/2, K=7, G1=171₈, G2=133₈ |
| Image width | 1568 px (196 MCUs × 8 px) |
| Default frequency | 433 MHz (in companion GRC script) |

## Project layout

```
src/
├── constants.rs       LRPT signal parameters, JPEG tables
├── imaging.rs         Image loading, MCU extraction
├── compression.rs     DCT + quantization + Huffman
├── ccsds.rs           Source packets, VCDU multiplexer, CADU builder
├── reed_solomon.rs    RS(255,223) interleave-4
├── scrambler.rs       CCSDS PN scrambler
├── differential.rs    NRZ-M encoder
├── convolutional.rs   Rate-1/2 K=7 convolutional encoder (stateful)
├── modulation.rs      OQPSK + RRC pulse shaping
├── output.rs          WAV / raw / stdout sinks
└── main.rs            CLI + encoding pipeline
```

## Testing

```sh
cargo test --release
```

## License

MIT — see [LICENSE](LICENSE).

## References

- CCSDS 131.0-B-3 — TM Synchronization and Channel Coding
- CCSDS 132.0-B-3 — TM Space Data Link Protocol
- CCSDS 120.0-G-3 — TM Synchronization and Channel Coding Summary of Concept and Rationale
- J.-M. Friedt, *Decoding digital weather satellite images: the LRPT protocol from Meteor-M2* (2019)
- [SatDump](https://github.com/SatDump/SatDump) — reference decoder
