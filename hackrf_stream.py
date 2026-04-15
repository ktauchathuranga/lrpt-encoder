#!/usr/bin/env python3
"""
LRPT Baseband → HackRF Transmitter (no GNU Radio required)

Reads raw I/Q samples (interleaved int8) from stdin, upsamples from
288 kHz to the HackRF's minimum 2 MHz sample rate, and transmits via
hackrf_transfer.

Works on macOS and Linux — only requires numpy + scipy (no gr-osmosdr).

Usage:
    lrpt-encoder photo.png --stdout --stdout-format int8 2>/dev/null | \\
        python3 hackrf_stream.py

    lrpt-encoder photo.png --stdout --stdout-format int8 2>/dev/null | \\
        python3 hackrf_stream.py --freq 433000000 --gain 20

Requirements:
    pip install numpy scipy

WARNING: Transmitting on frequencies you are not licensed for is illegal.
         Default frequency is 433 MHz (ISM band in many regions).
         Always use minimum necessary power for testing.
"""

import argparse
import subprocess
import sys
import signal

import numpy as np
from scipy.signal import resample_poly

INPUT_RATE = 288_000       # lrpt-encoder output sample rate
MIN_SDR_RATE = 2_000_000   # hackrf_transfer minimum


def main():
    parser = argparse.ArgumentParser(
        description="Transmit LRPT baseband via hackrf_transfer (no GNU Radio needed)"
    )
    parser.add_argument(
        "--freq", "-f", type=int, default=433_000_000,
        help="Transmit frequency in Hz (default: 433000000)"
    )
    parser.add_argument(
        "--gain", "-x", type=int, default=0,
        help="TX VGA gain in dB, 0-47 (default: 0)"
    )
    parser.add_argument(
        "--amp", "-a", type=int, default=0, choices=[0, 1],
        help="RF amplifier: 0=off, 1=on (default: 0)"
    )
    parser.add_argument(
        "--sdr-rate", type=int, default=MIN_SDR_RATE,
        help=f"HackRF sample rate in Hz (default: {MIN_SDR_RATE})"
    )
    parser.add_argument(
        "--repeat", "-R", action="store_true",
        help="Enable TX repeat mode in hackrf_transfer"
    )

    args = parser.parse_args()

    if args.sdr_rate < MIN_SDR_RATE:
        print(f"Error: --sdr-rate must be >= {MIN_SDR_RATE}", file=sys.stderr)
        sys.exit(1)

    # Compute resampling ratio
    from math import gcd
    g = gcd(args.sdr_rate, INPUT_RATE)
    up = args.sdr_rate // g
    down = INPUT_RATE // g

    print("=" * 55, file=sys.stderr)
    print("LRPT Encoder → HackRF (direct, no GNU Radio)", file=sys.stderr)
    print("=" * 55, file=sys.stderr)
    print(f"  Frequency:    {args.freq / 1e6:.3f} MHz", file=sys.stderr)
    print(f"  TX gain:      {args.gain} dB", file=sys.stderr)
    print(f"  RF amp:       {'ON' if args.amp else 'OFF'}", file=sys.stderr)
    print(f"  Input rate:   {INPUT_RATE} Hz", file=sys.stderr)
    print(f"  Output rate:  {args.sdr_rate} Hz", file=sys.stderr)
    print(f"  Resample:     ×{up}/{down}", file=sys.stderr)
    print("=" * 55, file=sys.stderr)

    # ---- Step 1: Read all input ----
    print("Reading input samples...", file=sys.stderr)
    raw = sys.stdin.buffer.read()
    if not raw:
        print("Error: no input data received.", file=sys.stderr)
        sys.exit(1)

    iq_int8 = np.frombuffer(raw, dtype=np.int8)
    if len(iq_int8) % 2 != 0:
        iq_int8 = iq_int8[:-1]

    n_samples = len(iq_int8) // 2
    print(f"  Read {n_samples} I/Q samples ({len(raw)} bytes)", file=sys.stderr)
    print(f"  Duration: {n_samples / INPUT_RATE:.2f}s at {INPUT_RATE} Hz", file=sys.stderr)

    # Deinterleave I and Q, normalize to [-1, 1]
    i_f = iq_int8[0::2].astype(np.float32) / 127.0
    q_f = iq_int8[1::2].astype(np.float32) / 127.0

    # ---- Step 2: Resample entire signal at once (no chunk boundaries) ----
    print("Resampling (this may take a moment)...", file=sys.stderr)
    i_up = resample_poly(i_f, up, down)
    q_up = resample_poly(q_f, up, down)

    n_out = len(i_up)
    print(f"  Output: {n_out} samples ({n_out / args.sdr_rate:.2f}s at {args.sdr_rate} Hz)", file=sys.stderr)

    # Convert back to int8, interleave
    i_out = np.clip(i_up * 127.0, -128, 127).astype(np.int8)
    q_out = np.clip(q_up * 127.0, -128, 127).astype(np.int8)
    del i_up, q_up, i_f, q_f  # free memory

    out = np.empty(n_out * 2, dtype=np.int8)
    out[0::2] = i_out
    out[1::2] = q_out
    del i_out, q_out

    out_bytes = out.tobytes()
    del out

    # ---- Step 3: Pipe to hackrf_transfer ----
    cmd = [
        "hackrf_transfer",
        "-t", "/dev/stdin",
        "-f", str(args.freq),
        "-s", str(args.sdr_rate),
        "-x", str(args.gain),
        "-a", str(args.amp),
    ]
    if args.repeat:
        cmd.append("-R")

    print(f"  hackrf cmd:   {' '.join(cmd)}", file=sys.stderr)
    print(file=sys.stderr)

    try:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    except FileNotFoundError:
        print("Error: hackrf_transfer not found in PATH.", file=sys.stderr)
        print("Install HackRF tools: https://github.com/greatscottgadgets/hackrf", file=sys.stderr)
        sys.exit(1)

    def sigint_handler(sig, frame):
        print("\nStopping...", file=sys.stderr)
        proc.terminate()
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    print("Transmitting... Press Ctrl+C to stop.", file=sys.stderr)

    try:
        # Write in blocks to allow Ctrl+C responsiveness
        block = 262144  # 256 KB write blocks
        offset = 0
        while offset < len(out_bytes):
            end = min(offset + block, len(out_bytes))
            proc.stdin.write(out_bytes[offset:end])
            offset = end
        proc.stdin.close()
        proc.wait()
    except BrokenPipeError:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        try:
            proc.stdin.close()
        except Exception:
            pass
        proc.wait()

    print(f"\nDone. {n_out} samples transmitted.", file=sys.stderr)


if __name__ == "__main__":
    main()
