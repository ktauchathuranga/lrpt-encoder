#!/usr/bin/env python3
"""
LRPT QPSK Baseband → HackRF Transmitter

Reads raw I/Q samples (interleaved int16 LE) from stdin and transmits
via HackRF using GNU Radio + gr-osmosdr.

The lrpt-encoder outputs baseband QPSK I/Q at 288 kHz sample rate.
This script resamples to the HackRF's sample rate and transmits.

Usage:
    lrpt-encoder photo.jpg --stdout 2>/dev/null | python3 qpsk_stream.py
    lrpt-encoder --capture --stdout --continuous --cam-idx 0 2>/dev/null | \\
        python3 qpsk_stream.py --freq 433000000 --gain 0

Requirements:
    - GNU Radio 3.10+
    - gr-osmosdr (for HackRF support)
    - pip install numpy (usually included with GNU Radio)

WARNING: Transmitting on frequencies you are not licensed for is illegal.
         Default frequency is 433 MHz (ISM band in many regions).
         Always use minimum necessary power (--gain 0) for testing.
"""

import argparse
import sys
import struct
import numpy as np

try:
    from gnuradio import gr, blocks, filter as gr_filter
    from gnuradio.filter import firdes
    import osmosdr
except ImportError:
    print("Error: GNU Radio and gr-osmosdr are required.", file=sys.stderr)
    print("Install with: sudo dnf install gnuradio gr-osmosdr", file=sys.stderr)
    sys.exit(1)


class LRPTTransmitter(gr.top_block):
    def __init__(self, freq, gain, amp_on, input_rate=288000, sdr_rate=2000000):
        gr.top_block.__init__(self, "LRPT Transmitter")

        # Source: stdin as interleaved int16 I/Q
        self.source = blocks.file_descriptor_source(
            gr.sizeof_short * 1, 0  # stdin = fd 0
        )

        # Convert int16 to float
        self.short_to_float = blocks.short_to_float(1, 32768.0)

        # Deinterleave I and Q
        self.deinterleave = blocks.deinterleave(gr.sizeof_float)

        # Combine into complex
        self.float_to_complex = blocks.float_to_complex(1)

        # Rational resampler: 288000 → 2000000
        # 2000000 / 288000 = 125/18
        self.resampler = gr_filter.rational_resampler_ccc(
            interpolation=125,
            decimation=18,
            taps=firdes.low_pass(1.0, sdr_rate, input_rate / 2, input_rate / 10),
        )

        # HackRF sink
        self.sink = osmosdr.sink(args="hackrf=0")
        self.sink.set_sample_rate(sdr_rate)
        self.sink.set_center_freq(freq, 0)
        self.sink.set_freq_corr(0, 0)
        self.sink.set_gain(gain, 0)
        self.sink.set_if_gain(gain, 0)
        self.sink.set_bb_gain(gain, 0)

        if amp_on:
            self.sink.set_gain(14, "AMP", 0)
            print(f"Amplifier: ON (+14 dB)", file=sys.stderr)
        else:
            self.sink.set_gain(0, "AMP", 0)
            print(f"Amplifier: OFF", file=sys.stderr)

        # Connect the flowgraph
        self.connect(self.source, self.short_to_float)
        self.connect(self.short_to_float, self.deinterleave)
        self.connect((self.deinterleave, 0), (self.float_to_complex, 0))  # I
        self.connect((self.deinterleave, 1), (self.float_to_complex, 1))  # Q
        self.connect(self.float_to_complex, self.resampler)
        self.connect(self.resampler, self.sink)

        print(f"Frequency: {freq / 1e6:.3f} MHz", file=sys.stderr)
        print(f"TX Gain:   {gain} dB", file=sys.stderr)
        print(f"Input:     {input_rate} Hz (QPSK baseband)", file=sys.stderr)
        print(f"SDR Rate:  {sdr_rate} Hz", file=sys.stderr)
        print(f"Modulation: QPSK @ 72 ksps (Meteor LRPT)", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Transmit LRPT QPSK baseband via HackRF"
    )
    parser.add_argument(
        "--freq", "-f",
        type=int,
        default=433000000,
        help="Transmit frequency in Hz (default: 433000000)"
    )
    parser.add_argument(
        "--gain", "-g",
        type=int,
        default=0,
        help="TX gain in dB (0-47, default: 0)"
    )
    parser.add_argument(
        "--amp-on",
        action="store_true",
        default=False,
        help="Enable 14 dB amplifier (default: off)"
    )
    parser.add_argument(
        "--amp-off",
        action="store_true",
        default=True,
        help="Disable amplifier (default)"
    )
    parser.add_argument(
        "--sdr-rate",
        type=int,
        default=2000000,
        help="SDR sample rate in Hz (default: 2000000)"
    )

    args = parser.parse_args()

    amp = args.amp_on and not args.amp_off

    print("=" * 50, file=sys.stderr)
    print("LRPT Encoder → HackRF Transmitter", file=sys.stderr)
    print("=" * 50, file=sys.stderr)

    try:
        tb = LRPTTransmitter(
            freq=args.freq,
            gain=args.gain,
            amp_on=amp,
            sdr_rate=args.sdr_rate,
        )
        tb.start()
        print("Transmitting... Press Ctrl+C to stop.", file=sys.stderr)
        tb.wait()
    except KeyboardInterrupt:
        print("\nStopping transmission.", file=sys.stderr)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
