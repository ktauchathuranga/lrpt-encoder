/// CCSDS Space Data Link Protocol implementation
///
/// Implements the packet/frame structure used by Meteor-M satellites:
/// - CCSDS Source Packets (Space Packet Protocol)
/// - MPDU (Multiplexing Protocol Data Unit)
/// - VCDU (Virtual Channel Data Unit)
///
/// Frame structure: VCDU(892 bytes) → RS → Scramble → ASM → Conv → QPSK

use crate::constants::*;

/// Build a CCSDS source packet from a payload.
///
/// The payload is copied as-is after the 6-byte primary header.
pub fn build_source_packet(
    apid: u16,
    seq_count: u16,
    payload: &[u8],
) -> Vec<u8> {
    let packet_data_length = (payload.len() - 1) as u16;

    let mut packet = Vec::with_capacity(6 + payload.len());

    // Primary Header (6 bytes)
    // Version(000) | Type(0) | SecHdrFlag(1) | APID(11 bits)
    let word0: u16 = (0b000_0_1 << 11) | (apid & 0x7FF);
    packet.push((word0 >> 8) as u8);
    packet.push(word0 as u8);

    // SeqFlags(11=standalone) | SeqCount(14 bits)
    let word1: u16 = (0b11 << 14) | (seq_count & 0x3FFF);
    packet.push((word1 >> 8) as u8);
    packet.push(word1 as u8);

    // Packet Data Length
    packet.push((packet_data_length >> 8) as u8);
    packet.push(packet_data_length as u8);

    packet.extend_from_slice(payload);

    packet
}

/// Build one M2-x LRPT MSU-MR segment packet.
///
/// Payload format expected by SatDump's `Segment` parser:
/// [day(2), ms(4), us(2), MCUN(1), QT(1), DC/AC(1), QFM(2), QF(1), data...]
pub fn build_msumr_segment_packet(
    apid: u16,
    seq_count: u16,
    day: u16,
    ms_of_day: u32,
    mcu_number: u8,
    quality: u8,
    compressed_data: &[u8],
) -> Vec<u8> {
    let mut payload = Vec::with_capacity(14 + compressed_data.len());

    // CCSDS day segmented time
    payload.push((day >> 8) as u8);
    payload.push(day as u8);
    payload.push((ms_of_day >> 24) as u8);
    payload.push((ms_of_day >> 16) as u8);
    payload.push((ms_of_day >> 8) as u8);
    payload.push(ms_of_day as u8);
    payload.push(0x00); // microseconds MSB
    payload.push(0x00); // microseconds LSB

    // Segment header expected by SatDump LRPT reader
    payload.push(mcu_number); // MCUN: 0,14,28,...,182
    payload.push(0x00); // QT
    payload.push(0x00); // DC/AC packed nibbles
    payload.push(0xFF); // QFM high
    payload.push(0xF0); // QFM low
    payload.push(quality); // QF

    payload.extend_from_slice(compressed_data);

    build_source_packet(apid, seq_count, &payload)
}

/// Build one APID 70 telemetry packet.
///
/// M2-x uses a 43-packet transmission cycle: 14 segments * 3 channels + 1
/// telemetry packet. Keeping this cadence helps SatDump line reconstruction.
pub fn build_msumr_telemetry_packet(
    seq_count: u16,
    day: u16,
    ms_of_day: u32,
    msumr_id: u8,
    payload_override: Option<&[u8]>,
) -> Vec<u8> {
    let mut payload = payload_override
        .map(|bytes| bytes.to_vec())
        .unwrap_or_else(|| vec![0u8; 62]);

    // Ensure required field offsets exist even if a short custom payload is provided.
    if payload.len() < 21 {
        payload.resize(21, 0x00);
    }

    payload[0] = (day >> 8) as u8;
    payload[1] = day as u8;
    payload[2] = (ms_of_day >> 24) as u8;
    payload[3] = (ms_of_day >> 16) as u8;
    payload[4] = (ms_of_day >> 8) as u8;
    payload[5] = ms_of_day as u8;

    // SatDump parses MSU-MR ID from payload[8 + 12] high nibble.
    payload[20] = (msumr_id & 0x0F) << 4;

    build_source_packet(70, seq_count, &payload)
}

/// VCDU builder - multiplexes source packets into fixed-size VCDUs.
///
/// Supports two modes:
/// - One-shot: [`multiplex`] takes a full list of packets and returns all VCDUs.
/// - Streaming: [`push_packet`] to add packets incrementally, [`drain_vcdus`]
///   to emit complete VCDUs so far, and [`finalize`] to pad and emit the
///   last partial frame.
pub struct VcduBuilder {
    scid: u8,
    vcid: u8,
    vcdu_counter: u32,
    /// Bytes waiting to be packed into VCDUs.
    pending: Vec<u8>,
    /// Offsets of packet starts within `pending` (relative to start of `pending`).
    pending_offsets: Vec<usize>,
}

impl VcduBuilder {
    pub fn new(scid: u8, vcid: u8) -> Self {
        VcduBuilder {
            scid,
            vcid,
            vcdu_counter: 0,
            pending: Vec::new(),
            pending_offsets: Vec::new(),
        }
    }

    /// Streaming: append one source packet to the pending buffer.
    pub fn push_packet(&mut self, pkt: &[u8]) {
        self.pending_offsets.push(self.pending.len());
        self.pending.extend_from_slice(pkt);
    }

    /// Streaming: emit all complete VCDUs available given currently-pending bytes.
    pub fn drain_vcdus(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while self.pending.len() >= MPDU_DATA_SIZE {
            let vcdu = self.build_one_vcdu(MPDU_DATA_SIZE, false);
            out.push(vcdu);
            // Drop the consumed MPDU_DATA_SIZE bytes from pending and shift offsets.
            self.pending.drain(..MPDU_DATA_SIZE);
            self.pending_offsets.retain(|&o| o >= MPDU_DATA_SIZE);
            for off in &mut self.pending_offsets {
                *off -= MPDU_DATA_SIZE;
            }
        }
        out
    }

    /// Streaming: flush remaining pending bytes (padded) as a final VCDU.
    /// If nothing is pending, emits one idle VCDU.
    pub fn finalize(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        if !self.pending.is_empty() {
            let taken = self.pending.len();
            let vcdu = self.build_one_vcdu(taken, true);
            self.pending.clear();
            self.pending_offsets.clear();
            out.push(vcdu);
        } else if self.vcdu_counter == 0 {
            out.push(self.build_idle_vcdu());
        }
        out
    }

    /// Build one VCDU consuming `take` bytes from the front of `pending`.
    fn build_one_vcdu(&mut self, take: usize, pad: bool) -> Vec<u8> {
        let mut vcdu = Vec::with_capacity(VCDU_DATA_SIZE);

        // VCDU Primary Header (6 bytes)
        let word0: u16 = (0b01 << 14) | ((self.scid as u16) << 6) | (self.vcid as u16 & 0x3F);
        vcdu.push((word0 >> 8) as u8);
        vcdu.push(word0 as u8);

        let counter = self.vcdu_counter & 0xFFFFFF;
        vcdu.push((counter >> 16) as u8);
        vcdu.push((counter >> 8) as u8);
        vcdu.push(counter as u8);
        vcdu.push(0x00);

        // VCDU insert zone (2 bytes)
        vcdu.push(0x00);
        vcdu.push(0x00);

        // First header pointer = offset of first packet start within this VCDU's data zone.
        let mut first_header_ptr: u16 = 0x7FF;
        for &offset in &self.pending_offsets {
            if offset < take {
                first_header_ptr = offset as u16;
                break;
            }
        }

        vcdu.push((first_header_ptr >> 8) as u8);
        vcdu.push(first_header_ptr as u8);

        vcdu.extend_from_slice(&self.pending[..take]);

        if pad {
            while vcdu.len() < VCDU_DATA_SIZE {
                vcdu.push(0xFF);
            }
        }
        assert_eq!(vcdu.len(), VCDU_DATA_SIZE);

        self.vcdu_counter += 1;
        vcdu
    }

    /// One-shot: multiplex a list of source packets into VCDUs.
    pub fn multiplex(&mut self, packets: &[Vec<u8>]) -> Vec<Vec<u8>> {
        for pkt in packets {
            self.push_packet(pkt);
        }
        let mut out = self.drain_vcdus();
        out.extend(self.finalize());
        out
    }

    fn build_idle_vcdu(&mut self) -> Vec<u8> {
        let mut vcdu = Vec::with_capacity(VCDU_DATA_SIZE);

        let word0: u16 = (0b01 << 14) | ((self.scid as u16) << 6) | (self.vcid as u16 & 0x3F);
        vcdu.push((word0 >> 8) as u8);
        vcdu.push(word0 as u8);

        let counter = self.vcdu_counter & 0xFFFFFF;
        vcdu.push((counter >> 16) as u8);
        vcdu.push((counter >> 8) as u8);
        vcdu.push(counter as u8);
        vcdu.push(0x00);

        // VCDU insert zone
        vcdu.push(0x00);
        vcdu.push(0x00);

        // First header pointer = 0x7FE (idle frame)
        vcdu.push(0x07);
        vcdu.push(0xFE);

        // Fill with 0xFF
        while vcdu.len() < VCDU_DATA_SIZE {
            vcdu.push(0xFF);
        }

        self.vcdu_counter += 1;
        vcdu
    }
}

/// Build one raw CADU (ASM + RS-coded and scrambled frame) before
/// differential and convolutional coding.
pub fn build_cadu(vcdu: &[u8]) -> Vec<u8> {
    use crate::reed_solomon;
    use crate::scrambler;

    assert_eq!(vcdu.len(), VCDU_DATA_SIZE);

    // 1. Reed-Solomon encode (892 → 1020 bytes)
    let mut coded = reed_solomon::encode_frame(vcdu);
    assert_eq!(coded.len(), CODED_FRAME_SIZE);

    // 2. Scramble the coded frame
    scrambler::ccsds_scramble(&mut coded);

    // 3. Prepend ASM to form CADU
    let mut cadu = Vec::with_capacity(CADU_SIZE);
    cadu.extend_from_slice(&CCSDS_ASM);
    cadu.extend_from_slice(&coded);
    assert_eq!(cadu.len(), CADU_SIZE);

    cadu
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_source_packet_structure() {
        let payload = vec![0x42; 15];
        let pkt = build_source_packet(64, 0, &payload);
        assert_eq!(pkt.len(), 6 + payload.len());

        // Check APID
        let apid = ((pkt[0] as u16 & 0x07) << 8) | pkt[1] as u16;
        assert_eq!(apid, 64);

        // Check sequence flags (standalone = 0b11)
        let seq_flags = (pkt[2] >> 6) & 0x03;
        assert_eq!(seq_flags, 0b11);
    }

    #[test]
    fn test_vcdu_size() {
        let mut builder = VcduBuilder::new(DEFAULT_SCID, DEFAULT_VCID);
        let pkt = vec![0u8; 200];
        let vcdus = builder.multiplex(&[pkt]);
        assert!(vcdus.iter().all(|v| v.len() == VCDU_DATA_SIZE));
    }

    #[test]
    fn test_build_cadu_length() {
        let vcdu = vec![0u8; VCDU_DATA_SIZE];
        let cadu = build_cadu(&vcdu);
        assert_eq!(cadu.len(), CADU_SIZE);
        assert_eq!(&cadu[..4], &CCSDS_ASM);
    }

    #[test]
    fn test_vcdu_counter_increments() {
        let mut builder = VcduBuilder::new(DEFAULT_SCID, DEFAULT_VCID);
        let big_data = vec![0u8; MPDU_DATA_SIZE * 3];
        let vcdus = builder.multiplex(&[big_data]);
        assert!(vcdus.len() >= 3);
    }
}
