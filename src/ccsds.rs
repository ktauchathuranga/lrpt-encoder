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
) -> Vec<u8> {
    let mut payload = vec![0u8; 62];

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

/// VCDU builder - multiplexes source packets into fixed-size VCDUs
pub struct VcduBuilder {
    scid: u8,
    vcid: u8,
    vcdu_counter: u32,
}

impl VcduBuilder {
    pub fn new(scid: u8, vcid: u8) -> Self {
        VcduBuilder {
            scid,
            vcid,
            vcdu_counter: 0,
        }
    }

    /// Multiplex a list of source packets into VCDUs
    /// Returns a list of 892-byte VCDU frames
    pub fn multiplex(&mut self, packets: &[Vec<u8>]) -> Vec<Vec<u8>> {
        // Concatenate all packet data
        let mut packet_stream: Vec<u8> = Vec::new();
        let mut packet_offsets: Vec<usize> = Vec::new(); // Start offset of each packet
        for pkt in packets {
            packet_offsets.push(packet_stream.len());
            packet_stream.extend_from_slice(pkt);
        }

        let mut vcdus = Vec::new();
        let mut stream_pos: usize = 0;

        while stream_pos < packet_stream.len() {
            let mut vcdu = Vec::with_capacity(VCDU_DATA_SIZE);

            // VCDU Primary Header (6 bytes)
            // Version(01) | SCID(8 bits) | VCID(6 bits)
            let word0: u16 = (0b01 << 14) | ((self.scid as u16) << 6) | (self.vcid as u16 & 0x3F);
            vcdu.push((word0 >> 8) as u8);
            vcdu.push(word0 as u8);

            // VCDU Counter (24 bits)
            let counter = self.vcdu_counter & 0xFFFFFF;
            vcdu.push((counter >> 16) as u8);
            vcdu.push((counter >> 8) as u8);
            vcdu.push(counter as u8);

            // Replay flag(0) | Spare(0000000)
            vcdu.push(0x00);

            // VCDU insert zone (2 bytes in Meteor LRPT AOS frames)
            vcdu.push(0x00);
            vcdu.push(0x00);

            // MPDU Header (2 bytes)
            // Spare(5 bits) | First Header Pointer(11 bits)
            // Find if any packet starts within this VCDU's data zone
            let data_start = stream_pos;
            let data_end = (stream_pos + MPDU_DATA_SIZE).min(packet_stream.len());
            let mut first_header_ptr: u16 = 0x7FF; // No packet header in this frame
            for &offset in &packet_offsets {
                if offset >= data_start && offset < data_end {
                    first_header_ptr = (offset - data_start) as u16;
                    break;
                }
            }

            vcdu.push((first_header_ptr >> 8) as u8);
            vcdu.push(first_header_ptr as u8);

            // Data zone
            vcdu.extend_from_slice(&packet_stream[data_start..data_end]);

            // Pad with fill data (0xFF) if needed
            while vcdu.len() < VCDU_DATA_SIZE {
                vcdu.push(0xFF);
            }

            assert_eq!(vcdu.len(), VCDU_DATA_SIZE);
            vcdus.push(vcdu);

            stream_pos += MPDU_DATA_SIZE;
            self.vcdu_counter += 1;
        }

        // If no packets at all, generate at least one idle VCDU
        if vcdus.is_empty() {
            vcdus.push(self.build_idle_vcdu());
        }

        vcdus
    }

    /// Build an idle (fill) VCDU
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

/// Apply the full CCSDS channel coding chain to a VCDU:
/// RS encode → Scramble → ASM + NRZ-M → Conv encode
/// Returns the convolutionally-encoded CADU bits as bytes
pub fn encode_vcdu_with_state(vcdu: &[u8], conv_state: &mut u8, nrzm_last_bit: &mut u8) -> Vec<u8> {
    use crate::convolutional;
    use crate::differential;
    let cadu = build_cadu(vcdu);

    // 4. NRZ-M encode CADU bits (continuous across CADUs)
    let diff_cadu = differential::nrzm_encode_bits(&cadu, nrzm_last_bit);

    // 5. Convolutional encode (rate 1/2, doubles the bits)
    convolutional::encode_with_state(&diff_cadu, conv_state)
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
    fn test_encode_vcdu_output_length() {
        let vcdu = vec![0u8; VCDU_DATA_SIZE];
        let mut conv_state: u8 = 0;
        let mut nrzm_last_bit: u8 = 0;
        let encoded = encode_vcdu_with_state(&vcdu, &mut conv_state, &mut nrzm_last_bit);
        // CADU = 1024 bytes = 8192 bits
        // Conv rate 1/2: 16384 bits = 2048 bytes
        assert_eq!(encoded.len(), CADU_SIZE * 2);
    }

    #[test]
    fn test_vcdu_counter_increments() {
        let mut builder = VcduBuilder::new(DEFAULT_SCID, DEFAULT_VCID);
        let big_data = vec![0u8; MPDU_DATA_SIZE * 3];
        let vcdus = builder.multiplex(&[big_data]);
        assert!(vcdus.len() >= 3);
    }
}
