/// CCSDS Reed-Solomon (255,223) Encoder
///
/// Field: GF(2^8) with primitive polynomial x^8 + x^7 + x^2 + x + 1 (0x187)
/// Generator polynomial roots: α^(112 + 11*i), i=0..31 (mod 255)
/// Interleave depth: 4
///
/// Input: 892-byte VCDU → Output: 1020-byte coded frame (892 data + 128 parity)

use crate::constants::*;

/// GF(2^8) arithmetic tables
struct GfTables {
    exp: [u8; 512], // exp[i] = α^i, doubled for convenience
    log: [u8; 256], // log[x] = i such that α^i = x
}

impl GfTables {
    fn new() -> Self {
        let mut exp = [0u8; 512];
        let mut log = [0u8; 256];

        let mut val: u16 = 1;
        for i in 0..255u16 {
            exp[i as usize] = val as u8;
            exp[(i + 255) as usize] = val as u8; // wrap-around for convenience
            log[val as usize] = i as u8;
            val <<= 1;
            if val & 0x100 != 0 {
                val ^= RS_PRIM_POLY;
            }
        }
        log[0] = 0; // undefined, but set to 0 for safety

        GfTables { exp, log }
    }

    #[inline]
    fn mul(&self, a: u8, b: u8) -> u8 {
        if a == 0 || b == 0 {
            return 0;
        }
        self.exp[(self.log[a as usize] as usize) + (self.log[b as usize] as usize)]
    }

    #[inline]
    fn div(&self, a: u8, b: u8) -> u8 {
        if a == 0 {
            return 0;
        }
        assert!(b != 0, "GF division by zero");
        let la = self.log[a as usize] as isize;
        let lb = self.log[b as usize] as isize;
        let mut idx = la - lb;
        if idx < 0 {
            idx += 255;
        }
        self.exp[idx as usize]
    }
}

/// RS(255,223) generator polynomial coefficients
/// Coefficients are in ascending order (constant term first), matching
/// libcorrect's internal representation.
fn compute_generator(gf: &GfTables) -> [u8; RS_2T + 1] {
    let mut genpoly: Vec<u8> = vec![1u8];

    for i in 0..RS_2T {
        let root_exp = ((RS_PRIM as usize) * ((RS_FIRST_ROOT as usize) + i)) % 255;
        let root = gf.exp[root_exp];

        // Multiply current polynomial by (x + root) over GF(2^8).
        let mut next = vec![0u8; genpoly.len() + 1];
        for (j, &coef) in genpoly.iter().enumerate() {
            next[j] ^= gf.mul(coef, root);
            next[j + 1] ^= coef;
        }
        genpoly = next;
    }

    let mut out = [0u8; RS_2T + 1];
    out.copy_from_slice(&genpoly);
    out
}

/// Encode a single RS(255,223) codeword
/// data: 223 bytes → returns 32 parity bytes
fn rs_encode_block(gf: &GfTables, genpoly: &[u8; RS_2T + 1], data: &[u8]) -> [u8; RS_2T] {
    assert_eq!(data.len(), RS_K);

    // Mirror libcorrect encode flow:
    // 1) Build polynomial coefficients in ascending order where data[0]
    //    is the highest-order term.
    // 2) Compute remainder of dividend mod generator.
    // 3) Emit parity from highest remainder order to lowest.
    let mut modpoly = [0u8; RS_N];
    for (i, &byte) in data.iter().enumerate() {
        modpoly[RS_N - 1 - i] = byte;
    }

    let gen_lead = genpoly[RS_2T];
    assert!(gen_lead != 0, "generator leading coefficient must be non-zero");

    // Long division from highest order down to generator order.
    for i in (RS_2T..RS_N).rev() {
        let top = modpoly[i];
        if top == 0 {
            continue;
        }

        let q_coeff = if gen_lead == 1 { top } else { gf.div(top, gen_lead) };
        let q_order = i - RS_2T;

        for (j, &g) in genpoly.iter().enumerate() {
            if g != 0 {
                modpoly[q_order + j] ^= gf.mul(g, q_coeff);
            }
        }
    }

    let mut parity = [0u8; RS_2T];
    for i in 0..RS_2T {
        parity[i] = modpoly[RS_2T - 1 - i];
    }

    parity
}

/// Encode a 892-byte VCDU frame with RS(255,223) interleave-4
/// Returns 1020-byte coded frame (892 data + 128 parity)
pub fn encode_frame(data: &[u8]) -> Vec<u8> {
    assert_eq!(data.len(), VCDU_DATA_SIZE);

    let gf = GfTables::new();
    let genpoly = compute_generator(&gf);

    // De-interleave: split 892 bytes into 4 sub-blocks of 223 bytes
    let mut sub_blocks = [[0u8; RS_K]; RS_INTERLEAVE];
    for i in 0..VCDU_DATA_SIZE {
        sub_blocks[i % RS_INTERLEAVE][i / RS_INTERLEAVE] = data[i];
    }

    // Encode each sub-block
    let mut parities = [[0u8; RS_2T]; RS_INTERLEAVE];
    for i in 0..RS_INTERLEAVE {
        parities[i] = rs_encode_block(&gf, &genpoly, &sub_blocks[i]);
    }

    // Build output: data + interleaved parity
    let mut output = Vec::with_capacity(CODED_FRAME_SIZE);
    output.extend_from_slice(data);

    // Interleave parity bytes
    for i in 0..RS_2T {
        for j in 0..RS_INTERLEAVE {
            output.push(parities[j][i]);
        }
    }

    assert_eq!(output.len(), CODED_FRAME_SIZE);
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gf_tables() {
        let gf = GfTables::new();
        // α^0 = 1
        assert_eq!(gf.exp[0], 1);
        // α^1 = 2
        assert_eq!(gf.exp[1], 2);
        // α^255 should wrap to α^0 = 1
        assert_eq!(gf.exp[255], gf.exp[0]);
    }

    #[test]
    fn test_gf_mul() {
        let gf = GfTables::new();
        // Multiply by 1 = identity
        assert_eq!(gf.mul(0x42, 1), 0x42);
        // Multiply by 0 = 0
        assert_eq!(gf.mul(0x42, 0), 0);
        // Commutative
        assert_eq!(gf.mul(3, 7), gf.mul(7, 3));
    }

    #[test]
    fn test_encode_frame_length() {
        let data = vec![0u8; VCDU_DATA_SIZE];
        let coded = encode_frame(&data);
        assert_eq!(coded.len(), CODED_FRAME_SIZE);
    }

    #[test]
    fn test_encode_frame_data_preserved() {
        let mut data = vec![0u8; VCDU_DATA_SIZE];
        for (i, b) in data.iter_mut().enumerate() {
            *b = (i & 0xFF) as u8;
        }
        let coded = encode_frame(&data);
        // First 892 bytes should be the original data
        assert_eq!(&coded[..VCDU_DATA_SIZE], &data[..]);
    }

    #[test]
    fn test_zero_data_nonzero_parity() {
        // All-zero data should produce all-zero parity for systematic RS
        let data = vec![0u8; VCDU_DATA_SIZE];
        let coded = encode_frame(&data);
        let parity = &coded[VCDU_DATA_SIZE..];
        assert!(parity.iter().all(|&b| b == 0));
    }
}
