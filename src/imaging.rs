/// Image loading and MCU block extraction for LRPT encoding
///
/// Loads images, resizes to LRPT width (1568 pixels), converts to grayscale,
/// and extracts 8x8 MCU blocks for JPEG compression.
use crate::constants::{IMAGE_WIDTH, MCU_SIZE, MCUS_PER_ROW};
use image::{GenericImageView, Luma};
use std::path::Path;

/// Load an image, resize to LRPT width, and convert to grayscale rows
pub fn load_image(path: &Path) -> Result<Vec<Vec<u8>>, Box<dyn std::error::Error>> {
    eprintln!("Loading image: {:?}", path);

    let img = image::open(path)?;
    let (orig_width, orig_height) = img.dimensions();

    // Resize to LRPT width, maintain aspect ratio
    let new_height = (orig_height as f32 * IMAGE_WIDTH as f32 / orig_width as f32) as u32;
    // Round up to multiple of MCU_SIZE
    let new_height = ((new_height + MCU_SIZE as u32 - 1) / MCU_SIZE as u32) * MCU_SIZE as u32;

    eprintln!(
        "Resizing from {}x{} to {}x{}",
        orig_width, orig_height, IMAGE_WIDTH, new_height
    );

    let resized = img.resize_exact(
        IMAGE_WIDTH as u32,
        new_height,
        image::imageops::FilterType::Lanczos3,
    );

    let grayscale = resized.to_luma8();

    let mut rows = Vec::with_capacity(new_height as usize);
    for y in 0..new_height {
        let mut row = Vec::with_capacity(IMAGE_WIDTH);
        for x in 0..IMAGE_WIDTH as u32 {
            let pixel: Luma<u8> = *grayscale.get_pixel(x, y);
            row.push(pixel.0[0]);
        }
        rows.push(row);
    }

    eprintln!("Processed {} lines of image data", rows.len());
    Ok(rows)
}

/// Load an image and resize to a specific height (for matching channel heights)
pub fn load_image_to_height(
    path: &Path,
    target_height: u32,
) -> Result<Vec<Vec<u8>>, Box<dyn std::error::Error>> {
    eprintln!("Loading image: {:?}", path);

    let img = image::open(path)?;
    let (orig_width, orig_height) = img.dimensions();

    eprintln!(
        "Resizing from {}x{} to {}x{} (matching height)",
        orig_width, orig_height, IMAGE_WIDTH, target_height
    );

    let resized = img.resize_exact(
        IMAGE_WIDTH as u32,
        target_height,
        image::imageops::FilterType::Lanczos3,
    );

    let grayscale = resized.to_luma8();

    let mut rows = Vec::with_capacity(target_height as usize);
    for y in 0..target_height {
        let mut row = Vec::with_capacity(IMAGE_WIDTH);
        for x in 0..IMAGE_WIDTH as u32 {
            let pixel: Luma<u8> = *grayscale.get_pixel(x, y);
            row.push(pixel.0[0]);
        }
        rows.push(row);
    }

    Ok(rows)
}

/// Extract 8x8 MCU blocks from image rows for one MCU row
/// Returns 196 MCU blocks (each 8x8 pixels)
pub fn extract_mcu_row(rows: &[Vec<u8>], mcu_row_index: usize) -> Vec<[[u8; MCU_SIZE]; MCU_SIZE]> {
    let start_y = mcu_row_index * MCU_SIZE;
    let mut mcus = Vec::with_capacity(MCUS_PER_ROW);

    for mcu_col in 0..MCUS_PER_ROW {
        let start_x = mcu_col * MCU_SIZE;
        let mut block = [[0u8; MCU_SIZE]; MCU_SIZE];

        for dy in 0..MCU_SIZE {
            let y = start_y + dy;
            for dx in 0..MCU_SIZE {
                let x = start_x + dx;
                if y < rows.len() && x < rows[y].len() {
                    block[dy][dx] = rows[y][x];
                }
                // Out-of-bounds pixels stay as 0 (black padding)
            }
        }

        mcus.push(block);
    }

    mcus
}

/// Split a color image into 3 grayscale channels (R, G, B)
pub fn load_image_rgb(
    path: &Path,
) -> Result<(Vec<Vec<u8>>, Vec<Vec<u8>>, Vec<Vec<u8>>), Box<dyn std::error::Error>> {
    eprintln!("Loading color image: {:?}", path);

    let img = image::open(path)?;
    let (orig_width, orig_height) = img.dimensions();

    let new_height = (orig_height as f32 * IMAGE_WIDTH as f32 / orig_width as f32) as u32;
    let new_height = ((new_height + MCU_SIZE as u32 - 1) / MCU_SIZE as u32) * MCU_SIZE as u32;

    let resized = img.resize_exact(
        IMAGE_WIDTH as u32,
        new_height,
        image::imageops::FilterType::Lanczos3,
    );

    let rgb = resized.to_rgb8();

    let mut r_rows = Vec::with_capacity(new_height as usize);
    let mut g_rows = Vec::with_capacity(new_height as usize);
    let mut b_rows = Vec::with_capacity(new_height as usize);

    for y in 0..new_height {
        let mut r_row = Vec::with_capacity(IMAGE_WIDTH);
        let mut g_row = Vec::with_capacity(IMAGE_WIDTH);
        let mut b_row = Vec::with_capacity(IMAGE_WIDTH);
        for x in 0..IMAGE_WIDTH as u32 {
            let pixel = rgb.get_pixel(x, y);
            r_row.push(pixel[0]);
            g_row.push(pixel[1]);
            b_row.push(pixel[2]);
        }
        r_rows.push(r_row);
        g_rows.push(g_row);
        b_rows.push(b_row);
    }

    eprintln!("Processed {} lines of RGB data (3 channels)", r_rows.len());
    Ok((r_rows, g_rows, b_rows))
}

/// Calculate number of MCU rows for a given image height
pub fn mcu_row_count(image_height: usize) -> usize {
    (image_height + MCU_SIZE - 1) / MCU_SIZE
}

/// Load a camera frame and convert to grayscale rows
#[cfg(feature = "input-v4l")]
pub fn camera_frame_to_rows(frame: &image::RgbImage) -> Vec<Vec<u8>> {
    let (width, height) = frame.dimensions();

    // Resize to LRPT width
    let new_height = (height as f32 * IMAGE_WIDTH as f32 / width as f32) as u32;
    let new_height = ((new_height + MCU_SIZE as u32 - 1) / MCU_SIZE as u32) * MCU_SIZE as u32;

    let resized = image::imageops::resize(
        frame,
        IMAGE_WIDTH as u32,
        new_height,
        image::imageops::FilterType::Lanczos3,
    );

    let grayscale = image::DynamicImage::ImageRgb8(
        image::RgbImage::from_raw(IMAGE_WIDTH as u32, new_height, resized.into_raw()).unwrap(),
    )
    .to_luma8();

    let mut rows = Vec::with_capacity(new_height as usize);
    for y in 0..new_height {
        let mut row = Vec::with_capacity(IMAGE_WIDTH);
        for x in 0..IMAGE_WIDTH as u32 {
            row.push(grayscale.get_pixel(x, y).0[0]);
        }
        rows.push(row);
    }

    rows
}
