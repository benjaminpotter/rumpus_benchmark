use std::{error::Error, path::Path};

use rumpus::{
    CameraFrd,
    image::RayImage,
    ray::{GlobalFrame, RayFrame, SensorFrame},
};
use sguaba::Coordinate;
use uom::si::angle::degree;

pub fn sensor_to_global(
    ray_image: &RayImage<SensorFrame>,
    zenith_coord: &Coordinate<CameraFrd>,
) -> RayImage<GlobalFrame> {
    let pixels: Vec<_> = ray_image
        .ray_pixels()
        .filter_map(|ray| ray.as_ref())
        .cloned()
        .map(|ray| ray.into_global_frame(zenith_coord.clone()))
        .collect();

    RayImage::from_pixels(pixels)
}

pub fn rays_to_bytes<F: RayFrame>(ray_image: &RayImage<F>) -> (Vec<u8>, Vec<u8>) {
    // Map the AoP values in the RayImage to RGB colours.
    // Draw missing pixels as white.
    let aop_image: Vec<u8> = ray_image
        .ray_pixels()
        .flat_map(|pixel| match pixel {
            Some(ray) => to_rgb(ray.aop().angle().get::<degree>(), -90.0, 90.0)
                .expect("aop in between -90 and 90"),
            None => [255, 255, 255],
        })
        .collect();

    // Map the DoP values in the RayImage to RGB colours.
    // Draw missing pixels as white.
    let dop_image: Vec<u8> = ray_image
        .ray_pixels()
        .flat_map(|pixel| match pixel {
            Some(ray) => to_rgb(ray.dop().into_inner(), 0.0, 1.0).expect("dop in between 0 and 1"),
            None => [255, 255, 255],
        })
        .collect();

    (aop_image, dop_image)
}

pub fn write_image<P: AsRef<Path>>(
    path: P,
    bytes: &Vec<u8>,
    cols: u32,
    rows: u32,
) -> Result<(), Box<dyn Error + 'static>> {
    image::save_buffer(&path, &bytes, cols, rows, image::ExtendedColorType::Rgb8)?;
    Ok(())
}

// Map an f64 on the interval [x_min, x_max] to an RGB color.
fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
    if x < x_min || x > x_max {
        return None;
    }

    let interval_width = x_max - x_min;
    let x_norm = ((x - x_min) / interval_width * 255.).floor() as u8;

    let r = vec![
        255,
        x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(224)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let g = vec![
        255,
        x_norm
            .checked_sub(32)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(160)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let b = vec![
        255,
        x_norm
            .checked_add(127)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    Some([r, g, b])
}
