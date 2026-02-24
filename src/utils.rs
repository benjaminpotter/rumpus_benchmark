use rumpus::{
    image::RayImage,
    optic::PixelCoordinate,
    ray::{GlobalFrame, Ray, SensorFrame},
};
use uom::si::{angle::radian, f64::Angle};

/// Shifts the ray_image ignoring any tilt!
pub fn sensor_to_global(
    ray_image: &RayImage<SensorFrame>,
    origin: &PixelCoordinate,
) -> RayImage<GlobalFrame> {
    let rays: Vec<_> = ray_image
        .pixels()
        .map(|px| {
            let ray = px.ray()?;

            let px_coord = PixelCoordinate::new(px.row(), px.col());

            let shift = shift_by(px_coord, origin);
            let angle = ray.aop().into_global_frame(-shift);
            Some(Ray::<GlobalFrame>::new(angle, ray.dop()))
        })
        .collect();

    RayImage::from_rays(rays, ray_image.rows(), ray_image.cols()).unwrap()
}

#[allow(clippy::cast_precision_loss)]
fn shift_by(coord: PixelCoordinate, origin: &PixelCoordinate) -> Angle {
    let y0 = origin.row() as f64;
    let x0 = origin.col() as f64;

    let y1 = coord.row() as f64;
    let x1 = coord.col() as f64;

    let y = -y1 + y0;
    let x = x1 - x0;

    Angle::new::<radian>(y.atan2(x))
}
