use rumpus::{
    image::RayImage,
    ray::{GlobalFrame, Ray, SensorFrame},
};
use uom::si::{angle::radian, f64::Angle};

/// Shifts the ray_image ignoring any tilt!
pub fn sensor_to_global(ray_image: &RayImage<SensorFrame>) -> RayImage<GlobalFrame> {
    let rays: Vec<_> = ray_image
        .pixels()
        .map(|px| {
            let ray = px.ray().clone()?;
            let shift = shift_by(px.row(), px.col(), ray_image.rows(), ray_image.cols());
            let angle = ray.aop().into_global_frame(-shift);
            Some(Ray::<GlobalFrame>::new(angle, *ray.dop()))
        })
        .collect();

    RayImage::from_rays(rays, ray_image.rows(), ray_image.cols()).unwrap()
}

fn shift_by(row: usize, col: usize, rows: usize, cols: usize) -> Angle {
    let row = row as f64;
    let col = col as f64;
    let rows = rows as f64;
    let cols = cols as f64;

    let y0 = rows / 2.;
    let y = -row + y0;

    let x0 = cols / 2.;
    let x = col - x0;

    Angle::new::<radian>(y.atan2(x))
}
