use rumpus::{
    image::RayImage,
    optic::PixelCoordinate,
    prelude::{Aop, Dop},
    ray::{GlobalFrame, Ray, SensorFrame},
};
use uom::si::{
    angle::{degree, radian},
    f64::Angle,
};

pub fn weighted_rmse<F: Copy>(
    simulated: &RayImage<F>,
    measured: &RayImage<F>,
    dop_threshold: Dop,
) -> f64 {
    let mut sum_weighted_errors = 0.0f64;
    let mut sum_weights = 0.0f64;
    let mut samples = 0.;

    for rpx in measured.pixels() {
        if let Some(measured_ray) = rpx.ray()
            && let Some(simulated_ray) = simulated.ray(rpx.row(), rpx.col())
        {
            let weight = measured_ray.dop();
            if weight < dop_threshold {
                continue;
            }

            let error = Angle::from(measured_ray.aop() - simulated_ray.aop())
                .get::<degree>()
                .powf(2.);

            sum_weights += weight;
            sum_weighted_errors += weight * error;
            samples += 1.;
        }
    }

    (sum_weighted_errors / sum_weights / samples).sqrt()
}

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

            let shift = angle_of(px_coord, origin);
            let angle = ray.aop().into_global_frame(-shift);
            Some(Ray::<GlobalFrame>::new(angle, ray.dop()))
        })
        .collect();

    RayImage::from_rays(rays, ray_image.rows(), ray_image.cols()).unwrap()
}

#[allow(clippy::cast_precision_loss)]
pub fn angle_of(coord: PixelCoordinate, origin: &PixelCoordinate) -> Angle {
    let y0 = origin.row() as f64;
    let x0 = origin.col() as f64;

    let y1 = coord.row() as f64;
    let x1 = coord.col() as f64;

    let y = -y1 + y0;
    let x = x1 - x0;

    Angle::new::<radian>(y.atan2(x))
}

pub fn binary_threshold<F: Copy>(ray_image: &RayImage<F>) -> RayImage<F> {
    let aop_target = Aop::from_angle_wrapped(Angle::new::<degree>(90.));
    let aop_threshold = Angle::new::<degree>(0.1);
    let dop_threshold = Dop::clamped(0.2);

    let rays = ray_image.rays().map(|ray_opt| {
        ray_opt.copied().filter(|&ray| {
            ray.dop() >= dop_threshold && ray.aop().in_thres(aop_target, aop_threshold)
        })
    });

    RayImage::from_rays(rays, ray_image.rows(), ray_image.cols()).unwrap()
}
