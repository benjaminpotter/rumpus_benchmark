use chrono::{DateTime, Local, Utc};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use rumpus::{
    CameraEnu, CameraFrd,
    camera::{Camera, Lens},
    image::{ImageSensor, IntensityImage, RayImage},
    iter::RayIterator,
    model::SkyModel,
    ray::{GlobalFrame, Ray, RayFrame, SensorFrame},
};
use sguaba::{Bearing, Coordinate, engineering::Orientation, systems::Wgs84};
use std::{
    error::Error,
    path::{Path, PathBuf},
};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
};

const FOCAL_LENGTH_MM: f64 = 8.0;

struct Config {
    write_images: bool,
}

fn main() {
    let config = Config { write_images: true };

    let sim = make_simulation();
    let result = sim.simulate();

    // Define required parameters.
    let input_path = "../rumpus/testing/intensity.png";
    // TODO: Is this the right pixel size?
    let pixel_size = Length::new::<micron>(3.45);
    let image_reader = ImageReader { pixel_size };

    // TODO: Convert to global frame.
    let image = image_reader.read_image(input_path).unwrap();
    // TODO: Is this the right focal length?
    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let lens = Lens::from_focal_length(focal_length).expect("focal length is greater than zero");
    let orientation = Orientation::<CameraEnu>::tait_bryan_builder()
        .yaw(Angle::new::<degree>(0.0))
        .pitch(Angle::new::<degree>(0.0))
        .roll(Angle::new::<degree>(0.0))
        .build();
    let cam = Camera::new(lens, orientation);
    let zenith_coord = cam
        .trace_from_sky(
            Bearing::<CameraEnu>::builder()
                .azimuth(Angle::ZERO)
                .elevation(Angle::HALF_TURN / 2.)
                .expect("elevation is on range -90 to 90")
                .build(),
        )
        .expect("zenith is always above the horizon");
    let ray_image = sensor_to_global(&image, &zenith_coord);

    // Print the ground truth position
    // Highlight if its tilted
    // For aop and dop:
    // Print the rms error
    // Print the standard error
    // Could do rms error when low dop are excluded
    // Create new directory with results
    // Name directory using timestamp
    // Write out all four images

    let timestamp = Local::now().to_rfc3339();
    let results_dir = PathBuf::from(&timestamp);
    std::fs::create_dir(&results_dir).unwrap();

    for (prefix, ray_image) in [("simulated", result.ray_image()), ("measured", &ray_image)] {
        let (aop_image, dop_image) = rays_to_bytes(ray_image);

        let filename = format!("{}_aop.png", prefix);
        let mut path = results_dir.clone();
        path.push(&filename);
        write_image(path, &aop_image, 1224, 1024).unwrap();

        let filename = format!("{}_dop.png", prefix);
        let mut path = results_dir.clone();
        path.push(&filename);
        write_image(path, &dop_image, 1224, 1024).unwrap();
    }
}

struct Simulation {
    lens: Lens,
    image_sensor: ImageSensor,
    coords: Vec<Coordinate<CameraFrd>>,
    sky_model: SkyModel,
    camera: Camera,
}

impl Simulation {
    fn simulate(&self) -> SimulationResult {
        let rays: Vec<Ray<_>> = self
            .coords
            .par_iter()
            .filter_map(|coord| {
                // TODO: Save the runtime for each pixel.
                let bearing_cam_enu = self
                    .camera
                    .trace_from_sensor(*coord)
                    .expect("coord on sensor plane");
                let aop = self.sky_model.aop(bearing_cam_enu)?;
                let dop = self.sky_model.dop(bearing_cam_enu)?;

                Some(Ray::new(*coord, aop, dop))
            })
            .collect();

        let ray_image = RayImage::from_rays_with_sensor(rays, &self.image_sensor)
            .expect("no ray hits the same pixel");

        SimulationResult { ray_image }
    }
}

struct SimulationResult {
    ray_image: RayImage<GlobalFrame>,
}

impl SimulationResult {
    fn ray_image(&self) -> &RayImage<GlobalFrame> {
        &self.ray_image
    }
}

struct ImageReader {
    pixel_size: Length,
}

impl ImageReader {
    fn read_image<P: AsRef<Path>>(
        &self,
        path: P,
    ) -> Result<RayImage<SensorFrame>, Box<dyn Error + 'static>> {
        // Open a new image and ensure it is in single channel greyscale format.
        let raw_image = image::ImageReader::open(&path)?.decode()?.into_luma8();

        // Create a new IntensityImage from the input image.
        let (width, height) = raw_image.dimensions();
        let intensity_image =
            IntensityImage::from_bytes(width as u16, height as u16, &raw_image.into_raw())
                .expect("image dimensions are even");

        // Filter the rays from the intensity image by DoP.
        // Convert the sparse RayIterator into a dense RayImage using the specs of
        // the image sensor as a RaySensor.
        let ray_image: RayImage<SensorFrame> = intensity_image
            .rays(self.pixel_size, self.pixel_size)
            .ray_image(&ImageSensor::new(
                self.pixel_size,
                self.pixel_size,
                intensity_image.height(),
                intensity_image.width(),
            ))
            .expect("no ray hits the same pixel");

        Ok(ray_image)
    }
}

fn make_simulation() -> Simulation {
    // TODO: Is this the right pixel_size?
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let image_rows = 1024;
    let image_cols = 1224;
    // Use a small focal length to see more of the sky.
    // TODO: Is this the right focal length?
    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let latitude = Angle::new::<degree>(44.2187);
    let longitude = Angle::new::<degree>(-76.4747);
    let time = "2025-06-13T16:26:47+00:00";
    let orientation = Orientation::<CameraEnu>::tait_bryan_builder()
        .yaw(Angle::new::<degree>(0.0))
        .pitch(Angle::new::<degree>(0.0))
        .roll(Angle::new::<degree>(0.0))
        .build();

    let lens = Lens::from_focal_length(focal_length).expect("focal length is greater than zero");
    let image_sensor = ImageSensor::new(pixel_size, pixel_size, image_rows, image_cols);
    let coords: Vec<Coordinate<CameraFrd>> = (0..image_rows)
        .flat_map(|row| (0..image_cols).map(move |col| (row, col)))
        .map(|(row, col)| image_sensor.at_pixel(row, col).unwrap())
        .collect();

    let sky_model = SkyModel::from_wgs84_and_time(
        Wgs84::builder()
            .latitude(latitude)
            .expect("latitude is between -90 and 90")
            .longitude(longitude)
            .altitude(Length::ZERO)
            .build(),
        time.parse::<DateTime<Utc>>()
            .expect("valid datetime string"),
    );

    let camera = Camera::new(lens.clone(), orientation);

    Simulation {
        lens,
        image_sensor,
        coords,
        sky_model,
        camera,
    }
}

fn sensor_to_global(
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

fn write_image<P: AsRef<Path>>(
    path: P,
    bytes: &Vec<u8>,
    cols: u32,
    rows: u32,
) -> Result<(), Box<dyn Error + 'static>> {
    image::save_buffer(&path, &bytes, cols, rows, image::ExtendedColorType::Rgb8)?;
    Ok(())
}

fn rays_to_bytes<F: RayFrame>(ray_image: &RayImage<F>) -> (Vec<u8>, Vec<u8>) {
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

// Map an f64 on the interval [x_min, x_max] to an RGB color.
pub fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
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
