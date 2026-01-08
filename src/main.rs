use chrono::{DateTime, Local, Utc};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use rumpus::{
    CameraEnu, CameraFrd,
    camera::{Camera, Lens},
    image::{ImageSensor, RayImage},
    model::SkyModel,
    ray::{GlobalFrame, Ray},
};
use rumpus_benchmark::{
    io::{DatasetReader, ImageReader},
    utils::{rays_to_bytes, sensor_to_global, write_image},
};
use sguaba::{Bearing, Coordinate, engineering::Orientation, systems::Wgs84};
use std::path::PathBuf;
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
    max_frames: Option<usize>,
    write_images: bool,
}

fn main() {
    let config = Config {
        write_images: true,
        max_frames: Some(1),
    };

    let timestamp = Local::now().to_rfc3339();
    let results_dir = PathBuf::from(&timestamp);
    std::fs::create_dir(&results_dir).unwrap();

    let dataset_path = "/home/ben-work/git/secondary/polcam_dataset/2025-11-24/rmc";
    let mut reader = DatasetReader::from_path(&dataset_path).unwrap();

    let mut frame_count = 0;
    while let Some(result) = reader.read_frame() {
        let Ok(frame) = result else {
            eprintln!("failed to read frame");
            break;
        };

        let sim = make_simulation();
        let result = sim.simulate();

        // Define required parameters.
        let input_path = "../rumpus/testing/intensity.png";
        // TODO: Is this the right pixel size?
        let pixel_size = Length::new::<micron>(3.45);
        let image_reader = ImageReader::new(pixel_size);

        // TODO: Convert to global frame.
        let image = image_reader.read_image(input_path).unwrap();
        // TODO: Is this the right focal length?
        let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
        let lens =
            Lens::from_focal_length(focal_length).expect("focal length is greater than zero");
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

        if config.write_images {
            for (prefix, ray_image) in [("simulated", result.ray_image()), ("measured", &ray_image)]
            {
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

        if let Some(max_frames) = config.max_frames
            && frame_count >= max_frames
        {
            break;
        }

        frame_count += 1;
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
