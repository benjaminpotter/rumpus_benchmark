use chrono::{DateTime, Local, Utc};
use clap::Parser;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use rumpus::{
    CameraEnu, CameraFrd,
    camera::{Camera, Lens},
    image::{ImageSensor, RayImage},
    model::SkyModel,
    ray::{GlobalFrame, Ray},
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz, CarXyz, InsEnu},
    utils::{rays_to_bytes, sensor_to_global, write_image},
};
use sguaba::{
    Bearing, Coordinate, coordinate,
    engineering::{Orientation, Pose},
    system,
    systems::{EquivalentTo, Wgs84},
};
use std::path::{Path, PathBuf};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
};

system!(pub struct CarFrd using FRD);

const FOCAL_LENGTH_MM: f64 = 8.0;

#[derive(Parser)]
struct Cli {
    dataset_path: PathBuf,

    #[arg(short, long)]
    max_frames: Option<usize>,

    #[arg(short, long)]
    write_images: bool,

    #[arg(short, long, default_value_t = 1)]
    step: usize,
}

impl Cli {
    fn image_dir(&self) -> PathBuf {
        self.dataset_path.join("camera_driver_gv_vis_image_raw")
    }

    fn ins_path(&self) -> PathBuf {
        self.dataset_path
            .join("novatel_oem7_inspva/novatel_oem7_inspva.csv")
    }

    fn time_path(&self) -> PathBuf {
        self.dataset_path
            .join("novatel_oem7_time/novatel_oem7_time.csv")
    }
}

fn main() {
    let config = Cli::parse();
    let timestamp = Local::now().to_rfc3339();
    let results_dir = PathBuf::from(&timestamp);
    std::fs::create_dir(&results_dir).unwrap();

    let cam_in_car = systems::cam_to_car().transform(Orientation::<CamXyz>::aligned());
    let ins_path = config.ins_path();
    let ins_reader = InsReader::new();
    let ins_frames = ins_reader.read_csv(&ins_path).unwrap();

    let time_path = config.time_path();
    let time_reader = TimeReader::new();
    let time_frames = time_reader.read_csv(&time_path).unwrap();

    // TODO: Is this the right focal length?
    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let lens = Lens::from_focal_length(focal_length).expect("focal length is greater than zero");
    // TODO: Is this the right pixel size?
    let pixel_size = Length::new::<micron>(3.45);
    let image_reader = ImageReader::new(pixel_size);

    let mut frame_count = 0;
    for (i, (time_frame, ins_frame)) in time_frames.zip(ins_frames).enumerate().step_by(config.step)
    {
        let car_in_ins_enu = ins_frame.orientation;
        let cam_in_ins_enu = systems::car_to_ins(car_in_ins_enu).transform(cam_in_car);
        let sim = make_simulation(
            ins_frame.position,
            cam_in_ins_enu.cast().orientation(),
            time_frame.time,
        );
        let result = sim.simulate();

        let image_path = config.image_dir().join(image_path_from_frame(i));
        let image = image_reader.read_image(image_path).unwrap();

        let cam = Camera::new(lens, cam_in_ins_enu.cast().orientation());
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

        if config.write_images {
            for (prefix, ray_image) in [("simulated", result.ray_image()), ("measured", &ray_image)]
            {
                let (aop_image, dop_image) = rays_to_bytes(ray_image);

                let filename = format!("{prefix}_aop_{i:04}.png");
                let mut path = results_dir.clone();
                path.push(&filename);
                write_image(path, &aop_image, 1224, 1024).unwrap();

                let filename = format!("{prefix}_dop_{i:04}.png");
                let mut path = results_dir.clone();
                path.push(&filename);
                write_image(path, &dop_image, 1224, 1024).unwrap();
            }
        }

        frame_count += 1;
        if let Some(max_frames) = config.max_frames
            && frame_count >= max_frames
        {
            break;
        }
    }
}

fn image_path_from_frame(frame_index: usize) -> impl AsRef<Path> {
    format!("camera_driver_gv_vis_image_raw_{:04}.png", frame_index)
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

fn make_simulation(
    position: Wgs84,
    orientation: Orientation<CameraEnu>,
    time: DateTime<Utc>,
) -> Simulation {
    // TODO: Is this the right pixel_size?
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let image_rows = 1024;
    let image_cols = 1224;
    // Use a small focal length to see more of the sky.
    // TODO: Is this the right focal length?
    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);

    let lens = Lens::from_focal_length(focal_length).expect("focal length is greater than zero");
    let image_sensor = ImageSensor::new(pixel_size, pixel_size, image_rows, image_cols);
    let coords: Vec<Coordinate<CameraFrd>> = (0..image_rows)
        .flat_map(|row| (0..image_cols).map(move |col| (row, col)))
        .map(|(row, col)| image_sensor.at_pixel(row, col).unwrap())
        .collect();

    let sky_model = SkyModel::from_wgs84_and_time(position, time);
    let camera = Camera::new(lens.clone(), orientation);

    Simulation {
        lens,
        image_sensor,
        coords,
        sky_model,
        camera,
    }
}
