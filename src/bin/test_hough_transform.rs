use chrono::Local;
use clap::Parser;
use rumpus::{
    image::RayImage,
    optic::{Camera, PinholeOptic, PixelCoordinate, RayDirection},
    prelude::{Aop, Dop},
    ray::GlobalFrame,
    simulation::Simulation,
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz, InsEnu, up_in_cam},
    utils::{angle_of, sensor_to_global, weighted_rmse},
};
use sguaba::engineering::Orientation;
use std::{
    path::{Path, PathBuf},
    time::Instant,
};
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::{micron, millimeter},
};

const FOCAL_LENGTH_MM: f64 = 8.0;

fn main() {
    let config = Cli::parse();

    // Make a new directory to hold results.
    let timestamp = Local::now().to_rfc3339();
    let results_dir = PathBuf::from(&timestamp);
    std::fs::create_dir(&results_dir).unwrap();

    // Setup reader for INS position and orientation measurements.
    let ins_path = config.ins_path();
    let ins_reader = InsReader::new();
    let ins_frames = ins_reader.read_csv(&ins_path).unwrap();

    // Define orientation of the camera in the car frame.
    let cam_in_car = systems::cam_to_car().transform(Orientation::<CamXyz>::aligned());

    // Setup reader for INS time measurements.
    let time_path = config.time_path();
    let time_reader = TimeReader::new();
    let time_frames = time_reader.read_csv(&time_path).unwrap();

    // Setup reader for polarization images.
    let image_reader = ImageReader::new();

    // Setup camera model.
    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let pixel_size = Length::new::<micron>(3.45);
    let camera = Camera::new(
        PinholeOptic::from_focal_length(focal_length),
        pixel_size * 2.0,
        1024,
        1224,
    );

    // Open a new CSV file to store results.
    let csv_path = results_dir.join("results.csv");
    let mut frame_writer = csv::Writer::from_path(csv_path).unwrap();

    let mut frame_count = 0;
    for (frame_index, (time_frame, ins_frame)) in
        time_frames.zip(ins_frames).enumerate().step_by(config.step)
    {
        print_frame_status(frame_index, frame_count, config.max_frames, None);

        let t0 = Instant::now();

        // Read the polarization image from this frame.
        let image_path = config.image_dir().join(image_path_from_frame(frame_index));
        let image = match image_reader.read_image(image_path) {
            Ok(image) => image,
            Err(e) => {
                eprintln!("failed to read image: {e}");
                continue;
            }
        };

        let car_in_ins_enu = ins_frame.orientation;
        let cam_in_ins_enu = systems::car_to_ins(car_in_ins_enu).transform(cam_in_car);
        let cam_in_ecef = systems::ins_to_ecef(&ins_frame.position).transform(cam_in_ins_enu);

        let up = up_in_cam(car_in_ins_enu).normalized();
        let azimuth = up.y().atan2(up.x());
        // HACK: I do not know why the trait bounds for ...z().acos(); are violated...
        let polar = Angle::new::<radian>(up.z().value.acos());
        let ray_direction = RayDirection::from_angles(polar, azimuth);
        let Some(up_pixel) = camera.trace_from_bearing(ray_direction) else {
            println!("global zenith is outside of camera fov! skipping...");
            continue;
        };

        let measured = sensor_to_global(&image, &up_pixel);
        let accum = hough_transform(&measured);
        let estimated_yaw = accum.max();

        let csv_path = results_dir.join(format!("frame_{frame_index:04}_results.csv"));
        accum.to_csv(csv_path).unwrap();

        // Write results from this frame to the CSV file.
        let elapsed_ms = t0.elapsed().as_millis();
        let (car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let _ = frame_writer.serialize(FrameRecord {
            frame_index,
            elapsed_ms,
            car_yaw_deg: car_yaw.get::<degree>(),
            car_pitch_deg: car_pitch.get::<degree>(),
            car_roll_deg: car_roll.get::<degree>(),
            estimated_yaw_deg: estimated_yaw.get::<degree>(),
        });

        print_frame_status(
            frame_index,
            frame_count,
            config.max_frames,
            Some(elapsed_ms),
        );

        frame_count += 1;
        if let Some(max_frames) = config.max_frames
            && frame_count >= max_frames
        {
            break;
        }
    }
}

struct Accumulator {
    votes: Vec<u64>,
}

impl Accumulator {
    /// Create a new accumulator with
    fn new(size: usize) -> Self {
        let votes = vec![0; size];
        Self { votes }
    }

    fn angle_to_index(angle: Angle, resolution: Angle) -> usize {
        todo!()
    }

    fn index_to_angle(index: usize, resolution: Angle) -> Angle {
        todo!()
    }

    fn vote(&mut self, angle: Angle) {
        todo!()
    }

    fn max(&self) -> Angle {
        todo!()
    }

    fn to_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let mut writer = csv::Writer::from_path(path)?;

        Ok(())
    }
}

fn hough_transform(ray_image: &RayImage<GlobalFrame>) -> Accumulator {
    let aop_target = Aop::from_angle_wrapped(Angle::new::<degree>(90.));
    let aop_threshold = Angle::new::<degree>(0.1);
    let dop_threshold = Dop::clamped(0.2);
    let angle_resolution = Angle::new::<degree>(0.1);
    let origin = PixelCoordinate::new(512, 612);

    let mut acc = Accumulator::new(angle_resolution);

    for px in ray_image.pixels() {
        let Some(ray) = px.ray() else {
            continue;
        };

        if ray.dop() < dop_threshold {
            continue;
        }

        if !ray.aop().in_thres(aop_target, aop_threshold) {
            continue;
        }

        let coord = PixelCoordinate::new(px.row(), px.col());
        let angle = angle_of(coord, &origin);

        // add to accumulator
        acc.vote(angle);
    }

    acc
}

fn image_path_from_frame(frame_index: usize) -> impl AsRef<Path> {
    format!("camera_driver_gv_vis_image_raw_{:04}.png", frame_index)
}

fn print_frame_status(
    frame_index: usize,
    frame_count: usize,
    max_frames: Option<usize>,
    elapsed_millis: Option<u128>,
) {
    let max_frames_fmt = match max_frames {
        Some(max_frames) => format!("{max_frames:04}"),
        None => "????".to_string(),
    };

    let elapsed_millis_fmt = match elapsed_millis {
        Some(elapsed_millis) => format!("in {elapsed_millis:05} ms"),
        None => "".to_string(),
    };

    let frame_number = frame_count + 1;
    println!("[{frame_number:04}/{max_frames_fmt}] frame {frame_index:04} {elapsed_millis_fmt}");
}

#[derive(Parser)]
struct Cli {
    dataset_path: PathBuf,

    #[arg(short, long)]
    max_frames: Option<usize>,

    #[arg(short, long)]
    write_images: bool,

    #[arg(short, long, default_value_t = 1)]
    step: usize,

    #[arg(short, long, default_value_t = 0.1)]
    resolution_deg: f64,
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

    fn resolution(&self) -> Angle {
        Angle::new::<degree>(self.resolution_deg)
    }
}

#[derive(serde::Serialize)]
struct FrameRecord {
    frame_index: usize,
    elapsed_ms: u128,
    car_pitch_deg: f64,
    car_roll_deg: f64,
    car_yaw_deg: f64,
    estimated_yaw_deg: f64,
}

#[derive(serde::Serialize)]
struct AccumulatorRecord {
    angle_deg: f64,
    votes: u64,
}
