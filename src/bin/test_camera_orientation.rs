use chrono::Local;
use clap::Parser;
use rumpus::{
    optic::{Camera, PinholeOptic, RayDirection}, prelude::Dop, simulation::Simulation
};
use rumpus_benchmark::{
    grid::OrientationGrid, io::{ImageReader, InsReader, TimeReader}, systems::{self, CamXyz, InsEnu, up_in_cam}, utils::{sensor_to_global, weighted_rmse}
};
use sguaba::engineering::Orientation;
use std::{
    ops::Range, path::{Path, PathBuf}, time::Instant
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

    // Setup reader for INS time measurements.
    let time_path = config.time_path();
    let time_reader = TimeReader::new();
    let time_frames = time_reader.read_csv(&time_path).unwrap();

    // Setup reader for polarization images.
    let dop_threshold = config.dop_threshold();
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

    // Construct a grid of orientations about the ideal _aligned_ orientation.
    let grid = OrientationGrid::<CamXyz>::builder()
        .relative_to(Orientation::<CamXyz>::aligned())
        .with_roll_range(Angle::new::<degree>(-2.5), Angle::new::<degree>(2.5), Angle::new::<degree>(0.25))
        .with_pitch_range(Angle::new::<degree>(-2.5), Angle::new::<degree>(2.5), Angle::new::<degree>(0.25))
        .build();

    let mut frame_count = 0;
    let ranges = config.parse_ranges();
    for (frame_index, (time_frame, ins_frame)) in
        time_frames.zip(ins_frames).enumerate().step_by(config.step)
    {
        let mut include = false;
        for range in &ranges {
            include = include || range.contains(&frame_index);
        }

        if !include {
            continue;
        }

        if config.verbose {
            print_frame_status(frame_index, frame_count, config.max_frames, None);
        }

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


        let interval_size = 5.;
        let car_in_ins_enu = ins_frame.orientation;
        let (car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let iters = config.iters_at_resolution(interval_size);

        for (orientation_index, cam_in_camxyz) in grid.iter().enumerate() {
            let (cam_yaw, cam_pitch, cam_roll) = cam_in_camxyz.to_tait_bryan_angles();
            // Define orientation of the camera in the car frame.
            let cam_in_car = systems::cam_to_car().transform(cam_in_camxyz);
            let mut yaw_offset = -Angle::new::<degree>(interval_size / 2.);

            for candidate_index in 0..iters {
                let t1 = Instant::now();

                // Figure out the orientation of the camera in the ECEF frame.
                let car_in_ins_enu: Orientation<InsEnu> = Orientation::tait_bryan_builder()
                    .yaw(car_yaw + yaw_offset)
                    .pitch(car_pitch)
                    .roll(car_roll)
                    .build();

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
                let simulation = Simulation::new(camera, cam_in_ecef, time_frame.time);
                let simulated = simulation.par_ray_image();
                let weighted_rmse = weighted_rmse(&simulated, &measured, dop_threshold);

                let elapsed_ms = t1.elapsed().as_millis();
                let _ = frame_writer.serialize(FrameRecord {
                    frame_index,
                    datetime_utc: time_frame.time.to_rfc3339(),
                    car_roll_deg: car_roll.get::<degree>(),
                    car_pitch_deg: car_pitch.get::<degree>(),
                    car_yaw_deg: car_yaw.get::<degree>(),
                    orientation_index,
                    cam_roll_deg: cam_roll.get::<degree>(),
                    cam_pitch_deg: cam_pitch.get::<degree>(),
                    cam_yaw_deg: cam_yaw.get::<degree>(),
                    candidate_index,
                    elapsed_ms,
                    yaw_offset_deg: yaw_offset.get::<degree>(),
                    weighted_rmse,
                });

                if config.verbose {
                    match config.max_frames {
                        Some(max_frames) => println!(
                            "[{:04}/{:04}] frame {:04}: [{:04}/{:04}] candidate in {:05} ms",
                            frame_count + 1,
                            max_frames,
                            frame_index,
                            candidate_index + 1,
                            iters,
                            elapsed_ms
                        ),
                        None => println!(
                            "[{:04}/????] frame {:04}: [{:04}/{:04}] candidate in {:05} ms",
                            frame_count + 1,
                            frame_index,
                            candidate_index + 1,
                            iters,
                            elapsed_ms
                        ),
                    }
                }

                yaw_offset += config.resolution();
            }
        }

        if config.verbose {
            print_frame_status(
                frame_index,
                frame_count,
                config.max_frames,
                Some(t0.elapsed().as_millis()),
            );
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

    #[arg(long, num_args = 1.., value_delimiter = ' ')]
    ranges: Vec<usize>,

    #[arg(short, long, default_value_t = 0.1)]
    resolution_deg: f64,

    #[arg(long, default_value_t = 0.0)]
    dop_threshold: f64,

    #[arg(short, long)]
    verbose: bool,
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

    fn iters_at_resolution(&self, interval_size: f64) -> usize {
        (interval_size / self.resolution_deg) as usize
    }

    fn resolution(&self) -> Angle {
        Angle::new::<degree>(self.resolution_deg)
    }

    fn dop_threshold(&self) -> Dop {
        Dop::clamped(self.dop_threshold)
    }

    fn parse_ranges(&self) -> Vec<Range<usize>> {
        let mut result = Vec::new();

        if (self.ranges.len() & 1) != 0 {
            println!("warn: passed unbounded range");
        }

        for pair in self.ranges.chunks(2) {
            result.push(pair[0]..pair[1]);
        }

        return result;
    }
}

#[derive(serde::Serialize)]
struct FrameRecord {
    frame_index: usize,
    datetime_utc: String,
    car_roll_deg: f64,
    car_pitch_deg: f64,
    car_yaw_deg: f64,
    orientation_index: usize,
    cam_roll_deg: f64,
    cam_pitch_deg: f64,
    cam_yaw_deg: f64,
    candidate_index: usize,
    elapsed_ms: u128,
    yaw_offset_deg: f64,
    weighted_rmse: f64,
}


