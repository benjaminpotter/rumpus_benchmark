use chrono::Local;
use clap::Parser;
use rumpus::{
    optic::{Camera, PinholeOptic, RayDirection},
    simulation::Simulation,
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz, InsEnu, up_in_cam},
    utils::{sensor_to_global, weighted_rmse},
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
        let image = image_reader.read_image(image_path).unwrap();

        let csv_path = results_dir.join(format!("frame_{frame_index:04}_results.csv"));
        let mut candidate_writer = csv::Writer::from_path(csv_path).unwrap();

        let interval_size = 10.;
        let car_in_ins_enu = ins_frame.orientation;
        let (car_yaw, pitch, roll) = car_in_ins_enu.to_tait_bryan_angles();
        let mut yaw_offset = -Angle::new::<degree>(interval_size / 2.);

        let iters = config.iters_at_resolution(interval_size);
        for candidate_index in 0..iters {
            let t1 = Instant::now();

            // Figure out the orientation of the camera in the ECEF frame.
            let car_in_ins_enu: Orientation<InsEnu> = Orientation::tait_bryan_builder()
                .yaw(car_yaw + yaw_offset)
                .pitch(pitch)
                .roll(roll)
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
            let weighted_rmse = weighted_rmse(&simulated, &measured);

            let _ = candidate_writer.serialize(CandidateRecord {
                frame_index,
                car_yaw_deg: car_yaw.get::<degree>(),
                yaw_offset_deg: yaw_offset.get::<degree>(),
                weighted_rmse,
            });

            match config.max_frames {
                Some(max_frames) => println!(
                    "[{:04}/{:04}] frame {:04}: [{:04}/{:04}] candidate in {:05} ms",
                    frame_count + 1,
                    max_frames,
                    frame_index,
                    candidate_index + 1,
                    iters,
                    t1.elapsed().as_millis(),
                ),
                None => println!(
                    "[{:04}/????] frame {:04}: [{:04}/{:04}] candidate in {:05} ms",
                    frame_count + 1,
                    frame_index,
                    candidate_index + 1,
                    iters,
                    t1.elapsed().as_millis(),
                ),
            }

            yaw_offset += config.resolution();
        }

        // Write results from this frame to the CSV file.
        let (_car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let _ = frame_writer.serialize(FrameRecord {
            frame_index,
            car_yaw_deg: car_yaw.get::<degree>(),
            car_pitch_deg: car_pitch.get::<degree>(),
            car_roll_deg: car_roll.get::<degree>(),
        });

        print_frame_status(
            frame_index,
            frame_count,
            config.max_frames,
            Some(t0.elapsed().as_millis()),
        );

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

    fn iters_at_resolution(&self, interval_size: f64) -> usize {
        (interval_size / self.resolution_deg) as usize
    }

    fn resolution(&self) -> Angle {
        Angle::new::<degree>(self.resolution_deg)
    }
}

#[derive(serde::Serialize)]
struct FrameRecord {
    frame_index: usize,
    car_pitch_deg: f64,
    car_roll_deg: f64,
    car_yaw_deg: f64,
}

#[derive(serde::Serialize)]
struct CandidateRecord {
    frame_index: usize,
    car_yaw_deg: f64,
    weighted_rmse: f64,
    yaw_offset_deg: f64,
}
