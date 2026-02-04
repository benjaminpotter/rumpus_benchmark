use chrono::Local;
use clap::Parser;
use rumpus::{
    estimate::pattern_match::Matcher,
    image::{Jet, RayImage},
    optic::{Camera, PinholeOptic},
    simulation::Simulation,
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz},
    utils::sensor_to_global,
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

#[derive(serde::Serialize)]
struct Record {
    frame_index: usize,
    car_pitch_deg: f64,
    car_roll_deg: f64,
}

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

    let matcher = Matcher::new();

    // Open a new CSV file to store results.
    let csv_path = results_dir.join("results.csv");
    let mut writer = csv::Writer::from_path(csv_path).unwrap();

    let mut frame_count = 0;
    for (i, (time_frame, ins_frame)) in time_frames.zip(ins_frames).enumerate().step_by(config.step)
    {
        let t0 = Instant::now();

        // Figure out the orientation of the camera in the ECEF frame.
        let car_in_ins_enu = ins_frame.orientation;
        let cam_in_ins_enu = systems::car_to_ins(car_in_ins_enu).transform(cam_in_car);
        let cam_in_ecef = systems::ins_to_ecef(&ins_frame.position).transform(cam_in_ins_enu);

        // Read the polarization image from this frame.
        let image_path = config.image_dir().join(image_path_from_frame(i));
        let image = image_reader.read_image(image_path).unwrap();

        let estimated_orientation =
            matcher.orientation_of(image, cam_in_ecef.position(), time_frame.time);

        // Simulate an image for this timestep.
        // let simulation = Simulation::new(camera.clone(), cam_in_ecef, time_frame.time);
        // let simulated = simulation.par_ray_image();

        // Write results from this frame to the CSV file.
        let (_car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let _ = writer.serialize(Record {
            frame_index: i,
            car_pitch_deg: car_pitch.get::<degree>(),
            car_roll_deg: car_roll.get::<degree>(),
        });

        match config.max_frames {
            Some(max_frames) => println!(
                "[{:04}/{:04}] frame {:04} in {:05} ms",
                frame_count + 1,
                max_frames,
                i,
                t0.elapsed().as_millis()
            ),
            None => println!(
                "[{:04}/????] in {:05} ms",
                frame_count + 1,
                t0.elapsed().as_millis()
            ),
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
