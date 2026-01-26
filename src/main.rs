use chrono::Local;
use clap::Parser;
use rumpus::{
    image::Jet,
    optic::{Camera, PinholeOptic},
    simulation::Simulation,
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz},
    utils::sensor_to_global,
};
use sguaba::{Vector, builder::vector, engineering::Orientation};
use std::path::{Path, PathBuf};
use uom::si::{
    angle::degree,
    f64::Length,
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

    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let pixel_size = Length::new::<micron>(3.45);
    let image_reader = ImageReader::new(pixel_size);
    let camera = Camera::new(
        PinholeOptic::from_focal_length(focal_length),
        pixel_size * 2.0,
        1024,
        1224,
    );

    let csv_path = results_dir.join("results.csv");
    let mut writer = csv::Writer::from_path(csv_path).unwrap();

    let mut frame_count = 0;
    for (i, (time_frame, ins_frame)) in time_frames.zip(ins_frames).enumerate().step_by(config.step)
    {
        let car_in_ins_enu = ins_frame.orientation;
        let cam_in_ins_enu = systems::car_to_ins(car_in_ins_enu).transform(cam_in_car);
        let cam_in_ecef = systems::ins_to_ecef(&ins_frame.position).transform(cam_in_ins_enu);
        let simulation = Simulation::new(camera.clone(), cam_in_ecef, time_frame.time);
        let sim_image = simulation.ray_image();

        let image_path = config.image_dir().join(image_path_from_frame(i));
        let image = image_reader.read_image(image_path).unwrap();
        let ray_image = sensor_to_global(&image);

        let (_car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let _ = writer.serialize(Record {
            frame_index: frame_count,
            car_pitch_deg: car_pitch.get::<degree>(),
            car_roll_deg: car_roll.get::<degree>(),
        });

        if config.write_images {
            for (prefix, ray_image) in [("simulated", &sim_image), ("measured", &ray_image)] {
                let filename = format!("{prefix}_aop_{i:04}.png");
                let path = results_dir.join(&filename);
                let _ = image::save_buffer(
                    path,
                    &ray_image.aop_bytes(&Jet),
                    1224,
                    1024,
                    image::ExtendedColorType::Rgb8,
                );

                let filename = format!("{prefix}_dop_{i:04}.png");
                let path = results_dir.join(&filename);
                let _ = image::save_buffer(
                    path,
                    &ray_image.dop_bytes(&Jet),
                    1224,
                    1024,
                    image::ExtendedColorType::Rgb8,
                );
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
