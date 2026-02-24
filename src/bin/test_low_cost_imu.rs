use chrono::Local;
use clap::Parser;
use rumpus::{
    image::{Jet, RayImage},
    optic::{Camera, PinholeOptic, RayDirection},
    simulation::Simulation,
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz, up_in_cam},
    utils::{sensor_to_global, weighted_rmse},
};
use sguaba::engineering::Orientation;
use std::{
    path::{Path, PathBuf},
    time::Instant,
};

#[allow(clippy::similar_names)]
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

    let csv_path = results_dir.join("results.csv");
    let mut writer = csv::Writer::from_path(csv_path).unwrap();

    let mut frame_count = 0;
    for (i, (time_frame, ins_frame)) in time_frames.zip(ins_frames).enumerate().step_by(config.step)
    {
        let t0 = Instant::now();

        let car_in_ins_enu = ins_frame.orientation;
        let cam_in_ins_enu = systems::car_to_ins(car_in_ins_enu).transform(cam_in_car);
        let cam_in_ecef = systems::ins_to_ecef(&ins_frame.position).transform(cam_in_ins_enu);
    }
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
