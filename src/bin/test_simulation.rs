use chrono::Local;
use clap::Parser;
use rumpus::{
    image::{Gray, Jet, RayImage, RayMap},
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
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::{micron, millimeter},
};

const FOCAL_LENGTH_MM: f64 = 8.0;

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

    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let pixel_size = Length::new::<micron>(3.45);
    let image_reader = ImageReader::new();
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
        let t0 = Instant::now();

        let car_in_ins_enu = ins_frame.orientation;
        let cam_in_ins_enu = systems::car_to_ins(car_in_ins_enu).transform(cam_in_car);
        let cam_in_ecef = systems::ins_to_ecef(&ins_frame.position).transform(cam_in_ins_enu);
        let simulation = Simulation::new(camera, cam_in_ecef, time_frame.time);
        let simulated = simulation.par_ray_image();

        let up = up_in_cam(car_in_ins_enu).normalized();
        let azimuth = up.y().atan2(up.x());
        // HACK: I do not know why the trait bounds for ...z().acos(); are violated...
        let polar = Angle::new::<radian>(up.z().value.acos());
        let ray_direction = RayDirection::from_angles(polar, azimuth);
        let Some(up_pixel) = camera.trace_from_bearing(ray_direction) else {
            println!("global zenith is outside of camera fov! skipping...");
            continue;
        };

        let image_path = config.image_dir().join(image_path_from_frame(i));
        let image = image_reader.read_image(image_path).unwrap();
        let measured = sensor_to_global(&image, &up_pixel);

        let weighted_rmse = weighted_rmse(&simulated, &measured);

        let (_car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let _ = writer.serialize(Record {
            frame_index: i,
            origin_row: up_pixel.row(),
            origin_col: up_pixel.col(),
            car_pitch_deg: car_pitch.get::<degree>(),
            car_roll_deg: car_roll.get::<degree>(),
            weighted_rmse,
        });

        if config.write_images {
            // Get measured dop as a byte.
            let bytes = measured.dop_bytes(&Gray);

            for (prefix, ray_image) in [("simulated", &simulated), ("measured", &measured)] {
                let filename = format!("{prefix}_aop_{i:04}.png");
                let path = results_dir.join(&filename);
                let aop_bytes = ray_image.aop_bytes(&Jet);
                let _ = image::save_buffer(
                    path,
                    &aop_bytes,
                    1224,
                    1024,
                    image::ExtendedColorType::Rgb8,
                );

                // Interleave alpha with RGB bytes.
                let mut aop_with_alpha = Vec::with_capacity(bytes.len() * 4);
                for (rgb, &a) in aop_bytes.chunks_exact(3).zip(&bytes) {
                    aop_with_alpha.extend_from_slice(rgb);
                    aop_with_alpha.push(a);
                }
                let filename = format!("{prefix}_aop_rgba_{i:04}.png");
                let path = results_dir.join(&filename);
                let _ = image::save_buffer(
                    path,
                    &aop_with_alpha,
                    1224,
                    1024,
                    image::ExtendedColorType::Rgba8,
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
    origin_row: usize,
    origin_col: usize,
    car_pitch_deg: f64,
    car_roll_deg: f64,
    weighted_rmse: f64,
}
