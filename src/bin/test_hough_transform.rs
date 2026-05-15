use clap::Parser;
use rumpus::{
    image::{Jet, RayImage},
    optic::{Camera, PinholeOptic, PixelCoordinate, RayDirection},
    ray::GlobalFrame,
};
use rumpus_benchmark::{
    io::{ImageReader, InsReader, TimeReader},
    systems::{self, CamXyz, InsEnu, up_in_cam},
    utils::{angle_of, binary_threshold, sensor_to_global},
};
use sguaba::{Bearing, bearing, engineering::Orientation, vector};
use std::{
    path::{Path, PathBuf},
    time::Instant,
};
use uom::{
    ConstZero,
    si::{
        angle::{degree, radian},
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
};

const FOCAL_LENGTH_MM: f64 = 8.0;

fn main() {
    let config = Cli::parse();

    // Make a new directory to hold results.
    let results_dir = PathBuf::from("results");
    std::fs::create_dir_all(&results_dir).unwrap();

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
        let binary_ray_image = binary_threshold(&measured);
        let accum = hough_transform(&binary_ray_image);
        let estimated_solar_azimuth = accum.max();
        let solar_bearing_camxyz = Bearing::<CamXyz>::builder()
            .azimuth(estimated_solar_azimuth)
            .elevation(Angle::ZERO)
            .expect("elevation is in [-90°, 90°]")
            .build();

        let solar_bearing_carxyz = systems::cam_to_car().transform(solar_bearing_camxyz);
        let solar_bearing_insenu =
            systems::car_to_ins(car_in_ins_enu).transform(solar_bearing_carxyz);

        let accumulator_dump = results_dir.join(format!("accumulator_{frame_index:04}.csv"));
        let _ = accum.to_csv(&accumulator_dump);

        let car_lat = ins_frame.position.latitude().get::<degree>();
        let car_lon = ins_frame.position.longitude().get::<degree>();

        let solar_position =
            spa::solar_position::<spa::StdFloatOps>(time_frame.time, car_lat, car_lon).unwrap();

        // Write results from this frame to the CSV file.
        let elapsed_ms = t0.elapsed().as_millis();
        let (car_yaw, car_pitch, car_roll) = car_in_ins_enu.to_tait_bryan_angles();
        let _ = frame_writer.serialize(FrameRecord {
            frame_index,
            elapsed_ms,
            utc_time: time_frame.time.to_rfc3339(),
            car_lat,
            car_lon,
            car_yaw_deg: car_yaw.get::<degree>(),
            car_pitch_deg: car_pitch.get::<degree>(),
            car_roll_deg: car_roll.get::<degree>(),
            // clockwise from north
            solar_azimuth_deg: solar_position.azimuth,
            solar_zenith_deg: solar_position.zenith_angle,
            estimated_solar_azimuth_camxyz_deg: solar_bearing_camxyz.azimuth().get::<degree>(),
            estimated_solar_azimuth_carxyz_deg: solar_bearing_carxyz.azimuth().get::<degree>(),
            estimated_solar_azimuth_insenu_deg: solar_bearing_insenu.azimuth().get::<degree>(),
            accumulator_dump,
        });

        if config.write_images {
            write_images(&results_dir, frame_index, measured, binary_ray_image);
        }

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
    /// Create a new accumulator with `size` slots.
    fn new(size: usize) -> Self {
        let votes = vec![0; size];
        Self { votes }
    }

    fn slot_angle(&self) -> Angle {
        Angle::new::<degree>(180.) / self.votes.len() as f64
    }

    /// Convert an `angle` to a slot index.
    fn angle_to_index(&self, angle: Angle) -> usize {
        let normalized = angle.get::<degree>().rem_euclid(180.0);
        let index = (normalized / self.slot_angle().get::<degree>()) as usize;
        index.min(self.votes.len() - 1)
    }

    /// Convert a slot `index` to an angle.
    fn index_to_angle(&self, index: usize) -> Angle {
        self.slot_angle() * index as f64
    }

    /// Increment the slot at `angle` by one.
    fn vote(&mut self, angle: Angle) {
        let index = self.angle_to_index(angle);
        self.votes[index] += 1;
    }

    fn votes(self) -> Vec<u64> {
        self.votes
    }

    fn angles(&self) -> Vec<Angle> {
        (0..self.votes.len())
            .map(|i| self.index_to_angle(i))
            .collect()
    }

    /// Apply a mean filter kernel with `width` to `self.votes`.
    fn mean_filter(&mut self, width: usize) {
        if width <= 1 || self.votes.is_empty() {
            return;
        }

        let n = self.votes.len();
        let mut filtered_votes = vec![0; n];
        let half_width = (width / 2) as i64;

        for i in 0..n {
            let mut sum: u64 = 0;
            let mut count: u64 = 0;

            for dw in -half_width..=half_width {
                // Use modulo arithmetic to handle circular wrapping (0° wrap to 180°)
                let idx = ((i as i64 + dw).rem_euclid(n as i64)) as usize;
                sum += self.votes[idx];
                count += 1;
            }

            filtered_votes[i] = sum / count;
        }

        self.votes = filtered_votes;
    }

    /// Return the slot with the most votes.
    fn max(&self) -> Angle {
        let best_index = self
            .votes
            .iter()
            .enumerate()
            .max_by_key(|(_, v)| *v)
            .map_or(0, |(i, _)| i);
        self.index_to_angle(best_index)
    }

    /// Print a CSV with (slot angle, vote count) pairs.
    fn to_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let mut writer = csv::Writer::from_path(path)?;
        for (index, &votes) in self.votes.iter().enumerate() {
            writer.serialize(AccumulatorRecord {
                angle_deg: self.index_to_angle(index).get::<degree>(),
                votes,
            })?;
        }
        writer.flush()?;
        Ok(())
    }
}

#[derive(serde::Serialize)]
struct AccumulatorRecord {
    angle_deg: f64,
    votes: u64,
}

fn hough_transform(ray_image: &RayImage<GlobalFrame>) -> Accumulator {
    let slot_count = 1800;
    let origin = PixelCoordinate::new(512, 612);

    let mut acc = Accumulator::new(slot_count);

    for px in ray_image.pixels() {
        if px.ray().is_none() {
            continue;
        }

        let coord = PixelCoordinate::new(px.row(), px.col());
        let angle = angle_of(coord, &origin);

        // add to accumulator
        acc.vote(angle);
    }

    acc.mean_filter(8);
    acc
}

fn image_path_from_frame(frame_index: usize) -> impl AsRef<Path> {
    format!("camera_driver_gv_vis_image_raw_{:04}.png", frame_index)
}

fn write_images<P, F>(
    results_dir: P,
    frame_index: usize,
    measured: RayImage<F>,
    binary_ray_image: RayImage<F>,
) where
    F: Copy,
    P: AsRef<Path>,
{
    let filename = format!("aop_{frame_index:04}.png");
    let path = results_dir.as_ref().join(&filename);
    let aop_bytes = measured.aop_bytes(&Jet);
    let _ = image::save_buffer(path, &aop_bytes, 1224, 1024, image::ExtendedColorType::Rgb8);

    let filename = format!("binary_aop_{frame_index:04}.png");
    let path = results_dir.as_ref().join(&filename);
    let binary_aop_bytes = binary_ray_image.aop_bytes(&Jet);
    let _ = image::save_buffer(
        path,
        &binary_aop_bytes,
        1224,
        1024,
        image::ExtendedColorType::Rgb8,
    );
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
    utc_time: String,
    car_lat: f64,
    car_lon: f64,
    car_pitch_deg: f64,
    car_roll_deg: f64,
    car_yaw_deg: f64,
    accumulator_dump: PathBuf,
    solar_azimuth_deg: f64,
    solar_zenith_deg: f64,
    estimated_solar_azimuth_insenu_deg: f64,
    estimated_solar_azimuth_camxyz_deg: f64,
    estimated_solar_azimuth_carxyz_deg: f64,
}
