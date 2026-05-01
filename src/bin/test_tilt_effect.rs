use chrono::{DateTime, Utc};
use indicatif::ProgressIterator;
use rumpus::{
    light::dop::Dop,
    optic::{Camera, PinholeOptic},
    simulation::Simulation,
};
use rumpus_benchmark::{
    grid::OrientationGrid,
    systems::{self, CamXyz, InsEnu},
    utils::weighted_rmse,
};
use sguaba::{
    engineering::{Orientation, Pose},
    math::RigidBodyTransform,
    systems::{Ecef, Wgs84},
};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{meter, micron, millimeter},
    },
};

fn main() {
    let camera = setup_camera_model();

    // Open a new CSV file to store results.
    let csv_path = "tilt_effect_results.csv";
    let mut frame_writer = csv::Writer::from_path(csv_path).unwrap();

    let time = "2025-11-24T15:37:35Z".parse::<DateTime<Utc>>().unwrap();
    let aligned = Orientation::<CamXyz>::aligned();
    let misaligned = Orientation::<CamXyz>::tait_bryan_builder()
        .yaw(Angle::ZERO)
        .pitch(Angle::new::<degree>(-0.25))
        .roll(Angle::new::<degree>(-0.25))
        .build();

    // Construct a grid of orientations about the _aligned_ orientation.
    let grid = OrientationGrid::<InsEnu>::builder()
        .relative_to(Orientation::<InsEnu>::aligned())
        .with_yaw_range(
            Angle::new::<degree>(-180.),
            Angle::new::<degree>(179.),
            Angle::new::<degree>(1.),
        )
        .build();

    for (orientation_index, car_in_enu) in grid.iter().enumerate().progress() {
        let misaligned_cam_in_ecef = camxyz_to_ecef(misaligned, car_in_enu);
        let simulation = Simulation::new(camera, misaligned_cam_in_ecef, time);
        let misaligned_ray_image = simulation.par_ray_image();

        let aligned_cam_in_ecef = camxyz_to_ecef(aligned, car_in_enu);
        let simulation = Simulation::new(camera, aligned_cam_in_ecef, time);
        let aligned_ray_image = simulation.par_ray_image();

        let dop_threshold = Dop::clamped(0.0);
        let weighted_rmse = weighted_rmse(&misaligned_ray_image, &aligned_ray_image, dop_threshold);

        let (car_yaw, car_pitch, car_roll) = car_in_enu.to_tait_bryan_angles();
        let _ = frame_writer.serialize(FrameRecord {
            orientation_index,
            car_roll_deg: car_roll.get::<degree>(),
            car_pitch_deg: car_pitch.get::<degree>(),
            car_yaw_deg: car_yaw.get::<degree>(),
            weighted_rmse,
        });
    }
}

fn camxyz_to_ecef(
    cam_in_camxyz: Orientation<CamXyz>,
    car_in_enu: Orientation<InsEnu>,
) -> Pose<Ecef> {
    let position = Wgs84::builder()
        .latitude(Angle::new::<degree>(44.235019))
        .unwrap()
        .longitude(Angle::new::<degree>(-76.469104))
        .altitude(Length::new::<meter>(45.037201))
        .build();

    let cam_in_car = systems::cam_to_car().transform(cam_in_camxyz);
    let cam_in_enu = systems::car_to_ins(car_in_enu).transform(cam_in_car);
    let enu_to_ecef = unsafe { RigidBodyTransform::ecef_to_enu_at(&position) }.inverse();

    enu_to_ecef.transform(cam_in_enu)
}

fn setup_camera_model() -> Camera<PinholeOptic> {
    const FOCAL_LENGTH_MM: f64 = 8.0;

    // Setup camera model.
    let focal_length = Length::new::<millimeter>(FOCAL_LENGTH_MM);
    let pixel_size = Length::new::<micron>(3.45);

    Camera::new(
        PinholeOptic::from_focal_length(focal_length),
        pixel_size * 2.0,
        1024,
        1224,
    )
}

#[derive(serde::Serialize)]
struct FrameRecord {
    orientation_index: usize,
    car_roll_deg: f64,
    car_pitch_deg: f64,
    car_yaw_deg: f64,
    weighted_rmse: f64,
}
