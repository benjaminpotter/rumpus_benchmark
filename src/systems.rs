use rumpus::CameraEnu;
use sguaba::{
    Vector,
    engineering::{Orientation, Pose},
    math::RigidBodyTransform,
    system,
    systems::{EquivalentTo, Wgs84},
};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::meter,
    },
};

// The body frame of the camera.
// Defined in terms of the CarXyz frame.
system!(pub struct CamXyz using right-handed XYZ);

// The body frame of the car.
// Defined in terms of the InsEnu frame.
system!(pub struct CarXyz using right-handed XYZ);

// The earth bounded frame provided by the INS.
system!(pub struct InsEnu using ENU);

// The InsEnu frame has the same axes orientation as the CameraEnu frame.
// Their origin will be slightly different, but we ignore that here.
unsafe impl EquivalentTo<CameraEnu> for InsEnu {}

impl InsEnu {
    pub fn orientation_from_inspva(azimuth: f64, pitch: f64, roll: f64) -> Orientation<Self> {
        // azimuth is left-handed from north in the INSPVA spec.
        // convert it to right-handed by negating.
        let yaw = Angle::new::<degree>(-azimuth);
        let pitch = Angle::new::<degree>(pitch);
        let roll = Angle::new::<degree>(roll);

        // requires right-handed
        Orientation::tait_bryan_builder()
            .yaw(yaw)
            .pitch(pitch)
            .roll(roll)
            .build()
    }

    pub fn position_from_inspva(lat: f64, lon: f64, height: f64) -> Wgs84 {
        let latitude = Angle::new::<degree>(lat);
        let longitude = Angle::new::<degree>(lon);
        let altitude = Length::new::<meter>(height);

        Wgs84::builder()
            .latitude(latitude)
            .expect("latitude out of bounds")
            .longitude(longitude)
            .altitude(altitude)
            .build()
    }
}

pub fn cam_to_car() -> RigidBodyTransform<CamXyz, CarXyz> {
    let cam_aligned_to_car = Orientation::<CamXyz>::tait_bryan_builder()
        // TODO: I am not certain this is right, I think there may be more problems in the
        // simulation code.
        .yaw(Angle::HALF_TURN / 2.0)
        .pitch(Angle::ZERO)
        .roll(Angle::ZERO)
        .build();

    let translation = Vector::<CamXyz>::zero();

    // Encodes a rotation from the camera frame into the car frame.
    // The cam_in_car is where the camera body axes are aligned with the car body axes.
    // SAFETY: There is a positional offset between CamXyz and CarXyz, but we ignore it.
    let rotation = unsafe { cam_aligned_to_car.map_as_zero_in::<CarXyz>() };

    unsafe { RigidBodyTransform::new(translation, rotation) }
}

pub fn car_to_ins(car_in_ins: Orientation<InsEnu>) -> RigidBodyTransform<CarXyz, InsEnu> {
    let translation = Vector::<CarXyz>::zero();

    // Given the orientation of the car in the INS frame called car_in_ins, we map the axes of
    // CarXyz to the orientation of car_in_ins.
    // Then, we can go from CarXyz to InsEnu using the inverse of this rotation.
    // SAFETY: We assume that the origin of CarXyz is coincident with InsEnu's origin.
    let rotation = unsafe { car_in_ins.map_as_zero_in::<CarXyz>() }.inverse();

    unsafe { RigidBodyTransform::new(translation, rotation) }
}
