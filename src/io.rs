use std::{
    collections::HashMap,
    error::Error,
    fs::File,
    ops::{Deref, DerefMut},
    path::Path,
};

use csv::Reader;
use rumpus::{
    image::{ImageSensor, IntensityImage, RayImage},
    iter::RayIterator,
    ray::SensorFrame,
};
use sguaba::{engineering::Orientation, system, systems::Wgs84};
use uom::si::{
    angle::degree,
    f64::{Angle, Length},
    length::meter,
};

system!(pub struct InsEnu using ENU);

pub struct InsReader;
pub struct InsFrame {
    pub position: Wgs84,
    pub orientation: Orientation<InsEnu>,
}

impl InsReader {
    pub fn new() -> Self {
        Self
    }

    pub fn read_csv<P: AsRef<Path>>(
        &self,
        path: P,
    ) -> Result<Box<dyn Iterator<Item = InsFrame>>, Box<dyn Error + 'static>> {
        let mut reader = csv::Reader::from_path(path)?;
        let mut frames = Vec::new();
        for result in reader.records() {
            let record = result?;

            let latitude = Angle::new::<degree>(record.get(13).unwrap().parse()?);
            let longitude = Angle::new::<degree>(record.get(14).unwrap().parse()?);
            let height = Length::new::<meter>(record.get(15).unwrap().parse()?);

            let roll = Angle::new::<degree>(record.get(19).unwrap().parse()?);
            let pitch = Angle::new::<degree>(record.get(20).unwrap().parse()?);
            let azimuth = Angle::new::<degree>(record.get(21).unwrap().parse()?);

            let position = Wgs84::builder()
                .latitude(latitude)
                .unwrap()
                .longitude(longitude)
                .altitude(height)
                .build();

            let orientation = Orientation::<InsEnu>::tait_bryan_builder()
                .yaw(azimuth)
                .pitch(pitch)
                .roll(roll)
                .build();

            frames.push(InsFrame {
                position,
                orientation,
            });
        }

        Ok(Box::new(frames.into_iter()))
    }
}

pub struct ImageReader {
    pixel_size: Length,
}
pub struct ImageFrame {}

impl ImageReader {
    pub fn new(pixel_size: Length) -> Self {
        Self { pixel_size }
    }

    pub fn read_image<P: AsRef<Path>>(
        &self,
        path: P,
    ) -> Result<RayImage<SensorFrame>, Box<dyn Error + 'static>> {
        // Open a new image and ensure it is in single channel greyscale format.
        let raw_image = image::ImageReader::open(&path)?.decode()?.into_luma8();

        // Create a new IntensityImage from the input image.
        let (width, height) = raw_image.dimensions();
        let intensity_image =
            IntensityImage::from_bytes(width as u16, height as u16, &raw_image.into_raw())
                .expect("image dimensions are even");

        // Filter the rays from the intensity image by DoP.
        // Convert the sparse RayIterator into a dense RayImage using the specs of
        // the image sensor as a RaySensor.
        let ray_image: RayImage<SensorFrame> = intensity_image
            .rays(self.pixel_size, self.pixel_size)
            .ray_image(&ImageSensor::new(
                self.pixel_size,
                self.pixel_size,
                intensity_image.height(),
                intensity_image.width(),
            ))
            .expect("no ray hits the same pixel");

        Ok(ray_image)
    }
}
