use crate::systems::InsEnu;
use chrono::{DateTime, TimeZone, Utc};
use rumpus::{
    image::{IntensityImage, RayImage},
    ray::SensorFrame,
};
use sguaba::{engineering::Orientation, systems::Wgs84};
use std::{error::Error, path::Path};
use uom::si::f64::Length;

pub struct TimeReader;
pub struct TimeFrame {
    pub time: DateTime<Utc>,
}

impl TimeReader {
    pub fn new() -> Self {
        Self
    }

    pub fn read_csv<P: AsRef<Path>>(
        &self,
        path: P,
    ) -> Result<Box<dyn Iterator<Item = TimeFrame>>, Box<dyn Error + 'static>> {
        let mut reader = csv::Reader::from_path(path)?;
        let mut frames = Vec::new();
        for result in reader.records() {
            let record = result?;

            let start_idx = 17;
            let year: i32 = record.get(start_idx + 0).unwrap().parse()?;
            assert_eq!(year, 2025);
            let month: u32 = record.get(start_idx + 1).unwrap().parse()?;
            let day: u32 = record.get(start_idx + 2).unwrap().parse()?;
            let hour: u32 = record.get(start_idx + 3).unwrap().parse()?;
            let min: u32 = record.get(start_idx + 4).unwrap().parse()?;
            let msec: u32 = record.get(start_idx + 5).unwrap().parse()?;
            let sec = msec / 1000;

            let time = Utc
                .with_ymd_and_hms(year, month, day, hour, min, sec)
                .unwrap();
            frames.push(TimeFrame { time });
        }

        Ok(Box::new(frames.into_iter()))
    }
}

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

            let lat = record.get(13).unwrap().parse()?;
            let lon = record.get(14).unwrap().parse()?;
            let height = record.get(15).unwrap().parse()?;
            let position = InsEnu::position_from_inspva(lat, lon, height);

            let roll = record.get(19).unwrap().parse()?;
            let pitch = record.get(20).unwrap().parse()?;
            let azimuth = record.get(21).unwrap().parse()?;
            let orientation = InsEnu::orientation_from_inspva(azimuth, pitch, roll);

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
            IntensityImage::from_bytes(width as usize, height as usize, &raw_image.into_raw())
                .expect("image dimensions are even");

        Ok(RayImage::from_rays(
            intensity_image.rays().map(|ray| Some(ray)),
            intensity_image.height(),
            intensity_image.width(),
        )?)
    }
}
