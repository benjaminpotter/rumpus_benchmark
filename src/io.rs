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
use sguaba::{engineering::Orientation, system};
use uom::si::{
    angle::degree,
    f64::{Angle, Length},
};

system!(pub struct InsEnu using ENU);

pub struct DatasetReader {
    ins_headers: csv::StringRecord,
    ins_reader: csv::Reader<File>,
}

impl DatasetReader {
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn Error + 'static>> {
        let ins_topic_path = "";
        let mut ins_reader = csv::Reader::from_path(&ins_topic_path)?;
        let ins_headers = ins_reader.headers()?.clone();
        Ok(DatasetReader {
            ins_headers,
            ins_reader,
        })
    }

    pub fn read_frame<'a>(&'a mut self) -> Option<Result<Frame<'a>, Box<dyn Error + 'static>>> {
        let mut valid = false;

        let mut ins_record = csv::StringRecord::new();
        valid = match self.ins_reader.read_record(&mut ins_record) {
            // Is true if we just read a valid record.
            Ok(valid) => valid,
            Err(err) => return Some(Err(err.into())),
        };

        if !valid {
            return None;
        }

        let ins_record = Record::from_strings(&self.ins_headers, &ins_record);
        let frame = Frame::new(ins_record);
        Some(Ok(frame))
    }
}

pub struct Record<'a> {
    inner: HashMap<&'a str, String>,
}

impl<'a> Record<'a> {
    fn from_strings(headers: &'a csv::StringRecord, record: &csv::StringRecord) -> Record<'a> {
        assert_eq!(headers.len(), record.len());

        let mut inner = HashMap::new();
        for (header, data) in headers.iter().zip(record.into_iter()) {
            let old = inner.insert(header, data.to_string());
            assert_eq!(old, None);
        }

        Self { inner }
    }
}

impl<'a> Deref for Record<'a> {
    type Target = HashMap<&'a str, String>;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<'a> DerefMut for Record<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

pub struct Frame<'a> {
    ins_record: Record<'a>,
}

impl<'a> Frame<'a> {
    fn new(ins_record: Record<'a>) -> Self {
        Self { ins_record }
    }

    pub fn ins_orientation(&self) -> Orientation<InsEnu> {
        Orientation::<InsEnu>::tait_bryan_builder()
            .yaw(Angle::new::<degree>(0.0))
            .pitch(Angle::new::<degree>(0.0))
            .roll(Angle::new::<degree>(0.0))
            .build()
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
