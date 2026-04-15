// orientation grid search
//
// want to have an iterator over Orientation
// grid has a resolution for each euler angle (yaw, pitch, roll)
// grid has bounds for each euler angle
// want to be able to fix certain euler angles
// for example, say the yaw should be fixed, but provide intervals for the pitch and roll
//
// provide a base Orientation
// return orientations in a grid from the base orientation
// called relative_to or something
// there could also be an _absolute_ method that assumes relative to an orientation of zero
// (aligned)
//
// Example of how to create an Orientation:
// let car_in_ins_enu: Orientation<InsEnu> = Orientation::tait_bryan_builder()
//     .yaw(car_yaw + yaw_offset)
//     .pitch(car_pitch)
//     .roll(car_roll)
//     .build();

use sguaba::engineering::Orientation;
use uom::si::{angle::radian, f64::Angle};

/// Defines the range and step size for a single Euler angle axis.
#[derive(Clone, Copy, Debug)]
struct AxisRange {
    min: Angle,
    max: Angle,
    step: Angle,
}

impl AxisRange {
    fn new(min: Angle, max: Angle, step: Angle) -> Self {
        Self { min, max, step }
    }

    /// Returns an iterator over the angle values in this range (inclusive of min,
    /// exclusive of max if it doesn't land exactly on a step).
    fn values(self) -> impl Iterator<Item = Angle> {
        let min_rad = self.min.get::<radian>();
        let max_rad = self.max.get::<radian>();
        let step_rad = self.step.get::<radian>();

        // Guard against zero / negative step to avoid infinite loops.
        assert!(step_rad > 0.0, "AxisRange step must be positive");

        let count = ((max_rad - min_rad) / step_rad).floor() as usize + 1;

        (0..count).map(move |i| Angle::new::<radian>(min_rad + i as f64 * step_rad))
    }
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Builder for [`OrientationGrid`].
pub struct OrientationGridBuilder<In> {
    base: Orientation<In>,
    yaw_range: Option<AxisRange>,
    pitch_range: Option<AxisRange>,
    roll_range: Option<AxisRange>,
}

impl<In: Copy> OrientationGridBuilder<In> {
    fn new(base: Orientation<In>) -> Self {
        Self {
            base,
            yaw_range: None,
            pitch_range: None,
            roll_range: None,
        }
    }

    /// Set the base orientation that fixed axes are taken from (default:
    /// `Orientation::aligned()`).
    pub fn relative_to(mut self, base: Orientation<In>) -> Self {
        self.base = base;
        self
    }

    /// Vary the yaw axis from `min` to `max` in steps of `step`.
    pub fn with_yaw_range(mut self, min: Angle, max: Angle, step: Angle) -> Self {
        self.yaw_range = Some(AxisRange::new(min, max, step));
        self
    }

    /// Vary the pitch axis from `min` to `max` in steps of `step`.
    pub fn with_pitch_range(mut self, min: Angle, max: Angle, step: Angle) -> Self {
        self.pitch_range = Some(AxisRange::new(min, max, step));
        self
    }

    /// Vary the roll axis from `min` to `max` in steps of `step`.
    pub fn with_roll_range(mut self, min: Angle, max: Angle, step: Angle) -> Self {
        self.roll_range = Some(AxisRange::new(min, max, step));
        self
    }

    /// Consume the builder and produce an [`OrientationGrid`].
    pub fn build(self) -> OrientationGrid<In> {
        OrientationGrid {
            base: self.base,
            yaw_range: self.yaw_range,
            pitch_range: self.pitch_range,
            roll_range: self.roll_range,
            _phan: std::marker::PhantomData,
        }
    }
}

// ---------------------------------------------------------------------------
// OrientationGrid
// ---------------------------------------------------------------------------

/// An iterator-source over a 3-D Euler-angle grid of [`Orientation`] values.
///
/// Each axis (yaw, pitch, roll) can either be fixed to the value of the
/// `relative_to` base orientation, or swept across a specified range.  The
/// `iter()` method yields the full Cartesian product of all active axes.
///
/// # Example
///
/// ```rust,ignore
/// use sguaba::engineering::Orientation;
/// use uom::si::{angle::degree, f64::Angle};
///
/// let grid = OrientationGrid::builder()
///     .relative_to(Orientation::aligned())
///     .with_pitch_range(
///         Angle::new::<degree>(-5.),
///         Angle::new::<degree>(5.),
///         Angle::new::<degree>(1.),
///     )
///     .build();
///
/// for orientation in grid.iter() {
///     // process orientation …
/// }
/// ```
pub struct OrientationGrid<In> {
    base: Orientation<In>,
    yaw_range: Option<AxisRange>,
    pitch_range: Option<AxisRange>,
    roll_range: Option<AxisRange>,
    _phan: std::marker::PhantomData<In>,
}

impl<In: Copy> OrientationGrid<In> {
    /// Create a new builder, defaulting the base orientation to `Orientation::aligned()`.
    pub fn builder() -> OrientationGridBuilder<In> {
        OrientationGridBuilder::new(Orientation::aligned())
    }

    /// Extract the fixed (base) yaw/pitch/roll angles from the base orientation.
    fn base_euler(&self) -> (Angle, Angle, Angle) {
        self.base.to_tait_bryan_angles()
    }

    /// Iterate over every [`Orientation`] in the grid.
    ///
    /// Axes that were given a range are swept; axes that were not are held
    /// fixed at the corresponding value of the `relative_to` orientation.
    pub fn iter(&self) -> OrientationGridIter<'_, In> {
        let (base_yaw, base_pitch, base_roll) = self.base_euler();

        // Build a flat Vec of (yaw, pitch, roll) triples — the Cartesian product
        // of whichever axes are active.
        let yaws: Vec<Angle> = match self.yaw_range {
            Some(r) => r.values().collect(),
            None => vec![base_yaw],
        };
        let pitches: Vec<Angle> = match self.pitch_range {
            Some(r) => r.values().collect(),
            None => vec![base_pitch],
        };
        let rolls: Vec<Angle> = match self.roll_range {
            Some(r) => r.values().collect(),
            None => vec![base_roll],
        };

        let mut orientations: Vec<Orientation<In>> = Vec::with_capacity(yaws.len() * pitches.len() * rolls.len());
        for &y in &yaws {
            for &p in &pitches {
                for &r in &rolls {
                    orientations.push(
                        Orientation::tait_bryan_builder()
                            .yaw(y)
                            .pitch(p)
                            .roll(r)
                            .build(),
                    );
                }
            }
        }

        OrientationGridIter {
            orientations,
            index: 0,
            _phan: std::marker::PhantomData,
        }
    }
}

// ---------------------------------------------------------------------------
// Iterator
// ---------------------------------------------------------------------------

/// Iterator produced by [`OrientationGrid::iter`].
pub struct OrientationGridIter<'a, In> {
    orientations: Vec<Orientation<In>>,
    index: usize,
    _phan: std::marker::PhantomData<&'a In>,
}

impl<'a, In: Copy> Iterator for OrientationGridIter<'a, In> {
    type Item = Orientation<In>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.orientations.len() {
            let o = self.orientations[self.index];
            self.index += 1;
            Some(o)
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.orientations.len() - self.index;
        (remaining, Some(remaining))
    }
}

impl<'a, In: Copy> ExactSizeIterator for OrientationGridIter<'a, In> {}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use sguaba::{engineering::Orientation, system};
    use uom::si::angle::degree;

    system!(struct DummySystem using right-handed XYZ);

    #[test]
    fn orientation_grid() {
        let grid = OrientationGrid::<DummySystem>::builder()
            // this would be the default so not strictly necessary
            .relative_to(Orientation::aligned())
            // this specifies the bounds and resolution of the grid
            .with_pitch_range(
                Angle::new::<degree>(-5.),
                Angle::new::<degree>(5.),
                Angle::new::<degree>(1.),
            )
            // roll and yaw should always use the value from the relative_to orientation
            .build();

        for orientation in grid.iter() {
            // do something with the orientations…
            let _ = orientation;
        }
    }

    #[test]
    fn pitch_only_yields_correct_count() {
        // -5 to +5 in steps of 1 => 11 values
        let grid = OrientationGrid::<DummySystem>::builder()
            .with_pitch_range(
                Angle::new::<degree>(-5.),
                Angle::new::<degree>(5.),
                Angle::new::<degree>(1.),
            )
            .build();

        assert_eq!(grid.iter().count(), 11);
    }

    #[test]
    fn cartesian_product_of_two_axes() {
        // 3 yaw × 5 pitch = 15 orientations
        let grid = OrientationGrid::<DummySystem>::builder()
            .with_yaw_range(
                Angle::new::<degree>(0.),
                Angle::new::<degree>(20.),
                Angle::new::<degree>(10.),
            )
            .with_pitch_range(
                Angle::new::<degree>(-2.),
                Angle::new::<degree>(2.),
                Angle::new::<degree>(1.),
            )
            .build();

        assert_eq!(grid.iter().count(), 3 * 5);
    }

    #[test]
    fn fixed_axes_come_from_base_orientation() {
        use uom::si::angle::radian;

        let base_yaw = Angle::new::<degree>(30.);
        let base_pitch = Angle::new::<degree>(10.);
        let base_roll = Angle::new::<degree>(-5.);

        let base: Orientation<DummySystem> = Orientation::tait_bryan_builder()
            .yaw(base_yaw)
            .pitch(base_pitch)
            .roll(base_roll)
            .build();

        // Only sweep roll; yaw and pitch should stay fixed at the base values.
        let grid = OrientationGrid::builder()
            .relative_to(base)
            .with_roll_range(
                Angle::new::<degree>(-1.),
                Angle::new::<degree>(1.),
                Angle::new::<degree>(1.),
            )
            .build();

        for o in grid.iter() {
            let (yaw, pitch, _roll) = o.to_tait_bryan_angles();
            let tol = 1e-9_f64;
            assert!(
                (yaw.get::<radian>() - base_yaw.get::<radian>()).abs() < tol,
                "yaw should be fixed"
            );
            assert!(
                (pitch.get::<radian>() - base_pitch.get::<radian>()).abs() < tol,
                "pitch should be fixed"
            );
        }
    }
}

