//! Miscellaneous utility structs and functions.

use std::fmt::Debug;

use cgmath::num_traits::Float;
use serde::{Deserialize, Serialize};

/// An interval on the real number line.
#[derive(Copy, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Interval<T> {
    pub min: T,
    pub max: T,
}

impl<T> Interval<T> {
    /// Creates a new interval.
    pub const fn new(min: T, max: T) -> Self {
        Self { min, max }
    }
}

impl<T: std::cmp::PartialOrd> Interval<T> {
    /// Returns true if this interval overlaps with the other.
    pub fn overlaps(&self, other: &Self) -> bool {
        self.max > other.min && other.max > self.min
    }

    /// Returns true if this interval contains the value.
    pub fn contains(&self, value: T) -> bool {
        value >= self.min && value <= self.max
    }
}

impl<T: std::ops::Sub<T, Output = T> + Copy> Interval<T> {
    /// Gets the magnitude of the interval.
    pub fn length(&self) -> T {
        self.max - self.min
    }
}

impl<T: Copy> Interval<T> {
    /// Gets the interval as an array.
    pub fn as_array(&self) -> [T; 2] {
        [self.min, self.max]
    }
}

impl<T: Float> Interval<T> {
    /// Creates an interval with the given centre and radius.
    pub fn disc(centre: T, radius: T) -> Self {
        Self {
            min: centre - radius,
            max: centre + radius,
        }
    }

    /// Returns the centre/mid-point of the interval.
    pub fn midpoint(&self) -> T {
        T::from(0.5).unwrap() * (self.min + self.max)
    }

    /// Computes the gap between two intervals.
    /// Will be negative if the intervals overlap.
    pub fn clearance_with(&self, other: &Self) -> T {
        T::max(other.min - self.max, self.min - other.max)
    }

    /// Computes the distance between a point and the interval.
    /// Will be negative if the point is within the interval.
    pub fn distance(&self, other: T) -> T {
        T::max(other - self.max, self.min - other)
    }

    pub fn lerp(&self, t: T) -> T {
        self.min + t * (self.max - self.min)
    }

    pub fn inv_lerp(&self, value: T) -> T {
        (value - self.min) / (self.max - self.min)
    }
}

impl<T: Float> std::ops::Add<T> for Interval<T> {
    type Output = Interval<T>;

    fn add(self, rhs: T) -> Self::Output {
        Self {
            min: self.min + rhs,
            max: self.max + rhs,
        }
    }
}

impl<T: Float> std::ops::Sub<T> for Interval<T> {
    type Output = Interval<T>;

    fn sub(self, rhs: T) -> Self::Output {
        Self {
            min: self.min - rhs,
            max: self.max - rhs,
        }
    }
}

impl<T: Debug> Debug for Interval<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Interval({:?}, {:?})", &self.min, &self.max)
    }
}

pub fn rotated_range(count: usize, start: usize) -> impl Iterator<Item = usize> {
    (0..count)
        .map(move |i| i + start)
        .map(move |i| if i >= count { i - count } else { i })
}
