use crate::util::Interval;

/// A lookup table

#[derive(Clone)]
pub struct LookupTable<T> {
    offset: f64,
    step: f64,
    values: Vec<T>,
}

impl<T> LookupTable<T> {
    /// Creates a lookup table from a sample function.
    pub fn from_samples(range: Interval<f64>, step: f64, f: impl FnMut(f64) -> T) -> Self {
        let offset = range.min;
        let num_samples = (range.length() / step).ceil() as usize;
        let xs = (0..num_samples).map(|i| offset + ((i as f64) + 0.5) * step);
        let values = xs.map(f).collect();
        Self {
            offset,
            step,
            values,
        }
    }

    /// Samples the lookup table.
    pub fn sample(&self, x: f64) -> &T {
        let idx = (x - self.offset) / self.step;
        let idx = usize::min(idx as u32 as usize, self.values.len() - 1);
        &self.values[idx]
    }
}

#[cfg(test)]
mod test {
    use super::LookupTable;
    use crate::util::Interval;

    #[test]
    fn basic_lut() {
        let range = Interval::new(50.0, 200.0);
        let lut = LookupTable::from_samples(range, 5.0, |x| 2.0 * x);

        assert_eq!(*lut.sample(20.0), 105.0);

        assert_eq!(*lut.sample(50.0), 105.0);
        assert_eq!(*lut.sample(52.0), 105.0);
        assert_eq!(*lut.sample(54.0), 105.0);

        assert_eq!(*lut.sample(90.0), 185.0);
        assert_eq!(*lut.sample(92.0), 185.0);
        assert_eq!(*lut.sample(94.0), 185.0);

        assert_eq!(*lut.sample(195.0), 395.0);
        assert_eq!(*lut.sample(197.0), 395.0);
        assert_eq!(*lut.sample(199.0), 395.0);

        assert_eq!(*lut.sample(888.0), 395.0);
    }
}
