use crate::util::Interval;

/// A lookup table

#[derive(Clone)]
pub struct LookupTable {
    offset: f64,
    step: f64,
    values: Vec<f64>
}

impl LookupTable {
    /// Creates a lookup table from a sample function.
    pub fn from_samples(range: Interval<f64>, step: f64, f: impl FnMut(f64) -> f64) -> Self {
        let offset = range.min;
        let num_samples = (range.length() / step).ceil() as usize;
        let xs = (0..num_samples).map(|i| offset + ((i as f64) + 0.5) * step);
        let values = xs.map(f).collect();
        Self { offset, step, values }
    }
    
    /// Samples the lookup table.
    pub fn sample(&self, x: f64) -> f64 {
        let idx = (x - self.offset) / self.step;
        let idx = usize::min(idx as u32 as usize, self.values.len() - 1);
        self.values[idx]
    }
}