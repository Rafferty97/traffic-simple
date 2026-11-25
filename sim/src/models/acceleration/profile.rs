//! Each lane's speed profile: the fastest a driver would go at each point
//! along it, its nominal limit lowered where it bends, as a few points with
//! straight lines between them. Derived from the network when a simulation is
//! built or restored, not saved.

use crate::network::{LaneId, SimNetwork};

/// The most a driver accepts sideways on a bend, in m/s²: as AASHTO's side
/// friction factors for intersection curves at 15–25 km/h have it.
pub const CURVE_ACCEL: f64 = 3.5;

/// The least a profile lowers a lane's speed to for a bend, in m/s.
pub const MIN_CURVE_SPEED: f64 = 3.0;

/// How far a profile's straight lines may stray from the speed at each vertex,
/// in m/s.
const TOLERANCE: f64 = 0.3;

/// Every lane's speed profile, by lane: its points, `(s, speed)`, in one list,
/// each lane's a range of it.
#[derive(Clone, Debug, Default)]
pub struct Profiles {
    points: Vec<(f64, f64)>,
    ranges: Vec<(u32, u32)>,
    /// Each lane's lowest speed.
    lowest: Vec<f64>,
}

impl Profiles {
    pub fn new(network: &SimNetwork) -> Self {
        let mut profiles = Self::default();
        let mut speeds = Vec::new();
        for (_, lane) in network.lanes() {
            let geometry = lane.geometry();
            let limit = lane.speed_limit();
            let s: Vec<f64> = geometry.vertex_s().collect();
            let p: Vec<_> = geometry.centerline().collect();
            // At each vertex, the speed for its bend: its turn over the mean of
            // the segments either side. Each end takes the vertex next to it.
            speeds.clear();
            speeds.extend((0..p.len()).map(|i| {
                let i = i.clamp(1, p.len().saturating_sub(2).max(1));
                if p.len() < 3 {
                    return limit;
                }
                let (a, b) = (p[i] - p[i - 1], p[i + 1] - p[i]);
                let curvature = a.angle_to(b).abs() / (0.5 * (a.length() + b.length()));
                match curvature > 0.0 {
                    true => limit.min((CURVE_ACCEL / curvature).sqrt().max(MIN_CURVE_SPEED)),
                    false => limit,
                }
            }));
            let start = profiles.points.len() as u32;
            simplify(&s, &speeds, 0, s.len() - 1, &mut profiles.points);
            profiles.points.push((s[s.len() - 1], speeds[s.len() - 1]));
            profiles.ranges.push((start, profiles.points.len() as u32));
            profiles.lowest.push(speeds.iter().copied().fold(limit, f64::min));
        }
        profiles
    }

    /// A lane's points, `(s, speed)`, in order along it.
    pub fn of(&self, lane: LaneId) -> &[(f64, f64)] {
        let (a, b) = self.ranges[lane.0 as usize];
        &self.points[a as usize..b as usize]
    }

    /// A lane's speed at `s` along it, between its points, or at the nearer
    /// end beyond them.
    pub fn at(&self, lane: LaneId, s: f64) -> f64 {
        let points = self.of(lane);
        let i = points.partition_point(|&(x, _)| x <= s);
        match i {
            0 => points[0].1,
            i if i == points.len() => points[i - 1].1,
            i => {
                let ((s0, v0), (s1, v1)) = (points[i - 1], points[i]);
                v0 + (v1 - v0) * (s - s0) / (s1 - s0)
            }
        }
    }

    /// A lane's lowest speed.
    pub fn lowest(&self, lane: LaneId) -> f64 {
        self.lowest[lane.0 as usize]
    }
}

/// The points of `speeds` along `s`, from the `from`th to before the `to`th,
/// that a straight line from each kept point to the next keeps within
/// [`TOLERANCE`] of every speed between them: Ramer–Douglas–Peucker.
fn simplify(s: &[f64], speeds: &[f64], from: usize, to: usize, out: &mut Vec<(f64, f64)>) {
    let line = |i: usize| {
        let t = (s[i] - s[from]) / (s[to] - s[from]).max(f64::MIN_POSITIVE);
        speeds[from] + t * (speeds[to] - speeds[from])
    };
    let worst = (from + 1..to)
        .map(|i| (i, (speeds[i] - line(i)).abs()))
        .max_by(|a, b| a.1.total_cmp(&b.1));
    match worst {
        Some((i, off)) if off > TOLERANCE => {
            simplify(s, speeds, from, i, out);
            simplify(s, speeds, i, to, out);
        }
        _ => out.push((s[from], speeds[from])),
    }
}
