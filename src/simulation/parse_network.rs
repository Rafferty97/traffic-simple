use thiserror::Error;

use crate::geometry::{Lane, LaneError};
use crate::network::{Network, Path, PathSegment};
use crate::simulation::{Link, Simulation};

#[derive(Error, Debug)]
pub enum NetworkError {
    #[error("invalid path: {0}")]
    LaneError(LaneError),
}

impl Simulation {
    pub fn insert_network(&mut self, network: &Network) -> Result<(), NetworkError> {
        for road in &network.roads {
            let lane = create_lane(&road.path, road.width)?;
            self.links
                .insert_with_key(|id| Link { id, name: format!("link:{}", road.id), lane });
        }

        Ok(())
    }
}

fn create_lane(path: &Path, width: f64) -> Result<Lane, NetworkError> {
    let mut points = vec![];

    for segment in &path.0 {
        match *segment {
            PathSegment::Line { from, to } => {
                points.push(from.into());
                points.push(to.into());
            }
        }
    }

    Ok(Lane::new(&points, width)?)
}

impl From<LaneError> for NetworkError {
    fn from(err: LaneError) -> Self {
        Self::LaneError(err)
    }
}
