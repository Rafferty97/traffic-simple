use glam::DVec2;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct Network {
    pub roads: Vec<Road>,
}

#[derive(Serialize, Deserialize)]
pub struct Road {
    pub id: String,
    pub path: Path,
    pub width: f64,
}

#[derive(Serialize, Deserialize)]
pub struct Path(pub Vec<PathSegment>);

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
#[serde(tag = "t")]
#[serde(rename_all = "snake_case")]
pub enum PathSegment {
    Line {
        from: Pos,
        to: Pos,
    },
    Arc {
        center: Pos,
        radius: f64,
        start_angle: f64,
        sweep: f64,
    },
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub struct Pos(f64, f64);

impl From<Pos> for DVec2 {
    fn from(value: Pos) -> Self {
        DVec2 { x: value.0, y: value.1 }
    }
}
