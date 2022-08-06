use crate::math::Point2d;
#[cfg(feature = "debug")]
use serde_json::json;

#[cfg(feature = "debug")]
thread_local!(
    static DEBUG_FRAME: std::cell::RefCell<Vec<serde_json::Value>> = Default::default();
);

#[allow(unused)]
pub fn debug_line(name: &str, p1: Point2d, p2: Point2d) {
    #[cfg(feature = "debug")]
    DEBUG_FRAME.with(|frame| {
        frame.borrow_mut().push(json!({
            "type": "line",
            "name": name,
            "p1": [p1.x, p1.y],
            "p2": [p2.x, p2.y],
        }))
    })
}

#[allow(unused)]
pub fn debug_circle(name: &str, centre: Point2d, radius: f64) {
    #[cfg(feature = "debug")]
    DEBUG_FRAME.with(|frame| {
        frame.borrow_mut().push(json!({
            "type": "line",
            "name": name,
            "centre": [centre.x, centre.y],
            "radius": radius
        }))
    })
}

#[cfg(feature = "debug")]
pub fn take_debug_frame() -> serde_json::Value {
    json!(DEBUG_FRAME.with(|frame| frame.take()))
}
