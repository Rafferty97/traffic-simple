use glam::DVec2;

pub trait DebugSink {
    type Object: ObjectSink;

    fn advance_frame(&mut self);

    fn object(&mut self, label: &str) -> Self::Object;
}

pub trait ObjectSink {
    fn line(&mut self, points: &[DVec2]);

    fn point(&mut self, point: DVec2, label: Option<&str>);

    fn datum(&mut self, key: &str, value: &str);
}
