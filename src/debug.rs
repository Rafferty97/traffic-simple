use glam::DVec2;

pub trait DebugSink {
    type Object<'a>: ObjectSink
    where
        Self: 'a;

    fn advance_frame(&mut self);

    fn object<'a>(&'a mut self, label: &str) -> Self::Object<'a>
    where
        Self: 'a;
}

pub trait ObjectSink {
    fn polyline(&mut self, points: &[DVec2], closed: bool);

    fn point(&mut self, point: DVec2, label: Option<&str>);

    fn datum(&mut self, key: &str, value: &str);
}
