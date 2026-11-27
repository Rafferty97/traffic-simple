//! Scenarios scripted in JS: the `network` and `sim` APIs bound for QuickJS.
//!
//! The JS API is the Rust API, names in camelCase, data crossing in its serde
//! form through JSON: a script's objects are what the Rust types serialize to,
//! and classes such as `Path` serialize themselves (`toJSON`), so they can sit
//! inside the data passed back. A script builds a network, hands it to
//! `simulate`, and inserts vehicles into the simulation that returns; the last
//! simulation it makes is the scenario's.
//!
//! ```js
//! const net = new Network(Side.Left);
//! const road = net.addRoad({
//!   path: new Path([0, 0]).lineTo([200, 0]),
//!   speedLimit: kmh(60),
//!   forward: [{ offset: -1.75, left: 1.75, right: 1.75 }],
//! });
//! const sim = simulate(net);
//! sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, { lane: road.lane(0), s: 10, speed: kmh(50) });
//! ```

use std::cell::RefCell;
use std::rc::Rc;

use glam::DVec2;
use rquickjs::class::Trace;
use rquickjs::context::EvalOptions;
use rquickjs::prelude::Func;
use rquickjs::{CatchResultExt, Class, Context, Ctx, Exception, JsLifetime, Runtime, Value};
use serde::Serialize;
use serde::de::DeserializeOwned;
use traffic_network::{Network, Path, RenderGeometry, RoadId, kmh, mph, render};
use traffic_sim::{Driver, Placement, Side, Simulation, VehicleSpec};

/// How far drawn curves stray from the network's at most, in meters: finer
/// than the simulation's lanes, which only vehicles follow.
const RENDER_TOLERANCE: f64 = 0.005;

/// A scenario as its script left it: its simulation and what to draw for it.
pub struct Scenario {
    pub sim: Simulation,
    pub render: RenderGeometry,
}

/// What a script's `simulate` calls made: the last of them.
#[derive(Default)]
struct Host {
    made: Option<(Rc<RefCell<Simulation>>, RenderGeometry)>,
}

/// Runs a scenario's script, `file` naming it in errors, and returns the last
/// simulation it made, or why it made none.
pub fn run(source: &str, file: &str) -> Result<Scenario, String> {
    let host = Rc::new(RefCell::new(Host::default()));
    {
        let runtime = Runtime::new().map_err(|e| e.to_string())?;
        let context = Context::full(&runtime).map_err(|e| e.to_string())?;
        context.with(|ctx| -> Result<(), String> {
            define(&ctx, &host).catch(&ctx).map_err(|e| e.to_string())?;
            let mut options = EvalOptions::default();
            options.filename = Some(file.to_string());
            ctx.eval_with_options::<(), _>(source, options)
                .catch(&ctx)
                .map_err(|e| e.to_string())
        })?;
        // The runtime goes here, and with it the script's hold on its
        // simulations.
    }
    let made = host.borrow_mut().made.take();
    let (sim, render) = made.ok_or("the script didn't call simulate")?;
    let sim = Rc::try_unwrap(sim).map_err(|_| "the script's simulation is still held")?;
    Ok(Scenario { sim: sim.into_inner(), render })
}

/// Defines the API's globals.
fn define<'js>(ctx: &Ctx<'js>, host: &Rc<RefCell<Host>>) -> rquickjs::Result<()> {
    let globals = ctx.globals();
    Class::<JsNetwork>::define(&globals)?;
    Class::<JsPath>::define(&globals)?;
    Class::<JsRoadId>::define(&globals)?;
    Class::<JsSimulation>::define(&globals)?;
    globals.set("kmh", Func::from(kmh))?;
    globals.set("mph", Func::from(mph))?;
    let host = host.clone();
    let simulate = move |ctx: Ctx<'js>, net: Class<'js, JsNetwork>| {
        let net = &net.borrow().inner;
        let fail =
            |e: traffic_network::CompileError| Exception::throw_message(&ctx, &e.to_string());
        let sim = traffic_network::simulate(net).map_err(fail)?;
        let render = render(net, RENDER_TOLERANCE).map_err(fail)?;
        let sim = Rc::new(RefCell::new(sim));
        host.borrow_mut().made = Some((sim.clone(), render));
        Class::instance(ctx.clone(), JsSimulation { sim })
    };
    globals.set("simulate", Func::from(simulate))?;
    // Constants, as their serde forms, frozen.
    let constants = [
        (
            "Side",
            to_json(&[("Left", Side::Left), ("Right", Side::Right)]),
        ),
        (
            "VehicleSpec",
            to_json(&[("PASSENGER_CAR", VehicleSpec::PASSENGER_CAR)]),
        ),
        ("Driver", to_json(&[("DEFAULT", Driver::DEFAULT)])),
    ];
    for (name, json) in constants {
        let value = ctx.json_parse(json)?;
        globals.set(name, value)?;
        ctx.eval::<(), _>(format!("Object.freeze({name});"))?;
    }
    Ok(())
}

/// Pairs of names and values as a JSON object.
fn to_json<T: Serialize>(pairs: &[(&str, T)]) -> String {
    let map: serde_json::Map<String, serde_json::Value> = pairs
        .iter()
        .map(|(k, v)| {
            (
                k.to_string(),
                serde_json::to_value(v).expect("it serializes"),
            )
        })
        .collect();
    serde_json::Value::Object(map).to_string()
}

/// A JS value as a Rust one, through its serde form, or a thrown error saying
/// why it isn't one.
fn from_js<'js, T: DeserializeOwned>(ctx: &Ctx<'js>, value: Value<'js>) -> rquickjs::Result<T> {
    let json = ctx.json_stringify(value)?.map(|s| s.to_string()).transpose()?;
    let json = json.unwrap_or_else(|| "null".into());
    serde_json::from_str(&json).map_err(|e| Exception::throw_type(ctx, &e.to_string()))
}

/// A Rust value as a JS one, through its serde form.
fn into_js<'js, T: Serialize>(ctx: &Ctx<'js>, value: &T) -> rquickjs::Result<Value<'js>> {
    ctx.json_parse(serde_json::to_string(value).expect("it serializes"))
}

/// A point, `[x, y]`.
fn point<'js>(ctx: &Ctx<'js>, value: Value<'js>) -> rquickjs::Result<DVec2> {
    from_js(ctx, value)
}

#[derive(Trace, JsLifetime)]
#[rquickjs::class(rename = "Network")]
pub struct JsNetwork {
    #[qjs(skip_trace)]
    inner: Network,
}

#[rquickjs::methods(rename_all = "camelCase")]
impl JsNetwork {
    #[qjs(constructor)]
    fn new<'js>(ctx: Ctx<'js>, drives_on: Value<'js>) -> rquickjs::Result<Self> {
        Ok(Self { inner: Network::new(from_js(&ctx, drives_on)?) })
    }

    fn add_road<'js>(&mut self, ctx: Ctx<'js>, road: Value<'js>) -> rquickjs::Result<JsRoadId> {
        Ok(JsRoadId { inner: self.inner.add_road(from_js(&ctx, road)?) })
    }

    #[qjs(rename = "toJSON")]
    fn to_json<'js>(&self, ctx: Ctx<'js>) -> rquickjs::Result<Value<'js>> {
        into_js(&ctx, &self.inner)
    }
}

#[derive(Trace, JsLifetime, Clone)]
#[rquickjs::class(rename = "Path")]
pub struct JsPath {
    #[qjs(skip_trace)]
    inner: Path,
}

#[rquickjs::methods(rename_all = "camelCase")]
impl JsPath {
    #[qjs(constructor)]
    fn new<'js>(ctx: Ctx<'js>, start: Value<'js>) -> rquickjs::Result<Self> {
        Ok(Self { inner: Path::new(point(&ctx, start)?) })
    }

    fn line_to<'js>(&self, ctx: Ctx<'js>, to: Value<'js>) -> rquickjs::Result<Self> {
        Ok(Self { inner: self.inner.clone().line_to(point(&ctx, to)?) })
    }

    fn cubic_to<'js>(
        &self,
        ctx: Ctx<'js>,
        c1: Value<'js>,
        c2: Value<'js>,
        to: Value<'js>,
    ) -> rquickjs::Result<Self> {
        let [c1, c2, to] = [point(&ctx, c1)?, point(&ctx, c2)?, point(&ctx, to)?];
        Ok(Self { inner: self.inner.clone().cubic_to(c1, c2, to) })
    }

    fn slice(&self, from: f64, to: f64) -> Self {
        Self { inner: self.inner.slice(from, to) }
    }

    fn length(&self) -> f64 {
        self.inner.length()
    }

    #[qjs(rename = "toJSON")]
    fn to_json<'js>(&self, ctx: Ctx<'js>) -> rquickjs::Result<Value<'js>> {
        into_js(&ctx, &self.inner)
    }
}

#[derive(Trace, JsLifetime, Clone, Copy)]
#[rquickjs::class(rename = "RoadId")]
pub struct JsRoadId {
    #[qjs(skip_trace)]
    inner: RoadId,
}

#[rquickjs::methods(rename_all = "camelCase")]
impl JsRoadId {
    /// Throws, where Rust panics, if the road has no such lane.
    fn lane<'js>(&self, ctx: Ctx<'js>, i: usize) -> rquickjs::Result<Value<'js>> {
        let count = self.inner.lane_count();
        if i >= count {
            let message = format!(
                "road {} has {count} lanes, not {}",
                self.inner.index(),
                i + 1
            );
            return Err(Exception::throw_range(&ctx, &message));
        }
        into_js(&ctx, &self.inner.lane(i))
    }

    fn lane_count(&self) -> usize {
        self.inner.lane_count()
    }

    #[qjs(rename = "toJSON")]
    // Bound to JS, which passes the receiver by reference.
    #[allow(clippy::wrong_self_convention)]
    fn to_json<'js>(&self, ctx: Ctx<'js>) -> rquickjs::Result<Value<'js>> {
        into_js(&ctx, &self.inner)
    }
}

#[derive(Trace, JsLifetime)]
#[rquickjs::class(rename = "Simulation")]
pub struct JsSimulation {
    #[qjs(skip_trace)]
    sim: Rc<RefCell<Simulation>>,
}

#[rquickjs::methods(rename_all = "camelCase")]
impl JsSimulation {
    fn insert<'js>(
        &self,
        ctx: Ctx<'js>,
        spec: Value<'js>,
        driver: Value<'js>,
        at: Value<'js>,
    ) -> rquickjs::Result<Value<'js>> {
        let spec: VehicleSpec = from_js(&ctx, spec)?;
        let driver: Driver = from_js(&ctx, driver)?;
        let at: Placement = from_js(&ctx, at)?;
        let lanes = self.sim.borrow().network().lanes().count();
        if at.lane.0 as usize >= lanes {
            return Err(Exception::throw_range(
                &ctx,
                &format!("no lane {}", at.lane.0),
            ));
        }
        let id = self.sim.borrow_mut().insert(spec, driver, at);
        into_js(&ctx, &id)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn a_script_builds_a_road_and_inserts_a_car() {
        let scenario = run(
            r#"
            const net = new Network(Side.Left);
            const path = new Path([0, 0]).lineTo([200, 0]).cubicTo([300, 0], [350, 50], [350, 150]);
            const road = net.addRoad({
              path: path.slice(10, path.length()),
              speedLimit: kmh(60),
              forward: [{ offset: -1.75, left: 1.75, right: 1.75 }],
              reverse: [{ offset: 1.75, left: 1.75, right: 1.75 }],
            });
            const sim = simulate(net);
            if (VehicleSpec.PASSENGER_CAR.maxSpeed === undefined || Driver.DEFAULT.speedFactor === undefined) {
              throw new Error("fields aren't in camelCase");
            }
            sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, { lane: road.lane(1), s: 10, speed: kmh(50) });
            "#,
            "test.js",
        )
        .unwrap();
        assert_eq!(scenario.sim.network().lanes().count(), 2);
        let (_, car) = scenario.sim.vehicles().next().expect("a car");
        assert_eq!(car.lane(), traffic_sim::LaneId(1));
        assert!(!scenario.render.roads.is_empty());
    }

    #[test]
    fn a_script_s_errors_name_where_they_happened() {
        let error = run(
            "const net = new Network(Side.Left);\nnet.addRoad({});",
            "broken.js",
        )
        .err()
        .unwrap();
        assert!(error.contains("broken.js"), "{error}");
        let error = run("const x = 1;", "nothing.js").err().unwrap();
        assert!(error.contains("simulate"), "{error}");
    }

    #[test]
    fn every_scenario_runs() {
        let dir = std::path::Path::new(crate::scenarios::DIR);
        for file in crate::scenarios::list(dir).unwrap() {
            let source = std::fs::read_to_string(&file.path).unwrap();
            let scenario = run(&source, &file.path.to_string_lossy());
            let scenario = scenario.unwrap_or_else(|e| panic!("{}: {e}", file.name));
            let mut sim = scenario.sim;
            assert!(sim.vehicles().count() > 0, "{} has no vehicles", file.name);
            for _ in 0..900 {
                sim.step();
            }
        }
    }

    #[test]
    fn the_editors_declarations_declare_every_global() {
        let path = std::path::Path::new(crate::scenarios::DIR).join("api.d.ts");
        let declarations = std::fs::read_to_string(path).unwrap();
        let runtime = Runtime::new().unwrap();
        let context = Context::full(&runtime).unwrap();
        context.with(|ctx| {
            // The globals a bare context has, then those the API adds.
            let names = |ctx: &Ctx| -> Vec<String> {
                let keys = ctx.globals().keys::<String>();
                keys.collect::<rquickjs::Result<_>>().unwrap()
            };
            let bare = names(&ctx);
            define(&ctx, &Rc::default()).unwrap();
            let added: Vec<String> =
                names(&ctx).into_iter().filter(|n| !bare.contains(n)).collect();
            assert!(added.len() >= 8, "{added:?}");
            for name in added {
                let declared = ["const", "class", "function"]
                    .iter()
                    .any(|kind| declarations.contains(&format!("declare {kind} {name}")));
                assert!(declared, "api.d.ts doesn't declare {name}");
            }
        });
    }

    #[test]
    fn a_lane_beyond_the_road_s_throws() {
        let error = run(
            r#"
            const net = new Network(Side.Right);
            const road = net.addRoad({
              path: new Path([0, 0]).lineTo([100, 0]),
              speedLimit: mph(30),
              forward: [{ offset: 0, left: 1.75, right: 1.75 }],
            });
            road.lane(1);
            "#,
            "lanes.js",
        )
        .err()
        .unwrap();
        assert!(error.contains("has 1 lanes"), "{error}");
    }
}
