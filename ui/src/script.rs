//! Scenarios scripted in JS: the `network` and `sim` APIs bound for QuickJS.
//!
//! The JS API is the Rust API, names in camelCase, data crossing in its serde
//! form through JSON: a script's objects are what the Rust types serialize to,
//! and classes such as `Path` serialize themselves (`toJSON`), so they can sit
//! inside the data passed back. A script builds a network, compiles it, makes
//! a simulation of it, and inserts vehicles where the compiled network places
//! them; the last simulation it makes is the scenario's.
//!
//! ```js
//! const net = new Network(Side.Left);
//! const road = net.addRoad({
//!   path: new Path([0, 0]).lineTo([200, 0]),
//!   speedLimit: kmh(60),
//!   forward: [{ offset: -1.75, left: 1.75, right: 1.75 }],
//! });
//! const compiled = compile(net);
//! const sim = compiled.simulate();
//! sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(road, 0, 10, kmh(50)));
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
use traffic_network::{Compiled, Network, Path, RenderGeometry, RoadId, kmh, mph, render};
use traffic_sim::{Driver, LaneId, Placement, Route, Side, Simulation, VehicleSpec};

/// How far drawn curves stray from the network's at most, in meters: finer
/// than the simulation's lanes, which only vehicles follow.
const RENDER_TOLERANCE: f64 = 0.005;

/// A scenario as its script left it: its simulation and what to draw for it.
pub struct Scenario {
    pub sim: Simulation,
    pub render: RenderGeometry,
}

/// What a script's `simulate` calls made: the last of them, with what to draw
/// for its network.
#[derive(Default)]
struct Host {
    made: Option<(Rc<RefCell<Simulation>>, RenderGeometry)>,
}

/// Runs a scenario's script, `file` naming it in errors, and returns the last
/// simulation it made, or why it made none. A script in TypeScript, `file`
/// ending in `.ts`, has its types stripped first, and its errors name places
/// in it as it was written.
pub fn run(source: &str, file: &str) -> Result<Scenario, String> {
    let stripped = match file.ends_with(".ts") {
        true => Some(crate::typescript::strip(source, file)?),
        false => None,
    };
    let code = stripped.as_ref().map_or(source, |s| s.code.as_str());
    let in_script = |error: String| match &stripped {
        Some(stripped) => stripped.map_error(file, &error),
        None => error,
    };
    let host = Rc::new(RefCell::new(Host::default()));
    {
        let runtime = Runtime::new().map_err(|e| e.to_string())?;
        let context = Context::full(&runtime).map_err(|e| e.to_string())?;
        context
            .with(|ctx| -> Result<(), String> {
                define(&ctx, &host).catch(&ctx).map_err(|e| e.to_string())?;
                let mut options = EvalOptions::default();
                options.filename = Some(file.to_string());
                ctx.eval_with_options::<(), _>(code, options)
                    .catch(&ctx)
                    .map_err(|e| e.to_string())
            })
            .map_err(in_script)?;
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
    Class::<JsCompiled>::define(&globals)?;
    Class::<JsSimulation>::define(&globals)?;
    globals.set("kmh", Func::from(kmh))?;
    globals.set("mph", Func::from(mph))?;
    let host = host.clone();
    let compile = move |ctx: Ctx<'js>, net: Class<'js, JsNetwork>| {
        let net = &net.borrow().inner;
        let fail =
            |e: traffic_network::CompileError| Exception::throw_message(&ctx, &e.to_string());
        let compiled = traffic_network::compile(net).map_err(fail)?;
        let render = render(net, RENDER_TOLERANCE).map_err(fail)?;
        let inner = Rc::new((compiled, render));
        Class::instance(ctx.clone(), JsCompiled { inner, host: host.clone() })
    };
    globals.set("compile", Func::from(compile))?;
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

    fn overlap<'js>(
        &mut self,
        ctx: Ctx<'js>,
        a: Value<'js>,
        b: Value<'js>,
    ) -> rquickjs::Result<()> {
        self.inner.overlap(from_js(&ctx, a)?, from_js(&ctx, b)?);
        Ok(())
    }

    fn join<'js>(
        &mut self,
        ctx: Ctx<'js>,
        from: Value<'js>,
        to: Value<'js>,
    ) -> rquickjs::Result<()> {
        self.inner.join(from_js(&ctx, from)?, from_js(&ctx, to)?);
        Ok(())
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

    fn arc_to<'js>(&self, ctx: Ctx<'js>, center: Value<'js>, sweep: f64) -> rquickjs::Result<Self> {
        Ok(Self {
            inner: self.inner.clone().arc_to(point(&ctx, center)?, sweep),
        })
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
#[rquickjs::class(rename = "Compiled")]
pub struct JsCompiled {
    /// The network compiled, and what to draw for it.
    #[qjs(skip_trace)]
    inner: Rc<(Compiled, RenderGeometry)>,
    #[qjs(skip_trace)]
    host: Rc<RefCell<Host>>,
}

#[rquickjs::methods(rename_all = "camelCase")]
impl JsCompiled {
    /// A simulation of it, which becomes the scenario's.
    fn simulate<'js>(&self, ctx: Ctx<'js>) -> rquickjs::Result<Class<'js, JsSimulation>> {
        let (compiled, render) = &*self.inner;
        let sim = Rc::new(RefCell::new(compiled.simulate()));
        self.host.borrow_mut().made = Some((sim.clone(), render.clone()));
        Class::instance(ctx, JsSimulation { sim, destinations: Default::default() })
    }

    fn place<'js>(
        &self,
        ctx: Ctx<'js>,
        road: Value<'js>,
        lane: usize,
        s: f64,
        speed: f64,
    ) -> rquickjs::Result<Value<'js>> {
        let road: RoadId = from_js(&ctx, road)?;
        let at = self.inner.0.place(road, lane, s, speed);
        let at = at.map_err(|e| Exception::throw_range(&ctx, &e.to_string()))?;
        into_js(&ctx, &at)
    }

    /// Throws, where Rust panics, if the road isn't the network's or has no
    /// such lane.
    fn lanes<'js>(
        &self,
        ctx: Ctx<'js>,
        road: Value<'js>,
        lane: usize,
    ) -> rquickjs::Result<Value<'js>> {
        let road: RoadId = from_js(&ctx, road)?;
        let compiled = &self.inner.0;
        // Placing a vehicle checks the same, without panicking.
        if let Err(e) = compiled.place(road, lane, 0.0, 0.0) {
            return Err(Exception::throw_range(&ctx, &e.to_string()));
        }
        into_js(&ctx, &compiled.lanes(road, lane))
    }
}

#[derive(Trace, JsLifetime)]
#[rquickjs::class(rename = "Simulation")]
pub struct JsSimulation {
    #[qjs(skip_trace)]
    sim: Rc<RefCell<Simulation>>,
    /// How many destinations the script has registered.
    #[qjs(skip_trace)]
    destinations: std::cell::Cell<u32>,
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
        if let Route::Destination(d) = at.route
            && d.0 >= self.destinations.get()
        {
            let message = format!("no destination {}", d.0);
            return Err(Exception::throw_range(&ctx, &message));
        }
        let id = self.sim.borrow_mut().insert(spec, driver, at);
        into_js(&ctx, &id)
    }

    /// Throws, where Rust panics, if a lane isn't the network's.
    fn add_destination<'js>(
        &self,
        ctx: Ctx<'js>,
        lanes: Value<'js>,
    ) -> rquickjs::Result<Value<'js>> {
        let lanes: Vec<LaneId> = from_js(&ctx, lanes)?;
        let count = self.sim.borrow().network().lanes().count();
        if let Some(lane) = lanes.iter().find(|l| l.0 as usize >= count) {
            return Err(Exception::throw_range(&ctx, &format!("no lane {}", lane.0)));
        }
        let id = self.sim.borrow_mut().add_destination(&lanes);
        self.destinations.set(self.destinations.get() + 1);
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
            const compiled = compile(net);
            const sim = compiled.simulate();
            if (VehicleSpec.PASSENGER_CAR.maxSpeed === undefined || Driver.DEFAULT.speedFactor === undefined) {
              throw new Error("fields aren't in camelCase");
            }
            sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(road, 1, 10, kmh(50)));
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
    fn a_script_joins_roads() {
        let scenario = run(
            r#"
            const net = new Network(Side.Left);
            const lane = { offset: 0, left: 1.75, right: 1.75 };
            const a = net.addRoad({ path: new Path([0, 0]).lineTo([100, 0]), speedLimit: kmh(60), forward: [lane] });
            const b = net.addRoad({ path: new Path([100, 0]).lineTo([200, 0]), speedLimit: kmh(60), forward: [lane] });
            net.join(a, b);
            const compiled = compile(net);
            const sim = compiled.simulate();
            sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(a, 0, 10, kmh(50)));
            const [onward] = compiled.lanes(b, 0);
            if (onward !== 1) throw new Error(`road b's lane is ${onward}`);
            "#,
            "join.js",
        )
        .unwrap();
        let lanes = scenario.sim.network();
        assert_eq!(
            lanes.lane(traffic_sim::LaneId(0)).successors(),
            [traffic_sim::LaneId(1)]
        );
        // And a join that doesn't meet up throws.
        let error = run(
            r#"
            const net = new Network(Side.Left);
            const lane = { offset: 0, left: 1.75, right: 1.75 };
            const a = net.addRoad({ path: new Path([0, 0]).lineTo([100, 0]), speedLimit: kmh(60), forward: [lane] });
            const b = net.addRoad({ path: new Path([110, 0]).lineTo([200, 0]), speedLimit: kmh(60), forward: [lane] });
            net.join(a, b);
            compile(net);
            "#,
            "gap.js",
        )
        .err()
        .unwrap();
        assert!(error.contains("meets no lane"), "{error}");
    }

    #[test]
    fn a_script_in_typescript_runs_and_its_errors_name_its_own_lines() {
        let script = r#"
            interface Lanes { offset: number }
            const lane = (l: Lanes): RoadLane => ({ offset: l.offset, left: 1.75, right: 1.75 });
            const net: Network = new Network(Side.Left);
            const road: RoadId = net.addRoad({
              path: new Path([0, 0]).lineTo([100, 0]),
              speedLimit: kmh(60),
              forward: [lane({ offset: 0 })],
            });
            const compiled = compile(net);
            const sim = compiled.simulate();
            sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(road, 0, 10, kmh(50)));
            "#;
        let scenario = run(script, "typed.ts").unwrap();
        assert_eq!(scenario.sim.vehicles().count(), 1);
        // A failure on line 4 of the script, after an interface stripping
        // takes out.
        let error = run(
            "interface A {
  x: number;
}
const net = new Network(Side.Left) as unknown as A;
net.nothing();",
            "broken.ts",
        )
        .err()
        .unwrap();
        assert!(error.contains("broken.ts:5:"), "{error}");
    }

    #[test]
    fn a_script_routes_vehicles_to_destinations_it_registers_and_no_others() {
        let script = |register: bool| {
            format!(
                r#"
                const net = new Network(Side.Left);
                const road = net.addRoad({{
                  path: new Path([0, 0]).lineTo([100, 0]),
                  speedLimit: kmh(60),
                  forward: [{{ offset: 0, left: 1.75, right: 1.75 }}],
                }});
                const compiled = compile(net);
                const sim = compiled.simulate();
                const end = {register} ? sim.addDestination(compiled.lanes(road, 0)) : 0;
                const at = compiled.place(road, 0, 10, kmh(50));
                sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, {{ ...at, route: {{ Destination: end }} }});
                "#
            )
        };
        let scenario = run(&script(true), "routed.js").unwrap();
        let (id, _) = scenario.sim.vehicles().next().unwrap();
        assert!(matches!(
            scenario.sim.route(id),
            Some(Route::Destination(_))
        ));
        let error = run(&script(false), "unrouted.js").err().unwrap();
        assert!(error.contains("no destination 0"), "{error}");
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
            compile(net).place(road, 1, 10, 0);
            "#,
            "lanes.js",
        )
        .err()
        .unwrap();
        assert!(error.contains("has 1 lanes"), "{error}");
    }
}
