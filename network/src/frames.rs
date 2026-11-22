//! Freeze frames built by script: a scenario set up and run to a moment worth
//! looking at, for the UI to open paused.

use traffic_sim::{
    BarrierKind, LaneId, Light, MovementId, NoopSink, Placement, Route, SimState, Simulation,
    VehicleId, VehicleSpec,
};

use crate::{Compiled, RenderGeometry, Turn, compile, scenarios};

/// A simulation's state at one moment, what to draw for its network, and the
/// vehicle to select.
pub struct Frame {
    pub state: SimState,
    pub render: RenderGeometry,
    pub selected: Option<VehicleId>,
}

/// The counterexample to a junction's queue being free of cycles without its
/// fourth invariant (see `docs/DESIGN.md`, Queue invariants): on a signalised
/// crossroads of 4-lane roads, a right turner T waits at its waiting point
/// for a gap, its light now red; a car X from the cross street, its light
/// green, is placed inside the junction, behind T's first stretch, as if
/// nothing had held it at its line. X's way crosses both T's waiting stretch
/// and its turn: X waits for T's body to leave its way, and T, giving way to
/// X, waits for X. T is selected.
pub fn turner_cycle() -> Frame {
    let compiled =
        compile(&scenarios::signalised_crossroads(2, 50.0 / 3.6)).expect("the crossroads compile");
    let net = &compiled.sim;
    let movement = |lane: LaneId| {
        net.lane(lane).barriers().iter().find_map(|b| match b.kind {
            BarrierKind::Signal { movement } => Some(movement),
            _ => None,
        })
    };
    let (plan, first) = net.signals().next().expect("a signal plan");
    let together = |a: MovementId, b: MovementId| {
        let bits = (1u64 << (a.0 - first.0)) | (1u64 << (b.0 - first.0));
        plan.phases.iter().any(|p| p.green & bits == bits)
    };
    let ways = compiled.connectors[0].iter().copied().zip(compiled.turns[0].iter().copied());
    let straight = || ways.clone().filter(|&(_, turn)| turn == Turn::Straight).map(|(l, _)| l);
    // A right turn, waiting at the start of its second lane; a straight
    // connector from the cross street, never green with it, crossing both its
    // lanes; and one oncoming, green with it, crossing its second.
    let (t_first, x_lane, o_lane) = ways
        .clone()
        .filter(|&(_, turn)| turn == Turn::Right)
        .find_map(|(first_lane, _)| {
            let second = *net.lane(first_lane).successors().first()?;
            let t_movement = movement(first_lane)?;
            let with = |l: LaneId| movement(l).is_some_and(|m| together(m, t_movement));
            let meets = |l: LaneId, lane: LaneId| net.conflict_between(l, lane).is_some();
            let x = straight().find(|&x| meets(x, first_lane) && meets(x, second) && !with(x))?;
            let o = straight().find(|&o| meets(o, second) && !meets(o, first_lane) && with(o))?;
            Some((first_lane, x, o))
        })
        .expect("a right turn, a straight across both its lanes, and one oncoming");
    let (t_movement, x_movement) = (movement(t_first).unwrap(), movement(x_lane).unwrap());
    let approach = net.lane(t_first).predecessors()[0];
    let approach_length = net.lane(approach).geometry().length();
    let Compiled { sim: network, render, .. } = compiled;
    let mut sim = Simulation::new(network, NoopSink);
    let car = VehicleSpec::passenger_car();
    let run_until = |sim: &mut Simulation, done: &dyn Fn(&Simulation) -> bool| {
        for _ in 0..(120 * 60) {
            if done(sim) {
                return;
            }
            sim.step();
        }
        panic!("the turner cycle never came about");
    };
    let at_rest = |lane: LaneId, s: f64| Placement { lane, s, speed: 0.0, route: Route::random(0) };
    // Just past its line, short of its conflicts, so the junction queues it.
    let inside = car.rear_axle + 0.5;
    // On T's green, an oncoming car held just inside the junction, queued, so T
    // waits for it at its waiting point.
    run_until(&mut sim, &|sim| sim.light(t_movement).light == Light::Green);
    let o = sim.insert(car, at_rest(o_lane, inside));
    sim.hold(o);
    // T, coming up to the junction to turn right.
    let t = (0..)
        .find_map(|seed| {
            let at = Placement {
                lane: approach,
                s: approach_length - 40.0,
                speed: 8.0,
                route: Route::random(seed),
            };
            let id = sim.insert(car, at);
            if sim.get(id).unwrap().path().contains(&t_first) {
                return Some(id);
            }
            sim.remove(id);
            None
        })
        .expect("a route turning right");
    run_until(&mut sim, &|sim| {
        sim.get(t).is_some_and(|v| v.lane() == t_first && v.speed() == 0.0)
    });
    // Once T's light is red and X's green, X just inside the junction, as if
    // nothing had held it at its line, queued after T's first stretch. Then
    // the oncoming car goes, and T and X are left waiting for each other.
    run_until(&mut sim, &|sim| {
        sim.light(t_movement).light == Light::Red && sim.light(x_movement).light == Light::Green
    });
    sim.insert(car, at_rest(x_lane, inside));
    for _ in 0..30 {
        sim.step();
    }
    sim.release(o);
    run_until(&mut sim, &|sim| sim.get(o).is_none());
    for _ in 0..(5 * 60) {
        sim.step();
    }
    Frame { state: sim.state().clone(), render, selected: Some(t) }
}

#[cfg(test)]
mod test {
    use super::*;
    use traffic_sim::JunctionId;

    #[test]
    fn the_turner_and_the_car_wait_on_each_other_for_good() {
        let frame = turner_cycle();
        let t = frame.selected.expect("T is selected");
        let mut sim = Simulation::restore(frame.state, NoopSink);
        let queue = |sim: &Simulation| sim.queue(JunctionId(0)).to_vec();
        let x = queue(&sim).iter().map(|e| e.id).find(|&id| id != t).expect("X is queued");
        // T is queued for its first stretch only, X after it, right through.
        let entries = queue(&sim);
        assert_eq!(entries.len(), 2, "{entries:?}");
        assert!(
            entries[0].id == t && entries[0].until.is_some(),
            "{entries:?}"
        );
        assert!(
            entries[1].id == x && entries[1].until.is_none(),
            "{entries:?}"
        );
        // And neither ever moves again.
        for _ in 0..(60 * 60) {
            sim.step();
            for id in [t, x] {
                assert_eq!(
                    sim.get(id).unwrap().speed(),
                    0.0,
                    "{id:?} moved at {:.1} s",
                    sim.time()
                );
            }
        }
    }
}
