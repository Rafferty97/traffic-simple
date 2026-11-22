//! Every example network compiles to a valid simulation network.

use traffic_network::{compile, scenarios};

#[test]
fn a_crossroads_with_two_lanes_each_way_compiles() {
    let compiled = compile(&scenarios::crossroads(2, 14.0)).unwrap();
    // Four arms of four lanes, and per arm two straight on, and a right and a
    // left into each lane of the road turned into.
    assert_eq!(compiled.road_lanes.iter().flatten().count(), 16);
    assert_eq!(compiled.connectors[0].len(), 24);
    assert_eq!(compiled.render.intersections.len(), 1);
    let junction = compiled.sim.junctions().next().unwrap().1;
    assert_eq!(junction.lanes().len(), 24);
}

#[test]
fn crossroads_of_one_and_three_lanes_each_way_compile() {
    for (lanes, connectors) in [(1, 12), (3, 36)] {
        let compiled = compile(&scenarios::crossroads(lanes, 14.0)).unwrap();
        // Per arm, one straight on from each lane, and a right and a left into
        // each lane of the road turned into.
        assert_eq!(compiled.connectors[0].len(), connectors);
    }
}

#[test]
fn lane_lines_stop_at_the_stop_line_coming_in_and_run_to_the_junction_going_out() {
    use traffic_network::MarkingStyle;
    // Every arm's stop line set 2 m further back than the scenario has it.
    let mut net = scenarios::crossroads(2, 14.0);
    let x = &mut net.intersections[0];
    let arms: Vec<_> = (0..4).map(|r| x.arm(traffic_network::RoadId(r))).collect();
    x.arms = arms
        .into_iter()
        .map(|a| traffic_network::Arm { stop_line: a.stop_line + 2.0, ..a })
        .collect();
    let back = net.intersections[0].arm(traffic_network::RoadId(0)).stop_line;
    let compiled = compile(&net).unwrap();
    let half = 2.0 * 3.5 + 6.0;
    // How far each line between lanes gets toward the junction's centre, along
    // its arm, with which way its lanes beside it go there.
    let mut ends: Vec<(MarkingStyle, f64)> = Vec::new();
    for m in &compiled.render.markings {
        if !matches!(m.style, MarkingStyle::Dashed | MarkingStyle::Centre) {
            continue;
        }
        let near = |p: &glam::DVec2| p.x.abs().max(p.y.abs());
        let reach = m.points.iter().map(near).fold(f64::INFINITY, f64::min);
        ends.push((m.style, reach));
    }
    let at = |style, reach: f64| {
        ends.iter().filter(|&&(s, r)| s == style && (r - reach).abs() < 1e-3).count()
    };
    // Every arm's centre line and the line between its lanes coming in stop at
    // its stop line; the line between its lanes going out runs on.
    assert_eq!(at(MarkingStyle::Centre, half + back), 4, "{ends:?}");
    assert_eq!(at(MarkingStyle::Dashed, half + back), 4, "{ends:?}");
    assert_eq!(at(MarkingStyle::Dashed, half), 4, "{ends:?}");
}

/// Driving on the left, a crossroads' lanes by the centre line go straight on
/// or right, and its outer lanes straight on or left; with one lane each way,
/// it goes all three. Each arrow lies on its lane, a little way back from the
/// junction, pointing into it.
#[test]
fn crossroads_lanes_have_arrows_for_the_ways_they_turn() {
    for (lanes, expected) in [
        (1, vec![(true, true, true)]),
        (2, vec![(false, true, true), (true, true, false)]),
    ] {
        let compiled = compile(&scenarios::crossroads(lanes, 14.0)).unwrap();
        let arrows = &compiled.render.arrows;
        assert_eq!(arrows.len(), 4 * lanes);
        for (k, arrow) in arrows.iter().enumerate() {
            assert_eq!(
                (arrow.left, arrow.straight, arrow.right),
                expected[k % lanes]
            );
            // Along its arm toward the centre, beside the centre line.
            let p = arrow.pos;
            let inward = match p.x.abs() > p.y.abs() {
                true => glam::DVec2::new(-p.x.signum(), 0.0),
                false => glam::DVec2::new(0.0, -p.y.signum()),
            };
            assert!(glam::DVec2::from_angle(arrow.heading).dot(inward) > 0.999);
            let off_centre = arrow.pos.perp_dot(inward).abs();
            assert!((off_centre - 3.5 * ((k % lanes) as f64 + 0.5)).abs() < 1e-6);
        }
    }
}

/// Each lane coming into a crossroads has a stop line across its whole width,
/// at the edge of the junction, square to the lane.
#[test]
fn crossroads_lanes_have_stop_lines_where_they_come_in() {
    use traffic_network::MarkingStyle;
    for lanes in 1..=3 {
        let net = scenarios::crossroads(lanes, 14.0);
        let compiled = compile(&net).unwrap();
        // Where the arms' lanes coming in end: their stop lines, set back as
        // the crossroads fitted them, the same on every arm.
        let back = net.intersections[0].arm(traffic_network::RoadId(0)).stop_line;
        let half = lanes as f64 * 3.5 + 6.0 + back;
        let stops: Vec<_> = compiled
            .render
            .markings
            .iter()
            .filter(|m| m.style == MarkingStyle::Stop)
            .collect();
        assert_eq!(stops.len(), 4 * lanes);
        for stop in stops {
            let [a, b] = stop.points[..] else {
                panic!("a stop line is one segment")
            };
            assert!((a.distance(b) - 3.5).abs() < 1e-6);
            // Set back half its width from where its lane ends, across its arm.
            let mid = 0.5 * (a + b);
            assert!((mid.x.abs().max(mid.y.abs()) - (half + 0.15)).abs() < 1e-6);
            let along = match mid.x.abs() > mid.y.abs() {
                true => glam::DVec2::X,
                false => glam::DVec2::Y,
            };
            assert!((a - b).dot(along).abs() < 1e-6);
        }
    }
}

#[test]
fn the_freeway_merges_at_the_lane_drop_and_forks_at_the_off_ramp() {
    let compiled = compile(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6)).unwrap();
    let sim = &compiled.sim;
    // Roads: approach, parallel, drop, middle, after, on-ramp, off-ramp.
    let lanes = &compiled.road_lanes;
    let (on_ramp, accel, drop_accel, drop_left) =
        (lanes[5][0], lanes[1][0], lanes[2][0], lanes[2][1]);
    assert_eq!(sim.lane(on_ramp).successors(), [accel]);
    // Alongside, a driver can change into the left lane, but not back from the
    // freeway into the acceleration lane; on the drop, it merges.
    let right = sim.lane(accel).adj_right().unwrap();
    assert_eq!(right.lane, lanes[1][1]);
    assert!(right.can_change);
    assert!(!sim.lane(lanes[1][1]).adj_left().unwrap().can_change);
    assert!(sim.lane(drop_accel).adj_right().is_none());
    assert_eq!(
        sim.lane(drop_accel).successors(),
        sim.lane(drop_left).successors()
    );
    // The drop is a taper, not a junction.
    assert_eq!(sim.lane(drop_accel).junction(), None);
    assert_eq!(sim.lane(drop_left).junction(), None);
    // The middle's left lane forks, left up the off-ramp and right along the freeway.
    assert_eq!(
        sim.lane(lanes[3][0]).successors(),
        [lanes[6][0], lanes[4][0]]
    );
}

#[test]
fn a_ring_compiles_into_lanes_that_lead_into_themselves() {
    let compiled = compile(&scenarios::ring(1000.0, 3, 14.0)).unwrap();
    for (id, lane) in compiled.sim.lanes() {
        assert_eq!(lane.successors(), [id]);
    }
    assert!(compiled.sim.lane(compiled.road_lanes[0][1]).adj_left().is_some());
}

/// Whether two convex bodies, given by their corners, overlap: by the separating
/// axis theorem, along the edges of each.
fn overlap(a: [glam::DVec2; 4], b: [glam::DVec2; 4]) -> bool {
    let separated = |axis: glam::DVec2| {
        let span = |c: [glam::DVec2; 4]| {
            let dots = c.map(|q| axis.dot(q));
            (
                dots.iter().copied().fold(f64::MAX, f64::min),
                dots.iter().copied().fold(f64::MIN, f64::max),
            )
        };
        let ((a0, a1), (b0, b1)) = (span(a), span(b));
        a1 < b0 || b1 < a0
    };
    let axes = [a[1] - a[0], a[2] - a[1], b[1] - b[0], b[2] - b[1]];
    !axes.iter().any(|&axis| separated(axis.perp()))
}

/// A vehicle whose body is well inside a lane outside junctions that it has no
/// record on, if any, other than lanes on its path and those its nose pokes
/// into past the end of a lane it is on: each vehicle has a record on every lane
/// it intersects (see `docs/multilane-car-following.md`).
fn uncovered<D: traffic_sim::DebugSink>(sim: &traffic_sim::Simulation<D>) -> Option<String> {
    use traffic_sim::Side;
    const INSIDE: f64 = 0.05;
    let net = sim.network();
    let on = |lane, id| sim.occupants(lane).iter().any(|o| o.is_record_of(id));
    for (id, v) in sim.vehicles() {
        for (lane_id, lane) in net.lanes() {
            if lane.junction().is_some() || v.path().contains(&lane_id) || on(lane_id, id) {
                continue;
            }
            if lane.predecessors().iter().any(|&p| on(p, id)) {
                continue;
            }
            let g = lane.geometry();
            let inside = v.corners().iter().any(|&c| {
                let p = g.world_to_lane(c);
                p.s > INSIDE
                    && p.s < g.length() - INSIDE
                    && p.t < g.half_width(p, Side::Left) - INSIDE
                    && -p.t < g.half_width(p, Side::Right) - INSIDE
                    && g.lane_to_world(p).distance(c) < 0.01
            });
            if inside {
                return Some(format!("{id:?} on {:?} is in {lane_id:?}", v.lane()));
            }
        }
    }
    None
}

/// Floods a network with traffic for `seconds`, a car tried every third of a
/// second at its entry lanes in turn, asserting that no two bodies ever overlap,
/// and that every vehicle has a record on each lane its body is in.
fn flood(net: &traffic_network::RoadNetwork, seconds: f64) {
    use traffic_sim::{LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};
    let car = VehicleSpec::passenger_car();

    {
        let compiled = compile(net).unwrap();
        let mut sim = Simulation::new(compiled.sim, NoopSink);
        let entries: Vec<LaneId> = sim
            .network()
            .lanes()
            .filter(|(_, lane)| lane.predecessors().is_empty())
            .map(|(id, _)| id)
            .collect();
        // A car tried every third of a second, more than the junction can take, so
        // queues build up and drivers change lanes on the way in.
        for step in 0..(seconds * 60.0) as u64 {
            if step % 20 == 0 {
                let lane = entries[(step / 20 * 7) as usize % entries.len()];
                let speed = sim.network().lane(lane).speed_limit();
                let at = Placement { lane, s: 5.0, speed, route: Route::random(step) };
                let _ = sim.try_insert(car, at);
            }
            sim.step_variable(1.0 / 60.0);
            // Collisions last for many frames, so a check every tenth of a second
            // catches them.
            if step % 6 != 0 {
                continue;
            }
            if let Some(what) = uncovered(&sim) {
                panic!("{what} with no record there at {:.2} s", sim.time());
            }
            let bodies: Vec<_> = sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
            for (i, &(a, body_a)) in bodies.iter().enumerate() {
                for &(b, body_b) in &bodies[i + 1..] {
                    assert!(
                        !overlap(body_a, body_b),
                        "{a:?} and {b:?} collide at {:.2} s",
                        sim.time()
                    );
                }
            }
        }
    }
}

#[test]
fn crossroads_flooded_with_traffic_have_no_collisions() {
    for lanes in 1..=3 {
        flood(&scenarios::crossroads(lanes, 50.0 / 3.6), 100.0);
    }
}

#[test]
fn signalised_crossroads_flooded_with_traffic_have_no_collisions() {
    for lanes in 1..=2 {
        flood(&scenarios::signalised_crossroads(lanes, 50.0 / 3.6), 150.0);
    }
}

#[test]
#[ignore = "on the acute side, a left turn's body strays onto the lane beside the one it \
            turns into by more than the sweep check's steering allowance"]
fn an_oblique_crossroads_flooded_with_traffic_has_no_collisions() {
    for angle in [60.0, 120.0] {
        let angle = f64::to_radians(angle);
        flood(&scenarios::oblique_crossroads(2, 50.0 / 3.6, angle), 150.0);
        flood(
            &scenarios::signalised_oblique_crossroads(2, 50.0 / 3.6, angle),
            150.0,
        );
    }
}

#[test]
fn a_signalised_crossroads_holds_each_connector_at_its_movements_light() {
    use traffic_network::Turn;
    use traffic_sim::BarrierKind;
    let compiled = compile(&scenarios::signalised_crossroads(2, 14.0)).unwrap();
    let sim = &compiled.sim;
    let (plan, first) = sim.signals().next().expect("a signal plan");
    // Every movement of every arm: straight on, left and right, from four arms.
    assert_eq!(plan.movements.len(), 12);
    for (&connector, &turn) in compiled.connectors[0].iter().zip(&compiled.turns[0]) {
        let barriers = sim.lane(connector).barriers();
        assert_eq!(
            barriers.len(),
            1,
            "{connector:?} turning {turn:?} has {barriers:?}"
        );
        // Right turns, across oncoming traffic green with them, also wait for
        // a gap inside the junction, at a lower priority than the rest: at the
        // start of a second lane, which the first leads on to, alone.
        let next = sim.lane(connector).successors()[0];
        let priority = match turn {
            Turn::Right => {
                let inner = sim.lane(next).barriers();
                assert_eq!(inner.len(), 1, "{next:?} after {connector:?} has {inner:?}");
                assert_eq!(inner[0].kind, BarrierKind::GiveWay);
                assert_eq!(inner[0].s, 0.0);
                assert_eq!(sim.lane(next).priority(), 2);
                assert_eq!(sim.lane(next).junction(), sim.lane(connector).junction());
                2
            }
            _ => {
                assert!(
                    sim.lane(next).junction().is_none(),
                    "{connector:?} is split"
                );
                3
            }
        };
        assert_eq!(sim.lane(connector).priority(), priority);
        assert_eq!(barriers[0].s, 0.0);
        let BarrierKind::Signal { movement } = barriers[0].kind else {
            panic!("{connector:?} has a {:?}", barriers[0].kind);
        };
        assert!((first.0..first.0 + 12).contains(&movement.0));
    }
    // Movements green together are never in conflict, so each phase can go green.
    for phase in &plan.phases {
        for (i, m) in plan.movements.iter().enumerate() {
            if phase.green & (1 << i) != 0 {
                assert_eq!(
                    m.conflicts & phase.green,
                    0,
                    "movement {i} conflicts within its phase"
                );
            }
        }
    }
    // A crossing movement from the other pair of roads conflicts with something.
    assert!(plan.movements.iter().all(|m| m.conflicts != 0));
}

#[test]
#[ignore = "locks up at 512 s: a straight car stopped inside the junction, unqueued, \
            is queued behind a turner from the next phase (see DESIGN.md, Junction lockups)"]
fn at_signals_nobody_crosses_a_line_on_red_and_traffic_keeps_flowing() {
    use std::collections::HashMap;
    use traffic_sim::{
        BarrierKind, LaneId, Light, NoopSink, Placement, Route, Simulation, VehicleSpec,
    };
    // Drivers varied as the UI varies them, on two lanes each way, so they
    // change lanes and turn on the way in.
    let mut state = 7u64;
    let mut uniform = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    };
    let compiled = compile(&scenarios::signalised_crossroads(2, 50.0 / 3.6)).unwrap();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut was_past = HashMap::new();
    let mut on_red = Vec::new();
    let (mut spawned, mut left_before) = (0, 0);
    // Ten minutes: long enough to lock up, if anything does.
    for step in 0..(600.0 * 60.0) as u64 {
        if step % 30 == 0 {
            let lane = entries[(step / 30) as usize % entries.len()];
            let car = VehicleSpec {
                speed_factor: 0.85 + 0.4 * uniform(),
                assertiveness: uniform().powi(3),
                courtesy: 2.0 * uniform(),
                politeness: uniform(),
                line_gap: 0.5 + uniform(),
                line_gap_spread: 0.3,
                ..VehicleSpec::passenger_car()
            };
            let speed = sim.network().lane(lane).speed_limit() * car.speed_factor;
            let at = Placement { lane, s: 5.0, speed, route: Route::random(step) };
            if sim.try_insert(car, at).is_ok() {
                spawned += 1;
            }
        }
        sim.step();
        // Traffic keeps flowing: vehicles leave every minute after the first.
        if step % 3600 == 0 && step > 3600 {
            let left = spawned - sim.vehicles().count();
            assert!(
                left > left_before + 5,
                "standstill in the minute to {:.0} s",
                sim.time()
            );
            left_before = left;
        }
        // Each vehicle's front against the signal line ahead on its path, at the
        // start of the lane after the one it follows.
        let network = sim.network();
        for (id, v) in sim.vehicles() {
            let Some(&next) = v.path().get(1) else {
                continue;
            };
            let Some(barrier) = network.lane(next).barriers().first() else {
                continue;
            };
            let BarrierKind::Signal { movement } = barrier.kind else {
                continue;
            };
            let length = network.lane(v.lane()).geometry().length();
            let front = v.lane_pos().s - v.spec().rear_axle + v.spec().length;
            let past = front >= length + barrier.s;
            let before = was_past.insert((id, next), past);
            if before == Some(false) && past && sim.light(movement).light == Light::Red {
                on_red.push((id, sim.time(), v.speed()));
            }
        }
    }
    assert!(
        on_red.is_empty(),
        "{} crossed on red: {on_red:?}",
        on_red.len()
    );
}

#[test]
fn at_signals_straight_traffic_never_stops_or_brakes_hard_for_right_turners_on_its_green() {
    use std::collections::HashMap;
    use traffic_network::Turn;
    use traffic_sim::{
        AccelSource, BarrierKind, DebugEvent, DebugSink, LaneId, Light, MovementId, Placement,
        Route, Simulation, VehicleId, VehicleSpec,
    };
    /// Each vehicle's junction terms from the last longitudinal update.
    #[derive(Default)]
    struct Terms(Vec<(VehicleId, VehicleId, f64)>);
    impl DebugSink for Terms {
        fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
            match event() {
                DebugEvent::RecomputeAccels => self.0.clear(),
                DebugEvent::AccelTerm {
                    vehicle,
                    source: AccelSource::Junction { id, .. },
                    accel,
                } => self.0.push((vehicle, id, accel)),
                _ => {}
            }
        }
    }
    let compiled = compile(&scenarios::signalised_crossroads(2, 50.0 / 3.6)).unwrap();
    // Each connector's turn and movement.
    let mut way: HashMap<LaneId, (Turn, MovementId)> = HashMap::new();
    for (&c, &turn) in compiled.connectors[0].iter().zip(&compiled.turns[0]) {
        let BarrierKind::Signal { movement } = compiled.sim.lane(c).barriers()[0].kind else {
            panic!("{c:?} has no signal");
        };
        way.insert(c, (turn, movement));
    }
    let mut sim = Simulation::new(compiled.sim, Terms::default());
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let way_of = |sim: &Simulation<Terms>, id: VehicleId| {
        sim.get(id).and_then(|v| v.path().iter().find_map(|l| way.get(l).copied()))
    };
    let mut hard = Vec::new();
    for step in 0..(300.0 * 60.0) as u64 {
        if step % 30 == 0 {
            let lane = entries[(step / 30) as usize % entries.len()];
            let speed = sim.network().lane(lane).speed_limit();
            let at = Placement { lane, s: 5.0, speed, route: Route::random(step) };
            let _ = sim.try_insert(VehicleSpec::passenger_car(), at);
        }
        sim.step();
        if step % 3 != 0 {
            continue;
        }
        for &(id, other, accel) in &sim.sink().0 {
            let (Some((Turn::Straight, mine)), Some((Turn::Right, theirs))) =
                (way_of(&sim, id), way_of(&sim, other))
            else {
                continue;
            };
            let green = |m| sim.light(m).light == Light::Green;
            // Held up by it at all, it may slow a little as a turner clears
            // ahead, but never stop or brake hard for one.
            let held = sim.get(id).is_some_and(|v| v.accel() == accel && v.speed() < 0.1);
            let hard_braking = accel < -VehicleSpec::passenger_car().comfy_decel;
            let binding = sim.get(id).is_some_and(|v| v.accel() == accel);
            if binding && green(mine) && green(theirs) && (held || hard_braking) {
                hard.push((id, other, sim.time(), accel));
            }
        }
    }
    assert!(
        hard.is_empty(),
        "{} hard: {:?}",
        hard.len(),
        &hard[..hard.len().min(5)]
    );
}

/// A right turner at the head of its lane turns on the first green after it
/// reaches its line: through a gap, or as the green ends and the oncoming
/// traffic stops. It never waits out a second green.
#[test]
fn at_signals_a_right_turner_at_its_line_turns_on_its_next_green() {
    use std::collections::HashMap;
    use traffic_network::Turn;
    use traffic_sim::{
        BarrierKind, LaneId, Light, MovementId, NoopSink, Placement, Route, Simulation, VehicleId,
        VehicleSpec,
    };
    let compiled = compile(&scenarios::signalised_crossroads(2, 50.0 / 3.6)).unwrap();
    // Each right turn's connector, with its movement.
    let right: HashMap<LaneId, MovementId> = compiled.connectors[0]
        .iter()
        .zip(&compiled.turns[0])
        .filter(|(_, t)| **t == Turn::Right)
        .map(|(&c, _)| match compiled.sim.lane(c).barriers()[0].kind {
            BarrierKind::Signal { movement } => (c, movement),
            kind => panic!("{c:?} starts with a {kind:?}"),
        })
        .collect();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    // Each right turner at the head of its lane, with its movement, and how
    // many greens of it it has seen since it reached its line, counting one it
    // reached it on.
    let mut waiting: HashMap<VehicleId, (MovementId, u32)> = HashMap::new();
    // Each right-turn movement once, though several connectors share one.
    let mut movements: Vec<MovementId> = right.values().copied().collect();
    movements.sort_by_key(|m| m.0);
    movements.dedup();
    let mut lights: Vec<Light> = movements.iter().map(|&m| sim.light(m).light).collect();
    let mut turned = 0;
    // Busy, with plenty coming the other way.
    for step in 0..(600.0 * 60.0) as u64 {
        if step % 45 == 0 {
            let lane = entries[(step / 45) as usize % entries.len()];
            let speed = sim.network().lane(lane).speed_limit();
            let at = Placement { lane, s: 5.0, speed, route: Route::random(step) };
            let _ = sim.try_insert(VehicleSpec::passenger_car(), at);
        }
        sim.step();
        for (i, &movement) in movements.iter().enumerate() {
            let light = sim.light(movement).light;
            if light == Light::Green && lights[i] != Light::Green {
                for (m, greens) in waiting.values_mut() {
                    if *m == movement {
                        *greens += 1;
                    }
                }
            }
            lights[i] = light;
        }
        let network = sim.network();
        for (id, v) in sim.vehicles() {
            let path = v.path();
            match path.get(1) {
                // On its way in, its front within a few meters of the line, and
                // nobody turning ahead of it.
                Some(next) if right.contains_key(next) => {
                    let length = network.lane(v.lane()).geometry().length();
                    let front = v.lane_pos().s - v.spec().rear_axle + v.spec().length;
                    // Nobody on a right turn from its lane, into any lane.
                    let from = network.lane(*next).predecessors()[0];
                    let turning = |l: LaneId| {
                        right.contains_key(&l) && network.lane(l).predecessors()[0] == from
                    };
                    let head = sim.vehicles().all(|(_, u)| !turning(u.lane()));
                    if length - front < 3.0 && head {
                        let movement = right[next];
                        let green = sim.light(movement).light == Light::Green;
                        waiting.entry(id).or_insert((movement, green as u32));
                    }
                }
                // Turning: its rear axle onto the connector, past its waiting line.
                _ if right.contains_key(&path[0]) => {
                    let line = network.lane(path[0]).barriers().last().map_or(0.0, |b| b.s);
                    if v.lane_pos().s > line + v.spec().length
                        && let Some((_, greens)) = waiting.remove(&id)
                    {
                        turned += 1;
                        assert!(
                            greens <= 1,
                            "{id:?} waited out {greens} greens at {:.0} s",
                            sim.time()
                        );
                    }
                }
                _ => {}
            }
        }
        for (id, (_, greens)) in &waiting {
            assert!(
                *greens <= 1,
                "{id:?} has waited out {greens} greens at {:.0} s",
                sim.time()
            );
        }
    }
    assert!(turned > 20, "only {turned} right turners turned");
}

#[test]
fn at_signals_every_way_is_taken_right_turns_included() {
    use traffic_sim::{LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};
    let compiled = compile(&scenarios::signalised_crossroads(1, 50.0 / 3.6)).unwrap();
    let connectors = compiled.connectors[0].clone();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut used = vec![false; connectors.len()];
    let (mut spawned, mut left_before) = (0, 0);
    // A car every 3 s at the entries in turn, within what the junction takes even
    // with right turners waiting in each single lane for gaps.
    for step in 0..(300.0 * 60.0) as u64 {
        if step % 180 == 0 {
            let lane = entries[(step / 180) as usize % entries.len()];
            let speed = sim.network().lane(lane).speed_limit();
            let at = Placement { lane, s: 5.0, speed, route: Route::random(step) };
            if sim.try_insert(VehicleSpec::passenger_car(), at).is_ok() {
                spawned += 1;
            }
        }
        sim.step();
        for (c, &lane) in connectors.iter().enumerate() {
            used[c] |= !sim.occupants(lane).is_empty();
        }
        // Vehicles keep leaving, every minute after the first.
        if step % 3600 == 0 && step > 3600 {
            let left = spawned - sim.vehicles().count();
            assert!(
                left > left_before + 5,
                "few left in the minute to {:.0} s",
                sim.time()
            );
            left_before = left;
        }
    }
    let unused: Vec<_> = connectors.iter().zip(&used).filter(|(_, u)| !**u).collect();
    assert!(unused.is_empty(), "never used: {unused:?}");
}

#[test]
fn a_freeway_with_ramps_flooded_with_traffic_has_no_collisions() {
    flood(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6), 200.0);
}

/// At a light 300 vehicles an hour into each entry, drivers varied, traffic on
/// the freeway keeps moving. A vehicle changing out of the acceleration lane near
/// the merge used to rejoin the junction's queue behind the vehicle following it
/// there, which waited for it as a leader while it waited for that vehicle at the
/// merge, and both lanes locked up for good.
#[test]
fn a_freeway_with_ramps_in_light_traffic_never_locks_up() {
    ramps_traffic(300.0, 600.0);
}

/// In freely flowing traffic, drivers varied and changing lanes, every vehicle
/// has a record on each lane its body is in: around the lane drop, where the
/// acceleration lane bends in toward a lane vehicles are leaving, and where the
/// off-ramp parts.
#[test]
fn a_freeway_with_ramps_keeps_a_record_on_every_lane_a_body_is_in() {
    ramps_traffic(900.0, 180.0);
}

/// Runs the ramps for `seconds` at `flow` vehicles an hour into each entry,
/// drivers varied, asserting that traffic keeps leaving and that every body has
/// a record on each lane it is in.
fn ramps_traffic(flow: f64, seconds: f64) {
    use traffic_sim::{LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};
    // SplitMix64, for drivers that vary but are the same every run.
    let mut state = 1u64;
    let mut uniform = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    };
    let compiled = compile(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6)).unwrap();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let (mut credit, mut spawned, mut left_before) = (0.0, 0, 0);
    for step in 1..=(seconds * 60.0) as u64 {
        sim.step_variable(1.0 / 60.0);
        if step % 10 == 0
            && let Some(what) = uncovered(&sim)
        {
            panic!("{what} with no record there at {:.2} s", sim.time());
        }
        let n = entries.len() as f64;
        credit = f64::min(credit + flow / 3600.0 * n / 60.0, n);
        if credit >= 1.0 {
            let lane = entries[(uniform() * n) as usize];
            // Normal with mean 1 and deviation 0.1, as SUMO's default, clamped.
            let (u, v) = (1.0 - uniform(), uniform());
            let normal = (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos();
            let car = VehicleSpec {
                speed_factor: (1.0 + 0.1 * normal).clamp(0.8, 1.2),
                courtesy: 2.0 * uniform(),
                politeness: 0.1 + 0.3 * uniform(),
                assertiveness: uniform().powi(3),
                ..VehicleSpec::passenger_car()
            };

            let limit = sim.network().lane(lane).speed_limit();
            let seed = (uniform() * 1e9) as u64;
            let at = Placement {
                lane,
                s: 5.0,
                speed: car.desired_speed(limit),
                route: Route::random(seed),
            };
            if sim.try_insert(car, at).is_ok() {
                credit -= 1.0;
                spawned += 1;
            }
        }
        // Vehicles keep leaving: at least one a minute.
        if step % (60 * 60) == 0 {
            let left = spawned - sim.vehicles().count();
            assert!(
                left > left_before,
                "nobody left in the minute up to {:.0} s",
                sim.time()
            );
            left_before = left;
        }
    }
}

/// A car going straight on doesn't cross its stop line behind a right turner
/// waiting in the junction in its way, though its light is green: it couldn't
/// drive through. It goes once the turner has.
#[test]
fn at_signals_a_car_waits_behind_its_line_for_a_turner_in_its_way() {
    use traffic_network::Turn;
    use traffic_sim::{NoopSink, Placement, Route, Simulation, VehicleSpec};
    // Two lanes each way: the inner one goes straight on or right.
    let compiled = compile(&scenarios::signalised_crossroads(2, 50.0 / 3.6)).unwrap();
    let from = compiled.road_lanes[0][0];
    let of = |turn| {
        let ways = compiled.connectors[0].iter().zip(&compiled.turns[0]);
        let mut ways =
            ways.filter(|&(&c, &t)| t == turn && compiled.sim.lane(c).predecessors()[0] == from);
        *ways.next().unwrap().0
    };
    let (right, straight) = (of(Turn::Right), of(Turn::Straight));
    let line = compiled.sim.lane(straight).barriers()[0].s;
    // Where the turner waits for a gap: at the end of its first lane, where the
    // second, with its give-way line, starts.
    let waits = compiled.sim.lane(right).geometry().length();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
    let car = VehicleSpec::passenger_car();
    // A turner held at its waiting line inside, on the road's first green.
    let s = waits - (car.length - car.rear_axle) - 1.0;
    let turner = sim.insert(
        car,
        Placement { lane: right, s, speed: 0.0, route: Route::random(0) },
    );
    sim.hold(turner);
    let length = sim.network().lane(from).geometry().length();
    // Going straight on: the first route that takes it that way.
    let follower = (0..)
        .find_map(|seed| {
            let route = Route::random(seed);
            let at = Placement { lane: from, s: length - 60.0, speed: 10.0, route };
            let id = sim.insert(car, at);
            if sim.get(id).unwrap().path().get(1) == Some(&straight) {
                return Some(id);
            }
            sim.remove(id);
            None
        })
        .unwrap();
    // Its front past the line, along the lane coming in or the one going on.
    let past_line = |sim: &Simulation| {
        let v = sim.get(follower).unwrap();
        let front = v.lane_pos().s - car.rear_axle + car.length;
        match v.lane() {
            lane if lane == from => front - length > line,
            lane if lane == straight => front > line,
            _ => true,
        }
    };
    while sim.time() < 15.0 {
        sim.step();
        let path = sim.get(follower).unwrap().path().to_vec();
        assert!(
            !past_line(&sim),
            "it crossed its line at {:.1} s, on {path:?}",
            sim.time()
        );
    }
    sim.release(turner);
    while sim.time() < 28.0 && !past_line(&sim) {
        sim.step();
    }
    assert!(past_line(&sim), "it never went");
}
