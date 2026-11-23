//! A keep-clear zone on a signalised crossroads' approach: the queue at a red
//! light backs past it and leaves it empty.

use traffic_network::{MarkingStyle, compile, scenarios};
use traffic_sim::{BarrierKind, LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};

mod common;

const LIMIT: f64 = 50.0 / 3.6;

#[test]
fn a_keep_clear_zone_is_a_barrier_on_each_lane_across_it_and_is_painted() {
    let compiled = compile(&scenarios::signalised_crossroads_keep_clear(2, LIMIT)).unwrap();
    let west = &compiled.road_lanes[2];
    let [from, to] = scenarios::KEEP_CLEAR_BACK;
    // The two lanes coming in, and not the two going out.
    for (i, &lane) in west.iter().enumerate() {
        let lane = compiled.sim.lane(lane);
        let zones: Vec<_> = lane
            .barriers()
            .iter()
            .filter_map(|b| match b.kind {
                BarrierKind::KeepClear { length } => Some((b.s, length)),
                _ => None,
            })
            .collect();
        if i >= 2 {
            assert!(zones.is_empty(), "lane {i} going out has {zones:?}");
            continue;
        }
        let length = lane.geometry().length();
        assert_eq!(zones.len(), 1, "lane {i} has {zones:?}");
        let (s, zone) = zones[0];
        // Back from the road's end, less how far its stop line is set back.
        let set_back = scenarios::signalised_crossroads(2, LIMIT).roads[2].length()
            - compiled.sim.lane(west[i]).geometry().length();
        assert!((s - (length + set_back - from)).abs() < 0.01, "at {s:.3}");
        assert!((zone - (from - to)).abs() < 0.01, "{zone:.3} long");
    }
    let painted = compiled.render.markings.iter().filter(|m| m.style == MarkingStyle::KeepClear);
    assert!(painted.count() > 2, "no outline and hatches");
}

#[test]
fn a_crossroads_with_a_keep_clear_zone_flooded_with_traffic_has_no_collisions() {
    common::flood(
        &scenarios::signalised_crossroads_keep_clear(2, LIMIT),
        150.0,
    );
}

/// Heavy traffic from the west, so its queue at each red light backs past the
/// zone: nobody comes to rest in it, but for a vehicle that changed lanes into
/// it from beside.
#[test]
fn the_queue_at_a_red_light_leaves_a_keep_clear_zone_empty() {
    let compiled = compile(&scenarios::signalised_crossroads_keep_clear(2, LIMIT)).unwrap();
    let west: Vec<LaneId> = compiled.road_lanes[2][..2].to_vec();
    let zone: Vec<(LaneId, f64, f64)> = west
        .iter()
        .map(|&l| {
            let b = compiled.sim.lane(l).barriers()[0];
            let BarrierKind::KeepClear { length } = b.kind else {
                panic!("{l:?} has no zone");
            };
            (l, b.s, b.s + length)
        })
        .collect();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
    let car = VehicleSpec::passenger_car();
    let (mut queued_past, mut in_zone) = (0, Vec::new());
    for step in 0..common::steps(300.0) {
        // A car every second from the west, each lane in turn.
        if step % common::steps(1.0) == 0 {
            let lane = west[(step / common::steps(1.0)) as usize % 2];
            let at = Placement { lane, s: 5.0, speed: LIMIT, route: Route::random(step) };
            let _ = sim.try_insert(car, at);
        }
        sim.step();
        if step % common::steps(0.1) != 0 {
            continue;
        }
        for (id, v) in sim.vehicles() {
            let Some(&(_, start, end)) = zone.iter().find(|z| z.0 == v.lane()) else {
                continue;
            };
            let rear = v.lane_pos().s - v.spec().rear_axle;
            let front = rear + v.spec().length;
            if v.speed() > 0.0 {
                continue;
            }
            if rear > end {
                queued_past += 1;
            } else if front > start && v.lane_change().is_none() {
                in_zone.push((id, sim.time(), rear - start, front - start));
            }
        }
    }
    assert!(queued_past > 100, "the queue never backed past the zone");
    assert!(
        in_zone.len() <= queued_past / 100,
        "{} at rest in the zone, of {queued_past} beyond it: {:?}",
        in_zone.len(),
        &in_zone[..in_zone.len().min(5)]
    );
}
