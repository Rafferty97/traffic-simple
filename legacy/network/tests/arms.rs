//! How roads meet intersections: stop lines set back and skewed, lanes going
//! out that start further on, and turning vehicles' sweeps kept off road lanes.

use glam::DVec2;
use traffic_network::{
    Arm, Compiled, LaneRef, MarkingStyle, RoadId, RoadNetwork, compile, fit_arms, scenarios,
};
use traffic_sim::{BarrierKind, LaneId};

/// The crossroads scenarios' roads all meet the intersection at their ends:
/// lanes `0..lanes` coming in, `lanes..2 * lanes` going out.
fn lane(c: &Compiled, road: usize, lane: usize) -> LaneId {
    c.road_lanes[road][lane]
}

fn length(c: &Compiled, id: LaneId) -> f64 {
    c.sim.lane(id).geometry().length()
}

fn points(c: &Compiled, id: LaneId) -> Vec<DVec2> {
    c.sim.lane(id).geometry().centerline().collect()
}

/// The connectors from road lane `from`.
fn from(c: &Compiled, from: LaneId) -> Vec<LaneId> {
    c.connectors[0]
        .iter()
        .copied()
        .filter(|&id| c.sim.lane(id).predecessors()[0] == from)
        .collect()
}

/// `net` with `road`'s arm, as the scenario fitted it, changed by `change`.
fn with_arm(mut net: RoadNetwork, road: usize, change: impl FnOnce(&mut Arm)) -> RoadNetwork {
    let x = &mut net.intersections[0];
    let mut arm = x.arm(RoadId(road));
    change(&mut arm);
    x.arms.retain(|a| a.road != arm.road);
    x.arms.push(arm);
    net
}

#[test]
fn a_stop_line_set_back_hands_the_stretch_before_it_to_the_connectors() {
    let net = scenarios::signalised_crossroads(1, 50.0 / 3.6);
    let plain = compile(&net).unwrap();
    let set = compile(&with_arm(net, 0, |a| a.stop_line += 5.0)).unwrap();
    let (inbound, outbound) = (lane(&set, 0, 0), lane(&set, 0, 1));
    assert!((length(&plain, inbound) - length(&set, inbound) - 5.0).abs() < 1e-3);
    assert!((length(&plain, outbound) - length(&set, outbound)).abs() < 1e-9);
    let end = *points(&set, inbound).last().unwrap();
    for id in from(&set, inbound) {
        let p = points(&set, id);
        assert_eq!(p[0], end, "{id:?} doesn't start where its lane ends");
        // Its first 5 m run on straight from there, its signal at its start.
        let along = (end - points(&set, inbound)[0]).normalize();
        let straight = p.iter().take_while(|&&q| (q - end).perp_dot(along).abs() < 1e-3);
        let run = straight.last().unwrap().distance(end);
        assert!(
            run >= 5.0 - 1e-3,
            "{id:?} turns within the stretch, {run:.3} m in"
        );
        let barrier = set.sim.lane(id).barriers()[0];
        assert!(matches!(barrier.kind, BarrierKind::Signal { .. }));
        assert_eq!((barrier.s, barrier.line), (0.0, None));
    }
    // Its stop line is painted where it now ends.
    let stop = set.render.markings.iter().filter(|m| m.style == MarkingStyle::Stop);
    let nearest = stop.map(|m| 0.5 * (m.points[0] + m.points[1])).map(|p| p.distance(end));
    let nearest = nearest.fold(f64::INFINITY, f64::min);
    assert!(
        (nearest - 0.15).abs() < 1e-3,
        "the stop line is {nearest:.3} m from its lane's end"
    );
}

#[test]
fn a_skewed_stop_line_holds_every_connector_by_its_line_from_where_it_is_safe() {
    let skew = 20f64.to_radians();
    let net = scenarios::signalised_crossroads(2, 50.0 / 3.6);
    let set = compile(&with_arm(net, 0, |a| a.skew = skew)).unwrap();
    // The lanes coming in each end at their own stop line, apart along the
    // road, side by side only to where the first of them ends.
    let ends: Vec<DVec2> =
        (0..2).map(|i| *points(&set, lane(&set, 0, i)).last().unwrap()).collect();
    let dir = {
        let p = points(&set, lane(&set, 0, 0));
        (p[p.len() - 1] - p[p.len() - 2]).normalize()
    };
    let apart = (ends[1] - ends[0]).dot(dir).abs();
    assert!(apart > 1.0, "they end {apart:.3} m apart");
    let [a, b] = [0, 1].map(|i| lane(&set, 0, i));
    let beside = set.sim.lane(a).adj_left().or(set.sim.lane(a).adj_right()).unwrap();
    assert_eq!(beside.lane, b);
    let length = |l| set.sim.lane(l).geometry().length();
    assert!(
        beside.until < length(a) - 1.0
            || set.sim.lane(b).adj_left().or(set.sim.lane(b).adj_right()).unwrap().until
                < length(b) - 1.0,
        "they are side by side to the end"
    );
    for i in 0..2 {
        for id in from(&set, lane(&set, 0, i)) {
            // Its connector wholly in the junction, kept clear, its barrier,
            // with the line, at its start: its lane ends where the line
            // crosses its centre, so its edges there are either side of it.
            let first = set.sim.lane(id);
            assert!(first.junction().is_some() && first.keep_clear(), "{id:?}");
            let barrier = set.sim.lane(id).barriers()[0];
            assert_eq!(barrier.s, 0.0, "{id:?}");
            let line = barrier.line.expect("a skewed stop line is a line");
            let geometry = set.sim.lane(id).geometry();
            let at = |t: f64| geometry.lane_to_world(geometry.lane_pos(0.0, t));
            assert!(
                line.short_of(at(0.0)).abs() < 1e-3,
                "{id:?}: not at the line"
            );
            let short = [1.75, -1.75].map(|t| line.short_of(at(t)));
            assert!(
                (short[0] + short[1]).abs() < 1e-3 && short[0].abs() > 0.1,
                "{id:?}: its edges are {short:?} short of the line"
            );
        }
    }
    // And its markings run along the line, painted behind it.
    let line = set.sim.lane(from(&set, lane(&set, 0, 0))[0]).barriers()[0].line.unwrap();
    let stop: Vec<_> =
        set.render.markings.iter().filter(|m| m.style == MarkingStyle::Stop).collect();
    let on_line = stop
        .iter()
        .filter(|m| m.points.iter().all(|&p| (line.short_of(p) - 0.15).abs() < 1e-3));
    assert_eq!(on_line.count(), 2, "both lanes' stop lines run along it");
}

#[test]
fn a_run_out_lets_turns_into_far_lanes_drift_across_it_and_others_run_along_it() {
    // A left turn from the west's kerb lane into the far lane of the south
    // road, as well as the nearest.
    let mut net = scenarios::crossroads(2, 50.0 / 3.6);
    let turn = (
        LaneRef { road: RoadId(0), lane: 1 },
        LaneRef { road: RoadId(3), lane: 2 },
    );
    net.intersections[0].connections.push(turn);
    let plain = fit_arms(&mut net).unwrap();
    let net = with_arm(net, 3, |a| a.run_out += 8.0);
    let run_out = net.intersections[0].arm(RoadId(3)).run_out;
    let set = compile(&net).unwrap();
    let (far, near) = (lane(&set, 3, 2), lane(&set, 3, 3));
    for id in [far, near] {
        assert!((length(&plain, id) - length(&set, id) - 8.0).abs() < 1e-3);
    }
    let into = |target| {
        let ids = from(&set, lane(&set, 0, 1));
        ids.into_iter().find(|&id| set.sim.lane(id).successors()[0] == target).unwrap()
    };
    // How far the connector strays from the line its lane runs on, over the
    // stretch it runs out along.
    let strays = |id: LaneId, target: LaneId| {
        let start = points(&set, target);
        let dir = (start[1] - start[0]).normalize();
        let geometry = set.sim.lane(id).geometry();
        let tail = geometry.vertex_s().zip(geometry.centerline());
        let tail = tail.filter(|&(s, _)| s >= geometry.length() - run_out);
        tail.map(|(_, p)| (p - start[0]).perp_dot(dir).abs()).fold(0.0, f64::max)
    };
    assert!(
        strays(into(near), near) < 1e-3,
        "the turn into the nearest lane doesn't run along it"
    );
    assert!(
        strays(into(far), far) > 0.1,
        "the turn into the far lane doesn't drift across"
    );
}
