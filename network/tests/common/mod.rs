//! Checks shared by the network's tests.

use traffic_sim::{ConflictKind, DebugSink, LaneId, Simulation};

/// How far a vehicle's front may be into a conflict it shouldn't have entered,
/// in meters, before it counts: the backstop's margin lets one committing just
/// as another does stop a little way in.
const ENTERED: f64 = 0.5;

/// The first of the junctions' queue invariants that doesn't hold, if any (see
/// `docs/DESIGN.md`, Queue invariants), described:
///
/// 1. Of two queued vehicles following one lane, the one ahead is queued first.
/// 2. No queued vehicle is well into a crossing conflict with one queued above
///    it that hasn't cleared it, unless following it the same general way, as
///    the junction lets it.
/// 3. A vehicle's entries run in the order of its path, none after one that
///    covers it right through.
///
/// The fourth, that the queue closes no cycle, shows as a lockup.
#[allow(dead_code)]
pub fn queue_violation<D: DebugSink>(sim: &Simulation<D>) -> Option<String> {
    let net = sim.network();
    for (junction, _) in net.junctions() {
        let queue = sim.queue(junction);
        let place = |id| queue.iter().position(|e| e.id == id);
        // 3.
        for (id, v) in sim.vehicles() {
            let at = |l: LaneId| v.path().iter().position(|&p| p == l);
            let entries: Vec<_> = queue.iter().filter(|e| e.id == id).collect();
            for pair in entries.windows(2) {
                match (pair[0].until(net, junction), pair[1].until(net, junction)) {
                    (None, _) => return Some(format!("{id:?} queued again after its whole way")),
                    (Some(a), Some(b)) if at(a).zip(at(b)).is_some_and(|(a, b)| a >= b) => {
                        return Some(format!("{id:?} queued out of its path's order"));
                    }
                    _ => {}
                }
            }
        }
        // 1.
        for (lane, _) in net.lanes() {
            let mut on: Vec<_> = sim
                .occupants(lane)
                .iter()
                .filter(|o| !o.is_ghost())
                .filter(|o| sim.get(o.id).is_some_and(|v| v.lane() == lane))
                .filter_map(|o| place(o.id).map(|p| (o.s(), o.id, p)))
                .collect();
            on.sort_by(|a, b| a.0.total_cmp(&b.0));
            for pair in on.windows(2) {
                let ((_, behind, p_behind), (_, ahead, p_ahead)) = (pair[0], pair[1]);
                if p_behind < p_ahead {
                    return Some(format!(
                        "{behind:?} is queued before {ahead:?}, ahead of it on {lane:?}"
                    ));
                }
            }
        }
        // 2. Each by the entry that covers the lane it is on: the first that
        // covers it right through, or stops at a lane after it on its path.
        let covering = |id, v: &traffic_sim::Vehicle| {
            let at = |l: LaneId| v.path().iter().position(|&p| p == l);
            queue.iter().position(|e| {
                e.id == id && e.until(net, junction).is_none_or(|u| at(u).is_none_or(|u| u > 0))
            })
        };
        let queued: Vec<_> = sim
            .vehicles()
            .filter_map(|(id, v)| covering(id, v).map(|p| (p, id, v)))
            .collect();
        for &(p_b, b, vb) in &queued {
            for &(p_a, a, va) in &queued {
                if p_a >= p_b || a == b {
                    continue;
                }
                let (mine, theirs) = (vb.lane(), va.lane());
                let Some(conflict) = net.conflict_between(mine, theirs) else {
                    continue;
                };
                if conflict.kind != ConflictKind::Cross {
                    continue;
                }
                let front = vb.lane_pos().s - vb.spec().rear_axle + vb.spec().length;
                let rear = va.lane_pos().s - va.spec().rear_axle;
                let entered = front > conflict.mine.start + ENTERED
                    && front - vb.spec().length < conflict.mine.end;
                let cleared = rear >= conflict.theirs.end;
                let geometry = net.lane(mine).geometry();
                let along = geometry.tangent_at(geometry.lane_pos(conflict.mine.start, 0.0));
                let same_way = glam::DVec2::from_angle(va.heading()).dot(along) > 0.0;
                if entered && !cleared && !same_way {
                    return Some(format!(
                        "{b:?} is in its conflict with {a:?}, queued above it, before it has cleared: {b:?} on {mine:?} front {front:.2} in {:?} at {:.2} m/s, {a:?} on {theirs:?} rear {rear:.2} of {:?} at {:.2} m/s; queue {:?}",
                        conflict.mine,
                        vb.speed(),
                        conflict.theirs,
                        va.speed(),
                        queue.iter().map(|e| (e.id, e.until(net, junction))).collect::<Vec<_>>()
                    ));
                }
            }
        }
    }
    None
}
