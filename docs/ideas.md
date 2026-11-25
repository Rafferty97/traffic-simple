# Ideas to explore

Investigations and improvements worth picking up, each with where it starts. Open problems already understood are in [Known issues](known-issues.md); these are the projects around them.

## Junctions and the queue

1. **Hook turners holding up straight-on traffic.** The turner's record on the straight-on branch persists because its turn swings back across it (see Known issues). Start in `sim/src/following/classify.rs`: should "diverged for good" ignore a converge that only comes after the vehicle's waiting barrier?
2. **Lifts refused for an edge the rule would reverse anyway.** `View::lift` in `sim/src/junction.rs` treats every decided pair as fixed. Could one lift move every vehicle the lower one also qualifies to pass, in a single Pearce–Kelly step, rather than relying on walking the nearest pair first?
3. **Where barriers still wait outside the order.** Four lockups came from barrier rules disagreeing with the junction: room beyond, giving way to vehicles inside, gap checks and following at forks. An audit of every place a barrier names a vehicle it waits on (`sim/src/models/gap_acceptance.rs`) against `junction::goes_after` might find the next before the stress test does.
4. **Swinging wide on tight turns** (see the design, Vehicle kinematics). Self-contained, and it changes how hook turns and trams look.

## Geometry

5. **Connector shapes from swept paths.** The widening, the tapers and `unfold` (see Known issues). Try a fixed width plus an explicit sweep check, and see whether conflicts and the stress tests still hold.
6. **Tram-only connectors sized by the tram.** Unignores the trams test on `poc/trams`: pick the design vehicle in `network/src/compile.rs`'s `swept` from the connector's allowed classes.
7. **A compile-time check on waiting points.** No turner's body at rest at its waiting line should be in another turner's way, as it is on the 60° crossroads. It no longer locks up, but it looks wrong.

## Vehicles

8. **Articulated vehicles** (see the design, Vehicle classes): a chain of sections, each on rails or trailing at its hitch, a car being one. Start with the data layout: sections inline on every vehicle, or in a side table.
9. **Curve speed that falls with speed.** One function in `sim/src/models/acceleration/profile.rs`; Reymond et al. (2001) is the reference. Watch the ramps and the ring.
10. **Trucks and buses as presets,** and bus lanes as the first use of lane classes beyond trams.

## Measuring

11. **The validation harness** (milestone 9). Saturation flow at signals and critical gaps at give-way lines are the quickest wins, and would show whether the junction's order costs capacity anywhere.
12. **Hook-turn throughput** against the old cycle check: 174 vehicles out with it and 210 without, at 250 veh/h per lane in, over 15 minutes. Where does it land now?
13. **Profiling a step on the 16 by 16 grid.** Deciding a pair now works out junction terms up to three times, in the reorder, the terms and the gap checks; caching each pair's needs per macro step may be measurable.

## Tidying

14. **Replays,** a hack to remove: the format now lives only in the UI.
15. **Speed profiles on large networks,** their cost, and whether connectors of the same shape could share one.
