# Plan: completing milestone 7, unsignalised intersections

Milestone 7 in `docs/DESIGN.md` covers priority roads with give-way and stop signs, all-way stops and keep-clear zones, plus turning on red at signals. All-way stops are done. This plan covers the rest.

It's a hand-off: it assumes you're starting cold. Read the sections below in order before changing anything.

## Before you start

- **Read `CLAUDE.md` and `docs/DESIGN.md` in full.** DESIGN.md is the canonical design. Change it in the same commit as the code it describes. If a step here conflicts with it, stop and ask Alexander rather than working round it.
- **Ownership rules:** Alexander must approve new dependencies, non-trivial `unsafe`, and any allocation or data-layout change inside the simulation loop (`Simulation::step` and everything it calls). Propose it, explain why, and wait. Adding a `BarrierKind` variant, or a field to a network type, is fine; adding state to `Vehicle` or `SimState` needs approval.
- **Commits:** small and focused, each once its design is settled and it's tested. Stage files by name, never `git add -A`, because Alexander edits the tree at the same time. End each message with the attribution line from the session's instructions.
- **Commands:**
  - Build: `cargo build --workspace`
  - Test: `cargo test --workspace --release`, with `--no-fail-fast` to see every failure.
  - Lint: `cargo clippy --workspace --all-targets -- -D warnings`
  - Format: `cargo fmt --all`
  - Stress test, a few minutes: `cargo test --release -p traffic-network --test stress -- --ignored`
  - UI: `cargo run -p ui -- <scenario>`. An unknown name lists the scenarios.
- **No rabbit holes.** If a fix starts to pile special cases on special cases, stop and describe the problem to Alexander. The junction design below was reached that way, and it's worth keeping simple.

## The design you're building on

The rules that matter most, all in DESIGN.md, under "Conflicts, merging and junctions", "Barriers and signals" and "Gap acceptance and priority":

- **Barriers decide when a vehicle may enter, and the junction keeps the order.**
  - Barriers are stop lines, give-way lines, signals and all-way stops.
  - The junction keeps committed vehicles apart in the order they were handed over, and keeps that order free of cycles by its own rule, the fourth queue invariant.
  - Nothing in the junction depends on a barrier for correctness. Keep it that way.
- **Barriers carry the lanes they give way to.** `Barrier::yields` points into `SimNetwork::yields`, and compilation works the sets out. The simulation has **no priority levels**. They exist only in `network/src/compile.rs`, to derive those sets: `PRIORITY_ROAD`, `STRAIGHT_OR_NEAR`, `FAR_SIDE`.
- **The gap check** is `has_gap` and `clear` in `sim/src/barrier.rs`:
  - **Lanes in the barrier's set:** the full check, going ahead of or behind each vehicle.
  - **Any other conflicting lane:** only committed vehicles count (queued, or past every barrier before the conflict), and only to go behind.
  - **Vehicles that count as stopping:** one held by a red light, or by the junction (`Queue::held`), will surely stop.
  - **No going ahead of** a queued vehicle.
  - **No going behind** one not yet queued, or one queued only as far as a waiting point that lies in the conflict.
- **Far-side turns wait inside the junction.** Compilation splits the connector at its first conflict with oncoming traffic. The second lane starts with a `GiveWay` barrier whose set is the oncoming lanes. The turner is queued a stretch at a time (`junction::Entry::until`).
- **Keep-clear lanes.** Every junction lane has `keep_clear` set, and a vehicle that can't stop before one is queued. This is separate from the keep-clear *zones* in this milestone, which are barriers.
- **Room beyond and giving way to vehicles inside** are barrier rules, and driver behaviour that the junction never relies on.

## What's left

1. **Stop signs:** a `Stop` barrier, which comes to rest and then takes a gap.
2. **Priority intersections:** `Control::Priority`, with give-way or stop signs on the minor approaches, and scenarios.
3. **Keep-clear zones:** marked stretches of road, as barriers, drawn in the UI.
4. **Turning on red.**
5. **Tests, stress and docs** for all of the above, and marking milestone 7 done.

Do them in this order. Each later step builds on the earlier ones, and each is a natural commit, or a few.

## Step 1: stop barriers

DESIGN.md's commit table already has a "Stop line" row: it commits once it has come to rest at the line and there's a gap.

- **Barrier kind:** add `BarrierKind::Stop` in `sim/src/network.rs`, documented like the others. The UI's debug view draws barriers by kind (see `draw_debug` in `ui/src/main.rs`), so give it a colour and add it to the layer legend.
- **Decision:** in `barrier::decide`, `Stop` commits when `vehicle.stopped_at.is_some()`, there's a gap (`has_gap`, with the barrier's set as for `GiveWay`), and `room()` holds.
- **Coming to rest:** the arrival record (`Vehicle::stopped_at`, set where `arrived` is computed in `longitudinal` in `simulation.rs`) is only set for `AllWayStop` today. Set it for `Stop` too. DESIGN.md has this as **Planned** ("the same record for stop lines"). It's an existing field, so it isn't a layout change.
- **Release:** `barrier::released` treats `GiveWay` and `AllWayStop` alike: released once at rest short of the line, unless already queued past it. Do the same for `Stop`.
- **Coming to rest means stopped:** `acceleration::stopping` and the `stopping` flag bring a car fully to rest. Check that a car really reaches `speed == 0.0` at a stop line. The all-way tests rely on the same thing.
- **Tests,** in a new `sim/tests/stop.rs` modelled on `sim/tests/all_way.rs` and `signals.rs`:
  - a car always comes to rest before crossing;
  - with traffic in its set approaching, it waits;
  - with a gap, it goes after stopping;
  - one at rest at the line isn't released while queued past it.

## Step 2: priority intersections

### Network

- **The control:** add `Control::Priority { major: Vec<RoadId>, signs: Vec<(RoadId, Sign)> }` in `network/src/lib.rs`, with `enum Sign { GiveWay, Stop }`. An approach not named in `major` gets the sign `signs` gives it, give-way by default. Document it, and the levels it implies (see the DESIGN.md bullet "Priority levels").
- **Compilation,** in `network/src/compile.rs`:
  - **Levels:** work out each connector's level with the existing constants: priority-road straight or near-side 3, priority-road far-side 2, minor-road straight or near-side 1, minor-road far-side 0.
  - **Minor approaches:** each connector from one gets a `GiveWay` or `Stop` barrier at its stop line, carrying the lanes of every higher-level connector it conflicts with. The stop line comes from `stops[k][c]`, on `entries[k][c]`, the first lane, which may be a lead-in.
  - **Far-side turns from the major road:** they get the same inner give-way treatment as at signals, with the sets for their oncoming traffic. "Can be moving at the same time" is always true at a priority intersection. Refactor rather than copy: today that logic lives inside `add_signals`, with signal phases deciding "together". Pull the far-side split-and-yield part out into a function taking a "together" predicate, used by both `add_signals` and a new `add_priority`.
  - **Splitting:** `compile_swept` already re-splits connectors at inner give-way barriers, from the waits that `add_signals` returns. Make `add_priority` return its waits the same way.
  - **Minor-road far-side turns:** decide with Alexander whether they also wait inside. Do they cross one carriageway, then wait for the other? The simplest choice is to take one gap from the line, giving way to everything of a higher level. Recommend that, and ask.
- **Scenarios,** in `network/src/scenarios.rs`:
  - `priority_crossroads(lanes, speed, sign)`: the first road, east to west, is major, and the second road's approaches get `sign`.
  - Possibly a **T-junction** of a major road and a minor road. It's more realistic, and the right-hand side-road turn is a good test.
  - Alexander has mentioned a **dual carriageway with median storage** for turning right. Don't build that without asking: it's a design discussion, not this milestone.
- **UI,** in `ui/src/main.rs`: add scenarios to `enum Scenario`, `Scenario::ALL`, `key`, `name` and `network`. For example `priority-1`, `priority-2`, `stop-1`.

### Behaviour to check

- **Minor-road cars wait for gaps** in major-road traffic. From rest, a car turning across a 50 km/h road should accept gaps of roughly 6–7.5 s, the Highway Capacity Manual's critical gaps; DESIGN.md says about 7 s.
- **Major-road traffic never brakes hard** for minor-road traffic. Adapt the signalised test `at_signals_straight_traffic_never_stops_or_brakes_hard_for_right_turners_on_its_green` in `network/tests/scenarios.rs`.
- **Stop-sign cars always come to rest** before going.

## Step 3: keep-clear zones

DESIGN.md, under "Intersection control", says a road may mark stretches keep clear. Each becomes a barrier at its start on every lane across it, carrying the zone's length. The commit table says it commits when there's room beyond the zone's far end for its length and minimum gap behind the nearest vehicle there. Render output has "**Planned:** keep-clear markings".

- **Network:** a road gets `keep_clear: Vec<(f64, f64)>`, stretches by distance along its reference line. Compilation turns each into a `BarrierKind::KeepClear { length }` on every lane across that stretch, at the stretch's start in each lane's own s.
- **Simulation:**
  - **The decision:** room beyond the zone, much as `room_beyond` works for a junction. Find the rearmost vehicle on the path past `s + length`, and check there's room for the vehicle and its minimum gap.
  - **Inside the zone:** vehicles stop for leaders as usual.
- **UI:** paint the zone as a hatched box, or in the "KEEP CLEAR" lettering style. Add `MarkingStyle::KeepClear` to `network/src/render.rs`, and draw it in `draw_pretty`.
- **Scenario:** a keep-clear zone across a road just before a crossroads, in the mouth of a side road such as a T-junction's. When traffic queues back from the crossroads, cars should leave the zone empty.
- **Keep-clear zones are driver behaviour.** The junction's correctness doesn't depend on them. Don't wire them into the junction.

## Step 4: turning on red

DESIGN.md, under "Turning on red", says a movement may allow it. On red, its signal barrier is then a stop line followed by a give-way line: the vehicle comes to rest, then takes a gap. Compilation allows it on near-side turns where the network says so, the default where traffic drives on the right, and on any movement a plan names.

- **Barrier:** `BarrierKind::Signal { movement, on_red: bool }`, or a separate flag. On red with `on_red` set, the barrier behaves as `Stop`: at rest, then `has_gap` against its set, then `room()`. Its set is the lanes its connector conflicts with. Lanes held by their own red count for nothing, because `held_by_light` already discounts them. On amber, it keeps today's rule: nothing commits.
- **Network:**
  - `RoadNetwork` gets a turn-on-red setting, which defaults on for `DrivesOn::Right`, and off for Australia.
  - `SignalPhase` or `Control::Signals` lists extra movements allowed to turn on red.
  - The near-side turn is `Turn::Left` when driving on the right, and `Turn::Right` when driving on the left (`Turn::far_side`).
- **Tests:** on red, a car turning near-side stops, then goes in a gap; one on a movement without it waits for green.
- **Scenario:** a signalised crossroads driving on the right, with turning on red. This may be the first right-hand-traffic scenario, so check that the crossroads builder supports `DrivesOn::Right`.

## Step 5: tests, stress and docs

- **Flood tests:** for each new scenario, add one to `network/tests/scenarios.rs`, using `flood()`. That checks collisions, body coverage and the queue invariants (`network/tests/common/mod.rs`).
- **Flow tests:** add one per scenario, like `all_way_stops_keep_traffic_flowing`.
- **Stress test:** extend `network/tests/stress.rs` to the priority crossroads (give-way and stop), the T-junction if built, the all-way stops, and the turn-on-red crossroads. All 200+ runs must pass.
- **DESIGN.md:**
  - update "Intersection control", the barrier commit table, "Turning on red", keep-clear zones, render output and the UI scenarios;
  - clear the **Planned** markers you complete;
  - mark milestone 7 **Done**.

## Watch out for

- **Don't put priority levels back into the simulation.** Everything a barrier gives way to must be in its yield set, worked out at compile time.
- **Don't make the junction rely on barriers.** If a lockup appears, find which queue invariant broke. The invariant checks in the flood and stress tests usually name it. Then fix it at its cause.
- **Splitting and lead-ins.** Connectors may be cut into a lead-in before the junction (at a skewed stop line), a first stretch, and a second stretch from an inner give-way line. A stop-line barrier goes on the connector's first lane (`entries[k][c]`), whatever that is. The fork rule traces branches back through lead-ins (`SimNetwork::first_of_connector`).
- **Barrier positions on skewed stop lines.** `Barrier::s` is the conservative point, and `Barrier::line` the exact line. Gaps use `barrier::gap_to`.
- **Hot paths.** `has_gap` and `clear` run at 20 Hz for heads at barriers. Don't allocate there; follow the existing `ArrayVec` patterns.
- **Freeze frames help with debugging.** See `network/src/frames.rs`. A scripted frame of a tricky case, opened in the UI, is far quicker to reason about than printed probes.

## Questions to settle with Alexander

1. **Minor-road far-side turns:** do they take one gap from the line, or wait inside? Recommend one gap from the line.
2. **The T-junction:** whether to build it, and its geometry. Recommend yes, a plain T.
3. **Keep-clear markings:** how they look in the pretty view.
4. **Turning on red:** whether it should be offered in an Australian (left-hand-traffic) scenario at all, or only as a right-hand-traffic option. It isn't legal in Australia.
