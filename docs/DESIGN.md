# Traffic Simulator — Master Design

This is the canonical design document. Change it only with Alexander's approval, ideally in the same commit as the code change it describes.

## Goals and working model

The goal is city and freeway traffic that looks realistic at 60 fps, built as simply and fast as possible. Where visual realism and aggregate accuracy conflict, realism wins.

**Goals**

- Believable motion through lane changes, merges, diverges and junctions, including hook turns
- 60 fps rendering with a simple, fast core
- Algorithms made visible in a debug view

**Non-goals**

- Calibrated aggregate outputs for policy work (SUMO and VISSIM already do this)
- Building networks from real-world data: networks are hand-crafted or procedural
- Demand modelling inside the core simulation
- Preventing gridlock: it is allowed, and fun to watch

**Working model**

- An LLM does almost all the typing. Alexander owns the architecture, data layout and module boundaries.
- The simulation step, the data layout and anything affecting determinism must be understood line by line before merging.
- "Swappable" means a clean module boundary, replaced by editing the module, not by traits or runtime dispatch.
- In this doc, unmarked statements are decisions. **Suggestion** marks proposals that haven't been explicitly confirmed yet. Open items are collected under Hard parts and open questions.

## Workspace and crates

One Rust workspace. The simulation crate depends on nothing else in the workspace; everything else depends on it.

| Crate                     | Owns                                                                                                      | Depends on       |
| ------------------------- | --------------------------------------------------------------------------------------------------------- | ---------------- |
| `sim`                     | Simulation state, models, update pipeline, public API, the `SimNetwork` type it consumes                  | —                |
| `network`                 | Source-of-truth network format; compiles it to a `SimNetwork` and to render geometry (and optionally SVG) | `sim`            |
| `ui`                      | egui app with a pretty view and a debug view                                                              | `sim`, `network` |
| `demand` (**Suggestion**) | Routing models and vehicle spawners, kept outside the core                                                | `sim`            |

**Suggestion:** `sim` defines `SimNetwork` and `network` produces it. This keeps the dependency graph one-way, so the core never sees authoring concepts like multi-lane roads.

## Network pipeline

One source-of-truth network description compiles into two products: a `SimNetwork` for the simulation and render geometry for the UI or SVG export.

- **Source of truth:** written by hand or by procedural generators. Real-world import is out of scope. The format is undecided and will grow organically.
- **`SimNetwork` output:** lanes as polylines, lane successors, permitted lane changes (lane pair, direction, s-range), precomputed conflicts, barriers and signal controllers.
- **Render output:** road and junction surfaces, lane markings, stop lines and keep-clear markings.
- **Compile-time work:** generating connector lanes through junctions, computing conflicts and placing barriers.

**Suggestion:** sample curves into lane polylines by tolerance, not at a fixed spacing: at most about 1–2 cm of sagitta (chord error, `c² / 8R`) and about 3° of turn per vertex. Gentle curves get long chords and tight junction connectors get short ones. This is how vector renderers flatten curves.

**Suggestion:** author at road level (roads with lane counts and offsets, junctions with turn movements), as OpenDRIVE and SUMO's edge/lane/connection model do, and expand to bare lanes during compilation. That is how people think about roads, and it keeps the sim free of road concepts.

Generating junction geometry (connector curves and junction surfaces) is the hardest part of this crate. SUMO's netconvert is largely heuristics for it; procedural networks keep it tractable.

## Lane model and geometry

The simulation has no roads, only lanes. Lanes may overlap, merge, diverge or cross, and lane changes may be permitted between them.

- **Geometry:** each lane is a polyline with arc length s and lateral offset t. The (s, t) ↔ world mapping is worked out and fast.
- **Relationships:** successors and predecessors (end-to-start connections); lane-change permissions (lane pair, direction, s-range); overlaps, found by conflict precomputation.
- **Multi-lane roads** exist only in the source-of-truth network. Compilation flattens them.

**Suggestion:** store cumulative arc length per vertex, and give each vehicle a cached segment index per lane it occupies. Vehicles move a fraction of a segment per frame, so projection becomes a forward scan from the cached index, close to O(1).

**Off the beaten path:** SUMO, VISSIM and Aimsun treat lane position as the source of truth and derive world position from it. Here world pose is the truth and lane coordinates are derived, as in autonomous-vehicle planners using Frenet frames and in OpenDRIVE's s/t coordinates. The reason is realistic motion through lane changes, merges and junctions.

## Simulation state and data layout

All vehicles live in one global `SlotMap`. Each lane keeps a list of occupant records sorted by s, and a vehicle appears in every lane it occupies.

**Vehicle state (sketch)**

- World pose (x, y, heading), speed, steering state
- Acceleration, held between longitudinal updates rather than recomputed each frame
- Path: the assigned lane followed by the next few lanes it will take, in a fixed-capacity inline list. Leader search and path tracking look along it across lane boundaries.
- Leader found by the last longitudinal update, with its gap. Interim, and to be removed: it can't represent more than one leader, and the debug view will read accumulated debug events instead (see Debug instrumentation).
- Barrier commitments
- Static attributes: vehicle class, dimensions, driver-model parameters

**Per-lane occupancy**

- Each record is `(s, vehicle id, projection hint)` and lives in the lane's list, not on the vehicle. There is one copy of s, so lists and vehicles can't drift apart, and sorting and leader search never touch the `SlotMap`.
- A record's s is the smaller of the vehicle's two rear corners' s on that lane. Followers care about where a vehicle's rear is. Lateral coordinates are discarded for now.
- **Joining:** a vehicle joins a lane when it advances onto it, which is when its rear axle passes the end of the previous lane.
- **Leaving:** it leaves a lane once both rear corners are past the lane's end. Leaving a lane with no successors removes the vehicle from the network.
- A vehicle on a loop can briefly have two records in the same lane, at two different s values. This is geometrically true and needs no special handling.
- Updated every substep, and cheaply: each record is re-projected from its hint, cleared records are dropped, and order is restored by insertion sort, which is O(n) on sorted input and cheap for nearly sorted input.
- Multi-lane membership is the general form of SUMO's "shadow lane" for a vehicle partway through a lane change.

**Suggestion:** start with an array of structs in the `SlotMap`. If profiling shows frame-rate work is memory-bound, split the hot per-frame fields (pose, speed, acceleration) into a dense struct of arrays.

## Timesteps and update pipeline

Work runs at three rates. Discrete decisions run at 5–10 Hz, acceleration at about 20 Hz, and steering and integration every frame. Each step flows strictly from state to derived state to decisions, then applies the decisions.

| Rate        | Runs                                                       | Writes to vehicle state                |
| ----------- | ---------------------------------------------------------- | -------------------------------------- |
| 5–10 Hz     | Lane-change decisions, barrier commitment, routing choices | Target lane, commitments, route cursor |
| ~20 Hz      | Longitudinal model (IDM+)                                  | Acceleration, leader                   |
| Every frame | Path tracking (pure pursuit), bicycle-model integration    | Pose, speed, steering                  |

**Data flow**

1. **Simulation state:** vehicles, occupancy, signal phases.
2. **Derived state**, computed from step 1 read-only: leaders, gaps, active conflicts, upcoming barriers.
3. **Decisions**, computed from steps 1–2 read-only: accelerations, lane changes, commitments.
4. **Apply:** decisions are written back to the simulation state.

Steps 2 and 3 only read, so they parallelise per vehicle (for example with rayon). Step 4 writes to disjoint vehicles.

The simulation rate is variable: `Simulation::step(dt)` advances by the frame's `dt`.

Long frames are split into substeps, so a slow frame can't destabilise integration or carry a vehicle past a barrier or conflict in one step.

**Suggestion:** fire the ~20 Hz and discrete updates from per-rate time accumulators.

**Suggestion:** stagger discrete updates across vehicles, for example by key modulo the number of ticks per decision period. This spreads load evenly and stops every vehicle reacting on the same tick.

## Vehicle kinematics

Vehicles move by a kinematic bicycle model, steered by pure pursuit toward a target path defined in lane space. World pose is the source of truth.

- **Inputs each frame:** held acceleration, a target path (the assigned lane, or a blended lateral offset during a lane change) and a lookahead distance.
- **Controller choice:** pure pursuit with a lookahead that scales with speed. It anticipates curves the way drivers do, cutting corners slightly, which reads as natural, and the guard rails bound how far it cuts. Stanley, which tracks the path more tightly, is the fallback if tracking on tight junction connectors looks wrong.
- **Guard rails:** a vehicle must never escape the lane geometry, for example by bounding its lateral offset from the assigned lane. The open question is how to do this without visible snapping.

**Suggestion:** describe a lane change as a target lateral offset that ramps from the current lane to the target lane over a few seconds, using a smooth blend such as cosine or quintic. The controller follows that offset, so the trajectory looks natural without being scripted.

## Longitudinal model

IDM+ (Schakel, van Arem and Netten, 2010) computes each vehicle's acceleration at about 20 Hz, behind a module boundary so it can be replaced. IDM+ splits IDM into a free-road term and an interaction term and takes the minimum instead of the sum. That gives a more realistic capacity, and it extends naturally to several leaders: the acceleration is the minimum over the free-road term and one interaction term per leader.

- **What it reacts to:** the nearest leader along the vehicle's path, vehicles in active conflicts projected into its own lane, and the nearest barrier it has not committed to. The most restrictive one wins.
- **Desired speed:** the smaller of the vehicle's maximum speed and its lane's speed limit.
- **Barriers as virtual leaders:** an uncommitted barrier is treated as a stationary vehicle at the barrier line. This is standard IDM practice from Treiber's work.
- **Evaluation:** the acceleration is a running minimum over the terms as they are computed. No list of leaders, conflicts or barriers is built.
- **Lane-change onset:** the model can see when each vehicle appeared on a lane. A newly appeared leader (a cut-in) does not cause sudden braking.
- **Heterogeneity:** IDM parameters (desired speed, time headway, maximum acceleration, comfortable deceleration, minimum gap) come from each vehicle's attributes.

**Suggestion:** keep the time a vehicle appeared on a lane on its occupant record, so a leader's record says how recently it appeared, and smoothing needs no per-vehicle leader state.

**Suggestion:** MOBIL's safety criterion already limits how hard a lane change can make the new follower brake. The appearance time then only has to smooth what is left, for example by easing from the previous leader's gap to the new one over 1–2 s.

## Lane membership, lane changes and diverging

All diverging traffic uses one mechanism: a vehicle stays registered on a lane it has left until it is geometrically clear of it.

- **Covers:** lane changes, diverging at junctions and taking off-ramps. No special cases are needed.
- **Effect:** while a vehicle straddles two lanes, followers on both lanes see it as a leader.
- **Where changes happen:** only inside a permitted lane-change s-range from the network.

**Suggestion:** use MOBIL (Treiber and Kesting's incentive-plus-safety model with a politeness factor) for discretionary changes. Drive mandatory changes from the routing model's required lane sequence.

**Suggestion:** plan for cooperative yielding, where a vehicle in the target lane slows to open a gap for a mandatory change. Without it, dense traffic leaves vehicles stuck at lane ends; SUMO's LC2013 model has explicit cooperation for this reason.

## Conflicts, merging and junctions

Merging and crossing traffic rely on lane-pair conflicts computed ahead of time. A conflict costs nothing until a vehicle approaches it, so cost scales with vehicles, not with the O(n²) conflict pairs in a junction.

- **Each conflict stores:** the two lanes, the overlapping s-range on each, its kind (merge or cross) and which side yields.
- **Merging:** a vehicle approaching a merge projects vehicles in the other lane's overlap range into its own s, and treats them as potential leaders or as traffic to yield to.
- **Crossing:** vehicles on crossing lanes must never overlap inside the conflict zone. Resolved by gap acceptance, plus arrival order where that sets priority (below).
- **Gridlock** is allowed.

**Suggestion:** give each yielding point one of two priority policies.

- **Fixed priority:** the yielding vehicle uses gap acceptance against priority traffic.
- **Arrival order:** for four-way stops and similar, vehicles join a junction queue when they stop at their line and are released first come, first served. Vehicles whose paths don't conflict may go together. SUMO models this as its `allway_stop` junction type.

Arrival order also breaks ties where two vehicles would otherwise wait for each other indefinitely.

**Gap acceptance** is a simple, conservative calculation from current speed, distance and maximum acceleration. The yielding vehicle goes only if it can clear the conflict zone before any priority vehicle could reach it, even if that vehicle accelerated at its maximum. Priority vehicles stopped at an uncommitted barrier are ignored, because they won't enter the zone, and this prevents deadlocks.

**Suggestion:** index conflicts per lane, sorted by start s. A vehicle then finds its upcoming conflicts by scanning forward from its own s within its lookahead, and only conflicts reached this way are active.

**Hook turns:** a right turn made from the left lane. The vehicle pulls forward to the left side of the intersection, waits, and completes the turn when the signal for the street it is entering turns green. This needs no new mechanism: a connector lane from the left lane, plus a mid-junction barrier tied to the cross street's signal phase.

## Barriers

Stop lines, give-way lines, traffic lights and the start of keep-clear zones are all barriers. A vehicle brakes for a barrier with IDM until it commits to crossing, then behaves as if the barrier is not there.

| Barrier                           | Commits when                                                                                                                        |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| Traffic light                     | It can no longer brake comfortably to stop, and the light is not yet amber or red                                                   |
| Stop line (**Suggestion**)        | It has come to a stop, and either the conflicting lanes have an acceptable gap or, at an all-way stop, it is first in arrival order |
| Give-way line (**Suggestion**)    | The conflicting lanes have an acceptable gap                                                                                        |
| Keep-clear start (**Suggestion**) | There is room beyond the far edge of the zone for the whole vehicle                                                                 |

- Amber and red are the same in the simulation.
- Signals run fixed cycles to start, behind a module boundary so actuated control can come later.

A green light exerts no deceleration, committed or not, unless there is no room across the junction. Then keep-clear rules apply.

## External API

Demand and routing live outside the `sim` crate. The simulation exposes vehicle insertion, lookup and removal, plus a pluggable routing model.

- **Insert** a vehicle from a spec: class, dimensions, driver-model parameters, starting lane and s, and its routing model.
- **Find** vehicles by key, by lane and s-range, and spatially (point or box) for picking in the UI.
- **Remove** a vehicle by key.
- **Hold** a vehicle: it brakes at its comfortable deceleration to a stop and waits until released. Held vehicles are kept in a short list on the simulation, not as a flag on every vehicle.
- **Pick** the vehicle nearest a point, within a tolerance, for the UI. Linear scan for now.
- **Routing model:** answers which lane to take next. Implementations include shortest path to a destination (Dijkstra or A*) and a random walk through the network.
- **Heterogeneity** comes from inserting vehicles with varied attributes. Distributions are the caller's concern, not the core's.

**Suggestion:** let insertion fail when there is no safe room at the requested position and speed. This stops vehicles popping into existence on top of others.

**Routing (interim):** until a routing model exists, a vehicle's path follows each lane's first successor.

**Suggestion:** the routing interface needs lookahead (the lanes required several links ahead), not just the next lane. Mandatory lane changes have to start well before the diverge; SUMO calls this "best lanes".

## Debug instrumentation

Algorithms report what they are doing through a debug sink that costs nothing when debug output is off. The mechanism is one generic parameter on the simulation.

| Option                      | Zero cost by                                | Trade-off                                                                 |
| --------------------------- | ------------------------------------------- | ------------------------------------------------------------------------- |
| Generic `Sim<D: DebugSink>` | A zero-sized no-op sink, monomorphised away | One extra type parameter; debug and headless builds coexist in one binary |
| Feature flag                | Code compiled out when the flag is off      | Simpler signatures; debug needs a separate build                          |

The generic was chosen: the UI runs with a recording sink and benchmarks with the no-op one.

**Suggestion:** pass closures (`sink.record(|| …)`) so building an event's data is skipped entirely when the sink does nothing.

**Events worth recording:** leader links and gaps, conflict activations, barrier commitments, lane-change decisions with their MOBIL scores, target paths and lookahead points.

The debug view shows what the simulation actually used, never a recomputation. Events are grouped by the update that produces them, and they accumulate in the sink:

- **Reset events:** each update first records a reset for its group ("recompute accelerations", "recompute motion" and, later, "recompute decisions"), which clears that group's earlier events. The UI never clears the sink itself.
- **Acceleration terms:** each term is its own event: the free-road term, each leader, conflict and barrier, and overrides such as holding. Every input to the final acceleration must be recorded, or the debug view's minimum won't match. The binding term is the one equal to the vehicle's stored acceleration, so a vehicle whose terms don't reach it shows that an event is missing.
- **Several updates in one step:** each update resets and refills its own group, and the UI reads only after `step` returns, so it always sees the last complete set.
- **Reset scope:** resets are global while every vehicle updates on the same tick, and removed vehicles' events vanish at the next reset. When an update is staggered across vehicles, its reset becomes per vehicle and the sink also needs a vehicle-removed event.

## UI

The `ui` crate renders the simulation with egui and offers two views: a pretty view and a debug view.

- **Pretty view:** road and junction surfaces and lane markings from the render geometry, vehicles as oriented shapes, and brake lights and indicators (cheap realism).
- **Debug view:** lane polylines with s ticks, per-lane occupancy, leader and gap lines, active conflicts, barriers with their commit state, and target paths. Clicking a vehicle opens an inspector.
- **Controls:** pause, single step, speed multiplier, camera pan and zoom.

**Suggestion:** start with egui's painter. If tessellation becomes the bottleneck at high vehicle counts, draw vehicles and roads through a custom wgpu paint callback with instancing, keeping egui for overlays and panels.

## Hard parts and open questions

The hardest problems are junction geometry and dense merging. Both have defeated simpler simulators.

**Hard parts**

1. Generating junction geometry: connector curves and junction surfaces.
2. Merges and mandatory lane changes in dense traffic, which need cooperative yielding.
3. Resolving crossing conflicts in a way that guarantees vehicles never overlap.
4. Choosing lanes several links ahead of a diverge.
5. Guard rails that keep vehicles on the lane geometry without visible snapping.
6. Inserting vehicles without visible pops.
7. Judging realism by eye, with no metric to check against.

**Open questions**

- How a vehicle joins its target lane during a lane change (milestone 3). Successor transitions join on advance, as above.

## Milestones

Each milestone is a demo that can be finished on its own; anything after it is optional.

1. Workspace skeleton: one straight lane, IDM, kinematics, the pretty and debug views. **Done.**
2. A ring road showing emergent stop-and-go waves, as in the Sugiyama experiment. **Done.**
3. A multi-lane freeway with MOBIL, continuous lane changes and multi-lane membership. It starts with the accumulating debug events described under Debug instrumentation, which replace the vehicle's stored leader.
4. An on-ramp merge and an off-ramp diverge, using conflicts and mandatory lane changes.
5. A single signalised intersection with barriers and a fixed signal cycle.
6. Hook turns.
7. A procedural grid with shortest-path and random-walk routing.

## References and prior art

These are the established approaches this design borrows from or deliberately departs from.

- Treiber and Kesting, _Traffic Flow Dynamics_: the textbook on IDM and MOBIL, by their authors.
- [traffic-simulation.de](https://traffic-simulation.de): Treiber's browser simulator, built on IDM and MOBIL with ramps and roundabouts. Readable source.
- [SUMO](https://eclipse.dev/sumo/): the reference lane-based microsimulator. See LC2013, shadow lanes, internal junction lanes and "best lanes".
- [A/B Street](https://github.com/a-b-street/abstreet): an open-source Rust traffic simulator with detailed write-ups on junction geometry and gridlock.
- [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/): road description with s/t lane coordinates and explicit junction connections.
- [Fix Your Timestep](https://gafferongames.com/post/fix_your_timestep/): fixed timesteps with interpolated rendering.
- Pure pursuit and Stanley path tracking: standard lateral controllers from autonomous-vehicle research.
