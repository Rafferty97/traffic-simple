# Traffic Simulator — Master Design

This is the canonical design: how the finished simulator works, both what is built and what is planned. Keep it current, changing it in the same commit as the code it describes, and ask Alexander before changing its goals.

## Goals and working model

The goal is city and freeway traffic that looks realistic at 60 fps, built as simply and fast as possible. Where visual realism and aggregate accuracy conflict, realism wins.

**Goals**

- Believable motion through lane changes, merges, diverges and junctions, including hook turns
- Trams, as in Melbourne: on tracks in the road or on their own, stopping at their stops, with cars stopping behind a tram at a stop in the road
- Intersections controlled as real ones are: signals, give-way and stop signs, all-way stops and keep-clear zones, with turning traffic giving way as drivers do
- Vehicles that find their way to destinations, changing lanes in good time
- 60 fps rendering with a simple, fast core: a step for 10,000 vehicles well inside a frame (**Suggestion:** under 2 ms on a laptop, as the benchmarks measure it)
- Traffic flow that doesn't break down in ways real traffic wouldn't
- Algorithms made visible in a debug view

**Non-goals**

- Calibrated aggregate outputs for policy work (SUMO and VISSIM already do this)
- Building networks from real-world data: networks are hand-crafted or procedural
- Demand modelling inside the core simulation
- Pedestrians, cyclists, passengers and parking
- Optimising signals: plans are fixed-time, with actuated control left to a module boundary
- Preventing gridlock: it is allowed, and fun to watch

**Working model**

- Claude does all the typing, and keeps the code tidy and modular as it goes. Alexander reviews as needed, owns the goals, and is asked about choices of taste or of the project's objectives.
- New dependencies, non-trivial `unsafe`, and allocation or data-layout changes in the simulation loop are proposed to Alexander first.
- "Swappable" means a clean module boundary, replaced by editing the module, not by traits or runtime dispatch.
- Where it can, the simulation follows established technique (SUMO, IDM and LMRS literature, game-engine practice). Its deliberate departures are world coordinates as the truth, how vehicles find their leaders across lanes (see [Multi-lane car following](multilane-car-following.md)), the junction backstop and merges; each is marked **Off the beaten path**.
- In this doc, unmarked statements are decisions. **Suggestion** marks proposals that haven't been explicitly confirmed yet. **Planned** marks designed parts not yet built. Open items are collected under Hard parts and open questions.

## Workspace and crates

One Rust workspace. The simulation crate depends on nothing else in the workspace; everything else depends on it.

| Crate                     | Owns                                                                                                      | Depends on       |
| ------------------------- | --------------------------------------------------------------------------------------------------------- | ---------------- |
| `sim`                     | Simulation state, models, update pipeline, public API, and the `SimNetwork` type it consumes             | —                |
| `network`                 | Road-level network format and scenarios; compiles them to a `SimNetwork` and to render geometry          | `sim`            |
| `ui`                      | egui app with a pretty view and a debug view                                                              | `sim`, `network` |
| `demand` (**Planned**)    | Spawners, origin–destination demand and vehicle mixes, kept outside the core                              | `sim`            |
| `validation` (**Planned**) | A headless harness measuring aggregate behaviour against published values                               | `sim`, `network`, `demand` |

`sim` defines `SimNetwork` and `network` produces it, so the dependency graph is one-way and the core never sees authoring concepts like multi-lane roads.

Inside `sim`, each part of the design has its own module: `lane` (geometry), `network` (`SimNetwork`), `occupancy` and `following` (records, the leader and follower searches, and classification), `junction` (the backstop), `merge`, `barrier` (**Planned:** barriers, commitment and gap acceptance), `lane_changing` (what the simulation measures for the lane-change model), `path`, and the models, each replaced by editing it: `acceleration`, `lane_change`, `lateral`, `routing` and `signal` (**Planned**). `simulation` holds the state, the update pipeline and the public API.

## Network pipeline

One road-level network description compiles into two products: a `SimNetwork` for the simulation and render geometry for the UI.

- **Source of truth:** built by procedural generators (`scenarios`) or by hand. Real-world import is out of scope. The format will change as it grows.
- **`SimNetwork` output:** lanes as polylines, their successors, neighbours with lane-change permissions, which side drivers keep to, how lanes that end are left, junctions with their conflicts, and signal plans with their barriers. **Planned:** the other barriers.
- **Render output:** road and junction surfaces, lane markings, stop lines and turn arrows. **Planned:** keep-clear markings.
- **Precision:** every lane point and half-width is snapped to a 1 mm grid as the lane is built, so a network serializes exactly as short text. Two grid points under 2 mm apart are the same point: snapping leaves two copies of one at most √2 mm apart, and the next distance between grid points is 2 mm. That is how lane ends are matched and checked, with no other tolerance.

The `network` crate describes networks at road level, as OpenDRIVE and SUMO's edge/lane/connection model do, and expands them to bare lanes during compilation. That is how people think about roads, and it keeps the sim free of road concepts.

- **Roads:** a reference line, a start pose followed by pieces (lines, arcs and cubic Béziers) that each continue from the last, and lanes beside it, each with an offset and a width that are constant or ease (smoothstep) from one value to another over a stretch, and a direction along the road. Clothoids can come later.
- **Sampling:** curves, junction connectors included, are sampled by tolerance, not at a fixed spacing: at most 1 cm of sagitta (`c² / 8R`) and 3° of turn per vertex, and where an offset or width eases, enough vertices that it strays at most 1 cm from the smoothstep between them. The vertex next to each end of a road goes on the tangent there, as far out as stays within 1 cm of the curve, so lanes end square across the road.
- **Lanes from one polyline:** a road's lanes are offset from one sampled reference polyline by its miters, so neighbouring lanes end square across the road, side by side, as `validate` requires, and meet the next road's lanes exactly.
- **Neighbours:** lanes of a road running side by side the same way and keeping their distance; lanes easing together, at a lane drop, or apart, are not. Each lane says whether vehicles may change into its neighbour on the left and on the right, as SUMO's `changeLeft` and `changeRight` do; both by default.
- **Lane use** (**Planned**): each lane says which vehicle classes may use it, as SUMO's `allow` and `disallow` do: all but trams by default, trams only on tram tracks, and buses only on bus lanes. A road's tram tracks are lanes like any other, either their own or shared with general traffic.
- **Keeping to a side:** the road network says which side drivers keep to on roads above a speed limit: in Australia, the left, above 80 km/h. Compilation marks those roads' lanes.
- **Lanes that end by merging,** as an acceleration lane does, say how drivers leave them: at the merge, taking turns (a zipper), or along the way (early). The ramps scenario offers both.
- **Joins:** a road's end joins the next road's start, and lanes connect wherever one ends where the other starts: two meeting one merge, one meeting two diverge, with nothing declared. Merging road lanes are not a junction: they are a merge (see Merges).
- **Intersections:** each lists which lanes connect to which, and each connection becomes a cubic Bézier from one lane's end to the other's start along their directions, its control points a third of the way across, its width blending from one to the other, and its speed limit lowered for its tightest bend (2 m/s² sideways, at least 3 m/s). An intersection's connectors are one junction.
- **Render output:** each road's surface between its outermost edges; lane markings, dashed between lanes going the same way, a centre line between opposite ones, and edge lines; and each intersection's surface, the mouths of its roads joined by cubic Bézier curbs.
- **Stop lines:** each lane coming into an intersection gets one across its end, 300 mm wide as AS 1742.2 has it, set back half that so its far edge is where the intersection starts.
- **Turn arrows:** each lane coming into an intersection gets an arrow 8 m back from its end for the ways its connections go, as SUMO classes a link's direction by its angle: straight on within 30° either way, otherwise left or right. Its ways are the connections' turn classes (see Intersection control). It's one of seven glyphs, every mix of left, straight and right, drawn in `ui/assets/lane-glyphs.svg` and converted once to triangles in `ui/src/glyphs.rs` by `ui/assets/convert-glyphs.mjs`, so the UI needs no SVG support. None is painted on a road shorter than 16 m.

**Intersection control** (partly **Planned**) turns how an intersection is controlled into barriers, priorities and signal plans, as SUMO's netconvert derives right of way and traffic-light logic from road priorities and turn directions, simplified.

- **Turn classes:** compilation classes each connection as straight on, within 30° either way, or a turn, and reports it beside the connectors; a turn is near-side, toward the side traffic drives on, which the network says (the left by default, as in Australia), or far-side, across oncoming traffic. Arrows, priorities and signal movements all use it. **Built:** the classes and the arrows reading them.
- **Control:** an intersection is controlled by priority, naming the roads that have it; as an all-way stop; or by signals, with a fixed-time plan. An approach without priority has a give-way or a stop sign. Uncontrolled, it has only the junction backstop. **Built:** uncontrolled and signals, with their priority levels and unprotected turns' waiting lines.
- **Priority levels:** each connector gets a level, the higher going first: straight on or near-side from a priority road, 3; far-side from one, 2; straight on or near-side from any other road, 1; far-side from any other, 0. All-way stops and signalised intersections treat every road as a priority road, so far-side turns still give way to oncoming traffic when both have a green.
- **Barriers from control:** each connector from an approach without priority starts with a give-way or stop barrier; every connector of an all-way stop with an all-way stop barrier; and every connector of a signalised intersection with a signal barrier for its movement. A far-side turn that conflicts with a connector of a higher level that can be moving at the same time (at a priority intersection, always; at signals, if their movements share a phase) also gets a give-way barrier inside the junction at the start of its first such conflict. There it waits for a gap, clear of the traffic behind it, as drivers turning across do.
- **Signal plans** (**Built**)**:** a plan is a list of phases, each the movements green in it and how long for. A movement is an approach and a turn class, whose connectors go and stop together; connectors only trams may use are movements of their own, so trams can have their own phases. Two movements conflict if any of their connectors do, except a far-side turn and the traffic it gives way to, which may share a phase as an unprotected turn (a filter turn, in Australian usage), the turner giving way to the oncoming traffic. A movement's amber time is ITE's, a second's reaction plus `v / (2 · 3 m/s²)` at its approach's limit, and its clearance the time to cross its longest connector and a car's length at that limit.
- **Hook turns:** at signals, an approach may make its far-side turns hook turns, as in Melbourne, keeping turning traffic out of the tram tracks in the middle of the road. Each connector leaves from the near-side lane and runs straight on to a waiting line, then turns. The waiting line comes from the geometry of the road on the near side, whose traffic will follow the turner into the road turned into: from the kerb-side corner of that road's stop line, offset by a fixed padding along the road and across it, a stop line perpendicular to that one, across the turner's way. There a signal barrier for the movement of the road turned into holds the turner until that road's light turns green, when it turns ahead of that road's traffic.
- **Keep-clear zones:** a road may mark stretches keep clear. Each becomes a barrier at its start on every lane across it, with the zone's length.
- **Exits and entries:** compilation reports each road end's lanes into and out of the network, so demand can place vehicles and name destinations by road.

Generating junction geometry is the hardest part of this crate. SUMO's netconvert is largely heuristics for it; procedural networks keep it tractable.

## Lane model and geometry

The simulation has no roads, only lanes. Lanes may overlap, merge, diverge or cross, and lane changes may be permitted between them.

- **Geometry:** each lane is a polyline with arc length s and lateral offset t. Each vertex has its own left and right half-widths, so a lane can taper and its centreline needn't be centred; between vertices they vary linearly in s, as OpenDRIVE gives lane width as a function of s. A projection from world to lane coordinates starts from a hint, the last one's result with its segment, and scans from there, so it is close to O(1) for a vehicle that has moved a little.
- **Serialized geometry:** a lane's geometry serializes as a string of vertices in whole millimetres, each `x,y,left,right`; with three values the half-widths are equal, and with two they are the previous vertex's: `0,0,1750 90000,0 100000,2000,2000,1750`. Being on the grid, it reads back exactly.
- **Module boundary:** a lane's geometry, `LaneGeometry`, is its own type in its own module; the `Lane` holds it with its speed limit, keep side, how it is left if it ends, and links, which only the network sets.
- **Relationships:** successors and predecessors (end-to-start connections), kept in order from left to right; left and right neighbours, at most one each side, with permission per ordered pair, since a change may be allowed one way only; and overlaps, found when the simulation is built (see [Multi-lane car following](multilane-car-following.md)).
- **Priority and barriers:** each junction lane has a priority level, and each lane its barriers, sorted by s.
- **Lane sections:** adjacent lanes start and end together, like the lanes of a SUMO edge or an OpenDRIVE lane section, so the neighbour of a lane's successor is the successor of its neighbour, which lets a vehicle map its path to the lanes beside it. `SimNetwork::validate` checks this, with mutual neighbours and successors starting where their lane ends.

**Off the beaten path:** SUMO, VISSIM and Aimsun treat lane position as the source of truth and derive world position from it. Here world pose is the truth and lane coordinates are derived, as in autonomous-vehicle planners using Frenet frames and in OpenDRIVE's s/t coordinates. The reason is realistic motion through lane changes, merges and junctions.

## Simulation state and data layout

All vehicles live in one global `SlotMap`, an array of structs. Each lane keeps a list of occupant records sorted by s, and a vehicle appears in every lane it occupies. If profiling shows frame-rate work is memory-bound, the hot fields could split into a struct of arrays (**Suggestion**).

The state is one struct, `SimState`, holding everything that can't be rebuilt from the rest: the network, the vehicles, the occupant lists, the junction queues, speed caps, the step count, time and config, each signal controller's state, and (**Planned**) the destinations registered. It is exactly what a freeze frame saves. What can be rebuilt is derived from it when a simulation is built or restored, and kept apart: the overlaps between lanes, the merges, how far beyond each lane the paths through it reach (see Searching back), and (**Planned**) each destination's route table.

**Vehicle state**

- World pose (x, y, heading), speed, and the curvature of the rear axle's path (its steering state)
- Acceleration, held between longitudinal updates
- Path: the lane it follows and the lanes it will take next, as far as its leader search, path tracking and lane change look, and as far as it would go in 15 s at each lane's speed limit, however short the lanes. The time is at the limits, not its speed, so a slow vehicle in a queue still knows where it is going, and merges and junctions ahead know of it; its leader search stays within its horizon however far the path reaches. A path ending inside a junction is extended until it is out of it, so a vehicle in a junction's queue always knows its way through. It is a `Vec`.
- Route: how it chooses its way at forks, with any state that needs (see Routing)
- Commitment: how many of the barriers ahead of its rear axle on its path it has committed to crossing, a `u8`, and (**Planned**) the step at which it came to rest at a stop line, if it has (see Barriers and signals)
- Indicator: the side it is indicating to and since when
- Lane-change state, owned by the lane-change model, and the old side and lane change in progress
- Old follower: the vehicle behind that it holds up most, as of the last longitudinal update, with that vehicle's acceleration from every other term, for the courtesy incentive
- Acceleration noise and its own random generator's state
- Class and static attributes (`VehicleSpec`): dimensions and driver parameters, with a preset for each class (see Vehicle classes)

**Per-lane occupancy**

- Each record is `(vehicle id, projection hint, classification, age)`, and lives in the lane's list, not on the vehicle. The hint is the lane coordinates of the vehicle's rearmost corner, so its s, the smaller of its rear corners' s on that lane: followers care about where a vehicle's rear is. There is one copy of s, so lists and vehicles can't drift apart, and sorting and leader search never touch the `SlotMap`. A ghost, a vehicle's indicator, is a record classified as one. The age is how long the vehicle has been in the lane's traffic (see Relaxation).
- **Classification, joining and leaving** are described in [Multi-lane car following](multilane-car-following.md). A vehicle joins a lane when it advances onto it, when its rear axle passes the end of the previous lane, or changes lanes into it.
- **Leaving the network:** a vehicle leaves when its rear axle passes the end of the lane it follows and that lane has no successors.
- A vehicle on a loop can briefly have two records in the same lane, at two different s values. This is geometrically true and needs no special handling.
- Records are updated every step: each is re-projected from its hint, cleared records are dropped, and order is restored by insertion sort, which is linear on nearly sorted input.
- Multi-lane membership is the general form of SUMO's "shadow lane" for a vehicle partway through a lane change.

## Timesteps and update pipeline

The simulation steps by a fixed time, set when it is made (`Config`): 1/60 s by default. `Simulation::step()` takes one step; `step_variable(dt)` takes as many as fit in a frame's `dt`, carrying what is left over, as "Fix Your Timestep" has it. A fixed step makes a run the same whatever the frame rate, and a slow frame can't destabilise integration or carry a vehicle past a conflict in one step.

| Rate                                 | Runs                                                                                     | Writes                            |
| ------------------------------------ | ---------------------------------------------------------------------------------------- | --------------------------------- |
| Every step                           | Signal controllers; path tracking (pure pursuit), bicycle-model integration, occupancy, barriers passed, junction queues | Signal states; pose, speed, steering; records; commitment |
| Every `accel_every` steps (3: 20 Hz) | Lane-change desire and gap checks; longitudinal model (IDM+), with barrier decisions    | Desire, indicator, lane; acceleration; commitment |

Each update flows strictly from state to derived state to decisions, then applies the decisions: decisions are computed from a read-only view of the state, so they could parallelise per vehicle, and then written back.

**UI:** the UI draws vehicles between where they were before the last step and where they are, by how far into the next step the frame has reached (`Simulation::alpha`), so they don't jump a step at a time when frames and steps don't line up. They are drawn a step behind.

## Vehicle kinematics

Vehicles move by a kinematic bicycle model, steered by pure pursuit toward a target path defined in lane space. World pose is the source of truth.

- **Controller:** pure pursuit, with a lookahead of 2 m plus 0.3 s at its speed. It anticipates curves the way drivers do, cutting corners slightly, which reads as natural. Stanley, which tracks more tightly, is the fallback if tight junction connectors look wrong.
- **Steering as curvature:** steering is stored as the curvature of the rear axle's path rather than as a front-wheel angle. Pure pursuit's curvature `2·sin(α) / d` is `2·(h × d) / |d|²` for heading `h` and offset `d` to the target, and the bicycle model turns by `dist · curvature`, so neither needs trigonometry. The steering lock is a curvature limit, `tan(δ_max) / wheelbase`, with δ_max 0.6 rad. This made a step about 30% faster than storing the angle.
- **Target path:** the lane it follows, or, during a lane change, an offset from it along a cubic (see Lane changes). The controller follows the offset, so the trajectory looks natural without being scripted.
- **Staying in lane:** outside a lane change, a vehicle's body should stay within its lane's edges. This is left to how vehicles are steered, not checked at runtime, which would cost a check per vehicle per neighbour. How to guarantee it is an open question.

## Longitudinal model

IDM+ (Schakel, van Arem and Netten, 2010) computes each vehicle's acceleration every `accel_every` steps, behind a module boundary. IDM+ splits IDM into a free-road term and an interaction term and takes the minimum instead of the sum, which gives a more realistic capacity and extends naturally to several leaders: the acceleration is the running minimum over the free-road term and one term per constraint, computed as they are found, with no list built.

- **What it reacts to:** the leaders along its path (below), the end of its known path, lower speed limits ahead, its old side while straddling, junction and merge terms (see those sections), synchronisation and the room it keeps while wanting to change lanes (see Lane changes), a ghost it makes room for, a speed cap, and the first barrier on its path it hasn't committed to (see Barriers and signals).
- **Which leaders:** a vehicle in the lane that this one can't pass, B, blocks the lane's whole width, so nothing behind it on a shared path can reach anything beyond it that B can't. So once the search finds B, it skips the in-lane vehicles beyond it, on B's lane and on the lanes after it, as far as B's path goes along this one's. This is one high-water mark, the first lane of the path that no in-lane leader covers. A vehicle converging onto the lane can't be passed and shields like any other. Ghosts only partly block a lane, and B may pass a vehicle diverging from the lane that this one can't, so neither is skipped, and neither shields anything. Each vehicle found is its own term, so a car whose leader turns off sees the queue beyond. Treiber's Human Driver Model reacts to several leaders too.
- **Horizon:** it looks ahead as far as s*, the desired gap approaching a stopped leader, `s0 + v·T + v² / (2·√(a·b))`, where a stopped leader's term is zero, or 5 m if that is further, so a leader in a queue at rest doesn't flicker in and out of view. A leader followed at its own speed, `s0 + v·T` behind, is always in view. One coming into view can change the acceleration abruptly, much as a driver catching up with stopped traffic brakes firmly at first; SUMO instead looks as far as braking distance, and Treiber's simulator looks at every leader. A vehicle that wants to change lanes looks at least as far as the room it keeps (see Room to finish).
- **End of the known path:** a stop line, a stationary virtual leader, unless the vehicle leaves the network there. Paths reach past the horizon, so it only binds where routing doesn't know the next lane.
- **Visiting order:** lane by lane, from the front of each lane's occupants back, each vehicle's search starting just after its own record, on the lane it follows, or, while it straddles, on its old lane. On a loop shorter than its horizon a vehicle can find itself ahead, which is geometrically true. SUMO plans each lane's vehicles from the front back in the same way.
- **Gaps:** a searching vehicle's rear is its record's s, and its front its length ahead, so every gap is a subtraction along the lanes searched.
- **Desired speed:** the lane's speed limit times the vehicle's speed factor, capped at its maximum speed, as SUMO's `speedFactor`; the factor's distribution is the caller's concern (SUMO's default: normal, mean 1, deviation 0.1).
- **Above the desired speed:** the free-road term is the Improved IDM's, `−b·(1 − (v0/v)^(aδ/b))`, slowing toward the desired speed at no more than comfortable deceleration, where IDM's would brake as hard as it could well above it.
- **Lower limits ahead:** a lane ahead with a lower limit is its own term, `(v_lim² − v²) / 2d` for one `d` ahead, once that is harsher than comfortable braking, and capped at twice that.
- **Ghosts:** the search reports every ghost it meets and the caller decides what one means. Checking a gap for a lane change, and in `try_insert`, a ghost is a leader, since a vehicle already indicating into the gap goes first. In the longitudinal model, the nearest ghost before any leader is one the vehicle may make room for.
- **Relaxation:** a vehicle that has just appeared in a lane's traffic, or that another has just appeared in front of, keeps a shorter time headway at first and drops back gradually, as in LMRS (Schakel, Knoop and van Arem, 2012). A record's age starts at zero when the vehicle changes lanes into the lane, enters a merge onto it or joins it from beside it, carries over from one lane to the next, and starts settled for an inserted vehicle. A leader term keeps `h(a)` of the follower's time headway, with `a` the younger of the two records' ages and `h` rising from LMRS's `T_min/T_max`, 0.56/1.2, back to 1 exponentially with a time constant of 25 s. The minimum gap stays whole.
- **Acceleration noise:** drivers don't hold a speed exactly. Each has a slowly varying noise added to its free-road term, an Ornstein-Uhlenbeck process with a time constant of 10 s and a per-driver spread (`accel_noise`), from its own seeded generator, so runs repeat, as in Treiber and Kesting's IDM with acceleration noise (2017). Following stays exact, so the noise never brings vehicles closer than IDM would.
- **Friction:** a driver goes at most 30 km/h faster than the traffic ahead in the lane beside it, the average over the next 8 s at that lane's limit, if at least three vehicles are there: the friction effect Jang and Cassidy (2012) measured beside congested lanes.
- **Parameters** come from each vehicle's attributes. `VehicleSpec::passenger_car` gives textbook values: IDM's as calibrated with LMRS, as OpenTrafficSim uses them, a = 1.25 m/s², b = 2.09 m/s², s0 = 3 m, T = 1.2 s, and an acceleration noise of 0.2 m/s².

## Lane membership, lane changes and diverging

How vehicles keep apart while they change lanes, merge and diverge, and when one may pass another, is described in [Multi-lane car following](multilane-car-following.md). This section covers the lane-change mechanics it relies on, and how drivers decide.

- **Where changes happen:** only into a neighbour the network permits changing into, and only where the change would be over, the whole body in the new lane, before the lanes beside the vehicle's path run out: about the lookahead, the change's length and the vehicle's length. The lanes beside run out at a junction too: no change starts in a junction or runs into one, as SUMO allows none on internal junction lanes. One that did could stop the vehicle at a conflict partway across, where its search of its old lane could wait on a vehicle that waits on it.
- **The decision is a jump:** when a vehicle changes lanes, the lane it follows switches to the target at once, and it joins the target's occupant list, its ghost becoming its record. Its pose doesn't move; it is simply off-centre on its new lane, and steering brings it onto the centreline.
- **Lateral path:** at the start of a change the vehicle fixes a cubic in u = s − s₀ along its new lane, from where pure pursuit was aiming on the old path, so the target point doesn't jump: offset t₀ and slope m₀ there, to zero offset and slope at u = L, `t(u) = t₀(2x³ − 3x² + 1) + L·m₀(x³ − 2x² + x)` with x = u/L. L = max(4 s at its speed, L_min), so peak lateral acceleration is about 1.3 m/s² across a 3.5 m lane at speed. L_min is the shortest cubic it can steer, √(6·|t₀| / κ_max) with κ_max its steering lock's curvature: about 9 m across a lane for a car. If the vehicle advances mid-change, s₀ shifts back by the old lane's length. A change is complete at the end of its cubic, and no other starts until then.
- **Old side:** the vehicle remembers which side its old lane is on, and maps its path to the old lanes beside it. While it straddles, it runs a second leader search from its record on the old lane, as far as the change's end, so it doesn't run into its old leader; its terms join its own. A vehicle that advances while straddling also joins its old lane's successors, so followers there see it. It straddles while it has a record on the old-side neighbour of the lane it follows, or of the lane before it, just after a section boundary.
- **Passing checks** are made as [Multi-lane car following](multilane-car-following.md) describes, the follower's nose placed at the other vehicle's rearmost corner along its own lane, or, along its old lanes, projected onto its own lane through world space.
- **`change_lane`** executes a change directly, with no safety check, so it can drive a vehicle into another. It is the execute half of the lane-change model's decisions and a testing hook.

**Lane changes: LMRS.** Drivers decide as in LMRS, the Lane-change Model with Relaxation and Synchronization (Schakel, Knoop and van Arem, 2012), with its published parameters. At each update every vehicle weighs up both sides cheaply, and only one whose desire has reached its threshold checks the gap, which takes searches.

1. **Desire** to change to each side, from −1 to 1, is the sum of incentives, each in units of desire:
   - **Route,** `d_r`: toward the neighbour from which its route needs fewer lane changes, by its route table (see Routing). For each change still needed, it rises linearly from 0 to 1 toward the point by which that change must be done, by distance (295 m per change to make) or time at its speed (43 s), whichever is higher, and the most urgent counts. A lane that ends by merging and is left early counts as ending there, so vehicles move out of it along the way. **Built so far:** only the lane left early, toward the neighbour that goes on.
   - **Speed,** `d_s`: how much faster traffic goes ahead in the lane beside it than in its own, over 69.6 km/h. Each lane's speed is the average of its vehicles over the next 8 s at its limit, no faster than the driver would like, and as fast as it would like where nobody is: an average, not the leader alone, so a driver moves over before its own lane has stopped. Each lane's speeds are summed in order once per update, so an average is two binary searches.
   - **Keeping to a side,** `d_b`: the free threshold toward the side the lane says to keep to.
   - **Courtesy:** letting by its old follower, by what that follower would gain, weighed by its politeness, in units of its comfortable deceleration.

   As LMRS has it, `d = d_r + θ·(d_s + d_b + d_c)`: the voluntary incentives count fully unless they work against the route, and not at all once the route's desire is strong enough to cooperate. The desire follows the incentives smoothed over 1 s, so a fleeting gain doesn't make a driver change.
2. **Thresholds:** at `d_free`, 0.365, the driver checks the gap and, if it lets it, indicates, and changes after indicating for at least 0.5 s. At `d_sync`, 0.577, it indicates even without a gap, asking for room, and synchronises: it adjusts its speed to the leader it would have in the target lane, its headway shortened by its desire, no harder than comfortably. At `d_coop`, 0.788, followers in the target lane make room for it, and it changes even without room to finish. It cancels once its desire falls below half its threshold, or no permitted neighbour remains on that side.
3. **Gap acceptance:** the gap is checked as if the driver were already settled in the target lane, level with where it is: its own acceleration there, against its leaders and any ghost ahead, and its new follower's with it as a leader, found by searching back. Both keep their time headways shortened by its desire, from the whole at no desire to `T_min/T_max` at a desire of 1, and the change is accepted if neither brakes harder than the desire times the driver's comfortable deceleration, raised toward b_safe (4 m/s²) by its assertiveness.
4. **Spreading the checks:** a refused check waits a few updates before the next, 4 (0.2 s) scaled by `1 + politeness − assertiveness/2`, so drivers' checks don't fall in step; at most `max_gap_checks` (64) are made in one update, a backstop, and the rest wait for the next. A vehicle with no lane it may change to and no desire left is skipped outright.

- **Searching back:** from a point on a lane, back through that lane's occupants, then the ends of the lanes leading into it, into each only if some vehicle's path through it reaches the lane searched from. Each lane keeps how far beyond its end the paths through it reach: raised whenever a path grows, so never short, and made exact again at each lane-change update. Going back round a loop, the distance only grows, so the search ends there too. On the lane searched from, every vehicle but a ghost counts; on lanes leading in, one counts if that lane is on its path. The first found is the follower if the point is within its horizon. This is how SUMO and Treiber's MOBIL find the follower on the target lane, and `try_insert` finds its follower the same way.
- **Nobody slips between:** the leader search ahead and the follower search behind split each lane at the vehicle's rear, so a vehicle alongside is one or the other, with a gap of zero or less, and refuses the change.
- **Room to finish:** a vehicle doesn't change lanes unless its rear axle would be past the cubic's midpoint before it had to stop behind its leader: the gap less its minimum gap, plus the leader's stopping distance at its comfortable deceleration. Otherwise it would be left straddling both lanes, blocking them. It may indicate without the room. While it wants to change, indicating or with half its threshold of desire, it keeps room behind its leaders to get halfway through a change from rest, so it isn't boxed in: a separate term, with that room as its minimum gap, which IDM settles on from above, never braking harder than comfortably, since it is a preference, not safety, which only binds in slow traffic. VISSIM has a minimum front clearance for changing lanes from a standstill, and drivers are taught to stop where they can see the tyres of the vehicle ahead.
- **Assertiveness:** drivers vary in how readily they change lanes, from 0 to 1, as SUMO's `lcAssertive` and `lcSpeedGain`: an assertive driver's thresholds are lower, by up to half, it accepts harder braking, and it waits less after a refusal. A few assertive drivers pull out into a faster lane and slow it, so a fast lane doesn't form, and the cautious follow. Its distribution is the caller's concern; the UI draws it as a uniform number cubed, about one driver in ten above 0.7.
- **Making room,** LMRS's cooperation: a follower whose search reaches a ghost before any leader may make room for it, treating it as a leader, if its owner's desire has reached `d_coop`. It starts if that takes no more braking than its courtesy, a per-driver attribute (0 never yields), and once it has chosen to, it keeps making room up to b_safe until the ghost is gone. Typically a few nearer followers drive on and a later one opens a gap. This is Hidas's (2002, 2005) cooperative lane changing, and SUMO's `lcCooperative`.
- **Two vehicles into one gap:** a ghost ahead counts as a leader in a vehicle's own gap check, but the follower search passes ghosts over, so of two indicating into the same gap, the one behind waits.
- **Across section boundaries:** when the vehicle advances, its ghost moves to the neighbour of its new lane, or, with none it may change into, the intent is cancelled.

**Module boundary:** the decision making lives in the lane-change model; the rest of the simulation sees only each vehicle's indicator and an opaque per-vehicle state. The simulation measures each driver's incentives and hands them to the model (`aim`), the model moves the desire and says whether it wants a gap checked (`step`), the simulation checks it by searching the network, and the model says what to do (`resolve`): nothing, indicate, cancel or change. Making room works the same way. This is SUMO's split between its lane changer and its lane-change models.

## Conflicts, merging and junctions

Lanes that cross or merge inside an intersection are grouped into a junction, and a junction keeps vehicles on its lanes apart. A lane belongs to at most one junction. Junctions are a dumb backstop: they keep vehicles that can no longer stop from running into each other, and nothing more. The order vehicles go in is for barriers (see Barriers and signals).

- **Conflicts:** a junction stores, for each ordered pair of its lanes, where they conflict, if they do: the kind (merge or cross) and the overlapping s-range on each lane, in a dense `lanes²` table, so a lookup gives "my" range and "theirs" directly. Lanes leading into the same lane merge; branches of a fork, lanes one after the other and neighbours never conflict, since the multi-lane following model keeps vehicles on them apart; any other pair whose outlines overlap crosses.
- **Finding overlaps:** each segment's piece of a lane's outline is a convex quadrilateral, and every pair of pieces from two lanes is clipped one against the other (Sutherland–Hodgman). Each lane's s-range runs from the first overlap to the last. An overlap thinner on average than rounding to the grid leaves doesn't count, so lanes touching along an edge don't conflict. A merge whose outlines don't overlap gets the lanes' ends.
- **One queue per junction,** part of the state and of snapshots: the vehicles **committed** there, which can no longer stop short of their conflicts, in the order they committed, and the lanes vehicles not yet committed are **approaching** it along.
- **Arriving:** a vehicle arrives at a junction once a lane of the junction is on its path and no other junction it hasn't entered lies before it, so when its path is extended or rebuilt, or it enters a junction. It adds the lanes of its path up to the first of the junction's lanes with a conflict to those the junction is approached along. The junction keeps no list of who is approaching, only where. A barrier before the junction on its path that it hasn't committed to holds it back too: it arrives once it commits, so vehicles waiting at a red light or a stop sign aren't approaching.
- **Committing:** every step, the junction scans back from the start of the conflict on each of its lanes being approached, along that lane's occupants and the lanes leading in that it is approached along, passing over ghosts, committed vehicles and vehicles whose path has left the way. At the first other vehicle: one headed onto the lane commits once it can no longer stop at `SAFE_BRAKING` (4 m/s²) before the conflict, or its front is past its start, and the scan goes on behind it; one that can still stop, or is headed elsewhere in the junction, stops the scan. So nobody commits before a vehicle ahead of it on its lane that might still go in, which has to be enforced, since IDM only roughly keeps a follower able to stop. Lanes a scan finds no one on, nor beyond, stop being approached. `SAFE_BRAKING` rather than comfortable deceleration keeps the commit order close to the order vehicles reach their conflicts; it cut hard braking by about four fifths at the ramps.
- **Releasing:** every step, a committed vehicle that could now stop at `SAFE_BRAKING` a car length short of its conflict is released, unless one committed after it is behind it on its way in. The car length is the hysteresis against committing.
- **Leaving:** a vehicle leaves the queue once no lane of the junction is on its path and it has no record on any of them, or when it is removed.
- **Keeping apart:** at each longitudinal update, the junction walks its committed vehicles and gives each follower a term from each conflict with each vehicle committed before it: until the one ahead has cleared the conflict on its lane, the gentlest of three ways of keeping out of its way, each safe on its own:
  - **Stopping short** of the conflict.
  - **Following its progress:** a virtual vehicle at the speed of the one ahead, as far past the start of the follower's conflict as that one's rear is past the end of its own, so it reaches that start just as the other clears: virtual vehicle mapping (Uno et al., 1999). Crossing streams roll through one behind another rather than stopping at the line.
  - **Following it,** while it moves the same general way as the follower's conflict runs, now and as it leaves its own: its nearest corner along that direction, at its speed along it. A convex body is nowhere nearer along a direction than its nearest corner, and checking its heading as it leaves too stops a vehicle turning across from leading the follower in and then swinging back.

  Between committing at `SAFE_BRAKING` and IDM's limit, `MAX_BRAKING` (9 m/s²), lies the backstop's margin, so two vehicles committing at once still stop apart. A follower past the start of its conflict is held back all the same: at a merge, dropping back is the only way out.
- **Vehicles at the front of each way in,** not yet committed, take terms from every committed vehicle too, so they slow gently for one in their way rather than braking hard once they commit.
- **Barriers inside a junction:** a committed vehicle claims no conflict beyond the next barrier on its path that it hasn't committed to, since it will stop there, so a turner waiting inside the junction holds back nobody on the lanes it waits for. When it commits to such a barrier, it moves to the back of the queue, behind everyone committed before: it goes now, not when it first committed.
- **Lane changes:** none start in a junction or run into one, so a vehicle is never partway across a junction's lanes. A committed vehicle that changes lanes on its way in keeps its place.
- **Deadlock:** waiting vehicles in a cycle are all stopped, and a stopped vehicle more than a car length short of its conflict is released, breaking the cycle. One can survive only among vehicles within a car length of their conflicts, which needs a vehicle cutting in just ahead of a committed one close to a merge, with another committing between them; it is watched for, not prevented. A vehicle can still be blocked by traffic beyond the junction: **gridlock** is allowed.
- **Cost:** per junction, O(c²) pairs at 20 Hz for c committed vehicles, tens at most, and each step a scan past them on each lane approached.

**Off the beaten path:** SUMO resolves junctions per link, from foe matrices and each vehicle's announced arrival time, with no global order. One first-come, first-served queue per junction, of the vehicles that can no longer stop, is closer to Dresner and Stone's AIM reservations and A/B Street's intersection queues, with barriers deciding when each vehicle commits.

## Merges

Where lanes outside junctions that were apart start to overlap, as at a lane drop or where two roads' lanes meet one, a merge orders the vehicles approaching and opens gaps between them before they meet. Inside the merge, vehicles keep apart by following each other, which keeps them from colliding but not from arriving side by side and braking hard. A merge's only job is to stop that. Merges are kept apart from junctions, in their own module: a junction orders vehicles once they can no longer stop, and forbids lane changes; a merge orders them early and lets them change lanes on the way in.

- **Which lanes:** a group of lanes outside junctions that overlap one another, joined pair by pair, where the two lanes of each pair have no predecessor in common and no two of their predecessors overlap: the boundary where disjoint lanes start merging. Its merge point is the start of its lanes. Merges are derived when the simulation is built, so they aren't part of a snapshot.
- **The order:** the vehicles approaching, by the distance from each one's rear to the merge point, nearest first. Nothing is stored: at each longitudinal update the order is worked out by walking back through the lanes leading in, as far as a path reaches, and marching back along them all at once, as the merge step of a merge sort does, taking the nearest of the lanes' next vehicles each time. A lane's vehicles count if they follow it and their path leads into the merge, so a vehicle is in the order once a merge lane is on its path, well before the merge, and leaves it once it is on a merge lane, where its followers find it through their records. A vehicle that pulls ahead goes first at once, as on the road; close to the merge point the terms keep vehicles from being level, so the order doesn't flicker.
- **By distance, not estimated arrival:** a vehicle's leader in the order is then never further from the merge point, which the term's slack covers. By estimated arrival, a fast vehicle could come before a slow one well ahead of it, which would brake as hard as it could to fall in behind a leader behind it, and a stopped vehicle's arrival never comes, so a queue locked up. A fast vehicle instead gives way to a slower one just ahead, gently, as drivers do.
- **The term:** each vehicle F whose front hasn't reached the merge point follows the one before it in the order, L, by IDM, as a virtual vehicle on F's lane as far from the merge point (Uno et al., 1999), with the headway it will keep just after merging, LMRS's `T_min/T_max` of its own. Slack is added to the gap, enough for F to be level with L at first, shrinking linearly to nothing at the merge point from 8 s before it at F's speed, or 50 m if that is further: so the gap opens gradually as both draw near, and queues go through by turns, each vehicle moving off as soon as its leader is clear. With less than 50 m, crawling queues reach the merge point side by side and go through by turns from rest, a third less getting through. The term never brakes harder than comfortably: before the merge point the lanes are apart, and inside the merge, following keeps vehicles apart however they arrive.
- **Cost:** at 20 Hz, a march along the occupants of the lanes leading into each merge, and one term per vehicle in it.

**Off the beaten path:** SUMO orders vehicles at its `zipper` junctions by estimated arrival, and Treiber's simulator treats an on-ramp as mandatory MOBIL lane changes. Ordering by distance and following a virtual leader is the usual pairing in cooperative merging (Uno et al., 1999); the shrinking slack is this simulator's own.

## Barriers and signals

Stop lines, give-way lines, signals and the starts of keep-clear zones are all barriers: a point on a lane, with a kind. A vehicle treats the first barrier on its path that it hasn't committed to as a stationary leader at the line, with its own gap to lines as the minimum gap, so it stops a little short of the line, until it commits to crossing; from then on the barrier isn't there for it. The gap is a driver parameter, `line_gap`, about 1 m, varied from driver to driver so a row of vehicles at a line doesn't stand in perfect step. This is how Treiber's simulator treats a red light, and how SUMO's link states hold vehicles at a junction. **Built:** signals, give-way lines and their barriers, commitment, room beyond and holding back arrival at a junction. **Planned:** the other kinds.

- **Commitment** is a count: how many of the barriers ahead of its rear axle on its path it has committed to, in path order. Passing a barrier takes one off, and a lane change or a new path wipes it, since its barriers are then others. A count rather than a flag lets it commit to two at once where it couldn't stop between them, such as a stop line and a waiting point just beyond it.
- **Only the head decides:** the leader search finds a barrier as it finds a leader, and an in-lane leader before it that hasn't committed to it shields it, as it would a vehicle beyond: that leader will stop there, so the follower stops behind it anyway. A leader that has committed to it, the barrier among the first of its count ahead of its rear axle, shields nothing, so the follower sees the barrier and decides for itself when it gets there. So the head for a barrier is the first vehicle on its way that hasn't committed to it, with no queue stored. Each barrier found carries its rank among those ahead of the vehicle, so one beyond a barrier it can't see, shielded, isn't decided out of turn. A vehicle's own committed barriers are found too, for release (below).
- **When it commits,** at each longitudinal update, decided from a read-only view of the state and applied after:

| Barrier          | Commits when                                                                                                                     |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| Signal           | The light is green and it can no longer stop at its comfortable deceleration a line gap short of the line; never on amber or red. Until then a green light, with room beyond (below), exerts nothing, and an amber or red one is a stop line. |
| Give-way line    | There is a gap (see Gap acceptance and priority)                                                                                 |
| Stop line        | It has come to rest at the line, and there is a gap                                                                              |
| All-way stop     | It has come to rest at the line, and no vehicle at a barrier of the junction whose connector conflicts with its own came to rest there earlier |
| Keep-clear start | There is room beyond the zone's far end for its length and minimum gap behind the nearest vehicle there                          |

- **Amber:** a vehicle past the point where it can stop comfortably has committed already, on green, and goes on; the rest stop. As the old prototype had it, nothing commits on amber or red, so a vehicle braking for a line can't creep up to it and then go. That stretch is the dilemma zone ITE's amber time covers at the limit; a driver well over the limit, committing further out, can reach the line just after it turns red, as on the road.
- **Room beyond:** every barrier into a junction is also a keep-clear start, its zone the junction lane: a vehicle doesn't enter unless there is room for it on the lane after, as the road rules require. A queue can still back into a junction from beyond it, so **gridlock** stays possible.
- **Coming to rest** at the line, as the head with the barrier first ahead, records the step it happens, for an all-way stop's arrival order. Vehicles that came to rest in the same step go in the barriers' order in the junction, so runs repeat.
- **Passing:** each step, a committed vehicle takes one off its count for each barrier its rear axle passes.
- **Release:** a committed vehicle slowed short of a barrier, by traffic ahead or a junction, may no longer need to go. At a signal that is no longer green, one that can still stop comfortably before the line is released from it, and from those after it, and decides afresh, as a driver who could stop does when the light changes; at a give-way line, one that has come to rest short of the line is, since the gap it took is stale. One that committed as the light turned amber can't stop comfortably, and stays committed.

**Turning on red** (**Planned**), as in North America and some of Europe: a movement may allow turning on red, so on red its signal barrier is a stop line followed by a give-way line, the vehicle coming to rest at it and then taking a gap (see Gap acceptance and priority), as SUMO's link state for a right turn on red has it. Traffic held at its own red counts for nothing in the gap, as everywhere. Compilation allows it on the near-side turns of a signalised intersection whose network says so, the default where traffic drives on the right, and on any movement a plan names, which covers turning left on red from one one-way street into another. It needs the stop line's coming to rest, so it comes with the other barrier kinds.

**Signals** (**Built**) run fixed-time plans, behind a module boundary (`models::signal`) so actuated control can come later. A controller is part of the network, and its state, the phase it is in and how long each movement has been in its state, is part of the saved state. The movements' lights are one flat array in the state, every controller's together, each controller writing its own; a signal barrier holds its movement's index into it, so reading its light is one lookup and nothing is copied.

- **Movements:** each is red, amber or green, with its amber and clearance times and a bitmask of the movements it conflicts with.
- **Phases:** each is a mask of movements to be green, with a duration counted from when all of them are green, so clearance doesn't eat into green time.
- **Changes:** a movement not green in the current phase turns amber, then red after its amber time; one green in it turns green once every conflicting movement has been red for that movement's clearance time. When the phase's duration has passed, the next phase begins.
- **Update:** each step computes every movement's next state from the current states, then applies them all. Times are counted in steps, so they never drift.

This is how the old prototype ran its signals.

## Gap acceptance and priority

At a give-way or stop barrier, the head goes only if no vehicle with priority over it could reach its conflicts in time. Priority comes from the connectors' levels (see Intersection control): on its lanes in the junction, up to the next barrier on its path, a vehicle gives way to every conflicting lane of a higher level, and to one of the same level whose vehicle has committed already, so that of two at the same level, the first to commit goes first.

- **The gap:** it goes if it can clear each conflict before any priority vehicle could reach it, plus 0.5 s, both accelerating as hard as they can up to their desired speeds. Covering d from speed v at acceleration a up to v_max takes `(√(v² + 2ad) − v) / a`, or, once at v_max, `t₁ + (d − d₁) / v_max` with `t₁ = (v_max − v) / a` and `d₁ = (v + v_max)·t₁ / 2`. It clears a crossing once its rear is past it. At a merge it must also stay ahead of the priority vehicle by that vehicle's time headway, so the vehicle behind needn't brake. From rest, a car turning across a 50 km/h road waits for about 6 s, close to the 6–7 s critical gaps the Highway Capacity Manual gives for turns from a minor road. SUMO judges gaps the same way, from foes' arrival times.
- **Finding priority vehicles:** from the start of each conflict, back along the priority lane's occupants and the lanes leading into it, as the follower search goes (see Searching back), and only into lanes some path through reaches the conflict. On each branch only the nearest vehicle headed onto the lane matters, since those behind it there arrive later; vehicles whose paths turn off are passed over, unless one waits in the lane at a barrier it hasn't committed to, such as a turner waiting for its own gap: nobody behind it can come until it goes. Without that, opposite turners would each wait for the queue held up behind the other. A vehicle that a red or amber light holds short of the conflict holds back everyone behind it, so its branch is done: it hasn't committed, and nothing commits on amber or red, so it will surely stop. That keeps a red light's queue from blocking an unprotected turn. A vehicle at rest at a green light, or at a give-way or stop line, may yet go, so it counts. Give-way lines can't deadlock, as a vehicle only gives way to higher levels, or to a vehicle of its own level that has committed. A committed vehicle still in a conflict refuses the gap outright.
- **Unprotected turns,** turning across oncoming traffic on a green with no arrow (filter turns, in Australian usage), are this with the barrier inside the junction: the turner waits at its first conflict with oncoming traffic, holding back nobody there (see Barriers inside a junction), and goes through a gap. Oncoming traffic held at its red light doesn't count, so a protected turn goes freely. A hook turner waits inside the junction too, but at a signal barrier, for the road it turns into.
- **Cost:** only heads at give-way and stop barriers check, at 20 Hz, with one backward search for each conflict with a lane of higher priority.

## Routing

Each vehicle carries a `Route`, given at insertion, which chooses the next lane when its path is extended past a lane with several successors; the choice is then part of its path. Route models are separate, in `models::routing`, each replaced by editing it.

- **Random:** a SplitMix64 generator seeded at insertion, drawn from only at forks, so the same seed takes the same way, and snapshots and replays reproduce it. It chooses only among its own lane's successors that its class may use.
- **Destination** (**Planned**): toward a destination, a set of lanes registered with the simulation. Each destination has a route table for each vehicle class sent there, derived when it is first needed outside a step, at registering or inserting, and rebuilt on restore:
  - **Time to the destination** from the start of every lane, at the speed limits, by Dijkstra's algorithm backward from the destination, over the lanes the vehicle's class may use. A lane change costs no time, since the vehicle travels on while changing, but a way that needs more changes than there is room for isn't a way: each change needs its own length (see Lateral path) at the limit, before the lanes it needs run out. Between ways equally quick, the one with fewer changes wins.
  - **Choosing:** at a fork, the successor with the least time, ties going to the first, so runs repeat. A vehicle that misses its way, failing to change lanes in time, simply takes the best way from where it is, so rerouting needs nothing more. One with no way left drives on and leaves wherever its lanes take it.
  - **Lane changes needed:** for each lane, how far along it a vehicle can go and still reach the destination with one, two, three or four lane changes, four or more counting as four: the old prototype's lane distances, as SUMO's "best lanes" does it. They are worked out backward from the destination over the lanes from which the time only falls: a lane that leads nowhere toward it must be left by its end, and a neighbour's distances count for one more change. The route desire reads them (see Lane changes).
  - **Cost:** five numbers per lane per destination, 20 bytes.

Every route's lane-change desire comes from a table: its destination's, or for random routes one derived for the network as a whole, where every lane leads somewhere and only lanes left early must be left by their end.

## Demand (Planned)

The `demand` crate places vehicles in the simulation and gives them routes, through the external API; the core knows nothing of it.

- **Flows:** a flow per entry lane, in vehicles per hour, with exponential headways (Poisson arrivals) from a seeded generator, as SUMO's flows can have. A vehicle that finds no room waits, in order, at its entry, and is inserted with `try_insert` as soon as there is room, so demand above capacity queues outside the network rather than vanishing, as SUMO's insertion backlog does.
- **Origin–destination demand:** flows from entries to destinations named by road ends, each vehicle drawing its destination from its entry's share. Without destinations, routes are random.
- **Closed networks:** a target count, kept by inserting into the largest gap and removing at random, as the UI does now on the ring.
- **Vehicle mix:** each vehicle's class drawn from a mix, then its driver from distributions: a speed factor normal with mean 1 and deviation 0.1, SUMO's default; assertiveness a uniform number cubed; courtesy and politeness uniform; and its gap to lines about 1 m.
- **Trams** run to a timetable on their routes, a destination each, rather than from flows.
- The UI's spawning moves here, and the UI uses the crate. Demand's generators aren't part of the simulation's state, so a freeze frame holds no demand, and a replay records the vehicles inserted, not how they were drawn.

## Vehicle classes (Planned)

Each vehicle has a class, a small enum: car, truck, bus or tram. A class restricts which lanes the vehicle may use, as SUMO's vehicle classes and lane permissions do, and has a `VehicleSpec` preset, its dimensions and driver parameters from published defaults (OpenTrafficSim's and SUMO's for those vehicle types).

- **Lane use:** each lane has a set of classes allowed on it, a bitmask. A vehicle's path only takes lanes its class may use, its route chooses only among them, and it changes lanes only into them. Classes change nothing else: the leader and follower searches, merges and junctions see every vehicle on a lane, allowed or not, since every body is in the way.
- **Trams** run on tram tracks, lanes only trams may use or shared with general traffic, and never change lanes. A tram follows its track's centreline exactly, its pose set from the lane rather than steered, since rails leave no room to cut a corner. It is drawn articulated, its sections following the track.
- **Tram stops** are barriers that only trams see, with a dwell time: a tram stops at one, waits there for the dwell, then commits. A stop in the road, with no platform between the tram and the lanes beside it, also puts a barrier on each lane beside the track, level with the rear of a tram at the stop, which only other classes see and which holds them while a tram dwells there, as Victoria's road rules require.

## External API

Demand lives outside the `sim` crate. The simulation exposes:

- **Making and stepping:** `Simulation::new` or `with_config`, `step`, `step_variable`, `alpha`, `time`.
- **Inserting:** `insert` places a vehicle on a lane's centreline with no checks, for tests and scripted setups. `try_insert` refuses when there is no safe room: when the new vehicle would overlap another, or it or its new follower would have to brake harder than `SAFE_BRAKING` because of the other; its speed is capped at its new leader's. Its leader and follower come from the same two searches a lane change uses, split at its rear.
- **Destinations** (**Planned**): `add_destination` registers a set of lanes and returns its id, for a destination route.
- **Reading:** a vehicle by key, every vehicle, each lane's occupants, each junction's queue and the lanes it is approached along, the network, each signal movement's light (`light`) and each vehicle's commitment. `vehicle_at` picks the vehicle nearest a point, within a tolerance, by a linear scan.
- **Changing:** `remove`; `change_lane`; `cap_speed`, under which a vehicle brakes at `SAFE_BRAKING` while above the cap and eases up to it from below, `hold` (a cap of 0) and `release`. Capped vehicles are kept in a short list, not as a field on every vehicle, and the cap is one more acceleration term.
- **Snapshot and restore** the whole state, network included, for freeze frames: `state` lends the `SimState`, to clone or, with the `serde` feature, serialize, and `restore` builds a simulation from one. A restored simulation continues bit for bit until it inserts a vehicle into a slot freed before the snapshot: `SlotMap` rebuilds its free list in index order, so that vehicle's id, and the update order after it, can differ.
- **Heterogeneity** comes from inserting vehicles with varied attributes. Distributions are the caller's concern, not the core's.

## Validation (Planned)

Visual realism comes first, but traffic flow mustn't break down in ways real traffic wouldn't, so a headless harness measures the simulator's aggregate behaviour against published values. It is a crate of its own, `validation`, depending on `sim`, `network` and `demand`, run on demand rather than as a test, and printing each measure beside its reference.

- **Measuring:** virtual detectors outside the core, counting the vehicles crossing a point and their speeds, and timing vehicles between points, from the vehicles' poses after each step, on scenarios built for each measure.
- **What it measures, against what:**
  - A freeway lane's capacity: about 2,400 veh/h at 120 km/h (Highway Capacity Manual).
  - A merged lane's throughput: 2,000–2,400 veh/h (see Merge capacity).
  - Capacity drop once a bottleneck breaks down: discharge a few percent to about 20% below the flow before it (Hall and Agyemang-Duah, 1991; later studies).
  - Stop-and-go waves: travelling upstream at about 15 km/h, whatever the traffic (Treiber and Kesting), and forming on a ring at Sugiyama's density.
  - Jam density: about 130–150 vehicles per km per lane.
  - Saturation flow at signals: about 1,900 veh/h per lane, about 1.9 s between vehicles once a queue is moving, with about 2 s of start-up lost time (Highway Capacity Manual).
  - Gap acceptance at give-way and stop signs: critical headways of about 4–7 s by movement, and follow-up headways of about 2–3.5 s (Highway Capacity Manual).
- **Reading it:** each measure prints with its reference range, so a change that moves a capacity shows up; ranges are references, not pass or fail. A measure far outside its range is a bug or a model fault to look into, and the reason goes into this document.

**Suggestion:** later, the same scenarios exported to SUMO, at road level through netconvert's plain XML, as a second reference for network-wide behaviour such as spillback and gridlock.

## Debug instrumentation

Algorithms report what they are doing through a debug sink that costs nothing when debug output is off: the simulation is generic over `DebugSink`, and events are passed as closures, `sink.record(|| …)`, so a zero-sized no-op sink compiles away entirely. The UI runs with a recording sink, and benchmarks with the no-op one. A feature flag would have kept signatures simpler but needed a separate build for debugging.

The debug view shows what the simulation actually used, never a recomputation. Events are grouped by the update that produces them, and accumulate in the sink:

- **Resets:** each update first records a reset for its group, "recompute accelerations" and "recompute motion" for everyone, and "recompute decision" per vehicle, which clears that group's earlier events; a removed vehicle's events go with a "vehicle removed" event. The UI never clears the sink itself, and reads only after a step returns, so it always sees the last complete set.
- **Acceleration terms:** each term is its own event, the free-road term and each leader, junction, merge, synchronisation, cap, ghost and barrier term. Every input to the acceleration is recorded, so the binding term is the one equal to the vehicle's acceleration, and one whose terms don't reach it shows an event is missing.
- **Also recorded:** lookahead points, passing checks, with the follower's predicted pose, the other vehicle's nearer rear corner and the gap between them, and lane-change evaluations, with each side's incentives, desire and any gap check. Each barrier decision, with the gap to the line and what the vehicle did: committed, passed for now, or held, with its term. **Planned:** each gap check, with every priority vehicle found, its reach time and the yielding vehicle's time to clear; and each signal change.

## UI

The `ui` crate renders the simulation with egui's painter and offers two views.

- **Pretty view:** road and junction surfaces, lane markings, stop lines and turn arrows from the render geometry, vehicles as oriented shapes drawn between steps, and indicators. Each signal barrier's lane is drawn from the barrier on in its movement's light's colour while green or amber, and left out while red, as the old prototype's visualiser drew them. **Planned:** keep-clear markings.
- **Debug view:** layers for lane outlines (their ends dashed), centrelines, s ticks, neighbours, occupants, vehicles, lane-change paths, leaders and merge orders, passing checks and lookahead points, barriers across their lanes in their lights' colours, and (**Planned**) gap checks. Clicking a vehicle opens an inspector with its state, its lane-change desires, its acceleration terms, its commitment and its last barrier decisions, and (**Planned**) the lane changes its route still needs.
- **Controls:** pause, single step, speed multiplier, camera pan and zoom, scenario choice, traffic (a target count on the ring, a flow per entry lane elsewhere, and (**Planned**) origin–destination demand), lane changes for the selected vehicle, and saving a replay or a freeze frame. Signalised crossroads are a scenario. **Planned scenarios:** priority crossroads, an all-way stop, a Melbourne crossroads with tram tracks and hook turns, and a procedural grid.

**Suggestion:** if tessellation becomes the bottleneck at high vehicle counts, draw vehicles and roads through a custom wgpu paint callback with instancing, keeping egui for overlays and panels.

## Hard parts and open questions

The hardest problems are junction geometry and dense merging. Both have defeated simpler simulators.

**Hard parts**

1. Generating junction geometry: connector curves and junction surfaces.
2. Choosing lanes several links ahead of a diverge, and mandatory lane changes in dense traffic.
3. Keeping vehicles on the lane geometry without visible snapping.
4. Inserting vehicles without visible pops.
5. Judging realism by eye, with no metric to check against.

**Open questions**

- **Staying in lane:** how to guarantee that a vehicle's body stays within its lane outside lane changes. Pure pursuit tracks a constant curve exactly and cuts in only where curvature changes: about 5 cm at 50 km/h on a 100 m bend, but more on tight junction connectors that start abruptly.
- **Waiting points:** where an unprotected turner should wait inside a junction, clear of both the traffic behind it and the traffic it gives way to, for any junction geometry. The first conflict with oncoming traffic serves the procedural scenarios. Hook turners' waiting lines come from the road on the near side (see Intersection control).
- **Shared lane edges:** neighbouring lanes each store their own vertices, so their ends match only to the grid. Lanes defined as borders offset from one reference line, as OpenDRIVE's lane sections are, would make them match exactly; nothing needs it yet.
- **Merge capacity:** relaxed headways let a merged lane carry about 2,600 veh/h, above the 2,000–2,400 of real lanes.

## Milestones

Each milestone is a demo that can be finished on its own; anything after it is optional.

1. Workspace skeleton: one straight lane, IDM, kinematics, the pretty and debug views. **Done.**
2. A ring road showing emergent stop-and-go waves, as in the Sugiyama experiment. **Done.**
3. A multi-lane freeway with continuous lane changes and multi-lane membership, first with MOBIL, now LMRS. `sim/tests/scenario.rs` runs a three-lane ring headless as a stress test. **Done.**
4. An on-ramp merge and an off-ramp diverge. Merges, with zipper or early merging, and the diverge's car following are done; taking the off-ramp needs destination routing and its route desire.
5. A single signalised intersection with barriers, a fixed signal plan and unprotected turns. **Done.**
6. Hook turns and trams: tram tracks in the middle of the road, tram stops, and vehicle classes restricting lane use.
7. Unsignalised intersections: priority roads with give-way and stop signs, all-way stops and keep-clear zones; and turning on red at signals.
8. A procedural grid with destination and random routing, origin–destination demand from the `demand` crate, and a mix of vehicle classes. Random routing is done.
9. The validation harness, measuring capacities, waves, saturation flows and gaps against published values.

## References and prior art

These are the established approaches this design borrows from or deliberately departs from.

- Treiber and Kesting, _Traffic Flow Dynamics_: the textbook on IDM, the Improved IDM and MOBIL, by their authors; and Treiber and Kesting (2017), on IDM with acceleration noise.
- Schakel, van Arem and Netten (2010), IDM+; and Schakel, Knoop and van Arem (2012), LMRS, with relaxation and synchronization, as OpenTrafficSim implements it.
- [traffic-simulation.de](https://traffic-simulation.de): Treiber's browser simulator, built on IDM and MOBIL with ramps and roundabouts. Readable source.
- [SUMO](https://eclipse.dev/sumo/): the reference lane-based microsimulator. See LC2013, shadow lanes, internal junction lanes, zipper junctions and "best lanes".
- Uno et al. (1999), virtual vehicle mapping for merging; Hidas (2002, 2005), cooperative lane changing; Jang and Cassidy (2012), the friction effect beside congested lanes.
- [A/B Street](https://github.com/a-b-street/abstreet): an open-source Rust traffic simulator with detailed write-ups on junction geometry and gridlock.
- [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/): road description with s/t lane coordinates and explicit junction connections.
- [Fix Your Timestep](https://gafferongames.com/post/fix_your_timestep/): fixed timesteps with interpolated rendering.
- Hall and Agyemang-Duah (1991), capacity drop at freeway bottlenecks; Sugiyama et al. (2008), jams forming on a ring with no bottleneck.
- Dresner and Stone (2008), autonomous intersection management by reservation; the Highway Capacity Manual (TRB), critical gaps at unsignalised intersections; and ITE's change interval, for amber and all-red times.
- Pure pursuit and Stanley path tracking: standard lateral controllers from autonomous-vehicle research.
