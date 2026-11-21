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
- Where it can, the simulation follows established technique (SUMO, IDM and LMRS literature, game-engine practice). Its deliberate departures are world coordinates as the truth, how vehicles find their leaders across lanes (see [Multi-lane car following](multilane-car-following.md)), the junction backstop and merges; each is marked **Off the beaten path**.
- In this doc, unmarked statements are decisions. **Suggestion** marks proposals that haven't been explicitly confirmed yet. **Planned** marks designed parts not yet built. Open items are collected under Hard parts and open questions.

## Workspace and crates

One Rust workspace. The simulation crate depends on nothing else in the workspace; everything else depends on it.

| Crate                     | Owns                                                                                                      | Depends on       |
| ------------------------- | --------------------------------------------------------------------------------------------------------- | ---------------- |
| `sim`                     | Simulation state, models, update pipeline, public API, and the `SimNetwork` type it consumes             | —                |
| `network`                 | Road-level network format and scenarios; compiles them to a `SimNetwork` and to render geometry          | `sim`            |
| `ui`                      | egui app with a pretty view and a debug view                                                              | `sim`, `network` |
| `demand` (**Suggestion**) | Routing models and vehicle spawners, kept outside the core                                                | `sim`            |

`sim` defines `SimNetwork` and `network` produces it, so the dependency graph is one-way and the core never sees authoring concepts like multi-lane roads.

## Network pipeline

One road-level network description compiles into two products: a `SimNetwork` for the simulation and render geometry for the UI.

- **Source of truth:** built by procedural generators (`scenarios`) or by hand. Real-world import is out of scope. The format will change as it grows.
- **`SimNetwork` output:** lanes as polylines, their successors, neighbours with lane-change permissions, which side drivers keep to, how lanes that end are left, and junctions with their conflicts. **Planned:** barriers and signal controllers.
- **Render output:** road and junction surfaces, lane markings, stop lines and turn arrows. **Planned:** keep-clear markings.
- **Precision:** every lane point and half-width is snapped to a 1 mm grid as the lane is built, so a network serializes exactly as short text.

The `network` crate describes networks at road level, as OpenDRIVE and SUMO's edge/lane/connection model do, and expands them to bare lanes during compilation. That is how people think about roads, and it keeps the sim free of road concepts.

- **Roads:** a reference line, a start pose followed by pieces (lines, arcs and cubic Béziers) that each continue from the last, and lanes beside it, each with an offset and a width that are constant or ease (smoothstep) from one value to another over a stretch, and a direction along the road. Clothoids can come later.
- **Sampling:** curves are sampled by tolerance, not at a fixed spacing: at most 1 cm of sagitta (`c² / 8R`) and 3° of turn per vertex, and where an offset or width eases, enough vertices that it strays at most 1 cm from the smoothstep between them. The vertex next to each end of a road goes on the tangent there, as far out as stays within 1 cm of the curve, so lanes end square across the road.
- **Lanes from one polyline:** a road's lanes are offset from one sampled reference polyline by its miters, so neighbouring lanes end square across the road, side by side, as `validate` requires, and meet the next road's lanes exactly.
- **Neighbours:** lanes of a road running side by side the same way and keeping their distance; lanes easing together, at a lane drop, or apart, are not. Each lane says whether vehicles may change into its neighbour on the left and on the right, as SUMO's `changeLeft` and `changeRight` do; both by default.
- **Keeping to a side:** the road network says which side drivers keep to on roads above a speed limit: in Australia, the left, above 80 km/h. Compilation marks those roads' lanes.
- **Lanes that end by merging,** as an acceleration lane does, say how drivers leave them: at the merge, taking turns (a zipper), or along the way (early). The ramps scenario offers both.
- **Joins:** a road's end joins the next road's start, and lanes connect wherever one ends where the other starts: two meeting one merge, one meeting two diverge, with nothing declared. Merging road lanes are not a junction: they are a merge (see Merges).
- **Intersections:** each lists which lanes connect to which, and each connection becomes a cubic Bézier from one lane's end to the other's start along their directions, its control points a third of the way across, its width blending from one to the other, and its speed limit lowered for its tightest bend (2 m/s² sideways, at least 3 m/s). An intersection's connectors are one junction.
- **Render output:** each road's surface between its outermost edges; lane markings, dashed between lanes going the same way, a centre line between opposite ones, and edge lines; and each intersection's surface, the mouths of its roads joined by cubic Bézier curbs.
- **Stop lines:** each lane coming into an intersection gets one across its end, 300 mm wide as AS 1742.2 has it, set back half that so its far edge is where the intersection starts.
- **Turn arrows:** each lane coming into an intersection gets an arrow 8 m back from its end for the ways its connections go, as SUMO classes a link's direction by its angle: straight on within 30° either way, otherwise left or right. It's one of seven glyphs, every mix of left, straight and right, drawn in `ui/assets/lane-glyphs.svg` and converted once to triangles in `ui/src/glyphs.rs` by `ui/assets/convert-glyphs.mjs`, so the UI needs no SVG support. None is painted on a road shorter than 16 m.

Generating junction geometry is the hardest part of this crate. SUMO's netconvert is largely heuristics for it; procedural networks keep it tractable.

## Lane model and geometry

The simulation has no roads, only lanes. Lanes may overlap, merge, diverge or cross, and lane changes may be permitted between them.

- **Geometry:** each lane is a polyline with arc length s and lateral offset t. Each vertex has its own left and right half-widths, so a lane can taper and its centreline needn't be centred; between vertices they vary linearly in s, as OpenDRIVE gives lane width as a function of s. A projection from world to lane coordinates starts from a hint, the last one's result with its segment, and scans from there, so it is close to O(1) for a vehicle that has moved a little.
- **Serialized geometry:** a lane's geometry serializes as a string of vertices in whole millimetres, each `x,y,left,right`; with three values the half-widths are equal, and with two they are the previous vertex's: `0,0,1750 90000,0 100000,2000,2000,1750`. Being on the grid, it reads back exactly.
- **Module boundary:** a lane's geometry, `LaneGeometry`, is its own type in its own module; the `Lane` holds it with its speed limit, keep side, how it is left if it ends, and links, which only the network sets.
- **Relationships:** successors and predecessors (end-to-start connections), kept in order from left to right; left and right neighbours, at most one each side, with permission per ordered pair, since a change may be allowed one way only; and overlaps, found when the simulation is built (see [Multi-lane car following](multilane-car-following.md)).
- **Lane sections:** adjacent lanes start and end together, like the lanes of a SUMO edge or an OpenDRIVE lane section, so the neighbour of a lane's successor is the successor of its neighbour, which lets a vehicle map its path to the lanes beside it. `SimNetwork::validate` checks this, with mutual neighbours and successors starting where their lane ends.

**Off the beaten path:** SUMO, VISSIM and Aimsun treat lane position as the source of truth and derive world position from it. Here world pose is the truth and lane coordinates are derived, as in autonomous-vehicle planners using Frenet frames and in OpenDRIVE's s/t coordinates. The reason is realistic motion through lane changes, merges and junctions.

## Simulation state and data layout

All vehicles live in one global `SlotMap`, an array of structs. Each lane keeps a list of occupant records sorted by s, and a vehicle appears in every lane it occupies. If profiling shows frame-rate work is memory-bound, the hot fields could split into a struct of arrays (**Suggestion**).

The state is one struct, `SimState`, holding everything that can't be rebuilt from the rest: the network, the vehicles, the occupant lists, the junction queues, speed caps, the step count, time and config. It is exactly what a freeze frame saves. What can be rebuilt, the overlaps between lanes and the merges, is derived from it when a simulation is built or restored, and kept apart.

**Vehicle state**

- World pose (x, y, heading), speed, and the curvature of the rear axle's path (its steering state)
- Acceleration, held between longitudinal updates
- Path: the lane it follows and the lanes it will take next, as far as its leader search, path tracking and lane change look, and as far as it would go in 15 s at each lane's speed limit, however short the lanes. The time is at the limits, not its speed, so a slow vehicle in a queue still knows where it is going, and merges and junctions ahead know of it; its leader search stays within its horizon however far the path reaches. A path ending inside a junction is extended until it is out of it, so a vehicle in a junction's queue always knows its way through. It is a `Vec`.
- Route: how it chooses its way at forks, with any state that needs (see Routing)
- Indicator: the side it is indicating to and since when
- Lane-change state, owned by the lane-change model, and the old side and lane change in progress
- Old follower: the vehicle behind that it holds up most, as of the last longitudinal update, with that vehicle's acceleration from every other term, for the courtesy incentive
- Acceleration noise and its own random generator's state
- Static attributes (`VehicleSpec`): dimensions and driver parameters. **Planned:** barrier commitments and vehicle classes.

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
| Every step                           | Path tracking (pure pursuit), bicycle-model integration, occupancy, junction queues      | Pose, speed, steering; records    |
| Every `accel_every` steps (3: 20 Hz) | Lane-change desire and gap checks; longitudinal model (IDM+)                             | Desire, indicator, lane; acceleration |

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

- **What it reacts to:** the leaders along its path (below), the end of its known path, lower speed limits ahead, its old side while straddling, junction and merge terms (see those sections), synchronisation and the room it keeps while wanting to change lanes (see Lane changes), a ghost it makes room for, and a speed cap. **Planned:** the nearest barrier it has not committed to, as a stationary leader at the line, as in Treiber's work.
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
   - **Route,** `d_r`: on a lane that ends by merging and is left early, it rises linearly from 0 to 1 toward the lane's end, by distance (295 m per change to make) or time at its speed (43 s), whichever is higher, toward the neighbour that goes on.
   - **Speed,** `d_s`: how much faster traffic goes ahead in the lane beside it than in its own, over 69.6 km/h. Each lane's speed is the average of its vehicles over the next 8 s at its limit, no faster than the driver would like, and as fast as it would like where nobody is: an average, not the leader alone, so a driver moves over before its own lane has stopped. Each lane's speeds are summed in order once per update, so an average is two binary searches.
   - **Keeping to a side,** `d_b`: the free threshold toward the side the lane says to keep to.
   - **Courtesy:** letting by its old follower, by what that follower would gain, weighed by its politeness, in units of its comfortable deceleration.

   As LMRS has it, `d = d_r + θ·(d_s + d_b + d_c)`: the voluntary incentives count fully unless they work against the route, and not at all once the route's desire is strong enough to cooperate. The desire follows the incentives smoothed over 1 s, so a fleeting gain doesn't make a driver change.
2. **Thresholds:** at `d_free`, 0.365, the driver checks the gap and, if it lets it, indicates, and changes after indicating for at least 0.5 s. At `d_sync`, 0.577, it indicates even without a gap, asking for room, and synchronises: it adjusts its speed to the leader it would have in the target lane, its headway shortened by its desire, no harder than comfortably. At `d_coop`, 0.788, followers in the target lane make room for it, and it changes even without room to finish. It cancels once its desire falls below half its threshold, or no permitted neighbour remains on that side.
3. **Gap acceptance:** the gap is checked as if the driver were already settled in the target lane, level with where it is: its own acceleration there, against its leaders and any ghost ahead, and its new follower's with it as a leader, found by searching back. Both keep their time headways shortened by its desire, from the whole at no desire to `T_min/T_max` at a desire of 1, and the change is accepted if neither brakes harder than the desire times the driver's comfortable deceleration, raised toward b_safe (4 m/s²) by its assertiveness.
4. **Spreading the checks:** a refused check waits a few updates before the next, 4 (0.2 s) scaled by `1 + politeness − assertiveness/2`, so drivers' checks don't fall in step; at most `max_gap_checks` (64) are made in one update, a backstop, and the rest wait for the next. A vehicle with no lane it may change to and no desire left is skipped outright.

- **Searching back:** from a point on a lane, back through that lane's occupants, then the ends of the lanes leading into it, as far as any vehicle's path reaches beyond its front. On the lane searched from, every vehicle but a ghost counts; on lanes leading in, one counts if that lane is on its path. The first found is the follower if the point is within its horizon. This is how SUMO and Treiber's MOBIL find the follower on the target lane, and `try_insert` finds its follower the same way.
- **Nobody slips between:** the leader search ahead and the follower search behind split each lane at the vehicle's rear, so a vehicle alongside is one or the other, with a gap of zero or less, and refuses the change.
- **Room to finish:** a vehicle doesn't change lanes unless its rear axle would be past the cubic's midpoint before it had to stop behind its leader: the gap less its minimum gap, plus the leader's stopping distance at its comfortable deceleration. Otherwise it would be left straddling both lanes, blocking them. It may indicate without the room. While it wants to change, indicating or with half its threshold of desire, it keeps room behind its leaders to get halfway through a change from rest, so it isn't boxed in: a separate term, with that room as its minimum gap, which IDM settles on from above, never braking harder than comfortably, since it is a preference, not safety, which only binds in slow traffic. VISSIM has a minimum front clearance for changing lanes from a standstill, and drivers are taught to stop where they can see the tyres of the vehicle ahead.
- **Assertiveness:** drivers vary in how readily they change lanes, from 0 to 1, as SUMO's `lcAssertive` and `lcSpeedGain`: an assertive driver's thresholds are lower, by up to half, it accepts harder braking, and it waits less after a refusal. A few assertive drivers pull out into a faster lane and slow it, so a fast lane doesn't form, and the cautious follow. Its distribution is the caller's concern; the UI draws it as a uniform number cubed, about one driver in ten above 0.7.
- **Making room,** LMRS's cooperation: a follower whose search reaches a ghost before any leader may make room for it, treating it as a leader, if its owner's desire has reached `d_coop`. It starts if that takes no more braking than its courtesy, a per-driver attribute (0 never yields), and once it has chosen to, it keeps making room up to b_safe until the ghost is gone. Typically a few nearer followers drive on and a later one opens a gap. This is Hidas's (2002, 2005) cooperative lane changing, and SUMO's `lcCooperative`.
- **Two vehicles into one gap:** a ghost ahead counts as a leader in a vehicle's own gap check, but the follower search passes ghosts over, so of two indicating into the same gap, the one behind waits.
- **Across section boundaries:** when the vehicle advances, its ghost moves to the neighbour of its new lane, or, with none it may change into, the intent is cancelled.

**Module boundary:** the decision making lives in the lane-change model; the rest of the simulation sees only each vehicle's indicator and an opaque per-vehicle state. The simulation measures each driver's incentives and hands them to the model (`aim`), the model moves the desire and says whether it wants a gap checked (`step`), the simulation checks it by searching the network, and the model says what to do (`resolve`): nothing, indicate, cancel or change. Making room works the same way. This is SUMO's split between its lane changer and its lane-change models.

**Suggestion:** drive mandatory changes from the routing model's required lanes, as LMRS's route desire toward the lanes a route needs.

## Conflicts, merging and junctions

Lanes that cross or merge inside an intersection are grouped into a junction, and a junction keeps vehicles on its lanes apart. A lane belongs to at most one junction. Junctions are a dumb backstop: they keep vehicles that can no longer stop from running into each other, and nothing more. The order vehicles go in is for barriers, with gap acceptance, signals and arrival order (**Planned**).

- **Conflicts:** a junction stores, for each ordered pair of its lanes, where they conflict, if they do: the kind (merge or cross) and the overlapping s-range on each lane, in a dense `lanes²` table, so a lookup gives "my" range and "theirs" directly. Lanes leading into the same lane merge; branches of a fork, lanes one after the other and neighbours never conflict, since the multi-lane following model keeps vehicles on them apart; any other pair whose outlines overlap crosses.
- **Finding overlaps:** each segment's piece of a lane's outline is a convex quadrilateral, and every pair of pieces from two lanes is clipped one against the other (Sutherland–Hodgman). Each lane's s-range runs from the first overlap to the last. Overlaps under 10 cm² don't count, so lanes touching along an edge don't conflict. A merge whose outlines don't overlap gets the lanes' ends.
- **One queue per junction,** part of the state and of snapshots: the vehicles **committed** there, which can no longer stop short of their conflicts, in the order they committed, and the lanes vehicles not yet committed are **approaching** it along.
- **Arriving:** a vehicle arrives at a junction once a lane of the junction is on its path and no other junction it hasn't entered lies before it, so when its path is extended or rebuilt, or it enters a junction. It adds the lanes of its path up to the first of the junction's lanes with a conflict to those the junction is approached along. The junction keeps no list of who is approaching, only where.
- **Committing:** every step, the junction scans back from the start of the conflict on each of its lanes being approached, along that lane's occupants and the lanes leading in that it is approached along, passing over ghosts, committed vehicles and vehicles whose path has left the way. At the first other vehicle: one headed onto the lane commits once it can no longer stop at `SAFE_BRAKING` (4 m/s²) before the conflict, or its front is past its start, and the scan goes on behind it; one that can still stop, or is headed elsewhere in the junction, stops the scan. So nobody commits before a vehicle ahead of it on its lane that might still go in, which has to be enforced, since IDM only roughly keeps a follower able to stop. Lanes a scan finds no one on, nor beyond, stop being approached. `SAFE_BRAKING` rather than comfortable deceleration keeps the commit order close to the order vehicles reach their conflicts; it cut hard braking by about four fifths at the ramps.
- **Releasing:** every step, a committed vehicle that could now stop at `SAFE_BRAKING` a car length short of its conflict is released, unless one committed after it is behind it on its way in. The car length is the hysteresis against committing.
- **Leaving:** a vehicle leaves the queue once no lane of the junction is on its path and it has no record on any of them, or when it is removed.
- **Keeping apart:** at each longitudinal update, the junction walks its committed vehicles and gives each follower a term from each conflict with each vehicle committed before it: until the one ahead has cleared the conflict on its lane, the gentlest of three ways of keeping out of its way, each safe on its own:
  - **Stopping short** of the conflict.
  - **Following its progress:** a virtual vehicle at the speed of the one ahead, as far past the start of the follower's conflict as that one's rear is past the end of its own, so it reaches that start just as the other clears: virtual vehicle mapping (Uno et al., 1999). Crossing streams roll through one behind another rather than stopping at the line.
  - **Following it,** while it moves the same general way as the follower's conflict runs, now and as it leaves its own: its nearest corner along that direction, at its speed along it. A convex body is nowhere nearer along a direction than its nearest corner, and checking its heading as it leaves too stops a vehicle turning across from leading the follower in and then swinging back.

  Between committing at `SAFE_BRAKING` and IDM's limit, `MAX_BRAKING` (9 m/s²), lies the backstop's margin, so two vehicles committing at once still stop apart. A follower past the start of its conflict is held back all the same: at a merge, dropping back is the only way out.
- **Vehicles at the front of each way in,** not yet committed, take terms from every committed vehicle too, so they slow gently for one in their way rather than braking hard once they commit.
- **Lane changes:** none start in a junction or run into one, so a vehicle is never partway across a junction's lanes. A committed vehicle that changes lanes on its way in keeps its place.
- **Deadlock:** waiting vehicles in a cycle are all stopped, and a stopped vehicle more than a car length short of its conflict is released, breaking the cycle. One can survive only among vehicles within a car length of their conflicts, which needs a vehicle cutting in just ahead of a committed one close to a merge, with another committing between them; it is watched for, not prevented. A vehicle can still be blocked by traffic beyond the junction: **gridlock** is allowed.
- **Cost:** per junction, O(c²) pairs at 20 Hz for c committed vehicles, tens at most, and each step a scan past them on each lane approached.

**Off the beaten path:** SUMO resolves junctions per link, from foe matrices and each vehicle's announced arrival time, with no global order. One first-come, first-served queue per junction, of the vehicles that can no longer stop, is closer to Dresner and Stone's AIM reservations and A/B Street's intersection queues, with barriers deciding when each vehicle commits.

**Suggestion:** give each yielding point one of two priority policies: fixed priority, the yielding vehicle using gap acceptance against priority traffic; or arrival order, first come, first served, as at four-way stops (SUMO's `allway_stop`).

**Gap acceptance** (**Planned**): the yielding vehicle goes only if it can clear the conflict before any priority vehicle whose path crosses it could reach it, even accelerating at its maximum, plus 0.5 s. Covering d from speed v at acceleration a up to v_max takes `(√(v² + 2ad) − v) / a`, or, once at v_max, `t₁ + (d − d₁) / v_max` with `t₁ = (v_max − v) / a` and `d₁ = (v + v_max)·t₁ / 2`. Priority vehicles stopped at an uncommitted barrier are ignored, which prevents deadlocks. Movements have integer priority levels; at equal levels, the queue's order decides.

**Hook turns:** a right turn made from the left lane: the vehicle pulls forward to the left side of the intersection, waits, and completes the turn when the cross street's signal turns green. This needs no new mechanism: a connector lane from the left lane, and a mid-junction barrier tied to the cross street's signal.

## Merges

Where lanes outside junctions that were apart start to overlap, as at a lane drop or where two roads' lanes meet one, a merge orders the vehicles approaching and opens gaps between them before they meet. Inside the merge, vehicles keep apart by following each other, which keeps them from colliding but not from arriving side by side and braking hard. A merge's only job is to stop that. Merges are kept apart from junctions, in their own module: a junction orders vehicles once they can no longer stop, and forbids lane changes; a merge orders them early and lets them change lanes on the way in.

- **Which lanes:** a group of lanes outside junctions that overlap one another, joined pair by pair, where the two lanes of each pair have no predecessor in common and no two of their predecessors overlap: the boundary where disjoint lanes start merging. Its merge point is the start of its lanes. Merges are derived when the simulation is built, so they aren't part of a snapshot.
- **The order:** the vehicles approaching, by the distance from each one's rear to the merge point, nearest first. Nothing is stored: at each longitudinal update the order is worked out by walking back through the lanes leading in, as far as a path reaches, and marching back along them all at once, as the merge step of a merge sort does, taking the nearest of the lanes' next vehicles each time. A lane's vehicles count if they follow it and their path leads into the merge, so a vehicle is in the order once a merge lane is on its path, well before the merge, and leaves it once it is on a merge lane, where its followers find it through their records. A vehicle that pulls ahead goes first at once, as on the road; close to the merge point the terms keep vehicles from being level, so the order doesn't flicker.
- **By distance, not estimated arrival:** a vehicle's leader in the order is then never further from the merge point, which the term's slack covers. By estimated arrival, a fast vehicle could come before a slow one well ahead of it, which would brake as hard as it could to fall in behind a leader behind it, and a stopped vehicle's arrival never comes, so a queue locked up. A fast vehicle instead gives way to a slower one just ahead, gently, as drivers do.
- **The term:** each vehicle F whose front hasn't reached the merge point follows the one before it in the order, L, by IDM, as a virtual vehicle on F's lane as far from the merge point (Uno et al., 1999), with the headway it will keep just after merging, LMRS's `T_min/T_max` of its own. Slack is added to the gap, enough for F to be level with L at first, shrinking linearly to nothing at the merge point from 8 s before it at F's speed, or 50 m if that is further: so the gap opens gradually as both draw near, and queues go through by turns, each vehicle moving off as soon as its leader is clear. With less than 50 m, crawling queues reach the merge point side by side and go through by turns from rest, a third less getting through. The term never brakes harder than comfortably: before the merge point the lanes are apart, and inside the merge, following keeps vehicles apart however they arrive.
- **Cost:** at 20 Hz, a march along the occupants of the lanes leading into each merge, and one term per vehicle in it.

**Off the beaten path:** SUMO orders vehicles at its `zipper` junctions by estimated arrival, and Treiber's simulator treats an on-ramp as mandatory MOBIL lane changes. Ordering by distance and following a virtual leader is the usual pairing in cooperative merging (Uno et al., 1999); the shrinking slack is this simulator's own.

## Barriers (Planned)

Stop lines, give-way lines, traffic lights and the start of keep-clear zones are all barriers. A vehicle brakes for a barrier with IDM, as for a stationary leader at the line, until it commits to crossing, then behaves as if the barrier is not there.

| Barrier                           | Commits when                                                                                                                        |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| Traffic light                     | It can no longer brake comfortably to stop, and the light is not yet amber or red                                                   |
| Stop line (**Suggestion**)        | It has come to a stop, and either the conflicting lanes have an acceptable gap or, at an all-way stop, it is first in arrival order |
| Give-way line (**Suggestion**)    | The conflicting lanes have an acceptable gap                                                                                        |
| Keep-clear start (**Suggestion**) | There is room beyond the far edge of the zone for the whole vehicle                                                                 |

- Amber and red are the same in the simulation.
- Signals run fixed cycles to start, behind a module boundary so actuated control can come later.
- **Movements:** a signal controls movements, each a group of lanes that go and stop together, with a bitmask of the movements it conflicts with, and its own amber time.
- **All-red clearance:** a movement can't turn green until every conflicting movement has been red for at least its clearance time.
- **Phases:** each phase is a mask of movements that should be green. Its duration counts from when all of its movements are green, so clearance doesn't eat into green time.
- **Update:** each step computes every movement's next state from the current states, then applies them all.

A green light exerts no deceleration unless there is no room across the junction; then keep-clear rules apply. This is how the old prototype ran its signals.

## External API

Demand and routing live outside the `sim` crate. The simulation exposes:

- **Making and stepping:** `Simulation::new` or `with_config`, `step`, `step_variable`, `alpha`, `time`.
- **Inserting:** `insert` places a vehicle on a lane's centreline with no checks, for tests and scripted setups. `try_insert` refuses when there is no safe room: when the new vehicle would overlap another, or it or its new follower would have to brake harder than `SAFE_BRAKING` because of the other; its speed is capped at its new leader's. Its leader and follower come from the same two searches a lane change uses, split at its rear.
- **Reading:** a vehicle by key, every vehicle, each lane's occupants, each junction's queue and the lanes it is approached along, and the network. `vehicle_at` picks the vehicle nearest a point, within a tolerance, by a linear scan.
- **Changing:** `remove`; `change_lane`; `cap_speed`, under which a vehicle brakes at `SAFE_BRAKING` while above the cap and eases up to it from below, `hold` (a cap of 0) and `release`. Capped vehicles are kept in a short list, not as a field on every vehicle, and the cap is one more acceleration term.
- **Snapshot and restore** the whole state, network included, for freeze frames: `state` lends the `SimState`, to clone or, with the `serde` feature, serialize, and `restore` builds a simulation from one. A restored simulation continues bit for bit until it inserts a vehicle into a slot freed before the snapshot: `SlotMap` rebuilds its free list in index order, so that vehicle's id, and the update order after it, can differ.
- **Heterogeneity** comes from inserting vehicles with varied attributes. Distributions are the caller's concern, not the core's.

**Routing:** each vehicle carries a `Route`, given at insertion, which chooses the next lane when its path is extended past a lane with several successors; the choice is then part of its path. For now the only route is random: a SplitMix64 generator seeded at insertion, drawn from only at forks, so the same seed takes the same way, and snapshots and replays reproduce it. A vehicle only chooses among its own lane's successors: reaching a branch from another lane needs mandatory lane changes.

**Suggestion:** the routing interface needs lookahead, the lanes that reach the route several links ahead, not just the next lane, so mandatory lane changes can start well before a diverge; SUMO calls this "best lanes". Shortest path to a destination (Dijkstra or A*) is the next route model.

## Debug instrumentation

Algorithms report what they are doing through a debug sink that costs nothing when debug output is off: the simulation is generic over `DebugSink`, and events are passed as closures, `sink.record(|| …)`, so a zero-sized no-op sink compiles away entirely. The UI runs with a recording sink, and benchmarks with the no-op one. A feature flag would have kept signatures simpler but needed a separate build for debugging.

The debug view shows what the simulation actually used, never a recomputation. Events are grouped by the update that produces them, and accumulate in the sink:

- **Resets:** each update first records a reset for its group, "recompute accelerations" and "recompute motion" for everyone, and "recompute decision" per vehicle, which clears that group's earlier events; a removed vehicle's events go with a "vehicle removed" event. The UI never clears the sink itself, and reads only after a step returns, so it always sees the last complete set.
- **Acceleration terms:** each term is its own event, the free-road term and each leader, junction, merge, synchronisation, cap and ghost term. Every input to the acceleration is recorded, so the binding term is the one equal to the vehicle's acceleration, and one whose terms don't reach it shows an event is missing.
- **Also recorded:** lookahead points, passing checks, with the follower's predicted pose, the other vehicle's nearer rear corner and the gap between them, and lane-change evaluations, with each side's incentives, desire and any gap check.

## UI

The `ui` crate renders the simulation with egui's painter and offers two views.

- **Pretty view:** road and junction surfaces, lane markings, stop lines and turn arrows from the render geometry, vehicles as oriented shapes drawn between steps, and indicators.
- **Debug view:** layers for lane outlines, centrelines, s ticks, neighbours, occupants, vehicles, lane-change paths, leaders and merge orders, passing checks and lookahead points. Clicking a vehicle opens an inspector with its state, its lane-change desires and its acceleration terms.
- **Controls:** pause, single step, speed multiplier, camera pan and zoom, scenario choice, traffic (a target count on the ring, a flow per entry lane elsewhere), lane changes for the selected vehicle, and saving a replay or a freeze frame.

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
- **Head of the queue:** only the first vehicle approaching a barrier should negotiate it; the ones behind just follow. Can a vehicle's forward search decide this alone, by being the head when the first thing ahead of it is the barrier rather than a leader?
- **Approaching priority traffic:** gap acceptance needs the priority vehicles that could reach a conflict in time. **Suggestion:** read them from the junction's queue, where they arrive as soon as their paths reach it; or search upstream from the conflict along the priority lanes, as a gap check searches for followers.
- **Merge capacity:** relaxed headways let a merged lane carry about 2,600 veh/h, above the 2,000–2,400 of real lanes.

## Milestones

Each milestone is a demo that can be finished on its own; anything after it is optional.

1. Workspace skeleton: one straight lane, IDM, kinematics, the pretty and debug views. **Done.**
2. A ring road showing emergent stop-and-go waves, as in the Sugiyama experiment. **Done.**
3. A multi-lane freeway with continuous lane changes and multi-lane membership, first with MOBIL, now LMRS. `sim/tests/scenario.rs` runs a three-lane ring headless as a stress test. **Done.**
4. An on-ramp merge and an off-ramp diverge. Merges, with zipper or early merging, and the diverge's car following are done; taking the off-ramp needs mandatory lane changes for a route.
5. A single signalised intersection with barriers and a fixed signal cycle. The junction backstop is done.
6. Hook turns.
7. A procedural grid with shortest-path and random-walk routing. Random routing is done.

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
- Pure pursuit and Stanley path tracking: standard lateral controllers from autonomous-vehicle research.
