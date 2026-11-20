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
- **`SimNetwork` output:** lanes as polylines, lane successors, lane neighbours with lane-change permissions, precomputed conflicts, barriers and signal controllers.
- **Render output:** road and junction surfaces, lane markings, stop lines and keep-clear markings.
- **Compile-time work:** generating connector lanes through junctions, computing conflicts and placing barriers.
- **Precision:** every lane point and half-width is snapped to a 1 mm grid as the lane is built, so a network serializes exactly as short text.

**First cut (a proof of concept):** the `network` crate (`traffic-network`) describes networks at road level, as OpenDRIVE and SUMO's edge/lane/connection model do, and expands them to bare lanes during compilation. That is how people think about roads, and it keeps the sim free of road concepts. The format will change; procedural generators (`scenarios`) build it, and the UI compiles and draws what they build.

- **Roads:** a reference line, a start pose followed by pieces (lines, arcs and cubic Béziers) that each continue from the last, and lanes beside it, each with an offset and a width that are constant or ease (smoothstep) from one value to another over a stretch, and a direction along the road. Clothoids can come later.
- **Sampling:** curves are sampled by tolerance, not at a fixed spacing: at most 1 cm of sagitta (chord error, `c² / 8R`) and 3° of turn per vertex, with a vertex at most every 2 m where an offset or width eases. Gentle curves get long chords and tight connectors short ones, as vector renderers flatten curves.
- **Lanes from one polyline:** a road's lanes are offset from one sampled reference polyline by its miters, so their segments stay parallel to the reference's, and the vertex next to each end of the road lies on the end tangent. Neighbouring lanes then end square across the road, side by side, as `validate` requires, and meet the next road's lanes exactly. Offsetting the curve itself for each lane would leave neighbours' ends a few centimetres apart on a bend.
- **Neighbours:** lanes of a road running side by side the same way and keeping their distance; lanes easing together, at a lane drop, or apart, where one appears, are not. Each lane says whether vehicles on it may change into its neighbour on the left and on the right, as SUMO's `changeLeft` and `changeRight` do; both by default. The ramps' freeway lane beside the acceleration lane may not change into it.
- **Joins:** a road's end joins the next road's start, and lanes connect wherever one ends where the other starts: two meeting one merge, one meeting two diverge, with nothing declared. Merging road lanes are not a junction: vehicles on lanes that overlap keep apart by following (see [Multi-lane car following](multilane-car-following.md)).
- **Intersections:** each lists which lanes connect to which, and each connection becomes a cubic Bézier from one lane's end to the other's start along their directions, its control points a third of the way across, its width blending from one to the other, and its speed limit lowered for its tightest bend (2 m/s² sideways, at least 3 m/s). An intersection's connectors are one junction.
- **Render output:** each road's surface between its outermost edges; lane markings, dashed between lanes going the same way, a centre line between opposite ones, and edge lines; and each intersection's surface, the mouths of its roads joined by cubic Bézier curbs that carry on along one road's edge and turn out along the next's, with edge lines along them.

Generating junction geometry (connector curves and junction surfaces) is the hardest part of this crate. SUMO's netconvert is largely heuristics for it; procedural networks keep it tractable.

## Lane model and geometry

The simulation has no roads, only lanes. Lanes may overlap, merge, diverge or cross, and lane changes may be permitted between them.

- **Geometry:** each lane is a polyline with arc length s and lateral offset t. The (s, t) ↔ world mapping is worked out and fast. Each vertex has its own left and right half-widths, so a lane can taper and its centreline needn't be centred; between vertices they vary linearly in s, as OpenDRIVE gives lane width as a function of s apart from the reference line. Widths are kept apart from the segments the mapping reads, since only the occupancy refresh, construction and rendering use them. SUMO gives each lane one width. Points and half-widths are snapped to the nearest millimetre as the geometry is built.
- **Serialized geometry:** a lane's geometry serializes as a string of vertices in whole millimetres, separated by spaces, each `x,y,left,right`; with three values the half-widths are equal, and with two they are the previous vertex's, so a lane of constant width gives them once: `0,0,1750 90000,0 100000,2000,2000,1750`. Being on the grid, it reads back exactly, and the segments are rebuilt from it. The rest of a network serializes through serde as usual.
- **Module boundary:** a lane's geometry, `LaneGeometry`, is its own type in its own module; the `Lane` beside `SimNetwork` holds it with its speed limit and links, which only the network sets.
- **Relationships:** successors and predecessors (end-to-start connections); left and right neighbours; overlaps, found by conflict precomputation. A lane's successors are kept in order from left to right, by the way they part, sorted as they are connected; its predecessors keep the order they were connected in.
- **Neighbours:** each lane has at most one neighbour on each side. Adjacency is symmetric, but lane-change permission is per ordered pair, because a change may be allowed one way only. Permission covers the whole lane pair; where a road changes from allowing changes to not, it splits into successive lanes.
- **Lane sections:** adjacent lanes start and end together, like the lanes of a SUMO edge or an OpenDRIVE lane section. Where one lane splits, its neighbours split at the same point. So the neighbour of a lane's successor is the successor of its neighbour, which lets a vehicle map its path to the lanes beside it. `SimNetwork::validate` checks this, along with mutual neighbours, neighbours starting and ending together, and successors starting where their lane ends.
- **Multi-lane roads** exist only in the source-of-truth network. Compilation flattens them.
- **Keeping to a side:** each lane says which side drivers on it keep to, `Keep::{Left, Right, NoPreference}`, an incentive for lane changes (see Lane changes). Where it applies is the network's to say: in Australia, keep left on roads with limits above 80 km/h, which the network crate sets from a rule on the road network.
- **Lanes that end by merging,** as an acceleration lane does, say how drivers leave them, `Leaving::{Zipper, Early}`: at the merge, taking turns, as Australian acceleration lanes long were, or along the way, wanting to more the nearer the end, as the broken lines beside newer ones ask. The ramps scenario offers both.

**Suggestion:** store cumulative arc length per vertex, and give each vehicle a cached segment index per lane it occupies. Vehicles move a fraction of a segment per frame, so projection becomes a forward scan from the cached index, close to O(1).

**Off the beaten path:** SUMO, VISSIM and Aimsun treat lane position as the source of truth and derive world position from it. Here world pose is the truth and lane coordinates are derived, as in autonomous-vehicle planners using Frenet frames and in OpenDRIVE's s/t coordinates. The reason is realistic motion through lane changes, merges and junctions.

## Simulation state and data layout

All vehicles live in one global `SlotMap`. Each lane keeps a list of occupant records sorted by s, and a vehicle appears in every lane it occupies.

**Vehicle state (sketch)**

- World pose (x, y, heading), speed, and the curvature of the rear axle's path (its steering state)
- Acceleration, held between longitudinal updates rather than recomputed each frame
- Path: the assigned lane followed by the lanes it will take next, as far ahead as its leader search, path tracking and lane change look, and as far as it would go in 15 s at each lane's speed limit, however short the lanes, and extended as it moves. Lanes can be arbitrarily short, so no count of lanes is enough: the path is bounded by distance and time, as SUMO's lookahead is. The time is at the speed limits, not the vehicle's speed, so a slow vehicle in a queue still knows where it is going, and merges and junctions ahead know of it; its leader search stays within its horizon however far the path reaches. Leader search and path tracking look along it across lane boundaries. It is a `Vec` for now; profiling will say whether other storage is worth it. A path that ends inside a junction is extended until it is out of it, so a vehicle in a junction's queue always knows its way through, and nobody is committed to a conflict before learning it is in the way.
- Route: how the vehicle chooses its way at forks, with any state that needs (see Routing).
- Barrier commitments
- Indicator: the side it is indicating to and since when, which other vehicles and the UI read
- Old follower: the vehicle behind that it holds up most, as of the last longitudinal update, with the gap between them and that vehicle's acceleration from every other term. The lane-change decision reads it, so it is part of the state, and of snapshots.
- Lane-change state: owned by the lane-change model and opaque to the rest of the simulation (see Lane membership, lane changes and diverging)
- Static attributes: vehicle class, dimensions, driver-model parameters

**Per-lane occupancy**

- Each record is `(vehicle id, projection hint, classification, age)`, the hint being the lane coordinates of the rearmost corner and so its s, with what the passing check needs (see [Multi-lane car following](multilane-car-following.md)), and lives in the lane's list, not on the vehicle. There is one copy of s, so lists and vehicles can't drift apart, and sorting and leader search never touch the `SlotMap`. A ghost is a record classified as one. The age is how long the vehicle has been in the lane's traffic (see Relaxation, under Longitudinal model).
- A record's s is the smaller of the vehicle's two rear corners' s on that lane. Followers care about where a vehicle's rear is.
- **Classification, joining and leaving:** how a record is classified (in lane, converging or diverging), which lanes a vehicle joins and when its records are dropped are described in [Multi-lane car following](multilane-car-following.md), which supersedes anything here about them. A vehicle joins a lane when it advances onto it, which is when its rear axle passes the end of the previous lane.
- **Leaving the network:** a vehicle leaves the network when its rear axle passes the end of the lane it follows and that lane has no successors, which is the moment it would otherwise advance; an old lane that dead-ends doesn't count.
- A vehicle on a loop can briefly have two records in the same lane, at two different s values. This is geometrically true and needs no special handling.
- Updated every step, and cheaply: each record is re-projected from its hint, cleared records are dropped, and order is restored by insertion sort, which is O(n) on sorted input and cheap for nearly sorted input.
- Multi-lane membership is the general form of SUMO's "shadow lane" for a vehicle partway through a lane change.

**Suggestion:** start with an array of structs in the `SlotMap`. If profiling shows frame-rate work is memory-bound, split the hot per-frame fields (pose, speed, acceleration) into a dense struct of arrays.

## Timesteps and update pipeline

The simulation steps by a fixed time, set when it is made (`Config`): 1/60 s by default. `Simulation::step()` takes one step; `step_variable(dt)` takes as many as fit in a frame's `dt`, carrying what is left over to the next frame, as "Fix Your Timestep" has it. A fixed step makes a run the same whatever the frame rate, and a slow frame can't destabilise integration or carry a vehicle past a barrier or conflict in one step. Work runs at three rates, each a whole number of steps. Each step flows strictly from state to derived state to decisions, then applies the decisions.

| Rate                                 | Runs                                                                                              | Writes to vehicle state            |
| ------------------------------------ | ------------------------------------------------------------------------------------------------- | ---------------------------------- |
| Every step                           | Path tracking (pure pursuit), bicycle-model integration                                           | Pose, speed, steering              |
| Every `accel_every` steps (3: 20 Hz) | Longitudinal model (IDM+); lane-change desire, and a gap check for drivers whose desire has reached their threshold | Acceleration; desire, target lane |
| When they happen                     | Barrier commitment, routing choices                                                               | Commitments, route cursor          |

**Data flow**

1. **Simulation state:** vehicles, occupancy, signal phases.
2. **Derived state**, computed from step 1 read-only: leaders, gaps, active conflicts, upcoming barriers.
3. **Decisions**, computed from steps 1–2 read-only: accelerations, lane changes, commitments.
4. **Apply:** decisions are written back to the simulation state.

Steps 2 and 3 only read, so they parallelise per vehicle (for example with rayon). Step 4 writes to disjoint vehicles.

Every vehicle runs its lane-change model as often as the longitudinal model, so it reacts without waiting for a turn, and cheaply: it moves the driver's desire toward what its incentives make it, from the speeds of the lanes ahead, and checks the gap, which takes searches, only once the desire reaches the driver's threshold. A check that fails waits a few updates before the next, longer for a more polite driver and shorter for a more assertive one, so checks don't fall in step; at most `max_gap_checks` (64) are made at once, a backstop, those beyond waiting for the next update. A vehicle with no lane it may change to, no desire left and nothing to wait for is skipped outright.

**UI:** the UI draws vehicles between where they were before the last step and where they are, by how far into the next step the frame has reached (`Simulation::alpha`), so they don't jump a step at a time when frames and steps don't line up. They are drawn a step behind.

## Vehicle kinematics

Vehicles move by a kinematic bicycle model, steered by pure pursuit toward a target path defined in lane space. World pose is the source of truth.

- **Inputs each frame:** held acceleration, a target path (the assigned lane, or a blended lateral offset during a lane change) and a lookahead distance.
- **Controller choice:** pure pursuit with a lookahead that scales with speed. It anticipates curves the way drivers do, cutting corners slightly, which reads as natural, and the guard rails bound how far it cuts. Stanley, which tracks the path more tightly, is the fallback if tracking on tight junction connectors looks wrong.
- **Steering as curvature:** steering is stored as the curvature of the rear axle's path, in 1/m, rather than as a front-wheel angle. Pure pursuit's curvature `2·sin(α) / d` is `2·(h × d) / |d|²` for heading `h` and offset `d` to the target, so it needs no trigonometry beyond the heading's direction, and the bicycle model turns by `dist · curvature` with no `tan`. The steering lock is a curvature limit, `tan(δ_max) / wheelbase`. This made a step about 30% faster than storing the angle.
- **Guard rails:** a vehicle must never escape the lane geometry, for example by bounding its lateral offset from the assigned lane. The open question is how to do this without visible snapping.
- **No corner cutting:** outside a lane change, a vehicle's body stays within its lane's edges. This is guaranteed by how vehicles are steered, not checked at runtime: nothing looks for bodies overhanging a neighboring lane, because that would cost a check per vehicle per neighbor. How to guarantee it is an open question.

After a lane change, the vehicle starts off-centre on its new lane. Its target path is a lateral offset that ramps from that starting offset to zero along a cubic (see Lane membership, lane changes and diverging). The controller follows that offset, so the trajectory looks natural without being scripted.

## Longitudinal model

IDM+ (Schakel, van Arem and Netten, 2010) computes each vehicle's acceleration at about 20 Hz, behind a module boundary so it can be replaced. IDM+ splits IDM into a free-road term and an interaction term and takes the minimum instead of the sum. That gives a more realistic capacity, and it extends naturally to several leaders: the acceleration is the minimum over the free-road term and one interaction term per leader.

- **What it reacts to:** the leaders along the vehicle's path (below), the vehicles committed before it at a junction it is committed at (see Conflicts, merging and junctions), and the nearest barrier it has not committed to. The most restrictive one wins.
- **Which leaders:** a vehicle in the lane that this one can't pass, B, blocks the lane's whole width, so nothing behind it on a shared path can reach anything beyond it that B can't, and whatever in the lane beyond constrains B reaches this one through B. So once the search finds B, it skips the in-lane vehicles beyond it, on B's lane and on the lanes after it, as far as B's path goes along this one's: where B's path parts from it, or ends, what lies beyond never constrains B. This is tracked as one high-water mark, the first lane of the path that no in-lane leader covers. A vehicle converging onto the lane after a lane change is in it, can't be passed, and shields like any other. Ghosts only partly block a lane, and B may pass a vehicle diverging from the lane that this one can't, so neither is ever skipped, and neither shields anything. Each vehicle found is its own term. A car behind a car that passes a stalled lane changer then sees the changer as soon as it can, not only once the car ahead is past it, and a car whose leader turns off sees the queue beyond. In the common case the first leader covers the whole horizon, and the search finds one vehicle plus any ghosts and diverging vehicles beyond it. Treiber's Human Driver Model reacts to several leaders too, for anticipation.
- **Horizon:** a vehicle looks ahead as far as s*, the desired gap approaching a stopped leader, `s0 + v·T + v² / (2·√(a·b))`, where a stopped leader's term is zero, or 5 m if that is further. A stopped leader coming into view there changes nothing unless the vehicle is accelerating, and then it stops: the point at which a driver lifts off and starts braking gently. The desired gap behind a leader at the vehicle's own speed, `s0 + v·T`, is shorter than s* by the v² term, so a leader followed at steady state is in view. That margin vanishes at rest, where s* is s0 and a queue comes to rest, so the 5 m floor keeps a leader there from flickering in and out of view. The horizon grows with speed, as s* does: about 75 m at 50 km/h and 310 m at 110 km/h for a typical car. Leaders beyond it are ignored, so one coming into view can change the acceleration abruptly, much as a driver catching up with stopped traffic brakes firmly at first; SUMO instead looks as far as braking distance, and Treiber's simulator looks at every leader.
- **End of the known path:** past the end of its path a vehicle doesn't know where it is going, so the end is a stop line, a stationary virtual leader like a barrier, unless the vehicle leaves the network there. Like a barrier, it is in the same forward stream as the vehicles, so the search meets constraints in order of distance; a stop line ends the search, since nothing beyond it could impose a harsher term. A routing model will say where it leaves; until one exists, it leaves wherever its path's last lane has no successors. Paths reach past the horizon, so the stop line only binds where routing doesn't know the next lane.
- **Visiting order:** the longitudinal update goes lane by lane, from the front of each lane's occupants back, and starts each vehicle's search at its own record, on the lane it follows, or, while it straddles, on its old lane, so no vehicle has to look itself up. A search from a record starts just after it, so the search needn't know which vehicle is searching; on a loop shorter than its horizon a vehicle can find itself ahead, which is geometrically true. SUMO plans each lane's vehicles from the front back in the same way.
- **Gaps:** a searching vehicle's rear is its record's s, or, without a record, where its rear would be, and its front is its length ahead of that, so every gap is a subtraction along the lanes searched. SUMO likewise keeps one position per vehicle and derives the other end from its length.
- **Desired speed:** the lane's speed limit times the vehicle's speed factor, capped at the vehicle's maximum speed. The speed factor is a vehicle attribute, as SUMO's `speedFactor`: without it every driver holds exactly the limit and nobody has a reason to overtake. Its distribution is the caller's concern: SUMO draws it from a normal distribution with mean 1 and, by default, deviation 0.1; the old prototype clamped its draws to [0.75, 1.25].
- **Lower limits ahead:** a lane ahead on the path with a lower limit is its own term. The vehicle needs `(v_lim² − v²) / 2d` to reach that limit by the lane's start, `d` ahead. The term applies only once that is harsher than comfortable braking, so braking starts just in time, and is capped at twice comfortable braking. This comes from the old prototype.
- **Barriers as virtual leaders:** an uncommitted barrier is treated as a stationary vehicle at the barrier line. This is standard IDM practice from Treiber's work.
- **Ghosts in the stream:** the search reports every ghost it meets and the caller decides what one means. Weighing up a change, and in `try_insert`, a ghost is a leader, since a vehicle already indicating into the gap goes first. In the longitudinal model, the nearest ghost, if no leader comes before it, is the one the vehicle may make room for.
- **Evaluation:** the acceleration is a running minimum over the terms as they are computed. No list of leaders, conflicts or barriers is built.
- **Relaxation:** a vehicle that has just appeared in a lane's traffic, or that another has just appeared in front of, keeps a shorter time headway at first and drops back gradually, as in Schakel, Knoop and van Arem's LMRS (2012) and VISSIM's safety distance reduction during lane changes. Each record has an age: how long its vehicle has been in that lane's traffic, ticked every step. It starts at zero when the vehicle changes lanes into the lane, enters a merge onto it or joins it from beside it, and carries over when it advances from one lane to the next, so a section boundary isn't an appearance. An inserted vehicle starts settled: insertion is checked for room at the whole headway, not a cut-in. A leader term keeps `h(a)` of the follower's time headway, with `a` the younger of the follower's own record, where its search starts, and the leader's, and `h` rising from LMRS's `T_min/T_max`, 0.56/1.2, at `a = 0` back to 1 exponentially, with LMRS's relaxation time of 25 s. The minimum gap stays whole. Checking an insertion, nothing is relaxed. Checking a gap for a lane change, the headways are shortened by the driver's desire instead, as LMRS does (see Lane changes).
- **Above the desired speed:** the free-road term is Treiber's Improved IDM's, `−b·(1 − (v0/v)^(aδ/b))`, slowing toward the desired speed at no more than comfortable deceleration, where IDM's would brake as hard as it could well above it, as when the speed limit or the traffic beside drops.
- **Acceleration noise:** drivers don't hold a speed exactly. Each has a slowly varying noise added to its free-road term, an Ornstein-Uhlenbeck process with a time constant of 10 s and a per-driver spread (`accel_noise`, 0.2 m/s² for the textbook car, 0 for none), drawn from its own seeded generator, so runs repeat, as in Treiber and Kesting's IDM with acceleration noise (2017). Following stays exact, so the noise never brings vehicles closer than IDM would.
- **Friction:** a driver goes at most 30 km/h faster than the traffic ahead in the lane beside it, the average over the next 8 s at that lane's limit, if there are at least three vehicles there, so passing a queue it slows: the friction effect Jang and Cassidy (2012) measured in carpool lanes beside congested ones.
- **Heterogeneity:** IDM parameters (desired speed, time headway, maximum acceleration, comfortable deceleration, minimum gap) come from each vehicle's attributes. `VehicleSpec::passenger_car` gives textbook values: IDM's as calibrated with LMRS, as OpenTrafficSim uses them, a = 1.25 m/s², b = 2.09 m/s², s0 = 3 m and T = 1.2 s.


## Lane membership, lane changes and diverging

How vehicles keep apart while they change lanes, merge and diverge, and when one may pass another, is described in [Multi-lane car following](multilane-car-following.md), which supersedes anything here about it. This section covers the lane-change mechanics it relies on.

- **Where changes happen:** only into a neighbour that the network permits changing into, and only where the change would be over, the whole body in the new lane, before the lanes beside the vehicle's path on that side run out: about the lookahead, the change's length and the vehicle's length. The lanes beside run out at a junction too, where a lane of the path or the lane beside it is in one, whatever neighbours the network gives them: no change starts in a junction or runs into one. One that did would leave the vehicle partway across the junction's lanes, where the junction doesn't know it is, and could stop it at a conflict partway through a change, where its search of its old lane could wait on a vehicle that waits on it. SUMO likewise allows no lane changes on internal junction lanes.
- **The decision is a jump:** when a vehicle decides to change lanes, its assigned lane switches to the target lane at once. That switch is what encodes the decision. It joins the target lane's occupant list at the same moment, just as advancing onto a successor does. Its pose doesn't move; it is simply now off-centre on its new lane, and the lateral model steers it smoothly onto the centreline.
- **Old side:** the vehicle remembers which side its old lane is on, as `Option<Side>`, and derives the old lanes by mapping its path to their neighbours on that side. While it straddles partway through the change, it runs a second leader search from its record on the old lane, when the longitudinal update reaches that record, continuing along those neighbours, so it doesn't run into its old leader. Its terms are combined with its own after every vehicle's update is computed. That search only reaches as far as the change's end (see [Multi-lane car following](multilane-car-following.md)). Once it has left, it doesn't run at all. Each leader found is its own acceleration term. A vehicle that starts a change too close behind its old leader stops partway, straddling, until that leader moves off or the passing check lets it by.
- **Across section boundaries:** a vehicle that advances while still straddling also joins every successor of its old lane, so followers there see it. The vehicle straddles while it has a record on the old-side neighbour of the lane it follows, or, just after a section boundary, of the lane before it: at a lane drop that is the only one left. Whenever it loses a record on a lane it isn't following, by that lane's end or its side, it checks this, and the old side clears once it no longer holds.
- **Lateral path:** at the start of a change the vehicle fixes a cubic in u = s − s₀ along its new lane. It starts at the point pure pursuit was aiming at on the old path, so the target point carries on without jumping: s₀ and t₀ are that point's coordinates on the new lane, and m₀ is the old path's slope dt/ds there (zero for parallel lanes). It runs to zero offset and zero slope at u = L: with x = u/L, t(u) = t₀(2x³ − 3x² + 1) + L·m₀(x³ − 2x² + x). L = max(v·T, L_min), with T = 4 s, so peak lateral acceleration is about 6·t₀/T² (about 1.3 m/s² across a 3.5 m lane) at speed. L_min is the shortest cubic the vehicle can steer, √(6·|t₀| / κ_max) with κ_max its steering lock's curvature, the cubic's sharpest curvature being at its ends: about 9 m across a 3.5 m lane for a car, so a change from a crawl is over in 9 m, not the 15 m an earlier fixed floor took. The start's slope is left out. Pure pursuit aims at the cubic's offset at its lookahead point, and at zero beyond L. If the vehicle advances mid-change, s₀ shifts back by the old lane's length.
- **Completion:** a change is complete when the vehicle reaches the end of its cubic. It can't start another until then.
- **Passing checks:** made as [Multi-lane car following](multilane-car-following.md) describes. Where the follower is placed: its nose 0.1 m short of the other vehicle's rearmost corner, measured along its own lane, so a check that fails would still not overlap. Along the lanes it follows, the record's s is that corner, so the gap to it places the follower exactly; along its old lanes, distances differ from its own lanes', so the corner is projected onto its own lane through world space. Checking a gap for a change, a vehicle is checked as if already settled on the target lanes' centrelines, as LMRS and MOBIL check a gap.
- **`change_lane`** executes a change directly, with no safety check, so it can drive a vehicle into another. It is the execute half of the intent protocol and a testing hook.

**Lane changes: LMRS.** Drivers decide to change lanes as in LMRS, the Lane-change Model with Relaxation and Synchronization (Schakel, Knoop and van Arem, 2012), with its published parameters. Every per-frame search looks only forward; a gap check may also search backwards, to find the vehicles a change would affect directly rather than waiting to hear from them.

1. **Desire:** each driver has a desire to change to each side, from −1 to 1, the sum of incentives, each in units of desire:
   - **Route,** `d_r`: on a lane that ends by merging, left early (see Lane model), it rises linearly from 0 to 1 toward the lane's end, by distance (295 m per change still to make) or time at its speed (43 s), whichever is higher; toward the neighbour that goes on, and the same against the other side.
   - **Speed,** `d_s`: how much faster traffic goes ahead in the lane beside it than in its own, over 69.6 km/h. Each lane's speed is the average of the vehicles in it over the next 8 s at its limit, no faster than the driver would like to go there, and as fast as it would like where nobody is: an average, not the leader alone, so a driver moves over before its own lane has stopped. Each lane's vehicles are summed in order once per update, so an average is two binary searches.
   - **Keeping to a side,** `d_b`: the free-change threshold toward the side the lane says to keep to (see Lane model), so a driver keeps left when the gap lets it.
   - **Courtesy:** letting by the follower it holds up most, by what that follower would gain, weighed by its politeness, in units of its comfortable deceleration. Its old follower needs no search: at each longitudinal update, a vehicle whose acceleration is bound by a leader's term records itself on that leader, with its acceleration from every other term, and each leader keeps the one that would gain most.

   As LMRS has it, `d = d_r + θ·(d_s + d_b + d_c)`: the voluntary incentives count fully unless they work against the route, and not at all once the route's desire is strong enough to cooperate. The desire follows the incentives smoothed over 1 s, so a fleeting gain doesn't make a driver change.
2. **Thresholds:** at 0.365 (`d_free`, lowered by up to half by assertiveness, below) the driver checks the gap and, if it lets it, indicates, then changes after indicating for at least 0.5 s. At 0.577 (`d_sync`) it indicates even without a gap, asking for room, and synchronises: it adjusts its speed to the leader it would have in the target lane, with its headway shortened by its desire, no harder than comfortably. At 0.788 (`d_coop`) followers in the target lane make room for it (below), and it changes even without room to finish (below). It cancels once its desire falls below half its threshold, or no permitted neighbour remains on that side.
3. **Gap acceptance:** the gap is checked as if the driver were already settled in the target lane, level with where it is: its own acceleration there, against its leaders and any ghost ahead, and its new follower's with it as a leader, found by searching back. Both keep their time headways shortened by its desire, from the whole at no desire to LMRS's `T_min/T_max`, 0.56/1.2, at a desire of 1; and the change is accepted if neither brakes harder than the desire times the driver's comfortable deceleration, raised toward b_safe by its assertiveness. The ghost is the vehicle's indicator: a flagged record in the target lane's occupant list, kept up to date each step like any other. Executing is the jump to the new lane, its ghost becomes its real record, and its desire starts afresh.

- **Searching back:** from a point on a lane, back through that lane's occupants, then the ends of the lanes leading into it. On the lane searched from, every vehicle counts, settled, moving in or moving out, since any of them can be in the way; on the lanes leading into it, one counts if the lane searched from is on its path, since its leader search ends where its path does. So the search goes back no further than any vehicle's path reaches beyond its front, a running maximum the simulation keeps. Ghosts don't count. The first vehicle found is the follower if the point is within its own horizon: further back, its own leader search wouldn't reach that far, so it isn't affected. This is how SUMO's lane-change model and Treiber's MOBIL implementation find the follower on the target lane. `try_insert` finds its follower the same way.
- **Nobody slips between:** the leader search ahead and the follower search behind split each lane at the same point, the vehicle's rear: a record whose rear is at or ahead of it is a leader candidate, and one whose rear is behind is a follower. So every vehicle beside it is one or the other, and one alongside is found with a gap of zero or less, which makes the change unsafe. A vehicle alongside might even be found as both; the change was invalid anyway. This is MOBIL's requirement that both new gaps be positive. `try_insert` relies on the same split.
- **Room to finish:** a vehicle doesn't change lanes unless it would be at least halfway through the change, its rear axle past the cubic's midpoint, before it had to stop behind its leader in its own lane: the gap less its minimum gap, plus the leader's stopping distance at its comfortable deceleration. Otherwise it would be left straddling both lanes, blocking them; at the ramps' heaviest flow, that cut the time vehicles spent crawling partway through a change by nine tenths. It may indicate without the room, and while it wants to change, indicating or with half its threshold of desire, it keeps room behind its leaders to get halfway through a change from rest, plus 1 m, so it isn't boxed in: a raised minimum gap, as a separate term never braking harder than comfortably, since it is a preference, not safety. At speed the leader's stopping distance covers it, so it only binds in slow traffic. VISSIM has a per-driver minimum clearance in front for changing lanes from a standstill, LMRS adjusts a driver's spacing before a change, and drivers are taught to stop where they can see the tyres of the vehicle ahead. A vehicle that wants to change lanes looks for leaders on its own lanes at least as far as that room. With a desire of `d_coop` it changes without it, or a vehicle stopped too close behind a stopped one might never change lanes.
- **Assertiveness:** drivers vary in how readily they change lanes, a per-driver attribute from 0 to 1, as SUMO's `lcAssertive` and `lcSpeedGain`. An assertive driver's thresholds are lower, by up to half, and it accepts harder braking, toward b_safe; its gap checks also wait less after a refusal. Its distribution is the caller's concern; the UI draws it as a uniform number cubed, mostly cautious and about one driver in ten above 0.7. A few assertive drivers pull out into a faster lane and slow it, so a fast lane doesn't form, and the cautious follow into the room that leaves.
- **Making room:** LMRS's cooperation. At each longitudinal update, a follower whose search reaches a ghost before any leader may choose to make room for it, treating it as a leader, if its owner's desire has reached `d_coop`. It starts if that takes no more braking than its courtesy, a per-driver attribute (0 never yields); once it has chosen to, it remembers whose ghost it is and keeps making room, unless that would take braking harder than b_safe, until the ghost is gone, so the choice doesn't flicker. Nearer followers would have to brake harder, so typically a few drive on and a later one, further back or more courteous, opens a gap. This is Hidas's (2002, 2005) cooperative lane changing, and SUMO's `lcCooperative`.
- **Parameters,** as module constants, LMRS's published values: `d_free` 0.365, `d_sync` 0.577, `d_coop` 0.788, `v_gain` 69.6 km/h, route distance 295 m and time 43 s, `T_min/T_max` 0.56/1.2, relaxation 25 s; and this simulator's own: desire smoothed over 1 s, incentives looked for over 8 s at the limit, minimum indicating time 0.5 s, 4 updates (0.2 s) between refused checks scaled by `1 + politeness − assertiveness/2`, assertiveness lowering thresholds by up to half, politeness per driver (default 0.2), courtesy per driver (default 1 m/s²), and b_safe 4 m/s².
- **Two vehicles into one gap:** a ghost ahead counts as a leader in a vehicle's own search, but the follower search passes ghosts over. So of two vehicles indicating into the same gap, from both sides or on the same tick, the one behind sees the other's ghost ahead and waits, while the one ahead may go; then the one behind sees it as a vehicle. Two vehicles may indicate at once.
- **Across section boundaries:** when the vehicle advances, its ghost moves to the neighbour of its new lane. If there is none, or the change isn't permitted, the intent is cancelled.

LMRS replaced MOBIL (Kesting, Treiber and Helbing), with advantage built up over time and frustration: MOBIL's gains come from the immediate leaders alone, so drivers moved over only once their own lane had stopped, and a fast lane formed beside a queue.

**Module boundary:** the decision making lives in the lane-change model; the rest of the simulation sees only each vehicle's indicator and an opaque per-vehicle model state. The simulation measures each driver's incentives and hands them to the model (`aim`); at each update, the model moves the desire and says whether it wants a gap checked (`step`); the simulation checks it, by searching the network, and the model says what to do (`resolve`): nothing, indicate, cancel or change, which the simulation carries out, moving the ghost and setting the indicator, and writes the state back in the apply step. A vehicle partway through a change or still straddling isn't asked. Making room works the same way: the simulation offers the ghost ahead, its owner's desire and what following it would take, and the model says whom the vehicle makes room for. This is SUMO's split: its lane changer gathers the neighbouring leaders and followers and its lane-change models own their state and decide.

**Suggestion:** drive mandatory changes from the routing model's required lane sequence, as LMRS's route desire toward the lanes a route needs.


## Conflicts, merging and junctions

Lanes that interact are grouped into junctions, and a junction keeps vehicles on its lanes apart. A lane belongs to at most one junction. Junctions are a dumb backstop: they keep vehicles that can no longer stop from running into each other, and nothing more. The order vehicles go in is for barriers, later, with gap acceptance, signals and arrival order.

- **Conflicts:** a junction stores, for each ordered pair of its lanes, where they conflict, if they do: the kind (merge or cross) and the overlapping s-range on each lane. It is a dense table, `lanes²` entries with each pair stored both ways round, so a lookup is two indexes and gives "my" range and "theirs" directly. A lane holds its junction and its index there.
- **Merge or cross** comes from connectivity, not geometry: lanes leading into the same lane merge. Lanes diverging from the same lane, lanes one after the other, and neighbours never conflict: joining every branch at a fork, and leader searches, keep vehicles on them apart. Any other pair whose outlines overlap crosses.
- **Finding overlaps:** each segment's piece of a lane's outline, between its ribs, is a convex quadrilateral, and every pair of pieces from the two lanes is clipped one against the other (Sutherland–Hodgman). Each lane's s-range runs from the first overlap to the last, the union if they overlap in several places, which shouldn't happen in practice. Overlaps under 10 cm² don't count, so lanes touching along an edge don't conflict. A merge whose outlines don't overlap gets the lanes' ends.
- **Building:** until the `network` crate exists, `SimNetwork::add_junction` takes a junction's lanes, once they are connected, and computes their conflicts.
- **One queue per junction,** part of the simulation state and of snapshots: the vehicles **committed** there, which can no longer stop short of their conflicts, in the order they committed, and the lanes vehicles not yet committed are **approaching** it along, sorted.
- **Arriving:** a vehicle arrives at a junction once a lane of the junction is on its path and nothing between it and the junction holds it back: no barrier it has not committed to, and no other junction it hasn't physically entered, by having a record on one of its lanes. So three events can make it arrive: its path is extended or rebuilt, it commits to a barrier, or it enters a junction. Arriving depends on speed only through how far its path reaches. Arriving, it adds the lanes of its path, from the one it is on to the first of the junction's lanes with a conflict, that lane included, to those the junction is approached along. The junction keeps no list of who is approaching, only where.
- **Committing:** every step, the junction scans back from the start of the conflict on each of its lanes being approached, a lane being in at most one junction and so having at most one conflict zone: along that lane's occupants, then the lanes leading in, branch by branch, following only lanes it is approached along. On each branch it passes over ghosts, committed vehicles and vehicles whose path has left the way and goes nowhere in the junction, and at the first other vehicle:
  - one headed onto the lane commits, joining the end of the committed vehicles, once it can no longer stop at `SAFE_BRAKING` (4 m/s²) before the conflict, or its front is past the start of it, and the scan goes on behind it; if it can still stop, the scan stops there;
  - one headed elsewhere in the junction, or whose path ends before it could leave the way, stops the scan: it may yet go in ahead of those behind it, and another lane's scan, or a later one, deals with it.

  So nobody commits before a vehicle ahead of it on its lane that might still go in. That has to be enforced: IDM keeps a follower able to stop behind its leader only roughly, and one closing fast on a slow leader, which can still stop, often can't stop itself. A lane a scan finds no one on, nor beyond, leaves those the junction is approached along, so vehicles only ever add lanes, and stale ones go once a scan passes them; lanes behind a vehicle a scan stopped at stay. `SAFE_BRAKING` rather than comfortable deceleration: vehicles at the front of the ways in already slow for committed vehicles in their way, so committing late costs no harder braking, and keeps the order vehicles commit in close to the order they reach their conflicts. Against comfortable deceleration, it cut hard braking by about four fifths at the ramps and up to three fifths at lightly loaded crossroads, though about a fifth more at the busiest crossroads. If braking at junctions is too harsh, vehicles should commit earlier. One with no conflict on its path never commits, and never needs to.
- **Releasing:** every step, a committed vehicle that could now stop at `SAFE_BRAKING` a car length short of its conflict is released, unless a vehicle committed after it is behind it on its way in. The junction itself can bring a committed vehicle to a stop short of its conflict, with the margin it has up to `MAX_BRAKING`, and so can traffic ahead; one that plainly can stop shouldn't hold back those committed after it. The car length is the hysteresis against committing, and keeps a vehicle held at the line committed. Releasing from the back of each lane keeps vehicles committed in their order along it. A released vehicle's way in is marked approached again, and it commits again, at the end, when it must.
- **Lane changes:** none start in a junction or run into one (see Lane membership, lane changes and diverging), so a vehicle is never partway across a junction's lanes. A committed vehicle that changes lanes on its way in keeps its place, and its new path may make it arrive elsewhere.
- **Leaving:** a vehicle leaves the queue once no lane of the junction is on its path and it has no record on any of them, so its whole body is clear, or when it is removed.
- **Keeping apart:** at each longitudinal update, after every vehicle's own terms, the junction walks its committed vehicles and, for each pair, ahead and behind, and for each committed vehicle and each vehicle at the front of a way in, looks up the conflicts between each junction lane on the follower's path and each of the junction's lanes. Each gives the follower a term, folded in with its others like its old side's, crossing or merging alike: until the one ahead has cleared the conflict on its lane, its rear past the far end, the gentlest of three ways of keeping out of its way, each safe on its own:
  - **Stopping short** of the conflict, as at a stopped vehicle there.
  - **Following its progress:** a virtual vehicle at the speed of the one ahead, as far past the start of the follower's conflict as that one's rear is past the end of its own, so it reaches that start just as the one ahead clears. This is virtual vehicle mapping, from cooperative merging (Uno et al., 1999). Crossing streams can roll through one behind another rather than stopping at the line.
  - **Following it,** while it moves the same general way as the follower's conflict runs and will still as it leaves its own (positive dot products of its heading, and of its lane's direction at the end of its conflict, with the follower's lane's direction at the start of its conflict): its nearest corner measured along that direction, from the start of the conflict, at its speed along that direction. A convex body is nowhere nearer along a direction than its nearest corner, at any angle, for a conflict straight on the follower's side; a lane bends one way by less than half a turn, so one heading the same way now and as it leaves never swings back across in between. Without that second test, a vehicle turning across could lead the follower into its conflict and then swing back into it. At a merge this is ordinary car-following through the taper.

  Between committing at `SAFE_BRAKING` and IDM's limit, `MAX_BRAKING`, lies the backstop's margin, so two vehicles committing at once still stop apart. A follower already past the start of its conflict is held back all the same: it shouldn't be there before the one ahead has cleared, and at a merge, where driving on never clears the way, dropping back is the only way out. One not headed onto the lane counts only while its body, lingering past the lane's end or beside it, is in the overlap, and only by stopping short or following it. This is conservative; gap acceptance (below) can let a follower go earlier. The debug view records only the term chosen.
- **Vehicles not yet committed** at the front of each way in, those the scans stopped at headed onto the lane scanned, take terms from every committed vehicle, as if committed after them, so they slow gently for a committed vehicle in their way rather than driving on until they commit and then braking hard. Those behind them follow them. They hold no one back.
- **Cost:** O(c²) pairs per junction at 20 Hz, with c the committed vehicles: tens at most. Each step, each lane being approached scans back past the committed vehicles on its way to the first vehicle not committed, usually one to three records, touching only lanes vehicles approach along; a junction approached along nothing costs nothing. Releasing checks each committed vehicle against those committed after it, O(c²) per step.
- **Deadlock:** a committed vehicle waits on vehicles committed before it, on its leaders, and on traffic beyond the junction. Commitment follows the order along each lane, so its leaders in the lane committed before it, but lane changes can still put a vehicle that commits later ahead of it. Waiting vehicles in a cycle are all stopped, and a stopped vehicle more than a car length short of its conflict is released, breaking the cycle. One can only survive among vehicles all within a car length of their conflicts or past them: never two in the same lane, nor one partway through a lane change, since none runs into a junction. That leaves a vehicle cutting in just ahead of a committed one, close to a merge, with another committing between them; it is watched for, not prevented. Vehicles not yet committed hold no one back in the junction, though those at the front of a way in wait on committed ones; a cycle through one still needs a committed vehicle stopped behind it, more than a car length short, which is released. A vehicle can still be blocked by traffic beyond the junction: **gridlock** is allowed.
- **Without barriers,** nothing orders vehicles before they commit, so a junction reached by busy conflicting lanes is collision-free chaos: vehicles commit as they come and the latecomers brake hard. That is the backstop working.

**Off the beaten path:** SUMO resolves junctions per link, from foe matrices and each vehicle's announced arrival time, with no global order. One first-come, first-served queue per junction, of the vehicles that can no longer stop, is closer to Dresner and Stone's AIM reservations and A/B Street's intersection queues, with barriers deciding when each vehicle commits.

**Suggestion:** give each yielding point one of two priority policies.

- **Fixed priority:** the yielding vehicle uses gap acceptance against priority traffic.
- **Arrival order:** for four-way stops and similar, vehicles go first come, first served, as the queue already orders them; at a stop line, a vehicle joins once it has come to a stop. SUMO models this as its `allway_stop` junction type.

**Gap acceptance** is a simple, conservative calculation from current speed, distance and maximum acceleration. The yielding vehicle goes only if it can clear the conflict zone before any priority vehicle could reach it, even if that vehicle accelerated at its maximum. Priority vehicles stopped at an uncommitted barrier are ignored, because they won't enter the zone, and this prevents deadlocks.

- **Earliest arrival:** a vehicle at speed v, accelerating at a up to a speed cap v_max, covers distance d in `(√(v² + 2ad) − v) / a` if it doesn't reach the cap, and otherwise in `t₁ + (d − d₁) / v_max`, where `t₁ = (v_max − v) / a` and `d₁ = (v + v_max)·t₁ / 2`. Add a 0.5 s buffer.
- **Only vehicles whose path crosses the conflict count:** a vehicle on the conflicting lane whose path turns off before the zone is ignored.
- **Approaching priority vehicles** are found by forward passes, not by searching upstream from the conflict.
- **Ties:** movements have integer priority levels. A higher level goes first; at equal levels, the queue's order decides. This breaks every tie deterministically.

**Hook turns:** a right turn made from the left lane. The vehicle pulls forward to the left side of the intersection, waits, and completes the turn when the signal for the street it is entering turns green. This needs no new mechanism: a connector lane from the left lane, plus a mid-junction barrier tied to the cross street's signal phase.

## Merges

Where lanes outside junctions that were apart start to overlap, as at a lane drop or where two roads' lanes meet one, a merge orders the vehicles approaching it and opens gaps between them before they meet. Inside the merge, vehicles keep apart by following each other (see [Multi-lane car following](multilane-car-following.md)), which keeps them from colliding but not from arriving side by side: the one behind then finds the other ahead with a negative gap and brakes as hard as it can. A merge's only job is to stop that happening. Merges are kept apart from junctions, in their own module: a junction is a backstop that orders vehicles once they can no longer stop, and forbids lane changes; a merge orders them early and lets them change lanes on the way in.

- **Which lanes:** a merge is a group of lanes outside junctions that overlap one another (see Preprocessing in [Multi-lane car following](multilane-car-following.md)), joined pair by pair, where the two lanes of each pair have no predecessor in common and no two of their predecessors overlap. Its merge point is the start of its lanes. Branches of a fork share their predecessor, so they are no merge, and lanes whose predecessors already overlap are inside a merge, not at its start. Merges are derived from the network when the simulation is built, as the overlaps are, so they aren't part of a snapshot.
- **The order:** the vehicles approaching a merge, nearest the merge point first, by the distance from each one's rear. Nothing about it is stored: at each longitudinal update it is worked out afresh, by walking back from the merge's lanes through the lanes leading in, as far as any path reaches (15 s at the speed limits), and marching back along all of them at once, as the merge step of a merge sort does, taking the nearest of the lanes' next vehicles each time: each lane's occupants are in order already. A lane's vehicles count if they follow it and their path leads into the merge, so a vehicle is in the order once a merge lane is on its path, and its path reaches 15 s ahead at the speed limits, so even one crawling in a queue is in it well before the merge; it leaves once it advances onto one of the merge's lanes, from when its followers find it there through their records. Each vehicle's term needs only the one before it, so the terms are worked out as the march goes, with nothing collected. Storing the order, with vehicles joining and leaving it as their paths changed, and sorting it, each made no measurable difference to speed.
- **By distance:** a vehicle that pulls ahead goes first at once, as on the road; close to the merge point the terms keep vehicles from being level, so the order doesn't flicker there, and hysteresis made no measurable difference. Two on the same lane of the merge keep their order along it by the same rule. By distance, not estimated arrival: a vehicle's leader in the order is then never further from the merge point, which the term's slack covers. Ordered by estimated arrival, a fast vehicle could come before a slow one well ahead of it, and the slow one would be told to fall in behind a leader that was behind it, braking as hard as it could, hundreds of metres out; a stopped vehicle's estimated arrival never comes, so a queue on the slower approach locked up. A fast vehicle gives way to a slower one just ahead in the other lane, gently, by easing off on the approach, as drivers do.
- **The term:** at each longitudinal update, the list is walked from the front, and each vehicle F whose front hasn't reached the merge point takes a term from the vehicle L before it, folded in with its other terms as a junction's are. F follows a virtual L by IDM: L as far from the merge point on F's lane, the gap to it from F's front to L's rear, each measured to the merge point (virtual vehicle mapping, Uno et al., 1999). It keeps the headway it will keep just after merging, half its own (see Relaxation), with slack added to the gap that shrinks linearly to nothing at the merge point from 8 s before it at F's speed, or 50 m if that is further, so a fast vehicle starts making room early and one crawling in a queue only near the merge point: at the start, enough for F to be level with L at its desired gap. So the gap opens gradually as both draw near, rather than all at once at the merge point, and in a queue the lanes go through by turns, the follower moving off as soon as its leader is clear, not waiting at the merge point for a whole gap. The term never brakes harder than F's comfortable deceleration: before the merge point the lanes are apart, so it only smooths the way in, and inside the merge, following keeps vehicles apart however they arrive. A shorter minimum than 50 m lets crawling queues reach the merge point side by side and go through by turns from rest: at 10 m, a third less got through. Capping it at `SAFE_BRAKING` instead doubled to quadrupled the hard braking at the ramps for no gain in throughput. An earlier form also took the gentler of the term and stopping at the merge point; that let a vehicle level with its leader drive on to the merge point and stop there, rather than drop back, so the lanes went through stop-start, one at a time. Once F is in the merge, its headway relaxes from that half back to the whole. A leader on F's own lane is followed as usual too; with the slack, the term asks less than that.
- **Lane changes** are allowed throughout: a vehicle that changes out of a lane leading to a merge is no longer in its order, and one that changes into one is.
- **No barrier:** the terms alone open the gaps.
- **Cost:** at 20 Hz, a walk of the lanes leading into each merge, a march along their occupants, tens at most, and one term per vehicle.

**Off the beaten path:** SUMO orders vehicles at its `zipper` junctions by estimated arrival, and Treiber's simulator treats an on-ramp as mandatory MOBIL lane changes. Ordering by distance and following a virtual leader is the usual pairing in cooperative merging (Uno et al., 1999); the shrinking slack and the relaxed headway into the merge are this simulator's own.

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
- **Movements:** a signal controls movements, each a group of lanes that go and stop together, with a bitmask of the movements it conflicts with. Each movement has its own amber time.
- **All-red clearance:** a movement can't turn green until every conflicting movement has been red for at least its clearance time.
- **Phases:** each phase is a mask of movements that should be green. A phase's duration counts from when all of its movements are green, so clearance times don't eat into green time.
- **Update:** each step computes every movement's next state from the current states, then applies them all, in keeping with the state → decisions → apply flow.

This is how the old prototype ran its signals.

A green light exerts no deceleration, committed or not, unless there is no room across the junction. Then keep-clear rules apply.

## External API

Demand and routing live outside the `sim` crate. The simulation exposes vehicle insertion, lookup and removal, plus a pluggable routing model.

- **Insert** a vehicle from a spec: class, dimensions, driver-model parameters, starting lane and s, and its routing model.
- **Find** vehicles by key, by lane and s-range, and spatially (point or box) for picking in the UI.
- **Remove** a vehicle by key.
- **Cap** a vehicle's speed: above the cap it brakes at `SAFE_BRAKING` (4 m/s²), so a held vehicle stops promptly, and nearing it from below it eases up to it, until released. A cap of 0 holds it: it brakes to a stop and waits. Capped vehicles are kept in a short list on the simulation, not as a field on every vehicle, and the cap is one more acceleration term, so it only binds when it is the lowest.
- **Pick** the vehicle nearest a point, within a tolerance, for the UI. Linear scan for now.
- **Snapshot and restore** the whole state at one instant, network and junction queues included, for freeze frames. The UI's replays, a log of every call that changes the simulation, start with the network as JSON, so they don't depend on whatever built it. With the `serde` feature a snapshot serializes; the UI saves them as JSON. A restored simulation continues bit for bit until it inserts a vehicle into a slot freed before the snapshot: `SlotMap` rebuilds its free list in index order, so that vehicle's id, and the update order after it, can differ.
- **Routing model:** answers which lane to take next. Implementations include shortest path to a destination (Dijkstra or A*) and a random walk through the network.
- **Heterogeneity** comes from inserting vehicles with varied attributes. Distributions are the caller's concern, not the core's.

- **Checked insertion:** `try_insert` refuses when there is no safe room: when the new vehicle would overlap another, or when it or its new follower would have to brake harder than `SAFE_BRAKING` (4 m/s², MOBIL's b_safe) because of the other. Its speed is capped at its new leader's. Its leader and follower come from the same two searches a lane change uses, split at its rear, so a vehicle alongside is one of them, with no gap, and refuses the insertion. `insert` stays unchecked, for tests and scripted setups.

**Routing:** each vehicle carries a `Route`, given at insertion, which chooses the next lane when its path is extended past a lane with several successors; the choice is then part of its path and doesn't change. For now the only route is random: a SplitMix64 generator seeded at insertion, drawn from only at forks, so the same seed takes the same way, and snapshots and replays reproduce it. A vehicle only chooses among its own lane's successors: reaching a branch from another lane needs mandatory lane changes.

**Suggestion:** the routing interface needs lookahead (the lanes required several links ahead), not just the next lane. Mandatory lane changes have to start well before the diverge; SUMO calls this "best lanes".

## Debug instrumentation

Algorithms report what they are doing through a debug sink that costs nothing when debug output is off. The mechanism is one generic parameter on the simulation.

| Option                      | Zero cost by                                | Trade-off                                                                 |
| --------------------------- | ------------------------------------------- | ------------------------------------------------------------------------- |
| Generic `Sim<D: DebugSink>` | A zero-sized no-op sink, monomorphised away | One extra type parameter; debug and headless builds coexist in one binary |
| Feature flag                | Code compiled out when the flag is off      | Simpler signatures; debug needs a separate build                          |

The generic was chosen: the UI runs with a recording sink and benchmarks with the no-op one.

**Suggestion:** pass closures (`sink.record(|| …)`) so building an event's data is skipped entirely when the sink does nothing.

**Events worth recording:** leader links and gaps, conflict activations, barrier commitments, lane-change desires, their incentives and gap checks, target paths and lookahead points, and checks for passing a vehicle ahead, with the follower's predicted pose, the other vehicle's nearer rear corner and the gap between them.

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

- **No corner cutting:** how to guarantee that a vehicle's body stays within its lane outside lane changes. Pure pursuit aims along the path from the vehicle's own position, so it tracks a constant curve exactly; it cuts in only where curvature changes, because it starts turning before a bend and straightens before its end. That is about 5 cm at 50 km/h on a 100 m bend, but more on tight junction connectors that start abruptly.
- **Head of the queue:** only the first vehicle approaching a barrier should negotiate it; the ones behind just follow. In the old prototype each lane walked its vehicles backwards from the line. Can a vehicle's forward search decide this alone, for example by being the head when the first thing ahead of it is the barrier rather than a leader? If not, barrier commitment is an intermittent decision, which may now search backwards (**Suggestion**).
- **Approaching priority traffic:** gap acceptance needs the priority vehicles that could reach a conflict in time. With forward passes only, how do they make themselves known at a conflict ahead of them? **Suggestion:** gap acceptance is an intermittent decision, and those may now search backwards, so the yielding vehicle could search upstream from the conflict along the priority lanes, as a lane change searches for its followers. That would replace the forward-pass decision above, so it needs approval. Alternatively (**Suggestion**), gap acceptance could read the priority vehicles that could reach it in time from the junction's queue, where they arrive as soon as their paths reach it.

## Milestones

Each milestone is a demo that can be finished on its own; anything after it is optional.

1. Workspace skeleton: one straight lane, IDM, kinematics, the pretty and debug views. **Done.**
2. A ring road showing emergent stop-and-go waves, as in the Sugiyama experiment. **Done.**
3. A multi-lane freeway with MOBIL, continuous lane changes and multi-lane membership. Began with the accumulating debug events described under Debug instrumentation, which replaced the vehicle's stored leader. The demo was a three-lane ring that starts empty, with vehicles spawned into the outer lane and despawned from the inner lane, so congestion on the outside drove lane changes leftward; `sim/tests/scenario.rs` still runs that scenario headless, as a stress test. The UI now keeps a target vehicle count instead, spawning at random lanes and points and despawning random vehicles. **Done.**
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
