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

**Suggestion:** store cumulative arc length per vertex, and give each vehicle a cached segment index per lane it occupies. Vehicles move a fraction of a segment per frame, so projection becomes a forward scan from the cached index, close to O(1).

**Off the beaten path:** SUMO, VISSIM and Aimsun treat lane position as the source of truth and derive world position from it. Here world pose is the truth and lane coordinates are derived, as in autonomous-vehicle planners using Frenet frames and in OpenDRIVE's s/t coordinates. The reason is realistic motion through lane changes, merges and junctions.

## Simulation state and data layout

All vehicles live in one global `SlotMap`. Each lane keeps a list of occupant records sorted by s, and a vehicle appears in every lane it occupies.

**Vehicle state (sketch)**

- World pose (x, y, heading), speed, and the curvature of the rear axle's path (its steering state)
- Acceleration, held between longitudinal updates rather than recomputed each frame
- Path: the assigned lane followed by the lanes it will take next, as far ahead as its leader search, path tracking and lane change look, however short the lanes, and extended as it moves or speeds up. Lanes can be arbitrarily short, so no count of lanes is enough: the path is bounded by distance, as SUMO's lookahead is. Leader search and path tracking look along it across lane boundaries. It is a `Vec` for now; profiling will say whether other storage is worth it. A path that ends inside a junction is extended until it is out of it, so a vehicle in a junction's queue always knows its way through, and nobody is committed to a conflict before learning it is in the way.
- Route: how the vehicle chooses its way at forks, with any state that needs (see Routing).
- Barrier commitments
- Indicator: the side it is indicating to and since when, which other vehicles and the UI read
- Old follower: the vehicle behind that it holds up most, as of the last longitudinal update, with the gap between them and that vehicle's acceleration from every other term. The lane-change decision reads it, so it is part of the state, and of snapshots.
- Lane-change state: owned by the lane-change model and opaque to the rest of the simulation (see Lane membership, lane changes and diverging)
- Static attributes: vehicle class, dimensions, driver-model parameters

**Per-lane occupancy**

- Each record is `(vehicle id, projection hint, classification)`, the hint being the lane coordinates of the rearmost corner and so its s, with what the passing check needs (see [Multi-lane car following](multilane-car-following.md)), and lives in the lane's list, not on the vehicle. There is one copy of s, so lists and vehicles can't drift apart, and sorting and leader search never touch the `SlotMap`. A ghost is a record classified as one.
- A record's s is the smaller of the vehicle's two rear corners' s on that lane. Followers care about where a vehicle's rear is.
- **Classification, joining and leaving:** how a record is classified (in lane, converging or diverging), which lanes a vehicle joins and when its records are dropped are described in [Multi-lane car following](multilane-car-following.md), which supersedes anything here about them. A vehicle joins a lane when it advances onto it, which is when its rear axle passes the end of the previous lane.
- **Leaving the network:** a vehicle leaves the network when its rear axle passes the end of the lane it follows and that lane has no successors, which is the moment it would otherwise advance; an old lane that dead-ends doesn't count.
- A vehicle on a loop can briefly have two records in the same lane, at two different s values. This is geometrically true and needs no special handling.
- Updated every substep, and cheaply: each record is re-projected from its hint, cleared records are dropped, and order is restored by insertion sort, which is O(n) on sorted input and cheap for nearly sorted input.
- Multi-lane membership is the general form of SUMO's "shadow lane" for a vehicle partway through a lane change.

**Suggestion:** start with an array of structs in the `SlotMap`. If profiling shows frame-rate work is memory-bound, split the hot per-frame fields (pose, speed, acceleration) into a dense struct of arrays.

## Timesteps and update pipeline

Work runs at three rates. Discrete decisions run at 5–10 Hz, acceleration at about 20 Hz, and steering and integration every frame. Each step flows strictly from state to derived state to decisions, then applies the decisions.

| Rate        | Runs                                                       | Writes to vehicle state                |
| ----------- | ---------------------------------------------------------- | -------------------------------------- |
| 5–10 Hz     | Lane-change decisions, barrier commitment, routing choices | Target lane, commitments, route cursor |
| ~20 Hz      | Longitudinal model (IDM+)                                  | Acceleration                           |
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

Discrete decisions are staggered across vehicles, so their cost is spread evenly across frames rather than landing on one. This also stops every vehicle reacting on the same tick. A cursor walks the vehicles in iteration order, wrapping around: each substep processes the whole part of an accumulated budget of n·h/P vehicles, where h is the substep and P the decision period, so each vehicle decides about once per P at any frame rate. Resuming uses `iter().skip(k)`, which is O(k) but cheap; a dense slot map or an explicit order list would make it O(1) if profiling asks.

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
- **Lane-change onset:** the model can see when each vehicle appeared on a lane. A newly appeared leader (a cut-in) does not cause sudden braking.
- **Heterogeneity:** IDM parameters (desired speed, time headway, maximum acceleration, comfortable deceleration, minimum gap) come from each vehicle's attributes.

**Suggestion:** keep the time a vehicle appeared on a lane on its occupant record, so a leader's record says how recently it appeared, and smoothing needs no per-vehicle leader state.

**Suggestion:** MOBIL's safety criterion already limits how hard a lane change can make the new follower brake. The appearance time then only has to smooth what is left, for example by easing from the previous leader's gap to the new one over 1–2 s.

## Lane membership, lane changes and diverging

How vehicles keep apart while they change lanes, merge and diverge, and when one may pass another, is described in [Multi-lane car following](multilane-car-following.md), which supersedes anything here about it. This section covers the lane-change mechanics it relies on.

- **Where changes happen:** only into a neighbour that the network permits changing into, and only where the change would be over, the whole body in the new lane, before the lanes beside the vehicle's path on that side run out: about the lookahead, the change's length and the vehicle's length. The lanes beside run out at a junction too, where a lane of the path or the lane beside it is in one, whatever neighbours the network gives them: no change starts in a junction or runs into one. One that did would leave the vehicle partway across the junction's lanes, where the junction doesn't know it is, and could stop it at a conflict partway through a change, where its search of its old lane could wait on a vehicle that waits on it. SUMO likewise allows no lane changes on internal junction lanes.
- **The decision is a jump:** when a vehicle decides to change lanes, its assigned lane switches to the target lane at once. That switch is what encodes the decision. It joins the target lane's occupant list at the same moment, just as advancing onto a successor does. Its pose doesn't move; it is simply now off-centre on its new lane, and the lateral model steers it smoothly onto the centreline.
- **Old side:** the vehicle remembers which side its old lane is on, as `Option<Side>`, and derives the old lanes by mapping its path to their neighbours on that side. While it straddles partway through the change, it runs a second leader search from its record on the old lane, when the longitudinal update reaches that record, continuing along those neighbours, so it doesn't run into its old leader. Its terms are combined with its own after every vehicle's update is computed. That search only reaches as far as the change's end (see [Multi-lane car following](multilane-car-following.md)). Once it has left, it doesn't run at all. Each leader found is its own acceleration term. A vehicle that starts a change too close behind its old leader stops partway, straddling, until that leader moves off or the passing check lets it by.
- **Across section boundaries:** a vehicle that advances while still straddling also joins every successor of its old lane, so followers there see it. The vehicle straddles while it has a record on the old-side neighbour of the lane it follows, or, just after a section boundary, of the lane before it: at a lane drop that is the only one left. Whenever it loses a record on a lane it isn't following, by that lane's end or its side, it checks this, and the old side clears once it no longer holds.
- **Lateral path:** at the start of a change the vehicle fixes a cubic in u = s − s₀ along its new lane. It starts at the point pure pursuit was aiming at on the old path, so the target point carries on without jumping: s₀ and t₀ are that point's coordinates on the new lane, and m₀ is the old path's slope dt/ds there (zero for parallel lanes). It runs to zero offset and zero slope at u = L: with x = u/L, t(u) = t₀(2x³ − 3x² + 1) + L·m₀(x³ − 2x² + x). L = max(v·T, L_min), with T = 4 s and L_min = 15 m, so peak lateral acceleration is about 6·t₀/T² (about 1.3 m/s² across a 3.5 m lane) at any speed above about 4 m/s. Pure pursuit aims at the cubic's offset at its lookahead point, and at zero beyond L. If the vehicle advances mid-change, s₀ shifts back by the old lane's length.
- **Completion:** a change is complete when the vehicle reaches the end of its cubic. It can't start another until then.
- **Passing checks:** made as [Multi-lane car following](multilane-car-following.md) describes. Where the follower is placed: its nose 0.1 m short of the other vehicle's rearmost corner, measured along its own lane, so a check that fails would still not overlap. Along the lanes it follows, the record's s is that corner, so the gap to it places the follower exactly; along its old lanes, distances differ from its own lanes', so the corner is projected onto its own lane through world space. Weighing up a change, a vehicle is checked as if already settled on the target lanes' centrelines, as MOBIL weighs a change.
- **`change_lane`** executes a change directly, with no safety check, so it can drive a vehicle into another. It is the execute half of the intent protocol and a testing hook.

**Lane-change intents.** Every per-frame search looks only forward. Intermittent decisions, at 5 Hz staggered and on insertion, may also search backwards, to find the vehicles they would affect directly rather than waiting to hear from them.

1. **Weigh up and build up:** at each decision tick, for each permitted neighbour, a vehicle projects itself onto it and runs forward searches from there, along the lanes beside its own path, as if already settled there, giving its prospective leader and its acceleration if it changed; its acceleration now is the one it holds, from the last longitudinal update, and a speed cap, holding it back in any lane, applies to both. For the followers its change would affect, it searches back from where it would be for its new follower in the target lane, and computes that follower's acceleration with it as a leader too. Its old follower in its own lane needs no search: at each longitudinal update, a vehicle whose acceleration is bound by a leader's term, against every term it has, cap and old side included, records itself on that leader, with the gap and its acceleration from every other term, and each leader keeps the one that would gain most. With it gone, the old follower would react to what it does, as in Treiber's MOBIL, so its acceleration then is the least of that and its terms against each of the changer's leaders, a forward search from the changer, further away by its length and the gap between them. That gives MOBIL's net incentive. As in SUMO's lane-change model, a driver doesn't act on the incentive at one instant: the amount above the threshold builds up over time, per side, while it lasts, and what has built up fades, halving every second, while it doesn't. So a clear gain intends a change within a few decisions, a marginal one only if it lasts several seconds, and a fleeting one not at all.
2. **Indicate:** once enough has built up for a side, it indicates, if the change would be safe for it and its new follower, or if it is frustrated enough to ask for room (below): a flagged ghost record in the target lane's occupant list, kept up to date each substep like any other record. The ghost is the vehicle's indicator. Another vehicle weighing up a change counts a ghost ahead of it as a leader, and a follower may choose to make room for one (below); otherwise ghosts brake no one.
3. **Change:** at each later decision tick, it keeps building up and checking. It changes once it has indicated for at least its minimum indicating time and the change is safe; executing is the jump to the new lane, its ghost simply becomes its real record, and its built-up advantage starts afresh. It cancels once what has built up fades below half the bar for indicating, or no permitted neighbour remains on that side; otherwise it keeps indicating, waiting for a gap. Safety holds back the change, not the indicator, so a driver whose change would cut someone off indicates and waits rather than flickering on and off, and the lower bar for cancelling stops built-up advantage hovering around the bar from flickering it either.

- **Searching back:** from a point on a lane, back through that lane's occupants, then the ends of the lanes leading into it. On the lane searched from, every vehicle counts, settled, moving in or moving out, since any of them can be in the way; on the lanes leading into it, one counts if the lane searched from is on its path, since its leader search ends where its path does. So the search goes back no further than any vehicle's path reaches beyond its front, a running maximum the simulation keeps. Ghosts don't count. The first vehicle found is the follower if the point is within its own horizon: further back, its own leader search wouldn't reach that far, so it isn't affected. This is how SUMO's lane-change model and Treiber's MOBIL implementation find the follower on the target lane. `try_insert` finds its follower the same way.
- **Nobody slips between:** the leader search ahead and the follower search behind split each lane at the same point, the vehicle's rear: a record whose rear is at or ahead of it is a leader candidate, and one whose rear is behind is a follower. So every vehicle beside it is one or the other, and one alongside is found with a gap of zero or less, which makes the change unsafe. A vehicle alongside might even be found as both; the change was invalid anyway. This is MOBIL's requirement that both new gaps be positive. `try_insert` relies on the same split.
- **Frustration:** a driver that wants to change lanes but can't safely, its advantage built up but the change unsafe, grows frustrated: one second per second blocked, up to 10 s, fading with a half-life of 5 s while it isn't, and starting afresh after a change. "Safe" means braking no harder than the driver accepts, for itself and the follower it cuts in front of, and that rises with frustration: its comfortable deceleration when calm, up to b_safe when fully frustrated. So a calm driver waits for a comfortable gap, and one held up takes tighter ones. This applies to discretionary changes too: a breakdown on a motorway makes a change effectively forced without any route saying so. As the critical gap shrinks with waiting in Ahmed's (1999) model, SUMO's impatience grows while a driver is held up, and VISSIM raises the accepted deceleration nearer where a change must happen.
- **Asking for room:** fully frustrated, a driver indicates even without a safe gap, so long as the follower in the target lane is moving (at least 2 m/s): in a stopped queue there is nobody to make room. It still changes only once the change is safe, by then usually because someone has made room.
- **Making room:** at each longitudinal update, a follower whose search reaches a ghost before any leader may choose to make room for it, treating it as a leader. It starts if that takes no more braking than its courtesy, a per-driver attribute (0 never yields); once it has chosen to, it remembers whose ghost it is and keeps making room, unless that would take braking harder than b_safe, until the ghost is gone, so the choice doesn't flicker. Nearer followers would have to brake harder, so typically a few drive on and a later one, further back or more courteous, opens a gap. This is Hidas's (2002, 2005) cooperative lane changing, and SUMO's `lcCooperative`.
- **Parameters,** as module constants for now: politeness per driver (default 0.2), courtesy per driver (default 1 m/s²), threshold 0.1 m/s², safe braking (b_safe) 4 m/s², advantage to intend a change 0.3 m/s (the net gain above the threshold, summed over time), at most 0.6 m/s built up, fading with a half-life of 1 s, minimum indicating time 0.5 s, and a decision every 0.2 s per vehicle. MOBIL is symmetric for now, with no bias toward either side; SUMO's equivalent has a keep-right term too.
- **Two vehicles into one gap:** a ghost ahead counts as a leader in a vehicle's own search, but the follower search passes ghosts over. So of two vehicles indicating into the same gap, from both sides or on the same tick, the one behind sees the other's ghost ahead and waits, while the one ahead may go; then the one behind sees it as a vehicle. Two vehicles may indicate at once.
- **Across section boundaries:** when the vehicle advances, its ghost moves to the neighbour of its new lane. If there is none, or the change isn't permitted, the intent is cancelled.

Discretionary changes use MOBIL (Kesting, Treiber and Helbing's incentive-plus-safety model with a politeness factor), through the intents above.

**Module boundary:** the decision making lives in the lane-change model; the rest of the simulation sees only each vehicle's indicator and an opaque per-vehicle model state. At a decision tick, the simulation hands the model the vehicle's indicator, a copy of its state, and a closure that measures a change to either side on demand: the vehicle's own acceleration now and in the target lane, and its old and new followers' accelerations before and after. The searches behind those measurements query the network, so they stay in the simulation. The model updates the state and returns a decision (nothing, indicate, cancel or change), which the simulation carries out, moving the ghost and setting the indicator, and writes the state back in the apply step. A vehicle partway through a change or still straddling isn't asked. Making room works the same way: the simulation offers the ghost ahead and what following it would take, and the model says whom the vehicle makes room for. This is SUMO's split: its lane changer gathers the neighbouring leaders and followers and its lane-change models (LC2013, SL2015) own their state and decide.

**Suggestion:** drive mandatory changes from the routing model's required lane sequence.

**Suggestion:** mandatory changes could start frustrated, or grow frustrated faster nearer the lane's end, so they ask for room in time.

## Conflicts, merging and junctions

Lanes that interact are grouped into junctions, and a junction keeps vehicles on its lanes apart. A lane belongs to at most one junction. Junctions are a dumb backstop: they keep vehicles that can no longer stop from running into each other, and nothing more. The order vehicles go in is for barriers, later, with gap acceptance, signals and arrival order.

- **Conflicts:** a junction stores, for each ordered pair of its lanes, where they conflict, if they do: the kind (merge or cross) and the overlapping s-range on each lane. It is a dense table, `lanes²` entries with each pair stored both ways round, so a lookup is two indexes and gives "my" range and "theirs" directly. A lane holds its junction and its index there.
- **Merge or cross** comes from connectivity, not geometry: lanes leading into the same lane merge. Lanes diverging from the same lane, lanes one after the other, and neighbours never conflict: joining every branch at a fork, and leader searches, keep vehicles on them apart. Any other pair whose outlines overlap crosses.
- **Finding overlaps:** each segment's piece of a lane's outline, between its ribs, is a convex quadrilateral, and every pair of pieces from the two lanes is clipped one against the other (Sutherland–Hodgman). Each lane's s-range runs from the first overlap to the last, the union if they overlap in several places, which shouldn't happen in practice. Overlaps under 10 cm² don't count, so lanes touching along an edge don't conflict. A merge whose outlines don't overlap gets the lanes' ends.
- **Building:** until the `network` crate exists, `SimNetwork::add_junction` takes a junction's lanes, once they are connected, and computes their conflicts.
- **One queue per junction,** part of the simulation state and of snapshots: the vehicles **committed** there, which can no longer stop short of their conflicts, in the order they committed, and the lanes vehicles not yet committed are **approaching** it along, sorted.
- **Arriving:** a vehicle arrives at a junction once a lane of the junction is on its path and nothing between it and the junction holds it back: no barrier it has not committed to, and no other junction it hasn't physically entered, by having a record on one of its lanes. So three events can make it arrive: its path is extended or rebuilt, it commits to a barrier, or it enters a junction. Arriving depends on speed only through how far its path reaches. Arriving, it adds the lanes of its path, from the one it is on to the first of the junction's lanes with a conflict, that lane included, to those the junction is approached along. The junction keeps no list of who is approaching, only where.
- **Committing:** every substep, the junction scans back from the start of the conflict on each of its lanes being approached, a lane being in at most one junction and so having at most one conflict zone: along that lane's occupants, then the lanes leading in, branch by branch, following only lanes it is approached along. On each branch it passes over ghosts, committed vehicles and vehicles whose path has left the way and goes nowhere in the junction, and at the first other vehicle:
  - one headed onto the lane commits, joining the end of the committed vehicles, once it can no longer stop at `SAFE_BRAKING` (4 m/s²) before the conflict, or its front is past the start of it, and the scan goes on behind it; if it can still stop, the scan stops there;
  - one headed elsewhere in the junction, or whose path ends before it could leave the way, stops the scan: it may yet go in ahead of those behind it, and another lane's scan, or a later one, deals with it.

  So nobody commits before a vehicle ahead of it on its lane that might still go in. That has to be enforced: IDM keeps a follower able to stop behind its leader only roughly, and one closing fast on a slow leader, which can still stop, often can't stop itself. A lane a scan finds no one on, nor beyond, leaves those the junction is approached along, so vehicles only ever add lanes, and stale ones go once a scan passes them; lanes behind a vehicle a scan stopped at stay. `SAFE_BRAKING` rather than comfortable deceleration: vehicles at the front of the ways in already slow for committed vehicles in their way, so committing late costs no harder braking, and keeps the order vehicles commit in close to the order they reach their conflicts. Against comfortable deceleration, it cut hard braking by about four fifths at the ramps and up to three fifths at lightly loaded crossroads, though about a fifth more at the busiest crossroads. If braking at junctions is too harsh, vehicles should commit earlier. One with no conflict on its path never commits, and never needs to.
- **Releasing:** every substep, a committed vehicle that could now stop at `SAFE_BRAKING` a car length short of its conflict is released, unless a vehicle committed after it is behind it on its way in. The junction itself can bring a committed vehicle to a stop short of its conflict, with the margin it has up to `MAX_BRAKING`, and so can traffic ahead; one that plainly can stop shouldn't hold back those committed after it. The car length is the hysteresis against committing, and keeps a vehicle held at the line committed. Releasing from the back of each lane keeps vehicles committed in their order along it. A released vehicle's way in is marked approached again, and it commits again, at the end, when it must.
- **Lane changes:** none start in a junction or run into one (see Lane membership, lane changes and diverging), so a vehicle is never partway across a junction's lanes. A committed vehicle that changes lanes on its way in keeps its place, and its new path may make it arrive elsewhere.
- **Leaving:** a vehicle leaves the queue once no lane of the junction is on its path and it has no record on any of them, so its whole body is clear, or when it is removed.
- **Keeping apart:** at each longitudinal update, after every vehicle's own terms, the junction walks its committed vehicles and, for each pair, ahead and behind, and for each committed vehicle and each vehicle at the front of a way in, looks up the conflicts between each junction lane on the follower's path and each of the junction's lanes. Each gives the follower a term, folded in with its others like its old side's, crossing or merging alike: until the one ahead has cleared the conflict on its lane, its rear past the far end, the gentlest of three ways of keeping out of its way, each safe on its own:
  - **Stopping short** of the conflict, as at a stopped vehicle there.
  - **Following its progress:** a virtual vehicle at the speed of the one ahead, as far past the start of the follower's conflict as that one's rear is past the end of its own, so it reaches that start just as the one ahead clears. This is virtual vehicle mapping, from cooperative merging (Uno et al., 1999). Crossing streams can roll through one behind another rather than stopping at the line.
  - **Following it,** while it moves the same general way as the follower's conflict runs and will still as it leaves its own (positive dot products of its heading, and of its lane's direction at the end of its conflict, with the follower's lane's direction at the start of its conflict): its nearest corner measured along that direction, from the start of the conflict, at its speed along that direction. A convex body is nowhere nearer along a direction than its nearest corner, at any angle, for a conflict straight on the follower's side; a lane bends one way by less than half a turn, so one heading the same way now and as it leaves never swings back across in between. Without that second test, a vehicle turning across could lead the follower into its conflict and then swing back into it. At a merge this is ordinary car-following through the taper.

  Between committing at `SAFE_BRAKING` and IDM's limit, `MAX_BRAKING`, lies the backstop's margin, so two vehicles committing at once still stop apart. A follower already past the start of its conflict is held back all the same: it shouldn't be there before the one ahead has cleared, and at a merge, where driving on never clears the way, dropping back is the only way out. One not headed onto the lane counts only while its body, lingering past the lane's end or beside it, is in the overlap, and only by stopping short or following it. This is conservative; gap acceptance (below) can let a follower go earlier. The debug view records only the term chosen.
- **Vehicles not yet committed** at the front of each way in, those the scans stopped at headed onto the lane scanned, take terms from every committed vehicle, as if committed after them, so they slow gently for a committed vehicle in their way rather than driving on until they commit and then braking hard. Those behind them follow them. They hold no one back.
- **Cost:** O(c²) pairs per junction at 20 Hz, with c the committed vehicles: tens at most. Each substep, each lane being approached scans back past the committed vehicles on its way to the first vehicle not committed, usually one to three records, touching only lanes vehicles approach along; a junction approached along nothing costs nothing. Releasing checks each committed vehicle against those committed after it, O(c²) per substep.
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
- **The order:** each merge keeps an ordered list of the vehicles approaching it, part of the simulation state and of snapshots. There are no scans of the network: a vehicle joins a merge's list when one of the merge's lanes comes onto its path beyond the lane it follows, when its path grows, it changes lanes or it is inserted, at its place by estimated arrival. It leaves once it advances onto one of the merge's lanes, from when its followers find it there through their records, or when a lane change takes the merge off its path, or when it is removed.
- **Estimated arrival:** the distance from its front to the merge point over its speed, at least 1 m/s, so a stopped vehicle's doesn't blow up. At each longitudinal update the list is re-sorted by adjacent swaps, so it changes only where it must:
  - two vehicles headed for the same lane of the merge keep the order they are in along it, nearest the merge point first, since neither can overtake the other there;
  - a vehicle whose front is past the merge point goes ahead of one whose front isn't;
  - otherwise the one behind goes ahead only once its estimated arrival is earlier by more than 0.5 s, so two arriving together don't swap back and forth.
- **The term:** at each longitudinal update, the list is walked from the front, and each vehicle F whose front hasn't reached the merge point takes a term from the vehicle L before it, folded in with its other terms as a junction's are. With `d_F` the distance from F's front to the merge point, `d_L` that from L's rear, and `g = s0 + v_L·T` the gap F wants behind L, from F's own minimum gap and headway at L's speed, L's rear is `g` past the merge point at `t = (d_L + g) / v_L`. F arrives no sooner by braking at `a = 2(d_F − v_F·t) / t²`, or, if that would stop it before `t`, by stopping at the merge point, `−v_F² / 2d_F`, as when L is stopped short of it. The term is the larger of that and IDM's term for stopping at the merge point, as at a stopped vehicle there, and no harsher than `MAX_BRAKING`: F never brakes harder to open a gap than to stop short, and never stops short where a gentler gap will do. A leader on F's own lane is followed as usual too; there the term asks little more, since at the desired gap behind it the term is zero.
- **Lane changes** are allowed throughout: changing out of a lane leading to a merge leaves its list, and changing into one joins it, at its place by estimated arrival.
- **No barrier:** the terms alone open the gaps.
- **Cost:** a list per merge of the vehicles approaching it, tens at most, sorted by adjacent swaps that are few once it is in order, and one term per vehicle in it, at 20 Hz.

**Off the beaten path:** SUMO orders vehicles at its `zipper` junctions by estimated arrival, and Treiber's simulator treats an on-ramp as mandatory MOBIL lane changes. Ordering by estimated arrival with hysteresis and opening each gap by the gentler of a kinematic gap term and stopping short is this simulator's own.

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

**Events worth recording:** leader links and gaps, conflict activations, barrier commitments, lane-change decisions with their MOBIL scores, target paths and lookahead points, and checks for passing a vehicle ahead, with the follower's predicted pose, the other vehicle's nearer rear corner and the gap between them.

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
