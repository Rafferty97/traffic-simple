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

**Suggestion:** sample curves into lane polylines by tolerance, not at a fixed spacing: at most about 1–2 cm of sagitta (chord error, `c² / 8R`) and about 3° of turn per vertex. Gentle curves get long chords and tight junction connectors get short ones. This is how vector renderers flatten curves.

**Suggestion:** author at road level (roads with lane counts and offsets, junctions with turn movements), as OpenDRIVE and SUMO's edge/lane/connection model do, and expand to bare lanes during compilation. That is how people think about roads, and it keeps the sim free of road concepts.

Generating junction geometry (connector curves and junction surfaces) is the hardest part of this crate. SUMO's netconvert is largely heuristics for it; procedural networks keep it tractable.

## Lane model and geometry

The simulation has no roads, only lanes. Lanes may overlap, merge, diverge or cross, and lane changes may be permitted between them.

- **Geometry:** each lane is a polyline with arc length s and lateral offset t. The (s, t) ↔ world mapping is worked out and fast.
- **Relationships:** successors and predecessors (end-to-start connections); left and right neighbours; overlaps, found by conflict precomputation.
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
- Path: the assigned lane followed by the next few lanes it will take, in a fixed-capacity inline list. Leader search and path tracking look along it across lane boundaries.
- Barrier commitments
- Static attributes: vehicle class, dimensions, driver-model parameters

**Per-lane occupancy**

- Each record is `(s, vehicle id, projection hint)` and lives in the lane's list, not on the vehicle. There is one copy of s, so lists and vehicles can't drift apart, and sorting and leader search never touch the `SlotMap`, with one exception: a passing check against a vehicle moving out of the lane reads that vehicle's rear corners.
- A record's s is the smaller of the vehicle's two rear corners' s on that lane. Followers care about where a vehicle's rear is.
- **Lateral position:** a record also says whether the vehicle is in that lane (following it, settled or converging, or lingering past its end) or diverging from it sideways, and toward which side. It is what the overtaking check reads (see Lane membership, lane changes and diverging).
- **Joining:** a vehicle joins a lane when it advances onto it, which is when its rear axle passes the end of the previous lane.
- **Leaving:** it leaves a lane once both rear corners are outside the lane's geometry: past its end after advancing to a successor, or beside it (more than half the lane's width from the centreline) after a lane change. A record on the lane a vehicle is following is never dropped for being beside it. A vehicle leaves the network when its rear axle passes the end of the lane it follows and that lane has no successors, which is the moment it would otherwise advance; an old lane that dead-ends doesn't count.
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

- **What it reacts to:** the nearest leader along the vehicle's path, vehicles in active conflicts projected into its own lane, and the nearest barrier it has not committed to. The most restrictive one wins.
- **Desired speed:** the lane's speed limit times the vehicle's speed factor, capped at the vehicle's maximum speed. The speed factor is a vehicle attribute, as SUMO's `speedFactor`: without it every driver holds exactly the limit and nobody has a reason to overtake. Its distribution is the caller's concern: SUMO draws it from a normal distribution with mean 1 and, by default, deviation 0.1; the old prototype clamped its draws to [0.75, 1.25].
- **Lower limits ahead:** a lane ahead on the path with a lower limit is its own term. The vehicle needs `(v_lim² − v²) / 2d` to reach that limit by the lane's start, `d` ahead. The term applies only once that is harsher than comfortable braking, so braking starts just in time, and is capped at twice comfortable braking. This comes from the old prototype.
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
- **Where changes happen:** only into a neighbour that the network permits changing into.
- **The decision is a jump:** when a vehicle decides to change lanes, its assigned lane switches to the target lane at once. That switch is what encodes the decision. It joins the target lane's occupant list at the same moment, just as advancing onto a successor does. Its pose doesn't move; it is simply now off-centre on its new lane, and the lateral model steers it smoothly onto the centreline.
- **Leaving the old lane:** it stays a member of the old lane until both rear corners are outside the old lane's geometry, so followers there keep seeing it while it straddles.
- **Old side:** the vehicle remembers which side its old lane is on, as `Option<Side>`, and derives the old lanes by mapping its path to their neighbours on that side. While it straddles partway through the change, it runs a second leader search from its rear axle's position on the old lane, continuing along those neighbours, so it doesn't run into its old leader. That search only reaches as far as the change's end: a vehicle it would only reach later can't be in its way, since by then it is wholly on its new lane. Once the change is complete, it doesn't run at all. Each leader found is its own acceleration term. A vehicle that starts a change too close behind its old leader stops partway, straddling, until that leader moves off or its course will take it clear of it (see Overtaking).
- **Across section boundaries:** a vehicle that advances while still straddling also joins the old-side neighbour of its new lane, so followers there see it. The vehicle straddles while it has a record on the old-side neighbour of the lane it follows. Whenever it loses a record on a lane it isn't following, by that lane's end or its side, it checks this, and the old side clears once it no longer holds.
- **Lateral path:** at the start of a change the vehicle fixes a cubic in u = s − s₀ along its new lane. It starts at the point pure pursuit was aiming at on the old path, so the target point carries on without jumping: s₀ and t₀ are that point's coordinates on the new lane, and m₀ is the old path's slope dt/ds there (zero for parallel lanes). It runs to zero offset and zero slope at u = L: with x = u/L, t(u) = t₀(2x³ − 3x² + 1) + L·m₀(x³ − 2x² + x). L = max(v·T, L_min), with T = 4 s and L_min = 15 m, so peak lateral acceleration is about 6·t₀/T² (about 1.3 m/s² across a 3.5 m lane) at any speed above about 4 m/s. Pure pursuit aims at the cubic's offset at its lookahead point, and at zero beyond L. If the vehicle advances mid-change, s₀ shifts back by the old lane's length.
- **Completion:** a change is complete when the vehicle reaches the end of its cubic. It can't start another until then.
- **Overtaking:** a vehicle still on a follower's lane may be mostly in the next lane, and a follower may itself be leaving the lane. Each record says whether its vehicle is in that lane or diverging from it toward one side, set whenever the record is made or refreshed; `change_lane` marks the old lane's record diverging at once. Passing is checked in world space, as a snapshot taken as the follower reaches the other vehicle's rear. That is only sound while the follower isn't moving toward the other vehicle sideways, since then that moment is the tightest and the gap only grows, so fixed rules decide which pairs are checked at all:
  - **In the lane:** never passed by a follower in the same lane, settled or converging. A follower leaving the lane, in its old-side search, may pass it on the side it is leaving toward.
  - **Diverging:** may be passed on the side it is leaving, unless the follower is itself changing lanes the same way, which is never passed.

  For a pair that is checked, the follower's pose comes from its planned course (its lane change's cubic, or else the centreline), with the heading following the cubic's slope, at the point where its nose is 0.1 m short of the other vehicle's rearmost corner, measured along its own lane, so a check that fails would still not overlap. Along the lanes it follows, the record's s is that corner, so the gap to it places the follower exactly. Along the old lanes, distances differ from its own lanes', on bends or where lanes spread, so the corner is projected onto its own lane through world space instead. The clearance is the distance across that heading from the follower's near edge to the other vehicle's nearer rear corner, as if the follower drove straight ahead; it passes with at least 0.5 m. Only rear corners are measured, which assumes the other vehicle is straight or turned away from the follower, as it is for every pair the rules let through. Being in world space, the check assumes nothing about how neighbouring lanes lie. Weighing up a change, a vehicle takes the cubic it would follow, the same one `change_lane` would give it. So a vehicle that starts a change close behind a stopped leader in its old lane drives on once its course will take it clear, and waits, straddling, only while it won't. On 3.5 m lanes the window for passing is narrow, since a straddling vehicle leaves once both rear corners are outside its old lane. The old prototype did the same for vehicles projected from neighboring lanes.

  **Off the beaten path:** planners and game engines check predicted trajectories in world space at several times; this takes one sample, which the rules make sufficient. If they are ever loosened, for vehicles that can share a lane side by side, it needs more samples along the follower's course.
- **`change_lane`** executes a change directly, with no safety check, so it can drive a vehicle into another. It is the execute half of the intent protocol and a testing hook.

**Lane-change intents.** Decisions use only forward searches; nothing searches backwards.

1. **Advertise:** at its decision tick, a vehicle projects itself onto its permitted neighbors and runs forward searches from there, giving its prospective leader and its own gain. If its own gain is worth it, it would brake gently enough and nobody is alongside (below), it advertises an intent: a flagged ghost record in the target lane's occupant list, kept up to date each substep like any other record. The ghost is the vehicle's indicator.
2. **Respond:** at each acceleration update, every response is cleared and recomputed by ordinary forward searches. The nearest follower on the target lane finds the ghost first. Ghosts brake no one, but that follower also computes its acceleration if the vehicle cut in, as a response to the intent. The follower behind the vehicle on its current lane finds it flagged as leaving, and also computes its acceleration if it weren't there. Each response has one writer, and responses are applied after the read-only searches.
3. **Decide:** at each later decision tick, the vehicle has everything MOBIL needs. It changes once MOBIL's net gain is worth it, the change is safe and it has indicated for at least its minimum indicating time. Executing is the jump to the new lane, and its ghost simply becomes its real record. It cancels only if its own gain falls to zero or below, or no permitted neighbor remains on that side; otherwise it keeps indicating, waiting for a gap. Politeness and safety hold back the change, not the indicator, so a driver whose change would cut someone off indicates and waits rather than flickering on and off; and cancelling has a lower bar than starting, so a gain hovering around the threshold doesn't flicker it either.

- **Nobody alongside:** leader search looks ahead of a vehicle's rear axle, and a follower's search ahead of its own, so a vehicle directly alongside in the target lane is seen by neither. Before indicating and before changing, a vehicle also checks that no record in the target lane's list, real or another vehicle's ghost, overlaps its own extent from rear to front there. This is MOBIL's requirement that both new gaps be positive. A record sits on the lane its rear axle is on, so near either end of the target lane the stretch can also overlap a vehicle whose front has crossed into it but whose rear axle hasn't, or whose rear has crossed out of it. It reads only the records next to that stretch, in the target lane's sorted list and, near its ends, its predecessors' and successors' lists, not a search.
- **No reply:** a missing reply from the new follower means wait, unless no vehicle is close enough behind the ghost to reply (the nearest one behind, on the target lane or a lane leading into it, with the gap from its front to the ghost within the leader horizon), and replies have had two acceleration updates to arrive. Then nobody is near enough behind to hit, and the change is safe on that count. A vehicle further back can't reach the ghost with its leader search, so it would never reply, and counting it would hold the change up for good. Other vehicles' ghosts behind don't count: only a vehicle can reply, and counting ghosts would deadlock a lane that several vehicles indicate into, each waiting on the others.
- **Parameters,** as module constants for now: politeness 0.2, threshold 0.1 m/s², safe braking (b_safe) 4 m/s², minimum indicating time 1 s, and a decision every 0.2 s per vehicle. MOBIL is symmetric for now, with no bias toward either side.
- **Two vehicles into one gap:** a ghost ahead counts as a leader in a vehicle's own search, a follower responds only to the nearest record ahead of it, and a missing response means not safe. So changes into the same gap happen strictly one after another, even from both sides or on the same tick. Two vehicles may indicate at once.
- **Across section boundaries:** when the vehicle advances, its ghost moves to the neighbour of its new lane. If there is none, or the change isn't permitted, the intent is cancelled.
- **Cooperative yielding** comes almost for free: a follower can choose to treat a ghost as a real leader to open a gap.

Discretionary changes use MOBIL (Kesting, Treiber and Helbing's incentive-plus-safety model with a politeness factor), through the intents above.

**Suggestion:** drive mandatory changes from the routing model's required lane sequence.

**Suggestion:** plan for cooperative yielding, where a vehicle in the target lane slows to open a gap for a mandatory change. Without it, dense traffic leaves vehicles stuck at lane ends; SUMO's LC2013 model has explicit cooperation for this reason.

## Conflicts, merging and junctions

Merging and crossing traffic rely on lane-pair conflicts computed ahead of time. A conflict costs nothing until a vehicle approaches it, so cost scales with vehicles, not with the O(n²) conflict pairs in a junction.

- **Each conflict stores:** the two lanes, the overlapping s-range on each, its kind (merge or cross) and which side yields.
- **Finding conflicts:** sample one lane's centerline every 0.25 m and project each sample onto the other lane. A sample is in the conflict when the distance between the centerlines is less than `r·(1 + |cos θ|)`, where r is about half a lane width and θ is the angle between the lanes: a full lane width where the lanes run parallel, shrinking to half a lane width where they cross at right angles. The s-range is padded by one sample at each end.
- **Merge or cross:** a merge when the lanes' directions across the zone have a dot product above 0.25, otherwise a crossing.
- **Zipper merges:** sort the vehicles approaching a merge on both lanes by front position, and have each follow the one ahead of it when that one is on the other lane. This is how the old prototype merged.
- **Merging:** a vehicle approaching a merge projects vehicles in the other lane's overlap range into its own s, and treats them as potential leaders or as traffic to yield to.
- **Crossing:** vehicles on crossing lanes must never overlap inside the conflict zone. Resolved by gap acceptance, plus arrival order where that sets priority (below).
- **Gridlock** is allowed.

**Suggestion:** give each yielding point one of two priority policies.

- **Fixed priority:** the yielding vehicle uses gap acceptance against priority traffic.
- **Arrival order:** for four-way stops and similar, vehicles join a junction queue when they stop at their line and are released first come, first served. Vehicles whose paths don't conflict may go together. SUMO models this as its `allway_stop` junction type. A vehicle joins the queue once it is within a set distance of the line and, at a stop line, has come to a stop; it then gets the next sequence number.

Arrival order also breaks ties where two vehicles would otherwise wait for each other indefinitely.

**Gap acceptance** is a simple, conservative calculation from current speed, distance and maximum acceleration. The yielding vehicle goes only if it can clear the conflict zone before any priority vehicle could reach it, even if that vehicle accelerated at its maximum. Priority vehicles stopped at an uncommitted barrier are ignored, because they won't enter the zone, and this prevents deadlocks.

- **Earliest arrival:** a vehicle at speed v, accelerating at a up to a speed cap v_max, covers distance d in `(√(v² + 2ad) − v) / a` if it doesn't reach the cap, and otherwise in `t₁ + (d − d₁) / v_max`, where `t₁ = (v_max − v) / a` and `d₁ = (v + v_max)·t₁ / 2`. Add a 0.5 s buffer.
- **Only vehicles whose path crosses the conflict count:** a vehicle on the conflicting lane whose path turns off before the zone is ignored.
- **Approaching priority vehicles** are found by forward passes, not by searching upstream from the conflict.
- **Ties:** movements have integer priority levels. A higher level goes first; at equal levels, arrival order decides, by a sequence number taken when each vehicle joined the queue. This breaks every tie deterministically.

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
- **Hold** a vehicle: it brakes at its comfortable deceleration to a stop and waits until released. Held vehicles are kept in a short list on the simulation, not as a flag on every vehicle.
- **Pick** the vehicle nearest a point, within a tolerance, for the UI. Linear scan for now.
- **Routing model:** answers which lane to take next. Implementations include shortest path to a destination (Dijkstra or A*) and a random walk through the network.
- **Heterogeneity** comes from inserting vehicles with varied attributes. Distributions are the caller's concern, not the core's.

- **Checked insertion:** `try_insert` refuses when there is no safe room: when the new vehicle would overlap another, or when it or its new follower would have to brake harder than `SAFE_BRAKING` (4 m/s², MOBIL's b_safe) because of the other. Its speed is capped at its new leader's. Its follower is the nearest vehicle behind it on its lane, or at the end of a lane leading into it, found by scanning for predecessors, which is fine outside the step. `insert` stays unchecked, for tests and scripted setups.

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
- **Head of the queue:** only the first vehicle approaching a barrier should negotiate it; the ones behind just follow. In the old prototype each lane walked its vehicles backwards from the line. Can a vehicle's forward search decide this alone, for example by being the head when the first thing ahead of it is the barrier rather than a leader, or is a backwards pass needed?
- **Approaching priority traffic:** gap acceptance needs the priority vehicles that could reach a conflict in time. With forward passes only, how do they make themselves known at a conflict ahead of them?

## Milestones

Each milestone is a demo that can be finished on its own; anything after it is optional.

1. Workspace skeleton: one straight lane, IDM, kinematics, the pretty and debug views. **Done.**
2. A ring road showing emergent stop-and-go waves, as in the Sugiyama experiment. **Done.**
3. A multi-lane freeway with MOBIL, continuous lane changes and multi-lane membership. Began with the accumulating debug events described under Debug instrumentation, which replaced the vehicle's stored leader. The demo is a three-lane ring that starts empty: the UI spawns vehicles into the outer lane and despawns them from the inner lane, each at a rate, so congestion on the outside drives lane changes leftward. `sim/tests/scenario.rs` runs the same scenario headless.
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
