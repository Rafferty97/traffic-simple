# Comparing with the old simulator (Suggestion)

A plan, not yet started: running the old simulator on the same networks and demand as this one, so scenarios can be compared headless and, optionally, side by side in the UI. Nothing here is decided; it is written down to come back to.

## Which old simulator

The newest old simulator is the `master` branch of this repository, not the crates.io release. `traffic-sim` 0.1.1 on crates.io predates its pathfinding, its route-driven lane changes, destinations and its newer traffic light API, all on `master`. `v3` branched from `master`'s tip.

Both packages are named `traffic-sim`, but a crate can depend on both, renaming one: a throwaway crate depending on `sim` by path and on `master` by git, as `old_sim = { package = "traffic-sim", git = "…", rev = "…" }`, builds and links with today's toolchain. Both are GPL-3.0-or-later.

## How the network maps

The old simulator is built through calls, `add_link`, `add_link_connection` and so on, so an adapter can build it from a compiled `SimNetwork`, lane for link, without the `network` crate changing.

| Here | Old (`master`) | Fit |
| --- | --- | --- |
| Lane polylines | `add_link`, with the polyline as a curve: its `ParametricCurve2d` trait is public, and it resamples each link every 0.5 m | Good |
| Successors | `add_link_connection` | Exact |
| Neighbours and lane-change permissions | `add_link_group`, lanes left to right, with a lane-change mask | Good |
| Junction conflicts and merges | `add_link_convergance` for each conflicting pair; it finds the conflict point from the curves itself | Good |
| Signal plans, amber, clearance | `TrafficLight`: movements, phases as masks, amber time, and how long a conflicting movement waits | Good |
| Give-way and stop lines | `TrafficControl::Yield { distance, must_stop, priority }` on the connector's link | Fair: needs the priority levels compilation works out, which `network` would have to report |
| Far-side turners waiting inside a junction | `Yield` on a link from the turner's give-way line along its connector, as old links yield where they are entered | Fair |
| Destinations | `set_vehicle_destination`, a link, with its own Dijkstra's algorithm and lane choice | Fair |
| All-way stops, keep-clear zones, turning on red, room beyond | Nothing | Missing |
| Lane widths, keeping to a side, lane changes for speed, merge ordering | Nothing: it changes lanes only for its route | Missing |
| Inserting | `add_vehicle`, at rest at a link's start, with no check for room | Poor |

## Making comparisons fair

- **Inserting:** old vehicles always start from rest at a link's start. A small `add_vehicle_at(link, pos, vel)` on `master` would let both simulators insert alike; the adapter checks for room itself, as `try_insert` does.
- **Determinism:** `randomise_velocity_adjusts` draws from an unseeded generator, so the adapter shouldn't call it; drivers' variation comes from the attributes it inserts them with.
- **Stepping:** both step at the same fixed `dt`, 1/30 s; the old simulator asks for no more than about 0.2 s.
- **What to compare on:** signalised crossroads, priority crossroads and T-junctions, the grid and the ramps. Where the old simulator lacks a feature, its behaviour differs by design: all-way stops, keep-clear zones, turning on red.

## The plan

1. **An adapter crate,** `legacy`, depending on `sim` and on `master` pinned to a commit: it builds an old simulation from a `SimNetwork`, keeping which link each lane became, inserts vehicles with matching attributes and destinations, and reads their poses and the lights back. About 400–600 lines, with tests.
2. **A headless comparison,** feeding both simulators the same scenario and the same demand, each vehicle's entry, departure time and exit, and reporting throughput, travel times and stops side by side: most of the planned `validation` crate (see [the design](DESIGN.md), Validation).
3. **The UI, side by side** (optional): two scenes sharing the camera, fed by the same spawner. The old simulator gets the pretty view only, its vehicles drawn from their position and direction and its signals from its lights; the inspector, debug layers and replays stay with the new one. Making the whole UI generic over both simulators would mean an enum or trait across code built on the new simulator's debug API, and isn't worth it.

Steps 1 and 2 are about a working session's effort, and step 3 about half that again.

## Needs approval

- **A new dependency:** the old simulator, and with it cgmath, rand 0.8, rand_distr, pathfinding, itertools 0.10, smallvec, once_cell and log. It would sit in its own crate, outside `sim` and its step; the UI would depend on it only for step 3, perhaps behind a cargo feature so everyday builds don't compile it.
- **Changes to `master`,** for inserting vehicles where and as fast as the new simulator does.
