# Plan: rebuilding `network` alongside scenarios scripted in JS

**Suggestion** throughout, until approved; once a slice is done, what it settled moves into [the design](DESIGN.md) and leaves this plan.

## Goal

The UI loads scenarios written in JS instead of choosing from a fixed set. A script builds a road network, compiles it, inserts vehicles, and runs alongside the simulation, reading it each step and acting on it: inserting and removing vehicles, forcing lane changes, holding vehicles and setting lights. Correctness stays in Rust tests, so translating a script into a Rust test should be mechanical.

The `network` crate is rebuilt at the same time, a feature at a time, each exposed to JS as it is built so its API can be tried out as it is designed. Its API grew by accretion, and is redesigned from scratch; its algorithms, much of them hard won, are carried over.

SUMO's TraCI is the model for scripting: an outside script steps the simulation and reads and changes it through its API. This plan differs in building networks at road level rather than importing them, and in running the script in-process rather than over a socket.

## Principles

- **The JS API is the Rust API.** It has the same operations, names in camelCase where Rust uses snake_case, the same argument types as plain objects, ids as opaque values, and errors thrown where Rust returns `Err`. Nothing exists in JS alone: anything a script needs goes into Rust first.
- **Data crosses as serde.** A JS object is the serde form of the Rust type, so a network a script describes is the new crate's network as JSON, and enums take serde's tagging. The bindings convert through serde, so they hold no mapping of their own, and a script's objects can be pasted into Rust as `serde_json::json!` or rewritten as struct literals. A network can be saved and loaded the same way.
- **New API, old algorithms.** Connector curves, swept paths and `unfold`, arms and fitting them, hook turns, and turning control into barriers and the lanes each gives way to are ported from the old crate and cleaned up, not derived afresh.
- **No new dependencies in `sim`.** The JS engine, `rquickjs` (QuickJS), goes in `ui` alone.

## The old crate

The current crate moves to `legacy/network`, as `traffic-network-legacy`, and keeps compiling, so its tests and the stress tests keep passing until each slice ports them. Where a slice is meant to compile what the old crate did, a test compiles a network through both and compares what comes out, lanes, links, conflicts and barriers, so a detail lost in porting shows; where a slice changes behaviour on purpose, the comparison is dropped for it. The legacy crate is deleted once nothing uses it.

The UI moves to scripts in the first slice and drops its fixed scenarios then: an older commit has them.

## The old API's rough edges

To settle as the slices reach them:

- **Description or tuning:** `Intersection` carries `connector_width`, `far_side_straight` and the six fields of `HookGeometry`. Some are geometry the network should state, and some are compiler parameters with defaults that could live apart from it.
- **Turning on red** is set in two places: for the whole network, and per movement.
- **`fit_arms` against `compile`:** two entry points, one of which rewrites the description.
- **What compiling returns:** parallel `Vec<Vec<…>>`s indexed by road and by connection. Scripts and tests want lookups: a road's lanes, a connection's connector, and a road end's lanes in and out (the design's Exits and entries, not built).
- **Ids:** roads are typed, but intersections and connections are bare indices, in errors too.
- **Building a network:** struct literals or builder methods, the same in Rust and JS.
- **Duplicates:** `DrivesOn` against `sim`'s `Side`.
- **One long compile:** `compile.rs` is 1,390 lines, `compile` itself about 500. The new crate compiles in stages: roads, connectors, control into barriers, and render geometry.
- **Who owns scenarios:** the old `scenarios` builds both the UI's scenarios and the tests' fixtures. The UI's become scripts in `scenarios/`, and the tests' move to test support, each written in Rust.

## Slices

Each slice is the Rust API, its serde form, its JS binding, a script to try it with, its render geometry, and its tests ported or written.

1. **The host and roads.** A minimal JS host in the UI: it loads a script from `scenarios/`, named on the command line or picked from a list; the script's `setup` builds and compiles a network and inserts vehicles, and its `step` runs after each simulation step. A script registers its own controls, each configured (a slider, a checkbox, a choice, with its range and default) with a callback for when its value changes. The network: roads with a reference line, lanes and their profiles, joins, so merges and diverges, and the ring and ramps scenarios as scripts. Traffic is the script's to spawn until `demand` exists.
2. **Intersections:** connections and connectors, uncontrolled.
3. **Control:** signals, then priority and all-way stops, then turning on red.
4. **Arms:** stop lines, skew and run outs, and fitting them; then keep-clear zones.
5. **Hook turns and tram tracks.**

Beside them, the simulation's API gains what scripts need that Rust callers lack, in Rust first:

- **Setting a light:** hold a movement at a light, and hand it back to its plan. Nothing sets a light from outside today; slice 3 needs it.
- **Events:** a script reads the state each step and finds its own events, as Rust tests do. Subscriptions, as TraCI has, come only if reading proves too slow.

The translation rules, JS form beside Rust form, are written down as a short table once slice 1 settles them, here and then in the design. Rust tests keep building their situations in Rust: where one shares a script's network, it is the script translated by those rules.

## Later: `demand`

The spawning that scripts, tests and the planned validation harness share: flows, the insertion backlog, origin–destination shares, target counts and vehicle mixes, as the design has it. It gets exposed to JS, so a script and its Rust translation spawn the same traffic from the same seed. It isn't needed for correctness work, so it waits.
