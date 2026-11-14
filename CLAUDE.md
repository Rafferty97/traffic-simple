# Traffic Simulator

A Rust traffic simulation aiming for city and freeway traffic that _looks_ realistic at 60 fps, built as simply and fast as possible. Visual realism beats aggregate accuracy.

@docs/DESIGN.md

## Ownership rules

- Alexander owns the architecture, data layout, module boundaries and public APIs. Do not change any of these without asking first. Propose the change, explain why, and wait.
- Alexander must approve, before they are added: new dependencies (including dev-dependencies), `unsafe` code, and any allocation inside the simulation loop (`Simulation::step` and everything it calls). Propose it, explain why, and wait.
- `docs/DESIGN.md` is the canonical design. If a task conflicts with it, stop and say so rather than working around it. Update it only when Alexander approves a design change, in the same commit as the code.
- When suggesting an approach, say how established tools (SUMO, OpenDRIVE, IDM/MOBIL literature, game-engine practice) handle it, and flag explicitly when a suggestion is off the beaten path.

## Working style

- Keep diffs small and reviewable. Alexander reviews on demand, not line by line. Changes to core types and data layout (struct/enum definitions), public APIs, new dependencies, `unsafe`, and any allocation or dynamic dispatch in per-frame hot paths need his explicit approval before you make them.
- In every change summary, call out any edits to the occupancy update, conflict activation, projection, or barrier commitment logic, any new special cases or branches added to the core, and any test assertions you added, changed or removed, so he knows where to look.
- Prefer the simplest design that meets the goal. If a change adds special cases, first ask whether a different representation would remove them.
- Don't add speculative abstractions. "Swappable" means a clean module boundary, not traits or dynamic dispatch, unless the design says otherwise.
- No trait objects or allocation in per-frame hot paths.
- Debug output goes through the generic `DebugSink` parameter using closures (`sink.record(|| …)`), so it costs nothing with the no-op sink.

## Workspace

- `sim`: core simulation; depends on nothing else in the workspace
- `network`: source-of-truth network → `SimNetwork` + render geometry
- `ui`: egui app with pretty and debug views
- `demand` (proposed): routing models and spawners

## Commands

<!-- Adjust to match the repo -->

- Build: `cargo build --workspace`
- Test: `cargo test --workspace`
- Lint: `cargo clippy --workspace --all-targets -- -D warnings`
- Format: `cargo fmt --all`
