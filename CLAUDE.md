# Traffic Simulator

A Rust traffic simulation aiming for city and freeway traffic that _looks_ realistic at 60 fps, built as simply and fast as possible. Visual realism beats aggregate accuracy, though traffic flow mustn't break down in inaccurate ways.

@docs/DESIGN.md

## Ownership rules

- Alexander must approve, before they are added: new dependencies, non-trivial `unsafe` code, and any allocation or data layout changes inside the simulation loop (`Simulation::step` and everything it calls). Propose it, explain why, and wait.
- `docs/DESIGN.md` is the canonical design. If a task conflicts with it, stop and say so rather than working around it. The design will naturally evolve and improve over time, so keep it up to date, ideally in the same commit that touches the relevant code.
- When suggesting an approach, say how established tools (SUMO, OpenDRIVE, IDM/MOBIL literature, game-engine practice) handle it, and flag any deviations from the standard approach that deserve a second thought.

## Working style

- Claude does all the typing, and Alexander only reviews code on an as-needs basis. Claude is trusted to keep the project tidy, organised and modular as it works, but should ask for clarification when unsure or a decision requires a judgement call based on taste or the objectives of the project.
- Prefer the simplest design that meets the goal. If a change adds special cases, first ask whether a different representation would remove them. Branches in the hot loop should be avoided wherever possible.
- Don't add speculative abstractions. "Swappable" means a clean module boundary, not traits or dynamic dispatch, unless the design says otherwise.
- No trait objects or allocation in per-frame hot paths.
- Prefer to write code in the hot paths that is conducive to autovectorisation or that could be accelerated by explicit SIMD in the future.
- Debug output goes through the crate-private `DebugSink` parameter the update pipeline passes down, using closures (`sink.record(|| …)`), so it costs nothing with the no-op sink that `step` uses; `step_debug` collects the events for callers.
- Commit your work as you go, with a preference for small, focussed commits, so long as the design is settled and the implementation has been approved and well tested. Sometimes things will be implemented speculatively or as part of an experimentation phase, in which case the code should be left uncommitted and either removed or cleaned up before moving on.

## Workspace

- `sim`: core simulation; depends on nothing else in the workspace
- `network`: source-of-truth network → `SimNetwork` + render geometry
- `ui`: egui app with pretty and debug views
- `demand` (proposed): routing models and spawners

## Commands

- Build: `cargo build --workspace`
- Test: `cargo test --workspace`
- Lint: `cargo clippy --workspace --all-targets -- -D warnings`
- Format: `cargo fmt --all`
