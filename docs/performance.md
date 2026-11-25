# Performance measurements

How fast a step is, measured over time, so a change's effect, or a slow drift, shows. The goal is a step for 10,000 vehicles well inside a frame (see [the design](DESIGN.md), Goals).

## How to measure

- **Benchmarks:** `cargo bench -p traffic-network --bench city -- city_step`: `10k_16x16` keeps 10,000 vehicles on a 16 by 16 grid of 200 m blocks; `flooded_4x4` floods a 4 by 4 grid of 100 m blocks with traffic. `junctions` and `sim/benches/ring.rs` cover junctions and lane changes.
- **Where:** on the idle desktop (`ssh alex-pc`, see below), not a laptop in use: on the laptop, whatever else was running moved a step by 15–20%, more than most changes do.
- **Compare only within one session, back to back.** The same tree measured 2.40 ms one evening and 2.56–2.58 ms later the same night on the same machine, so numbers from different sessions aren't comparable. To measure a change, benchmark the commits before and after one after the other, each from a clean `git archive` of the commit, so nothing uncommitted leaks in.
- **Profiling:** build with `--profile profiling` (release with debug symbols) and sample with samply, or, on macOS without it, record with `xctrace record --template 'Time Profiler' --launch -- <bench binary> --bench --profile-time 25 <bench>`, export the `time-profile` table with `xctrace export`, and resolve each sampled address with `atos -i -o <binary> -l <load address>`, which names inlined frames. Inlined frames' line attribution is approximate: check a surprising one against the code.

## Measurements

Times are criterion's middle estimate per step. Each table is one session.

### 2026-11-25, desktop (Ryzen 9 7900, Windows), back to back

| Commit | Change | `10k_16x16` | `flooded_4x4` | `crossroads_2` | `signals_2` | `ramps` |
| --- | --- | --- | --- | --- | --- | --- |
| `8e2050d` | before the junction's order of decided pairs | 2.456 ms | 189.6 µs | 34.74 µs | 37.36 µs | 16.64 µs |
| `5b6a749` | the order of decided pairs, whole connectors | 2.416 ms | 187.8 µs | 34.38 µs | 36.60 µs | 16.40 µs |

Deciding pairs from the junction's own terms, and lifting vehicles in the order, cost nothing measurable against the cycle check it replaced: every benchmark was a percent or two faster, within what back-to-back runs vary by.

### 2026-11-24, desktop (Ryzen 9 7900, Windows), back to back

| Commit | Change | `10k_16x16` | `flooded_4x4` |
| --- | --- | --- | --- |
| `65929c9` | before the three fixes below | 2.58 ms | 195.1 µs |
| `4daa415` | path reach memoised; lane-change permission once a side; junction scans' queue lookup by slot | 2.56 ms | 193.0 µs |
| `c8252a7` | slowing for lower limits ahead; the driver model | 2.58 ms | 195.7 µs |

The three fixes were worth about 1%. The profile they came from (below) had them at about 13% of a step between them, so either the sampler overstated them, or what they saved moved elsewhere; worth profiling again with samply, which sees inlined functions.

### 2026-11-24, laptop (Apple silicon), in use

| Commit | `10k_16x16` | `flooded_4x4` |
| --- | --- | --- |
| `65929c9` | 1.72 ms | 153 µs (135–170) |
| `4daa415` | 1.47 ms | 101 µs |

Not to be trusted: the first run's spread shows the machine was busy, and the desktop, back to back, found 1%, not 15–20%.

## Where the time goes

### 2026-11-25, laptop (Apple silicon), `297fb1f`

`xctrace` and `atos -i` on `10k_16x16`, 25 s, 28,500 samples in the step, so inlined functions are seen. The laptop was in use (load about 3), so the step times beside it, 1.41 ms for `10k_16x16`, 107 µs for `flooded_4x4`, 663 µs for `ring_step/10000`, are only a snapshot; the shares are what matter.

| Share | Phase | Inside it |
| --- | --- | --- |
| 37% | longitudinal update | own leader search and terms 23%, gap acceptance 9.7% of it (`has_gap` → `clear`, walking back the priority lanes); junction terms 8.8%; applying the decisions 4%; friction's lane speeds 3.4% |
| 22% | lane-change update | `incentives` 13%: `room_to_change` 6%, `record_index`'s linear search 3.5%; `measure_prospects` 2.5%; `recompute_through` 1.8% |
| 18% | motion | `smooth_point_along` 9.4%, placing the front axle; projecting the rear axle onto its lane 4.7% |
| 16% | occupancy | re-projecting each record's two rear corners and checking each between its edges, about 10% |
| 7% | junction queues | `reorder` 3.6%, `scan_back` and `stops_on` 3% |

Self time is spread thin, much of it in loads (`network.lane`, `geometry().length()`, comparing ids), which suggests a step is bound by memory more than arithmetic; not yet measured with counters. Worth chasing, most promising first:

1. **Each junction pair worked out once** (about 5–7%): `reorder` works out the needs of the queue's pairs, then `junction_terms` asks `decided` of every pair again, which works out `zone` and both `pair_accel`s, and `term` works out the terms again. `decided` could hand back the term it found, or the needs from ordering be kept in a reused buffer (a data layout change in the step: ask first).
2. **`record_index` in `incentives`** (3.5%): `lane_change::decide` walks vehicles in `SlotMap` order, so searches each one's lane for its record. Walking lane by lane, as `update_accels` does, gives the index for free, and touches memory in order.
3. **`room_to_change`** (6%): walks the path for each side at every lane-change update. How far a lane can be changed out of before the lanes beside run out or a junction comes depends only on the network, so could be derived per lane and side.
4. **Placing the front axle** (9.4%): `smooth_point_along` binary-searches its segment every step, with no hint, after summing the path's lane lengths, and branches on which corner it is near. A front-axle segment hint would make the search O(1) (a field on the vehicle: ask first).
5. **Occupancy** (10%): each record projects both rear corners separately and interpolates two half-widths for each. The second corner could start from the segment the first found, and `between_edges` interpolate both edges at once.
6. **Gap acceptance** (9.7%): `has_gap` loops over every lane of the junction for each lane of the path, calling `conflict_between`; a list of each lane's conflicts, derived at build, would cut that, but most of the cost is `clear` walking back, so measure first.

### 2026-11-24, laptop, `sample`

`sample` on the laptop, `10k_16x16`, 20 s, before the fixes above (`65929c9`), the most of a step first:

| Share | Where |
| --- | --- |
| 30% | the longitudinal update, inlined: the leader search, its terms, junction and merge terms |
| 22% | the step's motion and occupancy, inlined |
| 14% | lane-change incentives, `room_to_change` most of it |
| 11% | projecting world positions onto lanes |
| 4% | `junction::stops_on` |
| 4% | `extend_path` |
| 5% | gap acceptance |

`sample` can't see inside inlined functions, so the two largest want a profiler that can before anything is optimised there.
