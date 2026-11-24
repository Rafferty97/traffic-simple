# Performance measurements

How fast a step is, measured over time, so a change's effect, or a slow drift, shows. The goal is a step for 10,000 vehicles well inside a frame (see [the design](DESIGN.md), Goals).

## How to measure

- **Benchmarks:** `cargo bench -p traffic-network --bench city -- city_step`: `10k_16x16` keeps 10,000 vehicles on a 16 by 16 grid of 200 m blocks; `flooded_4x4` floods a 4 by 4 grid of 100 m blocks with traffic. `junctions` and `sim/benches/ring.rs` cover junctions and lane changes.
- **Where:** on the idle desktop (`ssh alex-pc`, see below), not a laptop in use: on the laptop, whatever else was running moved a step by 15–20%, more than most changes do.
- **Compare only within one session, back to back.** The same tree measured 2.40 ms one evening and 2.56–2.58 ms later the same night on the same machine, so numbers from different sessions aren't comparable. To measure a change, benchmark the commits before and after one after the other, each from a clean `git archive` of the commit, so nothing uncommitted leaks in.
- **Profiling:** build with `--profile profiling` (release with debug symbols) and sample with samply, or `sample` on macOS.

## Measurements

Times are criterion's middle estimate per step. Each table is one session.

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
