//! Where roads on the same ground are cut: wherever one of a road's lanes starts
//! or stops overlapping a lane of a road related to it, so that a merge's lanes
//! start about where they come together, and a fork's end about where they
//! part. The simulation orders vehicles approaching a merge only before its
//! lanes start (see `docs/DESIGN.md`, Merges).

use crate::Network;
use crate::compile::Piece;

/// How far before a lane starts overlapping another, or after it stops, its
/// road is cut, in meters: the lanes are sampled afresh after cutting, so this
/// keeps the piece outside the overlap from overlapping by a sliver.
const MARGIN: f64 = 0.25;

/// The shortest stretch a cut leaves, in meters: cuts closer together than
/// this, or to the road's ends, are made once, or not at all.
const SHORTEST: f64 = 0.05;

/// The pairs of roads whose lanes may overlap, each once, the lower index
/// first: each road and itself, roads joining a road in common, as two
/// merging into one or forking from one do, and roads declared to overlap.
pub(crate) fn related(net: &Network) -> Vec<[usize; 2]> {
    let mut pairs: Vec<[usize; 2]> = (0..net.roads().len()).map(|r| [r, r]).collect();
    let joins = net.joins();
    for (i, &[a_from, a_to]) in joins.iter().enumerate() {
        for &[b_from, b_to] in &joins[i + 1..] {
            if a_to == b_to {
                pairs.push([a_from.index(), b_from.index()]);
            }
            if a_from == b_from {
                pairs.push([a_to.index(), b_to.index()]);
            }
        }
    }
    pairs.extend(net.overlaps().iter().map(|[a, b]| [a.index(), b.index()]));
    for pair in &mut pairs {
        pair.sort();
    }
    pairs.sort();
    pairs.dedup();
    pairs
}

/// Where along its reference line each road is cut, in order, from its lanes
/// built whole, `whole[road]`: just before each lane starts overlapping a lane
/// of a road related to it, and just after it stops.
pub(crate) fn cuts(related: &[[usize; 2]], whole: &[Piece]) -> Vec<Vec<f64>> {
    let mut cuts = vec![Vec::new(); whole.len()];
    for &[r1, r2] in related {
        let (p1, p2) = (&whole[r1], &whole[r2]);
        for (a, (lane_a, _)) in p1.lanes.iter().enumerate() {
            for (b, (lane_b, _)) in p2.lanes.iter().enumerate() {
                if r1 == r2 && a >= b {
                    continue;
                }
                let Some([on_a, on_b]) = lane_a.geometry().overlap(lane_b.geometry()) else {
                    continue;
                };
                for (r, piece, lane, on) in [(r1, p1, a, on_a), (r2, p2, b, on_b)] {
                    let length = piece.lanes[lane].0.geometry().length();
                    let (start, end) = (on.start - MARGIN, on.end + MARGIN);
                    if start > SHORTEST {
                        cuts[r].push(piece.reference_s(lane, start));
                    }
                    if end < length - SHORTEST {
                        cuts[r].push(piece.reference_s(lane, end));
                    }
                }
            }
        }
    }
    for (road, cuts) in whole.iter().zip(&mut cuts) {
        let length = road.length();
        cuts.retain(|&s| s > SHORTEST && s < length - SHORTEST);
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|later, earlier| *later - *earlier < SHORTEST);
    }
    cuts
}
