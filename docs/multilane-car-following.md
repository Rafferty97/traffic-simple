# Multi-lane car following

This document describes the traffic simulator's approach to ensuring collision-free car following in a multi-lane environment
with lane changes and lanes that can merge and diverge.

## Scope and assumptions

- Lanes inside junctions, and crossing lanes generally, are out of scope: junctions keep vehicles on them apart.
- Vehicles that conflict at a merge are already separated longitudinally by the time they reach it. Two vehicles close to
  side by side could each find the other behind it and so miss one another; ordering them beforehand is the responsibility
  of the merge entity, not of this model.
- This model only ever turns "follow" into "pass": a record moves from converging to diverging, never back. Making a vehicle
  follow one it was free to pass, such as a vehicle cutting in, is the responsibility of other systems.
- Vehicles are confined as invariants 2 and 3 say. How that is guaranteed (no corner cutting, lane-change paths that don't
  overshoot) is left aside for now.

## Proof of correctness

The simulation relies on a set of invariants that together should guarantee realistic and collision-free behaviour:

1. Vehicles are registered occupants of every lane they do or will intersect (except lanes still ahead of them). A vehicle
   entering a lane is also inserted into every lane that overlaps it.
2. Vehicles in a lane are confined to that lane.
3. Vehicles changing lanes are confined to the union of their old lane and their new lane.
4. A vehicle searches along its own lane, and its old lane if changing lanes, for occupants, which become its candidate
   leaders. Its search of its old lane only reaches as far as its lane change's end (see Old-lane search).
5. Therefore, a vehicle cannot miss a candidate leader unless they will never intersect.
6. A vehicle follows every candidate leader unless it can prove that their paths don't intersect.
7. Thus, vehicles can never collide.

## Testing whether a vehicle can pass another

Consider a vehicle A on lane X, deciding whether it can pass a vehicle B on lane Y, as a result of finding B's occupancy
record on lane Z ahead of A's own occupancy record on lane Z.

### Occupancy records

An occupant is classified as either "in lane", "converging" or "diverging" to a certain side (left/right). A converging
record also says where it turns diverging, and to which side. A vehicle's ghost, its indicator, is a fourth kind of
record, which this model never follows or passes.

A record holds only the vehicle, its classification and the lane coordinates of its rearmost corner, which are both its
position along the lane and the hint for projecting it next. The pass check reads the leader's rear corners from the
vehicle itself.

### Old-lane search

A follower is only ever diverging on its old lane: at a fork it doesn't search the branches it didn't take. Its search of
its old lane only reaches as far as the end of its lane change, since from there it is confined to its new lane, so
nothing it would only reach beyond it can be in its way in the old lane. Any collision elsewhere would be in a lane they
both search, where that lane's check deals with it. Once A has actually left the lane, its record there is dropped, and
it doesn't search the lane at all.

Leaders found before the change's end that A will have left the lane by the time it reaches are passed by the check
below, whatever they are doing.

### Pass check

A's own record on Z is classified too, so each of A and B is in lane, converging or diverging. Both checks start from A's
projected pose: the pose A will have when its front reaches B's rear.

- **Gone:** if A is diverging and its projected body is wholly outside Z, it may pass B, whatever B is doing. A diverging
  vehicle never comes back into Z once it has left, and B is confined to Z and its own other lanes, so they can only meet
  in a lane A searches as its own (invariant 1), where that search deals with B. B moves on meanwhile, so A reaches it
  further on, further out of Z still.
- **Projection:** if at that pose A clears B's rear corners with sufficient margin, it may pass. This only has to check
  the trailing side of the diverging vehicle, and is only sound where neither vehicle moves toward the other sideways
  after that point.

The gone check applies whenever A is diverging. Whether the projection applies depends on the two classifications:

| Follower A \ leader B | In lane    | Converging | Diverging                                    |
| --------------------- | ---------- | ---------- | -------------------------------------------- |
| **In lane**           | follow     | follow     | projection                                   |
| **Converging**        | follow     | follow     | follow                                       |
| **Diverging**         | projection | follow     | projection if they diverge to opposite sides |

If neither check applies, or neither succeeds, A follows B.

- A converging leader may still move toward A, so projecting its rear corners proves nothing, though A may be gone.
- A converging follower can't pass anything. The mirror of the old-lane search's horizon would need B to have left the
  lane before A reaches it, and with no timing, B's only known position is now: if it had left Z, it would have no record
  there to find.
- Two vehicles diverging to the same side keep closing in sideways, so they follow.

### Classifying occupancy records

An occupancy record must be classified as "in lane", "converging" or "diverging".
Consider a vehicle V in lane X with an occupancy record on lane Z.

If X is Z, the record is "in lane" if V is not changing lanes, and "converging" if it is.

Otherwise, the record is converging until V's course can no longer bring it back toward Z, and diverging from then on.
That holds once both:

- V's lane change, if any, has ended, and
- V is past the diverge point of X and Z (see Preprocessing).

From there V is confined to X, which never closes in on Z again, so the projection check is sound. Before that, V's course
is traced geometrically (below). If both already hold when the record is made, as for a vehicle in its lane at a fork, the
trace is skipped and the record is diverging at once. Where X and Z merge, the diverge point is never reached, so the
record stays converging.

Since V is confined to X once its lane change has ended, a record only ever needs dropping as V leaves Z for good: once it
is diverging and its whole body is outside Z, or once its rear corners are past Z's end. A converging record is never
dropped for being beside Z. The whole body, not just the rear corners: where Z bends in toward V as V leaves it, V points
back across Z, and its front can still be in Z after its rear has left.

A vehicle on a loop shorter than its course can hold two records on one lane. A record a vehicle joins is only a
duplicate if the vehicle already has one within 2 m of it on that lane.

### Classifying occupancy records geometrically

Step through V's future course, a sample every metre, the first at its actual pose, its lane change and then its lane, until both conditions above hold, and find the point
after which these two always hold:

- While it intersects Z, it is moving monotonically away from Z in the specified direction. As a body turns into a lane
  change its tail swings back toward the lane it leaves by a few centimetres, so coming back by up to 0.1 m, from the
  furthest it has been, still counts as moving away: well inside the passing check's clearance.
- Once its body has stopped intersecting Z, it never intersects it again.

Until that point, the record is converging, and after that point, it is diverging. That point is memoised on the
converging record, as V's rear's position along Z, with the side it diverges toward, so the trace only runs once.

If the trace runs past the end of X or Z before the conditions hold, the record stays converging. It is classified again
when V advances onto the next lane, and when it joins the lanes that Z leads to.

## Preprocessing

When the network is built, every pair of lanes whose outlines overlap, outside junctions, is recorded, with each lane's
diverge point: the position along it after which the two lanes never converge again. It may be 0, as where a lane forks,
and is never reached where they merge.

Pairs that don't overlap have no diverge point. Once V's lane change has ended it is confined to X, which never meets Z, so
the trace only has to run to the end of the lane change.

Junctions keep vehicles on their lanes apart, except where branches leave the same lane: they rely on vehicles at a
fork finding each other, so fork branches are recorded even inside junctions.

The same table says which lanes a vehicle entering a lane is inserted into (invariant 1): every lane that overlaps it.
That applies to every lane it enters, including the successors of its old lane that a vehicle straddling a section
boundary joins.
