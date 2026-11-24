# Routing

How vehicles choose their way at forks, and what lane changes their routes need, behind the module boundary `models::routing`. The lane-change model turns those needs into desire (see [Lane changes](lane-change.md)).

Each vehicle carries a `Route`, given at insertion, which chooses the next lane when its path is extended past a lane with several successors; the choice is then part of its path. Route models are separate, in `models::routing`, each replaced by editing it, and say what lane changes a route needs, which the lane-change model turns into desire.

- **Random:** a SplitMix64 generator seeded at insertion, drawn from only at forks, so the same seed takes the same way, and snapshots and replays reproduce it. It chooses only among its own lane's successors that its class may use.
- **Destination** (**Planned**): toward a destination, a set of lanes registered with the simulation. Each destination has a route table for each vehicle class sent there, derived when it is first needed outside a step, at registering or inserting, and rebuilt on restore:
  - **Time to the destination** from the start of every lane, at the speed limits, by Dijkstra's algorithm backward from the destination, over the lanes the vehicle's class may use. A lane change costs no time, since the vehicle travels on while changing, but a way that needs more changes than there is room for isn't a way: each change needs its own length (see [the design](../DESIGN.md), Lateral path) at the limit, before the lanes it needs run out. Between ways equally quick, the one with fewer changes wins.
  - **Choosing:** at a fork, the successor with the least time, ties going to the first, so runs repeat. A vehicle that misses its way, failing to change lanes in time, simply takes the best way from where it is, so rerouting needs nothing more. One with no way left drives on and leaves wherever its lanes take it.
  - **Lane changes needed:** for each lane, how far along it a vehicle can go and still reach the destination with one, two, three or four lane changes, four or more counting as four: the old prototype's lane distances, as SUMO's "best lanes" does it. They are worked out backward from the destination over the lanes from which the time only falls: a lane that leads nowhere toward it must be left by its end, and a neighbour's distances count for one more change. The route desire reads them (see [Lane changes](lane-change.md)).
  - **Cost:** five numbers per lane per destination, 20 bytes.

Every route's lane-change desire comes from a table: its destination's, or for random routes one derived for the network as a whole, where every lane leads somewhere and only lanes left early must be left by their end.
