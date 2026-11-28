// name: Two roads merging, then forking
// Two one-lane roads 60 m apart curve in to run alongside each other, then
// squeeze together into one, which forks again 200 m on, vehicles choosing a
// branch at random. The lanes that squeeze together start where they begin to
// overlap: from further apart, merges lock up (see `docs/known-issues.md`,
// Merges starting far apart).

const net = new Network(Side.Left);
const lane = [{ offset: 0, left: 1.75, right: 1.75 }];
const road = (path) => net.addRoad({ path, speedLimit: kmh(60), forward: lane });

// North of the merged road, `side` 1, or south, -1.
const approach = (side) =>
  new Path([0, 30 * side]).lineTo([100, 30 * side]).cubicTo([175, 30 * side], [175, 1.75 * side], [250, 1.75 * side]);
const squeeze = (side) => new Path([250, 1.75 * side]).cubicTo([275, 1.75 * side], [275, 0], [300, 0]);
const branch = (side) => new Path([500, 0]).cubicTo([575, 0], [625, 30 * side], [700, 30 * side]).lineTo([800, 30 * side]);

const north = road(approach(1));
const south = road(approach(-1));
const northIn = road(squeeze(1));
const southIn = road(squeeze(-1));
const merged = road(new Path([300, 0]).lineTo([500, 0]));
const left = road(branch(1));
const right = road(branch(-1));

net.join(north, northIn);
net.join(south, southIn);
for (const from of [northIn, southIn]) net.join(from, merged);
for (const to of [left, right]) net.join(merged, to);

const sim = simulate(net);
for (let s = 10; s < 240; s += 25) {
  sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, { lane: north.lane(0), s, speed: kmh(50) });
  sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, { lane: south.lane(0), s: s + 8, speed: kmh(50) });
}
