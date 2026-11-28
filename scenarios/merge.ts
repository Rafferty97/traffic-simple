// name: Two roads merging, then forking
// Two one-lane roads 60 m apart curve together and merge into one, which forks
// again 200 m on, vehicles choosing a branch at random. Joining a road in
// common, the two are cut where their lanes start to overlap, and the
// branches where theirs stop, with nothing declared.

const net = new Network(Side.Left);
const lane = [{ offset: 0, left: 1.75, right: 1.75 }];
const road = (path: Path): RoadId =>
  net.addRoad({ path, speedLimit: kmh(60), forward: lane });

// North of the merged road, `side` 1, or south, -1.
const approach = (side: number): Path => new Path([0, 30 * side]).lineTo([150, 30 * side]);
const curveIn = (side: number): Path =>
  new Path([150, 30 * side]).cubicTo([225, 30 * side], [225, 0], [300, 0]);
const branch = (side: number): Path =>
  new Path([500, 0])
    .cubicTo([575, 0], [625, 30 * side], [700, 30 * side])
    .lineTo([800, 30 * side]);

const north = road(approach(1));
const south = road(approach(-1));
const northIn = road(curveIn(1));
const southIn = road(curveIn(-1));
const merged = road(new Path([300, 0]).lineTo([500, 0]));
const left = road(branch(1));
const right = road(branch(-1));

net.join(north, northIn);
net.join(south, southIn);
net.join(northIn, merged);
net.join(southIn, merged);
net.join(merged, left);
net.join(merged, right);

const compiled = compile(net);
const sim = compiled.simulate();
for (let s = 10; s < 140; s += 25) {
  sim.insert(
    VehicleSpec.PASSENGER_CAR,
    Driver.DEFAULT,
    compiled.place(north, 0, s, kmh(50)),
  );
  sim.insert(
    VehicleSpec.PASSENGER_CAR,
    Driver.DEFAULT,
    compiled.place(south, 0, s + 8, kmh(50)),
  );
}
