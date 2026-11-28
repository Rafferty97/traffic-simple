// name: Two lanes zipping into one
// A two-lane road whose outer lane eases in over the inner one, from 150 m to
// 230 m along, until the two are one, and runs on as a one-lane road. Where
// the lanes overlap there is no line between them, and traffic takes turns.

const net = new Network(Side.Left);
const width = 3.5;
const inner: RoadLane = { offset: -width / 2, left: width / 2, right: width / 2 };
const outer: RoadLane = [
  [150, -1.5 * width, width / 2, width / 2],
  [230, -width / 2, width / 2, width / 2],
];
const two = net.addRoad({
  path: new Path([0, 0]).lineTo([250, 0]),
  speedLimit: kmh(60),
  forward: [inner, outer],
});
const one = net.addRoad({
  path: new Path([250, 0]).lineTo([450, 0]),
  speedLimit: kmh(60),
  forward: [inner],
});
net.join(two, one);

const compiled = compile(net);
const sim = compiled.simulate();
for (let s = 10; s < 140; s += 20) {
  for (const lane of [0, 1]) {
    const at = compiled.place(two, lane, s + 10 * lane, kmh(50));
    sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, at);
  }
}
