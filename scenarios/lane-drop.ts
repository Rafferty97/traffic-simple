// name: A lane dropped
// A two-lane road whose outer lane is dropped, easing in over the inner one
// from 150 m to 230 m along, and runs on as a one-lane road. The lane going on
// is marked beside the taper with a short dashed line, and drivers bound for
// the road's end change out of the dropped lane before its taper where they
// can, going through it where they can't.

const net = new Network(Side.Left);
const width = 3.5;
const inner: RoadLane = { offset: -width / 2, left: width / 2, right: width / 2 };
const outer: RoadLane = {
  stations: [
    [150, -1.5 * width, width / 2, width / 2],
    [230, -width / 2, width / 2, width / 2],
  ],
  dropped: true,
};
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
const end = sim.addDestination(compiled.lanes(one, 0));
for (let s = 10; s < 140; s += 20) {
  for (const lane of [0, 1]) {
    const at = compiled.place(two, lane, s + 10 * lane, kmh(50));
    sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, { ...at, route: { Destination: end } });
  }
}
