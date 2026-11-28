// name: A straight road, two lanes each way
// Cars spaced along each lane of a 600 m road, faster in the inner lanes.

const net = new Network(Side.Left);
const road = net.addRoad({
  path: new Path([0, 0]).lineTo([600, 0]),
  speedLimit: kmh(80),
  // Driving on the left: forward lanes are to the right of the path.
  forward: [
    { offset: -1.75, left: 1.75, right: 1.75 },
    { offset: -5.25, left: 1.75, right: 1.75 },
  ],
  reverse: [
    { offset: 1.75, left: 1.75, right: 1.75 },
    { offset: 5.25, left: 1.75, right: 1.75 },
  ],
});

const compiled = compile(net);
const sim = compiled.simulate();
for (let lane = 0; lane < road.laneCount(); lane++) {
  const inner = lane % 2 === 0;
  for (let s = 20; s < 300; s += 40) {
    const at = compiled.place(road, lane, s + (inner ? 0 : 20), kmh(inner ? 70 : 60));
    sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, at);
  }
}
