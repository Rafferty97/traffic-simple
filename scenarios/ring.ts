// name: Three-lane ring
// A ring road joined to itself, three lanes clockwise, the inner one 1000 m
// round: about 160 m across, roomy enough for vehicles to accelerate, settle
// into gaps and change lanes.

const circumference = 1000;
const lanes = 3;
const width = 3.5;
const r = circumference / (2 * Math.PI);

// A whole turn clockwise, from the east.
const path = new Path([r, 0]).arcTo([0, 0], -2 * Math.PI);

const net = new Network(Side.Left);
// Going clockwise, the lanes outward of the path are on its left.
const forward = [];
for (let i = 0; i < lanes; i++) {
  forward.push({ offset: width * i, left: width / 2, right: width / 2 });
}
const ring = net.addRoad({ path, speedLimit: kmh(50), forward });
net.join(ring, ring);

const compiled = compile(net);
const sim = compiled.simulate();
const perLane = 20;
for (let i = 0; i < lanes; i++) {
  for (let j = 0; j < perLane; j++) {
    // Staggered across the lanes, along the path.
    const s = (circumference * (j + i / lanes)) / perLane;
    sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(ring, i, s, kmh(40)));
  }
}
