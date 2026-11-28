// name: Curves, one path shared by two roads
// One path of lines and cubic Béziers, sliced into two roads: a two-lane road
// along its first stretch, and a one-lane road narrowing on along the rest.

const path = new Path([0, 0])
  .lineTo([150, 0])
  .cubicTo([250, 0], [300, 50], [300, 150])
  .cubicTo([300, 250], [400, 300], [500, 300])
  .lineTo([650, 300]);
const split = 300;

const net = new Network(Side.Left);
const wide = net.addRoad({
  path: path.slice(0, split),
  speedLimit: kmh(60),
  forward: [
    { offset: -1.75, left: 1.75, right: 1.75 },
    { offset: -5.25, left: 1.75, right: 1.75 },
  ],
});
const narrow = net.addRoad({
  path: path.slice(split + 20, path.length()),
  speedLimit: mph(30),
  forward: [{ offset: -1.6, left: 1.6, right: 1.6 }],
});

const compiled = compile(net);
const sim = compiled.simulate();
for (let s = 10; s < 250; s += 30) {
  sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(wide, 0, s, kmh(50)));
  sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(wide, 1, s + 15, kmh(45)));
}
for (let s = 10; s < 300; s += 35) {
  sim.insert(VehicleSpec.PASSENGER_CAR, Driver.DEFAULT, compiled.place(narrow, 0, s, mph(25)));
}
