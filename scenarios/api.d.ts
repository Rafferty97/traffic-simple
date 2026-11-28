// The scenario API, as `ui/src/script.rs` binds it, for editors: every `.js`
// file here is checked against it (see `jsconfig.json`). Nothing reads it at
// runtime. Data takes its Rust types' serde forms, their fields in camelCase.
// Keep it in step with the bindings: a UI test checks it declares each global.

/** A point, `[x, y]`, in meters. */
type Point = [number, number];

/** A side of the road, or of a lane as it runs. */
type Side = "Left" | "Right";
declare const Side: { readonly Left: "Left"; readonly Right: "Right" };

/** A speed in km/h, in meters per second. */
declare function kmh(speed: number): number;
/** A speed in mph, in meters per second. */
declare function mph(speed: number): number;

/**
 * A curve in the plane: a start point and segments, each from where the last
 * ended, or a slice of one. Each method returns a new path.
 */
declare class Path {
  constructor(start: Point);
  /** The path, then straight on to `to`. */
  lineTo(to: Point): Path;
  /** The path, then a cubic Bézier to `to` with control points `c1` and `c2`. */
  cubicTo(c1: Point, c2: Point, to: Point): Path;
  /**
   * The path, then a circular arc about `center`, turning by `sweep` radians,
   * counterclockwise if positive, clockwise if negative: a whole turn is a
   * circle. It runs on smoothly only if the path ends square to `center`.
   */
  arcTo(center: Point, sweep: number): Path;
  /**
   * The stretch from `from` to `to`, distances along it in meters. A slice of a
   * slice is measured along the slice.
   */
  slice(from: number, to: number): Path;
  /** How long it is, in meters. */
  length(): number;
}

/** A lane of a road, in meters. */
interface RoadLane {
  /** From the road's path to the lane's centreline, positive to the path's left. */
  offset: number;
  /** From the centreline to the lane's left edge, as the lane runs. */
  left: number;
  /** From the centreline to the lane's right edge, as the lane runs. */
  right: number;
}

/** A road: a path, lanes beside it, and a speed limit. */
interface Road {
  /** The line the road is laid along, which its lanes are offset from. */
  path: Path;
  /** In meters per second (see `kmh` and `mph`). */
  speedLimit: number;
  /** The lanes running the way the path does. */
  forward: RoadLane[];
  /** The lanes running the other way, if any. */
  reverse?: RoadLane[];
}

/** A lane in the simulation. */
type LaneId = number;

/** A road of a network, and through it its lanes in the simulation. */
interface RoadId {
  /**
   * Its `i`th lane: forward lanes first, in the order given, then reverse ones.
   * Throws if it has no `i`th lane.
   */
  lane(i: number): LaneId;
  /** How many lanes it has. */
  laneCount(): number;
}

/** A road network. */
declare class Network {
  /** An empty network, its traffic driving on `drivesOn`. */
  constructor(drivesOn: Side);
  /** Adds a road, and returns its id. */
  addRoad(road: Road): RoadId;
  /**
   * Joins the end of `from` to the start of `to`, which should start where
   * `from` ends: `from`'s forward lanes run on into `to`'s, and `to`'s reverse
   * lanes into `from`'s. Lanes connect wherever one ends where another starts,
   * so two lanes meeting one merge, and one meeting two diverge. A road may
   * join itself, as a ring does. `simulate` throws if a lane at the join meets
   * none across it.
   */
  join(from: RoadId, to: RoadId): void;
}

type VehicleClass = "Car" | "Truck" | "Bus" | "Tram";

/** A vehicle's body and the longitudinal model's parameters, in meters, m/s and m/s². */
interface VehicleSpec {
  length: number;
  width: number;
  maxSpeed: number;
  maxAccel: number;
  comfyDecel: number;
  timeHeadway: number;
  minGap: number;
  /** From its rear bumper to its rear axle. */
  rearAxle: number;
  /** From its rear bumper to its front axle. */
  frontAxle: number;
  class: VehicleClass;
  /** Whether it runs on rails, its rear axle on its track. */
  rail: boolean;
}
declare const VehicleSpec: { readonly PASSENGER_CAR: VehicleSpec };

/** A driver's traits. */
interface Driver {
  /** How fast it likes to go, relative to the speed limit. */
  speedFactor: number;
  /** How much its acceleration wanders on the open road, in m/s². */
  accelNoise: number;
  /** How far short of a line it stops, on average, in meters. */
  lineGap: number;
  /** How far either way of `lineGap` its stops spread, in meters. */
  lineGapSpread: number;
  /** The hardest it brakes to let another in ahead, in m/s². */
  courtesy: number;
  /** How much it weighs others' gains against its own when changing lanes. */
  politeness: number;
  /** How readily it changes lanes, from 0 to 1. */
  assertiveness: number;
  /** Seeds its own random generator. */
  seed: number;
}
declare const Driver: { readonly DEFAULT: Driver };

/** A destination registered with the simulation. */
type DestinationId = number;

/** How a vehicle chooses its way at forks. */
type Route = { Random: { state: number } } | { Destination: DestinationId };

/** Where a vehicle is inserted. */
interface Placement {
  lane: LaneId;
  /** Its rear axle's distance along the lane, in meters. */
  s: number;
  /** In meters per second. */
  speed: number;
  /** Random choices from seed 0, if left out. */
  route?: Route;
}

/** A vehicle in the simulation. */
type VehicleId = { idx: number; version: number };

/** A running simulation, as `simulate` returns it. */
interface Simulation {
  /** Places a vehicle on a lane's centreline, with no checks. */
  insert(spec: VehicleSpec, driver: Driver, at: Placement): VehicleId;
}

/**
 * A simulation of `network`, compiled, with no vehicles yet: the last a script
 * makes is the scenario's. Throws if the network doesn't compile.
 */
declare function simulate(network: Network): Simulation;
