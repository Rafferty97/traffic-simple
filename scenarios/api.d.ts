// The scenario API, as `ui/src/script.rs` binds it, for editors: every script
// here, in TypeScript or JavaScript, is checked against it (see
// `tsconfig.json`). Nothing reads it at runtime: the UI strips a script's types
// and runs what is left. Data takes its Rust types' serde forms, their fields in camelCase.
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

/** A lane's profile across its road, in meters, the same all along it. */
interface Profile {
  /** From the road's path to the lane's centreline, positive to the path's left. */
  offset: number;
  /** From the centreline to the lane's left edge, as the lane runs. */
  left: number;
  /** From the centreline to the lane's right edge, as the lane runs. */
  right: number;
}

/**
 * A lane's profile across its road at `s` meters along the road's path:
 * `offset`, `left` and `right` as `Profile` has them.
 */
type Station = [s: number, offset: number, left: number, right: number];

/**
 * A lane of a road: its profile, the same all along, or stations in order
 * along the road, or a dropped lane's stations. Between each two stations its
 * offset and widths ease (smoothstep) from one's to the other's; before the
 * first and after the last they are those stations'. Every lane runs its
 * road's whole length: a change in how many lanes there are is another road.
 */
type RoadLane = Profile | Station[] | DroppedLane;

/**
 * A lane dropped where it eases in over another lane of its road, rather than
 * the two zipping together: its taper costs routing more, so drivers bound for
 * a destination change out of it before it, where they can, and the lane going
 * on is marked beside it with a short dashed line.
 */
interface DroppedLane {
  stations: Station[];
  dropped: boolean;
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

/**
 * A road of a network. Its lanes are numbered forward lanes first, in the order
 * given, then reverse ones; where they are in the simulation, `Compiled` says.
 */
interface RoadId {
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
  /**
   * Declares that `a` and `b` overlap where their lanes' outlines do, on the
   * same ground, so vehicles on them keep apart there. Otherwise roads are
   * unrelated where they overlap, as at different grades, unless they join a
   * road in common, as two roads merging into one or forking from one do; for
   * those, declaring it changes nothing.
   */
  overlap(a: RoadId, b: RoadId): void;
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

/** A running simulation, as `Compiled.simulate` returns it. */
interface Simulation {
  /** Places a vehicle on a lane's centreline, with no checks. */
  insert(spec: VehicleSpec, driver: Driver, at: Placement): VehicleId;
  /**
   * Registers `lanes` as a destination, for routes to it (`{ Destination: id }`
   * in a placement). Throws if a lane isn't the network's.
   */
  addDestination(lanes: LaneId[]): DestinationId;
}

/**
 * A network compiled for the simulation. Each road is cut wherever one of its
 * lanes starts or stops overlapping a lane of a road related to it, so a road's
 * lane may be several of the simulation's, one after the other.
 */
interface Compiled {
  /** A simulation of it, with no vehicles yet: the last a script makes is the scenario's. */
  simulate(): Simulation;
  /**
   * Where a vehicle goes to be `s` meters along `road`'s reference line, from
   * its start, on its `lane`th lane, at `speed`: reverse lanes are measured
   * from the road's start too. Throws if there is no such lane or `s` isn't
   * along the road.
   */
  place(road: RoadId, lane: number, s: number, speed: number): Placement;
  /** The simulation's lanes that are `road`'s `lane`th, in the order it runs along them. */
  lanes(road: RoadId, lane: number): LaneId[];
}

/** `network` compiled. Throws if it doesn't compile. */
declare function compile(network: Network): Compiled;
