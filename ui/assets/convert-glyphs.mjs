#!/usr/bin/env node
// One-time conversion of lane-glyphs.svg to triangles for direct rendering.
//
// Run from the repository root:
//
//     node ui/assets/convert-glyphs.mjs > ui/src/glyphs.rs && cargo fmt --all
//
// It understands only what lane-glyphs.svg uses: one <path> per glyph, its id
// the glyph's name, with a `translate` laying it out on the sheet and a `d` of
// absolute M, L, C and Z commands, in metres with the tail at the origin and the
// arrow pointing to -y. Each subpath is a separate outline. Curves are
// flattened, each outline is cut into triangles by ear clipping, and the result
// is written in the glyph's own frame: metres, origin at its anchor (the tail),
// +y the way the arrow points, +x to its right.

import { readFileSync } from "node:fs";

const SVG = "ui/assets/lane-glyphs.svg";
// Line segments per cubic Bézier.
const CURVE_STEPS = 8;
// Vertices are written to this many decimal places, a tenth of a millimetre,
// and rounded so before triangulating, so no triangle flips as it's written.
const DECIMALS = 4;
// Twice the area of the thinnest ear clipped, in square metres: a nearly flat
// one could still flip in f32.
const MIN_EAR = 1e-6;

// Points closer than this to the one before, in metres, are dropped, so a
// subpath closed a millimetre short of its start has no sliver edge.
const MERGE = 0.003;

/** A path's outlines, one per subpath, as lists of points, curves flattened. */
function parsePath(d) {
  const tokens = d.match(/[MLCZ]|-?\d*\.?\d+(?:e-?\d+)?/g);
  const outlines = [];
  let points = [];
  let i = 0;
  let cmd = null;
  const nums = (n) => tokens.slice(i, i + n).map(Number);
  const close = () => {
    const near = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]) < MERGE;
    const kept = points.filter((p, k) => k === 0 || !near(p, points[k - 1]));
    if (kept.length > 1 && near(kept[0], kept.at(-1))) kept.pop();
    if (kept.length >= 3) outlines.push(kept);
    points = [];
  };
  while (i < tokens.length) {
    if ("MLCZ".includes(tokens[i])) {
      cmd = tokens[i++];
      if (cmd === "Z") {
        close();
        continue;
      }
      if (cmd === "M" && points.length) close();
    }
    if (cmd === "M" || cmd === "L") {
      points.push(nums(2));
      i += 2;
    } else if (cmd === "C") {
      const [x1, y1, x2, y2, x, y] = nums(6);
      const [x0, y0] = points[points.length - 1];
      for (let k = 1; k <= CURVE_STEPS; k++) {
        const t = k / CURVE_STEPS;
        const u = 1 - t;
        const [a, b, c, e] = [u ** 3, 3 * u * u * t, 3 * u * t * t, t ** 3];
        points.push([a * x0 + b * x1 + c * x2 + e * x, a * y0 + b * y1 + c * y2 + e * y]);
      }
      i += 6;
    } else {
      throw new Error(`unsupported path command ${cmd}`);
    }
  }
  if (points.length) close();
  return outlines;
}

function area(poly) {
  let sum = 0;
  for (let i = 0; i < poly.length; i++) {
    const [p, q] = [poly.at(i - 1), poly[i]];
    sum += p[0] * q[1] - q[0] * p[1];
  }
  return 0.5 * sum;
}

const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);

const inside = (p, a, b, c) => cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0;

/** Ear clipping of a simple counterclockwise polygon, as index triples. */
function triangulate(poly) {
  const idx = poly.map((_, i) => i);
  const tris = [];
  while (idx.length > 3) {
    let found = false;
    for (let k = 0; k < idx.length; k++) {
      const [i, j, l] = [idx.at(k - 1), idx[k], idx[(k + 1) % idx.length]];
      const [a, b, c] = [poly[i], poly[j], poly[l]];
      if (cross(a, b, c) <= MIN_EAR) continue;
      if (idx.some((m) => m !== i && m !== j && m !== l && inside(poly[m], a, b, c))) continue;
      tris.push([i, j, l]);
      idx.splice(k, 1);
      found = true;
      break;
    }
    if (!found) throw new Error("no ear found: polygon not simple");
  }
  tris.push(idx);
  return tris;
}

const svg = readFileSync(SVG, "utf8");
const glyphs = [];
for (const [, name, d] of svg.matchAll(/<path id="(\w+)" transform="[^"]*" d="([^"]*)"/g)) {
  const vertices = [];
  const triangles = [];
  for (const outline of parsePath(d)) {
    // The path's `translate` only lays the glyphs out on the sheet, so it is
    // left out, and y is flipped to point the way the arrow does.
    const round = (x) => Number(x.toFixed(DECIMALS));
    const poly = outline.map(([x, y]) => [round(x), round(-y)]);
    if (area(poly) < 0) poly.reverse();
    const base = vertices.length;
    vertices.push(...poly);
    triangles.push(...triangulate(poly).map((t) => t.map((v) => base + v)));
  }
  glyphs.push({ name, vertices, triangles });
}

let out = "";
out += "//! Lane arrow glyphs, converted once from `ui/assets/lane-glyphs.svg` by\n";
out += "//! `ui/assets/convert-glyphs.mjs`: don't edit by hand.\n\n";
out += "// Coordinates, some of which look like well-known constants.\n";
out += "#![allow(clippy::approx_constant)]\n";
out += "// Every glyph on the sheet, whether or not the UI paints it yet.\n";
out += "#![allow(dead_code)]\n\n";
out += "/// A filled shape in its own frame, in meters: origin at its anchor, +y the\n";
out += "/// way the arrow points, +x to its right.\n";
out += "pub struct Glyph {\n    pub vertices: &'static [[f32; 2]],\n";
out += "    pub triangles: &'static [[u16; 3]],\n}\n";
for (const { name, vertices, triangles } of glyphs) {
  out += `\npub const ${name.toUpperCase()}: Glyph = Glyph {\n    vertices: &[\n`;
  for (const [x, y] of vertices) out += `        [${x.toFixed(DECIMALS)}, ${y.toFixed(DECIMALS)}],\n`;
  out += "    ],\n    triangles: &[\n";
  for (const [a, b, c] of triangles) out += `        [${a}, ${b}, ${c}],\n`;
  out += "    ],\n};\n";
}
process.stdout.write(out);
