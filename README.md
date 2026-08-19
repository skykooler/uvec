# uvec

A minimal vector-graphics data model: a planar arrangement of cubic bezier
curves with face-based fills. See `PARAMETERS` for the original specification.

## Model

- **Scene** owns vertices, curves, and fills. Curves are cubic beziers whose
  endpoints are shared `Vertex` objects; control points are stored per curve.
- Curves never cross: intersections (including self-intersections) are
  resolved by splitting curves at the crossing and creating a shared vertex.
  Vertices closer than `snap_distance` merge. Because of this the scene is
  always a planar arrangement, and a **Fill is a face** of that arrangement:
  one outer loop of (curve, direction) half-edges plus zero or more hole
  loops.
- Every fill remembers the point it was created at (its **anchor**). After
  each mutating operation the fill's loops are re-derived as the face
  containing the anchor. If a new curve bisects a filled region, the fill
  keeps the chamber holding its anchor; if the anchor's face becomes
  unbounded, the fill is deleted.

## Operations

| Op | Method |
|---|---|
| 1. Add curve | `add_curve([Point; 4]) -> Vec<CurveId>` (surviving pieces) |
| 2. Add fill at point | `add_fill(Point, Rgba8) -> Result<FillId, FillError>` (recolors if the point is already inside a fill; `NotEnclosed` if the point's face is unbounded) |
| 3. Delete curve | `delete_curve(CurveId)` (deletes fills referencing it) |
| 4. Delete fill | `delete_fill(FillId)` |
| 5. Replace curve | `replace_curve(CurveId, [Point; 4]) -> Result<Vec<CurveId>, BadId>` |
| 6. Move vertex | `move_vertex(VertexId, Point)` |

Queries: `curve_ids`, `vertex_ids`, `fill_ids`, `curve_points`,
`curve_polyline`, `vertex_pos`, `fill_color`, `fill_loops_flattened`.

## Rendered geometry

`curve_polyline` is the single source of truth for a curve's geometry: the
bezier flattened at the scene tolerance, with the portion inside a
snap-radius disk around each endpoint replaced by a straight stub. Straight
stubs radiating from one vertex cannot cross each other, which keeps the
polyline arrangement crossing-free near junctions — the property every
topology query (point location, face walking, even-odd tests) relies on.
Renderers must draw these polylines (and fill loops via
`fill_loops_flattened`, even-odd rule) to be pixel-consistent with the
model's decisions.

Intersection detection runs on these exact polylines (segment/segment, with
parameter tags mapped back to bezier parameters for splitting). Polylines
passing within `snap_distance / 4` of each other are welded as if they
crossed, so no impassably-narrow corridors survive. Near-coincident duplicate
edges (same endpoints, polylines within that tolerance) are deduplicated.

## Testing

`tests/fuzz.rs` draws random curves, applies random operations (fills kept
transparent), then adds one visible fill at a random enclosed point via the
library while performing a raster flood fill from the same point on a canvas
where all curves were stroked with supercover (grid-traversal) lines. The
re-rendered scene (scanline even-odd fill + strokes) must be pixel-identical
to the flooded canvas. If the flood escapes to the border, the library must
report `NotEnclosed`.

One tolerated artifact class: pixels whose center is inside the face but
which the flood cannot reach because strokes pinch the region shut at the
test resolution (wedge tips). A diff is accepted only if every differing
pixel is fill-side-only *and* a flood at 2x or 4x resolution reaches it —
a genuinely mis-filled face stays sealed at every scale.

```sh
cargo test                     # unit tests + repros + 100-iteration smoke
cargo test --test fuzz fuzz_10k -- --ignored   # the 10k-iteration gate
```
