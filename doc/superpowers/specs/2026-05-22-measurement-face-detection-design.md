# Face Detection for Measurement — Design

Date: 2026-05-22
Status: Approved design, ready for implementation planning

## Summary

Extend F3D's measurement mode so a click in the interior of a triangle selects
the whole flat **face** it belongs to, using the face's centroid as the
measurable point. The detected face region is highlighted so the user sees
what was selected. This complements the existing point (vertex) and edge
picking.

Motivating example: on a filleted cube, picking the center of one face and the
center of an adjacent face — the user wants to click anywhere on each face and
have F3D figure out the face and its center point automatically.

## Goals

- Auto-resolve a click in a triangle's interior to a **face**: the connected
  set of coplanar triangles around the picked triangle.
- Use the face's area-weighted **centroid** as the measurable point.
- Highlight the detected face region (and in the hover preview) so the user
  sees what was detected.
- Interoperate with existing point/edge picking and all axis modes.

## Non-Goals (this version)

- Circle / hole detection (diameter, center) — deferred to a separate feature;
  it requires edge-loop tracing and circle fitting, which is substantially
  more involved.
- Detecting curved "faces" as single logical surfaces — region growing is
  coplanar only; a curved surface yields a small local patch.
- Any new UI, option, hotkey, or panel control.

## Behavior & UX

### Pick resolution — third case

Today a click auto-resolves by proximity: near a vertex → point, otherwise →
nearest edge. It becomes three cases:

- within the snap tolerance of a vertex → **point**
- else within the snap tolerance of one of the triangle's edges → **edge**
- otherwise (interior of the triangle) → **face**

The snap tolerance is the existing cell-size-scaled tolerance already used for
vertex snapping. Today edge is the unconditional fallback; now it is gated by
edge proximity, and a click that is near neither a vertex nor an edge resolves
to a face. This is a behavior change: a click in a triangle interior, which
currently returns the nearest edge, now returns a face.

### What a face is

Starting from the picked triangle, region-grow across edge-adjacent triangles
whose normal is within a small angle tolerance of the picked triangle's
normal. The tolerance is a fixed constant (~15°). Region growing naturally
stops at sharp edges, so:

- clicking a flat face of a cube selects that whole face;
- clicking a curved surface (fillet, cylinder side) grows only a small local
  patch and degrades gracefully.

### What it produces

The measurable object for a face is the **area-weighted centroid** of the
region — a point. Measurement uses it like any point: face↔face is
centroid-to-centroid, and face↔point, face↔edge, plus every axis mode, all
work with no special handling.

### Annotation

When a face is picked, the detected region is highlighted with a translucent
tint (drawn in the existing measurement overlay renderer) and a marker is
placed at the centroid. The hover preview shows the same region highlight +
centroid for the face under the cursor, exactly as it previews points and
edges today.

### No new UI

No panel changes, no new option, no hotkey. This is purely an extension of the
existing click/hover auto-resolution.

## Architecture

No new files. Each change extends an existing unit.

### `MeasureObject` (in `measurementTools.h`)

`MeasureObject::Type` gains a `FACE` value alongside `POINT` and `EDGE`.

- For `FACE`, `P0` holds the region centroid (so distance math can read `P0`
  uniformly for points and faces).
- A new member carries the region geometry for highlighting:
  `std::vector<std::array<double, 3>> FacePoints` — the region's triangle
  vertices, three consecutive entries per triangle. Empty for `POINT`/`EDGE`.

### `ResolvePickedObject` signature change

Region growing needs the whole mesh, not a single cell. The resolver changes
from:

```cpp
MeasureObject ResolvePickedObject(
  const std::array<double, 3>& worldPos, vtkCell* cell, double snapTol);
```

to:

```cpp
MeasureObject ResolvePickedObject(
  const std::array<double, 3>& worldPos, vtkPolyData* mesh, vtkIdType cellId,
  double snapTol);
```

- Point/edge resolution uses `mesh->GetCell(cellId)` exactly as before.
- Face resolution region-grows over `mesh`.
- If `mesh` is null (the picked dataset is not a `vtkPolyData`), face
  resolution falls back to the single picked triangle: its centroid and its
  three vertices as the region.

This ripples cleanly: the interactor already obtains the picked `vtkDataSet*`
and cell id from `vtkCellPicker`. `measurementManager::HandlePick` and
`HandleHover` change their parameters from `vtkCell* cell` to
`vtkPolyData* mesh, vtkIdType cellId`; the interactor's `OnLeftButtonPress` and
`OnMouseMove` pass the picker's dataset (cast to `vtkPolyData`) and cell id.
`SnapToleranceForCell` continues to take the resolved `vtkCell*` obtained
inside the resolver.

### Region-grow helper (new, in `measurementTools`)

A helper performs the breadth-first region grow:

- From the picked triangle, traverse edge-adjacent triangles via
  `vtkPolyData::GetCellEdgeNeighbors` (after `vtkPolyData::BuildLinks`).
- Include a neighbor triangle when the angle between its normal and the
  **seed** triangle's normal is below the tolerance (comparing against the
  fixed seed normal, not a running average, avoids drift across a gently
  curved patch).
- Collect the triangle set; compute the area-weighted centroid and gather the
  triangle vertices.
- The result populates a `FACE` `MeasureObject`.

Triangle normals and areas use `vtkTriangle`/`vtkMath`. `measurementTools.cxx`
already links the VTK modules these need.

### `ComputeDistance`

`FACE` is treated identically to `POINT` — both measure from `P0`. The
existing point/edge branches generalize so that "point-like" means
`POINT || FACE`. No new distance cases are introduced.

### `measurementManager`

`UpdateActors` and `UpdateHoverActor` gain face handling: for a `FACE`
object they draw the centroid marker (as today for a point) and additionally
a translucent highlight of the region, built as a `vtkPolyData` from the
object's `FacePoints` and added to the existing layer-1 overlay renderer with
`LightingOff()`.

## Error / edge-case handling

- Picked dataset not a `vtkPolyData` → face falls back to the single picked
  triangle (centroid of that triangle, region = that triangle).
- A triangle whose every neighbor exceeds the angle tolerance → region is just
  that triangle; centroid is the triangle centroid.
- Degenerate (zero-area) triangles are skipped in the area-weighted centroid;
  if the whole region has zero area, the plain vertex average is used.
- Reloading or changing the model clears any measurement (already handled by
  the existing clear-on-load path), so stale face geometry cannot persist.

## Testing

- `ResolvePickedObject`'s unit tests in `library/testing/TestSDKMeasurement.cxx`
  move from a single `vtkTriangle` to a small multi-triangle `vtkPolyData`:
  - point check: a pick near a shared vertex still resolves to that vertex;
  - edge check: a pick near an edge still resolves to that edge;
  - face check: a pick in a triangle interior resolves to a `FACE` whose
    centroid equals the known centroid of the coplanar region;
  - region-grow stop check: a `vtkPolyData` with a flat region adjoining a
    steeply angled triangle — the region must include the flat triangles and
    exclude the angled one.
- The existing `TestSDKMeasurementInteraction` continues to cover the click
  path; it is extended with a click in a triangle interior.

## Future Work

- Circle / hole detection — detect a circular edge loop, report its center
  (as a measurable point) and its diameter. Deferred; needs edge-loop tracing
  and circle fitting.
- Treating a curved surface as one logical face.
- A configurable coplanarity tolerance.
