# Axis-Constrained Measurement — Design

Date: 2026-05-21
Status: Approved design, ready for implementation planning

## Summary

Extend F3D's measurement mode so a measurement can be constrained to a single
model axis (X, Y, or Z) instead of always reporting the straight-line distance
between the two picked objects. The overlay panel always shows the full
per-axis breakdown (ΔX, ΔY, ΔZ) plus the straight-line distance, and an axis
selector controls which value is primary and how the 3D annotation is drawn.

Motivating example: on a filleted cube, picking the center of one face and the
center of an adjacent face, the user wants the horizontal (single-axis)
distance, not the diagonal straight-line distance.

## Goals

- Add an axis mode to measurement: `Free` (straight line, current behavior),
  `X`, `Y`, or `Z`.
- Always display the per-axis breakdown (ΔX, ΔY, ΔZ) and the straight-line
  distance in the panel, so no mode switching is needed just to read a value.
- Let the user select the axis mode from the panel; the selection controls the
  primary reported value and the 3D annotation.
- Draw a right-angle "staircase" path for the selected axis, emphasizing the
  selected leg.

## Non-Goals (this version)

- Object detection (circles, faces, auto-centers) — that is a separate
  follow-up feature.
- Up-relative or view-relative axes — axes are model-space X/Y/Z only.
- A keyboard hotkey to cycle the axis — the panel selector is sufficient.
- Constraining to an arbitrary direction or to an edge's direction.

## Behavior & UX

### Axis modes

A measurement has an axis mode stored in a new option `ui.measurement.axis`,
one of `free` (default), `x`, `y`, `z`. It persists for the session and is
settable via CLI and config file, consistent with the unit options.

### Panel layout

The measurement panel gains an "Axis" section (a radio row) above the existing
"Units" section, and always shows the per-axis breakdown:

```
Distance: 42.300 mm            <- primary line, reflects the selected mode
ΔX 30.000   ΔY 28.000   ΔZ 5.000 mm
── Axis ──
( ) Free   (•) X   ( ) Y   ( ) Z
── Units ──
Model   [mm ▾]    Display  [mm ▾]
```

- The **primary line** reflects the selected mode: `free` → `Distance:
  <straight-line>`; `x` → `ΔX: <|ΔX|>`; `y`/`z` likewise.
- The **breakdown line** (`ΔX … ΔY … ΔZ …`) is always shown once two objects
  are selected, and is empty/absent while still selecting.
- The **radio row** selects the axis mode.

### 3D annotation

- `free` — the straight line between the two points (current behavior).
- `x` / `y` / `z` — a right-angle staircase path between the two closest
  points: the selected-axis leg is drawn first, starting at point A, and is
  emphasized (bright, thick); the other two legs are drawn dim. The path still
  connects A→B, showing how the straight-line distance decomposes.

All components are computed in model space and converted to display units with
the same model→display factor already used for the straight-line distance.

This applies to all measurement types — point↔point, point↔edge, edge↔edge —
decomposing the segment between the two closest points (`ClosestA`,
`ClosestB`).

## Architecture

No new files; each change extends a unit that already owns that responsibility.

### New option

`ui.measurement.axis` added to `library/options.json` (string, default
`free`). Accompanied by:

- a CLI flag `--measurement-axis` in `resources/cli-options.json`,
- a `LibOptionsNames` entry in `application/F3DOptionsTools.h`,
- a `doc/user/03-OPTIONS.md` entry.

Accepted values: `free`, `x`, `y`, `z`. Any other value is treated as `free`.

### Pure geometry helper (`measurementTools`)

A new pure, unit-testable function:

```cpp
/**
 * Return the 4 points of the right-angle "staircase" path from a to b for the
 * given axis (0=X, 1=Y, 2=Z). Point [0] is a, point [3] is b, and the leg
 * [0]->[1] is the selected-axis leg (drawn first from a). The remaining two
 * legs follow in ascending axis order.
 */
std::array<std::array<double, 3>, 4> ComputeAxisPath(
  const std::array<double, 3>& a, const std::array<double, 3>& b, int axis);
```

Per-axis component magnitudes are plain `std::fabs(b[i] - a[i])` subtractions;
no dedicated helper is required.

### `measurementManager`

- `GetResultString()` — the primary line now reflects `ui.measurement.axis`:
  `free` → `Distance: <straight>`, `x` → `ΔX: <|Δx|>`, etc. Values are
  converted to display units.
- new `GetComponentsString()` — returns the `ΔX … ΔY … ΔZ …` breakdown string
  (converted to display units), or an empty string when fewer than two objects
  are selected.
- `UpdateActors()` — when a result exists: `free` draws the straight line
  between `ClosestA`/`ClosestB` as today; `x`/`y`/`z` draws the staircase from
  `ComputeAxisPath`, with the selected leg emphasized (bright, thicker) and the
  other two legs dim. Exact emphasis/dim colors are chosen during
  implementation to stay distinct from the yellow selection markers and the
  cyan hover preview.

### Rendering path

`vtkF3DRenderer::ConfigureMeasurement(...)` is extended to also carry the
components-breakdown string and the current axis string. `vtkF3DUIActor` gains
matching members (`MeasurementComponents`, `MeasurementAxis`) with setters.
`vtkF3DImguiActor::RenderMeasurement()` renders the breakdown line and a
`Free / X / Y / Z` radio row (using `ImGui::RadioButton`) above the Units
section.

### Selector command

A new interactor command `set_measurement_axis`, a sibling of
`set_measurement_unit`:

```
set_measurement_axis <free|x|y|z>
```

It sets `ui.measurement.axis` via `options.setAsString`, calls
`measurementManager::RefreshPanel()`, and requests a render. The radio buttons
emit this single command (one command per change — avoiding the command-buffer
overwrite issue, the same pattern used by the unit combos).

## Error / edge-case handling

- Axis option set to an unrecognized value → treated as `free`.
- Breakdown/components requested with fewer than two objects selected →
  empty string; the panel shows only the selection prompt.
- Selected-axis component of zero length (the two points share that
  coordinate) → the emphasized leg has zero length; the staircase still draws
  the remaining legs and the reported value is `0`.
- `free` mode is unchanged from current behavior in every respect.

## Testing

- Unit tests for `ComputeAxisPath` in `library/testing/TestSDKMeasurement.cxx`:
  for each axis, verify point `[0]==a`, point `[3]==b`, the leg `[0]->[1]`
  lies along the selected axis with the correct length, and the three legs
  sum from `a` to `b`.
- An option round-trip check for `ui.measurement.axis`.
- The existing `TestSDKMeasurementInteraction` continues to cover the
  rendering path; it is extended to also trigger `set_measurement_axis`.

## Future Work

- Object detection — circle/hole detection (outer diameter, center point),
  face picking with an auto-computed centroid "point". Separate feature,
  planned next.
- Up-relative axes (a true "horizontal"/"vertical" basis derived from the
  scene up direction).
- A keyboard hotkey to cycle the axis mode.
