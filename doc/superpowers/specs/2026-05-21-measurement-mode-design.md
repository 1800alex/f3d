# Measurement Mode — Design

Date: 2026-05-21
Status: Approved design, ready for implementation planning

## Summary

Add a measurement mode to F3D that lets users measure the distance between two
objects in a loaded model. Users toggle the mode with <kbd>Shift</kbd>+<kbd>M</kbd>,
click two objects (points or edges), and see the distance rendered both as a 3D
line in the scene and in a bottom-right overlay panel. The overlay panel exposes
combo boxes for selecting model and display units, with preset unit conversions.

## Goals

- Toggle measurement mode on/off with <kbd>Shift</kbd>+<kbd>M</kbd>, listed in the help cheatsheet.
- Select two objects — mesh **vertices (points)** or **mesh edges** — by clicking.
- Compute and display the distance between the two selected objects.
- Display the result as a 3D connecting line plus a bottom-right overlay panel.
- Let users choose model units and display units from a preset list, with conversion.
- <kbd>Esc</kbd> clears the current selection and measurement.

## Non-Goals (this version)

- Selecting analytic primitives such as circles or arcs (F3D renders triangle
  meshes only — there is no native circle concept).
- Feature-edge detection (CAD-style edges). See Future Work.
- Angle measurement, single-edge length readout, coordinate readout.
- Accumulating multiple simultaneous measurements — only one exists at a time.
- Free-form custom unit entry — units come from a fixed preset list.

## Approach

A dedicated `measurementManager` class owns all measurement state, picking
resolution, unit conversion, and the 3D annotation actors. This mirrors the
existing `animationManager` pattern and keeps `interactor_impl.cxx` (already
~2200 lines) from growing further. The interactor delegates clicks and keys to
the manager. Rendering of the overlay panel reuses F3D's Dear ImGui UI layer.

### Considered alternatives

- **Everything inside `interactor_impl.cxx`** — rejected: bloats an already large
  file, hard to test.
- **New VTK class in `vtkext/`** — rejected: heavier than needed; standard
  `vtkActor`/`vtkTextActor` plus an ImGui panel are sufficient.

## Section 1: Units & the Measurement Panel

### Options

Two new options under `ui.measurement` in `library/options.json`:

- `ui.measurement.model_unit` (string, default `""`) — the unit the raw model
  geometry coordinates represent.
- `ui.measurement.display_unit` (string, default `""`) — the unit measurement
  results are displayed in.

Accepted values: `mm`, `cm`, `m`, `in`, `ft`, or empty (treated as `unitless`).
Settable via CLI (`--ui-measurement-model-unit=mm`), config file, or the F3D
console at runtime (`set ui.measurement.display_unit in`).

### Conversion

A conversion table maps each unit to a base (meters):
`mm=0.001, cm=0.01, m=1.0, in=0.0254, ft=0.3048`.

Displayed distance = `raw × (model_factor / display_factor)`.

If either unit is `unitless`/empty, no conversion is applied and the raw value
is shown with no unit suffix.

### Overlay panel

The measurement overlay is an ImGui panel anchored to the **bottom-right** of the
viewport. It contains:

- The current measurement result (e.g. `Distance: 42.3 mm`) or a prompt
  (`Select first object…` / `Select second object…`).
- A **Model units** combo box.
- A **Display units** combo box.

Both combos list the preset units. Changing a combo writes the corresponding
`ui.measurement.*` option live; because options are session state, the choice
persists for the session. CLI/config can pre-seed the initial values.

The panel is visible whenever measurement mode is active **or** a completed
measurement exists. It hides only when mode is off and no measurement exists.

A new virtual `RenderMeasurement()` method is added to the UI actor base class
`vtkF3DUIActor` alongside `RenderMetaData()` etc., and implemented in
`vtkF3DImguiActor`.

## Section 2: Interaction & Picking

### Entering/leaving the mode

A new command `toggle_measurement` is bound to <kbd>Shift</kbd>+<kbd>M</kbd> via
`addBinding`. The binding carries a documentation callback so it appears in the
<kbd>H</kbd> cheatsheet. Toggling flips a `MeasurementActive` flag.

### Click handling

The interactor currently observes only `MiddleButtonPressEvent`. A new
`LeftButtonPressEvent` observer is added:

- Measurement mode **off** → callback does nothing; the normal camera-rotation
  interactor style handles the click.
- Measurement mode **on** → pick at the cursor, consume the click (no camera
  rotation), forward the hit to `measurementManager`.

### Resolving point vs. edge

Using the existing `vtkCellPicker`/`vtkPointPicker`:

- The picked cell plus the exact pick position identifies the triangle.
- If the pick position is within a snap tolerance of a triangle **vertex**,
  select that vertex as a **point**.
- Otherwise snap to the nearest of the triangle's three **edges**, stored as its
  two endpoint coordinates.
- A click on empty space (no hit) is ignored.

This makes a single click type-aware — there is no separate sub-mode for points
versus edges.

### Two-step flow

1. First valid click → "first object" stored, marker drawn.
2. Second valid click → "second object" stored, distance computed and rendered.
3. Third valid click → discards the previous measurement and starts a fresh one
   ("one at a time, replace").

<kbd>Esc</kbd> while in measurement mode clears the current selection and
measurement.

### Distance semantics

- point ↔ point — straight-line distance.
- point ↔ edge — perpendicular distance from the point to the finite segment.
- edge ↔ edge — minimum distance between the two finite segments.

The 3D connecting line is drawn between the two closest points.

### Mesh-edge caveat

On a dense triangulated mesh (e.g. an STL of a CAD part), selectable "edges" are
**triangulation edges**, not CAD feature edges. Edge selection therefore picks
individual triangle edges. This is inherent to mesh data; feature-edge detection
is out of scope for this version (see Future Work).

## Section 3: `measurementManager` & Rendering

### New files

- `library/src/measurementManager.h`
- `library/src/measurementManager.cxx`

Registered in `library/CMakeLists.txt`. The manager is owned by the interactor
and constructed alongside it (mirroring `animationManager`).

### State

- `MeasurementActive` (bool) — mode on/off.
- `Selection` — 0, 1, or 2 picked objects. Each object is a tagged struct
  `{ Type: Point|Edge, p0, p1 }`; a `Point` uses `p0` only.
- Cached result — computed distance and the two closest points.

### Public interface

- `ToggleMeasurement()` / `IsActive()`
- `HandlePick(worldPos, vtkCell* cell)` — runs point-vs-edge resolution and
  advances the two-step flow.
- `Clear()` — used by <kbd>Esc</kbd>.
- Getters for the ImGui panel — `GetResultString()`, current unit state.

### 3D rendering

The manager builds VTK actors and adds them to the renderer:

- A small **sphere** marker per selected point.
- A short highlighted **tube/line** per selected edge.
- A **connecting line** between the two closest points once both objects are
  selected.

The manager shows/hides/updates this dedicated set of actors. No custom VTK
subclass is required.

### Overlay panel data flow

The `measurementManager` exposes the result string and unit state.
`vtkF3DRenderer` passes them to `vtkF3DImguiActor::RenderMeasurement()`, which
draws the bottom-right panel with the two combo boxes. Combo callbacks write the
`ui.measurement.*` options.

### Unit conversion

A small free function plus the unit→meters table lives in the manager. It is
used both for the displayed result and is independently unit-testable.

## Section 4: Help, Docs, Testing & Error Handling

### Cheatsheet / help menu

The `addBinding` call for `toggle_measurement` passes a documentation callback
(the `docTgl` pattern), so <kbd>Shift</kbd>+<kbd>M</kbd> appears automatically in
the <kbd>H</kbd> cheatsheet under the "Scene" group with its on/off state.

### Documentation updates

- `doc/user/04-INTERACTIONS.md` — add <kbd>Shift</kbd>+<kbd>M</kbd> to the
  bindings list and a "Measurement Mode" section describing the workflow.
- `doc/user/03-OPTIONS.md` — document `ui.measurement.model_unit` and
  `ui.measurement.display_unit`.
- `doc/user/CHANGELOG.md` — feature entry.

### Error / edge-case handling

- Click on empty space → ignored, no state change.
- No model loaded → mode toggles but all picks miss; harmless.
- Animation playing / 2D interaction mode → measurement still works; picking is
  camera-independent, no special handling.
- Reloading a file or loading a new one → clears any active selection and
  measurement (stale geometry).
- `unitless`/empty for either combo → raw value shown, no suffix, no conversion.

### Testing

- Unit tests for the conversion table and the three distance computations
  (point-point, point-edge, edge-edge) — pure functions, no rendering.
- An interaction test using F3D's `TestInteraction…` baseline-image pattern,
  simulating two clicks and <kbd>Esc</kbd>, verifying the panel and 3D line render.
- An option round-trip test for the two new options.

## Future Work

- **Feature-edge detection** — detect CAD-style feature edges (sharp edges,
  boundary edges) so users can select meaningful model edges instead of raw
  triangulation edges. This is a desired follow-up once measurement mode ships.
- **Circle / arc / hole detection** — detect circular edge loops and cylindrical
  holes to measure diameters and centers.
- **Additional measurement types** — angle between edges, single-edge length,
  point coordinate readout.
- **Multiple simultaneous measurements** — accumulate and manage several
  measurements at once.
