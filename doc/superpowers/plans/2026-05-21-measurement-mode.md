# Measurement Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a measurement mode to F3D that lets users measure the distance between two picked objects (mesh points or edges), with unit conversion and a 3D + overlay display.

**Architecture:** A new `f3d::detail::measurementManager` class owns measurement state, pick resolution, distance math, unit conversion, and 3D annotation actors. It is a by-value member of `interactor_impl::internals`. The interactor routes left-clicks and Escape to it when measurement mode is active. The overlay panel is rendered through F3D's existing Dear ImGui UI layer via a new `RenderMeasurement()` virtual on `vtkF3DUIActor`.

**Tech Stack:** C++17, VTK (vtkCellPicker, vtkPointPicker, vtkActor, vtkLineSource, vtkSphereSource), Dear ImGui, CMake, F3D options codegen (`library/options.json`).

**Spec:** `doc/superpowers/specs/2026-05-21-measurement-mode-design.md`

---

## File Structure

**New files:**
- `library/private/measurementManager.h` — class declaration + pure helper declarations (`UnitToMeters`, `ComputeDistance`, `ResolvePickedObject`, `MeasureObject`, `MeasureResult`).
- `library/src/measurementManager.cxx` — implementation.
- `library/testing/TestSDKMeasurement.cxx` — unit tests for the pure helpers.

**Modified files:**
- `library/options.json` — new `ui.measurement.model_unit` / `ui.measurement.display_unit` options.
- `library/CMakeLists.txt` — add `measurementManager.cxx` to the `libf3d` sources.
- `library/testing/CMakeLists.txt` — register `TestSDKMeasurement` (no-render list).
- `library/private/interactor_impl.h` — no change expected (manager lives in the `.cxx` `internals` struct).
- `library/src/interactor_impl.cxx` — own the manager, add the `toggle_measurement` command, the `Shift+M` binding, the `LeftButtonPress` observer, and Escape interception.
- `vtkext/private/module/vtkF3DUIActor.h` / `.cxx` — new `RenderMeasurement()` virtual + visibility/string setters.
- `vtkext/private/module/vtkF3DImguiActor.h` / `.cxx` — concrete `RenderMeasurement()` implementation with unit combos.
- `vtkext/private/module/vtkF3DRenderer.h` / `.cxx` — pass measurement data to the UI actor.
- `doc/user/04-INTERACTIONS.md`, `doc/user/03-OPTIONS.md`, `doc/user/CHANGELOG.md` — documentation.

---

## Task 1: Add measurement options

**Files:**
- Modify: `library/options.json`
- Modify: `doc/user/03-OPTIONS.md`

- [ ] **Step 1: Add the options to `library/options.json`**

Find the `"ui"` object (starts at line 181, `"ui": {`). Immediately after the `"axis"` block (the block `"axis": { "type": "bool", "default_value": "false" },`), insert:

```json
    "measurement": {
      "model_unit": {
        "type": "string",
        "default_value": ""
      },
      "display_unit": {
        "type": "string",
        "default_value": ""
      }
    },
```

This generates the accessors `options.ui.measurement.model_unit` and `options.ui.measurement.display_unit` (both `std::string`) in the generated `options.h`.

- [ ] **Step 2: Document the options in `doc/user/03-OPTIONS.md`**

Find the table/list of `ui.*` options in `doc/user/03-OPTIONS.md`. Add two rows in the same format as the surrounding `ui.*` entries:

```
ui.measurement.model_unit|string|-|Unit the model geometry coordinates are expressed in, used by measurement mode. One of `mm`, `cm`, `m`, `in`, `ft`, or empty for unitless.
ui.measurement.display_unit|string|-|Unit measurement results are displayed in. Same accepted values as `ui.measurement.model_unit`.
```

Match the exact column format already used in that file (inspect a neighbouring `ui.` row first and copy its delimiter style).

- [ ] **Step 3: Build to verify codegen succeeds**

Run: `cmake --build build --target libf3d 2>&1 | tail -20`
Expected: build succeeds; no errors about `options.json` parsing or `ui.measurement`.

- [ ] **Step 4: Commit**

```bash
git add library/options.json doc/user/03-OPTIONS.md
git commit -m "feat: add ui.measurement.model_unit and display_unit options"
```

---

## Task 2: Create `measurementManager.h` with the public interface and helper declarations

This task only declares types — no implementation, no test yet. It locks the interface every later task depends on.

**Files:**
- Create: `library/private/measurementManager.h`

- [ ] **Step 1: Write `library/private/measurementManager.h`**

```cpp
/**
 * @class   measurementManager
 * @brief   A private class managing measurement mode
 */

#ifndef f3d_measurementManager_h
#define f3d_measurementManager_h

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <vtkSmartPointer.h>

class vtkActor;
class vtkCell;

namespace f3d
{
class options;

namespace detail
{
class interactor_impl;
class window_impl;

/**
 * A selectable object: either a single point or a finite edge segment.
 * For POINT, only P0 is meaningful.
 */
struct MeasureObject
{
  enum class Type : std::uint8_t
  {
    POINT,
    EDGE
  };
  Type ObjType;
  std::array<double, 3> P0;
  std::array<double, 3> P1;
};

/**
 * Result of a distance computation: the distance plus the two closest points
 * (used to draw the connecting line).
 */
struct MeasureResult
{
  double Distance;
  std::array<double, 3> ClosestA;
  std::array<double, 3> ClosestB;
};

/**
 * Return meters-per-unit for a unit name (mm, cm, m, in, ft).
 * Returns std::nullopt for an empty string or an unknown unit.
 */
std::optional<double> UnitToMeters(const std::string& unit);

/**
 * Compute the minimum distance between two MeasureObjects and the two closest
 * points realizing that distance.
 * point-point: straight line. point-edge: perpendicular to the finite segment.
 * edge-edge: minimum distance between the two finite segments.
 */
MeasureResult ComputeDistance(const MeasureObject& a, const MeasureObject& b);

/**
 * Resolve a pick into a MeasureObject. worldPos is the picked world position,
 * cell is the picked triangle (must not be null). If worldPos is within snapTol
 * of one of the cell's points, returns a POINT at that vertex; otherwise returns
 * the EDGE of the cell nearest to worldPos.
 */
MeasureObject ResolvePickedObject(
  const std::array<double, 3>& worldPos, vtkCell* cell, double snapTol);

class measurementManager
{
public:
  measurementManager(options& options, window_impl& window);
  ~measurementManager();

  /**
   * Set the interactor, used to request renders after state changes.
   */
  void SetInteractor(interactor_impl* interactor);

  /**
   * Toggle measurement mode on/off. Turning it off clears the selection.
   */
  void ToggleMeasurement();

  /**
   * Whether measurement mode is currently active.
   */
  bool IsActive() const
  {
    return this->Active;
  }

  /**
   * Resolve a pick (world position + picked cell) into a point or edge and
   * advance the two-step selection. A null cell is ignored. The third pick
   * after a completed measurement discards it and starts fresh.
   */
  void HandlePick(const std::array<double, 3>& worldPos, vtkCell* cell);

  /**
   * Clear the current selection, measurement and 3D actors.
   */
  void Clear();

  /**
   * Whether at least one object is currently selected.
   */
  bool HasSelection() const
  {
    return !this->Selection.empty();
  }

  /**
   * The panel should be rendered whenever mode is active or a measurement exists.
   */
  bool IsPanelVisible() const
  {
    return this->Active || this->Result.has_value();
  }

  /**
   * Text shown in the overlay panel: a prompt while selecting, or the
   * formatted distance (converted to display units) once complete.
   */
  std::string GetResultString() const;

  measurementManager(const measurementManager&) = delete;
  void operator=(const measurementManager&) = delete;

private:
  /**
   * Rebuild the 3D annotation actors (markers + connecting line) from the
   * current selection/result and add them to the renderer.
   */
  void UpdateActors();

  /**
   * Remove all 3D annotation actors from the renderer.
   */
  void RemoveActors();

  options& Options;
  window_impl& Window;
  interactor_impl* Interactor = nullptr;

  bool Active = false;
  std::vector<MeasureObject> Selection;
  std::optional<MeasureResult> Result;

  std::vector<vtkSmartPointer<vtkActor>> Actors;
};
}
}
#endif
```

- [ ] **Step 2: Commit**

```bash
git add library/private/measurementManager.h
git commit -m "feat: add measurementManager interface declaration"
```

---

## Task 3: Implement and test unit conversion (`UnitToMeters`)

**Files:**
- Create: `library/src/measurementManager.cxx`
- Create: `library/testing/TestSDKMeasurement.cxx`
- Modify: `library/CMakeLists.txt`
- Modify: `library/testing/CMakeLists.txt`

- [ ] **Step 1: Add the source file to `library/CMakeLists.txt`**

In `library/CMakeLists.txt`, in the source list that currently contains `${CMAKE_CURRENT_SOURCE_DIR}/src/animationManager.cxx` (line ~48), add immediately after the `interactor_impl.cxx` line:

```cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/src/measurementManager.cxx
```

(Keep the list alphabetical if it is — place it after `levenshtein.cxx`.)

- [ ] **Step 2: Register the test in `library/testing/CMakeLists.txt`**

In `library/testing/CMakeLists.txt`, add `TestSDKMeasurement.cxx` to the `libf3dSDKTests_list` (the first `list(APPEND ...)` block, alphabetically near `TestSDKMultiColoring.cxx`):

```cmake
     TestSDKMeasurement.cxx
```

Then add `TestSDKMeasurement` to the `libf3dSDKTestsNoRender_list` block so it runs without a GPU:

```cmake
list(APPEND libf3dSDKTestsNoRender_list
     TestSDKEngineExceptions
     TestSDKLog
     TestSDKMeasurement
     TestSDKOptions
     TestSDKOptionsIO
     TestSDKScene)
```

- [ ] **Step 3: Write the failing test `library/testing/TestSDKMeasurement.cxx`**

```cpp
#include "PseudoUnitTest.h"

#include "measurementManager.h"

int TestSDKMeasurement([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
{
  PseudoUnitTest test;

  // --- UnitToMeters ---
  test("UnitToMeters mm", f3d::detail::UnitToMeters("mm").value() == approx(0.001));
  test("UnitToMeters cm", f3d::detail::UnitToMeters("cm").value() == approx(0.01));
  test("UnitToMeters m", f3d::detail::UnitToMeters("m").value() == approx(1.0));
  test("UnitToMeters in", f3d::detail::UnitToMeters("in").value() == approx(0.0254));
  test("UnitToMeters ft", f3d::detail::UnitToMeters("ft").value() == approx(0.3048));
  test("UnitToMeters empty", !f3d::detail::UnitToMeters("").has_value());
  test("UnitToMeters unknown", !f3d::detail::UnitToMeters("furlong").has_value());

  return test.result();
}
```

- [ ] **Step 4: Write `library/src/measurementManager.cxx` with only `UnitToMeters` implemented**

```cpp
#include "measurementManager.h"

#include <map>

namespace f3d::detail
{
//----------------------------------------------------------------------------
std::optional<double> UnitToMeters(const std::string& unit)
{
  static const std::map<std::string, double> table = {
    { "mm", 0.001 },
    { "cm", 0.01 },
    { "m", 1.0 },
    { "in", 0.0254 },
    { "ft", 0.3048 },
  };
  const auto it = table.find(unit);
  if (it == table.end())
  {
    return std::nullopt;
  }
  return it->second;
}
}
```

- [ ] **Step 5: Build and run the test, verify it passes**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -20
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: build succeeds, test `TestSDKMeasurement` passes all 7 `UnitToMeters` checks.

- [ ] **Step 6: Commit**

```bash
git add library/src/measurementManager.cxx library/testing/TestSDKMeasurement.cxx library/CMakeLists.txt library/testing/CMakeLists.txt
git commit -m "feat: implement unit conversion for measurement mode"
```

---

## Task 4: Implement and test distance computation (`ComputeDistance`)

**Files:**
- Modify: `library/src/measurementManager.cxx`
- Modify: `library/testing/TestSDKMeasurement.cxx`

- [ ] **Step 1: Add failing distance tests to `TestSDKMeasurement.cxx`**

Insert before `return test.result();`:

```cpp
  using f3d::detail::MeasureObject;
  using Type = f3d::detail::MeasureObject::Type;

  // point-point: straight line distance
  {
    MeasureObject a{ Type::POINT, { 0, 0, 0 }, {} };
    MeasureObject b{ Type::POINT, { 3, 4, 0 }, {} };
    const auto r = f3d::detail::ComputeDistance(a, b);
    test("point-point distance", r.Distance == approx(5.0));
  }

  // point-edge: perpendicular distance to a finite segment
  {
    MeasureObject p{ Type::POINT, { 0, 2, 0 }, {} };
    MeasureObject e{ Type::EDGE, { -1, 0, 0 }, { 1, 0, 0 } };
    const auto r = f3d::detail::ComputeDistance(p, e);
    test("point-edge perpendicular distance", r.Distance == approx(2.0));
  }

  // point-edge: closest point is an endpoint when the foot of perpendicular
  // is outside the segment
  {
    MeasureObject p{ Type::POINT, { 5, 0, 0 }, {} };
    MeasureObject e{ Type::EDGE, { -1, 0, 0 }, { 1, 0, 0 } };
    const auto r = f3d::detail::ComputeDistance(p, e);
    test("point-edge endpoint distance", r.Distance == approx(4.0));
  }

  // edge-edge: minimum distance between two parallel finite segments
  {
    MeasureObject e1{ Type::EDGE, { 0, 0, 0 }, { 2, 0, 0 } };
    MeasureObject e2{ Type::EDGE, { 0, 3, 0 }, { 2, 3, 0 } };
    const auto r = f3d::detail::ComputeDistance(e1, e2);
    test("edge-edge distance", r.Distance == approx(3.0));
  }
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake --build build --target libf3dSDKTests 2>&1 | tail -20`
Expected: FAIL — link error, `ComputeDistance` is undefined.

- [ ] **Step 3: Implement `ComputeDistance` in `measurementManager.cxx`**

Add `#include <vtkMath.h>` and `#include <vtkLine.h>` to the includes, then add this implementation (a single internal helper plus `ComputeDistance`):

```cpp
namespace
{
// Closest point on the finite segment [s0,s1] to point p, written into out.
void ClosestPointOnSegment(
  const double p[3], const double s0[3], const double s1[3], double out[3])
{
  double seg[3];
  vtkMath::Subtract(s1, s0, seg);
  const double len2 = vtkMath::Dot(seg, seg);
  if (len2 == 0.0)
  {
    out[0] = s0[0];
    out[1] = s0[1];
    out[2] = s0[2];
    return;
  }
  double ps0[3];
  vtkMath::Subtract(p, s0, ps0);
  double t = vtkMath::Dot(ps0, seg) / len2;
  t = std::max(0.0, std::min(1.0, t));
  out[0] = s0[0] + t * seg[0];
  out[1] = s0[1] + t * seg[1];
  out[2] = s0[2] + t * seg[2];
}
}

namespace f3d::detail
{
//----------------------------------------------------------------------------
MeasureResult ComputeDistance(const MeasureObject& a, const MeasureObject& b)
{
  using Type = MeasureObject::Type;
  MeasureResult result{};

  if (a.ObjType == Type::POINT && b.ObjType == Type::POINT)
  {
    result.ClosestA = a.P0;
    result.ClosestB = b.P0;
  }
  else if (a.ObjType == Type::POINT && b.ObjType == Type::EDGE)
  {
    double out[3];
    ClosestPointOnSegment(a.P0.data(), b.P0.data(), b.P1.data(), out);
    result.ClosestA = a.P0;
    result.ClosestB = { out[0], out[1], out[2] };
  }
  else if (a.ObjType == Type::EDGE && b.ObjType == Type::POINT)
  {
    double out[3];
    ClosestPointOnSegment(b.P0.data(), a.P0.data(), a.P1.data(), out);
    result.ClosestA = { out[0], out[1], out[2] };
    result.ClosestB = b.P0;
  }
  else // EDGE - EDGE
  {
    double t1 = 0.0;
    double t2 = 0.0;
    double c1[3];
    double c2[3];
    // vtkLine::DistanceBetweenLineSegments returns the squared distance and
    // fills the closest points c1/c2 and parametric values t1/t2.
    vtkLine::DistanceBetweenLineSegments(const_cast<double*>(a.P0.data()),
      const_cast<double*>(a.P1.data()), const_cast<double*>(b.P0.data()),
      const_cast<double*>(b.P1.data()), c1, c2, t1, t2);
    result.ClosestA = { c1[0], c1[1], c1[2] };
    result.ClosestB = { c2[0], c2[1], c2[2] };
  }

  result.Distance = std::sqrt(vtkMath::Distance2BetweenPoints(
    result.ClosestA.data(), result.ClosestB.data()));
  return result;
}
}
```

Add `#include <algorithm>` and `#include <cmath>` to the top includes.

- [ ] **Step 4: Run the test, verify it passes**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -10
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: PASS — all `UnitToMeters` and the 4 distance checks pass.

- [ ] **Step 5: Commit**

```bash
git add library/src/measurementManager.cxx library/testing/TestSDKMeasurement.cxx
git commit -m "feat: implement distance computation for measurement mode"
```

---

## Task 5: Implement and test pick resolution (`ResolvePickedObject`)

**Files:**
- Modify: `library/src/measurementManager.cxx`
- Modify: `library/testing/TestSDKMeasurement.cxx`

- [ ] **Step 1: Add failing pick-resolution tests to `TestSDKMeasurement.cxx`**

Add `#include <vtkNew.h>`, `#include <vtkTriangle.h>`, `#include <vtkPoints.h>` to the test includes. Insert before `return test.result();`:

```cpp
  // Build a triangle with vertices A(0,0,0) B(10,0,0) C(0,10,0).
  {
    vtkNew<vtkTriangle> tri;
    tri->GetPoints()->SetPoint(0, 0.0, 0.0, 0.0);
    tri->GetPoints()->SetPoint(1, 10.0, 0.0, 0.0);
    tri->GetPoints()->SetPoint(2, 0.0, 10.0, 0.0);
    tri->GetPointIds()->SetId(0, 0);
    tri->GetPointIds()->SetId(1, 1);
    tri->GetPointIds()->SetId(2, 2);

    // A pick within snap tolerance of vertex B -> POINT at B.
    {
      const auto obj = f3d::detail::ResolvePickedObject({ 9.95, 0.02, 0.0 }, tri, 0.2);
      test("resolve pick snaps to vertex",
        obj.ObjType == f3d::detail::MeasureObject::Type::POINT &&
          obj.P0 == approx(std::array<double, 3>{ 10.0, 0.0, 0.0 }));
    }

    // A pick near the middle of edge A-B (far from any vertex) -> EDGE A-B.
    {
      const auto obj = f3d::detail::ResolvePickedObject({ 5.0, 0.05, 0.0 }, tri, 0.2);
      const bool isEdgeAB = obj.ObjType == f3d::detail::MeasureObject::Type::EDGE &&
        ((obj.P0 == approx(std::array<double, 3>{ 0.0, 0.0, 0.0 }) &&
           obj.P1 == approx(std::array<double, 3>{ 10.0, 0.0, 0.0 })) ||
          (obj.P0 == approx(std::array<double, 3>{ 10.0, 0.0, 0.0 }) &&
            obj.P1 == approx(std::array<double, 3>{ 0.0, 0.0, 0.0 })));
      test("resolve pick snaps to nearest edge", isEdgeAB);
    }
  }
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake --build build --target libf3dSDKTests 2>&1 | tail -20`
Expected: FAIL — link error, `ResolvePickedObject` is undefined.

- [ ] **Step 3: Implement `ResolvePickedObject` in `measurementManager.cxx`**

Add `#include <vtkCell.h>` and `#include <vtkPoints.h>` to the includes. Add to the `f3d::detail` namespace:

```cpp
//----------------------------------------------------------------------------
MeasureObject ResolvePickedObject(
  const std::array<double, 3>& worldPos, vtkCell* cell, double snapTol)
{
  MeasureObject obj{};
  vtkPoints* pts = cell->GetPoints();
  const vtkIdType nbPts = pts->GetNumberOfPoints();

  // 1. Snap to the nearest vertex if within snapTol.
  vtkIdType nearestVertex = -1;
  double bestVertexDist2 = snapTol * snapTol;
  for (vtkIdType i = 0; i < nbPts; ++i)
  {
    double p[3];
    pts->GetPoint(i, p);
    const double d2 = vtkMath::Distance2BetweenPoints(worldPos.data(), p);
    if (d2 <= bestVertexDist2)
    {
      bestVertexDist2 = d2;
      nearestVertex = i;
    }
  }
  if (nearestVertex >= 0)
  {
    double p[3];
    pts->GetPoint(nearestVertex, p);
    obj.ObjType = MeasureObject::Type::POINT;
    obj.P0 = { p[0], p[1], p[2] };
    return obj;
  }

  // 2. Otherwise pick the cell edge nearest to worldPos.
  vtkIdType bestEdge = 0;
  double bestEdgeDist2 = std::numeric_limits<double>::max();
  for (vtkIdType i = 0; i < nbPts; ++i)
  {
    double e0[3];
    double e1[3];
    pts->GetPoint(i, e0);
    pts->GetPoint((i + 1) % nbPts, e1);
    double closest[3];
    ClosestPointOnSegment(worldPos.data(), e0, e1, closest);
    const double d2 = vtkMath::Distance2BetweenPoints(worldPos.data(), closest);
    if (d2 < bestEdgeDist2)
    {
      bestEdgeDist2 = d2;
      bestEdge = i;
    }
  }
  double e0[3];
  double e1[3];
  pts->GetPoint(bestEdge, e0);
  pts->GetPoint((bestEdge + 1) % nbPts, e1);
  obj.ObjType = MeasureObject::Type::EDGE;
  obj.P0 = { e0[0], e0[1], e0[2] };
  obj.P1 = { e1[0], e1[1], e1[2] };
  return obj;
}
```

Add `#include <limits>` to the top includes.

- [ ] **Step 4: Run the test, verify it passes**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -10
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: PASS — all checks, including the 2 pick-resolution checks.

- [ ] **Step 5: Commit**

```bash
git add library/src/measurementManager.cxx library/testing/TestSDKMeasurement.cxx
git commit -m "feat: implement pick resolution for measurement mode"
```

---

## Task 6: Implement `measurementManager` class methods (state, no rendering yet)

**Files:**
- Modify: `library/src/measurementManager.cxx`

- [ ] **Step 1: Add includes and the constructor/destructor**

At the top of `measurementManager.cxx` add:

```cpp
#include "interactor_impl.h"
#include "options.h"
#include "window_impl.h"

#include <vtkActor.h>

#include <sstream>
```

Add the class method implementations to the `f3d::detail` namespace:

```cpp
//----------------------------------------------------------------------------
measurementManager::measurementManager(options& options, window_impl& window)
  : Options(options)
  , Window(window)
{
}

//----------------------------------------------------------------------------
measurementManager::~measurementManager() = default;

//----------------------------------------------------------------------------
void measurementManager::SetInteractor(interactor_impl* interactor)
{
  this->Interactor = interactor;
}

//----------------------------------------------------------------------------
void measurementManager::ToggleMeasurement()
{
  this->Active = !this->Active;
  if (!this->Active)
  {
    this->Clear();
  }
}

//----------------------------------------------------------------------------
void measurementManager::Clear()
{
  this->Selection.clear();
  this->Result.reset();
  this->RemoveActors();
}

//----------------------------------------------------------------------------
void measurementManager::HandlePick(
  const std::array<double, 3>& worldPos, vtkCell* cell)
{
  if (!this->Active || cell == nullptr)
  {
    return;
  }

  // A pick after a completed measurement starts a fresh one.
  if (this->Selection.size() == 2)
  {
    this->Selection.clear();
    this->Result.reset();
  }

  // Snap tolerance scales with the model: 1% of the picked cell's longest edge.
  double bounds[6];
  cell->GetBounds(bounds);
  const double diag = std::sqrt((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
    (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
    (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
  const double snapTol = 0.25 * diag;

  this->Selection.push_back(ResolvePickedObject(worldPos, cell, snapTol));

  if (this->Selection.size() == 2)
  {
    this->Result = ComputeDistance(this->Selection[0], this->Selection[1]);
  }

  this->UpdateActors();
}

//----------------------------------------------------------------------------
std::string measurementManager::GetResultString() const
{
  if (this->Selection.empty())
  {
    return "Select first object";
  }
  if (this->Selection.size() == 1)
  {
    return "Select second object";
  }

  // Both selected: format the distance, applying unit conversion if both
  // model and display units are known.
  double value = this->Result->Distance;
  std::string suffix;
  const std::optional<double> modelF = UnitToMeters(this->Options.ui.measurement.model_unit);
  const std::optional<double> displayF = UnitToMeters(this->Options.ui.measurement.display_unit);
  if (modelF.has_value() && displayF.has_value())
  {
    value = value * modelF.value() / displayF.value();
    suffix = " " + this->Options.ui.measurement.display_unit;
  }

  std::ostringstream oss;
  oss.precision(4);
  oss << "Distance: " << value << suffix;
  return oss.str();
}
```

- [ ] **Step 2: Add temporary empty `UpdateActors`/`RemoveActors` stubs**

Add to the `f3d::detail` namespace (these are filled in Task 9):

```cpp
//----------------------------------------------------------------------------
void measurementManager::UpdateActors()
{
}

//----------------------------------------------------------------------------
void measurementManager::RemoveActors()
{
}
```

- [ ] **Step 3: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add library/src/measurementManager.cxx
git commit -m "feat: implement measurementManager state machine"
```

---

## Task 7: Own the manager in the interactor and wire the toggle command + binding

**Files:**
- Modify: `library/src/interactor_impl.cxx`

- [ ] **Step 1: Include the header**

At the top of `library/src/interactor_impl.cxx`, near `#include "animationManager.h"` (line 3), add:

```cpp
#include "measurementManager.h"
```

- [ ] **Step 2: Add the manager as a member of the `internals` struct**

In the `internals` struct (look near `animationManager* AnimationManager;` at line ~672), add a by-value member. The `internals` constructor already receives `options& options` and `window_impl& window` — find the `internals(...)` constructor and add `MeasurementManager` to its member initializer list. Add the member declaration near `AnimationManager`:

```cpp
  measurementManager MeasurementManager;
```

And in the `internals` constructor initializer list (the one that initializes `Options`, `Window`, etc.), add:

```cpp
    , MeasurementManager(options, window)
```

(Place it consistently with the other members; the exact constructor is the `internals(options& options, window_impl& window, scene_impl& scene, interactor_impl& self)` constructor.)

- [ ] **Step 3: Set the interactor on the manager in the `interactor_impl` constructor**

In `interactor_impl::interactor_impl(...)` (line ~706), after `this->Internals->Window.SetInteractor(this);`, add:

```cpp
  this->Internals->MeasurementManager.SetInteractor(this);
```

- [ ] **Step 4: Register the `toggle_measurement` command**

In `interactor_impl::initCommands()` — find where `toggle_animation` is registered (line ~1337, the `this->addCommand("toggle_animation", ...)` call). Add a sibling command:

```cpp
  this->addCommand(
    "toggle_measurement",
    [&](const std::vector<std::string>&)
    {
      this->Internals->MeasurementManager.ToggleMeasurement();
      this->Internals->Window.GetRenderer()->SetCheatSheetConfigured(false);
      this->requestRender();
    },
    command_documentation_t{ "toggle_measurement", "toggle measurement mode on/off" });
```

- [ ] **Step 5: Register the `Shift+M` binding**

In `interactor_impl::initBindings()` — find the existing `M` binding line (`this->addBinding({mod_t::NONE, "M"}, "toggle ui.metadata", ...)` at line ~1704). Add immediately after it:

```cpp
  auto docMeasure = [&]()
  {
    return std::pair(std::string("Measurement mode"),
      this->Internals->MeasurementManager.IsActive() ? std::string("ON") : std::string("OFF"));
  };
  this->addBinding({ mod_t::SHIFT, "M" }, "toggle_measurement", "Scene", docMeasure,
    f3d::interactor::BindingType::TOGGLE);
```

- [ ] **Step 6: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: build succeeds.

- [ ] **Step 7: Commit**

```bash
git add library/src/interactor_impl.cxx
git commit -m "feat: wire measurement mode toggle command and Shift+M binding"
```

---

## Task 8: Route left-clicks and Escape to the manager

**Files:**
- Modify: `library/src/interactor_impl.cxx`

- [ ] **Step 1: Add a `LeftButtonPress` observer in the observer-setup block**

In the observer setup block (where `middleButtonPressCallback` is created, line ~126), add immediately after the middle-button observers:

```cpp
    vtkNew<vtkCallbackCommand> leftButtonPressCallback;
    leftButtonPressCallback->SetClientData(this);
    leftButtonPressCallback->SetCallback(OnLeftButtonPress);
    this->Style->AddObserver(vtkCommand::LeftButtonPressEvent, leftButtonPressCallback);
```

- [ ] **Step 2: Implement the `OnLeftButtonPress` static callback**

Add this static method next to `OnMiddleButtonPress` (line ~363) in the `internals` struct. It mirrors the picking logic from `OnMiddleButtonRelease` but consumes the click when measurement mode is active:

```cpp
  //----------------------------------------------------------------------------
  static void OnLeftButtonPress(vtkObject*, unsigned long, void* clientData, void*)
  {
    internals* self = static_cast<internals*>(clientData);

    if (!self->MeasurementManager.IsActive())
    {
      // Not in measurement mode: let the normal interactor style handle it.
      self->Style->OnLeftButtonDown();
      return;
    }

    const int* pos = self->VTKInteractor->GetEventPosition();
    vtkRenderer* renderer =
      self->VTKInteractor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();

    if (self->CellPicker->Pick(pos[0], pos[1], 0, renderer))
    {
      double picked[3];
      self->CellPicker->GetPickPosition(picked);
      vtkCell* cell = nullptr;
      vtkDataSet* ds = self->CellPicker->GetDataSet();
      const vtkIdType cellId = self->CellPicker->GetCellId();
      if (ds != nullptr && cellId >= 0)
      {
        cell = ds->GetCell(cellId);
      }
      if (cell != nullptr)
      {
        self->MeasurementManager.HandlePick(
          { picked[0], picked[1], picked[2] }, cell);
        self->Style->GetInteractor()->GetRenderWindow()->Render();
      }
    }
    // Click consumed: do NOT call OnLeftButtonDown(), so the camera does not rotate.
  }
```

Add the matching `#include <vtkDataSet.h>` near the other VTK includes at the top of the file if it is not already present.

> Note: `vtkCellPicker::GetDataSet()` and `GetCellId()` give the picked cell. If `GetDataSet()` returns null in practice (some VTK builds), fall back to `self->CellPicker->GetActors()` / mapper input — but try `GetDataSet()` first.

- [ ] **Step 3: Intercept Escape when a measurement selection exists**

Find the `OnKeyPress` static callback (it reads `rwi->GetKeySym()` around line 329 and calls `self->TriggerBinding(interaction, "")`). Before the `TriggerBinding` call, add an early return that handles Escape in measurement mode:

```cpp
    if (interaction == "Escape" && self->MeasurementManager.IsActive() &&
      self->MeasurementManager.HasSelection())
    {
      self->MeasurementManager.Clear();
      self->Style->GetInteractor()->GetRenderWindow()->Render();
      return;
    }
```

Place this right after `interaction` is computed and before the binding is triggered. This makes Escape clear the current measurement selection instead of toggling the console, but only while measurement mode is active AND something is selected — otherwise Escape behaves as before.

- [ ] **Step 4: Clear the measurement when a file is loaded or reloaded**

The interactor has no per-load callback, but `scene_impl` already holds an
`interactor_impl*` (`this->Internals->Interactor`, set via `SetInteractor`). Add
an implementation-only API on the interactor and call it from the scene load path.

In `library/private/interactor_impl.h`, in the "Implementation only API" group
(near `void SetAnimationManager(animationManager* manager);`), declare:

```cpp
  /**
   * Implementation only API.
   * Clear any active measurement selection. Called by the scene on file load.
   */
  void ClearMeasurement();
```

In `library/src/interactor_impl.cxx`, implement it next to `SetAnimationManager`
(line ~2162):

```cpp
//----------------------------------------------------------------------------
void interactor_impl::ClearMeasurement()
{
  this->Internals->MeasurementManager.Clear();
}
```

In `library/src/scene_impl.cxx`, in the internal load method — right before the
`this->AnimationManager.Initialize();` call (line ~156) — add:

```cpp
    if (this->Internals->Interactor)
    {
      this->Internals->Interactor->ClearMeasurement();
    }
```

Match the exact `Interactor` member access used elsewhere in `scene_impl.cxx`
(see line 865, `this->Internals->Interactor`).

- [ ] **Step 5: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: build succeeds.

- [ ] **Step 6: Commit**

```bash
git add library/src/interactor_impl.cxx
git commit -m "feat: route left-clicks and Escape to measurement manager"
```

---

## Task 9: Render the 3D annotation actors

**Files:**
- Modify: `library/src/measurementManager.cxx`

- [ ] **Step 1: Add VTK includes**

Add to the includes in `measurementManager.cxx`:

```cpp
#include <vtkF3DRenderer.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
```

(`vtkF3DRenderer.h` is in `vtkext/private/module/`; it is already on the `libf3d` private include path — `animationManager.cxx` includes it the same way.)

- [ ] **Step 2: Implement `RemoveActors`**

Replace the empty `RemoveActors` stub with:

```cpp
//----------------------------------------------------------------------------
void measurementManager::RemoveActors()
{
  vtkF3DRenderer* renderer = this->Window.GetRenderer();
  if (renderer != nullptr)
  {
    for (const auto& actor : this->Actors)
    {
      renderer->RemoveActor(actor);
    }
  }
  this->Actors.clear();
}
```

- [ ] **Step 3: Implement `UpdateActors`**

Replace the empty `UpdateActors` stub with:

```cpp
//----------------------------------------------------------------------------
void measurementManager::UpdateActors()
{
  this->RemoveActors();

  vtkF3DRenderer* renderer = this->Window.GetRenderer();
  if (renderer == nullptr)
  {
    return;
  }

  // Estimate a marker size from the renderer's visible bounds.
  double bounds[6];
  renderer->ComputeVisiblePropBounds(bounds);
  const double diag = std::sqrt((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
    (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
    (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
  const double markerRadius = (diag > 0.0 ? diag : 1.0) * 0.01;

  const auto addLine = [&](const std::array<double, 3>& a, const std::array<double, 3>& b,
                         double r, double g, double bl, double width)
  {
    vtkNew<vtkLineSource> line;
    line->SetPoint1(a[0], a[1], a[2]);
    line->SetPoint2(b[0], b[1], b[2]);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(line->GetOutputPort());
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, bl);
    actor->GetProperty()->SetLineWidth(width);
    actor->PickableOff();
    renderer->AddActor(actor);
    this->Actors.emplace_back(actor);
  };

  const auto addSphere = [&](const std::array<double, 3>& c)
  {
    vtkNew<vtkSphereSource> sphere;
    sphere->SetCenter(c[0], c[1], c[2]);
    sphere->SetRadius(markerRadius);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphere->GetOutputPort());
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.85, 0.0);
    actor->PickableOff();
    renderer->AddActor(actor);
    this->Actors.emplace_back(actor);
  };

  // Markers for each selected object.
  for (const MeasureObject& obj : this->Selection)
  {
    if (obj.ObjType == MeasureObject::Type::POINT)
    {
      addSphere(obj.P0);
    }
    else
    {
      addLine(obj.P0, obj.P1, 1.0, 0.85, 0.0, 4.0); // highlight the selected edge
    }
  }

  // Connecting line between the two closest points.
  if (this->Result.has_value())
  {
    addLine(this->Result->ClosestA, this->Result->ClosestB, 0.1, 0.8, 1.0, 2.0);
  }
}
```

- [ ] **Step 4: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: build succeeds.

- [ ] **Step 5: Manual smoke check**

Run F3D on a test model and exercise the feature:
```bash
./build/bin/f3d testing/data/cow.vtp
```
Press <kbd>Shift</kbd>+<kbd>M</kbd>, click two points on the model. Expected: two yellow sphere markers and a cyan connecting line appear. Press <kbd>Esc</kbd>: they disappear. (No overlay panel yet — that is Task 10.)

- [ ] **Step 6: Commit**

```bash
git add library/src/measurementManager.cxx
git commit -m "feat: render 3D measurement annotation actors"
```

---

## Task 10: Add the `RenderMeasurement` overlay panel

**Files:**
- Modify: `vtkext/private/module/vtkF3DUIActor.h`
- Modify: `vtkext/private/module/vtkF3DUIActor.cxx`
- Modify: `vtkext/private/module/vtkF3DImguiActor.h`
- Modify: `vtkext/private/module/vtkF3DImguiActor.cxx`
- Modify: `vtkext/private/module/vtkF3DRenderer.h`
- Modify: `vtkext/private/module/vtkF3DRenderer.cxx`

- [ ] **Step 1: Add measurement state + setters to `vtkF3DUIActor.h`**

In `vtkF3DUIActor.h`, after the `SetMetaData`/metadata members, add public setters:

```cpp
  /**
   * Set the measurement panel visibility. False by default.
   */
  void SetMeasurementVisibility(bool show);

  /**
   * Set the measurement result/prompt string. Empty by default.
   */
  void SetMeasurement(const std::string& measurement);

  /**
   * Set the measurement model/display unit strings (shown in the combo boxes).
   */
  void SetMeasurementUnits(const std::string& modelUnit, const std::string& displayUnit);
```

In the `protected:` section, after `RenderMetaData()`, add:

```cpp
  /**
   * Render the measurement panel UI widget
   */
  virtual void RenderMeasurement()
  {
  }
```

In the protected members area (near `MetaDataVisible` / `MetaData`), add:

```cpp
  bool MeasurementVisible = false;
  std::string Measurement = "";
  std::string MeasurementModelUnit = "";
  std::string MeasurementDisplayUnit = "";
```

- [ ] **Step 2: Implement the setters and call `RenderMeasurement` in `vtkF3DUIActor.cxx`**

In `vtkF3DUIActor.cxx`, implement the three setters near `SetMetaData` (copy the trivial-setter style used by neighbouring methods):

```cpp
//----------------------------------------------------------------------------
void vtkF3DUIActor::SetMeasurementVisibility(bool show)
{
  this->MeasurementVisible = show;
}

//----------------------------------------------------------------------------
void vtkF3DUIActor::SetMeasurement(const std::string& measurement)
{
  this->Measurement = measurement;
}

//----------------------------------------------------------------------------
void vtkF3DUIActor::SetMeasurementUnits(
  const std::string& modelUnit, const std::string& displayUnit)
{
  this->MeasurementModelUnit = modelUnit;
  this->MeasurementDisplayUnit = displayUnit;
}
```

In `vtkF3DUIActor::RenderOverlay` (line ~204), find the block that calls `this->RenderMetaData();` (guarded by `if (this->MetaDataVisible)` near line 263). After that `if` block add:

```cpp
  if (this->MeasurementVisible)
  {
    this->RenderMeasurement();
  }
```

- [ ] **Step 3: Declare `RenderMeasurement` override in `vtkF3DImguiActor.h`**

In `vtkF3DImguiActor.h`, after the `void RenderMetaData() override;` declaration, add:

```cpp
  /**
   * Render the measurement panel UI widget
   */
  void RenderMeasurement() override;
```

- [ ] **Step 4: Implement `RenderMeasurement` in `vtkF3DImguiActor.cxx`**

Add this implementation near `vtkF3DImguiActor::RenderMetaData`. It draws a bottom-right panel with the result text and two unit combo boxes. Changing a combo emits a `set` command through the same user-event channel the rest of the UI uses (`vtkF3DImguiConsole`/`vtkF3DUserEvents::TriggerEvent`); inspect how `vtkF3DImguiConsole.cxx` triggers commands and reuse that exact mechanism.

```cpp
void vtkF3DImguiActor::RenderMeasurement()
{
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  constexpr float margin = F3DStyle::GetDefaultMargin();

  static const char* units[] = { "", "mm", "cm", "m", "in", "ft" };
  constexpr int unitCount = 6;

  const auto unitIndex = [&](const std::string& u)
  {
    for (int i = 0; i < unitCount; ++i)
    {
      if (u == units[i])
      {
        return i;
      }
    }
    return 0;
  };

  const ImVec2 winSize(260.f, 0.f); // height auto
  ::SetupNextWindow(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - winSize.x - margin,
                      viewport->WorkPos.y + viewport->WorkSize.y - 110.f - margin),
    winSize);

  ImGuiStyle& style = ImGui::GetStyle();
  style.Colors[ImGuiCol_WindowBg] = ImVec4(
    this->BackdropColor[0], this->BackdropColor[1], this->BackdropColor[2], this->BackdropOpacity);

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings |
    ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_AlwaysAutoResize;

  ImGui::Begin("Measurement", nullptr, flags);
  ImGui::TextUnformatted(this->Measurement.c_str());
  ImGui::Separator();

  int modelIdx = unitIndex(this->MeasurementModelUnit);
  if (ImGui::Combo("Model units", &modelIdx, units, unitCount))
  {
    this->EmitMeasurementUnitChange("ui.measurement.model_unit", units[modelIdx]);
  }

  int displayIdx = unitIndex(this->MeasurementDisplayUnit);
  if (ImGui::Combo("Display units", &displayIdx, units, unitCount))
  {
    this->EmitMeasurementUnitChange("ui.measurement.display_unit", units[displayIdx]);
  }

  ImGui::End();
}
```

Add a small private helper `EmitMeasurementUnitChange(const std::string& optionName, const std::string& value)` to `vtkF3DImguiActor` that triggers the command `set <optionName> <value>` through the existing command channel. Look at how `vtkF3DImguiConsole::RenderConsole` submits a typed command (it invokes `vtkF3DUserEvents::TriggerEvent` on the output window with the command string) and replicate that. Declare the helper in the `private:` section of `vtkF3DImguiActor.h`.

> Note: an empty-string unit displays as a blank combo entry meaning "unitless". `::SetupNextWindow` is the file-local helper already used by `RenderMetaData`; confirm its signature in `vtkF3DImguiActor.cxx` and match it.

- [ ] **Step 5: Pass measurement data from `vtkF3DRenderer`**

In `vtkF3DRenderer.h`, declare a public method:

```cpp
  /**
   * Configure the measurement panel UI from the given state.
   */
  void ConfigureMeasurement(bool visible, const std::string& text,
    const std::string& modelUnit, const std::string& displayUnit);
```

In `vtkF3DRenderer.cxx`, implement it next to `ConfigureMetaData`:

```cpp
//----------------------------------------------------------------------------
void vtkF3DRenderer::ConfigureMeasurement(bool visible, const std::string& text,
  const std::string& modelUnit, const std::string& displayUnit)
{
  this->UIActor->SetMeasurementVisibility(visible);
  this->UIActor->SetMeasurement(text);
  this->UIActor->SetMeasurementUnits(modelUnit, displayUnit);
}
```

- [ ] **Step 6: Drive `ConfigureMeasurement` from `measurementManager::UpdateActors`**

In `library/src/measurementManager.cxx`, at the end of `UpdateActors()` (after the actor rebuilding), add:

```cpp
  renderer->ConfigureMeasurement(this->IsPanelVisible(), this->GetResultString(),
    this->Options.ui.measurement.model_unit, this->Options.ui.measurement.display_unit);
```

Also call `ConfigureMeasurement` from `ToggleMeasurement` and `Clear` so the panel appears/disappears with the mode even before any pick. Add a private helper to `measurementManager`:

```cpp
//----------------------------------------------------------------------------
void measurementManager::RefreshPanel()
{
  vtkF3DRenderer* renderer = this->Window.GetRenderer();
  if (renderer != nullptr)
  {
    renderer->ConfigureMeasurement(this->IsPanelVisible(), this->GetResultString(),
      this->Options.ui.measurement.model_unit, this->Options.ui.measurement.display_unit);
  }
}
```

Declare `void RefreshPanel();` in the `private:` section of `measurementManager.h`. Call `this->RefreshPanel();` at the end of `ToggleMeasurement()`, `Clear()`, and `UpdateActors()` (replacing the inline `ConfigureMeasurement` call from the previous step with `RefreshPanel()`).

- [ ] **Step 7: Build to verify it compiles**

Run: `cmake --build build 2>&1 | tail -20`
Expected: build succeeds for `libf3d`, `vtkextPrivate`, and the F3D application.

- [ ] **Step 8: Manual smoke check**

Run:
```bash
./build/bin/f3d testing/data/cow.vtp
```
Press <kbd>Shift</kbd>+<kbd>M</kbd>: a panel appears bottom-right showing "Select first object" plus two unit combos. Click two points: panel shows "Distance: …". Change "Model units" to `mm` and "Display units" to `in`: the displayed value updates and gains an `in` suffix. Press <kbd>Esc</kbd>: markers and line clear; panel still visible while mode is on showing the prompt again.

- [ ] **Step 9: Commit**

```bash
git add vtkext/private/module/vtkF3DUIActor.h vtkext/private/module/vtkF3DUIActor.cxx vtkext/private/module/vtkF3DImguiActor.h vtkext/private/module/vtkF3DImguiActor.cxx vtkext/private/module/vtkF3DRenderer.h vtkext/private/module/vtkF3DRenderer.cxx library/src/measurementManager.cxx library/private/measurementManager.h
git commit -m "feat: add measurement overlay panel with unit combo boxes"
```

---

## Task 11: Documentation and an interaction test

**Files:**
- Modify: `doc/user/04-INTERACTIONS.md`
- Modify: `doc/user/CHANGELOG.md`
- Create: `library/testing/TestSDKMeasurementInteraction.cxx` (interaction baseline test)
- Modify: `library/testing/CMakeLists.txt`

- [ ] **Step 1: Document the binding in `doc/user/04-INTERACTIONS.md`**

In the "Other options can be toggled..." hotkey list, after the `<kbd>M</kbd>: the display of the metadata` line, add:

```markdown
- <kbd>Shift</kbd>+<kbd>m</kbd>: toggle measurement mode.
```

After the "## Cycling Coloring" section, add a new section:

```markdown
## Measurement Mode

Press <kbd>Shift</kbd>+<kbd>M</kbd> to toggle measurement mode. While active,
left-click two objects in the model to measure the distance between them. Each
click selects either a mesh **vertex** (when clicking near one) or a mesh
**edge**. The distance is drawn as a line in the 3D scene and shown in a panel
in the bottom-right corner.

The panel has two combo boxes: **Model units** (the unit the model geometry is
in) and **Display units** (the unit to show results in). Supported units are
`mm`, `cm`, `m`, `in` and `ft`; leaving a combo blank means unitless and
disables conversion. These map to the `ui.measurement.model_unit` and
`ui.measurement.display_unit` options.

Press <kbd>Esc</kbd> while in measurement mode to clear the current selection.
Selecting a third object discards the previous measurement and starts a new one.

> [!NOTE]
> Edges are triangulation edges of the mesh, not CAD feature edges.
```

- [ ] **Step 2: Add a `CHANGELOG.md` entry**

In `doc/user/CHANGELOG.md`, under the current unreleased/top section, add a bullet matching the file's style:

```markdown
- Added a measurement mode (toggle with `Shift+M`) to measure distances between mesh points and edges, with unit conversion.
```

- [ ] **Step 3: Write the interaction test `library/testing/TestSDKMeasurementInteraction.cxx`**

Model this on an existing interaction test that drives clicks via the engine API. Inspect `library/testing/TestSDKInteractorCommand.cxx` and `TestSDKTriggerInteractions.cxx` for the exact API to: load a scene, get the interactor, trigger mouse position + button events, and compare a rendered image. The test should:

```cpp
#include "PseudoUnitTest.h"

#include <engine.h>
#include <interactor.h>
#include <scene.h>
#include <window.h>

#include <iostream>

int TestSDKMeasurementInteraction(int argc, char* argv[])
{
  const std::string dataPath = std::string(argv[1]) + "data/cow.vtp";

  f3d::engine eng = f3d::engine::create(true);
  eng.getWindow().setSize(300, 300);
  eng.getScene().add(dataPath);

  f3d::interactor& inter = eng.getInteractor();

  // Enable measurement mode.
  inter.triggerCommand("toggle_measurement");

  // Two clicks at different screen positions over the model.
  inter.triggerMousePosition(120, 150);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::PRESS, f3d::interactor::MouseButton::LEFT);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::RELEASE, f3d::interactor::MouseButton::LEFT);

  inter.triggerMousePosition(190, 150);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::PRESS, f3d::interactor::MouseButton::LEFT);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::RELEASE, f3d::interactor::MouseButton::LEFT);

  eng.getWindow().render();

  // Escape clears the measurement; mode stays on.
  inter.triggerKeyboardKey(f3d::interactor::InputAction::PRESS, "Escape");

  eng.getWindow().render();

  // No crash and commands accepted: pass.
  return EXIT_SUCCESS;
}
```

Verify the exact `triggerMouseButton` / `triggerMousePosition` / `triggerKeyboardKey` / `triggerCommand` signatures against `library/public/interactor.h` (they exist — see `interactor_impl.h`) and adjust argument types if needed. If the project convention for this kind of test is a baseline-image comparison, follow `TestSDKHelpers.h` (`RenderTest` / image comparison) and add a baseline PNG under `testing/baselines/`; otherwise the non-crashing smoke form above is acceptable.

- [ ] **Step 4: Register the interaction test in `library/testing/CMakeLists.txt`**

Add `TestSDKMeasurementInteraction.cxx` to the `if(F3D_MODULE_UI)` block of `libf3dSDKTests_list` (it needs rendering + UI):

```cmake
    TestSDKMeasurementInteraction.cxx
```

Do NOT add it to `libf3dSDKTestsNoRender_list` — it requires rendering.

- [ ] **Step 5: Build and run all measurement tests**

Run:
```bash
cmake --build build 2>&1 | tail -20
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: `TestSDKMeasurement` passes; `TestSDKMeasurementInteraction` passes (or is reported DISABLED if rendering tests are off in this build — that is acceptable, note it).

- [ ] **Step 6: Commit**

```bash
git add doc/user/04-INTERACTIONS.md doc/user/CHANGELOG.md library/testing/TestSDKMeasurementInteraction.cxx library/testing/CMakeLists.txt
git commit -m "docs: document measurement mode; add interaction test"
```

---

## Task 12: Full verification

**Files:** none (verification only)

- [ ] **Step 1: Clean build**

Run: `cmake --build build 2>&1 | tail -30`
Expected: builds with no errors or new warnings related to measurement code.

- [ ] **Step 2: Run the full library test suite**

Run: `ctest --test-dir build -L libf3d 2>&1 | tail -30`
Expected: no regressions; measurement tests pass (rendering tests may be DISABLED depending on build config).

- [ ] **Step 3: Manual end-to-end check**

Run `./build/bin/f3d testing/data/cow.vtp` and verify:
- <kbd>H</kbd> cheatsheet lists "Measurement mode" with ON/OFF state under the Scene group.
- <kbd>Shift</kbd>+<kbd>M</kbd> toggles the panel; plain <kbd>M</kbd> still toggles metadata.
- Two left-clicks produce markers, a connecting line, and a distance in the panel.
- Changing the unit combos converts the displayed value.
- <kbd>Esc</kbd> clears the selection.

- [ ] **Step 4: Final commit if any fixes were needed**

```bash
git add -A
git commit -m "fix: address measurement mode verification findings"
```

---

## Notes for the Implementer

- **Verification discipline:** never mark a step done without running its command and seeing the expected output. Use the `superpowers:verification-before-completion` skill before claiming the feature complete.
- **`vtkCellPicker` API uncertainty:** Task 8 Step 2 uses `GetDataSet()`/`GetCellId()`. If these do not yield a usable `vtkCell*` in this VTK version, the picker also exposes `GetActor()` + the actor's mapper input — use whichever reliably returns the picked triangle. The rest of the design does not depend on which path is used.
- **`SetupNextWindow` / `F3DStyle::GetDefaultMargin`:** these are existing helpers in `vtkF3DImguiActor.cxx` / `F3DStyle`; match their real signatures rather than assuming.
- **Out of scope (see spec "Future Work"):** feature-edge detection, circle/arc/hole detection, angle measurement, multiple simultaneous measurements. Do not implement these.
