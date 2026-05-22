# Face Detection for Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a click in a triangle's interior select the whole coplanar face it belongs to, using the face's area-weighted centroid as the measurable point.

**Architecture:** Extends F3D's measurement feature. `MeasureObject` gains a `FACE` type carrying a centroid plus the region's triangle geometry. `ResolvePickedObject` takes the picked dataset (not a single cell) so it can region-grow coplanar triangles. The 3D annotation highlights the detected face region.

**Tech Stack:** C++17, VTK (vtkPolyData connectivity, vtkTriangle, vtkMath), CMake.

**Spec:** `doc/superpowers/specs/2026-05-22-measurement-face-detection-design.md`

---

## Background for the implementer

The measurement feature already exists. Current state of the relevant code:

- `library/private/measurementTools.h` / `library/src/measurementTools.cxx` — pure helpers and the `MeasureObject` / `MeasureResult` structs. `MeasureObject` currently has `enum class Type { POINT, EDGE }`, `Type ObjType`, `std::array<double,3> P0`, `std::array<double,3> P1`. `measurementTools.cxx` is compiled into both `libf3d` and the `libf3dSDKTests` test binary; it links the VTK `CommonCore`/`CommonDataModel` modules.
- `ResolvePickedObject(const std::array<double,3>& worldPos, vtkCell* cell, double snapTol)` — currently: pick within `snapTol` of a vertex → `POINT`; otherwise → the nearest `EDGE` (unconditional fallback). There is a file-local anonymous-namespace helper `ClosestPointOnSegment(p, s0, s1, out)`.
- `ComputeDistance(a, b)` — branches on `POINT`/`EDGE` combinations.
- `library/src/measurementManager.cxx` — `measurementManager`. `HandlePick(worldPos, vtkCell*)` and `HandleHover(worldPos, vtkCell*) -> bool` call `ResolvePickedObject`. A file-local anonymous-namespace helper `SnapToleranceForCell(vtkCell*)` computes the snap tolerance from a cell's bounds. `UpdateActors()` rebuilds 3D actors via local lambdas `addLine`/`addSphere` (adding to the overlay renderer obtained from `GetOverlayRenderer()`); `UpdateHoverActor()` builds a single hover actor stored in the member `vtkSmartPointer<vtkActor> HoverActor`.
- `library/src/interactor_impl.cxx` — `OnLeftButtonPress` and `OnMouseMove` pick with `vtkCellPicker`, obtain `vtkDataSet* ds = CellPicker->GetDataSet()` and `vtkIdType cellId = CellPicker->GetCellId()`, currently do `cell = ds->GetCell(cellId)` and pass the `vtkCell*` to `HandlePick`/`HandleHover`.
- The build dir `build/` is CMake-configured with `BUILD_TESTING=ON`.

**Note on the spec:** the spec's architecture section names the resolver parameter `vtkPolyData* mesh`. This plan uses `vtkDataSet*` instead, because the picked cell must always be obtainable (even for non-polydata datasets) — region growing then casts to `vtkPolyData` internally. This is a deliberate, faithful refinement of the spec's intent ("face falls back to the single picked triangle for non-polydata").

---

## File Structure

No new files. Changes are confined to:

- `library/private/measurementTools.h` / `library/src/measurementTools.cxx` — `FACE` type, `FacePoints`, the resolver signature change, region-grow, `ComputeDistance`.
- `library/private/measurementManager.h` / `library/src/measurementManager.cxx` — caller signature changes, face highlight rendering.
- `library/src/interactor_impl.cxx` — pass dataset + cell id instead of a `vtkCell*`.
- `library/testing/TestSDKMeasurement.cxx`, `library/testing/TestSDKMeasurementInteraction.cxx` — tests.
- `doc/user/04-INTERACTIONS.md`, `doc/CHANGELOG.md` — docs.

---

## Task 1: `FACE` type on `MeasureObject` and in `ComputeDistance`

**Files:**
- Modify: `library/private/measurementTools.h`
- Modify: `library/src/measurementTools.cxx`
- Modify: `library/testing/TestSDKMeasurement.cxx`

- [ ] **Step 1: Extend `MeasureObject` in `measurementTools.h`**

Add `#include <vector>` to the include block. Replace the `MeasureObject` struct with:

```cpp
/**
 * A selectable object: a single point, a finite edge segment, or a face.
 * For POINT and FACE, P0 is the measurable point (the vertex / the face
 * centroid). For EDGE, P0 and P1 are the endpoints. FacePoints holds the
 * region's triangle vertices (three consecutive entries per triangle) for
 * highlighting; it is empty for POINT and EDGE.
 */
struct MeasureObject
{
  enum class Type : std::uint8_t
  {
    POINT,
    EDGE,
    FACE
  };
  Type ObjType;
  std::array<double, 3> P0;
  std::array<double, 3> P1;
  std::vector<std::array<double, 3>> FacePoints;
};
```

- [ ] **Step 2: Add a failing test for `ComputeDistance` with a `FACE`**

In `library/testing/TestSDKMeasurement.cxx`, just before the final `return test.result();`, add:

```cpp
  // A FACE measures from its centroid (P0), exactly like a POINT.
  {
    using f3d::detail::MeasureObject;
    MeasureObject face{};
    face.ObjType = MeasureObject::Type::FACE;
    face.P0 = { 0.0, 0.0, 0.0 };
    MeasureObject point{};
    point.ObjType = MeasureObject::Type::POINT;
    point.P0 = { 3.0, 4.0, 0.0 };
    const auto r = f3d::detail::ComputeDistance(face, point);
    test("face-point distance uses face centroid", r.Distance == approx(5.0));
  }
```

- [ ] **Step 3: Run the test, verify it fails**

Run: `cmake --build build --target libf3dSDKTests 2>&1 | tail -15`
Expected: FAIL — compile error: `FACE` is not a member of `MeasureObject::Type` (until Step 1 is built) — if Step 1 already built, the test instead fails at runtime because `ComputeDistance` does not handle `FACE`. Either way it is not yet passing.

- [ ] **Step 4: Make `ComputeDistance` treat `FACE` as `POINT`**

In `library/src/measurementTools.cxx`, at the very start of `ComputeDistance`'s body (before the existing `if`/`else` chain), normalize `FACE` to `POINT` on local copies so the existing logic applies unchanged:

```cpp
MeasureResult ComputeDistance(const MeasureObject& aIn, const MeasureObject& bIn)
{
  using Type = MeasureObject::Type;

  // A FACE measures from its centroid (P0), identically to a POINT.
  MeasureObject a = aIn;
  MeasureObject b = bIn;
  if (a.ObjType == Type::FACE)
  {
    a.ObjType = Type::POINT;
  }
  if (b.ObjType == Type::FACE)
  {
    b.ObjType = Type::POINT;
  }

  MeasureResult result{};
  ...
```

The function's parameters are renamed `aIn`/`bIn`; the rest of the body (the `if (a.ObjType == Type::POINT && ...)` chain and everything after) stays exactly as it is, now operating on the local `a`/`b`.

- [ ] **Step 5: Run the test, verify it passes**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -10
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: PASS — all prior checks plus `face-point distance uses face centroid`.

- [ ] **Step 6: Commit**

```bash
git add library/private/measurementTools.h library/src/measurementTools.cxx library/testing/TestSDKMeasurement.cxx
git commit -m "feat: add FACE type to MeasureObject, measured as its centroid"
```

---

## Task 2: Change `ResolvePickedObject` to take the dataset (mechanical, no behavior change)

This task changes the resolver signature so it has the whole mesh available, and updates every caller and the existing tests. Interior clicks still resolve to an edge — face logic is added in Task 3.

**Files:**
- Modify: `library/private/measurementTools.h`
- Modify: `library/src/measurementTools.cxx`
- Modify: `library/private/measurementManager.h`
- Modify: `library/src/measurementManager.cxx`
- Modify: `library/src/interactor_impl.cxx`
- Modify: `library/testing/TestSDKMeasurement.cxx`

- [ ] **Step 1: Change the declaration in `measurementTools.h`**

Replace the forward declaration `class vtkCell;` with `class vtkDataSet;`. Replace the `ResolvePickedObject` declaration and its doc comment with:

```cpp
/**
 * Resolve a pick into a MeasureObject. worldPos is the picked world position;
 * dataset is the picked mesh and cellId the picked triangle. A pick within
 * snapTol of a triangle vertex resolves to a POINT; a pick within snapTol of a
 * triangle edge resolves to that EDGE; otherwise it resolves to a FACE (the
 * coplanar region around the picked triangle, measured at its centroid).
 */
MeasureObject ResolvePickedObject(const std::array<double, 3>& worldPos,
  vtkDataSet* dataset, vtkIdType cellId, double snapTol);
```

(`vtkIdType` is available via `<vtkType.h>`; add `#include <vtkType.h>` to the header's include block if `vtkIdType` is not already visible — verify by building.)

- [ ] **Step 2: Update `ResolvePickedObject` in `measurementTools.cxx` (mechanical)**

Add `#include <vtkDataSet.h>` to the includes. Change the function so it obtains the cell from the dataset, keeping ALL existing point/edge logic. The new opening:

```cpp
MeasureObject ResolvePickedObject(const std::array<double, 3>& worldPos,
  vtkDataSet* dataset, vtkIdType cellId, double snapTol)
{
  MeasureObject obj{};
  vtkCell* cell = dataset->GetCell(cellId);
  vtkPoints* pts = cell->GetPoints();
  const vtkIdType nbPts = pts->GetNumberOfPoints();
  ...
```

The rest of the function body (vertex snapping → `POINT`; nearest-edge loop → `EDGE`) stays exactly as it is. `<vtkCell.h>` and `<vtkPoints.h>` are already included.

- [ ] **Step 3: Update `SnapToleranceForCell` and the caller signatures in `measurementManager`**

In `library/private/measurementManager.h`, change the two declarations:

```cpp
  void HandlePick(const std::array<double, 3>& worldPos, vtkDataSet* dataset, vtkIdType cellId);
```
```cpp
  bool HandleHover(const std::array<double, 3>& worldPos, vtkDataSet* dataset, vtkIdType cellId);
```

Replace the forward declaration `class vtkCell;` with `class vtkDataSet;` (add `#include <vtkType.h>` if `vtkIdType` is not visible — verify by building).

In `library/src/measurementManager.cxx`:

Add `#include <vtkDataSet.h>`. Change the anonymous-namespace helper `SnapToleranceForCell` to take the dataset and cell id:

```cpp
// Snap tolerance for pick resolution, scaled to the picked cell's size.
double SnapToleranceForCell(vtkDataSet* dataset, vtkIdType cellId)
{
  double bounds[6];
  dataset->GetCellBounds(cellId, bounds);
  const double diag = std::sqrt((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
    (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
    (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
  return 0.25 * diag;
}
```

Update `HandlePick`'s signature and its body's guard + resolver call:

```cpp
void measurementManager::HandlePick(
  const std::array<double, 3>& worldPos, vtkDataSet* dataset, vtkIdType cellId)
{
  if (!this->Active || dataset == nullptr || cellId < 0)
  {
    return;
  }

  // A pick after a completed measurement starts a fresh one.
  if (this->Selection.size() == 2)
  {
    this->Selection.clear();
    this->Result.reset();
  }

  this->Selection.push_back(
    ResolvePickedObject(worldPos, dataset, cellId, SnapToleranceForCell(dataset, cellId)));

  if (this->Selection.size() == 2)
  {
    this->Result = ComputeDistance(this->Selection[0], this->Selection[1]);
  }

  this->ClearHover();
  this->UpdateActors();
}
```

Update `HandleHover` likewise — signature to `(worldPos, vtkDataSet* dataset, vtkIdType cellId)`, guard to `if (!this->Active || dataset == nullptr || cellId < 0) { return this->ClearHover(); }`, and the resolver call to `ResolvePickedObject(worldPos, dataset, cellId, SnapToleranceForCell(dataset, cellId))`. The rest of `HandleHover` (the equality check against `HoverObject`, `UpdateHoverActor`) is unchanged.

- [ ] **Step 4: Update the interactor in `interactor_impl.cxx`**

In `OnLeftButtonPress`, replace the block that extracts `cell` and calls `HandlePick`:

```cpp
    bool picked = false;
    if (self->CellPicker->Pick(pos[0], pos[1], 0, renderer))
    {
      double pickPos[3];
      self->CellPicker->GetPickPosition(pickPos);
      vtkDataSet* ds = self->CellPicker->GetDataSet();
      const vtkIdType cellId = self->CellPicker->GetCellId();
      if (ds != nullptr && cellId >= 0)
      {
        self->MeasurementManager.HandlePick({ pickPos[0], pickPos[1], pickPos[2] }, ds, cellId);
        self->Style->GetInteractor()->GetRenderWindow()->Render();
        picked = true;
      }
    }
```

(Keep the surrounding logic — the `picked` flag and the `if (!picked) { self->Style->OnLeftButtonDown(); }` fall-through — exactly as it is.)

In `OnMouseMove`, replace the block that extracts `cell` and calls `HandleHover`:

```cpp
    vtkDataSet* ds = nullptr;
    vtkIdType cellId = -1;
    double pickPos[3] = { 0.0, 0.0, 0.0 };
    if (self->CellPicker->Pick(pos[0], pos[1], 0, renderer))
    {
      self->CellPicker->GetPickPosition(pickPos);
      ds = self->CellPicker->GetDataSet();
      cellId = self->CellPicker->GetCellId();
    }

    const bool changed = (ds != nullptr && cellId >= 0)
      ? self->MeasurementManager.HandleHover({ pickPos[0], pickPos[1], pickPos[2] }, ds, cellId)
      : self->MeasurementManager.ClearHover();
```

If the existing `OnMouseMove` has slightly different variable names/structure, match by the `HandleHover` / `ClearHover` calls — those are the lines to update. The `#include <vtkCell.h>` in `interactor_impl.cxx` may now be unused; leave it (harmless) or remove it if you confirm nothing else uses `vtkCell` there.

- [ ] **Step 5: Update the existing `ResolvePickedObject` tests in `TestSDKMeasurement.cxx`**

The existing tests build a `vtkNew<vtkTriangle>` and pass it as a `vtkCell*`. They must build a `vtkPolyData` instead. Add `#include <vtkCellArray.h>` and `#include <vtkPolyData.h>` to the test includes (it already includes `<vtkNew.h>`, `<vtkPoints.h>`, `<vtkTriangle.h>`). Replace the existing triangle block — the one beginning `// Build a triangle with vertices A(0,0,0) B(10,0,0) C(0,10,0).` and containing the `resolve pick snaps to vertex` and `resolve pick snaps to nearest edge` checks — with:

```cpp
  // Build a one-triangle vtkPolyData: A(0,0,0) B(10,0,0) C(0,10,0).
  {
    vtkNew<vtkPoints> points;
    points->InsertNextPoint(0.0, 0.0, 0.0);
    points->InsertNextPoint(10.0, 0.0, 0.0);
    points->InsertNextPoint(0.0, 10.0, 0.0);
    vtkNew<vtkCellArray> polys;
    const vtkIdType triangle[3] = { 0, 1, 2 };
    polys->InsertNextCell(3, triangle);
    vtkNew<vtkPolyData> mesh;
    mesh->SetPoints(points);
    mesh->SetPolys(polys);
    mesh->BuildLinks();

    // A pick within snap tolerance of vertex B -> POINT at B.
    {
      const auto obj = f3d::detail::ResolvePickedObject({ 9.95, 0.02, 0.0 }, mesh, 0, 0.2);
      test("resolve pick snaps to vertex",
        obj.ObjType == f3d::detail::MeasureObject::Type::POINT &&
          obj.P0 == approx(std::array<double, 3>{ 10.0, 0.0, 0.0 }));
    }

    // A pick near the middle of edge A-B (far from any vertex) -> EDGE A-B.
    {
      const auto obj = f3d::detail::ResolvePickedObject({ 5.0, 0.05, 0.0 }, mesh, 0, 0.2);
      const bool isEdgeAB = obj.ObjType == f3d::detail::MeasureObject::Type::EDGE &&
        ((obj.P0 == approx(std::array<double, 3>{ 0.0, 0.0, 0.0 }) &&
           obj.P1 == approx(std::array<double, 3>{ 10.0, 0.0, 0.0 })) ||
          (obj.P0 == approx(std::array<double, 3>{ 10.0, 0.0, 0.0 }) &&
            obj.P1 == approx(std::array<double, 3>{ 0.0, 0.0, 0.0 })));
      test("resolve pick snaps to nearest edge", isEdgeAB);
    }
  }
```

- [ ] **Step 6: Build and run all tests**

Run:
```bash
cmake --build build 2>&1 | tail -15
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V 2>&1 | tail -20
```
Expected: clean build of `libf3d`, `vtkext`, `f3d`, and `libf3dSDKTests`; `TestSDKMeasurement` passes (point/edge checks under the new signature); `TestSDKMeasurementInteraction` passes.

- [ ] **Step 7: Commit**

```bash
git add library/private/measurementTools.h library/src/measurementTools.cxx library/private/measurementManager.h library/src/measurementManager.cxx library/src/interactor_impl.cxx library/testing/TestSDKMeasurement.cxx
git commit -m "refactor: ResolvePickedObject takes the picked dataset and cell id"
```

---

## Task 3: Region-grow and `FACE` resolution

**Files:**
- Modify: `library/src/measurementTools.cxx`
- Modify: `library/testing/TestSDKMeasurement.cxx`

- [ ] **Step 1: Add failing face tests to `TestSDKMeasurement.cxx`**

Insert before the final `return test.result();`:

```cpp
  // Two coplanar triangles in z=0 forming the unit square [0,1]x[0,1], plus a
  // third triangle that shares edge 1-2 with the square but is tilted steeply
  // up out of the z=0 plane.
  // Points: 0(0,0,0) 1(1,0,0) 2(1,1,0) 3(0,1,0) 4(2,1,2).
  {
    vtkNew<vtkPoints> points;
    points->InsertNextPoint(0.0, 0.0, 0.0);
    points->InsertNextPoint(1.0, 0.0, 0.0);
    points->InsertNextPoint(1.0, 1.0, 0.0);
    points->InsertNextPoint(0.0, 1.0, 0.0);
    points->InsertNextPoint(2.0, 1.0, 2.0);
    vtkNew<vtkCellArray> polys;
    const vtkIdType t0[3] = { 0, 1, 2 }; // flat, in z=0
    const vtkIdType t1[3] = { 0, 2, 3 }; // flat, in z=0
    const vtkIdType t2[3] = { 1, 4, 2 }; // tilted, shares edge 1-2 with t0
    polys->InsertNextCell(3, t0);
    polys->InsertNextCell(3, t1);
    polys->InsertNextCell(3, t2);
    vtkNew<vtkPolyData> mesh;
    mesh->SetPoints(points);
    mesh->SetPolys(polys);
    mesh->BuildLinks();

    // A pick in the interior of triangle 0 -> FACE covering the two flat
    // triangles; centroid is the unit square centre (0.5, 0.5, 0).
    const auto obj = f3d::detail::ResolvePickedObject({ 0.7, 0.2, 0.0 }, mesh, 0, 0.05);
    test("resolve interior pick is a face",
      obj.ObjType == f3d::detail::MeasureObject::Type::FACE);
    test("face centroid is the region centroid",
      obj.P0 == approx(std::array<double, 3>{ 0.5, 0.5, 0.0 }));

    // The region must include only the two flat triangles (6 vertex entries),
    // never the tilted ones, so every FacePoints entry has z == 0.
    bool allFlat = !obj.FacePoints.empty();
    for (const auto& p : obj.FacePoints)
    {
      if (std::fabs(p[2]) > 1e-9)
      {
        allFlat = false;
      }
    }
    test("face region stops at the sharp edge", allFlat);
    test("face region covers both flat triangles", obj.FacePoints.size() == 6);
  }
```

- [ ] **Step 2: Run the test, verify it fails**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -10
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V 2>&1 | tail -15
```
Expected: FAIL — the interior pick currently resolves to an `EDGE` (or the `FACE` checks fail), since face logic is not implemented yet.

- [ ] **Step 3: Add the region-grow helpers to `measurementTools.cxx`**

Add includes: `#include <vtkIdList.h>`, `#include <vtkPolyData.h>`, `#include <vtkTriangle.h>`, `#include <queue>`, `#include <set>`.

In the existing anonymous `namespace { ... }` block (the one with `ClosestPointOnSegment`), add:

```cpp
// Unit normal of triangle `cellId` of `mesh`, written into `normal`.
void TriangleNormal(vtkPolyData* mesh, vtkIdType cellId, double normal[3])
{
  vtkNew<vtkIdList> ptIds;
  mesh->GetCellPoints(cellId, ptIds);
  double p0[3];
  double p1[3];
  double p2[3];
  mesh->GetPoint(ptIds->GetId(0), p0);
  mesh->GetPoint(ptIds->GetId(1), p1);
  mesh->GetPoint(ptIds->GetId(2), p2);
  vtkTriangle::ComputeNormal(p0, p1, p2, normal);
}

// Breadth-first set of triangles coplanar with `seedCell` (normals within
// `angleToleranceDeg` of the seed triangle's normal), edge-connected. Bounded
// to `maxTriangles` to keep the worst case cheap on huge flat faces.
std::vector<vtkIdType> GrowCoplanarRegion(
  vtkPolyData* mesh, vtkIdType seedCell, double angleToleranceDeg, std::size_t maxTriangles)
{
  std::vector<vtkIdType> region;
  if (mesh->GetCellType(seedCell) != VTK_TRIANGLE)
  {
    return region;
  }
  mesh->BuildLinks(); // idempotent; required for GetCellNeighbors

  double seedNormal[3];
  TriangleNormal(mesh, seedCell, seedNormal);
  const double cosTol = std::cos(vtkMath::RadiansFromDegrees(angleToleranceDeg));

  std::set<vtkIdType> visited{ seedCell };
  std::queue<vtkIdType> frontier;
  frontier.push(seedCell);
  while (!frontier.empty() && region.size() < maxTriangles)
  {
    const vtkIdType current = frontier.front();
    frontier.pop();
    region.push_back(current);

    vtkNew<vtkIdList> cellPts;
    mesh->GetCellPoints(current, cellPts);
    if (cellPts->GetNumberOfIds() != 3)
    {
      continue;
    }
    for (int e = 0; e < 3; ++e)
    {
      vtkNew<vtkIdList> edge;
      edge->InsertNextId(cellPts->GetId(e));
      edge->InsertNextId(cellPts->GetId((e + 1) % 3));
      vtkNew<vtkIdList> neighbors;
      mesh->GetCellNeighbors(current, edge, neighbors);
      for (vtkIdType n = 0; n < neighbors->GetNumberOfIds(); ++n)
      {
        const vtkIdType nb = neighbors->GetId(n);
        if (visited.count(nb) != 0 || mesh->GetCellType(nb) != VTK_TRIANGLE)
        {
          continue;
        }
        double nbNormal[3];
        TriangleNormal(mesh, nb, nbNormal);
        if (vtkMath::Dot(seedNormal, nbNormal) >= cosTol)
        {
          visited.insert(nb);
          frontier.push(nb);
        }
      }
    }
  }
  return region;
}
```

- [ ] **Step 4: Add the `FACE` branch to `ResolvePickedObject`**

In `ResolvePickedObject`, the nearest-edge loop currently ends by unconditionally building an `EDGE`. Change the end of the function so the edge is only returned when the pick is within `snapTol` of it; otherwise resolve a `FACE`. Replace the final block — from where `bestEdge`/`bestEdgeDist2` are used to build the `EDGE` result — with:

```cpp
  // 2b. If the nearest edge is within snapTol, it is an EDGE pick.
  if (bestEdgeDist2 <= snapTol * snapTol)
  {
    double e0[3];
    double e1[3];
    pts->GetPoint(bestEdge, e0);
    pts->GetPoint((bestEdge + 1) % nbPts, e1);
    obj.ObjType = MeasureObject::Type::EDGE;
    obj.P0 = { e0[0], e0[1], e0[2] };
    obj.P1 = { e1[0], e1[1], e1[2] };
    return obj;
  }

  // 3. Otherwise it is a FACE: region-grow coplanar triangles around the
  //    picked triangle and measure at the area-weighted centroid.
  std::vector<vtkIdType> region;
  vtkPolyData* mesh = vtkPolyData::SafeDownCast(dataset);
  if (mesh != nullptr)
  {
    region = GrowCoplanarRegion(mesh, cellId, 15.0, 5000);
  }
  if (region.empty())
  {
    region.push_back(cellId); // fallback: the picked triangle alone
  }

  double weightedCentroid[3] = { 0.0, 0.0, 0.0 };
  double totalArea = 0.0;
  double vertexSum[3] = { 0.0, 0.0, 0.0 };
  int vertexCount = 0;
  obj.FacePoints.clear();
  for (const vtkIdType tri : region)
  {
    vtkNew<vtkIdList> triPts;
    dataset->GetCellPoints(tri, triPts);
    if (triPts->GetNumberOfIds() != 3)
    {
      continue;
    }
    double p[3][3];
    for (int k = 0; k < 3; ++k)
    {
      dataset->GetPoint(triPts->GetId(k), p[k]);
      obj.FacePoints.push_back({ p[k][0], p[k][1], p[k][2] });
      vertexSum[0] += p[k][0];
      vertexSum[1] += p[k][1];
      vertexSum[2] += p[k][2];
      ++vertexCount;
    }
    const double area = vtkTriangle::TriangleArea(p[0], p[1], p[2]);
    totalArea += area;
    for (int axis = 0; axis < 3; ++axis)
    {
      weightedCentroid[axis] += area * (p[0][axis] + p[1][axis] + p[2][axis]) / 3.0;
    }
  }

  obj.ObjType = MeasureObject::Type::FACE;
  if (totalArea > 0.0)
  {
    obj.P0 = { weightedCentroid[0] / totalArea, weightedCentroid[1] / totalArea,
      weightedCentroid[2] / totalArea };
  }
  else if (vertexCount > 0)
  {
    obj.P0 = { vertexSum[0] / vertexCount, vertexSum[1] / vertexCount,
      vertexSum[2] / vertexCount };
  }
  return obj;
}
```

`GrowCoplanarRegion` requires `dataset` to be a `vtkPolyData` for connectivity; the per-triangle vertex gathering uses `dataset` directly (`vtkDataSet::GetCellPoints` / `GetPoint` work for any dataset), so the non-polydata fallback (`region = {cellId}`) still produces a valid face.

- [ ] **Step 5: Run the test, verify it passes**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -10
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: PASS — point/edge checks plus all four new face checks (`resolve interior pick is a face`, `face centroid is the region centroid`, `face region stops at the sharp edge`, `face region covers both flat triangles`).

- [ ] **Step 6: Commit**

```bash
git add library/src/measurementTools.cxx library/testing/TestSDKMeasurement.cxx
git commit -m "feat: resolve interior picks to coplanar faces with centroid"
```

---

## Task 4: Render the face highlight

**Files:**
- Modify: `library/private/measurementManager.h`
- Modify: `library/src/measurementManager.cxx`

- [ ] **Step 1: Change the hover actor to a list and declare a highlight helper, in `measurementManager.h`**

Replace the member `vtkSmartPointer<vtkActor> HoverActor;` with:

```cpp
  std::vector<vtkSmartPointer<vtkActor>> HoverActors;
```

In the `private:` method section, after `UpdateHoverActor();`, add:

```cpp
  /**
   * Build a translucent highlight actor for a face region (triangle vertices,
   * three consecutive entries per triangle). Returns nullptr for an empty
   * region. The caller adds it to a renderer.
   */
  vtkSmartPointer<vtkActor> MakeFaceHighlightActor(
    const std::vector<std::array<double, 3>>& facePoints) const;
```

- [ ] **Step 2: Add includes and implement `MakeFaceHighlightActor` in `measurementManager.cxx`**

Add `#include <vtkCellArray.h>` and `#include <vtkPoints.h>` and `#include <vtkPolyData.h>` to the includes. Implement the helper (place it just before `UpdateActors`):

```cpp
//----------------------------------------------------------------------------
vtkSmartPointer<vtkActor> measurementManager::MakeFaceHighlightActor(
  const std::vector<std::array<double, 3>>& facePoints) const
{
  if (facePoints.size() < 3)
  {
    return nullptr;
  }
  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> triangles;
  for (std::size_t i = 0; i + 2 < facePoints.size(); i += 3)
  {
    vtkIdType ids[3];
    for (int k = 0; k < 3; ++k)
    {
      ids[k] = points->InsertNextPoint(facePoints[i + k].data());
    }
    triangles->InsertNextCell(3, ids);
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetPolys(triangles);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1.0, 0.85, 0.0); // same yellow as selection markers
  actor->GetProperty()->SetOpacity(0.35);
  actor->GetProperty()->LightingOff();
  actor->PickableOff();
  return actor;
}
```

- [ ] **Step 3: Handle `FACE` in `UpdateActors`**

In `UpdateActors`, the loop over `this->Selection` currently handles `POINT` (sphere) and `EDGE` (line). Replace that loop with one that also handles `FACE`:

```cpp
  // Markers for each selected object.
  for (const MeasureObject& obj : this->Selection)
  {
    if (obj.ObjType == MeasureObject::Type::POINT)
    {
      addSphere(obj.P0);
    }
    else if (obj.ObjType == MeasureObject::Type::EDGE)
    {
      addLine(obj.P0, obj.P1, 1.0, 0.85, 0.0, 4.0); // highlight the selected edge
    }
    else // FACE
    {
      vtkSmartPointer<vtkActor> highlight = this->MakeFaceHighlightActor(obj.FacePoints);
      if (highlight != nullptr)
      {
        overlay->AddActor(highlight);
        this->Actors.emplace_back(highlight);
      }
      addSphere(obj.P0); // centroid marker
    }
  }
```

(`overlay` is the local `vtkRenderer*` already obtained earlier in `UpdateActors`; `addSphere`/`addLine` are the existing local lambdas.)

- [ ] **Step 4: Rewrite `UpdateHoverActor` for the actor list and `FACE`**

Replace the whole body of `UpdateHoverActor` with:

```cpp
void measurementManager::UpdateHoverActor()
{
  vtkF3DRenderer* mainRenderer = this->Window.GetRenderer();
  vtkRenderer* overlay = this->GetOverlayRenderer();
  if (mainRenderer == nullptr || overlay == nullptr)
  {
    return;
  }

  for (const auto& actor : this->HoverActors)
  {
    overlay->RemoveActor(actor);
  }
  this->HoverActors.clear();

  if (!this->HoverObject.has_value())
  {
    return;
  }

  // Estimate a marker size from the scene's visible bounds.
  double bounds[6];
  mainRenderer->ComputeVisiblePropBounds(bounds);
  const double diag = std::sqrt((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
    (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
    (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
  const double markerRadius = (diag > 0.0 ? diag : 1.0) * 0.012;

  const MeasureObject& obj = this->HoverObject.value();

  // Light-cyan preview, distinct from the yellow committed-selection markers.
  const auto addHoverActor = [&](vtkSmartPointer<vtkActor> actor)
  {
    actor->GetProperty()->SetColor(0.4, 0.9, 1.0);
    actor->GetProperty()->LightingOff();
    actor->PickableOff();
    overlay->AddActor(actor);
    this->HoverActors.emplace_back(actor);
  };

  if (obj.ObjType == MeasureObject::Type::FACE)
  {
    vtkSmartPointer<vtkActor> highlight = this->MakeFaceHighlightActor(obj.FacePoints);
    if (highlight != nullptr)
    {
      highlight->GetProperty()->SetColor(0.4, 0.9, 1.0);
      addHoverActor(highlight);
    }
  }

  vtkNew<vtkPolyDataMapper> mapper;
  if (obj.ObjType == MeasureObject::Type::EDGE)
  {
    vtkNew<vtkLineSource> line;
    line->SetPoint1(obj.P0[0], obj.P0[1], obj.P0[2]);
    line->SetPoint2(obj.P1[0], obj.P1[1], obj.P1[2]);
    mapper->SetInputConnection(line->GetOutputPort());
  }
  else // POINT or FACE: a sphere at P0 (the vertex or the face centroid)
  {
    vtkNew<vtkSphereSource> sphere;
    sphere->SetCenter(obj.P0[0], obj.P0[1], obj.P0[2]);
    sphere->SetRadius(markerRadius);
    mapper->SetInputConnection(sphere->GetOutputPort());
  }
  vtkSmartPointer<vtkActor> marker = vtkSmartPointer<vtkActor>::New();
  marker->SetMapper(mapper);
  marker->GetProperty()->SetLineWidth(3.0);
  addHoverActor(marker);
}
```

Note: `MakeFaceHighlightActor` sets the actor color to yellow; the hover code overrides it to cyan via `SetColor` before adding, so the hover highlight is cyan and the committed-selection highlight stays yellow.

- [ ] **Step 5: Build to verify it compiles**

Run: `cmake --build build 2>&1 | tail -15`
Expected: clean build of `libf3d`, `vtkext`, and `f3d`.

- [ ] **Step 6: Optional smoke check (skip if headless)**

If a display is available: `./build/bin/f3d testing/data/cow.vtp`, press `Shift+M`, click in the interior of a triangle — expect a translucent yellow face highlight plus a centroid marker. If headless, skip and say so.

- [ ] **Step 7: Commit**

```bash
git add library/private/measurementManager.h library/src/measurementManager.cxx
git commit -m "feat: render translucent highlight for selected and hovered faces"
```

---

## Task 5: Documentation and interaction test

**Files:**
- Modify: `doc/user/04-INTERACTIONS.md`
- Modify: `doc/CHANGELOG.md`
- Modify: `library/testing/TestSDKMeasurementInteraction.cxx`

- [ ] **Step 1: Update the "Measurement Mode" section in `doc/user/04-INTERACTIONS.md`**

In the existing `## Measurement Mode` section, find the sentence describing what a click selects (it mentions selecting a mesh vertex or edge). Add, as a new sentence or paragraph right after it:

```markdown
Clicking in the interior of a triangle (away from any vertex or edge) selects
the whole coplanar **face** around it; the face's centroid is used as the
measured point and the detected region is highlighted.
```

- [ ] **Step 2: Add a `CHANGELOG.md` entry**

In `doc/CHANGELOG.md`, under the topmost "For F3D users" section, add a bullet matching the existing style:

```markdown
- Measurement mode can now select a whole coplanar face by clicking its interior, measuring from the face centroid.
```

- [ ] **Step 3: Extend `TestSDKMeasurementInteraction.cxx`**

In `library/testing/TestSDKMeasurementInteraction.cxx`, after the existing click sequence and before the final `return`, add a click intended to land in a triangle interior (a face pick), then a render:

```cpp
  // A click toward the centre of the model is likely to land in a triangle
  // interior and resolve to a face; it must not crash.
  inter.triggerMousePosition(150, 150);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::PRESS, f3d::interactor::MouseButton::LEFT);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::RELEASE, f3d::interactor::MouseButton::LEFT);
  eng.getWindow().render();
```

Match the exact local variable names (`inter`, `eng`) and the `triggerMousePosition` / `triggerMouseButton` API already used earlier in the same file; adapt if they differ.

- [ ] **Step 4: Build and run the measurement tests**

Run:
```bash
cmake --build build 2>&1 | tail -15
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V 2>&1 | tail -20
```
Expected: clean build; `TestSDKMeasurement` passes; `TestSDKMeasurementInteraction` passes (or is reported `Disabled`/`Not Run` if rendering tests are off — acceptable; report which).

- [ ] **Step 5: Commit**

```bash
git add doc/user/04-INTERACTIONS.md doc/CHANGELOG.md library/testing/TestSDKMeasurementInteraction.cxx
git commit -m "docs: document face picking; extend interaction test"
```

---

## Task 6: Full verification

**Files:** none (verification only)

- [ ] **Step 1: Clean build**

Run: `cmake --build build 2>&1 | tail -20`
Expected: builds with no errors or new warnings related to the measurement code.

- [ ] **Step 2: Run the measurement tests**

Run: `ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V 2>&1 | tail -20`
Expected: `TestSDKMeasurement` (point/edge/face/region-stop checks) and `TestSDKMeasurementInteraction` pass.

- [ ] **Step 3: Run the full libf3d suite for regressions**

Run: `ctest --test-dir build -L libf3d 2>&1 | tail -25`
Expected: no new failures. `TestSDKEngineRecreation` may intermittently time out in this environment as a pre-existing issue unrelated to this work — it is not a regression.

- [ ] **Step 4: Manual end-to-end check (skip if headless)**

Run `./build/bin/f3d testing/data/cow.vtp`: enter measurement mode, click a triangle interior and confirm a face highlight + centroid marker appear; confirm hovering over a face previews it; confirm face-to-face / face-to-point measurements report a distance; confirm clicking near a vertex or edge still gives point/edge.

- [ ] **Step 5: Final commit if any fixes were needed**

```bash
git add -A
git commit -m "fix: address face-detection verification findings"
```

---

## Notes for the Implementer

- **Verification discipline:** run each step's command and confirm the expected output before marking it done. Use `superpowers:verification-before-completion` before claiming the feature complete.
- **`vtkDataSet::GetCell` aliasing:** `GetCell` returns a pointer to an internal shared cell that is invalidated by the next `GetCell` call. The code paths here either fully consume the cell before any further `GetCell` call, or use `GetCellPoints`/`GetPoint`/`GetCellBounds`/`GetCellType` (which do not alias). Do not hold a `vtkCell*` across another `GetCell`.
- **Out of scope:** circle/hole detection, curved-surface faces, a configurable coplanarity tolerance. Do not implement these.
