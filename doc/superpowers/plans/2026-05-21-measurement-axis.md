# Axis-Constrained Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an F3D measurement be constrained to a model axis (X/Y/Z) — showing the per-axis breakdown and a right-angle staircase annotation — instead of always reporting the straight-line distance.

**Architecture:** Extends the existing measurement feature. A new `ui.measurement.axis` option drives an axis-aware result string, a per-axis breakdown string, and a staircase 3D annotation. A pure `ComputeAxisPath` helper (unit-tested) produces the staircase points. A `set_measurement_axis` interactor command + a radio row in the ImGui panel let the user pick the axis.

**Tech Stack:** C++17, VTK (vtkLineSource/vtkActor), Dear ImGui, CMake, F3D options codegen.

**Spec:** `doc/superpowers/specs/2026-05-21-measurement-axis-design.md`

---

## Background for the implementer

The measurement feature already exists. Key current state:

- `library/private/measurementTools.h` / `library/src/measurementTools.cxx` — pure helpers (`UnitToMeters`, `ComputeDistance`, `ResolvePickedObject`) and the `MeasureObject` / `MeasureResult` structs. `MeasureResult` has `Distance`, `ClosestA`, `ClosestB` (each `std::array<double,3>`). `measurementTools.cxx` is compiled into both `libf3d` and the `libf3dSDKTests` test binary.
- `library/private/measurementManager.h` / `library/src/measurementManager.cxx` — the `measurementManager` class. Relevant members: `options& Options`, `window_impl& Window`, `std::vector<MeasureObject> Selection`, `std::optional<MeasureResult> Result`. Public methods include `GetResultString()`, `RefreshPanel()`. Private `UpdateActors()` rebuilds the 3D actors and calls `RefreshPanel()` at its end.
- `vtkext/private/module/vtkF3DRenderer.{h,cxx}` — has `ConfigureMeasurement(bool visible, const std::string& text, const std::string& modelUnit, const std::string& displayUnit)` which forwards to the UI actor.
- `vtkext/private/module/vtkF3DUIActor.{h,cxx}` — base UI actor; has `SetMeasurementVisibility/SetMeasurement/SetMeasurementUnits` and protected members `MeasurementVisible`, `Measurement`, `MeasurementModelUnit`, `MeasurementDisplayUnit`, and a `virtual void RenderMeasurement()`.
- `vtkext/private/module/vtkF3DImguiActor.{h,cxx}` — `RenderMeasurement()` draws the panel; private `EmitMeasurementUnitChange(which, value)` emits the `set_measurement_unit` command.
- `library/src/interactor_impl.cxx` — has the `set_measurement_unit` command (sets the option via `Options.setAsString`, calls `MeasurementManager.RefreshPanel()`, `requestRender()`).

**Important encoding note:** use ASCII labels only (`X:`, `Y:`, `Z:`) — do NOT use the `Δ` character; the bundled ImGui font may lack that glyph.

---

## File Structure

No new files. Each change extends an existing unit:

- `library/options.json`, `resources/cli-options.json`, `application/F3DOptionsTools.h`, `doc/user/03-OPTIONS.md` — the new `ui.measurement.axis` option.
- `library/private/measurementTools.h` / `library/src/measurementTools.cxx` — `ComputeAxisPath` pure helper.
- `library/private/measurementManager.h` / `library/src/measurementManager.cxx` — axis-aware result string, components string, staircase actors, conversion helpers.
- `vtkext/private/module/vtkF3DRenderer.{h,cxx}`, `vtkF3DUIActor.{h,cxx}`, `vtkF3DImguiActor.{h,cxx}` — panel plumbing, breakdown line, axis radio row.
- `library/src/interactor_impl.cxx` — the `set_measurement_axis` command.
- `library/testing/TestSDKMeasurement.cxx`, `library/testing/TestSDKMeasurementInteraction.cxx` — tests.
- `doc/user/04-INTERACTIONS.md`, `doc/CHANGELOG.md` — docs.

---

## Task 1: Add the `ui.measurement.axis` option

**Files:**
- Modify: `library/options.json`
- Modify: `resources/cli-options.json`
- Modify: `application/F3DOptionsTools.h`
- Modify: `doc/user/03-OPTIONS.md`

- [ ] **Step 1: Add the option to `library/options.json`**

Find the existing `"measurement"` block inside `"ui"` (it contains `model_unit` and `display_unit`). Add an `axis` entry so the block reads:

```json
    "measurement": {
      "model_unit": {
        "type": "string",
        "default_value": "mm"
      },
      "display_unit": {
        "type": "string",
        "default_value": "mm"
      },
      "axis": {
        "type": "string",
        "default_value": "free"
      }
    },
```

- [ ] **Step 2: Add the CLI flag to `resources/cli-options.json`**

In the group object whose `options` array contains `"longName": "measurement-model-unit"`, add, adjacent to the other two measurement flags:

```json
{ "longName": "measurement-axis", "helpText": "Measurement axis constraint (free/x/y/z)", "valueHelper": "<axis>" }
```

Match the file's indentation/quoting. Validate: `python3 -m json.tool resources/cli-options.json > /dev/null && echo JSON_OK`.

- [ ] **Step 3: Add the `LibOptionsNames` mapping in `application/F3DOptionsTools.h`**

In the `LibOptionsNames` map, next to the existing `{ "measurement-model-unit", "ui.measurement.model_unit" },` and `{ "measurement-display-unit", ... }` entries, add:

```cpp
  { "measurement-axis", "ui.measurement.axis" },
```

- [ ] **Step 4: Document it in `doc/user/03-OPTIONS.md`**

After the `--measurement-display-unit` entry, add (match the surrounding heading format):

```
### `--measurement-axis=<axis>` (_string_, default: `free`)

Constrain measurement-mode results to a single model axis. One of `free` (straight-line distance), `x`, `y`, or `z`.
```

- [ ] **Step 5: Build to verify codegen + CLI wiring**

Run:
```bash
cmake --build build --target f3d 2>&1 | tail -15
./build/bin/f3d --help 2>&1 | grep -i measurement-axis
```
Expected: clean build; `--help` lists `--measurement-axis`.

- [ ] **Step 6: Commit**

```bash
git add library/options.json resources/cli-options.json application/F3DOptionsTools.h doc/user/03-OPTIONS.md
git commit -m "feat: add ui.measurement.axis option"
```

---

## Task 2: `ComputeAxisPath` pure helper (TDD)

**Files:**
- Modify: `library/private/measurementTools.h`
- Modify: `library/src/measurementTools.cxx`
- Modify: `library/testing/TestSDKMeasurement.cxx`

- [ ] **Step 1: Declare `ComputeAxisPath` in `measurementTools.h`**

After the `ResolvePickedObject` declaration, inside `namespace f3d { namespace detail {`, add:

```cpp
/**
 * Return the 4 points of the right-angle "staircase" path from a to b for the
 * given axis (0=X, 1=Y, 2=Z). Point [0] is a and point [3] is b. The leg
 * [0]->[1] is the selected-axis leg (drawn first from a); the other two legs
 * follow in ascending axis order. The three legs sum from a to b.
 */
std::array<std::array<double, 3>, 4> ComputeAxisPath(
  const std::array<double, 3>& a, const std::array<double, 3>& b, int axis);
```

- [ ] **Step 2: Add failing tests to `TestSDKMeasurement.cxx`**

Insert before the final `return test.result();`:

```cpp
  // --- ComputeAxisPath ---
  {
    const std::array<double, 3> a{ 0.0, 0.0, 0.0 };
    const std::array<double, 3> b{ 3.0, 4.0, 5.0 };

    for (int axis = 0; axis < 3; ++axis)
    {
      const auto path = f3d::detail::ComputeAxisPath(a, b, axis);
      const std::string tag = "ComputeAxisPath axis " + std::to_string(axis);

      // Endpoints.
      test(tag + " starts at a", path[0] == approx(a));
      test(tag + " ends at b", path[3] == approx(b));

      // The first leg lies purely along the selected axis with length |b-a|.
      std::array<double, 3> leg0{ path[1][0] - path[0][0], path[1][1] - path[0][1],
        path[1][2] - path[0][2] };
      double expectedLen = std::fabs(b[axis] - a[axis]);
      bool alongAxis = true;
      double legLen = 0.0;
      for (int i = 0; i < 3; ++i)
      {
        if (i == axis)
        {
          legLen = std::fabs(leg0[i]);
        }
        else if (std::fabs(leg0[i]) > 1e-9)
        {
          alongAxis = false;
        }
      }
      test(tag + " first leg along axis", alongAxis);
      test(tag + " first leg length", legLen == approx(expectedLen));

      // The three legs sum from a to b.
      std::array<double, 3> sum{ 0.0, 0.0, 0.0 };
      for (int k = 0; k < 3; ++k)
      {
        for (int i = 0; i < 3; ++i)
        {
          sum[i] += path[k + 1][i] - path[k][i];
        }
      }
      std::array<double, 3> delta{ b[0] - a[0], b[1] - a[1], b[2] - a[2] };
      test(tag + " legs sum to b-a", sum == approx(delta));
    }
  }
```

Add `#include <string>` to the test file's includes if it is not already present.

- [ ] **Step 3: Run the test, verify it fails**

Run: `cmake --build build --target libf3dSDKTests 2>&1 | tail -15`
Expected: FAIL — link error, `ComputeAxisPath` is undefined.

- [ ] **Step 4: Implement `ComputeAxisPath` in `measurementTools.cxx`**

Add inside the `namespace f3d::detail` block (alongside the other functions):

```cpp
//----------------------------------------------------------------------------
std::array<std::array<double, 3>, 4> ComputeAxisPath(
  const std::array<double, 3>& a, const std::array<double, 3>& b, int axis)
{
  // Each leg moves along a single model axis by that axis' delta.
  std::array<std::array<double, 3>, 3> legs{};
  for (int i = 0; i < 3; ++i)
  {
    legs[i] = { 0.0, 0.0, 0.0 };
    legs[i][i] = b[i] - a[i];
  }

  // Draw the selected axis leg first, then the remaining two in ascending order.
  std::array<int, 3> order{ axis, 0, 0 };
  int n = 1;
  for (int i = 0; i < 3; ++i)
  {
    if (i != axis)
    {
      order[n++] = i;
    }
  }

  std::array<std::array<double, 3>, 4> path{};
  path[0] = a;
  for (int k = 0; k < 3; ++k)
  {
    const std::array<double, 3>& leg = legs[order[k]];
    path[k + 1] = { path[k][0] + leg[0], path[k][1] + leg[1], path[k][2] + leg[2] };
  }
  return path;
}
```

- [ ] **Step 5: Run the test, verify it passes**

Run:
```bash
cmake --build build --target libf3dSDKTests 2>&1 | tail -10
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V
```
Expected: PASS — all prior checks plus the new `ComputeAxisPath` checks.

- [ ] **Step 6: Commit**

```bash
git add library/private/measurementTools.h library/src/measurementTools.cxx library/testing/TestSDKMeasurement.cxx
git commit -m "feat: add ComputeAxisPath staircase helper"
```

---

## Task 3: Axis-aware result string + per-axis breakdown

**Files:**
- Modify: `library/private/measurementManager.h`
- Modify: `library/src/measurementManager.cxx`

- [ ] **Step 1: Declare the new methods in `measurementManager.h`**

In the `public:` section, after `GetResultString()`, add:

```cpp
  /**
   * The per-axis breakdown ("X: .. Y: .. Z: .. <unit>"), converted to display
   * units. Empty until two objects are selected.
   */
  std::string GetComponentsString() const;
```

In the `private:` section (near the other private methods), add:

```cpp
  /**
   * Convert a model-space length to display units (identity when either unit
   * is unset/unknown).
   */
  double ConvertToDisplay(double modelValue) const;

  /**
   * The display-unit suffix (" mm" etc.), or empty when no conversion applies.
   */
  std::string DisplaySuffix() const;
```

- [ ] **Step 2: Implement the conversion helpers in `measurementManager.cxx`**

Add (inside `namespace f3d::detail`, e.g. just before `GetResultString`):

```cpp
//----------------------------------------------------------------------------
double measurementManager::ConvertToDisplay(double modelValue) const
{
  const std::optional<double> modelF = UnitToMeters(this->Options.ui.measurement.model_unit);
  const std::optional<double> displayF = UnitToMeters(this->Options.ui.measurement.display_unit);
  if (modelF.has_value() && displayF.has_value())
  {
    return modelValue * modelF.value() / displayF.value();
  }
  return modelValue;
}

//----------------------------------------------------------------------------
std::string measurementManager::DisplaySuffix() const
{
  const std::optional<double> modelF = UnitToMeters(this->Options.ui.measurement.model_unit);
  const std::optional<double> displayF = UnitToMeters(this->Options.ui.measurement.display_unit);
  if (modelF.has_value() && displayF.has_value())
  {
    return " " + this->Options.ui.measurement.display_unit;
  }
  return "";
}
```

- [ ] **Step 3: Rewrite `GetResultString` to be axis-aware**

Replace the current body of `measurementManager::GetResultString()` with:

```cpp
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

  const std::array<double, 3>& a = this->Result->ClosestA;
  const std::array<double, 3>& b = this->Result->ClosestB;
  const std::string& axis = this->Options.ui.measurement.axis;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  if (axis == "x")
  {
    oss << "X: " << this->ConvertToDisplay(std::fabs(b[0] - a[0])) << this->DisplaySuffix();
  }
  else if (axis == "y")
  {
    oss << "Y: " << this->ConvertToDisplay(std::fabs(b[1] - a[1])) << this->DisplaySuffix();
  }
  else if (axis == "z")
  {
    oss << "Z: " << this->ConvertToDisplay(std::fabs(b[2] - a[2])) << this->DisplaySuffix();
  }
  else
  {
    oss << "Distance: " << this->ConvertToDisplay(this->Result->Distance) << this->DisplaySuffix();
  }
  return oss.str();
}
```

- [ ] **Step 4: Implement `GetComponentsString`**

Add (after `GetResultString`):

```cpp
//----------------------------------------------------------------------------
std::string measurementManager::GetComponentsString() const
{
  if (this->Selection.size() < 2 || !this->Result.has_value())
  {
    return "";
  }
  const std::array<double, 3>& a = this->Result->ClosestA;
  const std::array<double, 3>& b = this->Result->ClosestB;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "X: " << this->ConvertToDisplay(std::fabs(b[0] - a[0])) << "   "
      << "Y: " << this->ConvertToDisplay(std::fabs(b[1] - a[1])) << "   "
      << "Z: " << this->ConvertToDisplay(std::fabs(b[2] - a[2])) << this->DisplaySuffix();
  return oss.str();
}
```

`<cmath>` (for `std::fabs`) and `<sstream>`/`<iomanip>` are already included in `measurementManager.cxx`.

- [ ] **Step 5: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: clean build. (`GetComponentsString` is not wired into the panel yet — that is Task 5. The build still succeeds.)

- [ ] **Step 6: Commit**

```bash
git add library/private/measurementManager.h library/src/measurementManager.cxx
git commit -m "feat: axis-aware measurement result string and per-axis breakdown"
```

---

## Task 4: Staircase 3D annotation

**Files:**
- Modify: `library/private/measurementManager.h`
- Modify: `library/src/measurementManager.cxx`

- [ ] **Step 1: Declare `RefreshMeasurement` in `measurementManager.h`**

In the `public:` section, after `RefreshPanel()`, add:

```cpp
  /**
   * Rebuild the 3D annotation and refresh the panel. Call after the axis
   * option changes, since the axis affects the drawn geometry.
   */
  void RefreshMeasurement();
```

- [ ] **Step 2: Implement `RefreshMeasurement` in `measurementManager.cxx`**

Add (e.g. just after `RefreshPanel`):

```cpp
//----------------------------------------------------------------------------
void measurementManager::RefreshMeasurement()
{
  this->UpdateActors();
}
```

(`UpdateActors` already calls `RefreshPanel` at its end.)

- [ ] **Step 3: Replace the connecting-line drawing in `UpdateActors` with axis-aware drawing**

In `measurementManager::UpdateActors()`, find the current block:

```cpp
  // Connecting line between the two closest points.
  if (this->Result.has_value())
  {
    addLine(this->Result->ClosestA, this->Result->ClosestB, 0.1, 0.8, 1.0, 2.0);
  }
```

Replace it with:

```cpp
  // Connecting annotation between the two closest points.
  if (this->Result.has_value())
  {
    const std::string& axis = this->Options.ui.measurement.axis;
    const int axisIdx = (axis == "x") ? 0 : (axis == "y") ? 1 : (axis == "z") ? 2 : -1;
    if (axisIdx < 0)
    {
      // Free mode: straight line.
      addLine(this->Result->ClosestA, this->Result->ClosestB, 0.1, 0.8, 1.0, 2.0);
    }
    else
    {
      // Axis mode: right-angle staircase, selected leg emphasized.
      const std::array<std::array<double, 3>, 4> path =
        ComputeAxisPath(this->Result->ClosestA, this->Result->ClosestB, axisIdx);
      addLine(path[0], path[1], 0.1, 0.8, 1.0, 4.0); // selected leg: bright cyan, thick
      addLine(path[1], path[2], 0.5, 0.5, 0.5, 1.5); // helper leg: dim grey
      addLine(path[2], path[3], 0.5, 0.5, 0.5, 1.5); // helper leg: dim grey
    }
  }
```

(`ComputeAxisPath` is declared in `measurementTools.h`, included transitively via `measurementManager.h`. The `addLine` lambda already exists in `UpdateActors` with signature `(a, b, r, g, bl, width)`.)

- [ ] **Step 4: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add library/private/measurementManager.h library/src/measurementManager.cxx
git commit -m "feat: draw right-angle staircase annotation for axis measurement"
```

---

## Task 5: Panel plumbing — pass components + axis to the UI actor

**Files:**
- Modify: `vtkext/private/module/vtkF3DUIActor.h`
- Modify: `vtkext/private/module/vtkF3DUIActor.cxx`
- Modify: `vtkext/private/module/vtkF3DRenderer.h`
- Modify: `vtkext/private/module/vtkF3DRenderer.cxx`
- Modify: `library/src/measurementManager.cxx`

- [ ] **Step 1: Add setters + members to `vtkF3DUIActor.h`**

After the `SetMeasurementUnits` declaration, add:

```cpp
  /**
   * Set the measurement per-axis breakdown string
   * Empty by default
   */
  void SetMeasurementComponents(const std::string& components);

  /**
   * Set the measurement axis (free/x/y/z)
   * "free" by default
   */
  void SetMeasurementAxis(const std::string& axis);
```

In the protected members section, after `MeasurementDisplayUnit`, add:

```cpp
  std::string MeasurementComponents = "";
  std::string MeasurementAxis = "free";
```

- [ ] **Step 2: Implement the setters in `vtkF3DUIActor.cxx`**

Next to `SetMeasurementUnits`, add:

```cpp
//----------------------------------------------------------------------------
void vtkF3DUIActor::SetMeasurementComponents(const std::string& components)
{
  this->MeasurementComponents = components;
}

//----------------------------------------------------------------------------
void vtkF3DUIActor::SetMeasurementAxis(const std::string& axis)
{
  this->MeasurementAxis = axis;
}
```

- [ ] **Step 3: Extend `ConfigureMeasurement` in `vtkF3DRenderer.h`**

Replace the `ConfigureMeasurement` declaration with:

```cpp
  /**
   * Configure the measurement panel UI from the given state.
   */
  void ConfigureMeasurement(bool visible, const std::string& text,
    const std::string& components, const std::string& axis, const std::string& modelUnit,
    const std::string& displayUnit);
```

- [ ] **Step 4: Extend `ConfigureMeasurement` in `vtkF3DRenderer.cxx`**

Replace the `ConfigureMeasurement` definition with:

```cpp
//----------------------------------------------------------------------------
void vtkF3DRenderer::ConfigureMeasurement(bool visible, const std::string& text,
  const std::string& components, const std::string& axis, const std::string& modelUnit,
  const std::string& displayUnit)
{
  this->UIActor->SetMeasurementVisibility(visible);
  this->UIActor->SetMeasurement(text);
  this->UIActor->SetMeasurementComponents(components);
  this->UIActor->SetMeasurementAxis(axis);
  this->UIActor->SetMeasurementUnits(modelUnit, displayUnit);
}
```

- [ ] **Step 5: Update `measurementManager::RefreshPanel` to pass the new arguments**

In `measurementManager.cxx`, replace the body of `RefreshPanel` with:

```cpp
void measurementManager::RefreshPanel()
{
  vtkF3DRenderer* renderer = this->Window.GetRenderer();
  if (renderer != nullptr)
  {
    renderer->ConfigureMeasurement(this->IsPanelVisible(), this->GetResultString(),
      this->GetComponentsString(), this->Options.ui.measurement.axis,
      this->Options.ui.measurement.model_unit, this->Options.ui.measurement.display_unit);
  }
}
```

- [ ] **Step 6: Build to verify it compiles**

Run: `cmake --build build 2>&1 | tail -15`
Expected: clean build of `vtkext` modules, `libf3d`, and `f3d`.

- [ ] **Step 7: Commit**

```bash
git add vtkext/private/module/vtkF3DUIActor.h vtkext/private/module/vtkF3DUIActor.cxx vtkext/private/module/vtkF3DRenderer.h vtkext/private/module/vtkF3DRenderer.cxx library/src/measurementManager.cxx
git commit -m "feat: pass measurement components and axis to the UI actor"
```

---

## Task 6: The `set_measurement_axis` command

**Files:**
- Modify: `library/src/interactor_impl.cxx`

- [ ] **Step 1: Register the command**

In `interactor_impl::initCommands()`, find the existing `set_measurement_unit` command registration. Immediately after it, add:

```cpp
  this->addCommand(
    "set_measurement_axis",
    [&](const std::vector<std::string>& args)
    {
      // args: <free|x|y|z>
      if (args.empty())
      {
        return;
      }
      this->Internals->Options.setAsString("ui.measurement.axis", args[0]);
      // The axis affects the drawn geometry, so rebuild actors, not just the panel.
      this->Internals->MeasurementManager.RefreshMeasurement();
      this->requestRender();
    },
    command_documentation_t{ "set_measurement_axis",
      "set the measurement axis (free|x|y|z) and refresh the measurement" });
```

- [ ] **Step 2: Build to verify it compiles**

Run: `cmake --build build --target libf3d 2>&1 | tail -15`
Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add library/src/interactor_impl.cxx
git commit -m "feat: add set_measurement_axis command"
```

---

## Task 7: Axis radio row + breakdown line in the panel

**Files:**
- Modify: `vtkext/private/module/vtkF3DImguiActor.h`
- Modify: `vtkext/private/module/vtkF3DImguiActor.cxx`

- [ ] **Step 1: Declare `EmitMeasurementAxisChange` in `vtkF3DImguiActor.h`**

After the `EmitMeasurementUnitChange` declaration, add:

```cpp
  /**
   * Emit a `set_measurement_axis` command. `axis` is "free", "x", "y" or "z".
   */
  void EmitMeasurementAxisChange(const std::string& axis);
```

- [ ] **Step 2: Implement `EmitMeasurementAxisChange` in `vtkF3DImguiActor.cxx`**

Next to `EmitMeasurementUnitChange`, add:

```cpp
//----------------------------------------------------------------------------
void vtkF3DImguiActor::EmitMeasurementAxisChange(const std::string& axis)
{
  std::string command = "set_measurement_axis " + axis;
  vtkOutputWindow::GetInstance()->InvokeEvent(
    vtkF3DUserEvents::TriggerEvent, const_cast<char*>(command.c_str()));
}
```

- [ ] **Step 3: Add the breakdown line and axis radio row in `RenderMeasurement`**

In `vtkF3DImguiActor::RenderMeasurement()`, find the line that renders the primary text:

```cpp
  ImGui::Begin("Measurement", nullptr, flags);
  ImGui::TextUnformatted(this->Measurement.c_str());
  ImGui::SeparatorText("Units");
```

Replace those three lines with:

```cpp
  ImGui::Begin("Measurement", nullptr, flags);
  ImGui::TextUnformatted(this->Measurement.c_str());

  if (!this->MeasurementComponents.empty())
  {
    ImGui::TextUnformatted(this->MeasurementComponents.c_str());
  }

  ImGui::SeparatorText("Axis");
  {
    static const char* axisNames[] = { "free", "x", "y", "z" };
    int axisIdx = 0;
    for (int i = 0; i < IM_ARRAYSIZE(axisNames); ++i)
    {
      if (this->MeasurementAxis == axisNames[i])
      {
        axisIdx = i;
      }
    }
    const int previous = axisIdx;
    ImGui::RadioButton("Free", &axisIdx, 0);
    ImGui::SameLine();
    ImGui::RadioButton("X", &axisIdx, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Y", &axisIdx, 2);
    ImGui::SameLine();
    ImGui::RadioButton("Z", &axisIdx, 3);
    if (axisIdx != previous)
    {
      this->EmitMeasurementAxisChange(axisNames[axisIdx]);
    }
  }

  ImGui::SeparatorText("Units");
```

(The rest of `RenderMeasurement` — the unit `unitRow` calls and `ImGui::End()` — is unchanged.)

- [ ] **Step 4: Build to verify it compiles**

Run: `cmake --build build 2>&1 | tail -15`
Expected: clean build of `vtkext` modules, `libf3d`, and `f3d`.

- [ ] **Step 5: Manual smoke check (skip if headless)**

Run `./build/bin/f3d testing/data/cow.vtp`, press `Shift+M`, click two points. Expect the panel to show the `X:/Y:/Z:` breakdown and a `Free/X/Y/Z` radio row. Selecting `X` should switch the primary line to `X: …` and redraw the 3D annotation as a staircase. If headless, skip and say so.

- [ ] **Step 6: Commit**

```bash
git add vtkext/private/module/vtkF3DImguiActor.h vtkext/private/module/vtkF3DImguiActor.cxx
git commit -m "feat: add axis radio row and breakdown line to measurement panel"
```

---

## Task 8: Documentation and tests

**Files:**
- Modify: `doc/user/04-INTERACTIONS.md`
- Modify: `doc/CHANGELOG.md`
- Modify: `library/testing/TestSDKMeasurementInteraction.cxx`

- [ ] **Step 1: Update the "Measurement Mode" section in `doc/user/04-INTERACTIONS.md`**

In the existing "## Measurement Mode" section, after the paragraph describing the units combo boxes, add a new paragraph:

```markdown
The panel also shows a per-axis breakdown (X, Y, Z) of the measurement and an
**Axis** selector. With `Free` selected the straight-line distance is reported;
selecting `X`, `Y` or `Z` reports the distance along that model axis and draws
a right-angle path showing how the straight-line distance decomposes. The axis
maps to the `ui.measurement.axis` option.
```

- [ ] **Step 2: Add a `CHANGELOG.md` entry**

In `doc/CHANGELOG.md`, under the current top "For F3D users" section, add a bullet matching the file's style:

```markdown
- Measurement mode can now be constrained to a model axis (X/Y/Z), showing the per-axis breakdown and a right-angle annotation.
```

- [ ] **Step 3: Extend `TestSDKMeasurementInteraction.cxx`**

In `library/testing/TestSDKMeasurementInteraction.cxx`, after the existing two-click sequence and before the final `return`, add an axis exercise and an option round-trip check:

```cpp
  // Exercise the axis command and verify the option round-trips.
  inter.triggerCommand("set_measurement_axis x");
  eng.getWindow().render();
  if (eng.getOptions().getAsString("ui.measurement.axis") != "x")
  {
    std::cerr << "ui.measurement.axis was not set to x" << std::endl;
    return EXIT_FAILURE;
  }

  inter.triggerCommand("set_measurement_axis free");
  eng.getWindow().render();
```

Ensure `#include <options.h>` and `#include <iostream>` are present in the test file's includes (add whichever is missing). Verify the exact accessor names against `library/public/options.h.in` / `library/public/engine.h` — `engine::getOptions()` returns `f3d::options&`; `options::getAsString(const std::string&)` returns the option's string value. Adapt if the real signatures differ.

- [ ] **Step 4: Build and run the measurement tests**

Run:
```bash
cmake --build build 2>&1 | tail -15
ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V 2>&1 | tail -20
```
Expected: clean build; `TestSDKMeasurement` passes; `TestSDKMeasurementInteraction` passes (or is reported DISABLED if rendering tests are off — acceptable, report which).

- [ ] **Step 5: Commit**

```bash
git add doc/user/04-INTERACTIONS.md doc/CHANGELOG.md library/testing/TestSDKMeasurementInteraction.cxx
git commit -m "docs: document axis measurement; extend interaction test"
```

---

## Task 9: Full verification

**Files:** none (verification only)

- [ ] **Step 1: Clean build**

Run: `cmake --build build 2>&1 | tail -20`
Expected: builds with no errors or new warnings related to the measurement code.

- [ ] **Step 2: Run the measurement test suite**

Run: `ctest --test-dir build -R 'libf3d::TestSDKMeasurement' -V 2>&1 | tail -20`
Expected: `TestSDKMeasurement` and `TestSDKMeasurementInteraction` pass.

- [ ] **Step 3: Run the full libf3d suite for regressions**

Run: `ctest --test-dir build -L libf3d 2>&1 | tail -25`
Expected: no new failures. Note: `TestSDKEngineRecreation` times out in this environment as a pre-existing issue unrelated to this work — it is not a regression.

- [ ] **Step 4: Manual end-to-end check (skip if headless)**

Run `./build/bin/f3d testing/data/cow.vtp`: enter measurement mode, measure two points, switch the Axis radio through Free/X/Y/Z and confirm the primary line, breakdown, and 3D staircase update; confirm `--measurement-axis=x` on the CLI pre-selects X.

- [ ] **Step 5: Final commit if any fixes were needed**

```bash
git add -A
git commit -m "fix: address axis-measurement verification findings"
```

---

## Notes for the Implementer

- **Verification discipline:** run each step's command and confirm the expected output before marking it done. Use `superpowers:verification-before-completion` before claiming the feature complete.
- **ASCII only:** the panel labels use `X:`/`Y:`/`Z:` — never the `Δ` glyph (font coverage).
- **`set_measurement_unit` is unchanged** — it still calls `RefreshPanel()` (units do not change geometry). Only `set_measurement_axis` calls `RefreshMeasurement()` (axis changes the staircase).
- **Out of scope:** object detection (circles/faces/auto-centers), up-relative axes, axis-cycle hotkey. Do not implement these.
