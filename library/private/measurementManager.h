/**
 * @class   measurementManager
 * @brief   A private class managing measurement mode
 */

#ifndef f3d_measurementManager_h
#define f3d_measurementManager_h

#include "measurementTools.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <vtkActor.h>
#include <vtkSmartPointer.h>

class vtkDataSet;
class vtkRenderer;

namespace f3d
{
class options;

namespace detail
{
class window_impl;

class measurementManager
{
public:
  measurementManager(options& options, window_impl& window);
  ~measurementManager();

  /**
   * Toggle measurement mode on/off. A completed measurement and its 3D
   * annotation remain visible after the mode is turned off; they are removed
   * by Clear() (triggered by Escape while in mode, or by loading a file).
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
   * Resolve a pick (world position + picked dataset/cell) into a point, edge,
   * or face and advance the two-step selection. A null dataset or invalid
   * cellId is ignored. The third pick after a completed measurement discards
   * it and starts fresh.
   */
  void HandlePick(const std::array<double, 3>& worldPos, vtkDataSet* dataset, vtkIdType cellId,
    bool faceMode);

  /**
   * Clear the current selection, measurement, hover preview and 3D actors.
   */
  void Clear();

  /**
   * Resolve a pick under the cursor into a preview ("hover") highlight of the
   * object that would be selected if clicked. A null dataset or invalid cellId
   * clears the preview. Returns true if the preview changed and a render is needed.
   */
  bool HandleHover(const std::array<double, 3>& worldPos, vtkDataSet* dataset, vtkIdType cellId,
    bool faceMode);

  /**
   * Remove the hover preview highlight, if any.
   * Returns true if a preview existed and was removed (a render is needed).
   */
  bool ClearHover();

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

  /**
   * The per-axis breakdown ("X: .. Y: .. Z: .. <unit>"), converted to display
   * units. Empty until two objects are selected.
   */
  std::string GetComponentsString() const;

  /**
   * Push the current measurement state to the renderer's UI panel. Call after
   * a measurement unit option changes so the displayed distance is recomputed.
   */
  void RefreshPanel();

  /**
   * Rebuild the 3D annotation and refresh the panel. Call after the axis
   * option changes, since the axis affects the drawn geometry.
   */
  void RefreshMeasurement();

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

  /**
   * Rebuild the hover preview actor from the current HoverObject (or remove it
   * when there is no hover preview).
   */
  void UpdateHoverActor();

  /**
   * Build a translucent highlight actor for a face region (triangle vertices,
   * three consecutive entries per triangle). Returns nullptr for an empty
   * region. The caller adds it to a renderer.
   */
  vtkSmartPointer<vtkActor> MakeFaceHighlightActor(
    const std::vector<std::array<double, 3>>& facePoints) const;

  /**
   * Return the layer-1 overlay renderer used to draw the measurement
   * annotation on top of the model, creating it on first use. Shares the scene
   * camera. Returns nullptr if there is no render window yet.
   */
  vtkRenderer* GetOverlayRenderer();

  /**
   * Convert a model-space length to display units (identity when either unit
   * is unset/unknown).
   */
  double ConvertToDisplay(double modelValue) const;

  /**
   * The display-unit suffix (" mm" etc.), or empty when no conversion applies.
   */
  std::string DisplaySuffix() const;

  options& Options;
  window_impl& Window;

  bool Active = false;
  std::vector<MeasureObject> Selection;
  std::optional<MeasureResult> Result;
  std::optional<MeasureObject> HoverObject;

  std::vector<vtkSmartPointer<vtkActor>> Actors;
  std::vector<vtkSmartPointer<vtkActor>> HoverActors;
  vtkSmartPointer<vtkRenderer> OverlayRenderer;
};
}
}
#endif
