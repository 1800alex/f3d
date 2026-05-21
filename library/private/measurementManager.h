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
