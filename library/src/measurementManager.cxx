#include "measurementManager.h"

#include "interactor_impl.h"
#include "options.h"
#include "window_impl.h"

#include <vtkActor.h>
#include <vtkCell.h>

#include <cmath>
#include <sstream>

namespace f3d::detail
{
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

  // Snap tolerance scales with the picked cell's size.
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

//----------------------------------------------------------------------------
void measurementManager::UpdateActors()
{
}

//----------------------------------------------------------------------------
void measurementManager::RemoveActors()
{
}
}
