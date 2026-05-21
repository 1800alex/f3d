#include "measurementManager.h"

#include "interactor_impl.h"
#include "options.h"
#include "window_impl.h"

#include <vtkActor.h>
#include <vtkCell.h>
#include <vtkF3DRenderer.h>
#include <vtkLineSource.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

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
}
