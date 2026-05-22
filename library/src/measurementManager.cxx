#include "measurementManager.h"

#include "options.h"
#include "window_impl.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDataSet.h>
#include <vtkF3DRenderer.h>
#include <vtkLineSource.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace
{
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
}

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
void measurementManager::ToggleMeasurement()
{
  this->Active = !this->Active;
  if (!this->Active)
  {
    // The completed measurement persists, but the transient hover preview
    // only makes sense while the mode is active.
    this->ClearHover();
  }
  this->RefreshPanel();
}

//----------------------------------------------------------------------------
void measurementManager::Clear()
{
  this->Selection.clear();
  this->Result.reset();
  this->RemoveActors();
  this->ClearHover();
  this->RefreshPanel();
}

//----------------------------------------------------------------------------
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

  // The hover preview is recomputed on the next mouse move; drop it now so it
  // does not sit on top of the freshly placed selection marker.
  this->ClearHover();
  this->UpdateActors();
}

//----------------------------------------------------------------------------
bool measurementManager::HandleHover(
  const std::array<double, 3>& worldPos, vtkDataSet* dataset, vtkIdType cellId)
{
  if (!this->Active || dataset == nullptr || cellId < 0)
  {
    return this->ClearHover();
  }

  const MeasureObject obj =
    ResolvePickedObject(worldPos, dataset, cellId, SnapToleranceForCell(dataset, cellId));

  // Skip the rebuild (and the render it triggers) when hovering the same object.
  if (this->HoverObject.has_value() && this->HoverObject->ObjType == obj.ObjType &&
    this->HoverObject->P0 == obj.P0 && this->HoverObject->P1 == obj.P1)
  {
    return false;
  }

  this->HoverObject = obj;
  this->UpdateHoverActor();
  return true;
}

//----------------------------------------------------------------------------
bool measurementManager::ClearHover()
{
  if (!this->HoverObject.has_value())
  {
    return false;
  }
  this->HoverObject.reset();
  this->UpdateHoverActor();
  return true;
}

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

//----------------------------------------------------------------------------
vtkRenderer* measurementManager::GetOverlayRenderer()
{
  vtkF3DRenderer* mainRenderer = this->Window.GetRenderer();
  if (mainRenderer == nullptr)
  {
    return nullptr;
  }
  vtkRenderWindow* renderWindow = mainRenderer->GetRenderWindow();
  if (renderWindow == nullptr)
  {
    return nullptr;
  }

  if (this->OverlayRenderer == nullptr)
  {
    // A layer-1 renderer renders after the scene with its own fresh depth
    // buffer, so measurement annotation always draws on top of the model.
    this->OverlayRenderer = vtkSmartPointer<vtkRenderer>::New();
    this->OverlayRenderer->SetLayer(1);
    this->OverlayRenderer->InteractiveOff();
    renderWindow->AddRenderer(this->OverlayRenderer);
  }
  if (renderWindow->GetNumberOfLayers() < 2)
  {
    renderWindow->SetNumberOfLayers(2);
  }
  // Share the scene camera so the annotation tracks the view.
  this->OverlayRenderer->SetActiveCamera(mainRenderer->GetActiveCamera());
  return this->OverlayRenderer;
}

//----------------------------------------------------------------------------
void measurementManager::RemoveActors()
{
  vtkRenderer* overlay = this->GetOverlayRenderer();
  if (overlay != nullptr)
  {
    for (const auto& actor : this->Actors)
    {
      overlay->RemoveActor(actor);
    }
  }
  this->Actors.clear();
}

//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
void measurementManager::UpdateActors()
{
  this->RemoveActors();

  vtkF3DRenderer* mainRenderer = this->Window.GetRenderer();
  vtkRenderer* overlay = this->GetOverlayRenderer();
  if (mainRenderer == nullptr || overlay == nullptr)
  {
    return;
  }

  // Estimate a marker size from the scene's visible bounds.
  double bounds[6];
  mainRenderer->ComputeVisiblePropBounds(bounds);
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
    actor->GetProperty()->LightingOff(); // flat color: the overlay layer has no lights
    actor->PickableOff();
    overlay->AddActor(actor);
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
    actor->GetProperty()->LightingOff(); // flat color: the overlay layer has no lights
    actor->PickableOff();
    overlay->AddActor(actor);
    this->Actors.emplace_back(actor);
  };

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

  this->RefreshPanel();
}

//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
void measurementManager::RefreshMeasurement()
{
  this->UpdateActors();
}
}
