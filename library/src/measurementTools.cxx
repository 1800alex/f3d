#include "measurementTools.h"

#include <vtkLine.h>
#include <vtkMath.h>

#include <algorithm>
#include <cmath>
#include <map>

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
