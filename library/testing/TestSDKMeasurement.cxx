#include "PseudoUnitTest.h"

#include "measurementTools.h"

#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>

#include <cmath>
#include <string>

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

  return test.result();
}
