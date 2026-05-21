#include "PseudoUnitTest.h"

#include "measurementTools.h"

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

  return test.result();
}
