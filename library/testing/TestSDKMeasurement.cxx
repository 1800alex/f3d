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

  return test.result();
}
