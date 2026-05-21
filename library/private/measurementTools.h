/**
 * @brief   Pure geometry and unit-conversion helpers for measurement mode.
 *
 * Declared separately from measurementManager so they can be unit-tested
 * without the renderer/interactor dependencies of the manager class.
 */

#ifndef f3d_measurementTools_h
#define f3d_measurementTools_h

#include <array>
#include <cstdint>
#include <optional>
#include <string>

class vtkCell;

namespace f3d
{
namespace detail
{
/**
 * A selectable object: either a single point or a finite edge segment.
 * For POINT, only P0 is meaningful.
 */
struct MeasureObject
{
  enum class Type : std::uint8_t
  {
    POINT,
    EDGE
  };
  Type ObjType;
  std::array<double, 3> P0;
  std::array<double, 3> P1;
};

/**
 * Result of a distance computation: the distance plus the two closest points
 * (used to draw the connecting line).
 */
struct MeasureResult
{
  double Distance;
  std::array<double, 3> ClosestA;
  std::array<double, 3> ClosestB;
};

/**
 * Return meters-per-unit for a unit name (mm, cm, m, in, ft).
 * Returns std::nullopt for an empty string or an unknown unit.
 */
std::optional<double> UnitToMeters(const std::string& unit);

/**
 * Compute the minimum distance between two MeasureObjects and the two closest
 * points realizing that distance.
 * point-point: straight line. point-edge: perpendicular to the finite segment.
 * edge-edge: minimum distance between the two finite segments.
 */
MeasureResult ComputeDistance(const MeasureObject& a, const MeasureObject& b);

/**
 * Resolve a pick into a MeasureObject. worldPos is the picked world position,
 * cell is the picked triangle (must not be null). If worldPos is within snapTol
 * of one of the cell's points, returns a POINT at that vertex; otherwise returns
 * the EDGE of the cell nearest to worldPos.
 */
MeasureObject ResolvePickedObject(
  const std::array<double, 3>& worldPos, vtkCell* cell, double snapTol);

/**
 * Return the 4 points of the right-angle "staircase" path from a to b for the
 * given axis (0=X, 1=Y, 2=Z). Point [0] is a and point [3] is b. The leg
 * [0]->[1] is the selected-axis leg (drawn first from a); the other two legs
 * follow in ascending axis order. The three legs sum from a to b.
 */
std::array<std::array<double, 3>, 4> ComputeAxisPath(
  const std::array<double, 3>& a, const std::array<double, 3>& b, int axis);
}
}
#endif
