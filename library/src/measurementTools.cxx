#include "measurementTools.h"

#include <vtkCell.h>
#include <vtkDataSet.h>
#include <vtkIdList.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>

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

// Unit normal of triangle `cellId` of `mesh`, written into `normal`.
void TriangleNormal(vtkPolyData* mesh, vtkIdType cellId, double normal[3])
{
  vtkNew<vtkIdList> ptIds;
  mesh->GetCellPoints(cellId, ptIds);
  double p0[3];
  double p1[3];
  double p2[3];
  mesh->GetPoint(ptIds->GetId(0), p0);
  mesh->GetPoint(ptIds->GetId(1), p1);
  mesh->GetPoint(ptIds->GetId(2), p2);
  vtkTriangle::ComputeNormal(p0, p1, p2, normal);
}

// Breadth-first set of triangles coplanar with `seedCell` (normals within
// `angleToleranceDeg` of the seed triangle's normal), edge-connected. Bounded
// to `maxTriangles` to keep the worst case cheap on huge flat faces.
std::vector<vtkIdType> GrowCoplanarRegion(
  vtkPolyData* mesh, vtkIdType seedCell, double angleToleranceDeg, std::size_t maxTriangles)
{
  std::vector<vtkIdType> region;
  if (mesh->GetCellType(seedCell) != VTK_TRIANGLE)
  {
    return region;
  }
  mesh->BuildLinks(); // idempotent; required for GetCellNeighbors

  double seedNormal[3];
  TriangleNormal(mesh, seedCell, seedNormal);
  const double cosTol = std::cos(vtkMath::RadiansFromDegrees(angleToleranceDeg));

  std::set<vtkIdType> visited{ seedCell };
  std::queue<vtkIdType> frontier;
  frontier.push(seedCell);
  while (!frontier.empty() && region.size() < maxTriangles)
  {
    const vtkIdType current = frontier.front();
    frontier.pop();
    region.push_back(current);

    vtkNew<vtkIdList> cellPts;
    mesh->GetCellPoints(current, cellPts);
    if (cellPts->GetNumberOfIds() != 3)
    {
      continue;
    }
    for (int e = 0; e < 3; ++e)
    {
      vtkNew<vtkIdList> edge;
      edge->InsertNextId(cellPts->GetId(e));
      edge->InsertNextId(cellPts->GetId((e + 1) % 3));
      vtkNew<vtkIdList> neighbors;
      mesh->GetCellNeighbors(current, edge, neighbors);
      for (vtkIdType n = 0; n < neighbors->GetNumberOfIds(); ++n)
      {
        const vtkIdType nb = neighbors->GetId(n);
        if (visited.count(nb) != 0 || mesh->GetCellType(nb) != VTK_TRIANGLE)
        {
          continue;
        }
        double nbNormal[3];
        TriangleNormal(mesh, nb, nbNormal);
        // |dot| so that triangles in the same plane with FLIPPED winding (their
        // normal points the opposite way) are still treated as coplanar. Common
        // in real-world meshes whose triangle winding is not strictly consistent.
        if (std::fabs(vtkMath::Dot(seedNormal, nbNormal)) >= cosTol)
        {
          visited.insert(nb);
          frontier.push(nb);
        }
      }
    }
  }
  return region;
}
}

namespace f3d::detail
{
//----------------------------------------------------------------------------
MeasureObject ResolvePickedObject(const std::array<double, 3>& worldPos,
  vtkDataSet* dataset, vtkIdType cellId, double snapTol)
{
  MeasureObject obj{};
  vtkCell* cell = dataset->GetCell(cellId);
  vtkPoints* pts = cell->GetPoints();
  const vtkIdType nbPts = pts->GetNumberOfPoints();

  // 1. Snap to the nearest vertex if within snapTol.
  vtkIdType nearestVertex = -1;
  double bestVertexDist2 = snapTol * snapTol;
  for (vtkIdType i = 0; i < nbPts; ++i)
  {
    double p[3];
    pts->GetPoint(i, p);
    const double d2 = vtkMath::Distance2BetweenPoints(worldPos.data(), p);
    if (d2 <= bestVertexDist2)
    {
      bestVertexDist2 = d2;
      nearestVertex = i;
    }
  }
  if (nearestVertex >= 0)
  {
    double p[3];
    pts->GetPoint(nearestVertex, p);
    obj.ObjType = MeasureObject::Type::POINT;
    obj.P0 = { p[0], p[1], p[2] };
    return obj;
  }

  // 2. Otherwise pick the cell edge nearest to worldPos.
  vtkIdType bestEdge = 0;
  double bestEdgeDist2 = std::numeric_limits<double>::max();
  for (vtkIdType i = 0; i < nbPts; ++i)
  {
    double e0[3];
    double e1[3];
    pts->GetPoint(i, e0);
    pts->GetPoint((i + 1) % nbPts, e1);
    double closest[3];
    ClosestPointOnSegment(worldPos.data(), e0, e1, closest);
    const double d2 = vtkMath::Distance2BetweenPoints(worldPos.data(), closest);
    if (d2 < bestEdgeDist2)
    {
      bestEdgeDist2 = d2;
      bestEdge = i;
    }
  }
  // 2b. If the nearest edge is within snapTol, it is an EDGE pick.
  if (bestEdgeDist2 <= snapTol * snapTol)
  {
    double e0[3];
    double e1[3];
    pts->GetPoint(bestEdge, e0);
    pts->GetPoint((bestEdge + 1) % nbPts, e1);
    obj.ObjType = MeasureObject::Type::EDGE;
    obj.P0 = { e0[0], e0[1], e0[2] };
    obj.P1 = { e1[0], e1[1], e1[2] };
    return obj;
  }

  // 3. Otherwise it is a FACE: region-grow coplanar triangles around the
  //    picked triangle and measure at the area-weighted centroid.
  std::vector<vtkIdType> region;
  vtkPolyData* mesh = vtkPolyData::SafeDownCast(dataset);
  if (mesh != nullptr)
  {
    region = GrowCoplanarRegion(mesh, cellId, 15.0, 5000);
  }
  if (region.empty())
  {
    region.push_back(cellId); // fallback: the picked triangle alone
  }

  double weightedCentroid[3] = { 0.0, 0.0, 0.0 };
  double totalArea = 0.0;
  double vertexSum[3] = { 0.0, 0.0, 0.0 };
  int vertexCount = 0;
  obj.FacePoints.clear();
  for (const vtkIdType tri : region)
  {
    vtkNew<vtkIdList> triPts;
    dataset->GetCellPoints(tri, triPts);
    if (triPts->GetNumberOfIds() != 3)
    {
      continue;
    }
    double p[3][3];
    for (int k = 0; k < 3; ++k)
    {
      dataset->GetPoint(triPts->GetId(k), p[k]);
      obj.FacePoints.push_back({ p[k][0], p[k][1], p[k][2] });
      vertexSum[0] += p[k][0];
      vertexSum[1] += p[k][1];
      vertexSum[2] += p[k][2];
      ++vertexCount;
    }
    const double area = vtkTriangle::TriangleArea(p[0], p[1], p[2]);
    totalArea += area;
    for (int axis = 0; axis < 3; ++axis)
    {
      weightedCentroid[axis] += area * (p[0][axis] + p[1][axis] + p[2][axis]) / 3.0;
    }
  }

  obj.ObjType = MeasureObject::Type::FACE;
  if (totalArea > 0.0)
  {
    obj.P0 = { weightedCentroid[0] / totalArea, weightedCentroid[1] / totalArea,
      weightedCentroid[2] / totalArea };
  }
  else if (vertexCount > 0)
  {
    obj.P0 = { vertexSum[0] / vertexCount, vertexSum[1] / vertexCount,
      vertexSum[2] / vertexCount };
  }
  return obj;
}

//----------------------------------------------------------------------------
MeasureResult ComputeDistance(const MeasureObject& aIn, const MeasureObject& bIn)
{
  using Type = MeasureObject::Type;

  // A FACE measures from its centroid (P0), identically to a POINT.
  MeasureObject a = aIn;
  MeasureObject b = bIn;
  if (a.ObjType == Type::FACE)
  {
    a.ObjType = Type::POINT;
  }
  if (b.ObjType == Type::FACE)
  {
    b.ObjType = Type::POINT;
  }

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

//----------------------------------------------------------------------------
std::array<std::array<double, 3>, 4> ComputeAxisPath(
  const std::array<double, 3>& a, const std::array<double, 3>& b, int axis)
{
  // Each leg moves along a single model axis by that axis' delta.
  std::array<std::array<double, 3>, 3> legs{};
  for (int i = 0; i < 3; ++i)
  {
    legs[i] = { 0.0, 0.0, 0.0 };
    legs[i][i] = b[i] - a[i];
  }

  // Draw the selected axis leg first, then the remaining two in ascending order.
  std::array<int, 3> order{ axis, 0, 0 };
  int n = 1;
  for (int i = 0; i < 3; ++i)
  {
    if (i != axis)
    {
      order[n++] = i;
    }
  }

  std::array<std::array<double, 3>, 4> path{};
  path[0] = a;
  for (int k = 0; k < 3; ++k)
  {
    const std::array<double, 3>& leg = legs[order[k]];
    path[k + 1] = { path[k][0] + leg[0], path[k][1] + leg[1], path[k][2] + leg[2] };
  }
  return path;
}

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
