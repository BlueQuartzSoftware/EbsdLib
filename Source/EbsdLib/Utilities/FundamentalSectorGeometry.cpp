#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace ebsdlib
{

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
FundamentalSectorGeometry::FundamentalSectorGeometry(std::vector<Vec3> boundaryNormals, std::vector<Vec3> vertices,
                                                     std::string colorKeyMode, int32_t supergroupIndex)
: m_BoundaryNormals(std::move(boundaryNormals))
, m_Vertices(std::move(vertices))
, m_ColorKeyMode(std::move(colorKeyMode))
, m_SupergroupIndex(supergroupIndex)
{
  computeBarycenter();
  precomputeAzimuthalCorrection();
}

// -----------------------------------------------------------------------
// Accessors
// -----------------------------------------------------------------------
const FundamentalSectorGeometry::Vec3& FundamentalSectorGeometry::barycenter() const
{
  return m_Barycenter;
}

const std::vector<FundamentalSectorGeometry::Vec3>& FundamentalSectorGeometry::vertices() const
{
  return m_Vertices;
}

const std::vector<FundamentalSectorGeometry::Vec3>& FundamentalSectorGeometry::boundaryNormals() const
{
  return m_BoundaryNormals;
}

const std::string& FundamentalSectorGeometry::colorKeyMode() const
{
  return m_ColorKeyMode;
}

int32_t FundamentalSectorGeometry::supergroupIndex() const
{
  return m_SupergroupIndex;
}

// -----------------------------------------------------------------------
// Vector math helpers
// -----------------------------------------------------------------------
FundamentalSectorGeometry::Vec3 FundamentalSectorGeometry::vecNormalize(const Vec3& v)
{
  double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if(len < 1.0e-15)
  {
    return {0.0, 0.0, 0.0};
  }
  return {v[0] / len, v[1] / len, v[2] / len};
}

FundamentalSectorGeometry::Vec3 FundamentalSectorGeometry::vecCross(const Vec3& a, const Vec3& b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double FundamentalSectorGeometry::vecDot(const Vec3& a, const Vec3& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double FundamentalSectorGeometry::vecAngle(const Vec3& a, const Vec3& b)
{
  double d = vecDot(a, b);
  d = std::clamp(d, -1.0, 1.0);
  return std::acos(d);
}

FundamentalSectorGeometry::Vec3 FundamentalSectorGeometry::vecNeg(const Vec3& v)
{
  return {-v[0], -v[1], -v[2]};
}

// -----------------------------------------------------------------------
// computeBarycenter
// -----------------------------------------------------------------------
void FundamentalSectorGeometry::computeBarycenter()
{
  if(m_Vertices.empty())
  {
    // For triclinic: use the north pole as the center of the hemisphere
    m_Barycenter = {0.0, 0.0, 1.0};
    return;
  }

  Vec3 sum = {0.0, 0.0, 0.0};
  for(const auto& v : m_Vertices)
  {
    sum[0] += v[0];
    sum[1] += v[1];
    sum[2] += v[2];
  }
  m_Barycenter = vecNormalize(sum);
}

// -----------------------------------------------------------------------
// isInside
// -----------------------------------------------------------------------
bool FundamentalSectorGeometry::isInside(const Vec3& h) const
{
  for(const auto& normal : m_BoundaryNormals)
  {
    if(vecDot(h, normal) < -1.0e-10)
    {
      return false;
    }
  }
  return true;
}

// -----------------------------------------------------------------------
// polarCoordinates
// -----------------------------------------------------------------------
std::pair<double, double> FundamentalSectorGeometry::polarCoordinates(const Vec3& h) const
{
  constexpr double k_Pi = 3.14159265358979323846;

  // Singularity guard: if h is at the barycenter
  double angleToCenter = vecAngle(h, m_Barycenter);
  if(angleToCenter < 1.0e-10)
  {
    return {0.0, 0.0};
  }

  // -------------------------------------------------------------------
  // RADIUS: normalized distance from center to boundary
  // -------------------------------------------------------------------
  // Normal to the great circle through center and h
  Vec3 gcNormal = vecNormalize(vecCross(m_Barycenter, h));

  double radius = std::numeric_limits<double>::infinity();
  double distCenterH = angleToCenter; // angle from center to h

  for(const auto& normal : m_BoundaryNormals)
  {
    // Intersection of the great circle (center,h) with boundary plane N_j.
    // The two great circles intersect at: P = normalize(cross(N_j, gcNormal)) and -P.
    Vec3 bp = vecNormalize(vecCross(normal, gcNormal));

    // Choose the intersection point on the same side as h relative to center.
    // We want the point P such that the arc center->P passes through or near h.
    // Test: if dot(h, bp) < 0, flip to -bp (choose the hemisphere containing h).
    if(vecDot(h, bp) < 0.0)
    {
      bp = vecNeg(bp);
    }

    double distCenterBp = vecAngle(m_Barycenter, bp);

    double ratio;
    if(distCenterBp < 1.0e-10)
    {
      // Center is on this boundary -- boundary is at distance 0, but the sector
      // is on the positive side. This shouldn't happen for well-defined sectors.
      // Treat as no constraint.
      continue;
    }
    else
    {
      ratio = distCenterH / distCenterBp;
    }

    // Handle NaN
    if(std::isnan(ratio))
    {
      ratio = 1.0;
    }

    radius = std::min(radius, ratio);
  }

  if(std::isinf(radius))
  {
    radius = 0.0;
  }
  radius = std::clamp(radius, 0.0, 1.0);

  // -------------------------------------------------------------------
  // AZIMUTHAL ANGLE: angle in the tangent plane at the barycenter
  // -------------------------------------------------------------------
  // Reference direction: project z-axis onto tangent plane at barycenter
  Vec3 ref = {0.0, 0.0, 1.0};
  // If barycenter is very close to z-axis, use x-axis instead
  if(std::abs(vecDot(ref, m_Barycenter)) > 0.99)
  {
    ref = {1.0, 0.0, 0.0};
  }

  // Project ref onto tangent plane at barycenter: rx = ref - dot(ref, center) * center
  double refDotCenter = vecDot(ref, m_Barycenter);
  Vec3 rx = {ref[0] - refDotCenter * m_Barycenter[0], ref[1] - refDotCenter * m_Barycenter[1], ref[2] - refDotCenter * m_Barycenter[2]};
  rx = vecNormalize(rx);

  // ry = center x rx (right-hand rule in tangent plane)
  Vec3 ry = vecNormalize(vecCross(m_Barycenter, rx));

  // Direction from center to h in tangent plane (not normalized -- fine for atan2)
  double hDotCenter = vecDot(h, m_Barycenter);
  Vec3 dv = {h[0] - hDotCenter * m_Barycenter[0], h[1] - hDotCenter * m_Barycenter[1], h[2] - hDotCenter * m_Barycenter[2]};

  double rho = std::atan2(vecDot(ry, dv), vecDot(rx, dv));
  rho = std::fmod(rho + 2.0 * k_Pi, 2.0 * k_Pi); // ensure [0, 2*pi)

  return {radius, rho};
}

// -----------------------------------------------------------------------
// precomputeAzimuthalCorrection -- identity mapping for now
// -----------------------------------------------------------------------
void FundamentalSectorGeometry::precomputeAzimuthalCorrection()
{
  constexpr double k_TwoPi = 2.0 * 3.14159265358979323846;
  for(size_t i = 0; i < k_AzimuthalTableSize; i++)
  {
    m_AzimuthalCorrectionTable[i] = static_cast<double>(i) / static_cast<double>(k_AzimuthalTableSize) * k_TwoPi;
  }
}

// -----------------------------------------------------------------------
// correctAzimuthalAngle -- linear interpolation into precomputed table
// -----------------------------------------------------------------------
double FundamentalSectorGeometry::correctAzimuthalAngle(double rhoRaw) const
{
  constexpr double k_TwoPi = 2.0 * 3.14159265358979323846;
  // Map rhoRaw into [0, 2*pi)
  double rho = std::fmod(rhoRaw, k_TwoPi);
  if(rho < 0.0)
  {
    rho += k_TwoPi;
  }

  // Fractional index into the table
  double fIdx = rho / k_TwoPi * static_cast<double>(k_AzimuthalTableSize);
  size_t idx0 = static_cast<size_t>(fIdx);
  double frac = fIdx - static_cast<double>(idx0);

  if(idx0 >= k_AzimuthalTableSize - 1)
  {
    return m_AzimuthalCorrectionTable[k_AzimuthalTableSize - 1];
  }

  // Linear interpolation
  return m_AzimuthalCorrectionTable[idx0] * (1.0 - frac) + m_AzimuthalCorrectionTable[idx0 + 1] * frac;
}

// =====================================================================
// Static factory methods for each Laue group
// =====================================================================
//
// Coordinate system: h = (sin(chi)*cos(eta), sin(chi)*sin(eta), cos(chi))
//   chi = acos(z)  -- polar angle from z-axis (north pole)
//   eta = atan2(y, x) -- azimuthal angle in xy-plane
//
// Boundary normals N define the interior as: dot(h, N) >= 0 for all N.
// For a meridian boundary at eta = alpha:
//   - The plane contains z-axis and direction [cos(alpha), sin(alpha), 0]
//   - Inward normal (toward smaller eta): [sin(alpha), -cos(alpha), 0]
//   - Inward normal (toward larger eta): [-sin(alpha), cos(alpha), 0]

// -----------------------------------------------------------------------
// cubicHigh: m-3m
// SST: eta in [0, 45deg], chi in [0, chiMax(eta)]
// chiMax(eta) = acos(sqrt(1/(2+tan^2(eta))))
//
// Vertices (on unit sphere):
//   [001] = {0, 0, 1}  at eta=0, chi=0
//   [101] = {s2, 0, s2} at eta=0, chi=45deg
//   [111] = {s3, s3, s3} at eta=45deg, chi=acos(1/sqrt3)
//
// Boundaries:
//   1. eta >= 0  =>  y >= 0, normal = [0, 1, 0]
//   2. eta <= 45  =>  normal = [sin45, -cos45, 0] = [s2, -s2, 0]
//   3. Hypotenuse: great circle from [101] to [111]
//      cross([1,0,1], [1,1,1]) = [-1, 0, 1] => normalized: [-s2, 0, s2]
//      Verify: dot([0,0,1], [-s2,0,s2]) = s2 > 0. [001] inside. Good.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::cubicHigh()
{
  double s2 = 1.0 / std::sqrt(2.0);
  double s3 = 1.0 / std::sqrt(3.0);
  return FundamentalSectorGeometry(
      // Boundary normals (dot(h, N) >= 0 defines interior)
      {{0.0, 1.0, 0.0},   // y >= 0: eta >= 0 boundary
       {s2, -s2, 0.0},    // eta <= 45deg boundary
       {-s2, 0.0, s2}},   // hypotenuse: great circle [101]-[111]
      // Vertices
      {{0.0, 0.0, 1.0},   // [001]
       {s2, 0.0, s2},     // [101]
       {s3, s3, s3}},     // [111]
      "standard");
}

// -----------------------------------------------------------------------
// cubicLow: m-3
// SST: eta in [0, 90deg], chi in [0, chiMax(eta)]
// Same chiMax formula as cubicHigh, but eta extends to 90deg.
//
// Vertices:
//   [001] = {0, 0, 1}  at eta=0, chi=0
//   [101] = {s2, 0, s2} at eta=0, chi=45deg (chiMax at eta=0)
//   [011] = {0, s2, s2} at eta=90, chi=45deg (chiMax at eta=90)
//   [111] = {s3, s3, s3} at eta=45deg, chi=acos(1/sqrt3) (peak of boundary curve)
//
// The curved boundary from [101] to [011] through [111] is NOT a single great circle.
// We approximate it with the great circle from [101] to [011]:
//   cross([1,0,1], [0,1,1]) = [-1, -1, 1] => normalized: [-1,-1,1]/sqrt3
//   Verify: dot([0,0,1], [-1,-1,1]/sqrt3) = 1/sqrt3 > 0. [001] inside. Good.
//   dot([1,1,1]/sqrt3, [-1,-1,1]/sqrt3) = (-1-1+1)/3 = -1/3 < 0. [111] outside! Bad.
//
// Instead, split into two great circle arcs: [101]-[111] and [111]-[011].
//   Arc [101]-[111]: normal [-s2, 0, s2] (same as cubicHigh hypotenuse)
//   Arc [111]-[011]: cross([1,1,1], [0,1,1]) = [1*1-1*1, 1*0-1*1, 1*1-1*0] = [0,-1,1]
//     normalized: [0,-s2,s2]
//     Verify: dot([0,0,1], [0,-s2,s2]) = s2 > 0. [001] inside. Good.
//
// But using both normals would over-constrain the sector. We need to be careful:
// The actual boundary is curved, and points near [111] may be "above" both great circles.
// Since [111] is ON both arcs (it's the shared vertex), this should work.
// The two great circle arcs define a convex region that is contained in the actual SST.
// This is a conservative approximation.
//
// Boundaries:
//   1. eta >= 0  =>  y >= 0, normal = [0, 1, 0]
//   2. eta <= 90  =>  x >= 0, normal = [1, 0, 0]
//   3. Arc [101]-[111]: normal = [-s2, 0, s2]
//   4. Arc [111]-[011]: normal = [0, -s2, s2]
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::cubicLow()
{
  double s2 = 1.0 / std::sqrt(2.0);
  double s3 = 1.0 / std::sqrt(3.0);
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0},   // y >= 0: eta >= 0
       {1.0, 0.0, 0.0},   // x >= 0: eta <= 90deg
       {-s2, 0.0, s2},    // hypotenuse arc [101]-[111]
       {0.0, -s2, s2}},   // hypotenuse arc [111]-[011]
      // Vertices
      {{0.0, 0.0, 1.0},   // [001]
       {s2, 0.0, s2},     // [101]
       {0.0, s2, s2},     // [011]
       {s3, s3, s3}},     // [111]
      "extended",
      1 // supergroup = CubicHigh
  );
}

// -----------------------------------------------------------------------
// hexagonalHigh: 6/mmm
// SST: eta in [0, 30deg], chi in [0, 90deg]
// Vertices: [0001], [10-10] (at eta=0,chi=90), [2-1-10]/[11-20] (at eta=30,chi=90)
// In Cartesian with z up:
//   [0001] = [0, 0, 1]
//   eta=0, chi=90 => [1, 0, 0]
//   eta=30, chi=90 => [cos30, sin30, 0] = [sqrt3/2, 1/2, 0]
// Boundaries:
//   1. eta >= 0  =>  y >= 0, normal = [0, 1, 0]
//   2. eta <= 30deg =>  normal pointing inward from the eta=30 meridian plane
//      Meridian at eta=30: plane through z and direction [cos30, sin30, 0]
//      Normal to this plane (pointing toward eta < 30): [sin30, -cos30, 0] = [1/2, -sqrt3/2, 0]
//      Verify: dot([1,0,0], [1/2,-sqrt3/2,0]) = 1/2 > 0. [eta=0] is inside. Good.
//   3. chi >= 0 is automatic on upper hemisphere (but we also need chi <= 90)
//      chi <= 90 => z >= 0, but points at chi=90 have z=0 which is on the boundary.
//      Actually, we don't need an explicit normal for z >= 0 since
//      the sector extends to the equator. But we need it to exclude the southern hemisphere.
//      Normal = [0, 0, 1] would only allow z > 0 (actually z >= 0 with our tolerance).
//      Let's not add it; the SST naturally lives in the upper hemisphere.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::hexagonalHigh()
{
  double s3h = std::sqrt(3.0) / 2.0; // cos(30)
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0},         // y >= 0: eta >= 0
       {0.5, -s3h, 0.0}},       // eta <= 30deg
      // Vertices
      {{0.0, 0.0, 1.0},         // [0001]
       {1.0, 0.0, 0.0},         // [10-10] at eta=0, chi=90
       {s3h, 0.5, 0.0}},        // [2-1-10] at eta=30, chi=90
      "standard");
}

// -----------------------------------------------------------------------
// hexagonalLow: 6/m
// SST: eta in [0, 60deg], chi in [0, 90deg]
// Vertices: [0001], [10-10] (eta=0, chi=90), [eta=60, chi=90]
// eta=60, chi=90 => [cos60, sin60, 0] = [1/2, sqrt3/2, 0]
// Boundaries:
//   1. eta >= 0  =>  normal = [0, 1, 0]
//   2. eta <= 60  =>  normal from the eta=60 meridian plane
//      [sin60, -cos60, 0] = [sqrt3/2, -1/2, 0]
//      Verify: dot([1,0,0], [sqrt3/2,-1/2,0]) = sqrt3/2 > 0. Inside. Good.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::hexagonalLow()
{
  double s3h = std::sqrt(3.0) / 2.0; // sin(60) = cos(30)
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0},         // y >= 0: eta >= 0
       {s3h, -0.5, 0.0}},       // eta <= 60deg
      // Vertices
      {{0.0, 0.0, 1.0},         // [0001]
       {1.0, 0.0, 0.0},         // at eta=0, chi=90
       {0.5, s3h, 0.0}},        // at eta=60, chi=90
      "extended",
      0 // supergroup = HexagonalHigh
  );
}

// -----------------------------------------------------------------------
// tetragonalHigh: 4/mmm
// SST: eta in [0, 45deg], chi in [0, 90deg]
// Vertices: [001], [100] (eta=0,chi=90), [110] (eta=45,chi=90)
// Boundaries:
//   1. eta >= 0  =>  normal = [0, 1, 0]
//   2. eta <= 45  =>  normal = [sin45, -cos45, 0] = [1/sqrt2, -1/sqrt2, 0]
//      Verify: dot([1,0,0], [s2,-s2,0]) = s2 > 0. Inside. Good.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::tetragonalHigh()
{
  double s2 = 1.0 / std::sqrt(2.0);
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0},         // y >= 0: eta >= 0
       {s2, -s2, 0.0}},         // eta <= 45deg
      // Vertices
      {{0.0, 0.0, 1.0},         // [001]
       {1.0, 0.0, 0.0},         // [100] at eta=0, chi=90
       {s2, s2, 0.0}},          // [110] at eta=45, chi=90
      "standard");
}

// -----------------------------------------------------------------------
// tetragonalLow: 4/m
// SST: eta in [0, 90deg], chi in [0, 90deg]
// Vertices: [001], [100] (eta=0,chi=90), [010] (eta=90,chi=90)
// Boundaries:
//   1. eta >= 0  =>  normal = [0, 1, 0]
//   2. eta <= 90  =>  normal = [1, 0, 0] (x >= 0)
//      Verify: dot([0,1,0], [1,0,0]) = 0. On boundary. Good.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::tetragonalLow()
{
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0},         // y >= 0: eta >= 0
       {1.0, 0.0, 0.0}},        // x >= 0: eta <= 90deg
      // Vertices
      {{0.0, 0.0, 1.0},         // [001]
       {1.0, 0.0, 0.0},         // [100] at eta=0, chi=90
       {0.0, 1.0, 0.0}},        // [010] at eta=90, chi=90
      "extended",
      8 // supergroup = TetragonalHigh
  );
}

// -----------------------------------------------------------------------
// trigonalHigh: -3m
// SST: eta in [-90, -30deg], chi in [0, 90deg]
// In the standard spherical coordinate convention used by the code:
//   h = (sin(chi)*cos(eta), sin(chi)*sin(eta), cos(chi))
// At eta=-90, chi=90: [0, -1, 0]
// At eta=-30, chi=90: [cos(-30), sin(-30), 0] = [sqrt3/2, -1/2, 0]
// Vertices: [001], [0,-1,0], [sqrt3/2, -1/2, 0]
//
// Boundaries:
//   1. eta >= -90  =>  The meridian at eta=-90 is the y-axis plane with x=0.
//      Points at eta > -90 (moving toward eta=-30) have x > 0 (for chi > 0).
//      So normal = [-1, 0, 0]? Let's verify:
//      At eta=-30: h = [sqrt3/2, -1/2, 0]. dot(h, [-1,0,0]) = -sqrt3/2 < 0. Wrong sign.
//      At eta=-90: h = [0, -1, 0]. dot(h, [-1,0,0]) = 0. On boundary.
//      So normal should be [1, 0, 0] (x >= 0).
//      But at eta=-90: [0,-1,0] has x=0. On boundary. Good.
//      Actually let me reconsider. eta=-90 means cos(eta)=0, sin(eta)=-1.
//      h = [sin(chi)*0, sin(chi)*(-1), cos(chi)] = [0, -sin(chi), cos(chi)].
//      For the boundary eta >= -90, we need everything with eta from -90 to -30.
//      At eta=-60 (interior): h = [sin(chi)*cos(-60), sin(chi)*sin(-60), cos(chi)]
//        = [sin(chi)*0.5, -sin(chi)*sqrt3/2, cos(chi)]
//      dot(h, [1,0,0]) = sin(chi)*0.5 > 0 when chi > 0. So x >= 0 works. But wait...
//      Hmm, cos(-90)=0, cos(-60)=0.5, cos(-30)=sqrt3/2. So for eta in [-90,-30], cos(eta) >= 0.
//      And sin(eta) < 0 for all these angles. So the x-component of h = sin(chi)*cos(eta) >= 0.
//      Normal = [0, -1, 0] won't work because at eta=-60 we get dot = sin(chi)*sqrt3/2 > 0. That works.
//      Actually we need 2 meridian boundaries: eta >= -90 and eta <= -30.
//
//   For eta >= -90deg (eta >= -90):
//     The plane at eta=-90 passes through z and [0,-1,0].
//     For eta > -90, cos(eta) > 0 => x > 0.
//     Normal = [1, 0, 0]... but the sector is entirely in x >= 0? Let me check.
//     Yes, cos(eta) >= 0 for eta in [-90, -30] (both are in the range where cos >= 0).
//     So normal = [1, 0, 0] works for the left boundary. But it's redundant for eta >= -90
//     since cos(-90) = 0 (boundary) and cos(-89) > 0 (inside).
//     Actually no: at eta = -90, x=0 which is exactly on the boundary. We need x >= 0.
//     But the boundary is the meridian at eta=-90, and the inward normal is [1,0,0].
//     Hmm wait, the meridian at eta=-90 is the plane y-z (x=0). Points with eta > -90
//     have x > 0. So the inward normal is indeed [1, 0, 0].
//
//   For eta <= -30deg:
//     The plane at eta=-30 passes through z and [cos(-30), sin(-30), 0] = [sqrt3/2, -1/2, 0].
//     The inward normal points toward more negative eta (from -30 toward -90).
//     Normal to a meridian at angle alpha is [-sin(alpha), cos(alpha), 0] (pointing toward decreasing eta).
//     For alpha = -30: [-sin(-30), cos(-30), 0] = [1/2, sqrt3/2, 0].
//     But that points toward INCREASING eta. We want the opposite: [-1/2, -sqrt3/2, 0].
//     Verify: dot([0,-1,0], [-1/2,-sqrt3/2,0]) = sqrt3/2 > 0. Inside. Good.
//     Verify: dot([sqrt3/2,-1/2,0], [-1/2,-sqrt3/2,0]) = -sqrt3/4 + sqrt3/4 = 0. On boundary. Good.
//     Verify: dot([1/2,-sqrt3/2,0], [-1/2,-sqrt3/2,0]) = -1/4 + 3/4 = 1/2 > 0. Inside (eta=-60). Good.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::trigonalHigh()
{
  double s3h = std::sqrt(3.0) / 2.0;
  return FundamentalSectorGeometry(
      // Boundary normals
      {{1.0, 0.0, 0.0},           // x >= 0: eta >= -90deg boundary
       {-0.5, -s3h, 0.0}},        // eta <= -30deg boundary
      // Vertices
      {{0.0, 0.0, 1.0},           // [001]
       {0.0, -1.0, 0.0},          // at eta=-90, chi=90
       {s3h, -0.5, 0.0}},         // at eta=-30, chi=90
      "standard");
}

// -----------------------------------------------------------------------
// trigonalLow: -3
// SST: eta in [-120, 0deg], chi in [0, 90deg]
// At eta=0, chi=90: [1, 0, 0]
// At eta=-120, chi=90: [cos(-120), sin(-120), 0] = [-1/2, -sqrt3/2, 0]
// Vertices: [001], [1,0,0], [-1/2, -sqrt3/2, 0]
//
// Boundaries:
//   1. eta >= -120:
//     Meridian at eta=-120, direction [-1/2, -sqrt3/2, 0].
//     Inward normal (toward more positive eta): [sin(120), -cos(120), 0] = [sqrt3/2, 1/2, 0]
//     Verify: dot([1,0,0], [sqrt3/2,1/2,0]) = sqrt3/2 > 0. Inside (eta=0). Good.
//     Verify: dot([-1/2,-sqrt3/2,0], [sqrt3/2,1/2,0]) = -sqrt3/4 - sqrt3/4 = -sqrt3/2? Wait.
//     Let me redo. [-1/2, -sqrt3/2, 0] dot [sqrt3/2, 1/2, 0] = -sqrt3/4 + (-sqrt3/2)(1/2) = -sqrt3/4 - sqrt3/4 = -sqrt3/2 < 0. Not on boundary!
//
//     Let me reconsider. Normal to meridian at angle alpha pointing inward (toward the sector).
//     The meridian at alpha goes through the z-axis in the direction [cos(alpha), sin(alpha), 0].
//     Its normal in the xy-plane is [-sin(alpha), cos(alpha), 0] (rotated 90 CCW).
//     For the sector with eta in [-120, 0], the interior is at eta > -120.
//     Going from eta=-120 toward eta=0 is CCW if viewed from +z (eta increases CCW).
//     The inward normal at eta=-120 should point toward the interior (higher eta).
//
//     Normal = [-sin(-120), cos(-120), 0] = [sqrt3/2, -1/2, 0].
//     Verify: dot([-1/2, -sqrt3/2, 0], [sqrt3/2, -1/2, 0]) = -sqrt3/4 + sqrt3/4 = 0. On boundary. Good.
//     Verify: dot([1, 0, 0], [sqrt3/2, -1/2, 0]) = sqrt3/2 > 0. Interior (eta=0). Good.
//     Verify interior point at eta=-60: [cos(-60), sin(-60), 0] = [1/2, -sqrt3/2, 0]
//       dot([1/2, -sqrt3/2, 0], [sqrt3/2, -1/2, 0]) = sqrt3/4 + sqrt3/4 = sqrt3/2 > 0. Good.
//
//   2. eta <= 0:
//     Meridian at eta=0, direction [1, 0, 0].
//     Normal pointing inward (toward more negative eta): [sin(0), -cos(0), 0] = [0, -1, 0].
//     Verify: dot([1, 0, 0], [0, -1, 0]) = 0. On boundary. Good.
//     Verify: dot([1/2, -sqrt3/2, 0], [0, -1, 0]) = sqrt3/2 > 0. Interior. Good.
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::trigonalLow()
{
  double s3h = std::sqrt(3.0) / 2.0;
  return FundamentalSectorGeometry(
      // Boundary normals
      {{s3h, -0.5, 0.0},           // eta >= -120deg
       {0.0, -1.0, 0.0}},          // eta <= 0deg (y <= 0)
      // Vertices
      {{0.0, 0.0, 1.0},            // [001]
       {1.0, 0.0, 0.0},            // at eta=0, chi=90
       {-0.5, -s3h, 0.0}},         // at eta=-120, chi=90
      "impossible");
}

// -----------------------------------------------------------------------
// orthorhombic: mmm
// SST: eta in [0, 90deg], chi in [0, 90deg]
// Vertices: [001], [100] (eta=0,chi=90), [010] (eta=90,chi=90)
// Boundaries:
//   1. eta >= 0  =>  normal = [0, 1, 0]
//   2. eta <= 90  =>  normal = [1, 0, 0] (x >= 0)
// (Same as tetragonal low geometry, but different color mode and no supergroup)
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::orthorhombic()
{
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0},         // y >= 0: eta >= 0
       {1.0, 0.0, 0.0}},        // x >= 0: eta <= 90deg
      // Vertices
      {{0.0, 0.0, 1.0},         // [001]
       {1.0, 0.0, 0.0},         // [100] at eta=0, chi=90
       {0.0, 1.0, 0.0}},        // [010] at eta=90, chi=90
      "standard");
}

// -----------------------------------------------------------------------
// monoclinic: 2/m
// SST: eta in [0, 180deg], chi in [0, 90deg]
// Vertices: [001], [100] (eta=0,chi=90), [-100] (eta=180,chi=90)
// Boundaries:
//   1. eta >= 0  =>  normal = [0, 1, 0]
//   2. eta <= 180  =>  normal = [0, -1, 0]
//      But wait: [0,1,0] and [0,-1,0] together mean y >= 0 AND y <= 0, i.e. y = 0.
//      That can't be right. Let me reconsider.
//      eta in [0, 180] means atan2(y,x) in [0, 180], which means y >= 0.
//      So the only meridian constraint is y >= 0, i.e., normal = [0, 1, 0].
//      The other boundary is chi <= 90 (hemisphere).
//      At eta=180: h = [sin(chi)*cos(180), sin(chi)*sin(180), cos(chi)] = [-sin(chi), 0, cos(chi)]
//      This has y = 0, which satisfies y >= 0. So a single normal [0,1,0] covers the entire sector.
//      Actually we need no upper bound on eta since eta <= 180 just means we're in the y >= 0 half.
//      The sector is a full half-hemisphere (y >= 0).
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::monoclinic()
{
  return FundamentalSectorGeometry(
      // Boundary normals
      {{0.0, 1.0, 0.0}},        // y >= 0: eta in [0, 180deg]
      // Vertices
      {{0.0, 0.0, 1.0},         // [001]
       {1.0, 0.0, 0.0},         // [100] at eta=0, chi=90
       {-1.0, 0.0, 0.0}},       // [-100] at eta=180, chi=90
      "extended",
      6 // supergroup = OrthoRhombic
  );
}

// -----------------------------------------------------------------------
// triclinic: -1
// SST: eta in [0, 180deg], chi in [0, 90deg]  (but this is the same as monoclinic!)
// Actually triclinic covers the entire upper hemisphere.
// For triclinic, there's no azimuthal constraint; the SST is the full hemisphere.
// No vertices (it's a continuous region without corners in the traditional SST sense).
// The only boundary is the equator (chi <= 90).
// -----------------------------------------------------------------------
FundamentalSectorGeometry FundamentalSectorGeometry::triclinic()
{
  return FundamentalSectorGeometry(
      // No boundary normals needed (full upper hemisphere is the SST)
      // But we may need to keep z >= 0 for safety, though isInside should
      // handle this implicitly for directions in the upper hemisphere.
      {},
      // No vertices
      {},
      "impossible");
}

} // namespace ebsdlib
