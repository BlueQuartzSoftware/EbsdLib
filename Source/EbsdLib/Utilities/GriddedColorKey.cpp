#include "EbsdLib/Utilities/GriddedColorKey.hpp"

#include <algorithm>
#include <cmath>

namespace ebsdlib
{

namespace
{
constexpr double k_Pi = 3.14159265358979323846;
constexpr double k_HalfPi = k_Pi / 2.0;
constexpr double k_DegToRad = k_Pi / 180.0;
} // namespace

GriddedColorKey::GriddedColorKey(IColorKey::Pointer innerKey, double resolutionDeg)
: m_InnerKey(std::move(innerKey))
, m_ResolutionDeg(resolutionDeg)
, m_ResolutionRad(resolutionDeg * k_DegToRad)
{
  // Grid covers eta in [0, pi] (180 degrees) and chi in [0, pi/2] (90 degrees)
  // This covers all possible Laue group SSTs
  m_EtaSteps = static_cast<int>(std::ceil(180.0 / resolutionDeg)) + 1;
  m_ChiSteps = static_cast<int>(std::ceil(90.0 / resolutionDeg)) + 1;
  precomputeGrid();
}

void GriddedColorKey::precomputeGrid()
{
  m_Grid.resize(m_EtaSteps);

  for(int ei = 0; ei < m_EtaSteps; ei++)
  {
    m_Grid[ei].resize(m_ChiSteps);
    double eta = static_cast<double>(ei) * m_ResolutionRad;

    for(int ci = 0; ci < m_ChiSteps; ci++)
    {
      double chi = static_cast<double>(ci) * m_ResolutionRad;

      // Convert spherical to Cartesian direction and compute color
      // via the inner key's Vec3 overload
      double sinChi = std::sin(chi);
      double cosChi = std::cos(chi);
      Vec3 dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), cosChi};

      m_Grid[ei][ci] = m_InnerKey->direction2Color(dir);
    }
  }
}

GriddedColorKey::Vec3 GriddedColorKey::lookupGrid(double eta, double chi) const
{
  // Map to grid indices via nearest-neighbor snapping
  int ei = static_cast<int>(std::round(eta / m_ResolutionRad));
  int ci = static_cast<int>(std::round(chi / m_ResolutionRad));

  // Clamp to grid bounds
  ei = std::clamp(ei, 0, m_EtaSteps - 1);
  ci = std::clamp(ci, 0, m_ChiSteps - 1);

  return m_Grid[ei][ci];
}

GriddedColorKey::Vec3 GriddedColorKey::direction2Color(const Vec3& direction) const
{
  // Convert direction to (eta, chi) and look up from grid
  double chi = std::acos(std::clamp(direction[2], -1.0, 1.0));
  double eta = std::atan2(direction[1], direction[0]);
  if(eta < 0.0)
  {
    eta += 2.0 * k_Pi;
  }
  return lookupGrid(eta, chi);
}

GriddedColorKey::Vec3 GriddedColorKey::direction2Color(double eta, double chi, const Vec3& angleLimits) const
{
  // Snap (eta, chi) to nearest grid coordinates so neighboring pixels in the
  // same cell return identical colors (flat-shaded patches, MTEX-style),
  // then ask the inner color key for the color at the snapped coordinates
  // using the caller-supplied angleLimits. We pass eta through unchanged
  // (no [0, 2π] wrap): some Laue classes have negative angleLimits[0]
  // (Trigonal-3 etaMin=-120°, -3m etaMin=-90°) and the TSL formula uses
  // |eta - etaMin| directly, which is correct only if eta retains its sign.
  // We cannot use the precomputed grid here because that grid was baked at
  // construction time using the inner key's *default* angle limits (cubic
  // m-3m for TSLColorKey), which are wrong for every other Laue class.
  const int ei = static_cast<int>(std::round(eta / m_ResolutionRad));
  const int ci = static_cast<int>(std::round(chi / m_ResolutionRad));
  double snappedEta = static_cast<double>(ei) * m_ResolutionRad;
  double snappedChi = static_cast<double>(ci) * m_ResolutionRad;

  // Clamp the snapped *chi* to [0, chiMax]. Without this, boundary pixels can
  // be pushed marginally outside the SST by the snap — for cubic m-3m chiMax
  // depends on eta, so the legend renderer passes angleLimits[2] =
  // chiMax(original_eta) but the snap shifts eta to a different cell whose
  // effective chiMax may differ. The TSL formula r = 1 - chi/chiMax then
  // goes negative, sqrt produces NaN, and the resulting cast-to-int produces
  // a stippled gray/dark line along the curved edge of the legend.
  //
  // We deliberately do NOT clamp snappedEta to [angleLimits[0],
  // angleLimits[1]]. Triclinic (-1) and any other class with a wide eta
  // range relies on the inner formula's |eta - etaMin| handling of out-of-
  // range eta to color the full IPF disk; clamping eta would collapse the
  // lower hemisphere of the disk to a single eta value.
  if(snappedChi < 0.0)
  {
    snappedChi = 0.0;
  }
  if(snappedChi > angleLimits[2])
  {
    snappedChi = angleLimits[2];
  }

  return m_InnerKey->direction2Color(snappedEta, snappedChi, angleLimits);
}

std::string GriddedColorKey::name() const
{
  return m_InnerKey->name() + " (gridded)";
}

IColorKey::Pointer GriddedColorKey::innerKey() const
{
  return m_InnerKey;
}

double GriddedColorKey::resolutionDeg() const
{
  return m_ResolutionDeg;
}

} // namespace ebsdlib
