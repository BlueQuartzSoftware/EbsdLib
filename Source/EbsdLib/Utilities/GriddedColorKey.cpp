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
  // Snap eta and chi to nearest grid point (flat shading).
  // Instead of computing the exact color at (eta, chi),
  // we return the precomputed color at the nearest grid point.
  // This produces flat-colored patches like MTEX's surf() rendering.
  double etaPositive = eta;
  if(etaPositive < 0.0)
  {
    etaPositive += 2.0 * k_Pi;
  }
  return lookupGrid(etaPositive, chi);
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
