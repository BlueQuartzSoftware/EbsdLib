#pragma once

#include "EbsdLib/EbsdLib.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ebsdlib
{

class EbsdLib_EXPORT FundamentalSectorGeometry
{
public:
  using Vec3 = std::array<double, 3>;

  FundamentalSectorGeometry(std::vector<Vec3> boundaryNormals, std::vector<Vec3> vertices, std::string colorKeyMode, int32_t supergroupIndex = -1);

  std::pair<double, double> polarCoordinates(const Vec3& h) const;
  double correctAzimuthalAngle(double rhoRaw) const;
  bool isInside(const Vec3& h) const;

  const Vec3& barycenter() const;
  const std::vector<Vec3>& vertices() const;
  const std::vector<Vec3>& boundaryNormals() const;
  const std::string& colorKeyMode() const;
  int32_t supergroupIndex() const;

  // Static factory methods for each Laue group
  static FundamentalSectorGeometry cubicHigh();      // m-3m
  static FundamentalSectorGeometry cubicLow();       // m-3
  static FundamentalSectorGeometry hexagonalHigh();  // 6/mmm
  static FundamentalSectorGeometry hexagonalLow();   // 6/m
  static FundamentalSectorGeometry tetragonalHigh(); // 4/mmm
  static FundamentalSectorGeometry tetragonalLow();  // 4/m
  static FundamentalSectorGeometry trigonalHigh();   // -3m
  static FundamentalSectorGeometry trigonalLow();    // -3
  static FundamentalSectorGeometry orthorhombic();   // mmm
  static FundamentalSectorGeometry monoclinic();     // 2/m
  static FundamentalSectorGeometry triclinic();      // -1

private:
  std::vector<Vec3> m_BoundaryNormals;
  std::vector<Vec3> m_Vertices;
  Vec3 m_Barycenter = {0.0, 0.0, 0.0};
  std::string m_ColorKeyMode;
  int32_t m_SupergroupIndex = -1;

  static constexpr size_t k_AzimuthalTableSize = 1000;
  std::array<double, k_AzimuthalTableSize> m_AzimuthalCorrectionTable = {};

  void computeBarycenter();
  void precomputeAzimuthalCorrection();

  static Vec3 vecNormalize(const Vec3& v);
  static Vec3 vecCross(const Vec3& a, const Vec3& b);
  static double vecDot(const Vec3& a, const Vec3& b);
  static double vecAngle(const Vec3& a, const Vec3& b);
  static Vec3 vecNeg(const Vec3& v);
};

} // namespace ebsdlib
