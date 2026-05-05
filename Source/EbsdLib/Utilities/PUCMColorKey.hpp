#pragma once

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/IColorKey.hpp"

#include <memory>
#include <string>

namespace ebsdlib
{

/**
 * @brief Perceptually Uniform Color Map (PUCM) IPF color key.
 *
 * Implements the EDAX OIM Analysis "perceptually uniform" IPF color
 * scheme by adapting William Lenthe's reference C++ implementation
 * (wlenthe/crystallography, BSD-3) of:
 *   Nolze, Gert and Hielscher Ralf, "Orientations Perfectly Colors,"
 *   J. Appl. Crystallogr. 49.5 (2016): 1786–1802.
 *
 * The wlenthe header is vendored verbatim at
 * EbsdLib/Utilities/wlenthe_orientation_coloring.hpp; this class is a
 * thin dispatch layer that selects the correct entry point per
 * Laue class.
 *
 * Rotation point group → wlenthe dispatch:
 *   "1"   (-1)     → coloring::hemiIpf
 *   "2"   (2/m)    → coloring::cyclicIpf<T, 2>
 *   "222" (mmm)    → coloring::dihedralIpf<T, 2>
 *   "3"   (-3)     → coloring::cyclicIpf<T, 3>
 *   "32"  (-3m)    → coloring::dihedralIpf<T, 3>
 *   "4"   (4/m)    → coloring::cyclicIpf<T, 4>
 *   "422" (4/mmm)  → coloring::dihedralIpf<T, 4>
 *   "6"   (6/m)    → coloring::cyclicIpf<T, 6>
 *   "622" (6/mmm)  → coloring::dihedralIpf<T, 6>
 *   "23"  (m-3)    → coloring::cubicLowIpf
 *   "432" (m-3m)   → coloring::cubicIpf
 */
class EbsdLib_EXPORT PUCMColorKey : public IColorKey
{
public:
  /**
   * @brief Construct a PUCM color key bound to a specific Laue class.
   * @param rotationPointGroup String matching LaueOps::getRotationPointGroup()
   *        (one of: "1", "2", "222", "3", "32", "4", "422", "6", "622",
   *        "23", "432"). Throws std::invalid_argument otherwise.
   */
  explicit PUCMColorKey(const std::string& rotationPointGroup);
  ~PUCMColorKey() override = default;

  Vec3 direction2Color(const Vec3& direction) const override;
  Vec3 direction2Color(double eta, double chi, const Vec3& angleLimits) const override;
  std::string name() const override;

  std::string rotationPointGroup() const;

private:
  // The wlenthe dispatch is selected by the rotation-point-group string;
  // we resolve it to an internal enum at construction so direction2Color
  // is just a switch.
  enum class Group : int
  {
    Triclinic,      // -1   hemiIpf
    Monoclinic,     // 2/m  cyclicIpf<2>
    Orthorhombic,   // mmm  dihedralIpf<2>
    TrigonalLow,    // -3   cyclicIpf<3>
    TrigonalHigh,   // -3m  dihedralIpf<3>
    TetragonalLow,  // 4/m   cyclicIpf<4>
    TetragonalHigh, // 4/mmm dihedralIpf<4>
    HexagonalLow,   // 6/m   cyclicIpf<6>
    HexagonalHigh,  // 6/mmm dihedralIpf<6>
    CubicLow,       // m-3   cubicLowIpf
    CubicHigh       // m-3m  cubicIpf
  };

  Group m_Group;
  std::string m_RotationPointGroup;
};

} // namespace ebsdlib
