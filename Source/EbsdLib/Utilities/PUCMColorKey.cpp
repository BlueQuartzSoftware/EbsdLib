#include "EbsdLib/Utilities/PUCMColorKey.hpp"

// Vendored wlenthe header has -Wshadow tripwires (e.g. local arrays
// named 'n' inside cubicToHemi where the enclosing function parameter
// is also 'n'). Suppress just for the include so we can keep the
// upstream file byte-identical for future re-syncs.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wshadow"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#endif
#include "EbsdLib/Utilities/wlenthe_orientation_coloring.hpp"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ebsdlib
{

namespace
{
PUCMColorKey::Vec3 dispatchPucm(int group, const PUCMColorKey::Vec3& direction)
{
  // Stack copies because the wlenthe API takes raw pointers.
  double n[3] = {direction[0], direction[1], direction[2]};
  double rgb[3] = {0.0, 0.0, 0.0};

  switch(static_cast<int>(group))
  {
  case 0: // Triclinic
    coloring::hemiIpf<double>(n, rgb);
    break;
  case 1: // Monoclinic 2/m
    coloring::cyclicIpf<double, 2>(n, rgb);
    break;
  case 2: // Orthorhombic mmm
    coloring::dihedralIpf<double, 2>(n, rgb);
    break;
  case 3: // Trigonal -3
    coloring::cyclicIpf<double, 3>(n, rgb);
    break;
  case 4: // Trigonal -3m
    coloring::dihedralIpf<double, 3>(n, rgb);
    break;
  case 5: // Tetragonal 4/m
    coloring::cyclicIpf<double, 4>(n, rgb);
    break;
  case 6: // Tetragonal 4/mmm
    coloring::dihedralIpf<double, 4>(n, rgb);
    break;
  case 7: // Hexagonal 6/m
    coloring::cyclicIpf<double, 6>(n, rgb);
    break;
  case 8: // Hexagonal 6/mmm
    coloring::dihedralIpf<double, 6>(n, rgb);
    break;
  case 9: // Cubic m-3
    coloring::cubicLowIpf<double>(n, rgb);
    break;
  case 10: // Cubic m-3m
    coloring::cubicIpf<double>(n, rgb);
    break;
  default:
    return {0.0, 0.0, 0.0};
  }
  return {rgb[0], rgb[1], rgb[2]};
}

int groupFromRotationPointGroup(const std::string& rpg)
{
  if(rpg == "1")
    return 0;
  if(rpg == "2")
    return 1;
  if(rpg == "222")
    return 2;
  if(rpg == "3")
    return 3;
  if(rpg == "32")
    return 4;
  if(rpg == "4")
    return 5;
  if(rpg == "422")
    return 6;
  if(rpg == "6")
    return 7;
  if(rpg == "622")
    return 8;
  if(rpg == "23")
    return 9;
  if(rpg == "432")
    return 10;
  throw std::invalid_argument("PUCMColorKey: unsupported rotation point group '" + rpg + "'");
}
} // namespace

PUCMColorKey::PUCMColorKey(const std::string& rotationPointGroup)
: m_Group(static_cast<Group>(groupFromRotationPointGroup(rotationPointGroup)))
, m_RotationPointGroup(rotationPointGroup)
{
  // The wlenthe coloring routines lazily populate static lookup tables
  // (cubicToHemi / cubicLowToHemi) on first call. Under ParallelDataAlgorithm
  // multiple worker threads race that init and corrupt the table, producing a
  // free_small_botch crash. Warm the per-group dispatch path once here so the
  // tables are fully populated before any concurrent reader can see them --
  // PUCMColorKey instances are themselves constructed under C++11
  // magic-statics locks (one per LaueOps subclass singleton), so this
  // construction-time warmup is serialized.
  const Vec3 k_WarmupDir = {0.5, 0.3, 0.7};
  (void)dispatchPucm(static_cast<int>(m_Group), k_WarmupDir);
}

PUCMColorKey::Vec3 PUCMColorKey::direction2Color(const Vec3& direction) const
{
  return dispatchPucm(static_cast<int>(m_Group), direction);
}

PUCMColorKey::Vec3 PUCMColorKey::direction2Color(double eta, double chi, const Vec3& angleLimits) const
{
  // PUCM operates on a Cartesian crystal direction directly. Convert
  // (eta, chi) -> unit vector. angleLimits is unused — PUCM has its own
  // per-Laue-class fundamental-sector geometry baked into the dispatch.
  (void)angleLimits;
  const double s = std::sin(chi);
  const Vec3 dir = {s * std::cos(eta), s * std::sin(eta), std::cos(chi)};
  return direction2Color(dir);
}

std::string PUCMColorKey::name() const
{
  return "PUCM (" + m_RotationPointGroup + ")";
}

std::string PUCMColorKey::rotationPointGroup() const
{
  return m_RotationPointGroup;
}

} // namespace ebsdlib
