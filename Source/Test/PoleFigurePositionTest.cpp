/* ============================================================================
 * PoleFigurePositionTest
 *
 * Position-space pole-figure validation against MTEX (v3 plan §P3.1).
 *
 * For each ideal canonical orientation × each unique Laue class × each
 * default plane family, this test computes every symmetry-equivalent crystal
 * direction in the sample frame, stereographic-projects it to (x, y) on the
 * unit disk, emits one CSV row per pole, and then compares the result
 * against a committed MTEX-generated golden CSV.
 *
 * The methodology, the per-Laue-class convention table, and the regeneration
 * procedure are documented in Data/Pole_Figure_Validation/ReadMe.md. The
 * companion MATLAB script that produces the golden lives there too.
 * ============================================================================ */
#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "UnitTestSupport.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace ebsdlib;

namespace
{
struct CanonicalOrientation
{
  std::string name;
  double phi1Deg;
  double phiDeg;
  double phi2Deg;
};

// Reference Bunge angles (degrees) — centers of the EMsoftSO3Sampler clouds
// in Data/Pole_Figure_Inputs/pole_figure_euler_data.dream3d.
const std::vector<CanonicalOrientation> k_CanonicalOrientations = {
    {"Cube", 0.0, 0.0, 0.0},  {"Goss", 0.0, 45.0, 0.0}, {"Brass", 35.0, 45.0, 0.0}, {"Copper", 90.0, 35.0, 45.0}, {"S", 59.0, 37.0, 63.0},    {"S1", 55.0, 30.0, 65.0},
    {"S2", 45.0, 35.0, 65.0}, {"R", 55.0, 75.0, 25.0},  {"RC_rd1", 0.0, 20.0, 0.0}, {"RC_rd2", 0.0, 35.0, 0.0},   {"RC_nd1", 20.0, 0.0, 0.0}, {"RC_nd2", 35.0, 0.0, 0.0},
};

// Stereographic projection of a unit-sphere direction onto the unit disk.
// Lower-hemisphere directions are folded antipodally (matches the existing
// PF rendering pipeline in ComputeStereographicProjection).
std::pair<double, double> projectStereographic(double x, double y, double z)
{
  if(z < 0.0)
  {
    x = -x;
    y = -y;
    z = -z;
  }
  // After folding, z is in [0, 1] so (1 + z) is in [1, 2] — no divide-by-zero.
  return {x / (1.0 + z), y / (1.0 + z)};
}

// Pre-compute the indices into LaueOps::GetAllOrientationOps() that uniquely
// represent each Laue class (one ops per rotationPointGroup).
std::vector<size_t> uniqueLaueOpsIndices(const std::vector<LaueOps::Pointer>& ops)
{
  std::vector<size_t> result;
  std::set<std::string> seen;
  for(size_t i = 0; i < ops.size(); ++i)
  {
    const std::string rpg = ops[i]->getRotationPointGroup();
    if(seen.insert(rpg).second)
    {
      result.push_back(i);
    }
  }
  return result;
}

bool nearlyEqual(double a, double b, double tol = 1.0e-5)
{
  return std::fabs(a - b) <= tol;
}

// Stereographic projection of upper-hemisphere unit vectors lands inside the
// unit disk; equator points (z = 0) land on the boundary circle. The
// `if z < 0 ... flip` rule used by both EbsdLib and MTEX is FP-unstable at
// z = 0, so antipodal pairs (+v, -v) on the equator can land at either
// (x, y) or (-x, -y) depending on which side of zero a rounding error lands.
// Both representations refer to the same crystallographic direction. We fold
// equator points to a canonical antipode before comparison: prefer y > 0,
// ties broken by x > 0. Interior points are untouched.
std::pair<double, double> canonicalizeEquator(double x, double y, double equatorEps = 1.0e-5)
{
  const double r2 = x * x + y * y;
  const double thresh = (1.0 - equatorEps) * (1.0 - equatorEps);
  if(r2 < thresh)
  {
    return {x, y};
  }
  if(y < -equatorEps)
  {
    return {-x, -y};
  }
  if(std::fabs(y) < equatorEps && x < 0.0)
  {
    return {-x, -y};
  }
  return {x, y};
}

// (orient_id, rotation_point_group, plane_family) -> list of (x, y).
using BucketKey = std::tuple<int, std::string, std::string>;
using BucketMap = std::map<BucketKey, std::vector<std::pair<double, double>>>;

// Parse the MTEX golden CSV. Schema must match what the test emits and what
// mtex_pole_figure_positions.m emits:
//   orient_id, orient_name, rotation_point_group, symmetry_name, plane_family, x, y
BucketMap parseGoldenCsv(const std::string& path)
{
  BucketMap out;
  std::ifstream f(path);
  if(!f.is_open())
  {
    return out;
  }
  std::string line;
  std::getline(f, line); // header
  while(std::getline(f, line))
  {
    if(line.empty())
    {
      continue;
    }
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string cell;
    while(std::getline(ss, cell, ','))
    {
      cols.push_back(cell);
    }
    if(cols.size() < 7)
    {
      continue;
    }
    const int orientId = std::stoi(cols[0]);
    const std::string& rpg = cols[2];
    const std::string& planeFamily = cols[4];
    const double x = std::stod(cols[5]);
    const double y = std::stod(cols[6]);
    auto canon = canonicalizeEquator(x, y);
    out[{orientId, rpg, planeFamily}].emplace_back(canon.first, canon.second);
  }
  return out;
}

// Greedy nearest-neighbor matcher within one bucket. Bucket sizes are at
// most ~24, so brute-force O(N^2) is fine. Returns the worst matched
// distance plus the offending pair (for diagnostic output on failure).
struct BucketMatchResult
{
  double maxDistance = 0.0;
  std::pair<double, double> ebPoint{0.0, 0.0};
  std::pair<double, double> mtPoint{0.0, 0.0};
  bool sizeMismatch = false;
};

BucketMatchResult greedyMatch(const std::vector<std::pair<double, double>>& a, const std::vector<std::pair<double, double>>& b)
{
  BucketMatchResult result;
  if(a.size() != b.size())
  {
    result.sizeMismatch = true;
    return result;
  }
  std::vector<bool> used(b.size(), false);
  for(const auto& ap : a)
  {
    double bestD = std::numeric_limits<double>::infinity();
    size_t bestJ = 0;
    for(size_t j = 0; j < b.size(); ++j)
    {
      if(used[j])
      {
        continue;
      }
      const double dx = ap.first - b[j].first;
      const double dy = ap.second - b[j].second;
      const double d = std::sqrt(dx * dx + dy * dy);
      if(d < bestD)
      {
        bestD = d;
        bestJ = j;
      }
    }
    used[bestJ] = true;
    if(bestD > result.maxDistance)
    {
      result.maxDistance = bestD;
      result.ebPoint = ap;
      result.mtPoint = b[bestJ];
    }
  }
  return result;
}
} // namespace

// -----------------------------------------------------------------------------
// Emit EbsdLib pole-figure positions for one hexagonal/trigonal Cartesian
// convention and validate every bucket (orientation x Laue class x plane
// family) against the matching MTEX golden CSV.
//
// The convention only changes the hex/trig families (a 30 deg basis rotation
// about c); for all other Laue classes the convention is ignored, so those
// buckets are identical between conventions. Each convention therefore needs
// its own MTEX golden, generated by Data/Pole_Figure_Validation/
// mtex_pole_figure_positions.m with the matching crystalSymmetry alignment.
// -----------------------------------------------------------------------------
static void validatePositionsAgainstMtex(const std::vector<LaueOps::Pointer>& ops, const std::vector<size_t>& uniqueOpsIndices, ebsdlib::HexConvention conv, const std::string& goldenPath,
                                         const std::string& ebCsvPath)
{
  REQUIRE(EnsureParentDirectoryExists(ebCsvPath));

  std::ofstream csv(ebCsvPath);
  REQUIRE(csv.is_open());
  csv << "orient_id,orient_name,rotation_point_group,symmetry_name,plane_family,x,y\n";
  csv << std::fixed;
  csv.precision(8);

  // Per-bucket (canonicalized) point list, populated during the emission
  // loop and compared against the MTEX golden after.
  BucketMap ebMap;

  for(size_t orientId = 0; orientId < k_CanonicalOrientations.size(); ++orientId)
  {
    const CanonicalOrientation& canon = k_CanonicalOrientations[orientId];
    std::vector<float> eulerVec = {static_cast<float>(canon.phi1Deg * ebsdlib::constants::k_PiOver180D), static_cast<float>(canon.phiDeg * ebsdlib::constants::k_PiOver180D),
                                   static_cast<float>(canon.phi2Deg * ebsdlib::constants::k_PiOver180D)};
    ebsdlib::FloatArrayType::Pointer eulersArr = ebsdlib::FloatArrayType::FromStdVector(eulerVec, 1ULL, 3ULL, "Eulers");

    for(size_t opsIndex : uniqueOpsIndices)
    {
      LaueOps::Pointer op = ops[opsIndex];
      const std::string rpg = op->getRotationPointGroup();
      const std::string symName = op->getSymmetryName();
      const std::array<int32_t, 3> symSizes = op->getNumSymmetry();
      const std::array<std::string, 3> familyNames = op->getDefaultPoleFigureNames(conv);

      // The three output buffers grow to 1 * symSize_i tuples each.
      std::vector<size_t> dims = {3ULL};
      ebsdlib::FloatArrayType::Pointer xyz0 = ebsdlib::FloatArrayType::CreateArray(static_cast<size_t>(symSizes[0]), dims, "xyz0", true);
      ebsdlib::FloatArrayType::Pointer xyz1 = ebsdlib::FloatArrayType::CreateArray(static_cast<size_t>(symSizes[1]), dims, "xyz1", true);
      ebsdlib::FloatArrayType::Pointer xyz2 = ebsdlib::FloatArrayType::CreateArray(static_cast<size_t>(symSizes[2]), dims, "xyz2", true);
      op->generateSphereCoordsFromEulers(eulersArr.get(), xyz0.get(), xyz1.get(), xyz2.get(), conv);

      ebsdlib::FloatArrayType* buffers[3] = {xyz0.get(), xyz1.get(), xyz2.get()};
      for(int family = 0; family < 3; ++family)
      {
        ebsdlib::FloatArrayType* buf = buffers[family];
        const int32_t symSize = symSizes[family];
        for(int32_t s = 0; s < symSize; ++s)
        {
          float* p = buf->getPointer(static_cast<size_t>(s) * 3);
          auto [px, py] = projectStereographic(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
          csv << orientId << "," << canon.name << "," << rpg << "," << symName << "," << familyNames[family] << "," << px << "," << py << "\n";

          // Populate the canonicalized per-bucket map for the MTEX comparison.
          auto canonPt = canonicalizeEquator(px, py);
          ebMap[{static_cast<int>(orientId), rpg, familyNames[family]}].emplace_back(canonPt.first, canonPt.second);
        }
      }
    }
  }
  csv.close();
  std::cout << "Wrote " << ebCsvPath << std::endl;

  // ---------------------------------------------------------------------------
  // Compare every bucket against the MTEX golden CSV. Tolerance of 1e-5 is
  // comfortably above float32 precision noise (the EbsdLib path stores
  // sphere coords as float; MTEX writes to 8 decimals) and well below any
  // crystallographic discrepancy worth investigating.
  // ---------------------------------------------------------------------------
  std::cout << "Loading MTEX golden: " << goldenPath << std::endl;
  BucketMap mtexMap = parseGoldenCsv(goldenPath);
  REQUIRE(!mtexMap.empty());

  // Bucket-key parity: every bucket in EbsdLib must be in the golden, and
  // vice versa. A missing key indicates a Laue-class or plane-family change
  // on one side.
  for(const auto& [key, pts] : ebMap)
  {
    INFO("EbsdLib emitted bucket missing from MTEX golden: orient_id=" << std::get<0>(key) << ", rpg=" << std::get<1>(key) << ", family=" << std::get<2>(key));
    REQUIRE(mtexMap.find(key) != mtexMap.end());
  }
  for(const auto& [key, pts] : mtexMap)
  {
    INFO("MTEX golden has bucket missing from EbsdLib output: orient_id=" << std::get<0>(key) << ", rpg=" << std::get<1>(key) << ", family=" << std::get<2>(key));
    REQUIRE(ebMap.find(key) != ebMap.end());
  }

  constexpr double k_BucketTol = 1.0e-5;
  double globalMax = 0.0;
  size_t bucketsCompared = 0;
  for(const auto& [key, ebPts] : ebMap)
  {
    const auto& mtPts = mtexMap.at(key);
    BucketMatchResult m = greedyMatch(ebPts, mtPts);
    bucketsCompared++;

    INFO("Bucket point-count mismatch: orient_id=" << std::get<0>(key) << ", rpg=" << std::get<1>(key) << ", family=" << std::get<2>(key) << ", ebsdlib=" << ebPts.size() << ", mtex=" << mtPts.size());
    REQUIRE_FALSE(m.sizeMismatch);

    INFO("Bucket exceeds tolerance: orient_id=" << std::get<0>(key) << ", rpg=" << std::get<1>(key) << ", family=" << std::get<2>(key) << ", max_d=" << m.maxDistance << ", worst pair: ebsdlib=("
                                                << m.ebPoint.first << ", " << m.ebPoint.second << ") -> mtex=(" << m.mtPoint.first << ", " << m.mtPoint.second << ")");
    REQUIRE(m.maxDistance < k_BucketTol);

    if(m.maxDistance > globalMax)
    {
      globalMax = m.maxDistance;
    }
  }
  std::cout << "Compared " << bucketsCompared << " buckets against MTEX golden; worst max-distance = " << globalMax << " (tolerance = " << k_BucketTol << ")" << std::endl;
}

TEST_CASE("ebsdlib::PoleFigurePositionTest::EmitCsv", "[EbsdLib][PoleFigurePositionTest]")
{
  const std::string baseDir = fmt::format("{}PoleFigurePositions", ebsdlib::unit_test::k_TestTempDir);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  std::vector<size_t> uniqueOpsIndices = uniqueLaueOpsIndices(ops);

  // ---------------------------------------------------------------------------
  // Hand-verifiable, convention-independent sanity check: Cube (Bunge 0,0,0)
  // under m-3m {001} produces the six crystal directions [±100], [0±10],
  // [0,0,±1]. After stereographic projection with antipodal folding, those land
  // at (1,0), (-1,0), (0,1), (0,-1), (0,0), (0,0) (the [001]/[00-1] pair both
  // fold to the disk center). Cubic ignores the hex convention.
  // ---------------------------------------------------------------------------
  {
    LaueOps::Pointer cubic;
    for(size_t i : uniqueOpsIndices)
    {
      if(ops[i]->getRotationPointGroup() == "432")
      {
        cubic = ops[i];
        break;
      }
    }
    REQUIRE(cubic != nullptr);

    std::vector<float> eulerVec = {0.0F, 0.0F, 0.0F};
    ebsdlib::FloatArrayType::Pointer eulersArr = ebsdlib::FloatArrayType::FromStdVector(eulerVec, 1ULL, 3ULL, "Eulers");
    const std::array<int32_t, 3> symSizes = cubic->getNumSymmetry();
    std::vector<size_t> dims = {3ULL};
    ebsdlib::FloatArrayType::Pointer xyz0 = ebsdlib::FloatArrayType::CreateArray(static_cast<size_t>(symSizes[0]), dims, "xyz0", true);
    ebsdlib::FloatArrayType::Pointer xyz1 = ebsdlib::FloatArrayType::CreateArray(static_cast<size_t>(symSizes[1]), dims, "xyz1", true);
    ebsdlib::FloatArrayType::Pointer xyz2 = ebsdlib::FloatArrayType::CreateArray(static_cast<size_t>(symSizes[2]), dims, "xyz2", true);
    cubic->generateSphereCoordsFromEulers(eulersArr.get(), xyz0.get(), xyz1.get(), xyz2.get(), ebsdlib::HexConvention::NotApplicable);

    std::vector<std::pair<double, double>> cubeM3mFamily0;
    for(int32_t s = 0; s < symSizes[0]; ++s)
    {
      float* p = xyz0->getPointer(static_cast<size_t>(s) * 3);
      cubeM3mFamily0.emplace_back(projectStereographic(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2])));
    }
    REQUIRE(cubeM3mFamily0.size() == 6);

    const std::vector<std::pair<double, double>> expected = {{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}, {0.0, 0.0}, {0.0, 0.0}};

    // Match by greedy nearest-neighbor — the order in which generateSphereCoords
    // emits poles is a private implementation detail we shouldn't pin to.
    std::vector<bool> matched(expected.size(), false);
    for(const auto& got : cubeM3mFamily0)
    {
      bool foundMatch = false;
      for(size_t e = 0; e < expected.size(); ++e)
      {
        if(matched[e])
        {
          continue;
        }
        if(nearlyEqual(got.first, expected[e].first) && nearlyEqual(got.second, expected[e].second))
        {
          matched[e] = true;
          foundMatch = true;
          break;
        }
      }
      INFO("Cube/m-3m/{001} pole (" << got.first << ", " << got.second << ") had no match in expected set");
      REQUIRE(foundMatch);
    }
    REQUIRE(std::all_of(matched.begin(), matched.end(), [](bool b) { return b; }));
  }

  // ---------------------------------------------------------------------------
  // X||a* (MTEX/Oxford) — the long-standing golden. Always validated.
  // ---------------------------------------------------------------------------
  validatePositionsAgainstMtex(ops, uniqueOpsIndices, ebsdlib::HexConvention::XParallelAStar, ebsdlib::unit_test::DataDir + "Pole_Figure_Validation/mtex_pole_figure_positions.csv",
                               fmt::format("{}/ebsdlib_pole_figure_positions.csv", baseDir));

  // ---------------------------------------------------------------------------
  // X||a (TSL/EDAX) — the 3.1 public default. Validated against a second MTEX
  // golden produced by the same script with the X||a alignment. Until that
  // golden is generated + committed, skip with a loud warning rather than fail
  // so the suite stays green during the release stabilization window.
  // ---------------------------------------------------------------------------
  const std::string xaGolden = ebsdlib::unit_test::DataDir + "Pole_Figure_Validation/mtex_pole_figure_positions_xa.csv";
  if(std::filesystem::exists(xaGolden))
  {
    validatePositionsAgainstMtex(ops, uniqueOpsIndices, ebsdlib::HexConvention::XParallelA, xaGolden, fmt::format("{}/ebsdlib_pole_figure_positions_xa.csv", baseDir));
  }
  else
  {
    WARN("X||a MTEX golden not found (" << xaGolden
                                        << "); skipping X||a position validation. Generate it by running "
                                           "Data/Pole_Figure_Validation/mtex_pole_figure_positions.m with the X||a alignment.");
  }
}
