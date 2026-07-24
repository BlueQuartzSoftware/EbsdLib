#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Texture/RandomAngleDistribution.h"

#include <numeric>
#include <vector>

using namespace ebsdlib;

namespace
{
// -----------------------------------------------------------------------------
// RandomAngleDistribution is a direct port of MTEX 6.1.0's
// geometry/@symmetry/calcAngleDistribution.m (lines 44-218). The reference
// constants below were generated with:
//
//   names={'6/mmm','m-3m','6/m','m-3','-1','2/m','mmm','4/m','4/mmm','-3','-3m'};
//   for i=1:numel(names)
//     cs=crystalSymmetry(names{i});
//     [ad,om]=calcAngleDistribution(cs);
//     oR=fundamentalRegion(cs);
//     fprintf('IDX %d NAME %s MAXANGLE %.12f\n', i-1, names{i}, oR.maxAngle);
//     idx=[1 25 50 75 100 125 150 175 200];
//     for k=idx
//       fprintf('  P %d %.12f %.12f\n', k, om(k), ad(k));
//     end
//   end
//
// The crystal-structure-index -> MTEX point-group-name map used below was
// cross-checked against EbsdLib/Core/EbsdLibConstants.h's CrystalStructure
// enum values and the ordering of LaueOps::GetAllOrientationOps():
//   0 Hexagonal_High  -> 6/mmm
//   1 Cubic_High      -> m-3m
//   2 Hexagonal_Low   -> 6/m
//   3 Cubic_Low       -> m-3
//   4 Triclinic       -> -1
//   5 Monoclinic      -> 2/m
//   6 OrthoRhombic    -> mmm
//   7 Tetragonal_Low  -> 4/m
//   8 Tetragonal_High -> 4/mmm
//   9 Trigonal_Low    -> -3
//   10 Trigonal_High  -> -3m

struct SamplePoint
{
  int index; // 1-based MATLAB index into the 200-point linspace
  double omega;
  double ad;
};

void CheckDistribution(uint32_t crystalStructure, double maxAngle, const std::vector<SamplePoint>& samples)
{
  REQUIRE(random_angle_distribution::MaxMisorientationAngle(crystalStructure) == Approx(maxAngle).epsilon(1.0e-6));

  std::vector<double> omega(200);
  for(size_t i = 0; i < omega.size(); i++)
  {
    omega[i] = maxAngle * static_cast<double>(i) / static_cast<double>(omega.size() - 1);
  }

  std::vector<double> ad = random_angle_distribution::Compute(crystalStructure, omega);
  REQUIRE(ad.size() == omega.size());

  for(const auto& sample : samples)
  {
    const size_t i = static_cast<size_t>(sample.index - 1);
    REQUIRE(omega[i] == Approx(sample.omega).epsilon(1.0e-6));
    // margin() handles the omega==maxAngle samples where the expected value
    // is exactly 0.0: Approx's relative epsilon alone requires an exact
    // match against 0 (scale defaults to 0), but rmag=tan(omega/2) sits
    // right at its pole there, leaving ~1e-13 floating-point noise.
    REQUIRE(ad[i] == Approx(sample.ad).epsilon(1.0e-6).margin(1.0e-9));
  }

  const double mean = std::accumulate(ad.cbegin(), ad.cend(), 0.0) / static_cast<double>(ad.size());
  REQUIRE(mean == Approx(1.0).epsilon(1.0e-6));
}
} // namespace

TEST_CASE("RandomAngleDistribution matches MTEX for m-3m", "[RandomAngleDistribution]")
{
  CheckDistribution(ebsdlib::CrystalStructure::Cubic_High, 1.096056815241,
                    {
                        {1, 0.000000000000, 0.000000000000},
                        {25, 0.132187756612, 0.073415322873},
                        {50, 0.269883336416, 0.304614948103},
                        {75, 0.407578916220, 0.689349878258},
                        {100, 0.545274496024, 1.220337029329},
                        {125, 0.682970075828, 1.887524743076},
                        {150, 0.820665655632, 2.293864296709},
                        {175, 0.958361235437, 1.401263125507},
                        {200, 1.096056815241, 0.000000000000},
                    });
}

TEST_CASE("RandomAngleDistribution matches MTEX for 6/mmm", "[RandomAngleDistribution]")
{
  CheckDistribution(ebsdlib::CrystalStructure::Hexagonal_High, 1.637833825000,
                    {
                        {1, 0.000000000000, 0.000000000000},
                        {25, 0.197527697487, 0.122258897932},
                        {50, 0.403285715703, 0.504392451654},
                        {75, 0.609043733920, 0.963781985348},
                        {100, 0.814801752136, 1.225761427732},
                        {125, 1.020559770352, 1.436029416921},
                        {150, 1.226317788568, 1.585715333366},
                        {175, 1.432075806784, 1.668504346069},
                        {200, 1.637833825000, 0.000000000000},
                    });
}

TEST_CASE("RandomAngleDistribution matches MTEX for -3", "[RandomAngleDistribution]")
{
  CheckDistribution(ebsdlib::CrystalStructure::Trigonal_Low, 3.141592653590,
                    {
                        {1, 0.000000000000, 0.000000000000},
                        {25, 0.378885546162, 0.213840156584},
                        {50, 0.773557990080, 0.858015264909},
                        {75, 1.168230433998, 1.601620985803},
                        {100, 1.562902877917, 1.740726967230},
                        {125, 1.957575321835, 1.612187686555},
                        {150, 2.352247765753, 1.235766685169},
                        {175, 2.746920209671, 0.669340528134},
                        {200, 3.141592653590, 0.000000000000},
                    });
}

TEST_CASE("RandomAngleDistribution matches MTEX for 2/m", "[RandomAngleDistribution]")
{
  CheckDistribution(ebsdlib::CrystalStructure::Monoclinic, 3.141592653590,
                    {
                        {1, 0.000000000000, 0.000000000000},
                        {25, 0.378885546162, 0.142560751959},
                        {50, 0.773557990080, 0.572012774922},
                        {75, 1.168230433998, 1.222576203885},
                        {100, 1.562902877917, 1.994223738334},
                        {125, 1.957575321835, 1.861602445795},
                        {150, 2.352247765753, 1.426946938453},
                        {175, 2.746920209671, 0.772891378985},
                        {200, 3.141592653590, 0.000000000000},
                    });
}

TEST_CASE("RandomAngleDistribution throws on unknown crystal structure", "[RandomAngleDistribution]")
{
  const std::vector<double> omega{0.0, 0.5, 1.0};
  REQUIRE_THROWS_AS(random_angle_distribution::Compute(ebsdlib::CrystalStructure::UnknownCrystalStructure, omega), std::invalid_argument);
  REQUIRE_THROWS_AS(random_angle_distribution::MaxMisorientationAngle(ebsdlib::CrystalStructure::UnknownCrystalStructure), std::invalid_argument);
}
