/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/LaueOps/CubicLowOps.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/HexagonalLowOps.h"
#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/LaueOps/MonoclinicOps.h"
#include "EbsdLib/LaueOps/OrthoRhombicOps.h"
#include "EbsdLib/LaueOps/TetragonalLowOps.h"
#include "EbsdLib/LaueOps/TetragonalOps.h"
#include "EbsdLib/LaueOps/TriclinicOps.h"
#include "EbsdLib/LaueOps/TrigonalLowOps.h"
#include "EbsdLib/LaueOps/TrigonalOps.h"
#include "EbsdLib/Math/Matrix3X3.hpp"
#include "EbsdLib/Texture/StatsGen.hpp"
#include "EbsdLib/Texture/Texture.hpp"

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

using namespace ebsdlib;
#if 0
template <class LaueOps>
void TestTextureMdf()
{
  LaueOps ops;
  std::cout << "======================================================" << std::endl;
  std::cout << ops.getNameOfClass() << " MDF Plot Values" << std::endl;

  int size = 10000;


  // Calculate the ODF Data
  using OdfValueType = double;
  using OdfContainerType = std::vector<OdfValueType>;

  std::cout << "   Generating ODF....." << std::endl;
  const Texture::ODFTableEntries odfTableEntries;

  OdfContainerType odf = Texture::CalculateODFData<OdfValueType, LaueOps, OdfContainerType>(odfTableEntries, true);

  // Allocate a new vector to hold the mdf data
  std::vector<float> mdf;
  int32_t err = 0;
  std::cout << "   Generating MDF....." << std::endl;
  try
  {
    // Calculate the MDF Data using the ODF data and the rows from the MDF Table model
    Texture::CalculateMDFData<float, LaueOps, std::vector<float>>(angles, axes, weights, odf, mdf, static_cast<size_t>(angles.size()));
    // Now generate the actual XY point data that gets plotted.
    // These are the output vectors

    int npoints = 36;
    std::vector<float> x(npoints);
    std::vector<float> y(npoints);
    std::cout << "   Generating MDF Plot Data....." << std::endl;

    err = StatsGen::GenMDFPlotData<float, LaueOps, std::vector<float>>(mdf, x, y, size);
    if(err < 0)
    {
      std::cout << "Error Generating MDF Plot Values" << std::endl;
      return;
    }
    std::cout << "    npoints: " << x.size() << std::endl;
    for(size_t i = 0; i < x.size(); i++)
    {
      std::cout << i << ": " << x[i] << ", " << y[i] << std::endl;
    }
  } catch([[maybe_unused]] const ebsdlib::method_not_implemented& exception)
  {
    std::cout << "   MDF Plot Values NOT implemented" << std::endl;
  }
}

TEST_CASE("ebsdlib::TextureTest::TestMdfGeneration", "[EbsdLib][TextureTest]")
{
  TestTextureMdf<CubicLowOps>();
  TestTextureMdf<CubicOps>();
  TestTextureMdf<HexagonalLowOps>();
  TestTextureMdf<HexagonalOps>();
  TestTextureMdf<TetragonalLowOps>();
  TestTextureMdf<TetragonalOps>();
  TestTextureMdf<TrigonalLowOps>();
  TestTextureMdf<TrigonalOps>();

  try
  {
    TestTextureMdf<TriclinicOps>();
  } catch(std::runtime_error e)
  {
  }
  try
  {
    TestTextureMdf<MonoclinicOps>();
  } catch(std::runtime_error e)
  {
  }
  try
  {
    TestTextureMdf<OrthoRhombicOps>();
  } catch(std::runtime_error e)
  {
  }
}

template <class LaueOps>
void TestTextureOdf()
{
  std::vector<float> e1s;
  std::vector<float> e2s;
  std::vector<float> e3s;
  std::vector<float> weights;
  std::vector<float> sigmas;
  std::vector<float> odf;

  LaueOps ops;
  size_t numEntries = e1s.size();
  Texture::CalculateODFData<float, LaueOps, std::vector<float>>(e1s, e2s, e3s, weights, sigmas, true, odf, numEntries);
}

TEST_CASE("ebsdlib::TextureTest::TestOdfGeneration", "[EbsdLib][TextureTest]")
{
  TestTextureOdf<CubicLowOps>();
  TestTextureOdf<CubicOps>();
  TestTextureOdf<HexagonalLowOps>();
  TestTextureOdf<HexagonalOps>();
  TestTextureOdf<MonoclinicOps>();
  TestTextureOdf<OrthoRhombicOps>();
  TestTextureOdf<TetragonalLowOps>();
  TestTextureOdf<TetragonalOps>();
  TestTextureOdf<TriclinicOps>();
  TestTextureOdf<TrigonalLowOps>();
  TestTextureOdf<TrigonalOps>();
}

TEST_CASE("ebsdlib::TextureTest::TestMatrix3X3", "[EbsdLib][TextureTest]")
{
  ebsdlib::Matrix3X3F matrix(1.0f, 2.0f, 3.0, 4.0f, 5.0f, 6.0f, 7.0, 8.0f, 9.0f);
  matrix[0] = 10.0f;
  matrix.data()[0] = 12.0f;
  matrix = matrix * matrix;
  matrix = matrix.multiplyInPlace(matrix);
  matrix = matrix + matrix;
  matrix = matrix - matrix;
  matrix = matrix * 22.0f;

  matrix = matrix.transpose();
  matrix = matrix.invert();
  matrix = matrix.adjoint();
  matrix = matrix.cofactor();
  matrix = matrix.minors();
  float det = matrix.determinant();
  matrix = matrix.normalize();
  matrix = ebsdlib::Matrix3X3F::Identity();
}

TEST_CASE("ebsdlib::TextureTest::DirectStructureMatrix", "[EbsdLib][DirectStructureMatrix]")
{
  std::array<double, 6> latticeParameters = {0.5, 0.5, 1.0, 90.0, 90.0, 90.0};

  Matrix3X3<double> dsm = Matrix3X3<double>::DirectStructureMatrix(latticeParameters);

  Matrix3X1<double> latticePoint(0.0, 0.0, 1.0);
  auto cartesian = dsm * latticePoint;
  std::cout << cartesian << std::endl;

  latticePoint = Matrix3X1<double>(1.0, 0.0, 0.0);
  std::cout << dsm * latticePoint << std::endl;

  latticePoint = Matrix3X1<double>(0.0, 1.0, 0.0);
  std::cout << dsm * latticePoint << std::endl;

  latticePoint = Matrix3X1<double>(0.0, 0.0, 1.0);
  std::cout << dsm * latticePoint << std::endl;
}

#endif

namespace
{
constexpr float k_Sigma3Angle = 60.0f * ebsdlib::constants::k_PiOver180F;
constexpr std::array<float, 3> k_Sigma3Axis = {1.0f, 1.0f, 1.0f};

std::vector<float> createUniformCubicOdf()
{
  const Texture::ODFTableEntries entries;
  return Texture::CalculateODFData<float, CubicOps, std::vector<float>>(entries, true);
}

int sigma3MdfBin()
{
  CubicOps ops;
  RodriguesDType rod = AxisAngleDType(k_Sigma3Axis[0], k_Sigma3Axis[1], k_Sigma3Axis[2], k_Sigma3Angle).toRodrigues();
  rod = ops.getMDFFZRod(rod);
  return ops.getMisoBin(rod);
}

std::vector<float> calculateCubicMdf(const std::vector<float>& inputWeights)
{
  std::vector<float> angles(inputWeights.size(), k_Sigma3Angle);
  std::vector<float> axes(inputWeights.size() * 3);
  for(size_t index = 0; index < inputWeights.size(); index++)
  {
    axes[index * 3] = k_Sigma3Axis[0];
    axes[index * 3 + 1] = k_Sigma3Axis[1];
    axes[index * 3 + 2] = k_Sigma3Axis[2];
  }

  std::vector<float> weights = inputWeights;
  const std::vector<float> odf = createUniformCubicOdf();
  std::vector<float> mdf;
  Texture::CalculateMDFData<float, CubicOps>(angles, axes, weights, odf, mdf, angles.size());
  return mdf;
}

std::vector<float> calculateSeededCubicMdf(const std::vector<float>& inputWeights, uint64_t seed)
{
  std::vector<float> angles(inputWeights.size(), k_Sigma3Angle);
  std::vector<float> axes(inputWeights.size() * 3);
  for(size_t index = 0; index < inputWeights.size(); index++)
  {
    axes[index * 3] = k_Sigma3Axis[0];
    axes[index * 3 + 1] = k_Sigma3Axis[1];
    axes[index * 3 + 2] = k_Sigma3Axis[2];
  }

  std::vector<float> weights = inputWeights;
  const std::vector<float> odf = createUniformCubicOdf();
  std::vector<float> mdf;
  std::mt19937_64 generator(seed);
  Texture::CalculateMDFData<float, CubicOps>(angles, axes, weights, odf, mdf, angles.size(), generator);
  return mdf;
}

void requireBitwiseEqual(const std::vector<float>& first, const std::vector<float>& second)
{
  REQUIRE(first.size() == second.size());
  for(size_t index = 0; index < first.size(); index++)
  {
    CAPTURE(index);
    REQUIRE(std::bit_cast<uint32_t>(first[index]) == std::bit_cast<uint32_t>(second[index]));
  }
}

float sumMdf(const std::vector<float>& mdf)
{
  return std::accumulate(mdf.cbegin(), mdf.cend(), 0.0f);
}
} // namespace

TEST_CASE("ebsdlib::TextureTest::CalculateMDFData normalizes weighted targets", "[EbsdLib][TextureTest]")
{
  const int targetBin = sigma3MdfBin();

  SECTION("One row reserves half of a cubic MDF")
  {
    const std::vector<float> mdf = calculateCubicMdf({2916.0f});
    REQUIRE(mdf[targetBin] == Approx(0.5f).margin(1.0e-6f));
    REQUIRE(sumMdf(mdf) == Approx(1.0f).margin(1.0e-5f));
  }

  SECTION("An overflowing row clamps the reserved mass")
  {
    const std::vector<float> mdf = calculateCubicMdf({500000.0f});
    REQUIRE(mdf[targetBin] == Approx(1.0f).margin(1.0e-6f));
    REQUIRE(sumMdf(mdf) == Approx(1.0f).margin(1.0e-5f));
    REQUIRE(std::none_of(mdf.cbegin(), mdf.cend(), [](float value) { return value < 0.0f; }));
  }

  SECTION("Duplicate rows accumulate in the folded bin")
  {
    const std::vector<float> mdf = calculateCubicMdf({1458.0f, 1458.0f});
    REQUIRE(mdf[targetBin] == Approx(0.5f).margin(1.0e-6f));
    REQUIRE(sumMdf(mdf) == Approx(1.0f).margin(1.0e-5f));
  }

  SECTION("Empty weights produce a random normalized MDF")
  {
    const std::vector<float> mdf = calculateCubicMdf({});
    REQUIRE(sumMdf(mdf) == Approx(1.0f).margin(1.0e-5f));
  }
}

TEST_CASE("ebsdlib::TextureTest::CalculateMDFData seeded generator is reproducible", "[EbsdLib][TextureTest]")
{
  constexpr uint64_t k_Seed = 0x5EED1234ULL;

  SECTION("Empty weights")
  {
    const std::vector<float> first = calculateSeededCubicMdf({}, k_Seed);
    const std::vector<float> second = calculateSeededCubicMdf({}, k_Seed);
    requireBitwiseEqual(first, second);
    REQUIRE(sumMdf(first) == Approx(1.0f).margin(1.0e-5f));
  }

  SECTION("Weighted target")
  {
    const std::vector<float> first = calculateSeededCubicMdf({2916.0f}, k_Seed);
    const std::vector<float> second = calculateSeededCubicMdf({2916.0f}, k_Seed);
    requireBitwiseEqual(first, second);
    REQUIRE(sumMdf(first) == Approx(1.0f).margin(1.0e-5f));
  }

  SECTION("Legacy overload")
  {
    const std::vector<float> legacy = calculateCubicMdf({});
    REQUIRE_FALSE(legacy.empty());
    REQUIRE(sumMdf(legacy) == Approx(1.0f).margin(1.0e-5f));
  }

  SECTION("Seeded plot data")
  {
    std::vector<float> firstMdf = calculateSeededCubicMdf({}, k_Seed);
    std::vector<float> secondMdf = firstMdf;
    std::vector<float> firstAngles;
    std::vector<float> firstFrequencies;
    std::vector<float> secondAngles;
    std::vector<float> secondFrequencies;
    std::mt19937_64 firstGenerator(k_Seed);
    std::mt19937_64 secondGenerator(k_Seed);
    REQUIRE(StatsGen::GenMDFPlotData<float, CubicOps>(firstMdf, firstAngles, firstFrequencies, 100000, firstGenerator) == 0);
    REQUIRE(StatsGen::GenMDFPlotData<float, CubicOps>(secondMdf, secondAngles, secondFrequencies, 100000, secondGenerator) == 0);
    requireBitwiseEqual(firstAngles, secondAngles);
    requireBitwiseEqual(firstFrequencies, secondFrequencies);
  }
}

TEST_CASE("ebsdlib::TextureTest::UniformOdfExcludesUnreachableBins", "[EbsdLib][TextureTest]")
{
  const auto odf = Texture::CalculateODFData<double, TriclinicOps, std::vector<double>>({}, true);
  CHECK(odf.front() == 0.0);
  CHECK(odf.back() == 0.0);
  CHECK(std::accumulate(odf.begin(), odf.end(), 0.0) == Approx(1.0));
}

TEMPLATE_TEST_CASE("ebsdlib::TextureTest::UniformOdfSamplingHasNoClampFallbacks", "[EbsdLib][TextureTest][OdfInBall]", HexagonalOps, CubicOps, HexagonalLowOps, CubicLowOps, TriclinicOps,
                   MonoclinicOps, OrthoRhombicOps, TetragonalLowOps, TetragonalOps, TrigonalLowOps, TrigonalOps)
{
  TestType ops;
  const auto odf = Texture::CalculateODFData<double, TestType, std::vector<double>>({}, true);
  std::mt19937_64 generator(5489);
  std::discrete_distribution<int> selectBin(odf.begin(), odf.end());
  std::uniform_real_distribution<double> offset(0.0, 1.0);
  size_t nonFiniteCount = 0;
  size_t above1799Count = 0;
  size_t above180Count = 0;
  ops.resetClampFallbackCount();
  constexpr size_t k_DrawCount = 200000;
  for(size_t sampleIdx = 0; sampleIdx < k_DrawCount; ++sampleIdx)
  {
    const int bin = selectBin(generator);
    double random[3] = {offset(generator), offset(generator), offset(generator)};
    const auto euler = ops.determineEulerAngles(random, bin);
    if(!std::isfinite(euler[0]) || !std::isfinite(euler[1]) || !std::isfinite(euler[2]))
    {
      ++nonFiniteCount;
    }
    if constexpr(std::is_same_v<TestType, TriclinicOps>)
    {
      const double angle = euler.toAxisAngle()[3] * 180.0 / constants::k_PiD;
      nonFiniteCount += !std::isfinite(angle);
      above1799Count += angle > 179.9;
      above180Count += angle > 180.0;
    }
  }
  std::cout << "ODF sampling " << ops.getNameOfClass() << ": draws=" << k_DrawCount << " clamp_fallbacks=" << ops.clampFallbackCount() << " non_finite=" << nonFiniteCount << std::endl;
  if constexpr(std::is_same_v<TestType, TriclinicOps>)
  {
    std::cout << "Triclinic angles: above_179.9=" << above1799Count << " above_180.0=" << above180Count << std::endl;
    CHECK(above180Count == 0);
  }
  CHECK(nonFiniteCount == 0);
  CHECK(ops.clampFallbackCount() == 0);
}

namespace
{
/**
 * @struct OdfGridFixture
 * @brief Independent grid dimensions and expected bin counts for one Laue class.
 */
struct OdfGridFixture
{
  const char* className;
  std::array<size_t, 3> bins;
  std::array<double, 3> halfWidths;
  size_t nonzeroBins;
  size_t partialBins;
};

// These independent grid fixtures retain the sampler's homochoric dimensions,
// including the 0.7f coefficient for the Monoclinic first axis.
const std::array<OdfGridFixture, 11> k_OdfGridFixtures = {{
    {"HexagonalOps", {36, 36, 12}, {0.75366927563336727, 0.75366927563336727, 0.26060550051600867}, 15552, 0},
    {"CubicOps", {18, 18, 18}, {0.38867959478510306, 0.38867959478510306, 0.38867959478510306}, 5832, 0},
    {"HexagonalLowOps", {72, 72, 12}, {1.3306700394914688, 1.3306700394914688, 0.26060550051600867}, 49760, 3136},
    {"CubicLowOps", {36, 36, 36}, {0.75366927563336727, 0.75366927563336727, 0.75366927563336727}, 46656, 0},
    {"TriclinicOps", {72, 72, 72}, {1.3306700394914688, 1.3306700394914688, 1.3306700394914688}, 205704, 20336},
    {"MonoclinicOps", {72, 36, 72}, {1.3004169905036356, 0.75366927563336727, 1.3306700394914688}, 138808, 10136},
    {"OrthorhombicOps", {36, 36, 36}, {0.75366927563336727, 0.75366927563336727, 0.75366927563336727}, 46656, 0},
    {"TetragonalLowOps", {72, 72, 18}, {1.3306700394914688, 1.3306700394914688, 0.75366927563336727}, 68528, 6240},
    {"TetragonalOps", {36, 36, 18}, {0.75366927563336727, 0.75366927563336727, 0.38867959478510306}, 23328, 0},
    {"TrigonalLowOps", {72, 72, 24}, {1.3306700394914688, 1.3306700394914688, 0.26060550051600867}, 99384, 5976},
    {"TrigonalOps", {36, 36, 24}, {0.75366927563336727, 0.75366927563336727, 0.51410390002343753}, 31104, 0},
}};
} // namespace

TEMPLATE_TEST_CASE("ebsdlib::TextureTest::OdfInBallFractionsMatchVolume", "[EbsdLib][TextureTest][OdfInBall]", HexagonalOps, CubicOps, HexagonalLowOps, CubicLowOps, TriclinicOps, MonoclinicOps,
                   OrthoRhombicOps, TetragonalLowOps, TetragonalOps, TrigonalLowOps, TrigonalOps)
{
  TestType ops;
  const auto fixtureIter = std::find_if(k_OdfGridFixtures.begin(), k_OdfGridFixtures.end(), [&ops](const auto& fixture) { return fixture.className == ops.getNameOfClass(); });
  REQUIRE(fixtureIter != k_OdfGridFixtures.end());
  const auto& fixture = *fixtureIter;
  REQUIRE(ops.getOdfNumBins() == fixture.bins);
  CHECK(ops.odfBinInBallFraction(-1) == 0.0);
  CHECK(ops.odfBinInBallFraction(static_cast<int>(ops.getODFSize())) == 0.0);
  CHECK_FALSE(ops.isOdfBinReachable(-1));
  CHECK_FALSE(ops.isOdfBinReachable(static_cast<int>(ops.getODFSize())));

  const auto odf = Texture::CalculateODFData<double, TestType, std::vector<double>>({}, true);
  const auto floatOdf = Texture::CalculateODFData<float, TestType, std::vector<float>>({}, true);
  REQUIRE(odf.size() == ops.getODFSize());
  CHECK(std::accumulate(odf.begin(), odf.end(), 0.0) == Approx(1.0).margin(1.0e-10));
  CHECK(std::accumulate(floatOdf.begin(), floatOdf.end(), 0.0) == Approx(1.0).margin(1.0e-6));

  std::vector<int> sampleBins;
  for(size_t z : {size_t(0), fixture.bins[2] / 2, fixture.bins[2] - 1})
  {
    for(size_t y : {size_t(0), fixture.bins[1] / 2, fixture.bins[1] - 1})
    {
      for(size_t x : {size_t(0), fixture.bins[0] / 2, fixture.bins[0] - 1})
      {
        sampleBins.push_back(static_cast<int>((z * fixture.bins[1] + y) * fixture.bins[0] + x));
      }
    }
  }

  size_t zeroCount = 0;
  size_t partialCount = 0;
  size_t nonzeroCount = 0;
  size_t invalidCount = 0;
  double totalFraction = 0.0;
  std::array<size_t, 10> boundarySamples = {};
  std::vector<double> fractions(odf.size());
  for(size_t binIndex = 0; binIndex < odf.size(); ++binIndex)
  {
    const int bin = static_cast<int>(binIndex);
    const double fraction = ops.odfBinInBallFraction(bin);
    fractions[binIndex] = fraction;
    totalFraction += fraction;
    zeroCount += fraction == 0.0;
    partialCount += fraction > 0.0 && fraction < 1.0;
    nonzeroCount += odf[binIndex] > 0.0;
    invalidCount += !std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0;
    invalidCount += ops.isOdfBinReachable(bin) != (fraction > 0.0);
    invalidCount += (odf[binIndex] == 0.0) != (fraction == 0.0);
    if(fraction > 0.0 && fraction < 1.0)
    {
      const size_t bucket = static_cast<size_t>(fraction * 10.0);
      if(boundarySamples[bucket]++ < 4)
      {
        sampleBins.push_back(bin);
      }
    }
  }
  CHECK(invalidCount == 0);
  CHECK(nonzeroCount == odf.size() - zeroCount);
  CHECK(nonzeroCount == fixture.nonzeroBins);
  CHECK(partialCount == fixture.partialBins);
  std::cout << "ODF bins " << ops.getNameOfClass() << ": total=" << odf.size() << " zero=" << zeroCount << " partial=" << partialCount << " nonzero=" << nonzeroCount << std::endl;

  bool weightsMatch = true;
  for(size_t binIndex = 0; binIndex < odf.size(); ++binIndex)
  {
    weightsMatch = weightsMatch && std::abs(odf[binIndex] - fractions[binIndex] / totalFraction) < 1.0e-14;
  }
  CHECK(weightsMatch);

  if constexpr(std::is_same_v<TestType, CubicOps> || std::is_same_v<TestType, HexagonalOps> || std::is_same_v<TestType, CubicLowOps> || std::is_same_v<TestType, OrthoRhombicOps> ||
               std::is_same_v<TestType, TetragonalOps> || std::is_same_v<TestType, TrigonalOps>)
  {
    // The previous empty-entry calculation adds 1 to every bin, then divides by the bin count.
    const std::vector<double> previousOdf(ops.getODFSize(), 1.0 / static_cast<double>(ops.getODFSize()));
    const std::vector<float> previousFloatOdf(ops.getODFSize(), 1.0f / static_cast<float>(ops.getODFSize()));
    CHECK(std::memcmp(odf.data(), previousOdf.data(), odf.size() * sizeof(double)) == 0);
    CHECK(std::memcmp(floatOdf.data(), previousFloatOdf.data(), floatOdf.size() * sizeof(float)) == 0);
    CHECK(zeroCount == 0);
    CHECK(partialCount == 0);
  }

  std::mt19937_64 generator(20260923);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  constexpr size_t k_MonteCarloPoints = 2000;
  const double radius = std::cbrt(3.0 * constants::k_PiD / 4.0);
  for(int bin : sampleBins)
  {
    const size_t index = static_cast<size_t>(bin);
    const std::array<size_t, 3> cell = {index % fixture.bins[0], (index / fixture.bins[0]) % fixture.bins[1], index / (fixture.bins[0] * fixture.bins[1])};
    size_t insideCount = 0;
    for(size_t sampleIdx = 0; sampleIdx < k_MonteCarloPoints; ++sampleIdx)
    {
      double radiusSquared = 0.0;
      for(size_t axis = 0; axis < 3; ++axis)
      {
        const double coordinate = fixture.halfWidths[axis] * (2.0 * (static_cast<double>(cell[axis]) + unit(generator)) / static_cast<double>(fixture.bins[axis]) - 1.0);
        radiusSquared += coordinate * coordinate;
      }
      insideCount += radiusSquared <= radius * radius;
    }
    const double measured = static_cast<double>(insideCount) / static_cast<double>(k_MonteCarloPoints);
    INFO("class=" << ops.getNameOfClass() << " bin=" << bin << " Monte Carlo=" << measured);
    CHECK(fractions[index] == Approx(measured).margin(0.05));
  }
}

TEST_CASE("ebsdlib::TextureTest::HomochoricClampFallbackIsObservable", "[EbsdLib][TextureTest][OdfInBall]")
{
  TriclinicOps ops;
  ops.resetClampFallbackCount();
  double random[3] = {0.5, 0.5, 0.5};
  const auto euler = ops.determineEulerAngles(random, 0);
  CHECK(std::isfinite(euler[0]));
  CHECK(std::isfinite(euler[1]));
  CHECK(std::isfinite(euler[2]));
  CHECK(ops.clampFallbackCount() == 1);
  ops.resetClampFallbackCount();
  CHECK(ops.clampFallbackCount() == 0);
}
