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
#include <iostream>
#include <numeric>
#include <string>
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
