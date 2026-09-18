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

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/ODFSectionUtilities.h"

#include <array>
#include <limits>
#include <vector>

using namespace ebsdlib;

TEST_CASE("ebsdlib::ODFSectionUtilities::LaueLimitsAndAngles", "[EbsdLib][ODFSectionUtilities]")
{
  const auto limits = GetODFEulerPlotLimits(CrystalStructure::Hexagonal_High);
  REQUIRE(limits.phi1MaxDeg == Approx(360.0));
  REQUIRE(limits.phiMaxDeg == Approx(90.0));
  REQUIRE(limits.phi2MaxDeg == Approx(60.0));
  REQUIRE(GenerateODFSectionAngles(CrystalStructure::Hexagonal_High, 6) == std::vector<double>{0.0, 10.0, 20.0, 30.0, 40.0, 50.0});
}

TEST_CASE("ebsdlib::ODFSectionUtilities::CountDensityToMUD", "[EbsdLib][ODFSectionUtilities]")
{
  auto values = DoubleArrayType::CreateArray(32, "CountDensity", true);
  values->initializeWithValue(1.0 / 32.0);
  ODFGridView grid{values.get(), {4, 2, 4}, {0.0, 0.0, 0.0}, {90.0, 90.0, 90.0}, ODFValueUnits::CountDensity, CrystalStructure::Triclinic};
  const auto result = PrepareODFSections(grid, 4);
  REQUIRE(result);
  const double step = constants::k_PiD / 2.0;
  const double expected = (1.0 / 32.0) * 8.0 * constants::k_PiD * constants::k_PiD / (step * step * step * std::sin(constants::k_PiD / 4.0));
  REQUIRE(result.sections.mudValues.front() == Approx(expected));
}

TEST_CASE("ebsdlib::ODFSectionUtilities::Phi2FastestSourceLinearization", "[EbsdLib][ODFSectionUtilities]")
{
  constexpr size_t k_Phi1Count = 4;
  constexpr size_t k_PhiCount = 2;
  constexpr size_t k_Phi2Count = 4;
  auto values = DoubleArrayType::CreateArray(k_Phi1Count * k_PhiCount * k_Phi2Count, "Phi2FastestSentinel", true);
  for(size_t phi1Index = 0; phi1Index < k_Phi1Count; phi1Index++)
  {
    for(size_t phiIndex = 0; phiIndex < k_PhiCount; phiIndex++)
    {
      for(size_t phi2Index = 0; phi2Index < k_Phi2Count; phi2Index++)
      {
        const size_t flatIndex = (phi1Index * k_PhiCount + phiIndex) * k_Phi2Count + phi2Index;
        values->setValue(flatIndex, 100.0 * static_cast<double>(phi1Index) + 10.0 * static_cast<double>(phiIndex) + static_cast<double>(phi2Index));
      }
    }
  }

  const ODFGridView grid{values.get(), {k_Phi1Count, k_PhiCount, k_Phi2Count}, {0.0, 0.0, 0.0}, {90.0, 90.0, 90.0}, ODFValueUnits::MUD, CrystalStructure::Triclinic};
  const auto result = PrepareODFSections(grid, 4);
  REQUIRE(result);
  REQUIRE(result.sections.mudValues == std::vector<double>{1.5, 101.5, 201.5, 301.5, 11.5, 111.5, 211.5, 311.5, 0.5, 100.5, 200.5, 300.5, 10.5, 110.5, 210.5, 310.5,
                                                           1.5, 101.5, 201.5, 301.5, 11.5, 111.5, 211.5, 311.5, 2.5, 102.5, 202.5, 302.5, 12.5, 112.5, 212.5, 312.5});
}

TEST_CASE("ebsdlib::ODFSectionUtilities::AllLaueLimits", "[EbsdLib][ODFSectionUtilities]")
{
  struct LimitsCase
  {
    uint32_t laueOpsIndex;
    ODFEulerPlotLimits expected;
  };

  const std::array<LimitsCase, 11> cases = {{{CrystalStructure::Hexagonal_High, {360.0, 90.0, 60.0}},
                                             {CrystalStructure::Cubic_High, {360.0, 90.0, 90.0}},
                                             {CrystalStructure::Hexagonal_Low, {360.0, 180.0, 60.0}},
                                             {CrystalStructure::Cubic_Low, {360.0, 90.0, 180.0}},
                                             {CrystalStructure::Triclinic, {360.0, 180.0, 360.0}},
                                             {CrystalStructure::Monoclinic, {360.0, 90.0, 360.0}},
                                             {CrystalStructure::OrthoRhombic, {360.0, 90.0, 180.0}},
                                             {CrystalStructure::Tetragonal_Low, {360.0, 180.0, 90.0}},
                                             {CrystalStructure::Tetragonal_High, {360.0, 90.0, 90.0}},
                                             {CrystalStructure::Trigonal_Low, {360.0, 180.0, 120.0}},
                                             {CrystalStructure::Trigonal_High, {360.0, 90.0, 120.0}}}};

  for(const auto& testCase : cases)
  {
    const auto limits = GetODFEulerPlotLimits(testCase.laueOpsIndex);
    REQUIRE(limits.phi1MaxDeg == Approx(testCase.expected.phi1MaxDeg));
    REQUIRE(limits.phiMaxDeg == Approx(testCase.expected.phiMaxDeg));
    REQUIRE(limits.phi2MaxDeg == Approx(testCase.expected.phi2MaxDeg));
  }
}

TEST_CASE("ebsdlib::ODFSectionUtilities::PeriodicPhi2Interpolation", "[EbsdLib][ODFSectionUtilities]")
{
  auto values = DoubleArrayType::CreateArray(32, "FourPlanes", true);
  for(size_t phi1Index = 0; phi1Index < 4; phi1Index++)
  {
    for(size_t phiIndex = 0; phiIndex < 2; phiIndex++)
    {
      for(size_t phi2Index = 0; phi2Index < 4; phi2Index++)
      {
        const size_t flatIndex = (phi1Index * 2 + phiIndex) * 4 + phi2Index;
        values->setValue(flatIndex, static_cast<double>(phi2Index + 1));
      }
    }
  }

  const ODFGridView grid{values.get(), {4, 2, 4}, {0.0, 0.0, 0.0}, {90.0, 90.0, 90.0}, ODFValueUnits::MUD, CrystalStructure::Triclinic};
  const auto result = PrepareODFSections(grid, 4);
  REQUIRE(result);
  REQUIRE(result.sections.sectionAnglesDeg == std::vector<double>{0.0, 90.0, 180.0, 270.0});
  REQUIRE(result.sections.phi1Count == 4);
  REQUIRE(result.sections.phiCount == 2);
  REQUIRE(result.sections.mudValues ==
          std::vector<double>{2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5});
  REQUIRE(result.sections.mudValues.front() == Approx(2.5));
  REQUIRE(result.sections.maximumDisplayedMUD == Approx(3.5));
}

TEST_CASE("ebsdlib::ODFSectionUtilities::HexagonalHighCrop", "[EbsdLib][ODFSectionUtilities]")
{
  auto values = DoubleArrayType::CreateArray(32, "HexagonalHighCrop", true);
  for(size_t phi1Index = 0; phi1Index < 4; phi1Index++)
  {
    for(size_t phiIndex = 0; phiIndex < 2; phiIndex++)
    {
      for(size_t phi2Index = 0; phi2Index < 4; phi2Index++)
      {
        const size_t flatIndex = (phi1Index * 2 + phiIndex) * 4 + phi2Index;
        values->setValue(flatIndex, static_cast<double>(phi1Index + 1 + phiIndex * 100));
      }
    }
  }

  const ODFGridView grid{values.get(), {4, 2, 4}, {0.0, 0.0, 0.0}, {90.0, 90.0, 90.0}, ODFValueUnits::MUD, CrystalStructure::Hexagonal_High};
  const auto result = PrepareODFSections(grid, 4);
  REQUIRE(result);
  REQUIRE(result.sections.sectionAnglesDeg == std::vector<double>{0.0, 15.0, 30.0, 45.0});
  REQUIRE(result.sections.phi1Count == 4);
  REQUIRE(result.sections.phiCount == 1);
  REQUIRE(result.sections.mudValues == std::vector<double>{1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0});
  REQUIRE(result.sections.maximumDisplayedMUD == Approx(4.0));
}

TEST_CASE("ebsdlib::ODFSectionUtilities::EmptyDisplayedCrop", "[EbsdLib][ODFSectionUtilities]")
{
  auto values = DoubleArrayType::CreateArray(4, "CoarseGrid", true);
  values->initializeWithValue(1.0);
  const ODFGridView grid{values.get(), {2, 1, 2}, {0.0, 0.0, 0.0}, {180.0, 180.0, 180.0}, ODFValueUnits::MUD, CrystalStructure::Hexagonal_High};
  const auto result = PrepareODFSections(grid, 6);
  REQUIRE_FALSE(result);
  REQUIRE(result.errorCode == -7502);
  REQUIRE(result.errorMessage.find("(2, 1, 2)") != std::string::npos);
  REQUIRE(result.errorMessage.find("180") != std::string::npos);
  REQUIRE(result.errorMessage.find("PHI=0") != std::string::npos);
}

TEST_CASE("ebsdlib::ODFSectionUtilities::Float32FractionalSpacing", "[EbsdLib][ODFSectionUtilities]")
{
  const size_t phiCount = GENERATE(size_t{7}, size_t{53});
  const double spacing = static_cast<float>(180.0 / static_cast<double>(phiCount));
  auto values = DoubleArrayType::CreateArray(4 * phiCount * phiCount * phiCount, "FractionalSpacing", true);
  values->initializeWithValue(1.0);
  ODFGridView grid{values.get(), {2 * phiCount, phiCount, 2 * phiCount}, {0.0, 0.0, 0.0}, {spacing, spacing, spacing}, ODFValueUnits::MUD, CrystalStructure::Hexagonal_High};
  const auto result = PrepareODFSections(grid, 6);
  INFO(result.errorMessage);
  REQUIRE(result);
  REQUIRE_FALSE(result.sections.mudValues.empty());
  for(const double value : result.sections.mudValues)
  {
    REQUIRE(value == Approx(1.0));
  }
  grid.spacingDeg = {spacing * 1.000001, spacing * 1.000001, spacing * 1.000001};
  const auto invalid = PrepareODFSections(grid, 6);
  REQUIRE_FALSE(invalid);
  REQUIRE(invalid.errorCode == -7502);
}

TEST_CASE("ebsdlib::ODFSectionUtilities::InvalidInput", "[EbsdLib][ODFSectionUtilities]")
{
  auto values = DoubleArrayType::CreateArray(32, "ValidValues", true);
  values->initializeWithValue(1.0);
  const ODFGridView validGrid{values.get(), {4, 2, 4}, {0.0, 0.0, 0.0}, {90.0, 90.0, 90.0}, ODFValueUnits::MUD, CrystalStructure::Triclinic};

  SECTION("Null values")
  {
    auto grid = validGrid;
    grid.values = nullptr;
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7500);
  }

  SECTION("Unsupported crystal code")
  {
    auto grid = validGrid;
    grid.laueOpsIndex = CrystalStructure::UnknownCrystalStructure;
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7501);
  }

  SECTION("Unsupported value units")
  {
    auto grid = validGrid;
    grid.units = static_cast<ODFValueUnits>(2);
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7502);
    REQUIRE(result.errorMessage.find("(2)") != std::string::npos);
    REQUIRE(result.errorMessage.find("MUD (0) or CountDensity (1)") != std::string::npos);
  }

  SECTION("Transposed dimensions")
  {
    auto grid = validGrid;
    grid.dimensions = {2, 4, 4};
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7502);
  }

  SECTION("Nonzero origin")
  {
    auto grid = validGrid;
    grid.originDeg = {0.0, 0.0, 1.0};
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7502);
  }

  SECTION("Nonuniform spacing")
  {
    auto grid = validGrid;
    grid.spacingDeg = {90.0, 45.0, 90.0};
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7502);
  }

  SECTION("Value count mismatch")
  {
    auto shortValues = DoubleArrayType::CreateArray(31, "ShortValues", true);
    shortValues->initializeWithValue(1.0);
    auto grid = validGrid;
    grid.values = shortValues.get();
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7503);
  }

  SECTION("Negative value")
  {
    auto negativeValues = DoubleArrayType::CreateArray(32, "NegativeValues", true);
    negativeValues->initializeWithValue(1.0);
    negativeValues->setValue(7, -0.25);
    auto grid = validGrid;
    grid.values = negativeValues.get();
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7504);
  }

  SECTION("Not-a-number value")
  {
    auto nanValues = DoubleArrayType::CreateArray(32, "NaNValues", true);
    nanValues->initializeWithValue(1.0);
    nanValues->setValue(11, std::numeric_limits<double>::quiet_NaN());
    auto grid = validGrid;
    grid.values = nanValues.get();
    const auto result = PrepareODFSections(grid, 4);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7504);
  }

  SECTION("One section")
  {
    const auto result = PrepareODFSections(validGrid, 1);
    REQUIRE_FALSE(result);
    REQUIRE(result.errorCode == -7505);
  }
}
