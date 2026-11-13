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

#include "ODFSectionUtilities.h"

#include "EbsdLib/Math/EbsdLibMath.h"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
using namespace ebsdlib;

constexpr int32_t k_NullValues = -7500;
constexpr int32_t k_InvalidLaueClass = -7501;
constexpr int32_t k_InvalidGrid = -7502;
constexpr int32_t k_ValueCountMismatch = -7503;
constexpr int32_t k_InvalidValue = -7504;
constexpr int32_t k_InvalidSectionCount = -7505;

constexpr std::array<ODFEulerPlotLimits, 11> k_Limits = {{
    {360.0, 90.0, 60.0},   // Hexagonal High
    {360.0, 90.0, 90.0},   // Cubic High
    {360.0, 180.0, 60.0},  // Hexagonal Low
    {360.0, 90.0, 180.0},  // Cubic Low
    {360.0, 180.0, 360.0}, // Triclinic
    {360.0, 90.0, 360.0},  // Monoclinic, EbsdLib unique-axis convention
    {360.0, 90.0, 180.0},  // Orthorhombic
    {360.0, 180.0, 90.0},  // Tetragonal Low
    {360.0, 90.0, 90.0},   // Tetragonal High
    {360.0, 180.0, 120.0}, // Trigonal Low
    {360.0, 90.0, 120.0}   // Trigonal High
}};

ODFSectionPreparationResult MakeError(int32_t code, std::string message)
{
  ODFSectionPreparationResult result;
  result.errorCode = code;
  result.errorMessage = std::move(message);
  return result;
}

bool CoversEulerCube(const ODFGridView& grid)
{
  constexpr std::array<double, 3> k_FullExtentsDeg = {360.0, 180.0, 360.0};
  constexpr double k_BinCountTolerance = 1.0e-6;
  for(size_t axis = 0; axis < grid.dimensions.size(); axis++)
  {
    const double binCount = k_FullExtentsDeg[axis] / grid.spacingDeg[axis];
    if(grid.dimensions[axis] == 0 || std::abs(binCount - static_cast<double>(grid.dimensions[axis])) > k_BinCountTolerance)
    {
      return false;
    }
  }
  return true;
}

size_t FullIndex(size_t phi1Index, size_t phiIndex, size_t phi2Index, const std::array<size_t, 3>& dimensions)
{
  return (phi1Index * dimensions[1] + phiIndex) * dimensions[2] + phi2Index;
}

size_t SectionIndex(size_t sectionIndex, size_t phiIndex, size_t phi1Index, size_t phi1Count, size_t phiCount)
{
  return (sectionIndex * phiCount + phiIndex) * phi1Count + phi1Index;
}

size_t WrapIndex(int64_t index, size_t count)
{
  const auto signedCount = static_cast<int64_t>(count);
  const int64_t remainder = index % signedCount;
  return static_cast<size_t>(remainder < 0 ? remainder + signedCount : remainder);
}

size_t CountCellCentersBelow(size_t cellCount, double spacingDeg, double maximumDeg)
{
  size_t displayedCount = 0;
  while(displayedCount < cellCount && (static_cast<double>(displayedCount) + 0.5) * spacingDeg < maximumDeg)
  {
    displayedCount++;
  }
  return displayedCount;
}
} // namespace

namespace ebsdlib
{
ODFEulerPlotLimits GetODFEulerPlotLimits(uint32_t laueOpsIndex)
{
  if(laueOpsIndex >= k_Limits.size())
  {
    return {};
  }
  return k_Limits[laueOpsIndex];
}

std::vector<double> GenerateODFSectionAngles(uint32_t laueOpsIndex, size_t sectionCount)
{
  if(laueOpsIndex >= k_Limits.size() || sectionCount == 0)
  {
    return {};
  }

  std::vector<double> sectionAnglesDeg(sectionCount, 0.0);
  const double sectionStepDeg = k_Limits[laueOpsIndex].phi2MaxDeg / static_cast<double>(sectionCount);
  for(size_t sectionIndex = 0; sectionIndex < sectionCount; sectionIndex++)
  {
    sectionAnglesDeg[sectionIndex] = static_cast<double>(sectionIndex) * sectionStepDeg;
  }
  return sectionAnglesDeg;
}

ODFSectionPreparationResult PrepareODFSections(const ODFGridView& grid, size_t sectionCount)
{
  if(grid.values == nullptr)
  {
    return MakeError(k_NullValues, "The ODF values pointer is null. Provide a valid scalar DoubleArrayType.");
  }
  switch(grid.units)
  {
  case ODFValueUnits::MUD:
  case ODFValueUnits::CountDensity:
    break;
  default:
    return MakeError(k_InvalidGrid, fmt::format("The ODF value-units code ({}) is not supported. Use MUD (0) or CountDensity (1).", static_cast<uint32_t>(grid.units)));
  }
  if(grid.laueOpsIndex >= k_Limits.size())
  {
    return MakeError(k_InvalidLaueClass, fmt::format("The Laue class index ({}) is not supported. Use an EbsdLib crystal-structure index from 0 through 10.", grid.laueOpsIndex));
  }
  if(sectionCount < 2)
  {
    return MakeError(k_InvalidSectionCount, fmt::format("The section count ({}) is not valid. Use at least 2 sections.", sectionCount));
  }
  if(grid.originDeg != std::array<double, 3>{0.0, 0.0, 0.0})
  {
    return MakeError(k_InvalidGrid,
                     fmt::format("The ODF grid origin ({}, {}, {}) degrees is not valid. Use a zero origin for phi1, PHI, and phi2.", grid.originDeg[0], grid.originDeg[1], grid.originDeg[2]));
  }
  if(!std::isfinite(grid.spacingDeg[0]) || grid.spacingDeg[0] <= 0.0 || grid.spacingDeg[1] != grid.spacingDeg[0] || grid.spacingDeg[2] != grid.spacingDeg[0])
  {
    return MakeError(k_InvalidGrid, fmt::format("The ODF grid spacing ({}, {}, {}) degrees is not valid. Use equal positive spacing for phi1, PHI, and phi2.", grid.spacingDeg[0], grid.spacingDeg[1],
                                                grid.spacingDeg[2]));
  }
  if(!CoversEulerCube(grid))
  {
    return MakeError(k_InvalidGrid, fmt::format("The ODF grid dimensions ({}, {}, {}) with spacing {} degrees do not cover the required 360 by 180 by 360 degree Euler cube.", grid.dimensions[0],
                                                grid.dimensions[1], grid.dimensions[2], grid.spacingDeg[0]));
  }
  if(grid.dimensions[2] > static_cast<size_t>(std::numeric_limits<int64_t>::max()))
  {
    return MakeError(k_InvalidGrid, fmt::format("The phi2 grid dimension ({}) is too large for periodic indexing.", grid.dimensions[2]));
  }

  if(grid.dimensions[0] > std::numeric_limits<size_t>::max() / grid.dimensions[1] || grid.dimensions[0] * grid.dimensions[1] > std::numeric_limits<size_t>::max() / grid.dimensions[2])
  {
    return MakeError(k_InvalidGrid, fmt::format("The ODF grid dimensions ({}, {}, {}) are too large to index safely.", grid.dimensions[0], grid.dimensions[1], grid.dimensions[2]));
  }
  const size_t expectedValueCount = grid.dimensions[0] * grid.dimensions[1] * grid.dimensions[2];
  if(grid.values->getNumberOfTuples() != expectedValueCount || grid.values->getNumberOfComponents() != 1)
  {
    return MakeError(k_ValueCountMismatch, fmt::format("The ODF array '{}' contains {} tuples with {} components, but the grid dimensions require {} scalar tuples.", grid.values->getName(),
                                                       grid.values->getNumberOfTuples(), grid.values->getNumberOfComponents(), expectedValueCount));
  }

  std::vector<double> fullMud(expectedValueCount, 0.0);
  const double stepRad = grid.spacingDeg[0] * constants::k_PiOver180D;
  for(size_t phi2Index = 0; phi2Index < grid.dimensions[2]; phi2Index++)
  {
    for(size_t phiIndex = 0; phiIndex < grid.dimensions[1]; phiIndex++)
    {
      const double phiCenterRad = (static_cast<double>(phiIndex) + 0.5) * stepRad;
      for(size_t phi1Index = 0; phi1Index < grid.dimensions[0]; phi1Index++)
      {
        const size_t fullIndex = FullIndex(phi1Index, phiIndex, phi2Index, grid.dimensions);
        const double sourceValue = grid.values->getValue(fullIndex);
        if(!std::isfinite(sourceValue) || sourceValue < 0.0)
        {
          return MakeError(k_InvalidValue,
                           fmt::format("The ODF array '{}' contains the invalid value {} at linear index {}. Values must be finite and nonnegative.", grid.values->getName(), sourceValue, fullIndex));
        }
        if(grid.units == ODFValueUnits::CountDensity)
        {
          fullMud[fullIndex] = sourceValue * 8.0 * constants::k_PiD * constants::k_PiD / (stepRad * stepRad * stepRad * std::sin(phiCenterRad));
        }
        else
        {
          fullMud[fullIndex] = sourceValue;
        }
      }
    }
  }

  ODFSectionPreparationResult result;
  auto& prepared = result.sections;
  prepared.limits = k_Limits[grid.laueOpsIndex];
  prepared.sectionAnglesDeg = GenerateODFSectionAngles(grid.laueOpsIndex, sectionCount);
  prepared.phi1Count = CountCellCentersBelow(grid.dimensions[0], grid.spacingDeg[0], prepared.limits.phi1MaxDeg);
  prepared.phiCount = CountCellCentersBelow(grid.dimensions[1], grid.spacingDeg[0], prepared.limits.phiMaxDeg);
  if(prepared.phi1Count == 0 || prepared.phiCount == 0)
  {
    return MakeError(k_InvalidGrid, fmt::format("The ODF grid dimensions ({}, {}, {}) with spacing ({}, {}, {}) degrees contain no usable displayed cells for Laue class index {} "
                                                "and plotting limits (phi1={}, PHI={}, phi2={}) degrees: displayed counts phi1={}, PHI={}. Use a finer full Euler grid.",
                                                grid.dimensions[0], grid.dimensions[1], grid.dimensions[2], grid.spacingDeg[0], grid.spacingDeg[1], grid.spacingDeg[2], grid.laueOpsIndex,
                                                prepared.limits.phi1MaxDeg, prepared.limits.phiMaxDeg, prepared.limits.phi2MaxDeg, prepared.phi1Count, prepared.phiCount));
  }
  prepared.mudValues.resize(sectionCount * prepared.phiCount * prepared.phi1Count, 0.0);

  for(size_t sectionIndex = 0; sectionIndex < sectionCount; sectionIndex++)
  {
    const double firstCenterDeg = 0.5 * grid.spacingDeg[2];
    const double coordinate = (prepared.sectionAnglesDeg[sectionIndex] - firstCenterDeg) / grid.spacingDeg[2];
    const double floorCoordinate = std::floor(coordinate);
    const size_t lowerPhi2Index = WrapIndex(static_cast<int64_t>(floorCoordinate), grid.dimensions[2]);
    const size_t upperPhi2Index = (lowerPhi2Index + 1) % grid.dimensions[2];
    const double fraction = coordinate - floorCoordinate;
    for(size_t phiIndex = 0; phiIndex < prepared.phiCount; phiIndex++)
    {
      for(size_t phi1Index = 0; phi1Index < prepared.phi1Count; phi1Index++)
      {
        const double lowerValue = fullMud[FullIndex(phi1Index, phiIndex, lowerPhi2Index, grid.dimensions)];
        const double upperValue = fullMud[FullIndex(phi1Index, phiIndex, upperPhi2Index, grid.dimensions)];
        const double value = (1.0 - fraction) * lowerValue + fraction * upperValue;
        prepared.mudValues[SectionIndex(sectionIndex, phiIndex, phi1Index, prepared.phi1Count, prepared.phiCount)] = value;
        prepared.maximumDisplayedMUD = std::max(prepared.maximumDisplayedMUD, value);
      }
    }
  }
  return result;
}
} // namespace ebsdlib
