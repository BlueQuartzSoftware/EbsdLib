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

#pragma once

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/EbsdLib.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ebsdlib
{
/**
 * @enum ODFValueUnits
 * @brief Specifies the units of an orientation distribution function grid.
 */
enum class ODFValueUnits : uint8_t
{
  MUD = 0,         ///< Specifies multiples of a uniform distribution.
  CountDensity = 1 ///< Specifies probability per Euler-space cell.
};

/**
 * @struct ODFGridView
 * @brief Describes a borrowed full-cube orientation distribution function grid.
 *
 * The caller owns the values array and must keep it valid during section preparation.
 * Flat source values use phi2-fastest row-major storage: `(phi1 * nPHI + PHI) * nphi2 + phi2`.
 */
struct EbsdLib_EXPORT ODFGridView
{
  /** @brief Source scalar values in phi2-fastest row-major order. The function does not take ownership. */
  DoubleArrayType* values = nullptr;
  /** @brief Cell counts listed in phi1, PHI, and phi2 order. */
  std::array<size_t, 3> dimensions = {0, 0, 0};
  /** @brief Grid origin in degrees for phi1, PHI, and phi2. */
  std::array<double, 3> originDeg = {0.0, 0.0, 0.0};
  /** @brief Cell spacing in degrees for phi1, PHI, and phi2. */
  std::array<double, 3> spacingDeg = {0.0, 0.0, 0.0};
  /** @brief Units of each source value. */
  ODFValueUnits units = ODFValueUnits::MUD;
  /** @brief EbsdLib crystal-structure index that selects Euler plot limits. */
  uint32_t laueOpsIndex = CrystalStructure::UnknownCrystalStructure;
};

/**
 * @struct ODFEulerPlotLimits
 * @brief Contains maximum Euler plot angles in degrees for one Laue class.
 */
struct EbsdLib_EXPORT ODFEulerPlotLimits
{
  /** @brief Exclusive maximum phi1 cell-center angle in degrees. */
  double phi1MaxDeg = 0.0;
  /** @brief Exclusive maximum PHI cell-center angle in degrees. */
  double phiMaxDeg = 0.0;
  /** @brief Periodic phi2 extent in degrees. */
  double phi2MaxDeg = 0.0;
};

/**
 * @struct PreparedODFSections
 * @brief Contains interpolated and cropped orientation distribution function sections.
 */
struct EbsdLib_EXPORT PreparedODFSections
{
  /** @brief Section-major values in MUD units. PHI is the middle axis, and phi1 is fastest. */
  std::vector<double> mudValues;
  /** @brief Exact phi2 section angles in degrees. */
  std::vector<double> sectionAnglesDeg;
  /** @brief Number of displayed phi1 cell centers. */
  size_t phi1Count = 0;
  /** @brief Number of displayed PHI cell centers. */
  size_t phiCount = 0;
  /** @brief Maximum value in the displayed section buffer, in MUD units. */
  double maximumDisplayedMUD = 0.0;
  /** @brief Euler plot limits for the selected Laue class. */
  ODFEulerPlotLimits limits;
};

/**
 * @struct ODFSectionPreparationResult
 * @brief Contains prepared sections or one validation error.
 */
struct EbsdLib_EXPORT ODFSectionPreparationResult
{
  /** @brief Prepared output. The value is empty when preparation fails. */
  PreparedODFSections sections;
  /** @brief Zero for success or a negative validation error code. */
  int32_t errorCode = 0;
  /** @brief Empty for success or a diagnostic message for failure. */
  std::string errorMessage;

  /**
   * @brief Reports whether section preparation succeeded.
   * @return True if errorCode is not negative.
   */
  explicit operator bool() const noexcept
  {
    return errorCode >= 0;
  }
};

/**
 * @brief Returns Euler plot limits for a supported Laue class.
 * @param laueOpsIndex EbsdLib crystal-structure index in the range [0, 10].
 * @return Plot limits in degrees. Unsupported indices return zero limits.
 */
EbsdLib_EXPORT ODFEulerPlotLimits GetODFEulerPlotLimits(uint32_t laueOpsIndex);

/**
 * @brief Generates exact periodic phi2 section angles for a supported Laue class.
 * @param laueOpsIndex EbsdLib crystal-structure index in the range [0, 10].
 * @param sectionCount Number of equal sections across the phi2 extent.
 * @return Section angles in degrees. Invalid input returns an empty vector.
 */
EbsdLib_EXPORT std::vector<double> GenerateODFSectionAngles(uint32_t laueOpsIndex, size_t sectionCount);

/**
 * @brief Validates an Euler grid, converts its values to MUD, and interpolates periodic phi2 sections.
 * @param grid Borrowed full-cube grid with scalar values and angles in degrees.
 * @param sectionCount Number of equal sections. The minimum valid value is two.
 * @return Prepared sections on success or a negative error code and diagnostic message on failure.
 */
EbsdLib_EXPORT ODFSectionPreparationResult PrepareODFSections(const ODFGridView& grid, size_t sectionCount);
} // namespace ebsdlib
