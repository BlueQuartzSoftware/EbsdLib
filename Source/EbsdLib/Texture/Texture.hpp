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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <random>
#include <vector>

#include <string>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/EbsdLibRandom.h"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"

namespace ebsdlib
{
/**
 * @brief This class holds default data for Orientation Distribution Function (ODF)
 * and Misorientation Distribution Functions (MDF)
 * calculations that the DREAM3D package will perform.
 *
 */
class Texture
{
public:
  virtual ~Texture() = default;

  using ODFTableEntry = struct
  {
    ebsdlib::Euler<double> euler;
    double weight;
    double sigma;
  };

  using ODFTableEntries = std::vector<ODFTableEntry>;

  /**
   * @brief This will calculate ODF data based on an array of weights that are
   * passed in and a Crystal Structure. This is templated on the container
   * type, LaueOps, and type of data. Containers that adhere to the STL Vector API
   * should be usable. std::vector falls into this category. The input data for the
   * euler angles is in Columnar fashion instead of row major format.
   * @param e1s The first euler angles
   * @param e2s The second euler angles
   * @param e3s The third euler angles
   * @param weights Array of weights values.
   * @param sigmas Array of sigma values.
   * @param normalize Should the ODF data be normalized by the totalWeight value
   * before returning.
   * @param odf (OUT) The ODF data that is generated from this function.
   * @param numEntries (OUT) The TotalWeight value that is also calculated
   */
  template <typename T, class LaueOps, class Container>
  static Container CalculateODFData(const ODFTableEntries& odfTableEntries, bool normalize)
  {
    LaueOps ops;

    // The number of ODF Bins is hard set at 5 degrees. This is something hard set inside
    // all the Laue classes. There is NO Changing this without a LOT of work
    std::array<size_t, 3> odfNumBins = ops.getOdfNumBins();

    std::vector<int32_t> textureBins(odfTableEntries.size());

    T addweight = 0;
    T totalAddWeight = 0;
    T totalWeight = T(ops.getODFSize());
    int bin, addbin;
    int bin1, bin2, bin3;
    int addbin1, addbin2, addbin3;
    T dist, fraction;

    // Loop on each odfTableEntry and build up the ODF Bins that the Euler belongs to
    for(size_t i = 0; i < odfTableEntries.size(); i++)
    {
      RodriguesDType rod = ops.getODFFZRod(odfTableEntries.at(i).euler.toRodrigues());
      bin = ops.getOdfBin(rod);
      textureBins[i] = bin;
    }

    // Create the ODF container and initialize all bins to ZERO
    Container odf(ops.getODFSize(), 0.0);

    // For each entry in the table...
    for(size_t i = 0; i < odfTableEntries.size(); i++)
    {
      const ODFTableEntry& odfTableEntry = odfTableEntries.at(i);
      bin = textureBins[i]; // Get the histogram bin that the Euler is assigned to
      bin1 = bin % odfNumBins[0];
      bin2 = static_cast<int32_t>((bin / odfNumBins[0]) % odfNumBins[1]);
      bin3 = bin / static_cast<int32_t>((odfNumBins[0] * odfNumBins[1]));
      for(int j = static_cast<int>(-odfTableEntry.sigma); j <= odfTableEntry.sigma; j++)
      {
        int jsqrd = j * j;
        for(int k = static_cast<int>(-odfTableEntry.sigma); k <= odfTableEntry.sigma; k++)
        {
          int ksqrd = k * k;
          for(int l = static_cast<int>(-odfTableEntry.sigma); l <= odfTableEntry.sigma; l++)
          {

            addbin1 = bin1 + int(j);
            addbin2 = bin2 + int(k);
            addbin3 = bin3 + int(l);

            if(addbin1 < 0)
            {
              continue;
            }
            if(addbin1 >= odfNumBins[0])
            {
              continue;
            }
            if(addbin2 < 0)
            {
              continue;
            }
            if(addbin2 >= odfNumBins[1])
            {
              continue;
            }
            if(addbin3 < 0)
            {
              continue;
            }
            if(addbin3 >= odfNumBins[2])
            {
              continue;
            }

            int lsqrd = l * l;
            addbin = static_cast<int>((addbin3 * odfNumBins[0] * odfNumBins[1]) + (addbin2 * odfNumBins[0]) + (addbin1));
            dist = static_cast<float>(std::pow(static_cast<double>(jsqrd + ksqrd + lsqrd), 0.5));

            double temp = dist / static_cast<double>(odfTableEntry.sigma);

            fraction = static_cast<float>(1.0 - (temp * temp));
            if(dist <= static_cast<int>(odfTableEntry.sigma))
            {
              addweight = (odfTableEntry.weight * fraction);
              if(odfTableEntry.sigma == 0.0)
              {
                addweight = odfTableEntry.weight;
              }
              odf[addbin] = odf[addbin] + addweight;
              totalAddWeight = totalAddWeight + addweight;
            }
          }
        }
      }
    }

    // These next loops *look* like they can be parallelized, but the arrays are not large enough
    // to see any benefit so don't go down that road. std::transform is also slower than the
    // manual loops that are coded.
    if(totalAddWeight > totalWeight)
    {
      float scale = (totalAddWeight / totalWeight);
      for(int i = 0; i < ops.getODFSize(); i++)
      {
        odf[i] = odf[i] / scale;
      }
    }
    else
    {
      float remainingWeight = totalWeight - totalAddWeight;
      float background = remainingWeight / static_cast<float>(ops.getODFSize());
      for(int i = 0; i < ops.getODFSize(); i++)
      {
        odf[i] += background;
      }
    }
    if(normalize)
    {
      // Normalize the odf
      for(int i = 0; i < ops.getODFSize(); i++)
      {
        odf[i] = odf[i] / totalWeight;
      }
    }

    return odf;
  }

  /**
   * @brief Calculates Misorientation Distribution Function (MDF) data.
   * @tparam T MDF value type.
   * @tparam LaueOps Symmetry operations for the selected crystal structure.
   * @tparam Container Random-access container type for the input and output arrays.
   * @param angles Misorientation angles in radians.
   * @param axes Misorientation axes with three components for each angle.
   * @param weights Multiples-of-random weights. A weight of w reserves w divided by the MDF bin count of the output mass.
   * @param odf Precomputed ODF data for the selected Laue class.
   * @param mdf Receives the normalized MDF data. The values always sum to 1 within floating-point rounding.
   * @param numEntries Number of rows to read from angles, axes, and weights.
   *
   * If the reserved mass exceeds the 10,000-sample budget, the function scales all reserved counts proportionally. The scaled counts use the complete budget.
   */
  template <typename T, class LaueOps, class Container>
  static void CalculateMDFData(Container& angles, Container& axes, Container& weights, const Container& odf, Container& mdf, size_t numEntries)
  {

    LaueOps orientationOps;
    const size_t odfsize = odf.size();
    const int mdfsize = orientationOps.getMDFSize();
    mdf.resize(orientationOps.getMDFSize());

    // Create a random-number generator.
    std::random_device randomDevice;
    std::mt19937_64 generator(randomDevice());
    std::mt19937_64::result_type seed = static_cast<std::mt19937_64::result_type>(std::chrono::steady_clock::now().time_since_epoch().count());
    generator.seed(seed);
    std::uniform_real_distribution<> distribution(0.0, 1.0);

    int mbin;
    int choose1, choose2;
    float totaldensity;
    float random1, random2, density;

    constexpr int64_t k_SampleCount = 10000;
    std::vector<int64_t> reservedCounts(static_cast<size_t>(mdfsize), 0);
    int64_t totalReservedCount = 0;
    const int aSize = static_cast<int>(numEntries);
    for(int i = 0; i < aSize; i++)
    {
      RodriguesDType rod = AxisAngleDType(axes[3 * i], axes[3 * i + 1], axes[3 * i + 2], angles[i]).toRodrigues();

      rod = orientationOps.getMDFFZRod(rod);
      mbin = orientationOps.getMisoBin(rod);
      const int64_t rowReservedCount = std::max<int64_t>(0, static_cast<int64_t>((weights[i] / static_cast<float>(mdfsize)) * static_cast<float>(k_SampleCount)));
      reservedCounts[mbin] += rowReservedCount;
      totalReservedCount += rowReservedCount;
    }

    if(totalReservedCount > k_SampleCount)
    {
      struct ScaledRemainder
      {
        size_t binIndex = 0;
        double remainder = 0.0;
      };

      const double scale = static_cast<double>(k_SampleCount) / static_cast<double>(totalReservedCount);
      std::vector<ScaledRemainder> scaledRemainders;
      int64_t scaledTotal = 0;
      for(size_t binIndex = 0; binIndex < reservedCounts.size(); binIndex++)
      {
        if(reservedCounts[binIndex] == 0)
        {
          continue;
        }

        const double scaledCount = static_cast<double>(reservedCounts[binIndex]) * scale;
        const int64_t integralCount = static_cast<int64_t>(scaledCount);
        reservedCounts[binIndex] = integralCount;
        scaledTotal += integralCount;
        scaledRemainders.push_back({binIndex, scaledCount - static_cast<double>(integralCount)});
      }

      std::sort(scaledRemainders.begin(), scaledRemainders.end(), [](const ScaledRemainder& lhs, const ScaledRemainder& rhs) {
        if(lhs.remainder == rhs.remainder)
        {
          return lhs.binIndex < rhs.binIndex;
        }
        return lhs.remainder > rhs.remainder;
      });
      for(int64_t index = 0; index < k_SampleCount - scaledTotal; index++)
      {
        reservedCounts[scaledRemainders[static_cast<size_t>(index)].binIndex]++;
      }
      totalReservedCount = k_SampleCount;
    }

    for(int i = 0; i < mdfsize; i++)
    {
      mdf[i] = static_cast<T>(-reservedCounts[static_cast<size_t>(i)]);
    }

    const int remainingcount = static_cast<int>(k_SampleCount - totalReservedCount);
    for(int i = 0; i < remainingcount; i++)
    {
      random1 = static_cast<float>(distribution(generator));
      random2 = static_cast<float>(distribution(generator));
      choose1 = 0;
      choose2 = 0;
      totaldensity = 0;
      for(size_t j = 0; j < odfsize; j++)
      {
        density = odf[j];
        float d = totaldensity;
        totaldensity = totaldensity + density;
        if(random1 >= d && random1 < totaldensity)
        {
          choose1 = static_cast<int>(j);
        }
        if(random2 >= d && random2 < totaldensity)
        {
          choose2 = static_cast<int>(j);
        }
      }
      // This is used to create a random Homochoric vector
      std::array<double, 3> randx3 = {distribution(generator), distribution(generator), distribution(generator)};
      EulerDType eu = orientationOps.determineEulerAngles(randx3.data(), choose1);
      QuatD q1 = eu.toQuaternion();

      randx3 = {distribution(generator), distribution(generator), distribution(generator)};
      eu = orientationOps.determineEulerAngles(randx3.data(), choose2);
      QuatD q2 = eu.toQuaternion();
      RodriguesDType ro = orientationOps.calculateMisorientation(q1, q2).toRodrigues();

      ro = orientationOps.getMDFFZRod(ro); // This operation is not implemented for all Laue classes.
      mbin = orientationOps.getMisoBin(ro);
      if(mdf[mbin] >= 0)
      {
        mdf[mbin]++;
      }
      if(mdf[mbin] < 0)
      {
        i = i - 1;
      }
    }
    double actualTotal = 0.0;
    for(int i = 0; i < mdfsize; i++)
    {
      if(mdf[i] < 0)
      {
        mdf[i] = -mdf[i];
      }
      actualTotal += static_cast<double>(mdf[i]);
    }
    for(int i = 0; i < mdfsize; i++)
    {
      mdf[i] = mdf[i] / static_cast<T>(actualTotal);
    }
  }

protected:
  Texture() = default;

public:
  Texture(const Texture&) = delete;            // Copy Constructor Not Implemented
  Texture(Texture&&) = delete;                 // Move Constructor Not Implemented
  Texture& operator=(const Texture&) = delete; // Copy Assignment Not Implemented
  Texture& operator=(Texture&&) = delete;      // Move Assignment Not Implemented
};
} // namespace ebsdlib
