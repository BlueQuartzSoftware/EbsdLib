/* ============================================================================
 * Copyright (c) 2017 BlueQuartz Software, LLC
 * All rights reserved.
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
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

namespace Stereographic::Utils
{

template <typename T>
EbsdLib::Matrix3X1<T> StereoToSpherical(const EbsdLib::Matrix3X1<T>& stereo)
{
  T sumOfSquares = stereo.dot();
  return {(2.0 * stereo[0]) / (1 + sumOfSquares), (2.0 * stereo[1]) / (1 + sumOfSquares), (1 - sumOfSquares) / (1 + sumOfSquares)};
}

template <typename T>
EbsdLib::Matrix3X1<T> StereoToSpherical(T x, T y)
{
  T sumOfSquares = x * x + y * y;
  return {(2.0 * x) / (1 + sumOfSquares), (2.0 * y) / (1 + sumOfSquares), (1 - sumOfSquares) / (1 + sumOfSquares)};
}

template <typename T>
EbsdLib::Matrix3X1<T> SphericalToStereo(const EbsdLib::Matrix3X1<T>& spherical)
{
  return {spherical[0] / (1 + spherical[2]), spherical[1] / (1 + spherical[2]), 0.0};
}

template <typename T>
EbsdLib::Matrix3X1<T> SphericalToStereo(T x, T y, T z)
{
  return {x / (1 + z), y / (1 + z), 0.0};
}

/**
 * @brief Function to transform points on a unit sphere into stereographic coords
 * @param points
 * @return
 */
template <typename T>
std::vector<EbsdLib::Matrix3X1<T>> TransformUnitSphereToStereographicCoords(const std::vector<EbsdLib::Matrix3X1<T>>& points)
{
  using Point3DType = EbsdLib::Matrix3X1<T>;
  std::vector<EbsdLib::Matrix3X1<T>> stereoPts;

  for(const auto& point : points)
  {
    if(point[2] < 0) // project southern hemisphere
    {
      stereoPts.emplace_back(Point3DType{(point[0] / (1.0F - point[2])), (point[1] / (1.0F - point[2])), 0});
    }
    else
    {
      stereoPts.emplace_back(Point3DType{(point[0] / (1.0F + point[2])), (point[1] / (1.0F + point[2])), 0});
    }
  }

  return stereoPts;
}

} // namespace Stereographic::Utils

class EbsdLib_EXPORT ComputeStereographicProjection
{
public:
  /**
   * @brief ComputeStereographicProjection
   * @param xyzCoords
   * @param config
   * @param intensity
   */
  ComputeStereographicProjection(EbsdLib::FloatArrayType* xyzCoords, PoleFigureConfiguration_t* config, EbsdLib::DoubleArrayType* intensity);

  virtual ~ComputeStereographicProjection();

  /**
   * @brief operator ()
   */
  void operator()() const;

protected:
  /**
   * @brief ComputeStereographicProjection
   */
  ComputeStereographicProjection();

private:
  EbsdLib::FloatArrayType* m_XYZCoords = nullptr;
  PoleFigureConfiguration_t* m_Config = nullptr;
  EbsdLib::DoubleArrayType* m_Intensity = nullptr;

public:
  ComputeStereographicProjection(const ComputeStereographicProjection&) = delete; // Copy Constructor Not Implemented
  ComputeStereographicProjection(ComputeStereographicProjection&&) = default;
  ComputeStereographicProjection& operator=(const ComputeStereographicProjection&) = delete; // Copy Assignment Not Implemented
  ComputeStereographicProjection& operator=(ComputeStereographicProjection&&) = delete;      // Move Assignment Not Implemented
};
