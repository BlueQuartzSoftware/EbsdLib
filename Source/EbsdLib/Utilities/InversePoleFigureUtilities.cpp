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

#include "InversePoleFigureUtilities.h"

#include <cmath>
#include <limits>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/Matrix3X3.hpp"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ModifiedLambertProjection.h"

using namespace ebsdlib;

// -----------------------------------------------------------------------------
InversePoleFigureUtilities::InversePoleFigureUtilities() = default;

// -----------------------------------------------------------------------------
InversePoleFigureUtilities::~InversePoleFigureUtilities() = default;

// -----------------------------------------------------------------------------
ebsdlib::FloatArrayType::Pointer InversePoleFigureUtilities::computeIPFDirections(const LaueOps& ops, ebsdlib::FloatArrayType* eulers, const Matrix3X1D& sampleDirection)
{
  size_t numOrientations = eulers->getNumberOfTuples();

  // Allocate output array for crystal directions (3 components per orientation)
  std::vector<size_t> cDims(1, 3);
  ebsdlib::FloatArrayType::Pointer directions = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "IPF_Directions", true);
  directions->initializeWithZeros();

  size_t numSymOps = ops.getNumSymOps();
  bool hasInversion = ops.getHasInversion();

  const ebsdlib::Matrix3X1D refDirection(sampleDirection);

  size_t validCount = 0;

  for(size_t i = 0; i < numOrientations; i++)
  {
    float* euler = eulers->getTuplePointer(i);
    EulerDType eu(static_cast<double>(euler[0]), static_cast<double>(euler[1]), static_cast<double>(euler[2]));

    QuatD q1 = eu.toQuaternion();
    OrientationMatrixDType om;

    bool found = false;
    ebsdlib::Matrix3X1D p;

    for(size_t j = 0; j < numSymOps; j++)
    {
      QuaternionDType qu(ops.getQuatSymOp(j) * q1);
      om = qu.toOrientationMatrix();
      ebsdlib::Matrix3X3D g(om.data());
      p = (g * refDirection).normalize();

      if(!hasInversion && p[2] < 0)
      {
        continue;
      }
      if(hasInversion && p[2] < 0)
      {
        p = p * -1.0;
      }

      double chi = std::acos(p[2]);
      double eta = std::atan2(p[1], p[0]);

      if(!ops.inUnitTriangle(eta, chi))
      {
        continue;
      }

      found = true;
      break;
    }

    if(found)
    {
      float* dirPtr = directions->getTuplePointer(validCount);
      dirPtr[0] = static_cast<float>(p[0]);
      dirPtr[1] = static_cast<float>(p[1]);
      dirPtr[2] = static_cast<float>(p[2]);
      validCount++;
    }
  }

  // Create a trimmed array with only the valid directions
  if(validCount < numOrientations)
  {
    ebsdlib::FloatArrayType::Pointer trimmed = ebsdlib::FloatArrayType::CreateArray(validCount, cDims, "IPF_Directions", true);
    float* srcPtr = directions->getPointer(0);
    float* dstPtr = trimmed->getPointer(0);
    std::copy(srcPtr, srcPtr + validCount * 3, dstPtr);
    return trimmed;
  }

  return directions;
}

// -----------------------------------------------------------------------------
ebsdlib::DoubleArrayType::Pointer InversePoleFigureUtilities::computeIPFIntensity(const LaueOps& ops, ebsdlib::FloatArrayType* ipfDirections, int imageWidth, int imageHeight, int lambertDim,
                                                                                  bool normalizeMRD, bool useStereographicSST)
{
  // Step 1: Bin the crystal directions into the Lambert projection
  float sphereRadius = 1.0f;
  ModifiedLambertProjection::Pointer lambert = ModifiedLambertProjection::LambertBallToSquare(ipfDirections, lambertDim, sphereRadius);

  // Step 2: Normalize the north square only (all SST directions have z >= 0)
  // We normalize manually to avoid division by zero in the south square
  ebsdlib::DoubleArrayType::Pointer northSquare = lambert->getNorthSquare();
  double* north = northSquare->getPointer(0);
  size_t nBins = static_cast<size_t>(lambertDim) * static_cast<size_t>(lambertDim);

  double northTotal = 0.0;
  for(size_t i = 0; i < nBins; i++)
  {
    northTotal += north[i];
  }

  if(northTotal > 0.0)
  {
    if(normalizeMRD)
    {
      // MRD: (count / totalCount) * totalBins
      double oneOverTotal = 1.0 / northTotal;
      for(size_t i = 0; i < nBins; i++)
      {
        north[i] = north[i] * oneOverTotal * static_cast<double>(nBins);
      }
    }
    // If not MRD, leave as raw counts
  }

  // Step 3: Create the output intensity image
  std::vector<size_t> tDims = {static_cast<size_t>(imageWidth * imageHeight)};
  std::vector<size_t> cDims = {1};
  ebsdlib::DoubleArrayType::Pointer intensity = ebsdlib::DoubleArrayType::CreateArray(tDims, cDims, "IPF_Intensity", true);
  double* intensityPtr = intensity->getPointer(0);

  if(useStereographicSST)
  {
    // Use the same stereographic projection as CreateIPFLegend (SST-only view)
    int imageDim = imageWidth; // Assumes square image
    for(int y = 0; y < imageHeight; y++)
    {
      for(int x = 0; x < imageWidth; x++)
      {
        int index = y * imageWidth + x;
        std::array<float, 3> sphereDir = {0.0f, 0.0f, 0.0f};

        if(!ops.mapPixelToSphereSST(x, y, imageDim, sphereDir))
        {
          intensityPtr[index] = -1.0;
          continue;
        }

        // Look up intensity from Lambert bins
        std::array<float, 2> sqCoord = {0.0f, 0.0f};
        bool isNorth = lambert->getSquareCoord(sphereDir.data(), sqCoord.data());
        if(isNorth)
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::NorthSquare, sqCoord.data());
        }
        else
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::SouthSquare, sqCoord.data());
        }
      }
    }
  }
  else
  {
    // Lambert azimuthal equal-area projection centered on north pole
    // Maps the upper hemisphere (z >= 0) to a disk of radius sqrt(2)
    float unitRadius = std::sqrt(2.0f);
    float span = 2.0f * unitRadius;
    float xres = span / static_cast<float>(imageWidth);
    float yres = span / static_cast<float>(imageHeight);

    int halfWidth = imageWidth / 2;
    int halfHeight = imageHeight / 2;

    for(int y = 0; y < imageHeight; y++)
    {
      for(int x = 0; x < imageWidth; x++)
      {
        int index = y * imageWidth + x;

        // Map pixel to equal-area projection coordinates
        float xtmp = static_cast<float>(x - halfWidth) * xres + (xres * 0.5f);
        float ytmp = static_cast<float>(y - halfHeight) * yres + (yres * 0.5f);

        float rhoSq = xtmp * xtmp + ytmp * ytmp;

        // Check if within hemisphere disk
        if(rhoSq > 2.0f)
        {
          intensityPtr[index] = -1.0; // Outside hemisphere
          continue;
        }

        // Inverse Lambert azimuthal equal-area projection (north pole centered)
        float t = std::sqrt(1.0f - rhoSq / 4.0f);
        std::array<float, 3> xyz = {xtmp * t, ytmp * t, 1.0f - rhoSq / 2.0f};

        // Compute chi (polar angle from z-axis) and eta (azimuthal angle)
        double chi = std::acos(static_cast<double>(xyz[2]));
        double eta = std::atan2(static_cast<double>(xyz[1]), static_cast<double>(xyz[0]));

        // Check if direction is inside the Standard Stereographic Triangle
        if(!ops.inUnitTriangle(eta, chi))
        {
          intensityPtr[index] = -1.0; // Outside SST
          continue;
        }

        // Look up the interpolated intensity from the Lambert projection
        std::array<float, 2> sqCoord = {0.0f, 0.0f};
        bool isNorth = lambert->getSquareCoord(xyz.data(), sqCoord.data());
        if(isNorth)
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::NorthSquare, sqCoord.data());
        }
        else
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::SouthSquare, sqCoord.data());
        }
      }
    }
  }

  return intensity;
}

// -----------------------------------------------------------------------------
void InversePoleFigureUtilities::createIPFColorImage(ebsdlib::DoubleArrayType* intensity, int imageWidth, int imageHeight, int numColors, double minScale, double maxScale,
                                                     ebsdlib::UInt8ArrayType* rgba)
{
  // Initialize the image with all zeros
  rgba->initializeWithZeros();
  uint32_t* rgbaPtr = reinterpret_cast<uint32_t*>(rgba->getPointer(0));

  // Get the color table
  std::vector<float> colors(numColors * 3, 0.0f);
  EbsdColorTable::GetColorTable(numColors, colors);

  double* dataPtr = intensity->getPointer(0);
  double range = maxScale - minScale;
  if(range <= 0.0)
  {
    range = 1.0;
  }

  for(int y = 0; y < imageHeight; y++)
  {
    for(int x = 0; x < imageWidth; x++)
    {
      size_t idx = static_cast<size_t>(y * imageWidth + x);
      double value = dataPtr[idx];

      // Pixels outside SST have value -1.0 -> set to white
      if(value < 0.0)
      {
        rgbaPtr[idx] = 0xFFFFFFFF; // White (ARGB)
        continue;
      }

      // Normalize to [0, 1] range
      double normalized = (value - minScale) / range;
      int bin = static_cast<int>(normalized * numColors);
      if(bin > numColors - 1)
      {
        bin = numColors - 1;
      }
      if(bin < 0)
      {
        bin = 0;
      }

      float r = colors[3 * bin];
      float g = colors[3 * bin + 1];
      float b = colors[3 * bin + 2];

      rgbaPtr[idx] = ebsdlib::RgbColor::dRgb(static_cast<int>(r * 255.0f), static_cast<int>(g * 255.0f), static_cast<int>(b * 255.0f), 255);
    }
  }
}
