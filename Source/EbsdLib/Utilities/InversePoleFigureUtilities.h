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
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/Matrix3X1.hpp"

#include <array>
#include <string>
#include <vector>

namespace ebsdlib
{

class LaueOps; // Forward declaration

/**
 * @struct InversePoleFigureConfiguration_t
 * @brief Configuration struct for generating Inverse Pole Figure density plots.
 * The IPF density plot shows how a sample direction distributes across crystal
 * directions within the Standard Stereographic Triangle (SST).
 */
struct InversePoleFigureConfiguration_t
{
  ebsdlib::FloatArrayType* eulers;            ///<* The Euler Angles (in Radians) to use for the inverse pole figure
  std::array<Matrix3X1D, 3> sampleDirections; ///<* 3 orthogonal sample reference directions (e.g., RD, TD, ND)
  int imageWidth;                             ///<* The width of the generated inverse pole figure image in pixels
  int imageHeight;                            ///<* The height of the generated inverse pole figure image in pixels
  int lambertDim;                             ///<* The dimensions in voxels of the Lambert Square used for binning/smoothing
  int numColors;                              ///<* The number of colors to use in the color map
  std::string colorMap;                       ///<* Name of the ColorMap to use
  bool normalizeMRD;                          ///<* true=normalize to MRD (Multiples of Random Distribution), false=raw counts
  std::vector<std::string> labels;            ///<* The labels for each of the 3 inverse pole figures (e.g., "RD", "TD", "ND")
  std::string phaseName;                      ///<* The name of the phase
  bool FlipFinalImage;                        ///<* If TRUE, the final image will be flipped across the X Axis so that +Y axis points UP
  /// Cartesian basis convention for hex/trig phases. Affects the Miller-
  /// index labels drawn around the SST in generateAnnotatedIPFDensity.
  /// Ignored for cubic / tetragonal / orthorhombic / monoclinic / triclinic.
  /// See ebsdlib::HexConvention.
  ebsdlib::HexConvention hexConvention = ebsdlib::HexConvention::XParallelAStar;
};

/**
 * @class InversePoleFigureUtilities InversePoleFigureUtilities.h /Utilities/InversePoleFigureUtilities.h
 * @brief This class provides static utility methods for generating Inverse Pole Figure (IPF) density plots.
 *
 * The IPF density plot shows the distribution of a sample direction across crystal directions
 * within the Standard Stereographic Triangle (SST) using equal-area projection and Lambert-based
 * smoothing.
 */
class EbsdLib_EXPORT InversePoleFigureUtilities
{
public:
  InversePoleFigureUtilities();
  virtual ~InversePoleFigureUtilities();

  /**
   * @brief Computes the crystal directions in the fundamental zone for all orientations
   * given a single sample reference direction. For each orientation (Euler angle set),
   * the sample direction is transformed into the crystal frame and the symmetry-equivalent
   * direction within the Standard Stereographic Triangle is found.
   * @param ops The LaueOps instance providing symmetry operations
   * @param eulers The Euler angles array (3-component tuples, in radians)
   * @param sampleDirection The sample reference direction (e.g., [0,0,1] for ND)
   * @return FloatArrayType with 3-component tuples (XYZ crystal directions on unit sphere)
   */
  static ebsdlib::FloatArrayType::Pointer computeIPFDirections(const LaueOps& ops, ebsdlib::FloatArrayType* eulers, const Matrix3X1D& sampleDirection);

  /**
   * @brief Computes the intensity image for a single inverse pole figure using Lambert
   * projection for binning and equal-area reprojection masked to the SST boundary.
   * @param ops The LaueOps instance providing symmetry operations and SST boundary
   * @param ipfDirections The crystal directions from computeIPFDirections
   * @param imageWidth Output image width in pixels
   * @param imageHeight Output image height in pixels
   * @param lambertDim Lambert square dimension for binning/smoothing
   * @param normalizeMRD true to normalize to MRD, false for raw counts
   * @return DoubleArrayType intensity image (imageWidth * imageHeight). Pixels outside SST have value -1.0.
   */
  static ebsdlib::DoubleArrayType::Pointer computeIPFIntensity(const LaueOps& ops, ebsdlib::FloatArrayType* ipfDirections, int imageWidth, int imageHeight, int lambertDim, bool normalizeMRD,
                                                               bool useStereographicSST = false);

  /**
   * @brief Converts an intensity image to RGBA with SST masking. Pixels inside the SST
   * are mapped to colors via the color table; pixels outside are set to white.
   * @param intensity The intensity image from computeIPFIntensity
   * @param imageWidth Image width in pixels
   * @param imageHeight Image height in pixels
   * @param numColors Number of colors in the color table
   * @param minScale Minimum intensity value for color mapping
   * @param maxScale Maximum intensity value for color mapping
   * @param rgba [output] RGBA image (4-component UInt8 array, imageWidth * imageHeight tuples)
   */
  static void createIPFColorImage(ebsdlib::DoubleArrayType* intensity, int imageWidth, int imageHeight, int numColors, double minScale, double maxScale, ebsdlib::UInt8ArrayType* rgba);

public:
  InversePoleFigureUtilities(const InversePoleFigureUtilities&) = delete;            // Copy Constructor Not Implemented
  InversePoleFigureUtilities(InversePoleFigureUtilities&&) = delete;                 // Move Constructor Not Implemented
  InversePoleFigureUtilities& operator=(const InversePoleFigureUtilities&) = delete; // Copy Assignment Not Implemented
  InversePoleFigureUtilities& operator=(InversePoleFigureUtilities&&) = delete;      // Move Assignment Not Implemented
};

} // namespace ebsdlib
