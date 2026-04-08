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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"

namespace canvas_ity
{
class canvas;
} // namespace canvas_ity

namespace ebsdlib
{

/**
 * @brief Layout arrangement for the 3 pole figures and legend in the composite image.
 */
enum class PoleFigureLayoutType : uint32_t
{
  Horizontal = 0, ///< 3 figures + legend side-by-side in a single row
  Vertical = 1,   ///< 3 figures + legend stacked in a single column
  Square = 2      ///< 2x2 grid arrangement
};

/**
 * @brief Configuration for generating a complete composite pole figure image.
 *
 * Contains both the parameters needed to generate individual pole figures
 * (Euler angles, image dimensions, color settings) and the composition-specific
 * parameters (layout type, phase info for the legend).
 */
struct EbsdLib_EXPORT CompositePoleFigureConfiguration_t
{
  // --- Pole figure generation parameters ---
  ebsdlib::FloatArrayType* eulers = nullptr;   ///< Euler angles in radians (3-component tuples)
  int imageDim = 512;                          ///< Height/width of each individual pole figure in pixels
  int lambertDim = 256;                        ///< Lambert square dimension for interpolation
  int numColors = 32;                          ///< Number of colors in the color table
  double minScale = 0.0;                       ///< Minimum intensity scale value
  double maxScale = 1.0;                       ///< Maximum intensity scale value
  float sphereRadius = 1.0f;                   ///< Sphere radius (should always be 1.0)
  bool discrete = false;                       ///< Use discrete point sampling instead of Lambert projection
  bool discreteHeatMap = false;                ///< Use heat map coloring for discrete mode
  std::string colorMap;                        ///< Name of the color map to use
  std::vector<std::string> labels;             ///< Labels for the 3 pole figures (e.g., "<001>", "<011>", "<111>")
  std::vector<unsigned int> order = {0, 1, 2}; ///< Display order of the 3 pole figures
  bool flipFinalImage = true;                  ///< Flip individual images so +Y points up

  // --- Composition parameters ---
  PoleFigureLayoutType layoutType = PoleFigureLayoutType::Horizontal; ///< How to arrange figures and legend
  uint32_t laueOpsIndex = 0;                                          ///< Index into LaueOps::GetAllOrientationOps()
  std::string phaseName;                                              ///< Material/phase name for the legend
  int32_t phaseNumber = 1;                                            ///< Phase number for the legend
  std::string title;                                                  ///< Title text drawn at the top of the composite image
};

/**
 * @brief Result of generating a composite pole figure image.
 */
struct EbsdLib_EXPORT CompositePoleFigureResult
{
  UInt8ArrayType::Pointer image; ///< RGBA image data (4 components per tuple, row-major order)
  int32_t height = 0;            ///< Image height in pixels (rows, slow dimension)
  int32_t width = 0;             ///< Image width in pixels (cols, fast dimension)
};

/**
 * @brief Internal layout metrics computed from image dimensions and layout type.
 */
struct LayoutMetrics
{
  int32_t pageWidth = 0;
  int32_t pageHeight = 0;
  float fontPtSize = 0.0f;
  float margins = 0.0f;
  float subCanvasWidth = 0.0f;
  float subCanvasHeight = 0.0f;
  std::array<std::array<float, 2>, 4> origins; ///< [0-2] = pole figure origins, [3] = legend origin
};

/**
 * @brief Generates complete composite pole figure images from Euler angle data.
 *
 * Given a set of Euler angles and a crystal symmetry (LaueOps index), this class
 * produces a single RGBA image containing 3 individual pole figures arranged
 * according to the specified layout, with axis labels, direction labels, and a
 * legend/scalar bar.
 *
 * The class is stateless. Each call to generateCompositeImage() is self-contained.
 *
 * Example usage:
 * @code
 * CompositePoleFigureConfiguration_t config;
 * config.eulers = eulerAnglesPtr.get();
 * config.imageDim = 512;
 * config.lambertDim = 256;
 * config.numColors = 32;
 * config.laueOpsIndex = crystalStructure;
 * config.phaseName = "Nickel";
 * config.phaseNumber = 1;
 * config.layoutType = PoleFigureLayoutType::Horizontal;
 *
 * PoleFigureCompositor compositor;
 * CompositePoleFigureResult result = compositor.generateCompositeImage(config);
 * // result.image contains RGBA data, result.width and result.height give dimensions
 * @endcode
 */
class EbsdLib_EXPORT PoleFigureCompositor
{
public:
  PoleFigureCompositor() = default;
  ~PoleFigureCompositor() = default;

  PoleFigureCompositor(const PoleFigureCompositor&) = delete;
  PoleFigureCompositor(PoleFigureCompositor&&) = delete;
  PoleFigureCompositor& operator=(const PoleFigureCompositor&) = delete;
  PoleFigureCompositor& operator=(PoleFigureCompositor&&) = delete;

  /**
   * @brief Generates a complete composite pole figure image.
   *
   * Produces 3 individual pole figures for the given Euler angles and crystal
   * symmetry, then composes them into a single RGBA image with axis labels,
   * direction labels, a legend/scalar bar, and a title, arranged according to
   * the specified layout type.
   *
   * @param config Configuration controlling pole figure generation, layout, and appearance.
   *               Note: minScale and maxScale may be updated by the pole figure generation
   *               to reflect the actual data range.
   * @return CompositePoleFigureResult containing the RGBA image and its dimensions
   */
  CompositePoleFigureResult generateCompositeImage(CompositePoleFigureConfiguration_t& config);

  /**
   * @brief Computes layout metrics without generating an image.
   *
   * Useful for callers that need to know the output image dimensions before
   * generating the composite.
   *
   * @param config Configuration with imageDim and layoutType set
   * @return LayoutMetrics with page dimensions and figure origins
   */
  static LayoutMetrics computeLayoutMetrics(const CompositePoleFigureConfiguration_t& config);

private:
  std::vector<UInt8ArrayType::Pointer> generatePoleFigures(CompositePoleFigureConfiguration_t& config);
  void preprocessImages(std::vector<UInt8ArrayType::Pointer>& images, int imageDim, bool flipFinalImage);
  UInt8ArrayType::Pointer compositeToCanvas(const CompositePoleFigureConfiguration_t& config, const std::vector<UInt8ArrayType::Pointer>& images, const LayoutMetrics& layout);

  static void drawPoleFigure(canvas_ity::canvas& context, const UInt8ArrayType& image, std::array<float, 2> origin, int imageDim, const std::string& directionLabel, float fontPtSize, float margins,
                             const std::vector<unsigned char>& latoBold, const std::vector<unsigned char>& firaSans);
  static void drawScalarBar(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                            const std::vector<unsigned char>& latoRegular);
  static void drawInfoBlock(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                            const std::vector<unsigned char>& latoRegular);
  static void drawTitle(canvas_ity::canvas& context, const std::string& title, float pageWidth, float fontPtSize, float margins, const std::vector<unsigned char>& latoBold);
  static UInt8ArrayType::Pointer flipAndMirror(UInt8ArrayType* src, int imageDim);
  static UInt8ArrayType::Pointer convertColorOrder(UInt8ArrayType* src, int imageDim);
};

} // namespace ebsdlib
