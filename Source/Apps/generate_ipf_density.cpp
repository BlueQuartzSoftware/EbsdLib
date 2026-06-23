/* ============================================================================
 * Copyright (c) 2025-2026 BlueQuartz Software, LLC
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

/**
 * @file generate_ipf_density.cpp
 * @brief Example program that generates Inverse Pole Figure (IPF) density images
 * for all 11 Laue classes using random orientations. The IPF density plot shows
 * how a sample direction distributes across crystal directions within the
 * Standard Stereographic Triangle (SST).
 *
 * For each Laue class, 3 TIFF images are generated corresponding to 3 orthogonal
 * sample directions: RD (Rolling Direction), TD (Transverse Direction), and
 * ND (Normal Direction).
 *
 * Additionally, a quaternion texture file is read and used to generate IPF density
 * images (ND direction) for all 11 Laue classes, demonstrating a strong near-cube
 * texture.
 *
 * Usage:
 *   generate_ipf_density [output_directory] [num_orientations]
 *
 * If no arguments are provided, output goes to the build's Testing/Temporary directory
 * and 5000 random orientations are used.
 */

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/InversePoleFigureUtilities.h"
#include "EbsdLib/Utilities/PngWriter.h"

#include "EbsdLib/Apps/EbsdLibFileLocations.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{

// -----------------------------------------------------------------------
// Generate random Euler angles with proper sampling of the orientation space.
// Uses a cosine distribution for Phi to ensure uniform coverage of SO(3).
// -----------------------------------------------------------------------
ebsdlib::FloatArrayType::Pointer generateRandomEulers(size_t numOrientations, unsigned int seed)
{
  std::vector<size_t> cDims = {3};
  auto eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);

  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> phi1Dist(0.0f, static_cast<float>(ebsdlib::constants::k_2PiD));
  std::uniform_real_distribution<float> cosDist(-1.0f, 1.0f);
  std::uniform_real_distribution<float> phi2Dist(0.0f, static_cast<float>(ebsdlib::constants::k_2PiD));

  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = phi1Dist(gen);           // phi1: [0, 2pi)
    ptr[1] = std::acos(cosDist(gen)); // Phi:  [0, pi] with uniform sphere coverage
    ptr[2] = phi2Dist(gen);           // phi2: [0, 2pi)
  }
  return eulers;
}

// -----------------------------------------------------------------------
// Generate Euler angles for a single-crystal texture: all orientations identical.
// -----------------------------------------------------------------------
ebsdlib::FloatArrayType::Pointer generateSingleCrystalEulers(size_t numOrientations, float phi1, float Phi, float phi2)
{
  std::vector<size_t> cDims = {3};
  auto eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);

  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = phi1;
    ptr[1] = Phi;
    ptr[2] = phi2;
  }
  return eulers;
}

// -----------------------------------------------------------------------
// Convert an ARGB UInt8ArrayType image to RGB by stripping the alpha channel,
// suitable for PngWriter::WriteColorImage with samplesPerPixel=3.
// -----------------------------------------------------------------------
ebsdlib::UInt8ArrayType::Pointer convertARGBtoRGB(ebsdlib::UInt8ArrayType* argbImage)
{
  size_t numPixels = argbImage->getNumberOfTuples();
  auto rgbImage = ebsdlib::UInt8ArrayType::CreateArray(numPixels, {3ULL}, argbImage->getName(), true);

  for(size_t i = 0; i < numPixels; i++)
  {
    uint8_t* argb = argbImage->getTuplePointer(i);
    uint8_t* rgb = rgbImage->getTuplePointer(i);

    // The ARGB data is stored as a uint32_t: (A << 24) | (R << 16) | (G << 8) | B
    // When accessed as bytes on a little-endian system: [B, G, R, A]
    uint32_t pixel = *reinterpret_cast<uint32_t*>(argb);
    rgb[0] = static_cast<uint8_t>((pixel >> 16) & 0xFF); // R
    rgb[1] = static_cast<uint8_t>((pixel >> 8) & 0xFF);  // G
    rgb[2] = static_cast<uint8_t>(pixel & 0xFF);         // B
  }
  return rgbImage;
}

// -----------------------------------------------------------------------
// Write a single IPF density image to a TIFF file.
// -----------------------------------------------------------------------
void writeIPFImage(ebsdlib::UInt8ArrayType* image, int width, int height, const std::string& filePath)
{
  auto rgbImage = convertARGBtoRGB(image);
  auto result = PngWriter::WriteColorImage(filePath, width, height, 3, rgbImage->data());
  if(result.first < 0)
  {
    std::cerr << "  ERROR writing " << filePath << ": " << result.second << std::endl;
  }
  else
  {
    std::cout << "  Wrote: " << filePath << std::endl;
  }
}

// -----------------------------------------------------------------------
// Generate and save annotated IPF density images for a single LaueOps instance.
// -----------------------------------------------------------------------
void generateIPFForLaueClass(const LaueOps& ops, ebsdlib::FloatArrayType* eulers, const std::string& outputDir, int imageWidth, int imageHeight, int lambertDim, const std::string& textureLabel)
{
  std::string className = ops.getSymmetryName();
  std::cout << "Generating IPF density for: " << className << " (" << textureLabel << ")" << std::endl;

  InversePoleFigureConfiguration_t config;
  config.eulers = eulers;
  config.sampleDirections = {Matrix3X1D(1.0, 0.0, 0.0), Matrix3X1D(0.0, 1.0, 0.0), Matrix3X1D(0.0, 0.0, 1.0)};
  config.imageWidth = imageWidth;
  config.imageHeight = imageHeight;
  config.lambertDim = lambertDim;
  config.numColors = 64;
  config.colorMap = "Default";
  config.normalizeMRD = true;
  config.labels = {"A1", "A2", "A3"};
  config.phaseName = className;
  config.FlipFinalImage = false;

  auto images = ops.generateAnnotatedIPFDensity(config);

  // Sanitize symmetry name for filename
  std::string safeName = className;
  for(auto& c : safeName)
  {
    if(c == '/' || c == '\\' || c == ' ' || c == '(' || c == ')')
    {
      c = '_';
    }
  }

  int canvasDim = static_cast<int>(static_cast<float>(imageWidth) * 1.5f);
  for(size_t i = 0; i < images.size(); i++)
  {
    std::ostringstream filePath;
    filePath << outputDir << "/" << safeName << "_IPF_" << config.labels[i] << "_" << textureLabel << ".png";
    auto result = PngWriter::WriteColorImage(filePath.str(), canvasDim, canvasDim, 3, images[i]->data());
    if(result.first < 0)
    {
      std::cerr << "  ERROR writing " << filePath.str() << ": " << result.second << std::endl;
    }
    else
    {
      std::cout << "  Wrote: " << filePath.str() << std::endl;
    }
  }
}

// -----------------------------------------------------------------------
// Read quaternion data from a CSV file and convert to Euler angles using
// the OrientationConverter system. Expected CSV format: X,Y,Z,W,Distance
// (with header line). Quaternions are in vector-scalar order (x, y, z, w).
// Returns FloatArrayType with 3 components (phi1, Phi, phi2) in radians.
// -----------------------------------------------------------------------
ebsdlib::FloatArrayType::Pointer readQuaternionFileAsEulers(const std::string& filePath)
{
  std::ifstream inFile(filePath);
  if(!inFile.is_open())
  {
    std::cerr << "ERROR: Could not open quaternion file: " << filePath << std::endl;
    return nullptr;
  }

  // Skip header line
  std::string line;
  std::getline(inFile, line);

  // Read all quaternion values (first 4 columns per line)
  std::vector<double> quatValues;
  while(std::getline(inFile, line))
  {
    if(line.empty())
    {
      continue;
    }

    auto tokens = EbsdStringUtils::split(line, ',');
    if(tokens.size() >= 4)
    {
      quatValues.push_back(std::atof(tokens[0].c_str())); // X
      quatValues.push_back(std::atof(tokens[1].c_str())); // Y
      quatValues.push_back(std::atof(tokens[2].c_str())); // Z
      quatValues.push_back(std::atof(tokens[3].c_str())); // W
    }
  }
  inFile.close();

  size_t numOrientations = quatValues.size() / 4;
  std::cout << "  Read " << numOrientations << " quaternions from file" << std::endl;

  // Wrap the quaternion data (4 components per tuple) and convert to Euler angles
  using DoubleArrayType = EbsdDataArray<double>;

  std::vector<size_t> quatDims = {4};
  auto inputQuats = DoubleArrayType::WrapPointer(quatValues.data(), numOrientations, quatDims, "Quaternions", false);

  auto quatConverter = QuaternionConverter<DoubleArrayType, double>::New();
  quatConverter->setInputData(inputQuats);
  quatConverter->convertRepresentationTo(ebsdlib::orientations::Type::Euler);
  auto eulerData = quatConverter->getOutputData();

  // Convert double Euler angles to float for the IPF pipeline
  std::vector<size_t> eulerDims = {3};
  auto eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, eulerDims, "EulerAngles", true);
  for(size_t i = 0; i < numOrientations; i++)
  {
    double* src = eulerData->getTuplePointer(i);
    float* dst = eulers->getTuplePointer(i);
    dst[0] = static_cast<float>(src[0]);
    dst[1] = static_cast<float>(src[1]);
    dst[2] = static_cast<float>(src[2]);
  }

  return eulers;
}

// -----------------------------------------------------------------------
// Generate and save a single IPF density image for one sample direction.
// -----------------------------------------------------------------------
void generateSingleIPFForLaueClass(const LaueOps& ops, ebsdlib::FloatArrayType* eulers, const Matrix3X1D& sampleDir, const std::string& dirLabel, const std::string& outputDir, int imageWidth,
                                   int imageHeight, int lambertDim, bool normalizeMRD, const std::string& textureLabel)
{
  std::string className = ops.getSymmetryName();
  std::string modeLabel = normalizeMRD ? "MRD" : "Counts";
  std::cout << "Generating IPF " << modeLabel << " for: " << className << " (" << textureLabel << ", " << dirLabel << ")" << std::endl;

  auto directions = InversePoleFigureUtilities::computeIPFDirections(ops, eulers, sampleDir);
  auto intensity = InversePoleFigureUtilities::computeIPFIntensity(ops, directions.get(), imageWidth, imageHeight, lambertDim, normalizeMRD);

  // Find min/max for color scaling (only pixels >= 0 are inside SST)
  double minVal = std::numeric_limits<double>::max();
  double maxVal = std::numeric_limits<double>::lowest();
  double* dataPtr = intensity->getPointer(0);
  for(size_t i = 0; i < intensity->getNumberOfTuples(); i++)
  {
    if(dataPtr[i] >= 0.0)
    {
      minVal = std::min(minVal, dataPtr[i]);
      maxVal = std::max(maxVal, dataPtr[i]);
    }
  }

  std::vector<size_t> cDims = {4};
  auto rgba = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(imageWidth * imageHeight), cDims, "RGBA", true);
  InversePoleFigureUtilities::createIPFColorImage(intensity.get(), imageWidth, imageHeight, 64, minVal, maxVal, rgba.get());

  // Sanitize symmetry name for use as a filename
  std::string safeName = className;
  for(auto& c : safeName)
  {
    if(c == '/' || c == '\\' || c == ' ' || c == '(' || c == ')')
    {
      c = '_';
    }
  }

  std::ostringstream filePath;
  filePath << outputDir << "/" << safeName << "_IPF_" << dirLabel << "_" << textureLabel << "_" << modeLabel << ".png";
  writeIPFImage(rgba.get(), imageWidth, imageHeight, filePath.str());
}

} // namespace

// =============================================================================
int main(int argc, char* argv[])
{
  // Parse command-line arguments
  std::string outputDir = ebsdlib::unit_test::k_TestTempDir + "/IPF_Density/";
  size_t numOrientations = 5000;

  if(argc >= 2)
  {
    outputDir = std::string(argv[1]);
    if(outputDir.back() != '/')
    {
      outputDir += '/';
    }
  }
  if(argc >= 3)
  {
    numOrientations = static_cast<size_t>(std::atoi(argv[2]));
    if(numOrientations < 100)
    {
      numOrientations = 100;
    }
  }

  // Create output directory
  std::filesystem::create_directories(outputDir);

  std::cout << "============================================================" << std::endl;
  std::cout << "  Inverse Pole Figure Density Image Generator" << std::endl;
  std::cout << "============================================================" << std::endl;
  std::cout << "  Output directory:    " << outputDir << std::endl;
  std::cout << "  Num orientations:    " << numOrientations << std::endl;
  std::cout << "  Image size:          256 x 256 pixels" << std::endl;
  std::cout << "  Lambert dimension:   64" << std::endl;
  std::cout << "  Normalization:       MRD" << std::endl;
  std::cout << "============================================================" << std::endl;
  std::cout << std::endl;

  int imageWidth = 1024;
  int imageHeight = 1024;
  int lambertDim = 64;

  // Get all LaueOps
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  // ---------------------------------------------------------------
  // Part 1: Random texture for all 11 Laue classes
  // ---------------------------------------------------------------
  std::cout << "--- Part 1: Random Texture (" << numOrientations << " random orientations) ---" << std::endl;
  std::cout << std::endl;

  auto randomEulers = generateRandomEulers(numOrientations, 12345);

  for(size_t index = 0; index < 11; index++)
  {
    generateIPFForLaueClass(*ops[index], randomEulers.get(), outputDir, imageWidth, imageHeight, lambertDim, "Random");
    std::cout << std::endl;
  }

  // ---------------------------------------------------------------
  // Part 2: Single-crystal (Cube) texture for Cubic High symmetry
  // This demonstrates a strong texture producing a concentrated spot.
  // Euler angles (0, 0, 0) = Cube texture: [001] || ND, [100] || RD
  // ---------------------------------------------------------------
  std::cout << "--- Part 2: Single Crystal (Cube) Texture - Cubic High ---" << std::endl;
  std::cout << std::endl;

  auto cubeEulers = generateSingleCrystalEulers(numOrientations, 0.0f, 0.0f, 0.0f);
  generateIPFForLaueClass(*ops[1], cubeEulers.get(), outputDir, imageWidth, imageHeight, lambertDim, "Cube");
  std::cout << std::endl;

  // ---------------------------------------------------------------
  // Part 3: Goss texture for Cubic High symmetry
  // Euler angles (0, pi/4, 0) = Goss texture: {110}<001>
  // ---------------------------------------------------------------
  std::cout << "--- Part 3: Goss Texture - Cubic High ---" << std::endl;
  std::cout << std::endl;

  auto gossEulers = generateSingleCrystalEulers(numOrientations, 0.0f, static_cast<float>(ebsdlib::constants::k_PiOver4D), 0.0f);
  generateIPFForLaueClass(*ops[1], gossEulers.get(), outputDir, imageWidth, imageHeight, lambertDim, "Goss");
  std::cout << std::endl;

  // ---------------------------------------------------------------
  // Part 4: Brass texture for Cubic High symmetry
  // Euler angles (35*pi/180, 45*pi/180, 0) = Brass-like texture: {110}<112>
  // ---------------------------------------------------------------
  std::cout << "--- Part 4: Brass Texture - Cubic High ---" << std::endl;
  std::cout << std::endl;

  float brassE0 = 35.0f * static_cast<float>(ebsdlib::constants::k_DegToRadD);
  float brassE1 = 45.0f * static_cast<float>(ebsdlib::constants::k_DegToRadD);
  float brassE2 = 0.0f;
  auto brassEulers = generateSingleCrystalEulers(numOrientations, brassE0, brassE1, brassE2);
  generateIPFForLaueClass(*ops[1], brassEulers.get(), outputDir, imageWidth, imageHeight, lambertDim, "Brass");
  std::cout << std::endl;

  // ---------------------------------------------------------------
  // Part 5: Texture from quaternion file for all 11 Laue classes
  // Reads quaternion orientations near (0,0,0,1) representing a strong
  // near-cube texture, converts to Euler angles via OrientationConverter,
  // and generates IPF density images (ND direction) for all Laue classes.
  // ---------------------------------------------------------------
  std::cout << "--- Part 5: Quaternion Texture File - All Laue Classes (ND) ---" << std::endl;
  std::cout << std::endl;

  std::string quatFilePath = ebsdlib::unit_test::DataDir + "IPF_Legend/quats_000_1_deg.txt";
  auto textureEulers = readQuaternionFileAsEulers(quatFilePath);
  if(textureEulers != nullptr)
  {
    std::cout << std::endl;

    Matrix3X1D nd(0.0, 0.0, 1.0);
    for(size_t index = 0; index < 11; index++)
    {
      generateSingleIPFForLaueClass(*ops[index], textureEulers.get(), nd, "ND", outputDir, imageWidth, imageHeight, lambertDim, true, "QuatTexture");
      std::cout << std::endl;
    }
  }
  else
  {
    std::cerr << "  Skipping Part 5: Could not load quaternion file." << std::endl;
    std::cout << std::endl;
  }

  std::cout << "============================================================" << std::endl;
  std::cout << "  Done! All IPF density images written to:" << std::endl;
  std::cout << "  " << outputDir << std::endl;
  std::cout << "============================================================" << std::endl;

  return 0;
}
