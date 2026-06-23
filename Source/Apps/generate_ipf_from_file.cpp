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
 * @file generate_ipf_from_file.cpp
 * @brief Example program that reads a .ctf or .ang EBSD data file and generates
 * Inverse Pole Figure (IPF) density images for each phase found in the data.
 *
 * For each phase, 3 TIFF images are generated corresponding to 3 orthogonal
 * sample directions: RD (Rolling Direction), TD (Transverse Direction), and
 * ND (Normal Direction).
 *
 * Usage:
 *   generate_ipf_from_file <input_file.ang|input_file.ctf> [output_directory]
 *
 * If no output directory is specified, images are written next to the input file.
 */

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/CtfPhase.h"
#include "EbsdLib/IO/HKL/CtfReader.h"
#include "EbsdLib/IO/TSL/AngPhase.h"
#include "EbsdLib/IO/TSL/AngReader.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/InversePoleFigureUtilities.h"
#include "EbsdLib/Utilities/PngWriter.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{

// -----------------------------------------------------------------------
// Generate and save annotated IPF density images for a given set of Euler
// angles using a specific LaueOps instance.
// -----------------------------------------------------------------------
void generateIPFForPhase(const LaueOps& ops, ebsdlib::FloatArrayType* eulers, const std::string& outputDir, int imageWidth, int imageHeight, int lambertDim, const std::string& phaseLabel)
{
  std::string className = ops.getSymmetryName();
  std::cout << "Generating annotated IPF density for phase: " << phaseLabel << " (" << className << ", " << eulers->getNumberOfTuples() << " orientations)" << std::endl;

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
  config.phaseName = phaseLabel;
  config.FlipFinalImage = false;

  auto images = ops.generateAnnotatedIPFDensity(config);

  // Sanitize phase name for filename
  std::string safeName = phaseLabel;
  for(auto& c : safeName)
  {
    if(c == '/' || c == '\\' || c == ' ' || c == '(' || c == ')')
    {
      c = '_';
    }
  }

  // Images are RGB (3 components), canvasDim x canvasDim
  int canvasDim = static_cast<int>(static_cast<float>(imageWidth) * 1.5f);
  for(size_t i = 0; i < images.size(); i++)
  {
    std::ostringstream filePath;
    filePath << outputDir << "/" << safeName << "_IPF_" << config.labels[i] << ".png";
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
// Holds orientation data extracted from an EBSD file, grouped by phase.
// -----------------------------------------------------------------------
struct PhaseData
{
  std::string phaseName;
  unsigned int laueOpsIndex = ebsdlib::CrystalStructure::UnknownCrystalStructure;
  ebsdlib::FloatArrayType::Pointer eulers;
};

// -----------------------------------------------------------------------
// Read a .ang file and return per-phase orientation data.
// ANG files store Euler angles in radians.
// -----------------------------------------------------------------------
std::vector<PhaseData> readAngFile(const std::string& filePath)
{
  AngReader reader;
  reader.setFileName(filePath);
  int err = reader.readFile();
  if(err < 0)
  {
    std::cerr << "ERROR: Failed to read .ang file: " << filePath << std::endl;
    return {};
  }

  size_t totalPoints = reader.getNumberOfElements();
  std::cout << "  Read " << totalPoints << " data points (" << reader.getXDimension() << " x " << reader.getYDimension() << ")" << std::endl;

  float* phi1 = reader.getPhi1Pointer(false);
  float* phi = reader.getPhiPointer(false);
  float* phi2 = reader.getPhi2Pointer(false);
  int* phaseData = reader.getPhaseDataPointer(false);

  std::vector<AngPhase::Pointer> phases = reader.getPhaseVector();

  // Build a map from phase index to LaueOps index and phase name.
  // ANG phase indices are 1-based.
  std::map<int, unsigned int> phaseToLaueOps;
  std::map<int, std::string> phaseToName;
  for(const auto& phase : phases)
  {
    int idx = phase->getPhaseIndex();
    phaseToLaueOps[idx] = phase->determineOrientationOpsIndex();
    std::string name = phase->getMaterialName();
    if(name.empty())
    {
      name = "Phase_" + std::to_string(idx);
    }
    phaseToName[idx] = name;
    std::cout << "  Phase " << idx << ": " << name << " (LaueOps index: " << phaseToLaueOps[idx] << ")" << std::endl;
  }

  // Group Euler angles by phase.
  // ANG phase data uses 0 for unindexed points; map those to phase 1
  // when phase 1 exists (consistent with make_ipf.cpp behavior).
  std::map<int, std::vector<float>> phaseEulerMap;
  for(size_t i = 0; i < totalPoints; i++)
  {
    int p = phaseData[i];
    if(p < 1 && phaseToLaueOps.find(1) != phaseToLaueOps.end())
    {
      p = 1;
    }
    if(phaseToLaueOps.find(p) == phaseToLaueOps.end())
    {
      continue;
    }
    if(phaseToLaueOps[p] >= ebsdlib::CrystalStructure::LaueGroupEnd)
    {
      continue;
    }
    phaseEulerMap[p].push_back(phi1[i]);
    phaseEulerMap[p].push_back(phi[i]);
    phaseEulerMap[p].push_back(phi2[i]);
  }

  // Convert grouped data into PhaseData structs
  std::vector<PhaseData> result;
  for(auto& [phaseIdx, eulerVec] : phaseEulerMap)
  {
    size_t numOrientations = eulerVec.size() / 3;
    if(numOrientations == 0)
    {
      continue;
    }

    PhaseData pd;
    pd.phaseName = phaseToName[phaseIdx];
    pd.laueOpsIndex = phaseToLaueOps[phaseIdx];

    std::vector<size_t> cDims = {3};
    pd.eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);
    std::memcpy(pd.eulers->getVoidPointer(0), eulerVec.data(), eulerVec.size() * sizeof(float));

    result.push_back(std::move(pd));
  }

  return result;
}

// -----------------------------------------------------------------------
// Read a .ctf file and return per-phase orientation data.
// CTF files store Euler angles in degrees; we convert to radians.
// -----------------------------------------------------------------------
std::vector<PhaseData> readCtfFile(const std::string& filePath)
{
  CtfReader reader;
  reader.setFileName(filePath);
  int err = reader.readFile();
  if(err < 0)
  {
    std::cerr << "ERROR: Failed to read .ctf file: " << filePath << std::endl;
    return {};
  }

  size_t totalPoints = reader.getNumberOfElements();
  std::cout << "  Read " << totalPoints << " data points (" << reader.getXDimension() << " x " << reader.getYDimension() << ")" << std::endl;

  float* euler1 = reader.getEuler1Pointer();
  float* euler2 = reader.getEuler2Pointer();
  float* euler3 = reader.getEuler3Pointer();
  int* phaseData = reader.getPhasePointer();

  std::vector<CtfPhase::Pointer> phases = reader.getPhaseVector();

  // Build a map from phase index to LaueOps index and phase name.
  // CTF phase indices are 1-based.
  std::map<int, unsigned int> phaseToLaueOps;
  std::map<int, std::string> phaseToName;
  for(const auto& phase : phases)
  {
    int idx = phase->getPhaseIndex();
    phaseToLaueOps[idx] = phase->determineOrientationOpsIndex();
    std::string name = phase->getPhaseName();
    if(name.empty())
    {
      name = "Phase_" + std::to_string(idx);
    }
    phaseToName[idx] = name;
    std::cout << "  Phase " << idx << ": " << name << " (LaueOps index: " << phaseToLaueOps[idx] << ")" << std::endl;
  }

  // Group Euler angles by phase, converting degrees to radians.
  // CTF phase data uses 0 for unindexed points; map those to phase 1
  // when phase 1 exists.
  const float degToRad = static_cast<float>(ebsdlib::constants::k_DegToRadD);
  std::map<int, std::vector<float>> phaseEulerMap;
  for(size_t i = 0; i < totalPoints; i++)
  {
    int p = phaseData[i];
    if(p < 1 && phaseToLaueOps.find(1) != phaseToLaueOps.end())
    {
      p = 1;
    }
    if(phaseToLaueOps.find(p) == phaseToLaueOps.end())
    {
      continue;
    }
    if(phaseToLaueOps[p] >= ebsdlib::CrystalStructure::LaueGroupEnd)
    {
      continue;
    }
    phaseEulerMap[p].push_back(euler1[i] * degToRad);
    phaseEulerMap[p].push_back(euler2[i] * degToRad);
    phaseEulerMap[p].push_back(euler3[i] * degToRad);
  }

  // Convert grouped data into PhaseData structs
  std::vector<PhaseData> result;
  for(auto& [phaseIdx, eulerVec] : phaseEulerMap)
  {
    size_t numOrientations = eulerVec.size() / 3;
    if(numOrientations == 0)
    {
      continue;
    }

    PhaseData pd;
    pd.phaseName = phaseToName[phaseIdx];
    pd.laueOpsIndex = phaseToLaueOps[phaseIdx];

    std::vector<size_t> cDims = {3};
    pd.eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);
    std::memcpy(pd.eulers->getVoidPointer(0), eulerVec.data(), eulerVec.size() * sizeof(float));

    result.push_back(std::move(pd));
  }

  return result;
}

} // namespace

// =============================================================================
int main(int argc, char* argv[])
{
  if(argc < 2)
  {
    std::cout << "Usage: generate_ipf_from_file <input_file.ang|input_file.ctf> [output_directory]" << std::endl;
    std::cout << std::endl;
    std::cout << "Reads an EBSD data file and generates Inverse Pole Figure density" << std::endl;
    std::cout << "images (RD, TD, ND) for each phase found in the data." << std::endl;
    return 1;
  }

  std::string inputFile = argv[1];
  std::filesystem::path inputPath(inputFile);

  if(!std::filesystem::exists(inputPath))
  {
    std::cerr << "ERROR: Input file does not exist: " << inputFile << std::endl;
    return 1;
  }

  // Determine output directory
  std::string outputDir;
  if(argc >= 3)
  {
    outputDir = argv[2];
  }
  else
  {
    outputDir = inputPath.parent_path().string();
    if(outputDir.empty())
    {
      outputDir = ".";
    }
  }

  // Create output directory if needed
  std::filesystem::create_directories(outputDir);

  // Determine file type from extension
  std::string ext = inputPath.extension().string();
  for(auto& c : ext)
  {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }

  std::cout << "============================================================" << std::endl;
  std::cout << "  IPF Density from EBSD File" << std::endl;
  std::cout << "============================================================" << std::endl;
  std::cout << "  Input file:      " << inputFile << std::endl;
  std::cout << "  File type:       " << ext << std::endl;
  std::cout << "  Output directory: " << outputDir << std::endl;
  std::cout << "============================================================" << std::endl;
  std::cout << std::endl;

  // Read the file
  std::vector<PhaseData> phaseDataVec;
  if(ext == ".ang")
  {
    std::cout << "Reading .ang file..." << std::endl;
    phaseDataVec = readAngFile(inputFile);
  }
  else if(ext == ".ctf")
  {
    std::cout << "Reading .ctf file..." << std::endl;
    phaseDataVec = readCtfFile(inputFile);
  }
  else
  {
    std::cerr << "ERROR: Unsupported file extension '" << ext << "'. Use .ang or .ctf" << std::endl;
    return 1;
  }

  if(phaseDataVec.empty())
  {
    std::cerr << "ERROR: No valid phase data found in file." << std::endl;
    return 1;
  }

  std::cout << std::endl;
  std::cout << "Found " << phaseDataVec.size() << " phase(s) with valid orientations." << std::endl;
  std::cout << std::endl;

  // Image generation parameters
  int imageWidth = 1024;
  int imageHeight = 1024;
  int lambertDim = 64;

  // Get all LaueOps
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  // Generate IPF density images for each phase
  for(const auto& pd : phaseDataVec)
  {
    if(pd.laueOpsIndex >= ops.size())
    {
      std::cerr << "  Skipping phase '" << pd.phaseName << "': invalid LaueOps index " << pd.laueOpsIndex << std::endl;
      continue;
    }

    generateIPFForPhase(*ops[pd.laueOpsIndex], pd.eulers.get(), outputDir, imageWidth, imageHeight, lambertDim, pd.phaseName);
    std::cout << std::endl;
  }

  std::cout << "============================================================" << std::endl;
  std::cout << "  Done! All IPF density images written to:" << std::endl;
  std::cout << "  " << outputDir << std::endl;
  std::cout << "============================================================" << std::endl;

  return 0;
}
