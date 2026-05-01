/* ============================================================================
 * make_pole_figure
 *
 * Reads a .ang or .ctf EBSD data file and produces, for each indexed phase, a
 * single composite pole-figure PNG using EbsdLib's PoleFigureCompositor.
 * Output is a horizontal layout (3 default plane families + legend side by
 * side) using continuous color-intensity rendering (Lambert-projection
 * density, not discrete points).
 *
 * Output filenames: <output_dir>/EbsdLib_Phase_<N>.png  where N is the phase
 * index from the input file (1-based).
 *
 * Usage:
 *   make_pole_figure <input_file.ang|input_file.ctf> <output_directory>
 *
 * Example:
 *   make_pole_figure /path/to/12.ang /path/to/Output/12/PoleFigures
 *   -> writes /path/to/Output/12/PoleFigures/EbsdLib_Phase_1.png (etc.)
 * ============================================================================ */
#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/CtfPhase.h"
#include "EbsdLib/IO/HKL/CtfReader.h"
#include "EbsdLib/IO/TSL/AngPhase.h"
#include "EbsdLib/IO/TSL/AngReader.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{
struct PhaseData
{
  int phaseIndex = 0;
  std::string phaseName;
  unsigned int laueOpsIndex = ebsdlib::CrystalStructure::UnknownCrystalStructure;
  ebsdlib::FloatArrayType::Pointer eulers;
};

// ---------------------------------------------------------------------------
// Read a TSL .ang file. Eulers are already in radians.
// Phase 0 (unindexed) is remapped to phase 1 (matching the existing apps'
// behavior) so users with single-phase scans get all data.
// ---------------------------------------------------------------------------
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
  int* phaseDataArr = reader.getPhaseDataPointer(false);
  float* ci = reader.getConfidenceIndexPointer(false);

  std::vector<AngPhase::Pointer> phases = reader.getPhaseVector();

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

  // Eulers are passed straight through to LaueOps. The legacy phi2-30° basal-
  // plane shift and the 90°-about-z sample-frame rotation that used to live
  // here have been removed; convention handling is now done inside LaueOps
  // via config.hexConvention = XParallelA below.
  std::map<int, std::vector<float>> phaseEulerMap;
  for(size_t i = 0; i < totalPoints; i++)
  {
    int p = phaseDataArr[i];
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
    if(ci[i] > 0.1)
    {
      phaseEulerMap[p].push_back(phi1[i]);
      phaseEulerMap[p].push_back(phi[i]);
      phaseEulerMap[p].push_back(phi2[i]);
    }
  }

  std::vector<PhaseData> result;
  for(auto& [idx, eulerVec] : phaseEulerMap)
  {
    size_t numOrientations = eulerVec.size() / 3;
    if(numOrientations == 0)
    {
      continue;
    }
    std::cout << "  Phase " << idx << " Num. Eulers: " << numOrientations << std::endl;
    PhaseData pd;
    pd.phaseIndex = idx;
    pd.phaseName = phaseToName[idx];
    pd.laueOpsIndex = phaseToLaueOps[idx];
    std::vector<size_t> cDims = {3};
    pd.eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);
    std::memcpy(pd.eulers->getVoidPointer(0), eulerVec.data(), eulerVec.size() * sizeof(float));
    result.push_back(std::move(pd));
  }
  return result;
}

// ---------------------------------------------------------------------------
// Read an Oxford .ctf file. Eulers are stored in degrees in the file and
// converted to radians here.
// ---------------------------------------------------------------------------
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
  int* phaseDataArr = reader.getPhasePointer();

  std::vector<CtfPhase::Pointer> phases = reader.getPhaseVector();

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

  const float degToRad = static_cast<float>(ebsdlib::constants::k_DegToRadD);
  std::map<int, std::vector<float>> phaseEulerMap;
  for(size_t i = 0; i < totalPoints; i++)
  {
    int p = phaseDataArr[i];
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

  std::vector<PhaseData> result;
  for(auto& [idx, eulerVec] : phaseEulerMap)
  {
    size_t numOrientations = eulerVec.size() / 3;
    if(numOrientations == 0)
    {
      continue;
    }
    PhaseData pd;
    pd.phaseIndex = idx;
    pd.phaseName = phaseToName[idx];
    pd.laueOpsIndex = phaseToLaueOps[idx];
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
  if(argc != 3)
  {
    std::cout << "Usage: make_pole_figure <input_file.ang|input_file.ctf> <output_directory>" << std::endl;
    std::cout << std::endl;
    std::cout << "Reads an EBSD data file and produces, for each indexed phase, one" << std::endl;
    std::cout << "composite pole-figure PNG using PoleFigureCompositor (horizontal layout," << std::endl;
    std::cout << "color-intensity rendering). Output filenames: EbsdLib_Phase_<N>.png" << std::endl;
    return 1;
  }

  const std::string inputFile = argv[1];
  const std::string outputDir = argv[2];

  std::filesystem::path inputPath(inputFile);
  if(!std::filesystem::exists(inputPath))
  {
    std::cerr << "ERROR: Input file does not exist: " << inputFile << std::endl;
    return 1;
  }

  std::filesystem::create_directories(outputDir);

  std::string ext = inputPath.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  std::cout << "============================================================" << std::endl;
  std::cout << "  make_pole_figure (PoleFigureCompositor / color intensity)" << std::endl;
  std::cout << "============================================================" << std::endl;
  std::cout << "  Input:    " << inputFile << std::endl;
  std::cout << "  Output:   " << outputDir << std::endl;
  std::cout << "============================================================" << std::endl;

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

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  for(const auto& pd : phaseDataVec)
  {
    if(pd.laueOpsIndex >= ops.size())
    {
      std::cerr << "  Skipping phase '" << pd.phaseName << "': invalid LaueOps index " << pd.laueOpsIndex << std::endl;
      continue;
    }

    LaueOps::Pointer op = ops[pd.laueOpsIndex];

    CompositePoleFigureConfiguration_t config;
    config.eulers = pd.eulers.get();
    config.imageDim = 512;
    config.lambertDim = 64;
    config.numColors = 32;
    config.minScale = 0.0;
    config.maxScale = 0.0; // 0 = auto-scale
    config.sphereRadius = 1.0F;
    config.discrete = false;        // continuous color intensity
    config.discreteHeatMap = false; // (only relevant when discrete=true)
    config.flipFinalImage = true;
    config.layoutType = PoleFigureLayoutType::Horizontal;
    config.laueOpsIndex = static_cast<uint32_t>(pd.laueOpsIndex);
    config.phaseName = pd.phaseName;
    config.phaseNumber = pd.phaseIndex;
    // make_pole_figure ingests TSL .ang / Oxford .ctf files, both of which
    // store orientations in the X||a (legacy / OIM-Analysis) basis. Pass that
    // through to LaueOps so the convention bridge is applied internally.
    config.hexConvention = ebsdlib::HexConvention::XParallelA;
    config.title = pd.phaseName + " (" + op->getSymmetryName() + ")";

    auto pfNames = op->getDefaultPoleFigureNames(config.hexConvention);
    config.labels = {pfNames[0], pfNames[1], pfNames[2]};
    config.order = {0, 1, 2};

    std::cout << std::endl;
    std::cout << "Generating composite for phase " << pd.phaseIndex << " (" << pd.phaseName << ", " << op->getSymmetryName() << ", " << pd.eulers->getNumberOfTuples() << " orientations)" << std::endl;

    PoleFigureCompositor compositor;
    CompositePoleFigureResult result = compositor.generateCompositeImage(config);
    if(result.image == nullptr || result.width <= 0 || result.height <= 0)
    {
      std::cerr << "  ERROR: PoleFigureCompositor returned an empty image." << std::endl;
      continue;
    }

    std::ostringstream filePath;
    filePath << outputDir << "/EbsdLib_Phase_" << pd.phaseIndex << ".png";
    auto writeResult = PngWriter::WriteColorImage(filePath.str(), result.width, result.height, 4, result.image->data());
    if(writeResult.first < 0)
    {
      std::cerr << "  ERROR writing " << filePath.str() << ": " << writeResult.second << std::endl;
    }
    else
    {
      std::cout << "  Wrote: " << filePath.str() << " (" << result.width << " x " << result.height << ")" << std::endl;
    }
  }

  std::cout << std::endl;
  std::cout << "============================================================" << std::endl;
  std::cout << "  Done." << std::endl;
  std::cout << "============================================================" << std::endl;
  return 0;
}
