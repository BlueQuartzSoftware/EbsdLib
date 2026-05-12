#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/CtfPhase.h"
#include "EbsdLib/IO/HKL/CtfReader.h"
#include "EbsdLib/IO/TSL/AngPhase.h"
#include "EbsdLib/IO/TSL/AngReader.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/IColorKey.hpp"
#include "EbsdLib/Utilities/PUCMColorKey.hpp"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

using FloatVec3Type = std::array<float, 3>;

using namespace ebsdlib;

/**
 * @brief The GenerateIPFColorsImpl class computes the IPF colors for each element in a geometry.
 * Uses LaueOps indices directly so it works with both .ang and .ctf phase data.
 */
class GenerateIPFColorsImpl
{
public:
  GenerateIPFColorsImpl(Matrix3X1F& referenceDir, const std::vector<float>& eulers, int32_t* phases, const std::vector<size_t>& laueOpsIndices, bool* goodVoxels, uint8_t* colors,
                        std::vector<LaueOps::Pointer> ops, ebsdlib::ColorKeyKind kind)
  : m_ReferenceDir(referenceDir)
  , m_CellEulerAngles(eulers)
  , m_CellPhases(phases)
  , m_LaueOpsIndices(laueOpsIndices)
  , m_GoodVoxels(goodVoxels)
  , m_CellIPFColors(colors)
  , m_Ops(std::move(ops))
  , m_Kind(kind)
  {
  }

  virtual ~GenerateIPFColorsImpl() = default;

  void run() const
  {
    const std::vector<LaueOps::Pointer>& ops = m_Ops;
    double refDir[3] = {m_ReferenceDir[0], m_ReferenceDir[1], m_ReferenceDir[2]};
    double dEuler[3] = {0.0, 0.0, 0.0};
    ebsdlib::Rgb argb = 0x00000000;
    int32_t phase = 0;
    bool calcIPF = false;
    size_t index = 0;
    int32_t numPhases = static_cast<int32_t>(m_LaueOpsIndices.size());

    size_t totalPoints = m_CellEulerAngles.size() / 3;
    for(size_t i = 0; i < totalPoints; i++)
    {
      phase = m_CellPhases[i];
      index = i * 3;
      m_CellIPFColors[index] = 0;
      m_CellIPFColors[index + 1] = 0;
      m_CellIPFColors[index + 2] = 0;
      dEuler[0] = m_CellEulerAngles[index];
      dEuler[1] = m_CellEulerAngles[index + 1];
      dEuler[2] = m_CellEulerAngles[index + 2];

      calcIPF = true;
      if(nullptr != m_GoodVoxels)
      {
        calcIPF = m_GoodVoxels[i];
      }
      if(phase >= numPhases)
      {
        std::cout << "phase > number of phases" << std::endl;
      }

      size_t currentLaueOpsIndex = m_LaueOpsIndices[phase];

      if(phase < numPhases && calcIPF && currentLaueOpsIndex < ebsdlib::CrystalStructure::LaueGroupEnd)
      {
        argb = ops[currentLaueOpsIndex]->generateIPFColor(dEuler, refDir, false, m_Kind);
        m_CellIPFColors[index] = static_cast<uint8_t>(ebsdlib::RgbColor::dRed(argb));
        m_CellIPFColors[index + 1] = static_cast<uint8_t>(ebsdlib::RgbColor::dGreen(argb));
        m_CellIPFColors[index + 2] = static_cast<uint8_t>(ebsdlib::RgbColor::dBlue(argb));
      }
    }
  }

private:
  Matrix3X1F m_ReferenceDir;
  const std::vector<float>& m_CellEulerAngles;
  int32_t* m_CellPhases;
  std::vector<size_t> m_LaueOpsIndices;

  bool* m_GoodVoxels;
  uint8_t* m_CellIPFColors;
  std::vector<LaueOps::Pointer> m_Ops;
  ebsdlib::ColorKeyKind m_Kind;
};

// -----------------------------------------------------------------------------
// Reads a .ang file and generates an IPF color map image.
// -----------------------------------------------------------------------------
int32_t executeAng(const std::string& filepath, const std::string& outputFile, Matrix3X1F& refDir, const std::vector<LaueOps::Pointer>& ops, ebsdlib::ColorKeyKind kind)
{
  AngReader reader;
  reader.setFileName(filepath);
  int32_t err = reader.readFile();
  if(err < 0)
  {
    std::cerr << "Error reading .ang file: " << filepath << std::endl;
    return err;
  }

  std::vector<int32_t> dims = {reader.getXDimension(), reader.getYDimension()};
  size_t totalPoints = reader.getNumberOfElements();

  // Build LaueOps index vector. Insert a dummy at index 0 since ANG phases are 1-based.
  std::vector<AngPhase::Pointer> angPhases = reader.getPhaseVector();
  std::vector<size_t> laueOpsIndices;
  laueOpsIndices.push_back(0); // Dummy for index 0
  for(const auto& phase : angPhases)
  {
    laueOpsIndices.push_back(phase->determineOrientationOpsIndex());
  }

  Matrix3X1F normRefDir = refDir.normalize();

  // ANG Euler angles are in radians — interleave into a single array
  float* phi1Ptr = reader.getPhi1Pointer(false);
  float* phiPtr = reader.getPhiPointer(false);
  float* phi2Ptr = reader.getPhi2Pointer(false);

  std::vector<float> eulers(3 * totalPoints);
  for(size_t i = 0; i < totalPoints; i++)
  {
    eulers[i * 3] = phi1Ptr[i];
    eulers[i * 3 + 1] = phiPtr[i];
    eulers[i * 3 + 2] = phi2Ptr[i];
  }

  // Map phase 0 (unindexed) to phase 1
  int32_t* phaseData = reader.getPhaseDataPointer(false);
  for(size_t i = 0; i < totalPoints; i++)
  {
    if(phaseData[i] < 1)
    {
      phaseData[i] = 1;
    }
  }

  bool* goodVoxels = nullptr;
  std::vector<uint8_t> ipfColors(totalPoints * 3, 0);
  GenerateIPFColorsImpl generateIPF(normRefDir, eulers, phaseData, laueOpsIndices, goodVoxels, ipfColors.data(), ops, kind);
  generateIPF.run();

  auto error = PngWriter::WriteColorImage(outputFile, dims[0], dims[1], 3, ipfColors.data());
  if(error.first < 0)
  {
    std::cerr << error.second << std::endl;
  }
  return error.first;
}

// -----------------------------------------------------------------------------
// Reads a .ctf file and generates an IPF color map image.
// CTF Euler angles are in degrees and must be converted to radians.
// -----------------------------------------------------------------------------
int32_t executeCtf(const std::string& filepath, const std::string& outputFile, Matrix3X1F& refDir, const std::vector<LaueOps::Pointer>& ops, ebsdlib::ColorKeyKind kind)
{
  CtfReader reader;
  reader.setFileName(filepath);
  int32_t err = reader.readFile();
  if(err < 0)
  {
    std::cerr << "Error reading .ctf file: " << filepath << std::endl;
    return err;
  }

  std::vector<int32_t> dims = {reader.getXDimension(), reader.getYDimension()};
  size_t totalPoints = reader.getNumberOfElements();

  // Build LaueOps index vector. Insert a dummy at index 0 since CTF phases are 1-based.
  std::vector<CtfPhase::Pointer> ctfPhases = reader.getPhaseVector();
  std::vector<size_t> laueOpsIndices;
  laueOpsIndices.push_back(0); // Dummy for index 0
  for(const auto& phase : ctfPhases)
  {
    laueOpsIndices.push_back(phase->determineOrientationOpsIndex());
  }

  Matrix3X1F normRefDir = refDir.normalize();

  // CTF Euler angles are in degrees — convert to radians and interleave
  float* euler1Ptr = reader.getEuler1Pointer();
  float* euler2Ptr = reader.getEuler2Pointer();
  float* euler3Ptr = reader.getEuler3Pointer();
  const float degToRad = static_cast<float>(ebsdlib::constants::k_DegToRadD);

  std::vector<float> eulers(3 * totalPoints);
  for(size_t i = 0; i < totalPoints; i++)
  {
    eulers[i * 3] = euler1Ptr[i] * degToRad;
    eulers[i * 3 + 1] = euler2Ptr[i] * degToRad;
    eulers[i * 3 + 2] = euler3Ptr[i] * degToRad;
  }

  // Map phase 0 (unindexed) to phase 1
  int* phaseData = reader.getPhasePointer();
  std::vector<int32_t> phases(totalPoints);
  for(size_t i = 0; i < totalPoints; i++)
  {
    phases[i] = (phaseData[i] < 1) ? 1 : phaseData[i];
  }

  bool* goodVoxels = nullptr;
  std::vector<uint8_t> ipfColors(totalPoints * 3, 0);
  GenerateIPFColorsImpl generateIPF(normRefDir, eulers, phases.data(), laueOpsIndices, goodVoxels, ipfColors.data(), ops, kind);
  generateIPF.run();

  auto error = PngWriter::WriteColorImage(outputFile, dims[0], dims[1], 3, ipfColors.data());
  if(error.first < 0)
  {
    std::cerr << error.second << std::endl;
  }
  return error.first;
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  if(argc < 3 || argc > 4)
  {
    std::cout << "Usage: make_ipf <input_file.ang|input_file.ctf> <output_image.png> [tsl|pucm]" << std::endl;
    std::cout << "  Optional 3rd argument selects the IPF color key for every Laue class." << std::endl;
    std::cout << "  Default is tsl." << std::endl;
    return 1;
  }

  std::cout << "WARNING: This program makes NO attempt to fix the sample and crystal reference frame issue." << std::endl;
  std::cout << "WARNING: You are probably *not* seeing the correct colors. Use something like DREAM.3D to fully correct for these issues." << std::endl;

  std::string filePath(argv[1]);
  std::string outPath(argv[2]);
  std::string colorKeyName = (argc == 4) ? std::string(argv[3]) : std::string("tsl");
  std::transform(colorKeyName.begin(), colorKeyName.end(), colorKeyName.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if(colorKeyName != "tsl" && colorKeyName != "pucm")
  {
    std::cerr << "ERROR: unknown color key '" << colorKeyName << "', use 'tsl' or 'pucm'" << std::endl;
    return 1;
  }

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  const ebsdlib::ColorKeyKind kind = (colorKeyName == "pucm") ? ebsdlib::ColorKeyKind::PUCM : ebsdlib::ColorKeyKind::TSL;

  // Determine file type from extension
  std::string ext = std::filesystem::path(filePath).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  Matrix3X1F referenceDir = {0.0f, 0.0f, 1.0f};

  std::cout << "Creating IPF Color Map (" << colorKeyName << ") for " << filePath << std::endl;

  int32_t result = -1;
  if(ext == ".ang")
  {
    result = executeAng(filePath, outPath, referenceDir, ops, kind);
  }
  else if(ext == ".ctf")
  {
    result = executeCtf(filePath, outPath, referenceDir, ops, kind);
  }
  else
  {
    std::cerr << "ERROR: Unsupported file extension '" << ext << "'. Use .ang or .ctf" << std::endl;
    return 1;
  }

  if(result < 0)
  {
    std::cerr << "Error creating the IPF Color map" << std::endl;
  }
  return result < 0 ? 1 : 0;
}
