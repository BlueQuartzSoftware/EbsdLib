
#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/EbsdReader.h"
#include "EbsdLib/IO/TSL/AngPhase.h"
#include "EbsdLib/IO/TSL/AngReader.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdMatrixMath.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/TiffWriter.h"

class Ang2IPF;

using FloatVec3Type = std::array<float, 3>;
/**
 * @brief The GenerateIPFColorsImpl class implements a threaded algorithm that computes the IPF
 * colors for each element in a geometry
 */
class GenerateIPFColorsImpl
{
public:
  GenerateIPFColorsImpl(Ang2IPF* filter, FloatVec3Type& referenceDir, const std::vector<float>& eulers, int32_t* phases, std::vector<AngPhase::Pointer>& crystalStructures, bool* goodVoxels,
                        uint8_t* colors)
  : m_Filter(filter)
  , m_ReferenceDir(referenceDir)
  , m_CellEulerAngles(eulers)
  , m_CellPhases(phases)
  , m_PhaseInfos(crystalStructures)
  , m_GoodVoxels(goodVoxels)
  , m_CellIPFColors(colors)
  {
  }

  virtual ~GenerateIPFColorsImpl() = default;

  void run() const
  {
    std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
    double refDir[3] = {m_ReferenceDir[0], m_ReferenceDir[1], m_ReferenceDir[2]};
    double dEuler[3] = {0.0, 0.0, 0.0};
    EbsdLib::Rgb argb = 0x00000000;
    int32_t phase = 0;
    bool calcIPF = false;
    size_t index = 0;
    int32_t numPhases = m_PhaseInfos.size();

    std::vector<size_t> laueOpsIndex(m_PhaseInfos.size());
    for(size_t i = 0; i < laueOpsIndex.size(); i++)
    {
      laueOpsIndex[i] = m_PhaseInfos[i]->determineLaueGroup();
    }

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

      // Make sure we are using a valid Euler Angles with valid crystal symmetry
      calcIPF = true;
      if(nullptr != m_GoodVoxels)
      {
        calcIPF = m_GoodVoxels[i];
      }
      // Sanity check the phase data to make sure we do not walk off the end of the array
      if(phase >= numPhases)
      {
        // m_Filter->incrementPhaseWarningCount();
        std::cout << "phase > number of phases" << std::endl;
      }

      size_t currentLaueOpsIndex = laueOpsIndex[phase];

      if(phase < numPhases && calcIPF && currentLaueOpsIndex < EbsdLib::CrystalStructure::LaueGroupEnd)
      {
        argb = ops[currentLaueOpsIndex]->generateIPFColor(dEuler, refDir, false);
        m_CellIPFColors[index] = static_cast<uint8_t>(EbsdLib::RgbColor::dRed(argb));
        m_CellIPFColors[index + 1] = static_cast<uint8_t>(EbsdLib::RgbColor::dGreen(argb));
        m_CellIPFColors[index + 2] = static_cast<uint8_t>(EbsdLib::RgbColor::dBlue(argb));

        //  std::cout << (int32_t)(m_CellIPFColors[index]) << "\t" << (int32_t)(m_CellIPFColors[index + 1]) << (int32_t)(m_CellIPFColors[index + 2]) << m_CellEulerAngles[index] << "\t"
        //            << m_CellEulerAngles[index + 1] << "\t" << m_CellEulerAngles[index + 2] << std::endl;
      }
    }
  }

private:
  Ang2IPF* m_Filter = nullptr;
  FloatVec3Type m_ReferenceDir;
  const std::vector<float>& m_CellEulerAngles;
  int32_t* m_CellPhases;
  std::vector<AngPhase::Pointer> m_PhaseInfos;

  bool* m_GoodVoxels;
  uint8_t* m_CellIPFColors;
};

// -----------------------------------------------------------------------------
class Ang2IPF
{
public:
  Ang2IPF()
  {
  }
  ~Ang2IPF() = default;

  Ang2IPF(const Ang2IPF&) = delete;            // Copy Constructor Not Implemented
  Ang2IPF(Ang2IPF&&) = delete;                 // Move Constructor Not Implemented
  Ang2IPF& operator=(const Ang2IPF&) = delete; // Copy Assignment Not Implemented
  Ang2IPF& operator=(Ang2IPF&&) = delete;      // Move Assignment Not Implemented

  std::array<float, 3> m_ReferenceDir = {0.0F, 0.0F, 1.0F};

  /**
   * @brief incrementPhaseWarningCount
   */
  void incrementPhaseWarningCount()
  {
    m_PhaseWarningCount++;
  }

  /**
   * @brief execute
   * @return
   */
  int32_t execute(const std::string& filepath, const std::string& outputFile)
  {
    m_PhaseWarningCount = 0;
    AngReader reader;
    reader.setFileName(filepath);
    int32_t err = reader.readFile();
    if(err < 0)
    {
      return err;
    }

    std::vector<int32_t> dims = {reader.getXDimension(), reader.getYDimension()};

    size_t totalPoints = reader.getNumberOfElements();
    std::vector<AngPhase::Pointer> crystalStructures = reader.getPhaseVector();
    crystalStructures.emplace(crystalStructures.begin(), AngPhase::New());
    // int32_t numPhase = static_cast<int32_t>(crystalStructures.size());

    // Make sure we are dealing with a unit 1 vector.
    std::array<float, 3> normRefDir = m_ReferenceDir; // Make a copy of the reference Direction
    EbsdMatrixMath::Normalize3x1(normRefDir[0], normRefDir[1], normRefDir[2]);

    float* phi1Ptr = reader.getPhi1Pointer(false);
    float* phiPtr = reader.getPhiPointer(false);
    float* phi2Ptr = reader.getPhi2Pointer(false);

    // We need to interleave the phi1, PHI, phi2 data into a single 3 component array
    std::vector<float> eulers(3 * totalPoints);

    for(size_t i = 0; i < totalPoints; i++)
    {
      eulers[i * 3] = phi1Ptr[i];
      eulers[i * 3 + 1] = phiPtr[i];
      eulers[i * 3 + 2] = phi2Ptr[i];
    }

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
    GenerateIPFColorsImpl generateIPF(this, normRefDir, eulers, phaseData, crystalStructures, goodVoxels, ipfColors.data());
    generateIPF.run();

    std::pair<int32_t, std::string> error = TiffWriter::WriteColorImage(outputFile, dims[0], dims[1], 3, ipfColors.data());
    if(error.first < 0)
    {
      std::cout << error.second << std::endl;
    }
    return error.first;
  }

private:
  int32_t m_PhaseWarningCount = {0};
};

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  if(argc != 3)
  {
    std::cout << "Program needs file path to .ang file" << std::endl;
    return 1;
  }
  std::cout << "WARNING: This program makes NO attempt to fix the sample and crystal reference frame issue that is common on TSL systems." << std::endl;
  std::cout << "WARNING: You are probably *not* seeing the correct colors. Use something like DREAM.3D to fully correct for these issues." << std::endl;
  std::string filePath(argv[1]);
  std::string outPath(argv[2]);
  std::cout << "Creating IPF Color Map for " << filePath << std::endl;

  Ang2IPF Ang2IPF;
  if(Ang2IPF.execute(filePath, outPath) < 0)
  {
    std::cout << "Error creating the IPF Color map" << std::endl;
  }
  return 0;
}
