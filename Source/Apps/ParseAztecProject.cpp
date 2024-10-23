
#include "EbsdLib/IO/HKL/CprReader.h"
#include "EbsdLib/LaueOps/LaueOps.h"

#include <iostream>

namespace detail
{
// const std::string k_CprPath = "/Volumes/OWC_Express_1M2/DREAM3D_Troubleshoot_Data/Benjamin_Layer1/Layer1.cpr";
const std::string k_CprPath = "/Volumes/OWC_Express_1M2/CPR_CRC_Test_Data/dg7kv5mcv3-1/17NZ42_Dauphinetwinnedsample_nearlyuntwinnedgrains.cpr";

} // namespace detail

std::string ConvertLaueGroupToString(int idx)
{
  switch(idx)
  {
  case 0:
    return "Hexagonal_High";
  case 1:
    return "Cubic_High";
  case 2:
    return "Hexagonal_Low";
  case 3:
    return "Cubic_Low";
  case 4:
    return "Triclinic";
  case 5:
    return "Monoclinic";
  case 6:
    return "OrthoRhombic";
  case 7:
    return "Tetragonal_Low";
  case 8:
    return "Tetragonal_High";
  case 9:
    return "Trigonal_Low";
  case 10:
    return "Trigonal_High";
  case 11:
    return "UnknownCrystalStructure";
  }
  return "Undefined";
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{

  //  if(argc != 3)
  //  {
  //    std::cout << "Program .cpr and .crc files as input." << std::endl;
  //    return 1;
  //  }

  CprReader reader;
  reader.setFileName(detail::k_CprPath);
  int error = reader.readHeaderOnly();
  if(error < 0)
  {
    std::cout << "Reading Header Failed: " << error << "\n";
    return 1;
  }
  // reader.printHeader(std::cout);
  // Write out a compatible DREAM3D "Ensemble" File
  auto phases = reader.getPhaseVector();
  std::cout << "[EnsembleInfo]\n"
            << "Number_Phases=" << phases.size() << std::endl;
  size_t idx = 0;
  for(const auto& phase : phases)
  {
    std::cout << std::endl;
    std::cout << "[" << ++idx << "]\n"
              << "CrystalStructure=" << ConvertLaueGroupToString(phase->determineOrientationOpsIndex()) << "\n"
              << "PhaseType=PrimaryPhase\n";
  }

  error = reader.readFile();
  if(error < 0)
  {
    std::cout << reader.getErrorMessage() << error << "\n";
    return 2;
  }

  // Dump all the data to the command line
  //  std::cout << "Phase,Bands,Error,Euler1,Euler2,Euler3,MAD,BC,BS\n";
  //  uint8_t* phasePtr = reader.getPhasePointer();
  //  uint8_t* bandsPtr = reader.getBandCountPointer();
  //  uint8_t* errorPtr = reader.getErrorPointer();
  //  float* phi1Ptr = reader.getEuler1Pointer();
  //  float* phiPtr = reader.getEuler2Pointer();
  //  float* phi2Ptr = reader.getEuler3Pointer();
  //  float* madPtr = reader.getMeanAngularDeviationPointer();
  //  uint8_t* bcPtr = reader.getBandContrastPointer();
  //  uint8_t* bsPtr = reader.getBandSlopePointer();
  //
  //  size_t numScanPoints = reader.getNumberOfElements();
  //  for(size_t i = 0; i < numScanPoints; i++)
  //  {
  //    std::cout << static_cast<int>(phasePtr[i]) << "," << static_cast<int>(bandsPtr[i]) << "," << static_cast<int>(errorPtr[i]) << "," << phi1Ptr[i] << "," << phiPtr[i] << "," << phi2Ptr[i] << ","
  //              << madPtr[i] << "," << static_cast<int>(bcPtr[i]) << "," << static_cast<int>(bsPtr[i]) << std::endl;
  //  }

  return 0;
}
