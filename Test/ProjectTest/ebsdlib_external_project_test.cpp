

#include "EbsdLib/LaueOps/TriclinicOps.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/TiffWriter.h"

#include <filesystem>
#include <sstream>
#include <string>

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  const std::string k_Output_Dir("Triclinic IPF Legend");
  std::filesystem::create_directories(k_Output_Dir);

  std::stringstream ss;
  int imageDim = 512;

  {
    TriclinicOps ops;

    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "|");
    std::filesystem::create_directories(ss.str());

    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "|") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "|") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
  }

  return 0;
}
