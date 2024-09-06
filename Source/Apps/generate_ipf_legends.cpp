
#include "EbsdLib/Utilities/TiffWriter.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/LaueOps/TrigonalOps.h"
#include "EbsdLib/LaueOps/OrthoRhombicOps.h"

#include <string>
#include <sstream>


const std::string k_Output_Dir("/tmp/ebsd_lib_legends/");

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  std::vector<float> data = {1.0F, 2.0F, 3.0F, 0.0F, 0.0F, 0.0F};
  std::transform(data.begin(), data.begin() + 3,
                 data.begin() + 3,           // write to the next triplet in memory
                 [](float value) { return value *= -1.0F; });

  for(const auto& value : data)
  {
    std::cout << value << std::endl;
  }

  int imageDim = 512;
  std::stringstream ss;
//  {
//    TrigonalOps ops;
//    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
//    ss.str("");
//    ss << k_Output_Dir << ops.getSymmetryName() << "_FULL.tiff";
//    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 4, legend->getPointer(0));
//    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
//    legend = ops.generateIPFTriangleLegend(imageDim, false);
//    ss.str("");
//    ss << k_Output_Dir << ops.getSymmetryName() << ".tiff";
//    result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 4, legend->getPointer(0));
//    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
//  }


  {
    OrthoRhombicOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << ops.getSymmetryName() << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 4, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
    legend = ops.generateIPFTriangleLegend(imageDim, false);
    ss.str("");
    ss << k_Output_Dir << ops.getSymmetryName() << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 4, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
  }
  return 0;
}
