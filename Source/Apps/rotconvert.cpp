#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <valarray>
#include <vector>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

std::map<std::string, int32_t> counts = {{"eu", 3}, {"om", 9}, {"ax", 4}, {"ro", 3}, {"qu", 4}, {"ho", 3}, {"cu", 3}};
std::vector<std::string> reps = {"eu", "om", "ax", "ro", "qu", "ho", "cu"};

// -----------------------------------------------------------------------------
void GenerateMethods()
{

  for(const auto& r1 : reps)
  {
    for(const auto& r2 : reps)
    {
      if(r1 == r2)
      {
        continue;
      }
      auto upper = r2;
      std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
      std::cout << "//--------------------------------------------------------------------------------" << std::endl;
      if(r1 == "qu")
      {
        std::cout << "void " << r1 << "2" << r2 << "(const QuatD& input)\n{" << std::endl;
        std::cout << "  OrientationD " << r2 << " = OrientationTransformation::" << r1 << "2" << r2 << "<QuatD,OrientationD>({"
                  << "input[0],input[1],input[2],input[4]"
                  << "});" << std::endl;
        std::cout << "  OrientationPrinters::Print_" << upper << "(" << r2 << ");" << std::endl;

        std::cout << "}" << std::endl;
      }
      else if(r2 == "qu")
      {
        std::cout << "void " << r1 << "2" << r2 << "(const OrientationD& input)\n{" << std::endl;
        std::cout << "  QuatD " << r2 << " = OrientationTransformation::" << r1 << "2" << r2 << "<OrientationD,QuatD>({";
        int32_t count = counts[r1];
        for(int32_t i = 0; i < count; i++)
        {
          std::cout << "input[" << static_cast<char>(i + 48) << "]";
          if(i < count - 1)
          {
            std::cout << ",";
          }
        }
        std::cout << "});" << std::endl;
        std::cout << "  OrientationPrinters::Print_" << upper << "(" << r2 << ");" << std::endl;
        std::cout << "}" << std::endl;
      }
      else
      {
        std::cout << "void " << r1 << "2" << r2 << "(const OrientationD& input)\n{" << std::endl;
        std::cout << "  OrientationD " << r2 << " = OrientationTransformation::" << r1 << "2" << r2 << "<OrientationD,OrientationD>({";
        int32_t count = counts[r1];
        for(int32_t i = 0; i < count; i++)
        {
          std::cout << "input[" << static_cast<char>(i + 48) << "]";
          if(i < count - 1)
          {
            std::cout << ",";
          }
        }
        std::cout << "});" << std::endl;
        std::cout << "  OrientationPrinters::Print_" << upper << "(" << r2 << ");" << std::endl;
        std::cout << "}" << std::endl;
      }
    }
  }
}

// -----------------------------------------------------------------------------
void GenerateLogicBlock()
{
  for(const auto& r1 : reps)
  {
    for(const auto& r2 : reps)
    {
      if(r1 == r2)
      {
        continue;
      }
      auto upper = r2;
      std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

      std::cout << "\nelse if(conversion == \"" << r1 << "2" << r2 << "\")\n{\n";

      if(r1 == "qu")
      {
        std::cout << "  QuatD " << r1 << "(array.data());\n";
      }
      else
      {
        std::cout << "  OrientationD " << r1 << "(array.data()," << counts[r1] << ");\n";
      }
      std::cout << "  OrientationTransformation::ResultType res = OrientationTransformation::" << r1 << "_check(" << r1 << ");\n";
      std::cout << "  if(res.result < 0) { std::cout << res.msg << std::endl;return 1;}\n";
      std::cout << "  " << r1 << "2" << r2 << "(" << r1 << ");\n";
      std::cout << "}";
    }
  }
  std::cout << std::endl;
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  if(argc != 4)
  {
    std::cout << "3 Arguments are needed IN ORDER. All arguments are needed." << std::endl;
    std::cout << "[1] The conversion type listed as xx2yy where xx and yy are one of:" << std::endl;
    for(const auto& rep : reps)
    {
      std::cout << "  " << rep << std::endl;
    }
    std::cout << "[2] Degrees or Radians (d | r)" << std::endl;
    std::cout << "[3] The input values as a comma separated list of values." << std::endl;
    std::cout << "  Note the following conventions:\n"
              << "   om (Orientation Matrix): row moving the fastest\n"
              << "   ax (Axis Angle): Vector Scalar <x, y, z> w\n"
              << "   ro (Rodrigues): 3 Components\n"
              << "   qu (Quaternion): Vector Scalar <x, y, z> w\n"
              << std::endl;
    std::cout << "Example invocation: rotconvert eu2qu d 23.4,45.6,87.23" << std::endl;
    return 1;
  }
  // rotconvert eu2qu d 23.4,45.6,87.23

  // Arg position 1 is the conversion type
  // Arg position 2 is the angle rep (-d for Degrees, -r for Radians)
  // Arg position 3 is the input
  GenerateLogicBlock();

  std::string conversion(argv[1]);
  EbsdStringUtils::StringTokenType tokens = EbsdStringUtils::split(argv[3], ',');
  std::vector<double> array(tokens.size(), 0.0);
  for(size_t i = 0; i < tokens.size(); i++)
  {
    array[i] = std::stof(tokens[i]);
    if(argv[2][0] == 'd')
    {
      array[i] *= M_PI / 180.0;
    }
  }

  double* repPtr = array.data();

  return 0;
}
