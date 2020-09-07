#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <valarray>
#include <vector>

#include "EbsdLib/Core/Orientation.hpp"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#include "Test/TestPrintFunctions.h"

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

//--------------------------------------------------------------------------------
void eu2om(const OrientationD& input)
{
  OrientationD om = OrientationTransformation::eu2om<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_OM(om);
}
//--------------------------------------------------------------------------------
void eu2ax(const OrientationD& input)
{
  OrientationD ax = OrientationTransformation::eu2ax<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_AX(ax);
}
//--------------------------------------------------------------------------------
void eu2ro(const OrientationD& input)
{
  OrientationD ro = OrientationTransformation::eu2ro<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_RO(ro);
}
//--------------------------------------------------------------------------------
void eu2qu(const OrientationD& input)
{
  QuatD qu = OrientationTransformation::eu2qu<OrientationD, QuatD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_QU(qu);
}
//--------------------------------------------------------------------------------
void eu2ho(const OrientationD& input)
{
  OrientationD ho = OrientationTransformation::eu2ho<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_HO(ho);
}
//--------------------------------------------------------------------------------
void eu2cu(const OrientationD& input)
{
  OrientationD cu = OrientationTransformation::eu2cu<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_CU(cu);
}
//--------------------------------------------------------------------------------
void om2eu(const OrientationD& input)
{
  OrientationD eu = OrientationTransformation::om2eu<OrientationD, OrientationD>({input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]});
  OrientationPrinters::Print_EU(eu);
}
//--------------------------------------------------------------------------------
void om2ax(const OrientationD& input)
{
  OrientationD ax = OrientationTransformation::om2ax<OrientationD, OrientationD>({input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]});
  OrientationPrinters::Print_AX(ax);
}
//--------------------------------------------------------------------------------
void om2ro(const OrientationD& input)
{
  OrientationD ro = OrientationTransformation::om2ro<OrientationD, OrientationD>({input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]});
  OrientationPrinters::Print_RO(ro);
}
//--------------------------------------------------------------------------------
void om2qu(const OrientationD& input)
{
  QuatD qu = OrientationTransformation::om2qu<OrientationD, QuatD>({input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]});
  OrientationPrinters::Print_QU(qu);
}
//--------------------------------------------------------------------------------
void om2ho(const OrientationD& input)
{
  OrientationD ho = OrientationTransformation::om2ho<OrientationD, OrientationD>({input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]});
  OrientationPrinters::Print_HO(ho);
}
//--------------------------------------------------------------------------------
void om2cu(const OrientationD& input)
{
  OrientationD cu = OrientationTransformation::om2cu<OrientationD, OrientationD>({input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]});
  OrientationPrinters::Print_CU(cu);
}
//--------------------------------------------------------------------------------
void ax2eu(const OrientationD& input)
{
  OrientationD eu = OrientationTransformation::ax2eu<OrientationD, OrientationD>({input[0], input[1], input[2], input[3]});
  OrientationPrinters::Print_EU(eu);
}
//--------------------------------------------------------------------------------
void ax2om(const OrientationD& input)
{
  OrientationD om = OrientationTransformation::ax2om<OrientationD, OrientationD>({input[0], input[1], input[2], input[3]});
  OrientationPrinters::Print_OM(om);
}
//--------------------------------------------------------------------------------
void ax2ro(const OrientationD& input)
{
  OrientationD ro = OrientationTransformation::ax2ro<OrientationD, OrientationD>({input[0], input[1], input[2], input[3]});
  OrientationPrinters::Print_RO(ro);
}
//--------------------------------------------------------------------------------
void ax2qu(const OrientationD& input)
{
  QuatD qu = OrientationTransformation::ax2qu<OrientationD, QuatD>({input[0], input[1], input[2], input[3]});
  OrientationPrinters::Print_QU(qu);
}
//--------------------------------------------------------------------------------
void ax2ho(const OrientationD& input)
{
  OrientationD ho = OrientationTransformation::ax2ho<OrientationD, OrientationD>({input[0], input[1], input[2], input[3]});
  OrientationPrinters::Print_HO(ho);
}
//--------------------------------------------------------------------------------
void ax2cu(const OrientationD& input)
{
  OrientationD cu = OrientationTransformation::ax2cu<OrientationD, OrientationD>({input[0], input[1], input[2], input[3]});
  OrientationPrinters::Print_CU(cu);
}
//--------------------------------------------------------------------------------
void ro2eu(const OrientationD& input)
{
  OrientationD eu = OrientationTransformation::ro2eu<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_EU(eu);
}
//--------------------------------------------------------------------------------
void ro2om(const OrientationD& input)
{
  OrientationD om = OrientationTransformation::ro2om<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_OM(om);
}
//--------------------------------------------------------------------------------
void ro2ax(const OrientationD& input)
{
  OrientationD ax = OrientationTransformation::ro2ax<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_AX(ax);
}
//--------------------------------------------------------------------------------
void ro2qu(const OrientationD& input)
{
  QuatD qu = OrientationTransformation::ro2qu<OrientationD, QuatD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_QU(qu);
}
//--------------------------------------------------------------------------------
void ro2ho(const OrientationD& input)
{
  OrientationD ho = OrientationTransformation::ro2ho<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_HO(ho);
}
//--------------------------------------------------------------------------------
void ro2cu(const OrientationD& input)
{
  OrientationD cu = OrientationTransformation::ro2cu<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_CU(cu);
}
//--------------------------------------------------------------------------------
void qu2eu(const QuatD& input)
{
  OrientationD eu = OrientationTransformation::qu2eu<QuatD, OrientationD>({input[0], input[1], input[2], input[4]});
  OrientationPrinters::Print_EU(eu);
}
//--------------------------------------------------------------------------------
void qu2om(const QuatD& input)
{
  OrientationD om = OrientationTransformation::qu2om<QuatD, OrientationD>({input[0], input[1], input[2], input[4]});
  OrientationPrinters::Print_OM(om);
}
//--------------------------------------------------------------------------------
void qu2ax(const QuatD& input)
{
  OrientationD ax = OrientationTransformation::qu2ax<QuatD, OrientationD>({input[0], input[1], input[2], input[4]});
  OrientationPrinters::Print_AX(ax);
}
//--------------------------------------------------------------------------------
void qu2ro(const QuatD& input)
{
  OrientationD ro = OrientationTransformation::qu2ro<QuatD, OrientationD>({input[0], input[1], input[2], input[4]});
  OrientationPrinters::Print_RO(ro);
}
//--------------------------------------------------------------------------------
void qu2ho(const QuatD& input)
{
  OrientationD ho = OrientationTransformation::qu2ho<QuatD, OrientationD>({input[0], input[1], input[2], input[4]});
  OrientationPrinters::Print_HO(ho);
}
//--------------------------------------------------------------------------------
void qu2cu(const QuatD& input)
{
  OrientationD cu = OrientationTransformation::qu2cu<QuatD, OrientationD>({input[0], input[1], input[2], input[4]});
  OrientationPrinters::Print_CU(cu);
}
//--------------------------------------------------------------------------------
void ho2eu(const OrientationD& input)
{
  OrientationD eu = OrientationTransformation::ho2eu<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_EU(eu);
}
//--------------------------------------------------------------------------------
void ho2om(const OrientationD& input)
{
  OrientationD om = OrientationTransformation::ho2om<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_OM(om);
}
//--------------------------------------------------------------------------------
void ho2ax(const OrientationD& input)
{
  OrientationD ax = OrientationTransformation::ho2ax<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_AX(ax);
}
//--------------------------------------------------------------------------------
void ho2ro(const OrientationD& input)
{
  OrientationD ro = OrientationTransformation::ho2ro<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_RO(ro);
}
//--------------------------------------------------------------------------------
void ho2qu(const OrientationD& input)
{
  QuatD qu = OrientationTransformation::ho2qu<OrientationD, QuatD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_QU(qu);
}
//--------------------------------------------------------------------------------
void ho2cu(const OrientationD& input)
{
  OrientationD cu = OrientationTransformation::ho2cu<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_CU(cu);
}
//--------------------------------------------------------------------------------
void cu2eu(const OrientationD& input)
{
  OrientationD eu = OrientationTransformation::cu2eu<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_EU(eu);
}
//--------------------------------------------------------------------------------
void cu2om(const OrientationD& input)
{
  OrientationD om = OrientationTransformation::cu2om<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_OM(om);
}
//--------------------------------------------------------------------------------
void cu2ax(const OrientationD& input)
{
  OrientationD ax = OrientationTransformation::cu2ax<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_AX(ax);
}
//--------------------------------------------------------------------------------
void cu2ro(const OrientationD& input)
{
  OrientationD ro = OrientationTransformation::cu2ro<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_RO(ro);
}
//--------------------------------------------------------------------------------
void cu2qu(const OrientationD& input)
{
  QuatD qu = OrientationTransformation::cu2qu<OrientationD, QuatD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_QU(qu);
}
//--------------------------------------------------------------------------------
void cu2ho(const OrientationD& input)
{
  OrientationD ho = OrientationTransformation::cu2ho<OrientationD, OrientationD>({input[0], input[1], input[2]});
  OrientationPrinters::Print_HO(ho);
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

  if(conversion == "eu2om")
  {
    OrientationD eu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::eu_check(eu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    eu2om(eu);
  }
  else if(conversion == "eu2ax")
  {
    OrientationD eu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::eu_check(eu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    eu2ax(eu);
  }
  else if(conversion == "eu2ro")
  {
    OrientationD eu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::eu_check(eu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    eu2ro(eu);
  }
  else if(conversion == "eu2qu")
  {
    OrientationD eu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::eu_check(eu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    eu2qu(eu);
  }
  else if(conversion == "eu2ho")
  {
    OrientationD eu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::eu_check(eu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    eu2ho(eu);
  }
  else if(conversion == "eu2cu")
  {
    OrientationD eu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::eu_check(eu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    eu2cu(eu);
  }
  else if(conversion == "om2eu")
  {
    OrientationD om(array.data(), 9);
    OrientationTransformation::ResultType res = OrientationTransformation::om_check(om);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    om2eu(om);
  }
  else if(conversion == "om2ax")
  {
    OrientationD om(array.data(), 9);
    OrientationTransformation::ResultType res = OrientationTransformation::om_check(om);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    om2ax(om);
  }
  else if(conversion == "om2ro")
  {
    OrientationD om(array.data(), 9);
    OrientationTransformation::ResultType res = OrientationTransformation::om_check(om);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    om2ro(om);
  }
  else if(conversion == "om2qu")
  {
    OrientationD om(array.data(), 9);
    OrientationTransformation::ResultType res = OrientationTransformation::om_check(om);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    om2qu(om);
  }
  else if(conversion == "om2ho")
  {
    OrientationD om(array.data(), 9);
    OrientationTransformation::ResultType res = OrientationTransformation::om_check(om);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    om2ho(om);
  }
  else if(conversion == "om2cu")
  {
    OrientationD om(array.data(), 9);
    OrientationTransformation::ResultType res = OrientationTransformation::om_check(om);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    om2cu(om);
  }
  else if(conversion == "ax2eu")
  {
    OrientationD ax(array.data(), 4);
    OrientationTransformation::ResultType res = OrientationTransformation::ax_check(ax);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ax2eu(ax);
  }
  else if(conversion == "ax2om")
  {
    OrientationD ax(array.data(), 4);
    OrientationTransformation::ResultType res = OrientationTransformation::ax_check(ax);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ax2om(ax);
  }
  else if(conversion == "ax2ro")
  {
    OrientationD ax(array.data(), 4);
    OrientationTransformation::ResultType res = OrientationTransformation::ax_check(ax);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ax2ro(ax);
  }
  else if(conversion == "ax2qu")
  {
    OrientationD ax(array.data(), 4);
    OrientationTransformation::ResultType res = OrientationTransformation::ax_check(ax);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ax2qu(ax);
  }
  else if(conversion == "ax2ho")
  {
    OrientationD ax(array.data(), 4);
    OrientationTransformation::ResultType res = OrientationTransformation::ax_check(ax);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ax2ho(ax);
  }
  else if(conversion == "ax2cu")
  {
    OrientationD ax(array.data(), 4);
    OrientationTransformation::ResultType res = OrientationTransformation::ax_check(ax);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ax2cu(ax);
  }
  else if(conversion == "ro2eu")
  {
    OrientationD ro(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ro_check(ro);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ro2eu(ro);
  }
  else if(conversion == "ro2om")
  {
    OrientationD ro(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ro_check(ro);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ro2om(ro);
  }
  else if(conversion == "ro2ax")
  {
    OrientationD ro(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ro_check(ro);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ro2ax(ro);
  }
  else if(conversion == "ro2qu")
  {
    OrientationD ro(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ro_check(ro);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ro2qu(ro);
  }
  else if(conversion == "ro2ho")
  {
    OrientationD ro(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ro_check(ro);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ro2ho(ro);
  }
  else if(conversion == "ro2cu")
  {
    OrientationD ro(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ro_check(ro);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ro2cu(ro);
  }
  else if(conversion == "qu2eu")
  {

    QuatD qu(repPtr[0], repPtr[2], repPtr[3], repPtr[4]);
    OrientationTransformation::ResultType res = OrientationTransformation::qu_check(qu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    qu2eu(qu);
  }
  else if(conversion == "qu2om")
  {
    QuatD qu(repPtr[0], repPtr[2], repPtr[3], repPtr[4]);
    OrientationTransformation::ResultType res = OrientationTransformation::qu_check(qu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    qu2om(qu);
  }
  else if(conversion == "qu2ax")
  {
    QuatD qu(repPtr[0], repPtr[2], repPtr[3], repPtr[4]);
    OrientationTransformation::ResultType res = OrientationTransformation::qu_check(qu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    qu2ax(qu);
  }
  else if(conversion == "qu2ro")
  {
    QuatD qu(repPtr[0], repPtr[2], repPtr[3], repPtr[4]);
    OrientationTransformation::ResultType res = OrientationTransformation::qu_check(qu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    qu2ro(qu);
  }
  else if(conversion == "qu2ho")
  {
    QuatD qu(repPtr[0], repPtr[2], repPtr[3], repPtr[4]);
    OrientationTransformation::ResultType res = OrientationTransformation::qu_check(qu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    qu2ho(qu);
  }
  else if(conversion == "qu2cu")
  {
    QuatD qu(repPtr[0], repPtr[2], repPtr[3], repPtr[4]);
    OrientationTransformation::ResultType res = OrientationTransformation::qu_check(qu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    qu2cu(qu);
  }
  else if(conversion == "ho2eu")
  {
    OrientationD ho(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ho_check(ho);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ho2eu(ho);
  }
  else if(conversion == "ho2om")
  {
    OrientationD ho(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ho_check(ho);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ho2om(ho);
  }
  else if(conversion == "ho2ax")
  {
    OrientationD ho(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ho_check(ho);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ho2ax(ho);
  }
  else if(conversion == "ho2ro")
  {
    OrientationD ho(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ho_check(ho);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ho2ro(ho);
  }
  else if(conversion == "ho2qu")
  {
    OrientationD ho(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ho_check(ho);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ho2qu(ho);
  }
  else if(conversion == "ho2cu")
  {
    OrientationD ho(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::ho_check(ho);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    ho2cu(ho);
  }
  else if(conversion == "cu2eu")
  {
    OrientationD cu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::cu_check(cu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    cu2eu(cu);
  }
  else if(conversion == "cu2om")
  {
    OrientationD cu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::cu_check(cu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    cu2om(cu);
  }
  else if(conversion == "cu2ax")
  {
    OrientationD cu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::cu_check(cu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    cu2ax(cu);
  }
  else if(conversion == "cu2ro")
  {
    OrientationD cu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::cu_check(cu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    cu2ro(cu);
  }
  else if(conversion == "cu2qu")
  {
    OrientationD cu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::cu_check(cu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    cu2qu(cu);
  }
  else if(conversion == "cu2ho")
  {
    OrientationD cu(array.data(), 3);
    OrientationTransformation::ResultType res = OrientationTransformation::cu_check(cu);
    if(res.result < 0)
    {
      std::cout << res.msg << std::endl;
      return 1;
    }
    cu2ho(cu);
  }

  return 0;
}
