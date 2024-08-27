
#include "EbsdLib/Core/EbsdLibConstants.h"

#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Math/Matrix3X3.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

const double sq22 = 0.7071067811865475244; // sqrt(2)/2
static double sq32 = std::sqrt(3.0) / 2.0;

const double half = 0.50; // 1/2

std::vector<QuatD> SYM_Qsymop = {
    QuatD(0.0, 0.0, 0.0, 0.0),        // 0: JUNK.
    QuatD(1.0, 0.0, 0.0, 0.0),        //  1: identity operator
    QuatD(0.0, 1.0, 0.0, 0.0),        //  2: 180@[100]
    QuatD(0.0, 0.0, 1.0, 0.0),        //  3: 180@[010]
    QuatD(0.0, 0.0, 0.0, 1.0),        //  4: 180@[001]
    QuatD(sq22, sq22, 0.0, 0.0),      //  5: 90@[100]
    QuatD(sq22, 0.0, sq22, 0.0),      //  6: 90@[010]
    QuatD(sq22, 0.0, 0.0, sq22),      //  7: 90@[001]
    QuatD(sq22, -sq22, 0.0, 0.0),     //  8: 270@[100]
    QuatD(sq22, 0.0, -sq22, 0.0),     //  9: 270@[010]
    QuatD(sq22, 0.0, 0.0, -sq22),     // 10: 270@[001]
    QuatD(0.0, sq22, sq22, 0.0),      // 11: 180@[110]
    QuatD(0.0, -sq22, sq22, 0.0),     // 12: 180@[-110]
    QuatD(0.0, 0.0, sq22, sq22),      // 13: 180@[011]
    QuatD(0.0, 0.0, -sq22, sq22),     // 14: 180@[0-11]
    QuatD(0.0, sq22, 0.0, sq22),      // 15: 180@[101]
    QuatD(0.0, -sq22, 0.0, sq22),     // 16: 180@[-101]
    QuatD(half, half, half, half),    // 17: 120@[111]
    QuatD(half, -half, -half, -half), // 18: 120@[-1-1-1]
    QuatD(half, half, -half, half),   // 19: 120@[1-11]
    QuatD(half, -half, half, -half),  // 20: 120@[-11-1]
    QuatD(half, -half, half, half),   // 21: 120@[-111]
    QuatD(half, half, -half, -half),  // 22: 120@[1-1-1]
    QuatD(half, -half, -half, half),  // 23: 120@[-1-11]
    QuatD(half, half, half, -half),   // 24: 120@[11-1]
    QuatD(sq32, 0.0, 0.0, half),      // 25:  60@[001]   (hexagonal/trigonal operators start here)
    QuatD(half, 0.0, 0.0, sq32),      // 26: 120@[001]
    QuatD(0.0, 0.0, 0.0, 1.0),        // 27: 180@[001]  (duplicate from above, but useful to keep it here)
    QuatD(-half, 0.0, 0.0, sq32),     // 28: 240@[001]
    QuatD(-sq32, 0.0, 0.0, half),     // 29: 300@[001]
    QuatD(0.0, 1.0, 0.0, 0.0),        // 30: 180@[100]
    QuatD(0.0, sq32, half, 0.0),      // 31: 180@[xxx]
    QuatD(0.0, half, sq32, 0.0),      // 32: 180@[xxx]
    QuatD(0.0, 0.0, 1.0, 0.0),        // 33: 180@[010]
    QuatD(0.0, -half, sq32, 0.0),     // 34: 180@[xxx]
    QuatD(0.0, -sq32, half, 0.0),     // 35: 180@[xxx]
};

std::vector<QuatD> InitRotationPointGroup(int rotPointGroup)
{
  std::vector<QuatD> sym_ops;

  // identity operator is part of all point groups
  // sym_ops.at = 0.D0                  // initialize all entries to zero
  sym_ops.push_back(SYM_Qsymop[1]);
  size_t i = 0;
  // select statement for each individual rotational point group (see typedefs.f90 for SYM_Qsymop definitions)
  switch(rotPointGroup)
  {
  case(1): // 1 (no additional symmetry elements)
    sym_ops.resize(1);
    // sym_ops.at(2-1) = -sym_ops.at(1)
    break;
  case(2): // 2  (we'll assume that the two-fold axis lies along the e_y-axis)
    sym_ops.resize(2);
    sym_ops.at(2 - 1) = SYM_Qsymop[3];
    break;
  case(222): // 222
    sym_ops.resize(4);
    for(i = 2; i <= 4; i++)
      sym_ops.at(i - 1) = SYM_Qsymop[i];

    break;
  case(4): // 4
    sym_ops.resize(4);
    sym_ops.at(2 - 1) = SYM_Qsymop[4];
    sym_ops.at(3 - 1) = SYM_Qsymop[7];
    sym_ops.at(4 - 1) = SYM_Qsymop[10];
    break;
  case(422): // 422
    sym_ops.resize(8);
    sym_ops.at(2 - 1) = SYM_Qsymop[4];
    sym_ops.at(3 - 1) = SYM_Qsymop[7];
    sym_ops.at(4 - 1) = SYM_Qsymop[10];
    sym_ops.at(5 - 1) = SYM_Qsymop[2];
    sym_ops.at(6 - 1) = SYM_Qsymop[3];
    sym_ops.at(7 - 1) = SYM_Qsymop[11];
    sym_ops.at(8 - 1) = SYM_Qsymop[12];
    break;
  case(3): // 3
    sym_ops.resize(3);
    sym_ops.at(2 - 1) = SYM_Qsymop[26];
    sym_ops.at(3 - 1) = SYM_Qsymop[28];
    // call FatalError('InitDictionaryIndexing','this symmetry has not yet been implemented (pg 3)')
    break;
  case(32): // 32 (needs special handling)
    sym_ops.resize(6);
    sym_ops.at(2 - 1) = SYM_Qsymop[26];
    sym_ops.at(3 - 1) = SYM_Qsymop[28];
    sym_ops.at(4 - 1) = SYM_Qsymop[30];
    sym_ops.at(5 - 1) = SYM_Qsymop[32];
    sym_ops.at(6 - 1) = SYM_Qsymop[34];
    // call FatalError('InitDictionaryIndexing','this symmetry has not yet been implemented (pg 32)')
    break;
  case(6): // 6
    sym_ops.resize(6);
    for(i = 25; i <= 29; i++) // i=25,29
      sym_ops.at(i - 23 - 1) = SYM_Qsymop[i];

    break;
  case(622): // 622
    sym_ops.resize(12);
    for(i = 25; i <= 35; i++) // do i=25,35
      sym_ops.at(i - 23 - 1) = SYM_Qsymop[i];

    break;
  case(23): // 23
    sym_ops.resize(12);
    for(i = 2; i <= 4; i++) // do i=2,4
      sym_ops.at(i - 1) = SYM_Qsymop[i];

    for(i = 17; i <= 24; i++) // do i=17,24
      sym_ops.at(4 + (i - 16) - 1) = SYM_Qsymop[i];

    break;
  case(432): // 432
    sym_ops.resize(24);
    for(i = 2; i <= 24; i++) // do i=2,24
      sym_ops.at(i - 1) = SYM_Qsymop[i];

    break;

  default: // this should never happen ...
    throw std::runtime_error("Rotation Point Group is not recognized");
  }

  return sym_ops;
}

void Print3x3(const OrientationD& m)
{
  std::cout << std::fixed;
  std::cout << std::setprecision(16);
  size_t i = 0;
  std::cout << "    {{" << m[i++] << ", " << m[i++] << ", " << m[i++] << "},\n";
  std::cout << "    {" << m[i++] << ", " << m[i++] << ", " << m[i++] << "},\n";
  std::cout << "    {" << m[i++] << ", " << m[i++] << ", " << m[i++] << "}},\n";
}

void PrintQuat(const QuatD& q, bool sv)
{
  std::cout << std::fixed;
  std::cout << std::setprecision(16);
  size_t i = 0;
  if(sv)
  {
    std::cout << "    QuatD(" << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[0] << ")";
  }
  {
    std::cout << "    QuatD(" << q[i++] << ", " << q[i++] << ", " << q[i++] << ", " << q[i++] << ")";
  }
}

void PrintRod(const OrientationD& q)
{
  std::cout << std::fixed;
  std::cout << std::setprecision(16);

  std::cout << "    {" << (std::isinf(q[0]) ? 10.0E12 : q[0]) << ", " << (std::isinf(q[1]) ? 10.0E12 : q[1]) << ", " << (std::isinf(q[2]) ? 10.0E12 : q[2]) << ", "
            << (std::isinf(q[3]) ? 10.0E12 : q[3]) << "}";
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  std::cout << "Starting SymOP GenCode" << std::endl;

  std::vector<int> rotPointGroup = {1, 2, 222, 4, 422, 3, 32, 6, 622, 23, 432};
  std::vector<std::string> names = {"TriclinicOps", "MonoclinicOps",   "OrthorhombicOps", "TetragonalLowOps", "TetragonalOps", "TrigonalLowOps",
                                    "TrigonalOps",  "HexagonalLowOps", "HexagonalOps",    "CubicLowOps",      "CubicOps"};

  for(size_t i = 0; i < rotPointGroup.size(); i++)
  {
    std::cout << "###########################################################################\n";
    std::cout << names[i] << "\n// Rotation Point Group: " << rotPointGroup[i] << std::endl;
    auto symOPs = InitRotationPointGroup(rotPointGroup[i]);

    for(auto& quat : symOPs)
    {
      quat = QuatD(quat[1], quat[2], quat[3], quat[0]);
    }

    std::cout << std::setprecision(8);
    std::cout << "// clang-format off\n";
    std::cout << "static const std::vector<QuatD> QuatSym ={\n";
    for(const auto& symOp : symOPs)
    {
      PrintQuat(symOp, false);
      std::cout << ",\n";
    }
    std::cout << "};\n\n";

    std::cout << "static const std::vector<OrientationD> RodSym = {\n";
    for(const auto& symOp : symOPs)
    {
      auto ro = OrientationTransformation::qu2ro<QuatD, OrientationD>(symOp);
      PrintRod(ro);
      std::cout << ",\n";
    }
    std::cout << "};\n\n";

    std::cout << "static const double MatSym[k_SymOpsCount][3][3] = {\n";
    for(const auto& symOp : symOPs)
    {
      auto om = OrientationTransformation::qu2om<QuatD, OrientationD>(symOp);
      Print3x3(om);
      std::cout << "    \n";
    }
    std::cout << "};\n";
    std::cout << "// clang-format on\n\n";
  }

  return -1;

  return 0;
}
