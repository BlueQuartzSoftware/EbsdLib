
#include "EbsdLib/Core/EbsdLibConstants.h"

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Math/Matrix3X3.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <iomanip>

void Print3x3(EbsdLib::Matrix3X3D& m)
{
    std::cout << std::fixed;
    std::cout << std::setprecision(1);
    size_t i=0;
    std::cout << "    {" << m[i++] << ", " <<  m[i++] << ", " <<  m[i++] << "},\n";
    std::cout << "    {" << m[i++] << ", " <<  m[i++] << ", " <<  m[i++] << "},\n";
    std::cout << "    {" << m[i++] << ", " <<  m[i++] << ", " <<  m[i++] << "}\n";
}

void PrintQuat(QuatD& q)
{
    std::cout << std::fixed;
    std::cout << std::setprecision(8);
    size_t i=0;
    std::cout << "    QuatD(" << q[i++] << ", " <<  q[i++] << ", " <<  q[i++]<< ", " <<  q[i++] << ")\n";
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    std::cout << "Starting SymOP GenCode" << std::endl;

    auto allLaueOps = LaueOps::GetAllOrientationOps();

    for(const auto& op : allLaueOps)
    {   
        std::cout << op->getSymmetryName() << "  " << op->getNameOfClass() << std::endl;
        int numSymOps = op->getNumSymOps();
        std::cout << "// clang-format off\n";
        std::cout << "static const double MatSym[k_SymOpsCount][3][3] = {\n";
        for(int opNum = 0; opNum < numSymOps; opNum++)
        {
            EbsdLib::Matrix3X3D matSymOp = op->getMatSymOpD(opNum);
            Print3x3(matSymOp);
            if(opNum < numSymOps-1 )
            {
                std::cout << "\n";
            }
        }
        std::cout << "};\n";
        std::cout << "// clang-format on\n";

        /* Generate the QuatSym Constants*/
        std::cout << std::setprecision(8);
        std::cout << "// clang-format off\n";
        std::cout << "static const std::vector<QuatD> QuatSym ={\n";
        for(int opNum = 0; opNum < numSymOps; opNum++)
        {
            EbsdLib::Matrix3X3D matSymOp = op->getMatSymOpD(opNum);
            OrientationD od(matSymOp.data(), 9);
            auto quat = OrientationTransformation::om2qu<OrientationD, QuatD>(od);

            PrintQuat(quat);
            if(opNum < numSymOps-1 )
            {
                std::cout << "\n";
            }
        }
        std::cout << "};\n";
        std::cout << "// clang-format on\n";        
    }

    return 0;
}