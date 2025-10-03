#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#include "Test/TestPrintFunctions.h"

bool quaternionsAreClose(const QuatD& q1, const QuatD& q2, double tolerance = 1e-6)
{
  return fabs(q1.w() - q2.w()) < tolerance && fabs(q1.x() - q2.x()) < tolerance && fabs(q1.y() - q2.y()) < tolerance && fabs(q1.z() - q2.z()) < tolerance;
}

int v1(QuatD q1, QuatD q2)
{
  q1 = q1.normalize();
  q2 = q2.normalize();
  const QuatD q2Negate = -q2;

  LaueOps::Pointer cubicOps = LaueOps::GetAllOrientationOps()[1];

  int numSymOps = cubicOps->getNumSymOps();
  for(int s1op = 0; s1op < numSymOps; ++s1op)
  {
    const QuatD s1 = cubicOps->getQuatSymOp(s1op);

    for(int s2op = 0; s2op < numSymOps; ++s2op)
    {
      const QuatD s2 = cubicOps->getQuatSymOp(s2op);
      const QuatD s2Inverse = s2.inverse();
      QuatD qPrime = s1 * q1 * s2Inverse;
      if(quaternionsAreClose(qPrime, q2) || quaternionsAreClose(qPrime, q2Negate))
      {
        std::cout << "V1: Quats are same" << std::endl;
        // std::cout << "q1: ";
        // OrientationPrinters::Print_QU(q1);
        // std::cout << "q2: ";
        // OrientationPrinters::Print_QU(q2);
        // std::cout << "s1: ";
        // OrientationPrinters::Print_QU(s1);
        // std::cout << "s2: ";
        // OrientationPrinters::Print_QU(s2);
        return 1;
      }
    }
  }
  std::cout << "V1: Quats are not the same\n";
  return 0;
}

int v2(QuatD q1, QuatD q2)
{
  double tolerance = 1e-6;
  q1 = q1.normalize();
  q2 = q2.normalize();

  LaueOps::Pointer cubicOps = LaueOps::GetAllOrientationOps()[1];
  double minDiff = 1.0;
  int numSymOps = cubicOps->getNumSymOps();
  for(int s1op = 0; s1op < numSymOps; ++s1op)
  {
    const QuatD s1 = cubicOps->getQuatSymOp(s1op);

    for(int s2op = 0; s2op < numSymOps; ++s2op)
    {
      const QuatD s2Con = cubicOps->getQuatSymOp(s2op).conjugate();
      QuatD qPrime = s1 * q1 * s2Con;

      double dot = qPrime.dotProduct(q2);
      double diff = std::abs(1.0 - dot);
      minDiff = std::min(minDiff, diff);
      if(diff < tolerance)
      {
        std::cout << "V2: Quats are same. minDiff = " << minDiff << "\n";
        // std::cout << "q1: ";
        // OrientationPrinters::Print_QU(q1);
        // std::cout << "q2: ";
        // OrientationPrinters::Print_QU(q2);
        // std::cout << "s1: ";
        // OrientationPrinters::Print_QU(s1);
        // std::cout << "s2: ";
        // OrientationPrinters::Print_QU(cubicOps->getQuatSymOp(s2op));
        return 1;
      }
    }
  }
  std::cout << "V2: Quats are not the same. minDiff = " << minDiff << "\n";
  return 0;
}

std::vector<double> ReadDoublesFromCSV(const std::string& filePath)
{
  std::vector<double> values;
  std::ifstream file(filePath);

  if(!file.is_open())
  {
    throw std::runtime_error("Unable to open file: " + filePath);
  }

  std::string line;
  while(std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string token;
    while(std::getline(ss, token, ','))
    {
      try
      {
        double value = std::stod(token);
        values.push_back(value);
      } catch(const std::invalid_argument& e)
      {
        std::cerr << "Warning: Skipping invalid token '" << token << "'\n";
      }
    }
  }

  return values;
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  std::vector<double> q = ReadDoublesFromCSV(argv[1]);
  for(int i = 0; i < q.size(); i = i + 8)
  {
    std::cout << (i / 8) << ": ";
    v2({q[i], q[i + 1], q[i + 2], q[i + 3]}, {q[i + 4], q[i + 5], q[i + 6], q[i + 7]});
  }
}
