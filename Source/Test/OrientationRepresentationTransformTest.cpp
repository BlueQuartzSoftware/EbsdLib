
#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/OrientationRepresentation.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "GenerateFunctionList.h"
#include "TestPrintFunctions.h"
#include "UnitTestSupport.hpp"

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace EbsdLib;

class OrientationRepresentationTransformTest
{
public:
  OrientationRepresentationTransformTest() = default;
  virtual ~OrientationRepresentationTransformTest() = default;

  OrientationRepresentationTransformTest(const OrientationRepresentationTransformTest&) = delete;            // Copy Constructor Not Implemented
  OrientationRepresentationTransformTest(OrientationRepresentationTransformTest&&) = delete;                 // Move Constructor Not Implemented
  OrientationRepresentationTransformTest& operator=(const OrientationRepresentationTransformTest&) = delete; // Copy Assignment Not Implemented
  OrientationRepresentationTransformTest& operator=(OrientationRepresentationTransformTest&&) = delete;      // Move Assignment Not Implemented

  EBSD_GET_NAME_OF_CLASS_DECL(OrientationRepresentationTransformTest)

  void Test1()
  {

    {
      EulerDType euler(0.0, 0.0, 0.0);
      QuatD quat = euler.toQuat();
      EulerDType e1 = euler.toOrientationMatrix().toEuler();
    }

    {
      QuatD quat = EulerDType(0.0f, 0.0f, 0.0f).toOrientationMatrix().toQuat();
      OrientationPrinters::Print_QU(quat);
    }

    {
      AxisAngleDType ax = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle();
      OrientationMatrixDType om = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle().toOrientationMatrix();
      RodriguesDType ro = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle().toOrientationMatrix().toRodrigues();
      HomochoricDType ho = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle().toOrientationMatrix().toRodrigues().toHomochoric();
      CubochoricDType cu = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle().toOrientationMatrix().toRodrigues().toHomochoric().toCubochoric();

      QuatD quat = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle().toQuat();
    }

    AxisAngleDType ax = EulerDType(0.0f, 1.570f, 0.0f).toAxisAngle();
    OrientationPrinters::Print_AX(ax);
  }

  std::string k_MethodNames[s_NumReps] = {"Euler", "OrientationMatrix", "Quaternion", "AxisAngle", "Rodrigues", "Homochoric", "Cubochoric", "Stereographic"};
  int k_CompDims[s_NumReps] = {3, 9, 4, 4, 4, 3, 3, 3};

  template <typename T>
  void printVector(const std::vector<T>& vec)
  {
    std::cout << "   GENERATE_TEST_METHOD(";
    for(size_t i = 0; i < vec.size(); i++)
    {
      // std::cout << vec[i];
      std::cout << k_MethodNames[vec[i]] << "";
      if(i < vec.size() - 1)
      {
        std::cout << ", ";
      }
    }
    std::cout << ");";
  }

  void StartTest()
  {
    //  std::vector<std::string> functionNames = OrientationConverter<float>::GetOrientationTypeStrings();

    GenerateFunctionList generator;
    std::vector<GenerateFunctionList::EntryType> entries = generator.GeneratePermutationsOfCombinations(s_NumReps, 3);
    // This outer loop will group the tests based on the first orientation representation
    for(int t = 0; t < s_NumReps; t++)
    {
      std::cout << "void " << k_MethodNames[t] << "Test() {\n";
      std::cout << "  TEST_PREAMBLE();\n\n";
      for(auto& entry : entries)
      {
        if(entry[0] != t)
        {
          continue;
        }
        entry.push_back(entry[0]);
        {
          // RunTestCase<double>(entry, 16);
          printVector(entry);
          std::cout << "\n";
        }
      }
      std::cout << "}\n\n";
    }
  }

#define TEST_PREAMBLE()                                                                                                                                                                                \
  using EbsdDataArrayType = EbsdDataArray<double>;                                                                                                                                                     \
  using EbsdDataArrayPointerType = EbsdDataArrayType::Pointer;                                                                                                                                         \
  size_t nSteps = 16;                                                                                                                                                                                  \
  std::map<std::string, EbsdDataArrayPointerType> attrMat;                                                                                                                                             \
  generate_test_data::GenerateEulers<double>(nSteps, attrMat);                                                                                                                                         \
  EbsdDataArrayType& inputArray = *(attrMat["eu"]);                                                                                                                                                    \
  size_t numTuples = inputArray.getNumberOfTuples();                                                                                                                                                   \
  double tolerance = 1.0E-3;                                                                                                                                                                           \
  std::string k_EulerStr("Euler");

#define GENERATE_TEST_METHOD_EU(TYPE1, TYPE2, TYPE3, TYPE4)                                                                                                                                            \
  {                                                                                                                                                                                                    \
    std::cout << "Starting " << #TYPE1 << " -> " << #TYPE2 << " -> " << #TYPE3 << " -> " << #TYPE4 << "\n";                                                                                            \
    for(size_t tupleIdx = 0; tupleIdx < numTuples; tupleIdx++)                                                                                                                                         \
    {                                                                                                                                                                                                  \
      const EulerDType euler = EulerDType(inputArray[tupleIdx * 3], inputArray[tupleIdx * 3 + 1], inputArray[tupleIdx * 3 + 2]).toOrientationMatrix().toEuler();                                       \
      const TYPE1##DType type1 = euler.to##TYPE1();                                                                                                                                                    \
      const TYPE2##DType type2 = type1.to##TYPE2();                                                                                                                                                    \
      const TYPE3##DType type3 = type2.to##TYPE3();                                                                                                                                                    \
      TYPE4##DType type4 = type3.to##TYPE4().toOrientationMatrix().toEuler();                                                                                                                          \
      bool withinTolerance = type4.isWithinTolerance(type1, tolerance);                                                                                                                                \
      if(!withinTolerance)                                                                                                                                                                             \
      {                                                                                                                                                                                                \
        std::cout << "Index: " << tupleIdx << "\n";                                                                                                                                                    \
        std::cout << type1 << "\n";                                                                                                                                                                    \
        std::cout << type4 << "\n";                                                                                                                                                                    \
      }                                                                                                                                                                                                \
      DREAM3D_REQUIRED(withinTolerance, ==, true)                                                                                                                                                      \
    }                                                                                                                                                                                                  \
  }

#define GENERATE_TEST_METHOD(TYPE1, TYPE2, TYPE3, TYPE4)                                                                                                                                               \
  {                                                                                                                                                                                                    \
    std::cout << "Starting " << #TYPE1 << " -> " << #TYPE2 << " -> " << #TYPE3 << " -> " << #TYPE4 << "\n";                                                                                            \
    for(size_t tupleIdx = 0; tupleIdx < numTuples; tupleIdx++)                                                                                                                                         \
    {                                                                                                                                                                                                  \
      const EulerDType euler = EulerDType(inputArray[tupleIdx * 3], inputArray[tupleIdx * 3 + 1], inputArray[tupleIdx * 3 + 2]).toOrientationMatrix().toEuler();                                       \
      const TYPE1##DType type1 = euler.to##TYPE1();                                                                                                                                                    \
      const TYPE2##DType type2 = type1.to##TYPE2();                                                                                                                                                    \
      const TYPE3##DType type3 = type2.to##TYPE3();                                                                                                                                                    \
      TYPE4##DType type4 = type3.to##TYPE4();                                                                                                                                                          \
      bool withinTolerance = type4.isWithinTolerance(type1, tolerance);                                                                                                                                \
      if(!withinTolerance)                                                                                                                                                                             \
      {                                                                                                                                                                                                \
        std::cout << "Index: " << tupleIdx << "\n";                                                                                                                                                    \
        std::cout << type1 << "\n";                                                                                                                                                                    \
        std::cout << type4 << "\n";                                                                                                                                                                    \
      }                                                                                                                                                                                                \
      DREAM3D_REQUIRED(withinTolerance, ==, true)                                                                                                                                                      \
    }                                                                                                                                                                                                  \
  }

  void EulerTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD_EU(Euler, OrientationMatrix, Quaternion, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Quaternion, OrientationMatrix, Euler);
    GENERATE_TEST_METHOD_EU(Euler, OrientationMatrix, AxisAngle, Euler);
    GENERATE_TEST_METHOD_EU(Euler, AxisAngle, OrientationMatrix, Euler);
    GENERATE_TEST_METHOD_EU(Euler, OrientationMatrix, Rodrigues, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Rodrigues, OrientationMatrix, Euler);
    GENERATE_TEST_METHOD_EU(Euler, OrientationMatrix, Homochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Homochoric, OrientationMatrix, Euler);
    GENERATE_TEST_METHOD_EU(Euler, OrientationMatrix, Cubochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Cubochoric, OrientationMatrix, Euler);
    GENERATE_TEST_METHOD_EU(Euler, OrientationMatrix, Stereographic, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Stereographic, OrientationMatrix, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Quaternion, AxisAngle, Euler);
    GENERATE_TEST_METHOD_EU(Euler, AxisAngle, Quaternion, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Quaternion, Rodrigues, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Rodrigues, Quaternion, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Quaternion, Homochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Homochoric, Quaternion, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Quaternion, Cubochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Cubochoric, Quaternion, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Quaternion, Stereographic, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Stereographic, Quaternion, Euler);
    GENERATE_TEST_METHOD_EU(Euler, AxisAngle, Rodrigues, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Rodrigues, AxisAngle, Euler);
    GENERATE_TEST_METHOD_EU(Euler, AxisAngle, Homochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Homochoric, AxisAngle, Euler);
    GENERATE_TEST_METHOD_EU(Euler, AxisAngle, Cubochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Cubochoric, AxisAngle, Euler);
    GENERATE_TEST_METHOD_EU(Euler, AxisAngle, Stereographic, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Stereographic, AxisAngle, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Rodrigues, Homochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Homochoric, Rodrigues, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Rodrigues, Cubochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Cubochoric, Rodrigues, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Rodrigues, Stereographic, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Stereographic, Rodrigues, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Homochoric, Cubochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Cubochoric, Homochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Homochoric, Stereographic, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Stereographic, Homochoric, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Cubochoric, Stereographic, Euler);
    GENERATE_TEST_METHOD_EU(Euler, Stereographic, Cubochoric, Euler);
  }

  void OrientationMatrixTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(OrientationMatrix, Euler, Quaternion, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Quaternion, Euler, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Euler, AxisAngle, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, AxisAngle, Euler, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Euler, Rodrigues, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Rodrigues, Euler, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Euler, Homochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Homochoric, Euler, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Euler, Cubochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Cubochoric, Euler, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Euler, Stereographic, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Stereographic, Euler, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Quaternion, AxisAngle, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, AxisAngle, Quaternion, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Quaternion, Rodrigues, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Rodrigues, Quaternion, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Quaternion, Homochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Homochoric, Quaternion, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Quaternion, Cubochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Cubochoric, Quaternion, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Quaternion, Stereographic, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Stereographic, Quaternion, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, AxisAngle, Rodrigues, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Rodrigues, AxisAngle, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, AxisAngle, Homochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Homochoric, AxisAngle, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, AxisAngle, Cubochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Cubochoric, AxisAngle, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, AxisAngle, Stereographic, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Stereographic, AxisAngle, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Rodrigues, Homochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Homochoric, Rodrigues, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Rodrigues, Cubochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Cubochoric, Rodrigues, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Rodrigues, Stereographic, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Stereographic, Rodrigues, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Homochoric, Cubochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Cubochoric, Homochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Homochoric, Stereographic, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Stereographic, Homochoric, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Cubochoric, Stereographic, OrientationMatrix);
    GENERATE_TEST_METHOD(OrientationMatrix, Stereographic, Cubochoric, OrientationMatrix);
  }

  void QuaternionTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(Quaternion, Euler, OrientationMatrix, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, OrientationMatrix, Euler, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Euler, AxisAngle, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, AxisAngle, Euler, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Euler, Rodrigues, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Rodrigues, Euler, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Euler, Homochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Homochoric, Euler, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Euler, Cubochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Cubochoric, Euler, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Euler, Stereographic, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Stereographic, Euler, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, OrientationMatrix, AxisAngle, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, AxisAngle, OrientationMatrix, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, OrientationMatrix, Rodrigues, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Rodrigues, OrientationMatrix, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, OrientationMatrix, Homochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Homochoric, OrientationMatrix, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, OrientationMatrix, Cubochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Cubochoric, OrientationMatrix, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, OrientationMatrix, Stereographic, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Stereographic, OrientationMatrix, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, AxisAngle, Rodrigues, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Rodrigues, AxisAngle, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, AxisAngle, Homochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Homochoric, AxisAngle, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, AxisAngle, Cubochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Cubochoric, AxisAngle, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, AxisAngle, Stereographic, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Stereographic, AxisAngle, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Rodrigues, Homochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Homochoric, Rodrigues, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Rodrigues, Cubochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Cubochoric, Rodrigues, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Rodrigues, Stereographic, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Stereographic, Rodrigues, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Homochoric, Cubochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Cubochoric, Homochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Homochoric, Stereographic, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Stereographic, Homochoric, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Cubochoric, Stereographic, Quaternion);
    GENERATE_TEST_METHOD(Quaternion, Stereographic, Cubochoric, Quaternion);
  }

  void AxisAngleTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(AxisAngle, Euler, OrientationMatrix, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, OrientationMatrix, Euler, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Euler, Quaternion, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Quaternion, Euler, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Euler, Rodrigues, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Rodrigues, Euler, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Euler, Homochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Homochoric, Euler, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Euler, Cubochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Cubochoric, Euler, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Euler, Stereographic, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Stereographic, Euler, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, OrientationMatrix, Quaternion, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Quaternion, OrientationMatrix, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, OrientationMatrix, Rodrigues, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Rodrigues, OrientationMatrix, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, OrientationMatrix, Homochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Homochoric, OrientationMatrix, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, OrientationMatrix, Cubochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Cubochoric, OrientationMatrix, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, OrientationMatrix, Stereographic, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Stereographic, OrientationMatrix, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Quaternion, Rodrigues, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Rodrigues, Quaternion, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Quaternion, Homochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Homochoric, Quaternion, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Quaternion, Cubochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Cubochoric, Quaternion, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Quaternion, Stereographic, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Stereographic, Quaternion, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Rodrigues, Homochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Homochoric, Rodrigues, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Rodrigues, Cubochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Cubochoric, Rodrigues, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Rodrigues, Stereographic, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Stereographic, Rodrigues, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Homochoric, Cubochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Cubochoric, Homochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Homochoric, Stereographic, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Stereographic, Homochoric, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Cubochoric, Stereographic, AxisAngle);
    GENERATE_TEST_METHOD(AxisAngle, Stereographic, Cubochoric, AxisAngle);
  }

  void RodriguesTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(Rodrigues, Euler, OrientationMatrix, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, OrientationMatrix, Euler, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Euler, Quaternion, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Quaternion, Euler, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Euler, AxisAngle, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, AxisAngle, Euler, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Euler, Homochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Homochoric, Euler, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Euler, Cubochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Cubochoric, Euler, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Euler, Stereographic, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Stereographic, Euler, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, OrientationMatrix, Quaternion, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Quaternion, OrientationMatrix, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, OrientationMatrix, AxisAngle, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, AxisAngle, OrientationMatrix, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, OrientationMatrix, Homochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Homochoric, OrientationMatrix, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, OrientationMatrix, Cubochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Cubochoric, OrientationMatrix, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, OrientationMatrix, Stereographic, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Stereographic, OrientationMatrix, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Quaternion, AxisAngle, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, AxisAngle, Quaternion, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Quaternion, Homochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Homochoric, Quaternion, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Quaternion, Cubochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Cubochoric, Quaternion, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Quaternion, Stereographic, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Stereographic, Quaternion, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, AxisAngle, Homochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Homochoric, AxisAngle, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, AxisAngle, Cubochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Cubochoric, AxisAngle, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, AxisAngle, Stereographic, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Stereographic, AxisAngle, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Homochoric, Cubochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Cubochoric, Homochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Homochoric, Stereographic, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Stereographic, Homochoric, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Cubochoric, Stereographic, Rodrigues);
    GENERATE_TEST_METHOD(Rodrigues, Stereographic, Cubochoric, Rodrigues);
  }

  void HomochoricTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(Homochoric, Euler, OrientationMatrix, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, OrientationMatrix, Euler, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Euler, Quaternion, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Quaternion, Euler, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Euler, AxisAngle, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, AxisAngle, Euler, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Euler, Rodrigues, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Rodrigues, Euler, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Euler, Cubochoric, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Cubochoric, Euler, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Euler, Stereographic, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Stereographic, Euler, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, OrientationMatrix, Quaternion, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Quaternion, OrientationMatrix, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, OrientationMatrix, AxisAngle, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, AxisAngle, OrientationMatrix, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, OrientationMatrix, Rodrigues, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Rodrigues, OrientationMatrix, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, OrientationMatrix, Cubochoric, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Cubochoric, OrientationMatrix, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, OrientationMatrix, Stereographic, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Stereographic, OrientationMatrix, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Quaternion, AxisAngle, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, AxisAngle, Quaternion, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Quaternion, Rodrigues, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Rodrigues, Quaternion, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Quaternion, Cubochoric, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Cubochoric, Quaternion, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Quaternion, Stereographic, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Stereographic, Quaternion, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, AxisAngle, Rodrigues, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Rodrigues, AxisAngle, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, AxisAngle, Cubochoric, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Cubochoric, AxisAngle, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, AxisAngle, Stereographic, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Stereographic, AxisAngle, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Rodrigues, Cubochoric, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Cubochoric, Rodrigues, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Rodrigues, Stereographic, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Stereographic, Rodrigues, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Cubochoric, Stereographic, Homochoric);
    GENERATE_TEST_METHOD(Homochoric, Stereographic, Cubochoric, Homochoric);
  }

  void CubochoricTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(Cubochoric, Euler, OrientationMatrix, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, OrientationMatrix, Euler, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Euler, Quaternion, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Quaternion, Euler, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Euler, AxisAngle, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, AxisAngle, Euler, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Euler, Rodrigues, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Rodrigues, Euler, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Euler, Homochoric, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Homochoric, Euler, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Euler, Stereographic, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Stereographic, Euler, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, OrientationMatrix, Quaternion, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Quaternion, OrientationMatrix, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, OrientationMatrix, AxisAngle, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, AxisAngle, OrientationMatrix, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, OrientationMatrix, Rodrigues, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Rodrigues, OrientationMatrix, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, OrientationMatrix, Homochoric, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Homochoric, OrientationMatrix, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, OrientationMatrix, Stereographic, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Stereographic, OrientationMatrix, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Quaternion, AxisAngle, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, AxisAngle, Quaternion, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Quaternion, Rodrigues, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Rodrigues, Quaternion, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Quaternion, Homochoric, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Homochoric, Quaternion, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Quaternion, Stereographic, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Stereographic, Quaternion, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, AxisAngle, Rodrigues, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Rodrigues, AxisAngle, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, AxisAngle, Homochoric, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Homochoric, AxisAngle, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, AxisAngle, Stereographic, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Stereographic, AxisAngle, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Rodrigues, Homochoric, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Homochoric, Rodrigues, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Rodrigues, Stereographic, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Stereographic, Rodrigues, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Homochoric, Stereographic, Cubochoric);
    GENERATE_TEST_METHOD(Cubochoric, Stereographic, Homochoric, Cubochoric);
  }

  void StereographicTest()
  {
    TEST_PREAMBLE();

    GENERATE_TEST_METHOD(Stereographic, Euler, OrientationMatrix, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, OrientationMatrix, Euler, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Euler, Quaternion, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Quaternion, Euler, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Euler, AxisAngle, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, AxisAngle, Euler, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Euler, Rodrigues, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Rodrigues, Euler, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Euler, Homochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Homochoric, Euler, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Euler, Cubochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Cubochoric, Euler, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, OrientationMatrix, Quaternion, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Quaternion, OrientationMatrix, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, OrientationMatrix, AxisAngle, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, AxisAngle, OrientationMatrix, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, OrientationMatrix, Rodrigues, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Rodrigues, OrientationMatrix, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, OrientationMatrix, Homochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Homochoric, OrientationMatrix, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, OrientationMatrix, Cubochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Cubochoric, OrientationMatrix, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Quaternion, AxisAngle, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, AxisAngle, Quaternion, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Quaternion, Rodrigues, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Rodrigues, Quaternion, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Quaternion, Homochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Homochoric, Quaternion, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Quaternion, Cubochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Cubochoric, Quaternion, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, AxisAngle, Rodrigues, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Rodrigues, AxisAngle, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, AxisAngle, Homochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Homochoric, AxisAngle, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, AxisAngle, Cubochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Cubochoric, AxisAngle, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Rodrigues, Homochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Homochoric, Rodrigues, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Rodrigues, Cubochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Cubochoric, Rodrigues, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Homochoric, Cubochoric, Stereographic);
    GENERATE_TEST_METHOD(Stereographic, Cubochoric, Homochoric, Stereographic);
  }

  // -----------------------------------------------------------------------------
  void operator()()
  {
    std::cout << "<===== Start " << getNameOfClass() << std::endl;

    int err = EXIT_SUCCESS;
    //   DREAM3D_REGISTER_TEST(StartTest());

    DREAM3D_REGISTER_TEST(Test1());

    DREAM3D_REGISTER_TEST(EulerTest());
    DREAM3D_REGISTER_TEST(OrientationMatrixTest());
    DREAM3D_REGISTER_TEST(QuaternionTest());
    DREAM3D_REGISTER_TEST(AxisAngleTest());
    DREAM3D_REGISTER_TEST(RodriguesTest());
    DREAM3D_REGISTER_TEST(HomochoricTest());
    DREAM3D_REGISTER_TEST(CubochoricTest());
    DREAM3D_REGISTER_TEST(StereographicTest());
  }
};
