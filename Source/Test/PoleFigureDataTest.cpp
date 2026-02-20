#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/PoleFigureData.h"

#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureDataTest::DefaultConstruction", "[EbsdLib][PoleFigureDataTest]")
{
  PoleFigureData pfd;
  REQUIRE(pfd.imageSize[0] == 0);
  REQUIRE(pfd.imageSize[1] == 0);
  REQUIRE(pfd.kernelRadius[0] == 3);
  REQUIRE(pfd.kernelRadius[1] == 3);
  REQUIRE(pfd.xData.empty());
  REQUIRE(pfd.yData.empty());
  REQUIRE(pfd.label.empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureDataTest::ParameterizedConstruction", "[EbsdLib][PoleFigureDataTest]")
{
  std::vector<float> xData = {1.0f, 2.0f, 3.0f};
  std::vector<float> yData = {4.0f, 5.0f, 6.0f};
  int32_t kernelRad[2] = {5, 7};
  int32_t size[2] = {256, 512};

  PoleFigureData pfd(xData, yData, "TestLabel", kernelRad, size);

  REQUIRE(pfd.xData.size() == 3);
  REQUIRE(pfd.yData.size() == 3);
  REQUIRE(pfd.xData[0] == Approx(1.0f));
  REQUIRE(pfd.yData[2] == Approx(6.0f));
  REQUIRE(pfd.label == "TestLabel");
  REQUIRE(pfd.kernelRadius[0] == 5);
  REQUIRE(pfd.kernelRadius[1] == 7);
  REQUIRE(pfd.imageSize[0] == 256);
  REQUIRE(pfd.imageSize[1] == 512);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureDataTest::CopyConstruction", "[EbsdLib][PoleFigureDataTest]")
{
  std::vector<float> xData = {1.0f, 2.0f};
  std::vector<float> yData = {3.0f, 4.0f};
  int32_t kernelRad[2] = {5, 7};
  int32_t size[2] = {100, 200};

  PoleFigureData original(xData, yData, "Original", kernelRad, size);
  PoleFigureData copy(original);

  REQUIRE(copy.xData == original.xData);
  REQUIRE(copy.yData == original.yData);
  REQUIRE(copy.label == original.label);
  REQUIRE(copy.imageSize[0] == original.imageSize[0]);
  REQUIRE(copy.imageSize[1] == original.imageSize[1]);
  REQUIRE(copy.kernelRadius[0] == original.kernelRadius[0]);
  REQUIRE(copy.kernelRadius[1] == original.kernelRadius[1]);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureDataTest::AssignmentOperator", "[EbsdLib][PoleFigureDataTest]")
{
  std::vector<float> xData = {10.0f, 20.0f};
  std::vector<float> yData = {30.0f, 40.0f};
  int32_t kernelRad[2] = {2, 4};
  int32_t size[2] = {64, 128};

  PoleFigureData original(xData, yData, "Assigned", kernelRad, size);
  PoleFigureData assigned;
  assigned = original;

  REQUIRE(assigned.xData == original.xData);
  REQUIRE(assigned.yData == original.yData);
  REQUIRE(assigned.label == "Assigned");
  REQUIRE(assigned.imageSize[0] == 64);
  REQUIRE(assigned.imageSize[1] == 128);
  REQUIRE(assigned.kernelRadius[0] == 2);
  REQUIRE(assigned.kernelRadius[1] == 4);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureDataTest::DataIndependenceAfterCopy", "[EbsdLib][PoleFigureDataTest]")
{
  std::vector<float> xData = {1.0f};
  std::vector<float> yData = {2.0f};
  int32_t kernelRad[2] = {3, 3};
  int32_t size[2] = {50, 50};

  PoleFigureData original(xData, yData, "Original", kernelRad, size);
  PoleFigureData copy(original);

  // Modify original
  original.xData.push_back(99.0f);
  original.label = "Modified";
  original.imageSize[0] = 999;

  // Copy should be unaffected
  REQUIRE(copy.xData.size() == 1);
  REQUIRE(copy.label == "Original");
  REQUIRE(copy.imageSize[0] == 50);
}
