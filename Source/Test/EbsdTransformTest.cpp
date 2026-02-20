#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdTransform.h"

#include <array>

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdTransformTest::TSLTransformation", "[EbsdLib][EbsdTransformTest]")
{
  // TSL/EDAX: sample=[180, 0, 1, 0], euler=[90, 0, 0, 1]
  std::array<float, 4> sampleTransformation = {180.0f, 0.0f, 1.0f, 0.0f};
  std::array<float, 4> eulerTransformation = {90.0f, 0.0f, 0.0f, 1.0f};

  auto result = EbsdTransform::IdentifyStandardTransformation(sampleTransformation, eulerTransformation);
  REQUIRE(result == ebsdlib::TSLdefault);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdTransformTest::HKLTransformation", "[EbsdLib][EbsdTransformTest]")
{
  // HKL: sample=[180, 0, 1, 0], euler=[0, 0, 0, 1]
  std::array<float, 4> sampleTransformation = {180.0f, 0.0f, 1.0f, 0.0f};
  std::array<float, 4> eulerTransformation = {0.0f, 0.0f, 0.0f, 1.0f};

  auto result = EbsdTransform::IdentifyStandardTransformation(sampleTransformation, eulerTransformation);
  REQUIRE(result == ebsdlib::HKLdefault);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdTransformTest::HEDMTransformation", "[EbsdLib][EbsdTransformTest]")
{
  // HEDM: sample=[0, 0, 0, 1], euler=[0, 0, 0, 1]
  std::array<float, 4> sampleTransformation = {0.0f, 0.0f, 0.0f, 1.0f};
  std::array<float, 4> eulerTransformation = {0.0f, 0.0f, 0.0f, 1.0f};

  auto result = EbsdTransform::IdentifyStandardTransformation(sampleTransformation, eulerTransformation);
  REQUIRE(result == ebsdlib::HEDMdefault);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdTransformTest::UnknownTransformation", "[EbsdLib][EbsdTransformTest]")
{
  // Unknown/custom parameters
  std::array<float, 4> sampleTransformation = {45.0f, 1.0f, 0.0f, 0.0f};
  std::array<float, 4> eulerTransformation = {45.0f, 1.0f, 0.0f, 0.0f};

  auto result = EbsdTransform::IdentifyStandardTransformation(sampleTransformation, eulerTransformation);
  REQUIRE(result == ebsdlib::UnknownCoordinateMapping);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdTransformTest::ClassName", "[EbsdLib][EbsdTransformTest]")
{
  REQUIRE(EbsdTransform::ClassName() == "EbsdTransform");

  EbsdTransform transform;
  REQUIRE(transform.getNameOfClass() == "EbsdTransform");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdTransformTest::ZeroRotation", "[EbsdLib][EbsdTransformTest]")
{
  // Both zeros but not matching HEDM pattern (different axis)
  std::array<float, 4> sampleTransformation = {0.0f, 1.0f, 0.0f, 0.0f};
  std::array<float, 4> eulerTransformation = {0.0f, 0.0f, 0.0f, 1.0f};

  auto result = EbsdTransform::IdentifyStandardTransformation(sampleTransformation, eulerTransformation);
  REQUIRE(result == ebsdlib::UnknownCoordinateMapping);
}
