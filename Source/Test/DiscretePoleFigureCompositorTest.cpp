#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/DiscretePoleFigureCompositor.h"

#include <array>

using namespace ebsdlib;

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::MarkerSprite", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  int size = 0;
  std::array<float, 3> red = {1.0f, 0.0f, 0.0f};
  UInt8ArrayType::Pointer sprite = RenderDiscreteMarkerSprite(red, 6.0f, size);

  REQUIRE(sprite != nullptr);
  REQUIRE(size >= 12);                            // diameter ~ 2*r + padding
  REQUIRE(size % 2 == 0);                          // even for centered blits
  REQUIRE(sprite->getNumberOfTuples() == static_cast<size_t>(size) * size);
  REQUIRE(sprite->getNumberOfComponents() == 4);

  // Center pixel is opaque red.
  uint8_t* center = sprite->getTuplePointer(static_cast<size_t>(size / 2) * size + size / 2);
  REQUIRE(center[3] == 255);
  REQUIRE(center[0] > 200); // red channel high
  REQUIRE(center[2] < 60);  // blue channel low

  // Corner pixel is fully transparent (outside the circle).
  uint8_t* corner = sprite->getTuplePointer(0);
  REQUIRE(corner[3] == 0);
}
