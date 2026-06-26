#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/DiscretePoleFigureCompositor.h"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"

#include <array>
#include <chrono>
#include <iostream>
#include <vector>

using namespace ebsdlib;

namespace
{
FloatArrayType::Pointer MakeEulers(size_t n)
{
  std::vector<size_t> compDims = {3};
  auto eulers = FloatArrayType::CreateArray(n, compDims, "TestEulers", true);
  for(size_t i = 0; i < n; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = static_cast<float>((i * 7 + 3) % 360) * 0.0174533f;
    ptr[1] = static_cast<float>((i * 13 + 5) % 180) * 0.0174533f;
    ptr[2] = static_cast<float>((i * 19 + 11) % 360) * 0.0174533f;
  }
  return eulers;
}

CompositePoleFigureConfiguration_t MakeConfig(FloatArrayType* eulers)
{
  CompositePoleFigureConfiguration_t config;
  config.eulers = eulers;
  config.imageDim = 256;
  config.discrete = true;
  config.discreteHeatMap = false;
  config.laueOpsIndex = 0; // Hexagonal-High (6/mmm)
  config.layoutType = PoleFigureLayoutType::Horizontal;
  config.phaseName = "TestPhase";
  config.phaseNumber = 1;
  config.title = "Discrete Vector Test";
  return config;
}
} // namespace

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::MarkerSprite", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  int size = 0;
  std::array<float, 3> red = {1.0f, 0.0f, 0.0f};
  UInt8ArrayType::Pointer sprite = RenderDiscreteMarkerSprite(red, 6.0f, size);

  REQUIRE(sprite != nullptr);
  REQUIRE(size >= 12);    // diameter ~ 2*r + padding
  REQUIRE(size % 2 == 0); // even for centered blits
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

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::ProducesImage", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  auto eulers = MakeEulers(500);
  CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());

  DiscretePoleFigureCompositor compositor;
  CompositePoleFigureResult result = compositor.generateCompositeImage(config);

  REQUIRE(result.image != nullptr);
  REQUIRE(result.width > 0);
  REQUIRE(result.height > 0);
  REQUIRE(result.image->getNumberOfComponents() == 4);
  REQUIRE(result.image->getNumberOfTuples() == static_cast<size_t>(result.width) * result.height);

  bool hasNonWhite = false;
  for(size_t i = 0; i < result.image->getNumberOfTuples() && !hasNonWhite; i++)
  {
    uint8_t* px = result.image->getTuplePointer(i);
    if(px[0] != 255 || px[1] != 255 || px[2] != 255)
    {
      hasNonWhite = true;
    }
  }
  REQUIRE(hasNonWhite);
}

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::IsDeterministic", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  auto eulers = MakeEulers(800);
  UInt8ArrayType::Pointer reference;
  for(int run = 0; run < 10; run++)
  {
    CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());
    DiscretePoleFigureCompositor compositor;
    CompositePoleFigureResult result = compositor.generateCompositeImage(config);
    REQUIRE(result.image != nullptr);
    if(run == 0)
    {
      reference = result.image;
      continue;
    }
    REQUIRE(result.image->getSize() == reference->getSize());
    size_t mismatches = 0;
    for(size_t i = 0; i < reference->getSize(); i++)
    {
      if((*reference)[i] != (*result.image)[i])
      {
        mismatches++;
      }
    }
    REQUIRE(mismatches == 0);
  }
}

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::DispatchRoutesByConfig", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  auto eulers = MakeEulers(400);

  {
    CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());
    config.discrete = true;
    config.discreteHeatMap = false;
    CompositePoleFigureResult result = GeneratePoleFigureComposite(config);
    REQUIRE(result.image != nullptr);
    REQUIRE(result.width > 0);
  }
  {
    CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());
    config.discrete = false;
    config.discreteHeatMap = false;
    CompositePoleFigureResult result = GeneratePoleFigureComposite(config);
    REQUIRE(result.image != nullptr);
    REQUIRE(result.width > 0);
  }
}

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::LargePointCountPerformance", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  // 250k orientations * (hex symmetry multiplicities up to 6) => >1M poles per figure.
  auto eulers = MakeEulers(250000);
  CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());
  config.imageDim = 512;

  const auto start = std::chrono::steady_clock::now();
  CompositePoleFigureResult result = GeneratePoleFigureComposite(config);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const double seconds = std::chrono::duration<double>(elapsed).count();
  std::cout << "Discrete >1M-pole render: " << seconds << " s" << std::endl;

  REQUIRE(result.image != nullptr);
  // Generous regression guard: decimation+sprite must keep this well under a minute.
  // A per-point arc+fill regression would take many minutes and trip this.
  REQUIRE(seconds < 60.0);
}

TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::HonorsFlipFinalImage", "[EbsdLib][DiscretePoleFigureCompositorTest]")
{
  auto eulers = MakeEulers(1500);

  auto render = [&](bool flip) {
    CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());
    config.flipFinalImage = flip;
    return GeneratePoleFigureComposite(config);
  };
  CompositePoleFigureResult up = render(true);
  CompositePoleFigureResult down = render(false);
  REQUIRE(up.image != nullptr);
  REQUIRE(down.image != nullptr);
  REQUIRE(up.width == down.width);

  // Figure box for family 1 (default order {0,1,2} => display slot 1).
  CompositePoleFigureConfiguration_t cfg = MakeConfig(eulers.get());
  LayoutMetrics layout = PoleFigureCompositor::computeLayoutMetrics(cfg);
  const auto origin = layout.origins[1];
  const int x0 = static_cast<int>(origin[0] + layout.margins);
  const int y0 = static_cast<int>(origin[1] + layout.fontPtSize * 2.0f + layout.margins * 2.0f);
  const int dim = cfg.imageDim;
  const int W = up.width;

  // Count bytes that differ within the figure-1 box; the marker layer must move.
  size_t boxDiffs = 0;
  for(int y = y0; y < y0 + dim; y++)
  {
    for(int x = x0; x < x0 + dim; x++)
    {
      const size_t idx = (static_cast<size_t>(y) * W + x) * 4;
      for(int c = 0; c < 4; c++)
      {
        if((*up.image)[idx + c] != (*down.image)[idx + c])
        {
          boxDiffs++;
        }
      }
    }
  }
  REQUIRE(boxDiffs > 0); // markers are vertically mirrored by the flag
}

// Hidden manual-validation aid. Run explicitly:
//   ./Bin/EbsdLibUnitTest "[DiscreteVisual]"
// Writes discrete_markers.png in the current directory for eyeballing vs MTEX/OIM.
TEST_CASE("ebsdlib::DiscretePoleFigureCompositorTest::VisualExport", "[.][DiscreteVisual]")
{
  auto eulers = MakeEulers(2600);
  CompositePoleFigureConfiguration_t config = MakeConfig(eulers.get());
  config.imageDim = 512;

  CompositePoleFigureResult result = GeneratePoleFigureComposite(config);
  REQUIRE(result.image != nullptr);
  auto r = PngWriter::WriteColorImage("./discrete_markers.png", result.width, result.height, 4, result.image->data());
  REQUIRE(r.first == 0);
}
