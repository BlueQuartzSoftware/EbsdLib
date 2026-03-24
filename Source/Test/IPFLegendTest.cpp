/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <catch2/catch.hpp>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/TSLColorKey.hpp"
#include "EbsdLib/Utilities/TiffWriter.h"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "UnitTestSupport.hpp"

#include <fstream>

#define IMAGE_WIDTH 512
#define IMAGE_HEIGHT 512

using namespace ebsdlib;

// TODO: This unit test needs to compare the output to something that has been verified as correct

TEST_CASE("ebsdlib::IPFLegendTest", "[EbsdLib][IPFLegendTest]")
{
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  for(size_t index = 0; index < 11; index++)
  {
    SECTION(ops[index]->getSymmetryName())
    {
      ebsdlib::UInt8ArrayType::Pointer image = ops[index]->generateIPFTriangleLegend(IMAGE_WIDTH, false);
      std::stringstream outputFilePathStream;
      outputFilePathStream << ebsdlib::unit_test::k_TestTempDir << "/" << ops[index]->getNameOfClass() << ".tiff";
      auto result = TiffWriter::WriteColorImage(outputFilePathStream.str(), IMAGE_WIDTH, IMAGE_WIDTH, 3, image->data());
      REQUIRE(result.first == 0);
    }
  }
}

TEST_CASE("ebsdlib::IPFLegendTest::NolzeHielscherLegend", "[EbsdLib][IPFLegendTest]")
{
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  for(size_t index = 0; index < 11; index++)
  {
    SECTION(ops[index]->getSymmetryName() + " NH Legend")
    {
      // Switch to NH color key for this operator
      // Use the cubicHigh sector as a simple stand-in for now
      // (the legend generation doesn't use the sector geometry directly --
      //  it goes through generateIPFColor which uses the color key's
      //  direction2Color(eta, chi, angleLimits) overload)
      auto nhKey = std::make_shared<ebsdlib::NolzeHielscherColorKey>(
        ebsdlib::FundamentalSectorGeometry::cubicHigh());
      ops[index]->setColorKey(nhKey);

      auto legend = ops[index]->generateIPFTriangleLegend(64, false);
      REQUIRE(legend != nullptr);
      REQUIRE(legend->getNumberOfTuples() > 0);

      // Verify the image has some non-white pixels (NH key produces colors)
      bool hasNonWhitePixel = false;
      size_t numTuples = legend->getNumberOfTuples();
      for(size_t i = 0; i < numTuples; i++)
      {
        uint8_t* pixel = legend->getTuplePointer(i);
        // Legend is RGB (3 components after alpha removal)
        if(legend->getNumberOfComponents() == 3)
        {
          if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255)
          {
            hasNonWhitePixel = true;
            break;
          }
        }
        else if(legend->getNumberOfComponents() == 4)
        {
          if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255)
          {
            hasNonWhitePixel = true;
            break;
          }
        }
      }
      REQUIRE(hasNonWhitePixel);

      // Reset to TSL for other tests
      ops[index]->setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
    }
  }
}
