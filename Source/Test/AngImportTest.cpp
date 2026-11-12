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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/IO/TSL/AngReader.h"

#ifdef EbsdLib_ENABLE_HDF5
#include "EbsdLib/IO/TSL/H5AngImporter.h"
#include "H5Support/H5Utilities.h"
#endif

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

using namespace ebsdlib;

TEST_CASE("ebsdlib::AngImportTest-TestNormalFile", "[EbsdLib][AngImportTest]")
{
  // This is just a normal Ang file, well formed and should read without error
  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::TestFile1);
  int err = reader.readFile();
  std::cout << reader.getErrorMessage();
  REQUIRE(err == 0);

  size_t numElements = reader.getNumberOfElements();
  REQUIRE(numElements == 160);
  float* ptr = reader.getPhi1Pointer();
  REQUIRE(ptr[159] == 12.56637f);
}

TEST_CASE("ebsdlib::AngImportTest-TestMissingHeaders", "[EbsdLib][AngImportTest]")
{

  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::MissingHeader1);
  int err = reader.readHeaderOnly();
  // It should read through this header just fine
  REQUIRE(err > 0);

  int value = reader.getNumEvenCols();
  REQUIRE(value == -1);

  value = reader.getNumOddCols();
  REQUIRE(value == -1);

  value = reader.getNumRows();
  REQUIRE(value == -1);

  float step = reader.getXStep();
  REQUIRE(step == 0.0f);

  step = reader.getYStep();
  REQUIRE(step == 0.0f);

  err = reader.readFile();
  std::cout << reader.getErrorMessage();
  REQUIRE(err == -110);
}

TEST_CASE("ebsdlib::AngImportTest-TestHexGrid", "[EbsdLib][AngImportTest]")
{
  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::HexHeader);
  int err = reader.readHeaderOnly();
  // It should read through this header just fine
  REQUIRE(err > 0);

  err = reader.readFile();
  std::cout << reader.getErrorMessage();
  REQUIRE(err == -400);
}

TEST_CASE("ebsdlib::AngImportTest-TestMissingGrid", "[EbsdLib][AngImportTest]")
{
  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::GridMissing);
  int err = reader.readHeaderOnly();
  // It should read through this header just fine
  REQUIRE(err > 0);

  err = reader.readFile();
  std::cout << reader.getErrorMessage();
  REQUIRE(err == -300);
}

TEST_CASE("ebsdlib::AngImportTest-TestShortFile", "[EbsdLib][AngImportTest]")
{
  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::ShortFile);
  int err = reader.readFile();
  // It should read through this header just fine but die when reading the file because there is not enough data being read
  std::cout << reader.getErrorMessage();
  REQUIRE(err < 0);
}

TEST_CASE("ebsdlib::AngImportTest-TestOutOfOrderPhase", "[EbsdLib][AngImportTest]")
{
  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::OutOfOrderPhase);
  int err = reader.readFile();
  REQUIRE(err == 0);

  auto phaseVector = reader.getPhaseVector();
  for(size_t i = 0; i < phaseVector.size(); i++)
  {
    REQUIRE(phaseVector[i]->getPhaseIndex() == i + 1);
  }
}

TEST_CASE("ebsdlib::AngImportTest-TestOutOfOrderGrid", "[EbsdLib][AngImportTest]")
{
  // Out_Of_Order.ang is a 189 x 201 square-grid scan whose rows are not stored in raster order.
  AngReader reader;
  reader.setFileName(ebsdlib::unit_test::AngImportTest::OutOfOrderGrid);
  int err = reader.readFile();
  REQUIRE(err == 0);

  const auto numCols = static_cast<size_t>(reader.getNumOddCols());
  const auto numRows = static_cast<size_t>(reader.getNumRows());
  REQUIRE(numCols == 189);
  REQUIRE(numRows == 201);

  const float* x = reader.getXPositionPointer();
  const float* y = reader.getYPositionPointer();
  REQUIRE(x != nullptr);
  REQUIRE(y != nullptr);
  const float xStep = reader.getXStep();
  const float yStep = reader.getYStep();
  const float xMin = *std::min_element(x, x + numCols * numRows);
  const float yMin = *std::min_element(y, y + numCols * numRows);

  size_t misplaced = 0;
  for(size_t i = 0; i < numCols * numRows; i++)
  {
    const auto col = static_cast<size_t>(std::llround((x[i] - xMin) / xStep));
    const auto row = static_cast<size_t>(std::llround((y[i] - yMin) / yStep));
    if(col != i % numCols || row != i / numCols)
    {
      misplaced++;
    }
  }
  REQUIRE(misplaced == 0);
}

namespace
{
void checkSquareGridColumns(const std::string& name, const std::string& columns, int expectedError)
{
  const auto path = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / (name + ".ang");
  std::ofstream out(path);
  REQUIRE(out.is_open());
  out << "# Phase 1\n# MaterialName Nickel\n# Symmetry 43\n# LatticeConstants 3.52 3.52 3.52 90 90 90\n"
      << "# GRID: SqrGrid\n# XSTEP: 1\n# YSTEP: 1\n# NROWS: 2\n"
      << columns;
  for(int i = 0; i < 6; ++i)
  {
    out << i + 1 << " 0 0 " << i % 3 << " " << i / 3 << " 100 1 1 0 0\n";
  }
  out.close();
  REQUIRE(out.good());

  AngReader reader;
  reader.setFileName(path.string());
  const int error = reader.readFile();
  INFO(reader.getErrorMessage());
  REQUIRE(error == expectedError);
  REQUIRE(reader.getErrorCode() == expectedError);
  if(expectedError < 0)
  {
    REQUIRE(reader.getErrorMessage().find(path.string()) != std::string::npos);
    if(expectedError == -120)
    {
      REQUIRE(reader.getErrorMessage().find("NCOLS_ODD=3") != std::string::npos);
      REQUIRE(reader.getErrorMessage().find("NCOLS_EVEN=2") != std::string::npos);
    }
    else
    {
      REQUIRE(reader.getErrorMessage().find("NCOLS_ODD=0") != std::string::npos);
      REQUIRE(reader.getErrorMessage().find("NCOLS_EVEN=0") != std::string::npos);
    }
    return;
  }
  REQUIRE(reader.getNumberOfElements() == 6);
  REQUIRE(reader.getXDimension() == 3);
  REQUIRE(reader.getNumOddCols() == 3);
  REQUIRE(reader.getNumEvenCols() == 3);
  const float* phi1Ptr = reader.getPhi1Pointer();
  REQUIRE(phi1Ptr != nullptr);
  for(size_t i = 0; i < 6; ++i)
  {
    REQUIRE(phi1Ptr[i] == static_cast<float>(i + 1));
  }
}
} // namespace

TEST_CASE("ebsdlib::AngImportTest-SquareGridColumnsEvenOnly", "[EbsdLib][AngImportTest]")
{
  checkSquareGridColumns("SquareGridColumnsEvenOnly", "# NCOLS_EVEN: 3\n", 0);
}

TEST_CASE("ebsdlib::AngImportTest-SquareGridColumnsOddOnly", "[EbsdLib][AngImportTest]")
{
  checkSquareGridColumns("SquareGridColumnsOddOnly", "# NCOLS_ODD: 3\n", 0);
}

TEST_CASE("ebsdlib::AngImportTest-SquareGridColumnsMismatch", "[EbsdLib][AngImportTest]")
{
  checkSquareGridColumns("SquareGridColumnsMismatch", "# NCOLS_ODD: 3\n# NCOLS_EVEN: 2\n", -120);
}

TEST_CASE("ebsdlib::AngImportTest-SquareGridColumnsNonpositive", "[EbsdLib][AngImportTest]")
{
  checkSquareGridColumns("SquareGridColumnsNonpositive", "# NCOLS_ODD: 0\n# NCOLS_EVEN: 0\n", -130);
}

TEST_CASE("ebsdlib::AngImportTest-SquareGridColumnsEqual", "[EbsdLib][AngImportTest]")
{
  checkSquareGridColumns("SquareGridColumnsEqual", "# NCOLS_ODD: 3\n# NCOLS_EVEN: 3\n", 0);
}
