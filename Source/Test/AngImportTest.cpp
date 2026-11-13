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
#include <string>
#include <vector>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/IO/TSL/AngReader.h"

#ifdef EbsdLib_ENABLE_HDF5
#include "EbsdLib/IO/TSL/H5AngImporter.h"
#include "H5Support/H5Utilities.h"
#endif

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

using namespace ebsdlib;

namespace
{
// Writes a 2 x 2 square-grid .ang file with one phase. phaseLines holds the LatticeConstants, NumberFamilies and hklFamilies lines.
std::string writePhaseAngFile(const std::string& name, const std::string& phaseLines)
{
  const std::string filePath = (std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / (name + ".ang")).string();
  std::ofstream out(filePath);
  REQUIRE(out.is_open());
  out << "# TEM_PIXperUM          1.000000\n#\n# Phase 1\n# MaterialName  \tNickel\n# Formula     \tNi\n# Symmetry              43\n"
      << phaseLines << "#\n# GRID: SqrGrid\n# XSTEP: 1.0\n# YSTEP: 1.0\n# NCOLS_ODD: 2\n# NCOLS_EVEN: 2\n# NROWS: 2\n";
  for(int i = 0; i < 4; ++i)
  {
    out << "0.1 0.2 0.3 " << i % 2 << " " << i / 2 << " 100 1 1 0 0\n";
  }
  out.close();
  REQUIRE(out.good());
  return filePath;
}
} // namespace

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

TEST_CASE("ebsdlib::AngImportTest-TestShortPhaseLines", "[EbsdLib][AngImportTest]")
{
  const std::string validLattice = "# LatticeConstants      3.560 3.560 3.560  90.000  90.000  90.000\n";
  const std::string validFamilies = "# NumberFamilies        2\n# hklFamilies   \t 1  1  1 1 8.469246 1\n# hklFamilies   \t 2  0  0 1 7.008760 1\n";

  SECTION("Valid phase lines")
  {
    AngReader reader;
    reader.setFileName(writePhaseAngFile("AngPhaseValid", validLattice + validFamilies));
    const int err = reader.readFile();
    INFO(reader.getErrorMessage());
    REQUIRE(err == 0);
    const auto phases = reader.getPhaseVector();
    REQUIRE(phases.size() == 1);
    REQUIRE(phases[0]->getLatticeConstants() == std::vector<float>{3.56f, 3.56f, 3.56f, 90.0f, 90.0f, 90.0f});
    REQUIRE(phases[0]->getHKLFamilies().size() == 2);
  }

  struct BadPhase
  {
    std::string name;
    std::string lines;
    int errorCode;
    std::string text;
  };
  const std::vector<BadPhase> badPhases = {
      {"AngPhaseShortLattice", "# LatticeConstants 3.52 3.52\n" + validFamilies, -117, "LatticeConstants 3.52 3.52"},
      {"AngPhaseLatticeText", "# LatticeConstants 3.52 3.52 abc 90 90 90\n" + validFamilies, -117, "LatticeConstants 3.52 3.52 abc 90 90 90"},
      {"AngPhaseShortFamily", validLattice + "# NumberFamilies        2\n# hklFamilies   \t 1  1  1 1 8.469246 1\n# hklFamilies 2 0\n", -118, "hklFamilies 2 0"},
  };
  for(const auto& badPhase : badPhases)
  {
    DYNAMIC_SECTION(badPhase.name)
    {
      const std::string filePath = writePhaseAngFile(badPhase.name, badPhase.lines);

      AngReader headerReader;
      headerReader.setFileName(filePath);
      CHECK(headerReader.readHeaderOnly() == badPhase.errorCode);

      AngReader reader;
      reader.setFileName(filePath);
      const int err = reader.readFile();
      INFO(reader.getErrorMessage());
      REQUIRE(err == badPhase.errorCode);
      const std::string message = reader.getErrorMessage();
      REQUIRE(message.find(filePath) != std::string::npos);
      REQUIRE(message.find("'" + badPhase.text + "'") != std::string::npos);
    }
  }
}
