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

#include <cstring>
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
