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
#include "EbsdLib/IO/HKL/H5CtfImporter.h"
#include "EbsdLib/IO/HKL/H5CtfVolumeReader.h"
#include "EbsdLib/IO/TSL/H5AngImporter.h"
#include "EbsdLib/IO/TSL/H5AngReader.h"
#include "EbsdLib/IO/TSL/H5AngVolumeReader.h"
#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
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

#ifdef EbsdLib_ENABLE_HDF5

TEST_CASE("ebsdlib::H5AngMissingSlice", "[EbsdLib][H5AngMissingSlice]")
{
  const std::string fileName = unit_test::k_TestTempDir + "H5AngMissingSlice.h5ebsd";
  struct Cleanup
  {
    std::string path;
    ~Cleanup()
    {
      std::error_code error;
      std::filesystem::remove(path, error);
    }
  } cleanup{fileName};
  AngReader source;
  source.setFileName(unit_test::AngImportTest::TestFile1);
  REQUIRE(source.readFile() == 0);
  {
    const hid_t fileId = H5Support::H5Utilities::createFile(fileName);
    REQUIRE(fileId >= 0);
    H5Support::H5ScopedFileSentinel sentinel(fileId, true);
    auto importer = H5AngImporter::New();
    REQUIRE(importer->importFile(fileId, 0, unit_test::AngImportTest::TestFile1) >= 0);
    using namespace H5Support;
    REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::ZStartIndex, int32_t{0}) >= 0);
    REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::ZEndIndex, int32_t{0}) >= 0);
    REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::XPoints, int64_t{source.getNumEvenCols()}) >= 0);
    REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::YPoints, int64_t{source.getNumRows()}) >= 0);
    for(const auto& key : {H5Ebsd::XResolution, H5Ebsd::YResolution, H5Ebsd::ZResolution})
    {
      REQUIRE(H5Lite::writeScalarDataset(fileId, key, 1.0f) >= 0);
    }
    REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::StackingOrder, uint32_t{RefFrameZDir::LowtoHigh}) >= 0);
    for(const auto& key : {H5Ebsd::SampleTransformationAngle, H5Ebsd::EulerTransformationAngle})
    {
      REQUIRE(H5Lite::writeScalarDataset(fileId, key, 0.0f) >= 0);
    }
    for(const auto& key : {H5Ebsd::SampleTransformationAxis, H5Ebsd::EulerTransformationAxis})
    {
      REQUIRE(H5Lite::writeVectorDataset(fileId, key, std::vector<hsize_t>{3}, std::vector<float>{0, 0, 1}) >= 0);
    }
    REQUIRE(H5Lite::writeStringDataset(fileId, H5Ebsd::Manufacturer, "TSL") >= 0);
  }
  SECTION("Empty path")
  {
    auto reader = H5AngReader::New();
    reader->setFileName(fileName);
    CHECK(reader->readFile() == -90040);
    CHECK(reader->getErrorCode() == -90040);
    CHECK(reader->getErrorMessage().find(fileName) != std::string::npos);
  }
  SECTION("File open failure")
  {
    auto reader = H5AngReader::New();
    reader->setFileName(fileName + ".missing");
    reader->setHDF5Path("0");
    CHECK(reader->readFile() == -90041);
    CHECK(reader->getErrorCode() == -90041);
    CHECK(reader->getErrorMessage().find(fileName + ".missing") != std::string::npos);
  }
  SECTION("Header or data failure survives file close")
  {
    const bool removeHeader = GENERATE(true, false);
    {
      const hid_t fileId = H5Support::H5Utilities::openFile(fileName, false);
      REQUIRE(fileId >= 0);
      H5Support::H5ScopedFileSentinel sentinel(fileId, true);
      REQUIRE(H5Ldelete(fileId, removeHeader ? "0/Header" : "0/Data", H5P_DEFAULT) >= 0);
    }
    auto reader = H5AngReader::New();
    reader->setFileName(fileName);
    reader->setHDF5Path("0");
    const int expected = removeHeader ? -90043 : -90044;
    CHECK(reader->readFile() == expected);
    CHECK(reader->getErrorCode() == expected);
    CHECK(reader->getErrorMessage().find(fileName) != std::string::npos);
    CHECK(reader->getErrorMessage().find("'0'") != std::string::npos);
  }
  SECTION("CTF missing slice also fails")
  {
    auto importer = H5CtfImporter::New();
    {
      const hid_t fileId = H5Support::H5Utilities::openFile(fileName, false);
      REQUIRE(fileId >= 0);
      H5Support::H5ScopedFileSentinel sentinel(fileId, true);
      REQUIRE(H5Ldelete(fileId, "0", H5P_DEFAULT) >= 0);
      REQUIRE(importer->importFile(fileId, 0, unit_test::CtfReaderTest::USInputFile1) >= 0);
    }
    int64_t x = 0;
    int64_t y = 0;
    importer->getDims(x, y);
    auto reader = H5CtfVolumeReader::New();
    reader->setFileName(fileName);
    reader->setSliceStart(0);
    REQUIRE(reader->loadData(x, y, 1, RefFrameZDir::LowtoHigh) == 0);
    reader = H5CtfVolumeReader::New();
    reader->setFileName(fileName);
    reader->setSliceStart(0);
    CHECK(reader->loadData(x, y, 2, RefFrameZDir::LowtoHigh) == -77000);
  }
  SECTION("Direct missing group")
  {
    auto reader = H5AngReader::New();
    reader->setFileName(fileName);
    reader->setHDF5Path("1");
    CHECK(reader->readFile() < 0);
    CHECK(reader->getErrorCode() == -90042);
    CHECK(reader->getErrorMessage().find("'1'") != std::string::npos);
    CHECK(reader->getErrorMessage().find(fileName) != std::string::npos);
  }
  SECTION("Volume extends past last slice")
  {
    auto reader = H5AngVolumeReader::New();
    reader->setFileName(fileName);
    reader->setSliceStart(0);
    REQUIRE(reader->loadData(source.getNumEvenCols(), source.getNumRows(), 1, RefFrameZDir::LowtoHigh) == 0);
    reader = H5AngVolumeReader::New();
    reader->setFileName(fileName);
    reader->setSliceStart(0);
    CHECK(reader->loadData(source.getNumEvenCols(), source.getNumRows(), 2, RefFrameZDir::LowtoHigh) < 0);
    CHECK(reader->getErrorCode() == -90042);
    CHECK_FALSE(reader->getErrorMessage().empty());
  }
}
#endif
