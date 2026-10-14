#include <catch2/catch.hpp>

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/BrukerNano/EspritConstants.h"
#include "EbsdLib/IO/BrukerNano/H5EspritReader.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "UnitTestSupport.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using namespace ebsdlib;

std::stringstream& operator<<(std::stringstream& out, ebsdlib::NumericTypes::Type value)
{
  out << static_cast<int32_t>(value);
  return out;
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::H5EspritReaderTest::TestH5EspritReader", "[EbsdLib][H5EspritReaderTest]")
{
  const std::string k_HDF5Path = std::string("Section_435");

  H5EspritReader::Pointer reader = H5EspritReader::New();

  reader->readHeaderOnly();
  int32_t err = reader->getErrorCode();
  DREAM3D_REQUIRED(err, ==, -1)

  reader->setHDF5Path(k_HDF5Path);
  reader->readHeaderOnly();
  err = reader->getErrorCode();
  DREAM3D_REQUIRED(err, ==, -387)

  reader->setFileName("Some Random Path");
  reader->readHeaderOnly();
  err = reader->getErrorCode();
  DREAM3D_REQUIRED(err, ==, -2)

  reader->setFileName(ebsdlib::unit_test::H5EspritReaderTest::InputFile);
  reader->readHeaderOnly();
  err = reader->getErrorCode();
  if(err < 0)
  {
    std::cout << reader->getErrorMessage();
  }

  int32_t ncols = reader->getNumColumns();
  DREAM3D_REQUIRED(ncols, ==, 600)

  int32_t nrows = reader->getNumRows();
  DREAM3D_REQUIRED(nrows, ==, 600)

  float xstep = static_cast<float>(reader->getXStep());
  DREAM3D_REQUIRED(xstep, ==, 0.0)
  float ystep = static_cast<float>(reader->getYStep());
  DREAM3D_REQUIRED(ystep, ==, 0.0)

  std::string grid = reader->getGrid();
  DREAM3D_REQUIRED(grid, ==, ebsdlib::H5Esprit::Isometric);
  DREAM3D_REQUIRED(err, >=, 0)
  int32_t patWidth = reader->getPatternWidth();
  DREAM3D_REQUIRED(patWidth, ==, 80)
  int32_t patHeight = reader->getPatternHeight();
  DREAM3D_REQUIRED(patHeight, ==, 60)

  int32_t xDim = reader->getXDimension();
  DREAM3D_REQUIRED(xDim, ==, 600)

  int32_t yDim = reader->getYDimension();
  DREAM3D_REQUIRED(yDim, ==, 600)

  std::list<std::string> scanNames;
  err = reader->readScanNames(scanNames);
  DREAM3D_REQUIRED(scanNames.size(), ==, 1)
  DREAM3D_REQUIRED(scanNames.front(), ==, k_HDF5Path)

  err = reader->readFile();

  //    void* ptr = reader->getPointerByName(ebsdlib::H5Esprit::DD);
  //    DREAM3D_REQUIRE_VALID_POINTER(ptr);
  void* ptr = reader->getPointerByName(ebsdlib::H5Esprit::MAD);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::MADPhase);
  //    DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::NIndexedBands);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::PCX);
  //    DREAM3D_REQUIRE_VALID_POINTER(ptr);
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::PCY);
  //    DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::PHI);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::Phase);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::RadonBandCount);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::RadonQuality);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::XBEAM);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::YBEAM);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::XSAMPLE);
  //    DREAM3D_REQUIRE_VALID_POINTER(ptr);
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::YSAMPLE);
  //    DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::phi1);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::phi2);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::RawPatterns);
  DREAM3D_REQUIRE_NULL_POINTER(ptr);

  //    numType = reader->getPointerType(ebsdlib::H5Esprit::DD);
  //    DREAM3D_REQUIRED(numType ==  ebsdlib::NumericTypes::Type::Float)
  ebsdlib::NumericTypes::Type numType = reader->getPointerType(ebsdlib::H5Esprit::MAD);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Float)
  //    numType = reader->getPointerType(ebsdlib::H5Esprit::MADPhase);
  //    DREAM3D_REQUIRED(numType ==  ebsdlib::NumericTypes::Type::Int32)
  numType = reader->getPointerType(ebsdlib::H5Esprit::NIndexedBands);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Int32)
  //    numType = reader->getPointerType(ebsdlib::H5Esprit::PCX);
  //    DREAM3D_REQUIRED(numType ==  ebsdlib::NumericTypes::Type::Float)
  //    numType = reader->getPointerType(ebsdlib::H5Esprit::PCY);
  //    DREAM3D_REQUIRED(numType ==  ebsdlib::NumericTypes::Type::Float)
  numType = reader->getPointerType(ebsdlib::H5Esprit::PHI);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Float)
  numType = reader->getPointerType(ebsdlib::H5Esprit::Phase);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Int32)
  numType = reader->getPointerType(ebsdlib::H5Esprit::RadonBandCount);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Int32)
  numType = reader->getPointerType(ebsdlib::H5Esprit::RadonQuality);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Float)
  numType = reader->getPointerType(ebsdlib::H5Esprit::XBEAM);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Int32)
  numType = reader->getPointerType(ebsdlib::H5Esprit::YBEAM);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Int32)
  //    numType = reader->getPointerType(ebsdlib::H5Esprit::XSAMPLE);
  //    DREAM3D_REQUIRED(numType ==  ebsdlib::NumericTypes::Type::Float)
  //    numType = reader->getPointerType(ebsdlib::H5Esprit::YSAMPLE);
  //    DREAM3D_REQUIRED(numType ==  ebsdlib::NumericTypes::Type::Float)
  numType = reader->getPointerType(ebsdlib::H5Esprit::phi1);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Float)
  numType = reader->getPointerType(ebsdlib::H5Esprit::phi2);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::Float)
  numType = reader->getPointerType(ebsdlib::H5Esprit::RawPatterns);
  DREAM3D_REQUIRE(numType == ebsdlib::NumericTypes::Type::UInt8);

  //    float* xsamplePtr = reader->getXSAMPLEPointer();
  //    DREAM3D_REQUIRE_VALID_POINTER(xsamplePtr);
  //    reader->releaseXSAMPLEOwnership();
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::XSAMPLE);
  //    DREAM3D_REQUIRE_NULL_POINTER(ptr);
  //    reader->deallocateArrayData<float>(xsamplePtr);

  //    float* ysamplePtr = reader->getYSAMPLEPointer();
  //    DREAM3D_REQUIRE_VALID_POINTER(ysamplePtr);
  //    reader->releaseYSAMPLEOwnership();
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::YSAMPLE);
  //    DREAM3D_REQUIRE_NULL_POINTER(ptr);
  //    reader->deallocateArrayData<float>(ysamplePtr);

  // std::cout << "=============================================================" << std::endl;
  std::set<std::string> arraysToRead;
  arraysToRead.insert(ebsdlib::H5Esprit::phi1);
  arraysToRead.insert(ebsdlib::H5Esprit::PHI);
  arraysToRead.insert(ebsdlib::H5Esprit::phi2);
  reader->setArraysToRead(arraysToRead);
  err = reader->readFile();

  ptr = reader->getPointerByName(ebsdlib::H5Esprit::phi1);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::phi2);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);
  ptr = reader->getPointerByName(ebsdlib::H5Esprit::PHI);
  DREAM3D_REQUIRE_VALID_POINTER(ptr);

  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::XSAMPLE);
  //    DREAM3D_REQUIRE_NULL_POINTER(ptr);
  //    ptr = reader->getPointerByName(ebsdlib::H5Esprit::YSAMPLE);
  //    DREAM3D_REQUIRE_NULL_POINTER(ptr);

#if 0
      reader->setReadPatternData(true);
      err = reader->readFile();
      err = reader->getErrorCode();
      if(err < 0)
      {
        std::cout << reader->getErrorMessage();
      }
      DREAM3D_REQUIRED(err, >=, 0)
#endif
}

TEST_CASE("ebsdlib::H5EspritReaderTest::TuplesArePlacedByBeamCoordinates", "[EbsdLib][H5EspritReaderTest]")
{
  // The Section_435 scan stores 11,146 of its 360,000 tuples out of raster order
  // (simplnx issue #1804). After reading, every tuple must sit at Y BEAM * 600 + X BEAM.
  H5EspritReader::Pointer reader = H5EspritReader::New();
  reader->setFileName(ebsdlib::unit_test::H5EspritReaderTest::InputFile);
  reader->setHDF5Path("Section_435");
  reader->setReadPatternData(false);
  int err = reader->readFile();
  DREAM3D_REQUIRED(err, >=, 0)

  const size_t numCols = 600;
  const size_t numTuples = 360000;
  DREAM3D_REQUIRED(reader->getNumberOfElements(), ==, numTuples)

  const auto* xBeam = static_cast<const int32_t*>(reader->getPointerByName(ebsdlib::H5Esprit::XBEAM));
  const auto* yBeam = static_cast<const int32_t*>(reader->getPointerByName(ebsdlib::H5Esprit::YBEAM));
  DREAM3D_REQUIRE_VALID_POINTER(xBeam)
  DREAM3D_REQUIRE_VALID_POINTER(yBeam)

  size_t misplaced = 0;
  for(size_t i = 0; i < numTuples; i++)
  {
    if(xBeam[i] != static_cast<int32_t>(i % numCols) || yBeam[i] != static_cast<int32_t>(i / numCols))
    {
      misplaced++;
    }
  }
  DREAM3D_REQUIRED(misplaced, ==, 0)

  // File tuples 30 and 31 are swapped in the raw file: file tuple 30 carries X BEAM 31 with
  // MAD 0.6622782 / phi1 -156.33928, file tuple 31 carries X BEAM 30 with MAD 0.9863951 / phi1 -104.250725.
  // After placement the measurement for cell 30 must be the second one.
  const auto* mad = static_cast<const float*>(reader->getPointerByName(ebsdlib::H5Esprit::MAD));
  const auto* phi1 = static_cast<const float*>(reader->getPointerByName(ebsdlib::H5Esprit::phi1));
  DREAM3D_REQUIRE_VALID_POINTER(mad)
  DREAM3D_REQUIRE_VALID_POINTER(phi1)
  DREAM3D_REQUIRE(std::abs(mad[30] - 0.9863951f) < 1.0e-5f)
  DREAM3D_REQUIRE(std::abs(mad[31] - 0.6622782f) < 1.0e-5f)
  DREAM3D_REQUIRE(std::abs(phi1[30] - (-104.250725f)) < 1.0e-3f)
  DREAM3D_REQUIRE(std::abs(phi1[31] - (-156.33928f)) < 1.0e-3f)
}

TEST_CASE("ebsdlib::H5EspritReaderTest::BeamCoordinatesAreReadEvenWhenNotRequested", "[EbsdLib][H5EspritReaderTest]")
{
  H5EspritReader::Pointer reader = H5EspritReader::New();
  reader->setFileName(ebsdlib::unit_test::H5EspritReaderTest::InputFile);
  reader->setHDF5Path("Section_435");
  reader->setReadPatternData(false);
  reader->setArraysToRead({ebsdlib::H5Esprit::MAD});
  int err = reader->readFile();
  DREAM3D_REQUIRED(err, >=, 0)
  const auto* mad = static_cast<const float*>(reader->getPointerByName(ebsdlib::H5Esprit::MAD));
  DREAM3D_REQUIRE_VALID_POINTER(mad)
  DREAM3D_REQUIRE(std::abs(mad[30] - 0.9863951f) < 1.0e-5f)
  DREAM3D_REQUIRE(std::abs(mad[31] - 0.6622782f) < 1.0e-5f)
}

TEST_CASE("ebsdlib::H5EspritReaderTest::ReportsInvalidBeamDatasets", "[EbsdLib][H5EspritReaderTest]")
{
  // Mutate a disposable copy so the checked-in scan remains unchanged.
  const std::filesystem::path outputPath = ebsdlib::unit_test::H5EspritReaderTest::OutputFile;
  std::filesystem::create_directories(outputPath.parent_path());
  std::filesystem::copy_file(ebsdlib::unit_test::H5EspritReaderTest::InputFile, outputPath, std::filesystem::copy_options::overwrite_existing);
  const hid_t fileId = H5Fopen(outputPath.string().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  REQUIRE(fileId >= 0);
  int expectedError = -90320;
  {
    H5Support::H5ScopedFileSentinel sentinel(fileId, true);
    SECTION("Missing X BEAM")
    {
      REQUIRE(H5Ldelete(fileId, "Section_435/EBSD/Data/X BEAM", H5P_DEFAULT) >= 0);
    }
    SECTION("Missing Y BEAM")
    {
      REQUIRE(H5Ldelete(fileId, "Section_435/EBSD/Data/Y BEAM", H5P_DEFAULT) >= 0);
    }
    SECTION("Duplicate beam coordinate")
    {
      std::vector<int32_t> xBeam(360000);
      REQUIRE(H5Support::H5Lite::readPointerDataset(fileId, "Section_435/EBSD/Data/X BEAM", xBeam.data()) >= 0);
      xBeam[1] = xBeam[0];
      const hid_t datasetId = H5Dopen2(fileId, "Section_435/EBSD/Data/X BEAM", H5P_DEFAULT);
      REQUIRE(datasetId >= 0);
      const herr_t writeError = H5Dwrite(datasetId, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, xBeam.data());
      REQUIRE(H5Dclose(datasetId) >= 0);
      REQUIRE(writeError >= 0);
      expectedError = -101103;
    }
  }

  H5EspritReader::Pointer reader = H5EspritReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path("Section_435");
  reader->setArraysToRead({ebsdlib::H5Esprit::MAD});
  REQUIRE(reader->readFile() == expectedError);
  REQUIRE(reader->getErrorCode() == expectedError);
  const std::string message = reader->getErrorMessage();
  REQUIRE(message.find("H5EspritReader Error:") == 0);
  REQUIRE(message.find("X BEAM") != std::string::npos);
  REQUIRE(message.find("Y BEAM") != std::string::npos);
  REQUIRE(message.find("600 x 600") != std::string::npos);
  if(expectedError == -101103)
  {
    REQUIRE(message.find("tuple 1") != std::string::npos);
  }
  std::filesystem::remove(outputPath);
}
