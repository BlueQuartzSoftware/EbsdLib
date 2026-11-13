#include <catch2/catch.hpp>

#include "EbsdLib/IO/HKL/CprReader.h"
#include "EbsdLib/IO/HKL/CtfReader.h"

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <string>

using namespace ebsdlib;

namespace
{
std::filesystem::path TestDir()
{
  const std::filesystem::path dir = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / "HklGridHeaderValidationTest";
  std::filesystem::create_directories(dir);
  return dir;
}

/**
 * @brief Writes a 2x2 .ctf file with numSlices slices. The grid header lines are
 * passed in so that a test can change them.
 * @return The path to the .ctf file
 */
std::string WriteCtfFile(const std::string& baseName, const std::string& gridLines, int numSlices = 1)
{
  const std::filesystem::path ctfPath = TestDir() / (baseName + ".ctf");
  std::ofstream out(ctfPath, std::ios::out | std::ios::trunc);
  out << "Channel Text File\n";
  out << "Prj\tTest.cpr\n";
  out << "Author\t[Unknown]\n";
  out << "JobMode\tGrid\n";
  out << gridLines;
  out << "AcqE1\t0\nAcqE2\t0\nAcqE3\t0\n";
  out << "Euler angles refer to Sample Coordinate system (CS0)!\tMag\t200\tCoverage\t100\tDevice\t0\tKV\t15\tTiltAngle\t70\tTiltAxis\t0\n";
  out << "Phases\t1\n";
  out << "2.87;2.87;2.87\t90;90;90\tIron\t11\t229\n";
  out << "Phase\tX\tY\tBands\tError\tEuler1\tEuler2\tEuler3\tMAD\tBC\tBS\n";
  for(int slice = 0; slice < numSlices; slice++)
  {
    out << "1\t0.0000\t0.0000\t8\t0\t10.0\t20.0\t30.0\t0.5\t100\t200\n";
    out << "1\t0.5000\t0.0000\t8\t0\t11.0\t21.0\t31.0\t0.5\t100\t200\n";
    out << "1\t0.0000\t0.5000\t8\t0\t12.0\t22.0\t32.0\t0.5\t100\t200\n";
    out << "1\t0.5000\t0.5000\t8\t0\t13.0\t23.0\t33.0\t0.5\t100\t200\n";
  }
  return ctfPath.string();
}

std::string CtfGridLines(const std::string& xStep, const std::string& yStep, const std::string& zCellsLine)
{
  return "XCells\t2\nYCells\t2\n" + zCellsLine + "XStep\t" + xStep + "\nYStep\t" + yStep + "\n";
}

/**
 * @brief Writes a .cpr/.crc pair with one phase and the phi1, Phi and phi2
 * fields. The .crc file holds 4 scan points.
 * @return The path to the .cpr file
 */
std::string WriteCprFile(const std::string& baseName, const std::string& gridDistX, const std::string& gridDistY, const std::string& xCells, const std::string& yCells)
{
  const std::filesystem::path cprPath = TestDir() / (baseName + ".cpr");
  {
    std::ofstream out(cprPath, std::ios::out | std::ios::trunc);
    out << "[General]\nAuthor=Test\nJobMode=Grid\n";
    out << "[Job]\nMagnification=200\nCoverage=100\nDevice=0\nkV=20\nTiltAngle=70\nTiltAxis=0\n";
    out << "GridDistX=" << gridDistX << "\nGridDistY=" << gridDistY << "\nxCells=" << xCells << "\nyCells=" << yCells << "\n";
    out << "[Phases]\nCount=1\n";
    out << "[Phase1]\nStructureName=Iron\nReference=Test\na=2.87\nb=2.87\nc=2.87\nalpha=90\nbeta=90\ngamma=90\nLaueGroup=11\nSpaceGroup=229\nID1=1\nID2=2\n";
    out << "[Fields]\nCount=3\nField1=3\nField2=4\nField3=5\n";
  }
  std::filesystem::path crcPath = cprPath;
  crcPath.replace_extension(".crc");
  {
    std::ofstream out(crcPath, std::ios::out | std::ios::trunc | std::ios::binary);
    for(int i = 0; i < 4; i++)
    {
      const uint8_t phase = 1;
      const std::array<float, 3> euler = {0.1F * static_cast<float>(i), 0.2F, 0.3F};
      out.write(reinterpret_cast<const char*>(&phase), sizeof(phase));
      out.write(reinterpret_cast<const char*>(euler.data()), sizeof(float) * euler.size());
    }
  }
  return cprPath.string();
}
} // namespace

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::HklGridHeaderValidationTest::CtfValidGrid", "[EbsdLib][HklGridHeaderValidationTest]")
{
  SECTION("ZCells missing reads one slice")
  {
    CtfReader reader;
    reader.setFileName(WriteCtfFile("ValidNoZCells", CtfGridLines("0.5", "0.5", "")));
    DREAM3D_REQUIRED(reader.readFile(), ==, 0)
    DREAM3D_REQUIRED(reader.getNumberOfElements(), ==, 4)
  }
  SECTION("ZCells 1")
  {
    CtfReader reader;
    reader.setFileName(WriteCtfFile("ValidOneZCell", CtfGridLines("0.5", "0.5", "ZCells\t1\n")));
    DREAM3D_REQUIRED(reader.readFile(), ==, 0)
    DREAM3D_REQUIRED(reader.getNumberOfElements(), ==, 4)
  }
  SECTION("ZCells 2")
  {
    CtfReader reader;
    reader.setFileName(WriteCtfFile("ValidTwoZCells", CtfGridLines("0.5", "0.5", "ZCells\t2\nZStep\t0.5\n"), 2));
    DREAM3D_REQUIRED(reader.readFile(), ==, 0)
    DREAM3D_REQUIRED(reader.getNumberOfElements(), ==, 8)
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::HklGridHeaderValidationTest::CtfInvalidGridIsAnError", "[EbsdLib][HklGridHeaderValidationTest]")
{
  std::string baseName;
  std::string gridLines;
  int numSlices = 1;
  int expectedError = 0;
  std::string expectedKey;

  // A 2D file with a bad step is also rejected later by the raster order check
  // (-101104), but a 3D file skips that check, so it is tested on its own.
  SECTION("Negative XStep, 3D")
  {
    baseName = "NegativeXStep3D";
    gridLines = CtfGridLines("-0.5", "0.5", "ZCells\t2\nZStep\t0.5\n");
    numSlices = 2;
    expectedError = -102310;
    expectedKey = "XStep";
  }
  SECTION("NaN YStep, 3D")
  {
    baseName = "NaNYStep3D";
    gridLines = CtfGridLines("0.5", "nan", "ZCells\t2\nZStep\t0.5\n");
    numSlices = 2;
    expectedError = -102311;
    expectedKey = "YStep";
  }
  SECTION("Negative XStep")
  {
    baseName = "NegativeXStep";
    gridLines = CtfGridLines("-0.5", "0.5", "");
    expectedError = -102310;
    expectedKey = "XStep";
  }
  SECTION("NaN XStep")
  {
    baseName = "NaNXStep";
    gridLines = CtfGridLines("nan", "0.5", "");
    expectedError = -102310;
    expectedKey = "XStep";
  }
  SECTION("Infinite XStep")
  {
    baseName = "InfXStep";
    gridLines = CtfGridLines("inf", "0.5", "");
    expectedError = -102310;
    expectedKey = "XStep";
  }
  SECTION("Negative YStep")
  {
    baseName = "NegativeYStep";
    gridLines = CtfGridLines("0.5", "-0.5", "");
    expectedError = -102311;
    expectedKey = "YStep";
  }
  SECTION("NaN YStep")
  {
    baseName = "NaNYStep";
    gridLines = CtfGridLines("0.5", "nan", "");
    expectedError = -102311;
    expectedKey = "YStep";
  }
  SECTION("ZCells 0")
  {
    baseName = "ZeroZCells";
    gridLines = CtfGridLines("0.5", "0.5", "ZCells\t0\n");
    expectedError = -102312;
    expectedKey = "ZCells";
  }
  SECTION("ZCells -2")
  {
    baseName = "NegativeZCells";
    gridLines = CtfGridLines("0.5", "0.5", "ZCells\t-2\n");
    expectedError = -102312;
    expectedKey = "ZCells";
  }

  const std::string filePath = WriteCtfFile(baseName, gridLines, numSlices);
  CtfReader reader;
  reader.setFileName(filePath);
  const int err = reader.readFile();
  DREAM3D_REQUIRED(err, ==, expectedError)
  DREAM3D_REQUIRED(reader.getErrorCode(), ==, expectedError)
  const std::string message = reader.getErrorMessage();
  DREAM3D_REQUIRE(message.find(filePath) != std::string::npos)
  DREAM3D_REQUIRE(message.find(expectedKey) != std::string::npos)
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::HklGridHeaderValidationTest::CprValidGrid", "[EbsdLib][HklGridHeaderValidationTest]")
{
  CprReader reader;
  reader.setFileName(WriteCprFile("ValidGrid", "0.5", "0.5", "2", "2"));
  DREAM3D_REQUIRED(reader.readFile(), ==, 0)
  DREAM3D_REQUIRED(reader.getNumberOfElements(), ==, 4)
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::HklGridHeaderValidationTest::CprInvalidGridIsAnError", "[EbsdLib][HklGridHeaderValidationTest]")
{
  std::string baseName;
  std::array<std::string, 4> grid = {"0.5", "0.5", "2", "2"};
  int expectedError = 0;
  std::string expectedKey;

  SECTION("Negative GridDistX")
  {
    baseName = "NegativeGridDistX";
    grid[0] = "-0.5";
    expectedError = -102320;
    expectedKey = "GridDistX";
  }
  SECTION("NaN GridDistX")
  {
    baseName = "NaNGridDistX";
    grid[0] = "nan";
    expectedError = -102320;
    expectedKey = "GridDistX";
  }
  SECTION("Negative GridDistY")
  {
    baseName = "NegativeGridDistY";
    grid[1] = "-0.5";
    expectedError = -102321;
    expectedKey = "GridDistY";
  }
  SECTION("NaN GridDistY")
  {
    baseName = "NaNGridDistY";
    grid[1] = "nan";
    expectedError = -102321;
    expectedKey = "GridDistY";
  }
  SECTION("Negative xCells and yCells")
  {
    // -1 * -1 is a 1 point grid, so the .crc file is long enough to be read.
    baseName = "NegativeXYCells";
    grid[2] = "-1";
    grid[3] = "-1";
    expectedError = -102322;
    expectedKey = "xCells";
  }
  SECTION("Negative yCells")
  {
    baseName = "NegativeYCells";
    grid[3] = "-1";
    expectedError = -102323;
    expectedKey = "yCells";
  }

  const std::string filePath = WriteCprFile(baseName, grid[0], grid[1], grid[2], grid[3]);
  CprReader reader;
  reader.setFileName(filePath);
  const int err = reader.readFile();
  DREAM3D_REQUIRED(err, ==, expectedError)
  DREAM3D_REQUIRED(reader.getErrorCode(), ==, expectedError)
  const std::string message = reader.getErrorMessage();
  DREAM3D_REQUIRE(message.find(filePath) != std::string::npos)
  DREAM3D_REQUIRE(message.find(expectedKey) != std::string::npos)
}
