#include <catch2/catch.hpp>

#include "EbsdLib/IO/HKL/CprReader.h"

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <string>

using namespace ebsdlib;

namespace
{
const std::string k_PhasesSection = R"([Phases]
Count=1
[Phase1]
StructureName=Iron
Reference=Test
a=2.87
b=2.87
c=2.87
alpha=90
beta=90
gamma=90
LaueGroup=11
SpaceGroup=229
ID1=1
ID2=2
)";

/**
 * @brief Writes a 2x2 .cpr/.crc pair with the phi1, Phi and phi2 fields. The
 * phases section is passed in so that a test can remove or change it.
 * @return The path to the .cpr file
 */
std::string WriteCprFile(const std::string& baseName, const std::string& phasesSection)
{
  const std::filesystem::path dir = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / "CprReaderTest";
  std::filesystem::create_directories(dir);
  const std::filesystem::path cprPath = dir / (baseName + ".cpr");
  {
    std::ofstream out(cprPath, std::ios::out | std::ios::trunc);
    out << "[General]\nAuthor=Test\nJobMode=Grid\n";
    out << "[Job]\nMagnification=200\nCoverage=100\nDevice=0\nkV=20\nTiltAngle=70\nTiltAxis=0\n";
    out << "GridDistX=0.5\nGridDistY=0.5\nxCells=2\nyCells=2\n";
    out << phasesSection;
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
TEST_CASE("ebsdlib::CprReaderTest::ValidFileReadsPhases", "[EbsdLib][CprReaderTest]")
{
  CprReader reader;
  reader.setFileName(WriteCprFile("ValidPhases", k_PhasesSection));
  DREAM3D_REQUIRED(reader.readFile(), ==, 0)
  DREAM3D_REQUIRED(reader.getPhaseVector().size(), ==, 1)
  DREAM3D_REQUIRED(reader.getNumberOfElements(), ==, 4)
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CprReaderTest::MissingOrInvalidPhaseCountIsAnError", "[EbsdLib][CprReaderTest]")
{
  std::string baseName;
  std::string phasesSection;

  SECTION("No [Phases] section")
  {
    baseName = "NoPhasesSection";
  }
  SECTION("No Count key")
  {
    baseName = "NoPhaseCount";
    phasesSection = "[Phases]\n";
  }
  SECTION("Count=0")
  {
    baseName = "ZeroPhaseCount";
    phasesSection = "[Phases]\nCount=0\n";
  }
  SECTION("Count=-2")
  {
    baseName = "NegativePhaseCount";
    phasesSection = "[Phases]\nCount=-2\n";
  }
  SECTION("Count=abc")
  {
    baseName = "TextPhaseCount";
    phasesSection = "[Phases]\nCount=abc\n";
  }

  const std::string filePath = WriteCprFile(baseName, phasesSection);

  CprReader headerReader;
  headerReader.setFileName(filePath);
  const int headerErr = headerReader.readHeaderOnly();
  DREAM3D_REQUIRED(headerErr, ==, -102300)
  DREAM3D_REQUIRED(headerReader.getErrorCode(), ==, -102300)
  DREAM3D_REQUIRE(headerReader.getErrorMessage().find(filePath) != std::string::npos)

  CprReader reader;
  reader.setFileName(filePath);
  const int err = reader.readFile();
  DREAM3D_REQUIRED(err, ==, -102300)
  DREAM3D_REQUIRED(reader.getErrorCode(), ==, -102300)
  DREAM3D_REQUIRE(reader.getErrorMessage().find(filePath) != std::string::npos)
  DREAM3D_REQUIRE(reader.getPhaseVector().empty())
}
