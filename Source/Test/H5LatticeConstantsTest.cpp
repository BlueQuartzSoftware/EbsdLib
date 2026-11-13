#include <catch2/catch.hpp>

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/IO/BrukerNano/EspritConstants.h"
#include "EbsdLib/IO/BrukerNano/H5EspritReader.h"
#include "EbsdLib/IO/H5EbsdVolumeInfo.h"
#include "EbsdLib/IO/HKL/CtfConstants.h"
#include "EbsdLib/IO/HKL/H5CtfImporter.h"
#include "EbsdLib/IO/HKL/H5CtfReader.h"
#include "EbsdLib/IO/HKL/H5CtfVolumeReader.h"
#include "EbsdLib/IO/TSL/AngConstants.h"
#include "EbsdLib/IO/TSL/H5AngImporter.h"
#include "EbsdLib/IO/TSL/H5AngReader.h"
#include "EbsdLib/IO/TSL/H5AngVolumeReader.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include <filesystem>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{
const std::filesystem::path k_OutputDir = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / "H5LatticeConstantsTest";

/**
 * @brief Replaces the lattice constants dataset at datasetPath with one that holds the given values.
 */
void ReplaceLatticeConstants(const std::filesystem::path& filePath, const std::string& datasetPath, const std::vector<float>& values)
{
  const hid_t fileId = H5Fopen(filePath.string().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  REQUIRE(fileId >= 0);
  H5Support::H5ScopedFileSentinel sentinel(fileId, true);
  REQUIRE(H5Lexists(fileId, datasetPath.c_str(), H5P_DEFAULT) > 0);
  REQUIRE(H5Ldelete(fileId, datasetPath.c_str(), H5P_DEFAULT) >= 0);
  const std::vector<hsize_t> dims = {static_cast<hsize_t>(values.size())};
  REQUIRE(H5Support::H5Lite::writePointerDataset(fileId, datasetPath, static_cast<int32_t>(dims.size()), dims.data(), values.data()) >= 0);
}

/**
 * @brief Imports one scan file into slice 1 of a new .h5ebsd file.
 */
void ImportSlice(EbsdImporter& importer, const std::filesystem::path& outputPath, const std::string& inputFile)
{
  std::filesystem::create_directories(outputPath.parent_path());
  std::filesystem::remove(outputPath);
  const hid_t fileId = H5Support::H5Utilities::createFile(outputPath.string());
  REQUIRE(fileId >= 0);
  H5Support::H5ScopedFileSentinel sentinel(fileId, true);
  REQUIRE(importer.importFile(fileId, 1, inputFile) >= 0);
}

void CheckLatticeMessage(const std::string& message, const std::string& className, const std::filesystem::path& filePath, size_t count)
{
  INFO(message);
  REQUIRE(message.find(className + " Error: The 'LatticeConstants' dataset of phase '1' in file '" + filePath.string() + "' holds " + std::to_string(count) +
                       " values but 6 (a, b, c, alpha, beta, gamma) are required.") != std::string::npos);
}
/**
 * @brief Writes the volume header datasets that H5EbsdVolumeInfo::readVolumeInfo() needs for a file with one slice at index 1.
 */
void WriteVolumeHeader(const std::filesystem::path& filePath, const std::string& manufacturer)
{
  using namespace H5Support;
  const hid_t fileId = H5Fopen(filePath.string().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  REQUIRE(fileId >= 0);
  H5ScopedFileSentinel sentinel(fileId, true);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::ZStartIndex, int32_t{1}) >= 0);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::ZEndIndex, int32_t{1}) >= 0);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::XPoints, int64_t{1}) >= 0);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::YPoints, int64_t{1}) >= 0);
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
  REQUIRE(H5Lite::writeStringDataset(fileId, H5Ebsd::Manufacturer, manufacturer) >= 0);
}
} // namespace

TEST_CASE("ebsdlib::H5LatticeConstantsTest::H5AngReader", "[EbsdLib][H5LatticeConstantsTest]")
{
  const std::filesystem::path outputPath = k_OutputDir / "LatticeConstants_Ang.h5ebsd";
  ImportSlice(*H5AngImporter::New(), outputPath, ebsdlib::unit_test::AngImportTest::TestFile1);
  const std::vector<float> latticeConstants =
      GENERATE(std::vector<float>{3.5f, 3.5f}, std::vector<float>{3.5f, 3.5f, 3.5f, 90.0f, 90.0f, 90.0f, 1.0f}, std::vector<float>{3.5f, 3.6f, 3.7f, 90.0f, 91.0f, 92.0f});
  ReplaceLatticeConstants(outputPath, "1/Header/Phases/1/" + ebsdlib::Ang::LatticeConstants, latticeConstants);

  H5AngReader::Pointer reader = H5AngReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path("1");
  const int err = reader->readHeaderOnly();
  INFO("Phase lattice constants count: " << (reader->getPhases().empty() ? 0 : reader->getPhases()[0]->getLatticeConstants().size()));
  if(latticeConstants.size() == 6)
  {
    REQUIRE(err >= 0);
    REQUIRE_FALSE(reader->getPhases().empty());
    REQUIRE(reader->getPhases()[0]->getLatticeConstants() == latticeConstants);
  }
  else
  {
    REQUIRE(err < 0);
    REQUIRE(reader->getErrorCode() == -90350);
    CheckLatticeMessage(reader->getErrorMessage(), "H5AngReader", outputPath, latticeConstants.size());
  }
  reader = H5AngReader::NullPointer();
  std::filesystem::remove(outputPath);
}

TEST_CASE("ebsdlib::H5LatticeConstantsTest::H5CtfReader", "[EbsdLib][H5LatticeConstantsTest]")
{
  const std::filesystem::path outputPath = k_OutputDir / "LatticeConstants_Ctf.h5ebsd";
  ImportSlice(*H5CtfImporter::New(), outputPath, ebsdlib::unit_test::CtfReaderTest::USInputFile1);
  const std::vector<float> latticeConstants =
      GENERATE(std::vector<float>{3.5f, 3.5f}, std::vector<float>{3.5f, 3.5f, 3.5f, 90.0f, 90.0f, 90.0f, 1.0f}, std::vector<float>{3.5f, 3.6f, 3.7f, 90.0f, 91.0f, 92.0f});
  ReplaceLatticeConstants(outputPath, "1/Header/Phases/1/" + ebsdlib::Ctf::LatticeConstants, latticeConstants);

  H5CtfReader::Pointer reader = H5CtfReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path("1");
  const int err = reader->readHeaderOnly();
  INFO("Phase lattice constants count: " << (reader->getPhases().empty() ? 0 : reader->getPhases()[0]->getLatticeConstants().size()));
  if(latticeConstants.size() == 6)
  {
    REQUIRE(err >= 0);
    REQUIRE_FALSE(reader->getPhases().empty());
    REQUIRE(reader->getPhases()[0]->getLatticeConstants() == latticeConstants);
  }
  else
  {
    REQUIRE(err < 0);
    REQUIRE(reader->getErrorCode() == -90350);
    CheckLatticeMessage(reader->getErrorMessage(), "H5CtfReader", outputPath, latticeConstants.size());
  }
  reader = H5CtfReader::NullPointer();
  std::filesystem::remove(outputPath);
}

TEST_CASE("ebsdlib::H5LatticeConstantsTest::H5EspritReader", "[EbsdLib][H5LatticeConstantsTest]")
{
  const std::filesystem::path outputPath = k_OutputDir / "LatticeConstants_Esprit.h5";
  std::filesystem::create_directories(outputPath.parent_path());
  std::filesystem::copy_file(ebsdlib::unit_test::H5EspritReaderTest::InputFile, outputPath, std::filesystem::copy_options::overwrite_existing);
  const std::vector<float> latticeConstants =
      GENERATE(std::vector<float>{3.5f, 3.5f}, std::vector<float>{3.5f, 3.5f, 3.5f, 90.0f, 90.0f, 90.0f, 1.0f}, std::vector<float>{3.5f, 3.6f, 3.7f, 90.0f, 91.0f, 92.0f});
  ReplaceLatticeConstants(outputPath, "Section_435/EBSD/Header/Phases/1/" + ebsdlib::H5Esprit::LatticeConstants, latticeConstants);

  H5EspritReader::Pointer reader = H5EspritReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path("Section_435");
  const int err = reader->readHeaderOnly();
  INFO("Phase lattice constants count: " << (reader->getPhaseVector().empty() ? 0 : reader->getPhaseVector()[0]->getLatticeConstants().size()));
  if(latticeConstants.size() == 6)
  {
    REQUIRE(err >= 0);
    REQUIRE_FALSE(reader->getPhaseVector().empty());
    REQUIRE(reader->getPhaseVector()[0]->getLatticeConstants() == latticeConstants);
  }
  else
  {
    REQUIRE(err < 0);
    REQUIRE(reader->getErrorCode() == -90350);
    CheckLatticeMessage(reader->getErrorMessage(), "H5EspritReader", outputPath, latticeConstants.size());
  }
  reader = H5EspritReader::NullPointer();
  std::filesystem::remove(outputPath);
}

TEST_CASE("ebsdlib::H5LatticeConstantsTest::VolumeReaderGetPhases", "[EbsdLib][H5LatticeConstantsTest]")
{
  const bool isCtf = GENERATE(false, true);
  const std::filesystem::path outputPath = k_OutputDir / (isCtf ? "LatticeConstants_CtfVolume.h5ebsd" : "LatticeConstants_AngVolume.h5ebsd");
  if(isCtf)
  {
    ImportSlice(*H5CtfImporter::New(), outputPath, ebsdlib::unit_test::CtfReaderTest::USInputFile1);
    ReplaceLatticeConstants(outputPath, "1/Header/Phases/1/" + ebsdlib::Ctf::LatticeConstants, {3.5f, 3.5f});
    WriteVolumeHeader(outputPath, "HKL");
  }
  else
  {
    ImportSlice(*H5AngImporter::New(), outputPath, ebsdlib::unit_test::AngImportTest::TestFile1);
    ReplaceLatticeConstants(outputPath, "1/Header/Phases/1/" + ebsdlib::Ang::LatticeConstants, {3.5f, 3.5f});
    WriteVolumeHeader(outputPath, "TSL");
  }

  // The volume reader must pass the slice reader's error on, so a caller that gets no phases can report why.
  H5EbsdVolumeReader::Pointer reader = isCtf ? std::static_pointer_cast<H5EbsdVolumeReader>(H5CtfVolumeReader::New()) : std::static_pointer_cast<H5EbsdVolumeReader>(H5AngVolumeReader::New());
  reader->setFileName(outputPath.string());
  REQUIRE(reader->readVolumeInfo() >= 0);
  const size_t phaseCount = isCtf ? std::dynamic_pointer_cast<H5CtfVolumeReader>(reader)->getPhases().size() : std::dynamic_pointer_cast<H5AngVolumeReader>(reader)->getPhases().size();
  CHECK(phaseCount == 0);
  CHECK(reader->getErrorCode() == -90350);
  CheckLatticeMessage(reader->getErrorMessage(), isCtf ? "H5CtfReader" : "H5AngReader", outputPath, 2);
  reader = H5EbsdVolumeReader::NullPointer();
  std::filesystem::remove(outputPath);
}
