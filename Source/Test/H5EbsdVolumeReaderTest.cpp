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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <catch2/catch.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/H5CtfImporter.h"
#include "EbsdLib/IO/HKL/H5CtfVolumeReader.h"
#include "EbsdLib/IO/TSL/H5AngImporter.h"
#include "EbsdLib/IO/TSL/H5AngVolumeReader.h"
#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

using namespace ebsdlib;

namespace
{
constexpr int k_AngSliceDimensionMismatch = -90050;
constexpr int k_CtfSliceDimensionMismatch = -77001;

struct ScopedFileRemover
{
  std::vector<std::string> paths;
  ~ScopedFileRemover()
  {
    for(const auto& path : paths)
    {
      std::error_code error;
      std::filesystem::remove(path, error);
    }
  }
};

/**
 * @brief Each cell stores a value that names its slice and cell: slice * 1000 + cell index.
 */
int32_t cellTag(int32_t slice, int32_t cell)
{
  return slice * 1000 + cell;
}

void writeAngSlice(const std::string& path, int32_t slice, int32_t nCols, int32_t nRows)
{
  std::ofstream out(path);
  out << "# TEM_PIXperUM          1.000000\n"
      << "# x-star                0.372300\n"
      << "# y-star                0.689300\n"
      << "# z-star                0.970100\n"
      << "# WorkingDistance       5.000000\n"
      << "#\n"
      << "# Phase 1\n"
      << "# MaterialName  \tNickel\n"
      << "# Formula     \tNi\n"
      << "# Info\t\t\n"
      << "# Symmetry              43\n"
      << "# LatticeConstants      3.520 3.520 3.520  90.000  90.000  90.000\n"
      << "# NumberFamilies        1\n"
      << "# hklFamilies   \t 1  1  1 1 0.000000\n"
      << "# Categories 0 0 0 0 0 \n"
      << "#\n"
      << "# GRID: SqrGrid\n"
      << "# XSTEP: 0.250000\n"
      << "# YSTEP: 0.250000\n"
      << "# NCOLS_ODD: " << nCols << "\n"
      << "# NCOLS_EVEN: " << nCols << "\n"
      << "# NROWS: " << nRows << "\n"
      << "#\n"
      << "# OPERATOR: \tAdministrator\n"
      << "#\n"
      << "# SAMPLEID: \t\n"
      << "#\n"
      << "# SCANID: \t\n"
      << "#\n";
  for(int32_t j = 0; j < nRows; j++)
  {
    for(int32_t i = 0; i < nCols; i++)
    {
      out << " 0.10000 0.20000 0.30000 " << i * 0.25f << " " << j * 0.25f << " " << cellTag(slice, j * nCols + i) << " 1.000 1 0 0.500\n";
    }
  }
}

void writeCtfSlice(const std::string& path, int32_t slice, int32_t nCols, int32_t nRows)
{
  std::ofstream out(path);
  out << "Channel Text File\n"
      << "Prj\tH5EbsdVolumeReaderTest\n"
      << "Author\t[Unknown]\n"
      << "JobMode\tGrid\n"
      << "XCells\t" << nCols << "\n"
      << "YCells\t" << nRows << "\n"
      << "XStep\t0.5\n"
      << "YStep\t0.5\n"
      << "AcqE1\t0\n"
      << "AcqE2\t0\n"
      << "AcqE3\t0\n"
      << "Euler angles refer to Sample Coordinate system (CS0)!\tMag\t200\tCoverage\t100\tDevice\t0\tKV\t15\tTiltAngle\t70\tTiltAxis\t0\n"
      << "Phases\t1\n"
      << "3.520;3.520;3.520\t90;90;90\tNickel\t11\t0\t0_5.0.6.0\t-67395467\t[Ni.cry]\n"
      << "Phase\tX\tY\tBands\tError\tEuler1\tEuler2\tEuler3\tMAD\tBC\tBS\n";
  for(int32_t j = 0; j < nRows; j++)
  {
    for(int32_t i = 0; i < nCols; i++)
    {
      out << "1\t" << i * 0.5f << "\t" << j * 0.5f << "\t8\t0\t10.0\t20.0\t30.0\t0.5\t" << cellTag(slice, j * nCols + i) << "\t0\n";
    }
  }
}

/**
 * @brief Writes a three-slice .h5ebsd file whose header states volumeX x volumeY cells per slice.
 * Slices 0 and 2 match the header; slice 1 has middleX x middleY cells.
 */
void writeVolume(const std::string& manufacturer, const std::string& h5Path, ScopedFileRemover& remover, int32_t volumeX, int32_t volumeY, int32_t middleX, int32_t middleY)
{
  const bool isAng = manufacturer == ebsdlib::Ang::Manufacturer;
  const hid_t fileId = H5Support::H5Utilities::createFile(h5Path);
  REQUIRE(fileId >= 0);
  H5Support::H5ScopedFileSentinel sentinel(fileId, true);

  EbsdImporter::Pointer importer = isAng ? std::static_pointer_cast<EbsdImporter>(H5AngImporter::New()) : std::static_pointer_cast<EbsdImporter>(H5CtfImporter::New());
  for(int32_t slice = 0; slice < 3; slice++)
  {
    const int32_t nCols = slice == 1 ? middleX : volumeX;
    const int32_t nRows = slice == 1 ? middleY : volumeY;
    const std::string slicePath = h5Path + "_slice" + std::to_string(slice) + (isAng ? ".ang" : ".ctf");
    remover.paths.push_back(slicePath);
    if(isAng)
    {
      writeAngSlice(slicePath, slice, nCols, nRows);
    }
    else
    {
      writeCtfSlice(slicePath, slice, nCols, nRows);
    }
    REQUIRE(importer->importFile(fileId, slice, slicePath) >= 0);
  }

  using namespace H5Support;
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::ZStartIndex, int64_t{0}) >= 0);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::ZEndIndex, int64_t{2}) >= 0);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::XPoints, int64_t{volumeX}) >= 0);
  REQUIRE(H5Lite::writeScalarDataset(fileId, H5Ebsd::YPoints, int64_t{volumeY}) >= 0);
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
    REQUIRE(H5Lite::writeVectorDataset(fileId, key, std::vector<hsize_t>{3}, std::vector<float>{0.0f, 0.0f, 1.0f}) >= 0);
  }
  REQUIRE(H5Lite::writeStringDataset(fileId, H5Ebsd::Manufacturer, manufacturer) >= 0);
}

H5EbsdVolumeReader::Pointer createVolumeReader(const std::string& manufacturer, const std::string& h5Path)
{
  H5EbsdVolumeReader::Pointer reader;
  if(manufacturer == ebsdlib::Ang::Manufacturer)
  {
    reader = H5AngVolumeReader::New();
  }
  else
  {
    reader = H5CtfVolumeReader::New();
  }
  reader->setFileName(h5Path);
  reader->setSliceStart(0);
  reader->setSliceEnd(2);
  reader->readAllArrays(true);
  return reader;
}

/**
 * @brief Returns the tag array (ANG image quality or CTF band contrast) as integers.
 */
std::vector<int32_t> readTags(const std::string& manufacturer, H5EbsdVolumeReader& reader, size_t count)
{
  std::vector<int32_t> tags(count, -1);
  if(manufacturer == ebsdlib::Ang::Manufacturer)
  {
    const auto* iq = static_cast<const float*>(reader.getPointerByName(ebsdlib::Ang::ImageQuality));
    REQUIRE(iq != nullptr);
    for(size_t i = 0; i < count; i++)
    {
      tags[i] = static_cast<int32_t>(iq[i]);
    }
  }
  else
  {
    const auto* bc = static_cast<const int32_t*>(reader.getPointerByName(ebsdlib::Ctf::BC));
    REQUIRE(bc != nullptr);
    for(size_t i = 0; i < count; i++)
    {
      tags[i] = bc[i];
    }
  }
  return tags;
}
} // namespace

TEST_CASE("ebsdlib::H5EbsdVolumeReader-SliceLargerThanVolume", "[EbsdLib][H5EbsdVolumeReader]")
{
  const std::string manufacturer = GENERATE(std::string(ebsdlib::Ang::Manufacturer), std::string(ebsdlib::Ctf::Manufacturer));
  const int expectedError = manufacturer == ebsdlib::Ang::Manufacturer ? k_AngSliceDimensionMismatch : k_CtfSliceDimensionMismatch;
  DYNAMIC_SECTION("Manufacturer " << manufacturer)
  {
    constexpr int32_t k_VolumeX = 6;
    constexpr int32_t k_VolumeY = 4;
    const std::string h5Path = unit_test::k_TestTempDir + "H5EbsdVolumeReader_LargeSlice_" + manufacturer + ".h5ebsd";
    ScopedFileRemover remover{{h5Path}};
    // Slice 1 has two more columns than the volume header states. The reader would start it at
    // column -1, so its first cell lands on the last cell of slice 0 and its rows wrap.
    writeVolume(manufacturer, h5Path, remover, k_VolumeX, k_VolumeY, k_VolumeX + 2, k_VolumeY);

    auto reader = createVolumeReader(manufacturer, h5Path);
    int64_t dims[3] = {0, 0, 0};
    REQUIRE(reader->getDims(dims[0], dims[1], dims[2]) >= 0);
    REQUIRE(dims[0] == k_VolumeX);
    REQUIRE(dims[1] == k_VolumeY);
    REQUIRE(dims[2] == 3);

    const int err = reader->loadData(dims[0], dims[1], dims[2], RefFrameZDir::LowtoHigh);

    // Slice 0 is read before the bad slice and must not be overwritten by it.
    const size_t sliceCells = static_cast<size_t>(k_VolumeX * k_VolumeY);
    const std::vector<int32_t> tags = readTags(manufacturer, *reader, sliceCells);
    for(size_t cell = 0; cell < sliceCells; cell++)
    {
      INFO("Slice 0 cell " << cell);
      CHECK(tags[cell] == cellTag(0, static_cast<int32_t>(cell)));
    }

    CHECK(err == expectedError);
    CHECK(reader->getErrorCode() == expectedError);
    const std::string message = reader->getErrorMessage();
    INFO("Error message: " << message);
    CHECK(message.find(h5Path) != std::string::npos);
    CHECK(message.find("Slice 1 ") != std::string::npos);
    CHECK(message.find("8 x 4") != std::string::npos);
    CHECK(message.find("6 x 4") != std::string::npos);
  }
}

TEST_CASE("ebsdlib::H5EbsdVolumeReader-SliceSmallerThanVolumeIsCentered", "[EbsdLib][H5EbsdVolumeReader]")
{
  // Converters such as simplnx EbsdToH5Ebsd write the largest slice size as the volume size, so a
  // smaller slice is valid. It is centered in the volume and the cells around it stay zero.
  const std::string manufacturer = GENERATE(std::string(ebsdlib::Ang::Manufacturer), std::string(ebsdlib::Ctf::Manufacturer));
  DYNAMIC_SECTION("Manufacturer " << manufacturer)
  {
    constexpr int32_t k_VolumeX = 6;
    constexpr int32_t k_VolumeY = 4;
    constexpr int32_t k_SliceX = 4;
    constexpr int32_t k_SliceY = 2;
    const std::string h5Path = unit_test::k_TestTempDir + "H5EbsdVolumeReader_SmallSlice_" + manufacturer + ".h5ebsd";
    ScopedFileRemover remover{{h5Path}};
    writeVolume(manufacturer, h5Path, remover, k_VolumeX, k_VolumeY, k_SliceX, k_SliceY);

    auto reader = createVolumeReader(manufacturer, h5Path);
    REQUIRE(reader->loadData(k_VolumeX, k_VolumeY, 3, RefFrameZDir::LowtoHigh) >= 0);

    const size_t sliceCells = static_cast<size_t>(k_VolumeX * k_VolumeY);
    const std::vector<int32_t> tags = readTags(manufacturer, *reader, 3 * sliceCells);
    for(int32_t z = 0; z < 3; z++)
    {
      for(int32_t y = 0; y < k_VolumeY; y++)
      {
        for(int32_t x = 0; x < k_VolumeX; x++)
        {
          int32_t expected = cellTag(z, y * k_VolumeX + x);
          if(z == 1)
          {
            const int32_t sx = x - (k_VolumeX - k_SliceX) / 2;
            const int32_t sy = y - (k_VolumeY - k_SliceY) / 2;
            const bool inside = sx >= 0 && sx < k_SliceX && sy >= 0 && sy < k_SliceY;
            expected = inside ? cellTag(1, sy * k_SliceX + sx) : 0;
          }
          INFO("z " << z << " y " << y << " x " << x);
          CHECK(tags[static_cast<size_t>(z) * sliceCells + static_cast<size_t>(y * k_VolumeX + x)] == expected);
        }
      }
    }
  }
}
