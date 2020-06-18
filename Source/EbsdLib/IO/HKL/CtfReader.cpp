/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
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
 * The code contained herein was partially funded by the followig contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "CtfReader.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "CtfPhase.h"
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

//#define PI_OVER_2f       90.0f
//#define THREE_PI_OVER_2f 270.0f
//#define TWO_PIf          360.0f
//#define ONE_PIf          180.0f

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
CtfReader::CtfReader()
{

  // Initialize the map of header key to header value
  m_HeaderMap[EbsdLib::Ctf::ChannelTextFile] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ctf::ChannelTextFile);
  m_HeaderMap[EbsdLib::Ctf::Prj] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ctf::Prj);
  m_HeaderMap[EbsdLib::Ctf::Author] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ctf::Author);
  m_HeaderMap[EbsdLib::Ctf::JobMode] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ctf::JobMode);
  m_HeaderMap[EbsdLib::Ctf::XCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::XCells);
  m_HeaderMap[EbsdLib::Ctf::YCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::YCells);
  m_HeaderMap[EbsdLib::Ctf::ZCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::ZCells);
  m_HeaderMap[EbsdLib::Ctf::XStep] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::XStep);
  m_HeaderMap[EbsdLib::Ctf::YStep] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::YStep);
  m_HeaderMap[EbsdLib::Ctf::ZStep] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::ZStep);
  m_HeaderMap[EbsdLib::Ctf::AcqE1] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::AcqE1);
  m_HeaderMap[EbsdLib::Ctf::AcqE2] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::AcqE2);
  m_HeaderMap[EbsdLib::Ctf::AcqE3] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::AcqE3);
  m_HeaderMap[EbsdLib::Ctf::Euler] = CtfStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ctf::Euler);
  m_HeaderMap[EbsdLib::Ctf::Mag] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::Mag);
  m_HeaderMap[EbsdLib::Ctf::Coverage] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::Coverage);
  m_HeaderMap[EbsdLib::Ctf::Device] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::Device);
  m_HeaderMap[EbsdLib::Ctf::KV] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::KV);
  m_HeaderMap[EbsdLib::Ctf::TiltAngle] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::TiltAngle);
  m_HeaderMap[EbsdLib::Ctf::TiltAxis] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::TiltAxis);
  m_HeaderMap[EbsdLib::Ctf::NumPhases] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(EbsdLib::Ctf::NumPhases);

  setXCells(0);
  setYCells(0);
  setZCells(1);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
CtfReader::~CtfReader() = default;

//// -----------------------------------------------------------------------------
////
//// -----------------------------------------------------------------------------
// void CtfReader::setPointerByName(const std::string& name, void* p)
//{
//  // First we need to see if the pointer already exists
//  std::map<std::string, DataParser::Pointer>::iterator iter = m_NamePointerMap.find(name);
//  if (iter == m_NamePointerMap.end())
//  {
//    // Data does not exist in Map
//    DataParser::Pointer dparser = getParser(name, nullptr, getXCells() * getYCells());
//    dparser->setVoidPointer(p);
//    m_NamePointerMap[name] = dparser;
//  }
//  else
//  {
//    DataParser::Pointer dparser = m_NamePointerMap[name];
//    void* ptr = dparser->getVoidPointer();
//    deallocateArrayData(ptr);
//    dparser->setVoidPointer(p);
//  }
//}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void* CtfReader::getPointerByName(const std::string& featureName)
{
  void* ptr = nullptr;
  if(m_NamePointerMap.find(featureName) != m_NamePointerMap.end())
  {
    ptr = m_NamePointerMap[featureName]->getVoidPointer();
  }
  return ptr;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::NumericTypes::Type CtfReader::getPointerType(const std::string& featureName)
{
  // std::cout << "featureName: " << featureName << std::endl;
  if(featureName == EbsdLib::Ctf::Phase)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::X)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::Y)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::Z)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::Bands)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::Error)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::Euler1)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::Euler2)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::Euler3)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::MAD)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ctf::BC)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::BS)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::GrainIndex)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourR)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourG)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourB)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  // std::cout << "THIS IS NOT GOOD. Featurename: " << featureName << " was not found in the list" << std::endl;
  return EbsdLib::NumericTypes::Type::UnknownNumType;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::getTypeSize(const std::string& featureName)
{
  if(featureName == EbsdLib::Ctf::Phase)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::X)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Y)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Z)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Bands)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Error)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Euler1)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Euler2)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::Euler3)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::MAD)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::BC)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::BS)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::GrainIndex)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourR)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourG)
  {
    return 4;
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourB)
  {
    return 4;
  }
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DataParser::Pointer CtfReader::getParser(const std::string& featureName, void* ptr, size_t size)
{
  // These are defaulted to a "3D" CTF file with Feature IDS already determined and their colors
  if(featureName == EbsdLib::Ctf::Phase)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 0);
  }
  if(featureName == EbsdLib::Ctf::X)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 1);
  }
  if(featureName == EbsdLib::Ctf::Y)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 2);
  }
  if(featureName == EbsdLib::Ctf::Z)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 3);
  }
  if(featureName == EbsdLib::Ctf::Bands)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 4);
  }
  if(featureName == EbsdLib::Ctf::Error)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 5);
  }
  if(featureName == EbsdLib::Ctf::Euler1)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 6);
  }
  if(featureName == EbsdLib::Ctf::Euler2)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 7);
  }
  if(featureName == EbsdLib::Ctf::Euler3)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 8);
  }
  if(featureName == EbsdLib::Ctf::MAD)
  {
    return FloatParser::New(static_cast<float*>(ptr), size, featureName, 9);
  }
  if(featureName == EbsdLib::Ctf::BC)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 10);
  }
  if(featureName == EbsdLib::Ctf::BS)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 11);
  }
  if(featureName == EbsdLib::Ctf::GrainIndex)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 12);
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourR)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 13);
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourG)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 14);
  }
  if(featureName == EbsdLib::Ctf::GrainRandomColourB)
  {
    return Int32Parser::New(static_cast<int32_t*>(ptr), size, featureName, 15);
  }
  return DataParser::NullPointer();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::readHeaderOnly()
{
  int err = 1;
  std::string buf;
  std::ifstream in(getFileName(), std::ios_base::in);
  setHeaderIsComplete(false);
  if(!in.is_open())
  {
    std::string msg = std::string("Ctf file could not be opened: ") + getFileName();
    setErrorCode(-100);
    setErrorMessage(msg);
    return -100;
  }

  std::string origHeader;
  setOriginalHeader(origHeader);
  m_PhaseVector.clear();

  // Parse the header
  std::vector<std::string> headerLines;
  err = getHeaderLines(in, headerLines);
  err = parseHeaderLines(headerLines);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::readFile()
{
  int err = 1;
  setErrorCode(0);
  setErrorMessage("");
  std::string buf;
  std::ifstream in(getFileName(), std::ios_base::in);
  setHeaderIsComplete(false);
  if(!in.is_open())
  {
    std::string msg = std::string("Ctf file could not be opened: ") + getFileName();
    setErrorCode(-100);
    setErrorMessage(msg);
    return -100;
  }
  std::string origHeader;
  setOriginalHeader(origHeader);
  m_PhaseVector.clear();

  // Parse the header
  std::vector<std::string> headerLines;
  err = getHeaderLines(in, headerLines);
  if(err < 0)
  {
    return err;
  }
  err = parseHeaderLines(headerLines);
  if(err < 0)
  {
    return err;
  }

  if(getXStep() == 0.0 || getYStep() == 0.0f)
  {
    setErrorMessage("Either the X Step or Y Step was Zero (0.0) which is NOT allowed. Please update the CTF file header with appropriate values.");
    return -102;
  }

  if(getXCells() == 0 || getYCells() == 0)
  {
    setErrorMessage("Either the X Cells or Y Cells was Zero (0) which is NOT allowed. Please update the CTF file header with appropriate values.");
    return -103;
  }

  err = readData(in);

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfReader::readOnlySliceIndex(int slice)
{
  m_SingleSliceRead = slice;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::readData(std::ifstream& in)
{
  std::string sBuf;
  std::stringstream ss(sBuf);
  // Initialize new pointers
  int32_t xCells = getXCells();
  if(xCells < 0)
  {
    setErrorCode(-110);
    std::string msg;
    std::stringstream ss(msg);
    ss << "The number of X Cells was reported as " << xCells << ". This value must be larger than ZERO. This error can be caused by "
       << " a missing X Cells header value, an incorrect  XCells value or a value of X Cells larger than 2^31.\n";
    setErrorMessage(msg);
    return -110;
  }
  int32_t yCells = getYCells();
  if(yCells < 0)
  {
    setErrorCode(-111);
    std::string msg;
    std::stringstream ss(msg);
    ss << "The number of Y Cells was reported as " << yCells << ". This value must be larger than ZERO. This error can be caused by "
       << " a missing Y Cells header value, an incorrect Y Cells value or a value of Y Cells larger than 2^31.\n";
    setErrorMessage(msg);
    return -111;
  }

  int32_t zCells = getZCells();
  int32_t zStart = 0;
  int32_t zEnd = zCells;

  if(zCells < 0 || m_SingleSliceRead >= 0)
  {
    zCells = 1;
  }
  size_t totalScanPoints = static_cast<size_t>(yCells * xCells * zCells);

  setNumberOfElements(totalScanPoints);

  std::string buf;

  // Read the column Headers and allocate the necessary arrays
  std::getline(in, buf);
  std::string originalHeader = getOriginalHeader();
  originalHeader = originalHeader + buf;
  setOriginalHeader(originalHeader);
  buf = EbsdStringUtils::trimmed(buf); // Remove leading and trailing whitespace

  EbsdStringUtils::StringTokenType tokens = EbsdStringUtils::split(buf, '\t'); // Tokenize the array with a tab

  EbsdLib::NumericTypes::Type pType = EbsdLib::NumericTypes::Type::UnknownNumType;
  int32_t size = tokens.size();
  bool didAllocate = false;
  for(int32_t i = 0; i < size; ++i)
  {
    std::string name = tokens[i];
    pType = getPointerType(name);
    if(EbsdLib::NumericTypes::Type::Int32 == pType)
    {
      Int32Parser::Pointer dparser = Int32Parser::New(nullptr, totalScanPoints, name, i);
      didAllocate = dparser->allocateArray(totalScanPoints);
      // Q_ASSERT_X(dparser->getVoidPointer() != nullptr, __FILE__, "Could not allocate memory for Integer data in CTF File.");
      if(didAllocate)
      {
        ::memset(dparser->getVoidPointer(), 0xAB, sizeof(int32_t) * totalScanPoints);
        m_NamePointerMap[name] = dparser;
      }
    }
    else if(EbsdLib::NumericTypes::Type::Float == pType)
    {
      FloatParser::Pointer dparser = FloatParser::New(nullptr, totalScanPoints, name, i);
      didAllocate = dparser->allocateArray(totalScanPoints);
      // Q_ASSERT_X(dparser->getVoidPointer() != nullptr, __FILE__, "Could not allocate memory for Integer data in CTF File.");
      if(didAllocate)
      {
        ::memset(dparser->getVoidPointer(), 0xAB, sizeof(float) * totalScanPoints);
        m_NamePointerMap[name] = dparser;
      }
    }
    else
    {
      sBuf.clear();
      ss << "Column Header '" << tokens[i] << "' is not a recognized column for CTF Files. Please recheck your .ctf file and report this error to the DREAM3D developers.";
      setErrorMessage(sBuf);
      return -107;
    }

    if(!didAllocate)
    {
      setErrorCode(-106);
      std::string msg;
      std::stringstream ss(msg);
      ss << "The CTF reader could not allocate memory for the data. Check the header for the number of X, Y and Z Cells.";
      ss << "\n X Cells: " << getXCells();
      ss << "\n Y Cells: " << getYCells();
      ss << "\n Z Cells: " << getZCells();
      ss << "\n Total Scan Points: " << totalScanPoints;
      setErrorMessage(msg);
      return -106; // Could not allocate the memory
    }
  }

  // Now start reading the data line by line
  int err = 0;
  size_t counter = 0;
  for(int slice = zStart; slice < zEnd; ++slice)
  {
    for(size_t row = 0; row < yCells; ++row)
    {
      for(size_t col = 0; col < xCells; ++col)
      {
        std::getline(in, buf);               // Read the line into a std:::string including the newline
        buf = EbsdStringUtils::trimmed(buf); // Remove leading and trailing whitespace

        if((m_SingleSliceRead < 0) || (m_SingleSliceRead >= 0 && slice == m_SingleSliceRead))
        {

          if(in.eof() && buf.empty()) // We have to have read to the end of the file AND the buffer is empty
                                      // otherwise we read EXACTLY the last line and we still need to parse the line.
          {
            //  ++counter; // We need to make sure this gets incremented before leaving
            break;
          }
          err = parseDataLine(buf, row, col, counter, xCells, yCells);
          if(err < 0)
          {
            return err;
          }
          ++counter;
        }
      }
      if(in.eof())
      {
        break;
      }
    }
    //   std::cout << ".ctf Z Slice " << slice << " Reading complete." << std::endl;
    if(m_SingleSliceRead >= 0 && slice == m_SingleSliceRead)
    {
      break;
    }
  }

  if(counter != getNumberOfElements() && in.eof())
  {
    sBuf.clear();
    ss << "Premature End Of File reached.\n" << getFileName() << "\nNumRows=" << getNumberOfElements() << "\ncounter=" << counter << "\nTotal Data Points Read=" << counter << "\n";
    setErrorMessage(sBuf);
    setErrorCode(-105);
    return -105;
  }
  return 0;
}

#if 0
#define PRINT_HTML_TABLE_ROW(p)                                                                                                                                                                        \
  std::cout << "<tr>\n    <td>" << p->getKey() << "</td>\n    <td>" << p->getHDFType() << "</td>\n";                                                                                                   \
  std::cout << "    <td colspan=\"2\"> Contains value for the header entry " << p->getKey() << "</td>\n</tr>" << std::endl;
#else
#define PRINT_HTML_TABLE_ROW(p)
#endif

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::parseHeaderLines(std::vector<std::string>& headerLines)
{
  int err = 0;
  int32_t size = headerLines.size();
  for(int32_t i = 0; i < size; ++i)
  {
    std::string line = headerLines[i];
    EbsdStringUtils::StringTokenType tabTokens = EbsdStringUtils::split(line, '\t'); // Tab Delimit the line

    if(line.find("Prj") == 0) // This is a special case/bug in HKL's writing code. This line is space delimited
    {
      line = EbsdStringUtils::trimmed(line);
      line = EbsdStringUtils::simplified(line);

      std::string value = line.substr(4); // Get the value part of the line
      EbsdHeaderEntry::Pointer p = m_HeaderMap["Prj"];
      if(nullptr == p.get())
      {
        std::cout << "---------------------------" << std::endl;
        std::cout << "Could not find header entry for key 'Prj'" << std::endl;
      }
      else
      {
        p->parseValue(value);
        PRINT_HTML_TABLE_ROW(p)
      }
    }
    else if(line.find(EbsdLib::Ctf::NumPhases) == 0)
    {
      std::string key = tabTokens[0];
      EbsdHeaderEntry::Pointer p = m_HeaderMap[key];
      p->parseValue(tabTokens[1]);
      int nPhases = getNumPhases();
      // We start the Phase Index at "1" instead of Zero by convention
      for(int p = 1; p <= nPhases; ++p)
      {
        ++i; // Increment the outer loop
        line = headerLines[i];
        CtfPhase::Pointer phase = CtfPhase::New();
        phase->setPhaseIndex(p);
        phase->parsePhase(line); // All the phase information is on a single line

        m_PhaseVector.push_back(phase);
      }
      ++i;
    }
    else if(line.find("Euler angles refer to Sample Coordinate system (CS0)!") == 0)
    {
      // We parse out lots of stuff from this one line
      // Mag
      EbsdHeaderEntry::Pointer p0 = m_HeaderMap[tabTokens[1]];
      p0->parseValue(tabTokens[2]);
      PRINT_HTML_TABLE_ROW(p0);
      // Coverage
      EbsdHeaderEntry::Pointer p1 = m_HeaderMap[tabTokens[3]];
      p1->parseValue(tabTokens[4]);
      PRINT_HTML_TABLE_ROW(p1);
      // Device
      EbsdHeaderEntry::Pointer p2 = m_HeaderMap[tabTokens[5]];
      p2->parseValue(tabTokens[6]);
      PRINT_HTML_TABLE_ROW(p2);
      // KV
      EbsdHeaderEntry::Pointer p3 = m_HeaderMap[tabTokens[7]];
      p3->parseValue(tabTokens[8]);
      PRINT_HTML_TABLE_ROW(p3);
      // TiltAngle
      EbsdHeaderEntry::Pointer p4 = m_HeaderMap[tabTokens[9]];
      p4->parseValue(tabTokens[10]);
      PRINT_HTML_TABLE_ROW(p4);
      // TiltAxis
      EbsdHeaderEntry::Pointer p5 = m_HeaderMap[tabTokens[11]];
      p5->parseValue(tabTokens[12]);
      PRINT_HTML_TABLE_ROW(p5);
    }
    else if(line.find("Channel Text File") == 0 || line.find(":Channel Text File") == 0)
    {
      // We do not really do anything with this entry
    }
    else // This is the generic Catch all
    {
      EbsdHeaderEntry::Pointer p = m_HeaderMap[tabTokens[0]];
      if(nullptr == p.get())
      {
        std::cout << "---------------------------" << std::endl;
        std::cout << "Could not find header entry for key '" << line[0] << "'" << std::endl;
        //        std::string upper(line[0]);
        //        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        //        std::cout << "#define ANG_" << upper << "     \"" << line[0] << "\"" << std::endl;
        //        std::cout << "const std::string " << line[0] << "(ANG_" << upper << ");" << std::endl;
        //        std::cout << "angInstanceProperty(AngHeaderEntry<float>. float, " << line[0] << "EbsdLib::Ctf::" << line[0] << std::endl;
        //        std::cout << "m_Headermap[EbsdLib::Ctf::" << line[0] << "] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ctf::" << line[0] << ");" << std::endl;
      }
      else
      {
        if(line.size() > 1)
        {
          p->parseValue(tabTokens[1]);
          PRINT_HTML_TABLE_ROW(p)
        }
      }
    }
  }
  return err;
}

// -----------------------------------------------------------------------------
//  Read the data part of the .ctf file
// -----------------------------------------------------------------------------
int CtfReader::parseDataLine(std::string& line, size_t row, size_t col, size_t offset, size_t xCells, size_t yCells)
{
  /* When reading the data there should be at least 11 cols of data.
   */
  //  float x, y,  p1, p, p2, mad;
  //  int phase, bCount, error, bc, bs;
  //  size_t offset = i;

  // Filter the line to convert European command style decimals to US/UK style points
  //  std::vector<char> cLine(line.size()+1);
  //  ::memcpy( &(cLine.front()), line.c_str(), line.size() + 1);
  for(int c = 0; c < line.size(); ++c)
  {
    if(line.at(c) == ',')
    {
      line[c] = '.';
    }
  }

  EbsdStringUtils::StringTokenType tokens = EbsdStringUtils::split(line, '\t');
  if(tokens.size() != m_NamePointerMap.size())
  {
    setErrorCode(-107);
    std::string msg;
    std::stringstream ss(msg);
    ss << "The number of tab delimited data columns (" << tokens.size() << ") does not match the number of tab delimited header columns (";
    ss << m_NamePointerMap.size() << "). Please check the CTF file for mistakes.";
    ss << "The error occurred at data row " << row << " which is " << row << " past ";
    ss << "the column header row.";
    ss << "\nThe CTF Reader will now abort reading any further in the file.";

    setErrorMessage(msg);
    return -106; // Could not allocate the memory
  }

  for(const auto& iter : m_NamePointerMap)
  {
    DataParser::Pointer dparser = iter.second;
    dparser->parse(tokens[dparser->getColumnIndex()], offset);
  }
  return 0;
}

#if 0
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<std::string> CtfReader::tokenize(char* buf, char delimiter)
{
  std::vector<std::string> output;
  std::string values(buf);
  std::string::size_type start = 0;
  std::string::size_type pos = 0;
  //  std::cout << "-----------------------------" << std::endl;
  while(pos != std::string::npos && pos != values.size() - 1)
  {
    pos = values.find(delimiter, start);
    output.push_back(values.substr(start, pos - start));
    //   std::cout << "Adding: " << output.back() << std::endl;
    if (pos != std::string::npos)
    {
      start = pos + 1;
    }
  }
  return output;
}
#endif

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::getHeaderLines(std::ifstream& reader, std::vector<std::string>& headerLines)
{
  int err = 0;
  std::string buf;
  int numPhases = -1;
  while(!reader.eof() && !getHeaderIsComplete())
  {
    std::getline(reader, buf);
    // Append the line to the complete header
    appendOriginalHeader(std::string(buf));

    // remove the newline at the end of the line
    buf = EbsdStringUtils::chop(buf, 1);
    headerLines.push_back(buf);
    if(buf.find("Phases") != std::string::npos)
    {
      std::vector<std::string> tokens = EbsdStringUtils::split(buf, '\t');
      numPhases = std::stoi(tokens.at(1));
      break; //
    }
  }
  // Now read the phases line
  for(int p = 0; p < numPhases; ++p)
  {
    std::getline(reader, buf);
    appendOriginalHeader(std::string(buf));

    // remove the newline at the end of the line
    buf = EbsdStringUtils::chop(buf, 1);
    headerLines.push_back(buf);
  }
  setHeaderIsComplete(true);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool CtfReader::isDataHeaderLine(const std::vector<std::string>& columns) const
{
  if(columns.size() != 11)
  {
    return false;
  }
  if(columns[0] != "Phase")
  {
    return false;
  }
  if(columns[9] != "BC")
  {
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::getXDimension()
{
  return getXCells();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfReader::setXDimension(int xdim)
{
  setXCells(xdim);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::getYDimension()
{
  return getYCells();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfReader::setYDimension(int ydim)
{
  setYCells(ydim);
}

#define CTF_SHUFFLE_ARRAY(tempPtr, var, m_msgType, numRows)                                                                                                                                            \
  for(size_t i = 0; i < numRows; ++i)                                                                                                                                                                  \
  {                                                                                                                                                                                                    \
    size_t nIdx = shuffleTable[i];                                                                                                                                                                     \
    tempPtr[nIdx] = var[i];                                                                                                                                                                            \
  }                                                                                                                                                                                                    \
  /* Copy the values back into the array over writing the original values*/                                                                                                                            \
  ::memcpy(var, tempPtr, numRows * sizeof(m_msgType));

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<std::string> CtfReader::getColumnNames()
{
  std::vector<std::string> keys;
  keys.reserve(m_NamePointerMap.size());
  for(const auto& entry : m_NamePointerMap)
  {
    keys.push_back(entry.first);
  }

  return keys;
}

#define CTF_PRINT_QSTRING(var, out) out << #var << ": " << get##var() << std::endl;

#define CTF_PRINT_HEADER_VALUE(var, out) out << #var << ": " << get##var() << std::endl;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void CtfReader::printHeader(std::ostream& out)
{
  std::cout << "-------------------- CtfReader Header Values --------------------" << std::endl;
  CTF_PRINT_QSTRING(Channel, out);
  CTF_PRINT_QSTRING(Prj, out);
  CTF_PRINT_QSTRING(Author, out);
  CTF_PRINT_QSTRING(JobMode, out);
  CTF_PRINT_HEADER_VALUE(XCells, out);
  CTF_PRINT_HEADER_VALUE(YCells, out);
  CTF_PRINT_HEADER_VALUE(XStep, out);
  CTF_PRINT_HEADER_VALUE(YStep, out);
  CTF_PRINT_HEADER_VALUE(AcqE1, out);
  CTF_PRINT_HEADER_VALUE(AcqE2, out);
  CTF_PRINT_HEADER_VALUE(AcqE3, out);
  CTF_PRINT_QSTRING(Euler, out);
  CTF_PRINT_HEADER_VALUE(Mag, out);
  CTF_PRINT_HEADER_VALUE(Coverage, out);
  CTF_PRINT_HEADER_VALUE(Device, out);
  CTF_PRINT_HEADER_VALUE(KV, out);
  CTF_PRINT_HEADER_VALUE(TiltAngle, out);
  CTF_PRINT_HEADER_VALUE(TiltAxis, out);
  CTF_PRINT_HEADER_VALUE(NumPhases, out);
  int nPhases = getNumPhases();
  for(int p = 0; p < nPhases; ++p)
  {
    out << "### Phase " << p << std::endl;
    m_PhaseVector[p]->printSelf(out);
  }

  std::cout << "----------------------------------------" << std::endl;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int CtfReader::writeFile(const std::string& filepath)
{
  int error = 0;

  FILE* f = fopen(filepath.c_str(), "wb");
  if(nullptr == f)
  {
    return -1;
  }

  std::string header = getOriginalHeader();
  fwrite(header.c_str(), 1, header.size(), f);
  int zStart = 0;
  int zEnd = getZCells();
  int yCells = getYCells();
  int xCells = getXCells();

  std::vector<std::string> colNames = getColumnNames();

  using ColInfoType = struct
  {
    std::string colName = {""};
    void* ptr = nullptr;
    int aType = {0};
  };

  std::vector<ColInfoType> colInfos(colNames.size());

  for(const auto& name : colNames)
  {

    ColInfoType colInfo;
    colInfo.colName = name;

    DataParser::Pointer dparser = m_NamePointerMap[name];

    colInfo.ptr = dparser->getVoidPointer();

    colInfo.aType = 2; // undefined value

    if(1 == dparser->IsA())
    {
      colInfo.aType = 0;
    }

    if(2 == dparser->IsA())
    {
      colInfo.aType = 1;
    }

    colInfos[dparser->getColumnIndex()] = colInfo;
  }

  size_t counter = 0;
  for(int slice = zStart; slice < zEnd; ++slice)
  {
    for(int row = 0; row < yCells; ++row)
    {
      for(int col = 0; col < xCells; ++col)
      {
        for(size_t n = 0; n < colInfos.size(); n++)
        {
          ColInfoType& colInfo = colInfos[n];
          if(0 == colInfo.aType) // int32 pointer
          {
            int32_t* i32Ptr = reinterpret_cast<int32_t*>(colInfo.ptr);
            fprintf(f, "%d\t", i32Ptr[counter]);
          }
          else if(1 == colInfo.aType) // float pointer
          {
            float* f32Ptr = reinterpret_cast<float*>(colInfo.ptr);
            fprintf(f, "%0.4f\t", f32Ptr[counter]);
          }
          else
          {
            EBSD_UNKNOWN_TYPE(false);
          }
        }
        counter++;
        fprintf(f, "\n");
      }
    }
  }

  fclose(f);
  f = nullptr;

  return error;
}

// -----------------------------------------------------------------------------
std::string CtfReader::getNameOfClass() const
{
  return std::string("_SUPERCtfReader");
}

// -----------------------------------------------------------------------------
std::string CtfReader::ClassName()
{
  return std::string("_SUPERCtfReader");
}
