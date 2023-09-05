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
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "AngReader.h"
#include "AngConstants.h"

#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/IO/EbsdReader.h"
#include "EbsdLib/Math/EbsdLibMath.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <utility>
#include <optional>

namespace
{

using Vec3Type = std::array<float, 3>;
using Size3Type = std::array<size_t, 3>;

std::optional<size_t> GetGridIndex(Vec3Type& coords, Vec3Type& m_Origin, Vec3Type& m_Spacing, Size3Type& m_Dimensions )
{
  if(coords[0] < m_Origin[0] || coords[0]  > (static_cast<float>(m_Dimensions[0]) * m_Spacing[0] + m_Origin[0]))
  {
    return {};
  }

  if(coords[1] < m_Origin[1] || coords[1] > (static_cast<float>(m_Dimensions[1]) * m_Spacing[1] + m_Origin[1]))
  {
    return {};
  }

  if(coords[2] < m_Origin[2] || coords[2] > (static_cast<float>(m_Dimensions[2]) * m_Spacing[2] + m_Origin[2]))
  {
    return {};
  }

  size_t x = static_cast<size_t>(std::floor((coords[0]  - m_Origin[0]) / m_Spacing[0]));
  if(x >= m_Dimensions[0])
  {
    return {};
  }

  size_t y = static_cast<size_t>(std::floor((coords[1] - m_Origin[1]) / m_Spacing[1]));
  if(y >= m_Dimensions[1])
  {
    return {};
  }

  size_t z = static_cast<size_t>(std::floor((coords[2] - m_Origin[2]) / m_Spacing[2]));
  if(z >= m_Dimensions[2])
  {
    return {};
  }

  return (m_Dimensions[1] * m_Dimensions[0] * z) + (m_Dimensions[0] * y) + x;
}
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AngReader::AngReader()
{
  // Init all the arrays to nullptr
  m_Phi1 = nullptr;
  m_Phi = nullptr;
  m_Phi2 = nullptr;
  m_Iq = nullptr;
  m_Ci = nullptr;
  m_PhaseData = nullptr;
  m_X = nullptr;
  m_Y = nullptr;
  m_SEMSignal = nullptr;
  m_Fit = nullptr;

  setNumFeatures(10);

  m_ReadHexGrid = false;

  // Initialize the map of header key to header value
  m_HeaderMap[EbsdLib::Ang::TEMPIXPerUM] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::TEMPIXPerUM);
  m_HeaderMap[EbsdLib::Ang::XStar] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::XStar);
  m_HeaderMap[EbsdLib::Ang::YStar] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::YStar);
  m_HeaderMap[EbsdLib::Ang::ZStar] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZStar);
  m_HeaderMap[EbsdLib::Ang::WorkingDistance] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::WorkingDistance);
  m_HeaderMap[EbsdLib::Ang::Grid] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::Grid);
  m_HeaderMap[EbsdLib::Ang::XStep] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::XStep);
  m_HeaderMap[EbsdLib::Ang::YStep] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::YStep);
  m_HeaderMap[EbsdLib::Ang::ZStep] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZStep); // NOT actually in the file>::NewEbsdHeaderEntry(); , but may be needed
  m_HeaderMap[EbsdLib::Ang::ZPos] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZPos);   // NOT actually in the file>::NewEbsdHeaderEntry(); , but may be needed
  m_HeaderMap[EbsdLib::Ang::ZMax] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::ZMax);   // NOT actually in the file>::NewEbsdHeaderEntry(); , but may be needed
  m_HeaderMap[EbsdLib::Ang::NColsOdd] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::NColsOdd);
  m_HeaderMap[EbsdLib::Ang::NColsEven] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::NColsEven);
  m_HeaderMap[EbsdLib::Ang::NRows] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::NRows);
  m_HeaderMap[EbsdLib::Ang::OPERATOR] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::OPERATOR);
  m_HeaderMap[EbsdLib::Ang::SAMPLEID] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::SAMPLEID);
  m_HeaderMap[EbsdLib::Ang::SCANID] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::SCANID);
  m_HeaderMap[EbsdLib::Ang::ColumnCount] = AngHeaderEntry<int>::NewEbsdHeaderEntry(EbsdLib::Ang::ColumnCount);
  m_HeaderMap[EbsdLib::Ang::ColumnHeaders] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::ColumnHeaders);
  m_HeaderMap[EbsdLib::Ang::ColumnUnits] = AngStringHeaderEntry::NewEbsdHeaderEntry(EbsdLib::Ang::ColumnUnits);

  // Give these values some defaults
  setNumOddCols(-1);
  setNumEvenCols(-1);
  setNumRows(-1);
  setXStep(0.0f);
  setYStep(0.0f);
}

// -----------------------------------------------------------------------------
//  Clean up any Memory that was allocated for this class
// -----------------------------------------------------------------------------
AngReader::~AngReader()
{
  if(m_Phi1Cleanup)
  {
    deallocateArrayData<float>(m_Phi1);
    m_Phi1 = nullptr;
  }
  if(m_PhiCleanup)
  {
    deallocateArrayData<float>(m_Phi);
    m_Phi = nullptr;
  }
  if(m_Phi2Cleanup)
  {
    deallocateArrayData<float>(m_Phi2);
    m_Phi2 = nullptr;
  }
  if(m_IqCleanup)
  {
    deallocateArrayData<float>(m_Iq);
    m_Iq = nullptr;
  }
  if(m_CiCleanup)
  {
    deallocateArrayData<float>(m_Ci);
    m_Ci = nullptr;
  }
  if(m_PhaseDataCleanup)
  {
    deallocateArrayData<int32_t>(m_PhaseData);
    m_PhaseData = nullptr;
  }
  if(m_XCleanup)
  {
    deallocateArrayData<float>(m_X);
    m_X = nullptr;
  }
  if(m_YCleanup)
  {
    deallocateArrayData<float>(m_Y);
    m_Y = nullptr;
  }
  if(m_SEMSignalCleanup)
  {
    deallocateArrayData<float>(m_SEMSignal);
    m_SEMSignal = nullptr;
  }
  if(m_FitCleanup)
  {
    deallocateArrayData<float>(m_Fit);
    m_Fit = nullptr;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void* AngReader::getPointerByName(const std::string& featureName)
{
  if(featureName == EbsdLib::Ang::Phi1)
  {
    return static_cast<void*>(m_Phi1);
  }
  if(featureName == EbsdLib::Ang::Phi)
  {
    return static_cast<void*>(m_Phi);
  }
  if(featureName == EbsdLib::Ang::Phi2)
  {
    return static_cast<void*>(m_Phi2);
  }
  if(featureName == EbsdLib::Ang::ImageQuality)
  {
    return static_cast<void*>(m_Iq);
  }
  if(featureName == EbsdLib::Ang::ConfidenceIndex)
  {
    return static_cast<void*>(m_Ci);
  }
  if(featureName == EbsdLib::Ang::PhaseData)
  {
    return static_cast<void*>(m_PhaseData);
  }
  if(featureName == EbsdLib::Ang::XPosition)
  {
    return static_cast<void*>(m_X);
  }
  if(featureName == EbsdLib::Ang::YPosition)
  {
    return static_cast<void*>(m_Y);
  }
  if(featureName == EbsdLib::Ang::SEMSignal)
  {
    return static_cast<void*>(m_SEMSignal);
  }
  if(featureName == EbsdLib::Ang::Fit)
  {
    return static_cast<void*>(m_Fit);
  }
  return nullptr;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::NumericTypes::Type AngReader::getPointerType(const std::string& featureName)
{
  if(featureName == EbsdLib::Ang::Phi1)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::Phi)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::Phi2)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::ImageQuality)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::ConfidenceIndex)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::PhaseData)
  {
    return EbsdLib::NumericTypes::Type::Int32;
  }
  if(featureName == EbsdLib::Ang::XPosition)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::YPosition)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::SEMSignal)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  if(featureName == EbsdLib::Ang::Fit)
  {
    return EbsdLib::NumericTypes::Type::Float;
  }
  return EbsdLib::NumericTypes::Type::UnknownNumType;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AngReader::readHeaderOnly()
{
  int err = 1;
  std::string buf;
  std::ifstream in(getFileName(), std::ios_base::in);
  setHeaderIsComplete(false);
  if(!in.is_open())
  {
    std::string msg = std::string("Ang file could not be opened: ") + getFileName();
    setErrorCode(-100);
    setErrorMessage(msg);
    return -100;
  }
  std::string origHeader;
  setOriginalHeader(origHeader);
  std::stringstream ostr(origHeader);
  m_PhaseVector.clear();

  while(!in.eof() && !getHeaderIsComplete())
  {
    std::getline(in, buf);

    parseHeaderLine(buf);
    if(!getHeaderIsComplete())
    {
      ostr << buf << "\n";
    }
  }
  // Update the Original Header variable
  setOriginalHeader(origHeader);
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AngReader::readFile()
{
  setErrorCode(0);
  setErrorMessage("");
  std::string buf;
  setHeaderIsComplete(false);

  std::ifstream in(getFileName(), std::ios_base::in);
  if(!in.is_open())
  {
    std::string msg = "Ang file could not be opened:" + getFileName();
    setErrorCode(-100);
    setErrorMessage(msg);
    return -100;
  }

  std::string origHeader;
  setOriginalHeader(origHeader);
  m_PhaseVector.clear();

  while(!in.eof() && !getHeaderIsComplete())
  {
    std::getline(in, buf);
    if(buf.at(0) != '#')
    {
      setHeaderIsComplete(true);
    }
    else
    {
      origHeader.append(buf).append("\n");
      parseHeaderLine(buf);
    }
  }
  // Update the Original Header variable
  setOriginalHeader(origHeader);

  if(getErrorCode() < 0)
  {
    return getErrorCode();
  }

  if(getXStep() == 0.0 || getYStep() == 0.0f)
  {
    std::string msg = std::string("Either the X Step or Y Step was Zero (0.0) and this is not allowed");
    setErrorCode(-110);
    setErrorMessage(msg);
    return -110;
  }
  if(m_PhaseVector.empty())
  {
    setErrorCode(-150);
    setErrorMessage("No phase was parsed in the header portion of the file. This possibly means that part of the header is missing.");
    return -150;
  }
  // We need to pass in the buffer because it has the first line of data
  readData(in, buf);
  if(getErrorCode() < 0)
  {
    return getErrorCode();
  }
  std::vector<int64_t> indexMap;
  std::pair<int, std::string> result = fixOrderOfData(indexMap);

  if(result.first < 0)
  {
    setErrorCode(result.first);
    setErrorMessage(result.second);
    return result.first;
  }

  std::vector<std::string> arrayNames = {"Phi1", "Phi", "Phi2", "X Position", "Y Position", "Image Quality", "Confidence Index", "PhaseData", "SEM Signal", "Fit"};
  for(const auto& arrayName : arrayNames)
  {
    void* oldArray = getPointerByName(arrayName);

    if(getPointerType(arrayName) == EbsdLib::NumericTypes::Type::Float)
    {
      CopyTupleUsingIndexList<float>(oldArray, indexMap);
    }
    else if(getPointerType(arrayName) == EbsdLib::NumericTypes::Type::Int32)
    {
      CopyTupleUsingIndexList<int32_t>(oldArray, indexMap);
    }
    else
    {
      std::cout << "Type returned was not of Float or int32. The Array name probably isn't correct." << std::endl;
    }
  }

  return getErrorCode();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngReader::readData(std::ifstream& in, std::string& buf)
{
  std::string streamBuf;
  std::stringstream ss(streamBuf);

  size_t totalDataPoints = 0;

  std::string grid = getGrid();

  int nOddCols = getNumOddCols();
  int nEvenCols = getNumEvenCols();
  int numRows = getNumRows();

  if(numRows < 1)
  {
    setErrorCode(-200);
    setErrorMessage("NumRows Sanity Check not correct. Check the entry for NROWS in the .ang file");
    return;
  }
  if(grid.find(EbsdLib::Ang::SquareGrid) == 0)
  {
    if(nOddCols > 0)
    {
      totalDataPoints = numRows * nOddCols; /* xCells = nOddCols;*/
    }
    else if(nEvenCols > 0)
    {
      totalDataPoints = numRows * nEvenCols; /* xCells = nEvexCells; */
    }
    else
    {
      totalDataPoints = 0;
    }
  }
  else if(grid.find(EbsdLib::Ang::HexGrid) == 0 && !m_ReadHexGrid)
  {
    setErrorCode(-400);
    setErrorMessage("Ang Files with Hex Grids Are NOT currently supported - Try converting them to Square Grid with the Hex2Sqr Converter filter.");
    return;
  }
  else if(grid.find(EbsdLib::Ang::HexGrid) == 0 && m_ReadHexGrid)
  {
    bool evenRow = false;
    totalDataPoints = 0;
    for(int r = 0; r < numRows; r++)
    {
      if(evenRow)
      {
        totalDataPoints = totalDataPoints + nEvenCols;
        evenRow = false;
      }
      else
      {
        totalDataPoints = totalDataPoints + nOddCols;
        evenRow = true;
      }
    }
  }
  else // Grid was not set
  {
    setErrorMessage("Ang file is missing the 'GRID' header entry.");
    setErrorCode(-300);
    return;
  }

  // Initialize all the pointers and allocate memory
  setNumberOfElements(totalDataPoints);
  size_t numBytes = totalDataPoints * sizeof(float);
  m_Phi1 = allocateArray<float>(totalDataPoints);
  m_Phi = allocateArray<float>(totalDataPoints);
  m_Phi2 = allocateArray<float>(totalDataPoints);
  m_Iq = allocateArray<float>(totalDataPoints);
  m_Ci = allocateArray<float>(totalDataPoints);
  m_PhaseData = allocateArray<int>(totalDataPoints);
  m_X = allocateArray<float>(totalDataPoints);
  m_Y = allocateArray<float>(totalDataPoints);
  m_SEMSignal = allocateArray<float>(totalDataPoints);
  m_Fit = allocateArray<float>(totalDataPoints);

  ::memset(m_Phi1, 0, numBytes);
  ::memset(m_Phi, 0, numBytes);
  ::memset(m_Phi2, 0, numBytes);
  ::memset(m_Iq, 0, numBytes);
  ::memset(m_Ci, 0, numBytes);
  ::memset(m_PhaseData, 0, numBytes);
  ::memset(m_X, 0, numBytes);
  ::memset(m_Y, 0, numBytes);
  ::memset(m_SEMSignal, 0, numBytes);
  ::memset(m_Fit, 0, numBytes);

  if(nullptr == m_Phi1 || nullptr == m_Phi || nullptr == m_Phi2 || nullptr == m_Iq || nullptr == m_SEMSignal || nullptr == m_Ci || nullptr == m_PhaseData || m_X == nullptr || m_Y == nullptr)
  {
    ss.str("");
    ss << "Internal pointers were nullptr at " << __FILE__ << "(" << __LINE__ << ")\n";
    setErrorMessage(ss.str());
    setErrorCode(-2500);
    return;
  }

  size_t counter = 1; // Because we are on the first line now.

  bool onEvenRow = false;
  int col = 0;

  int yChange = 0;
  float oldY = m_Y[0];
  int nxOdd = 0;
  int nxEven = 0;
  // int nRows = 0;

  for(size_t i = 0; i < totalDataPoints; ++i)
  {
    if(i > 0)
    {
      //  ::memset(buf, 0, bufSize); // Clear the buffer
      std::getline(in, buf);
      ++counter;
    }
    parseDataLine(buf, i);
    if(getErrorCode() < 0)
    {
      ss.str("");

      ss << "Error parsing the data line (Numeric conversion). Error code is " << getErrorCode() << " and occurred at data column " << m_ErrorColumn << " (Zero Based)\n"
         << buf << "\n*** Header information ***\nRows=" << numRows << " EvenCols=" << nEvenCols << " OddCols=" << nOddCols << "  Calculated Data Points: " << totalDataPoints
         << "\n***Parsing Position ***\nCurrent Row: " << yChange << "  Current Column Index: " << col << "  Current Data Point Count: " << counter << "\n";
      setErrorMessage(ss.str());
      break;
    }

    if(fabs(m_Y[i] - oldY) > 1e-6)
    {
      ++yChange;
      oldY = m_Y[i];
      onEvenRow = !onEvenRow;
      col = 0;
    }
    else
    {
      col++;
    }
    if(yChange == 0)
    {
      ++nxOdd;
    }
    if(yChange == 1)
    {
      ++nxEven;
    }
    if(in.eof())
    {
      break;
    }
  }

#if 0
  nRows = yChange + 1;

  std::cout << "Header: nRows: " << numRows << " Odd Cols: " << nOddCols << "  Even Cols: " << nEvenCols << std::endl;
  std::cout << "File:   nRows: " << nRows << " Odd Cols: " << nxOdd << "  Even Cols: " << nxEven << std::endl;
#endif

  if(getNumFeatures() < 10)
  {
    deallocateArrayData<float>(m_Fit);
  }
  if(getNumFeatures() < 9)
  {
    deallocateArrayData<float>(m_SEMSignal);
  }
  if(getErrorCode() < 0)
  {
    return;
  }

  if(counter != totalDataPoints && in.eof())
  {
    ss.str("");

    ss << "End of ANG file reached before all data was parsed.\n"
       << getFileName() << "\n*** Header information ***\nRows=" << numRows << " EvenCols=" << nEvenCols << " OddCols=" << nOddCols << "  Calculated Data Points: " << totalDataPoints
       << "\n***Parsing Position ***\nCurrent Row: " << yChange << "  Current Column Index: " << col << "  Current Data Point Count: " << counter << "\n";
    setErrorMessage(ss.str());
    setErrorCode(-600);
  }
}

// -----------------------------------------------------------------------------
//  Read the Header part of the ANG file
// -----------------------------------------------------------------------------
void AngReader::parseHeaderLine(std::string& buf)
{
  // Check to see if we are reading a header or data line.
  if(buf[0] != '#')
  {
    setHeaderIsComplete(true);
    return;
  }

  // For Version = 7 Files there is a notes section. Just read all of that into the m_Notes String and return
  if(m_InsideNotes)
  {
    m_Notes += buf + '\n'; // The readLine removed the newline character but probably left the carriage return character
    return;
  }
  // For Version = 7 Files there is a notes section. Just read all of that into the m_Notes String and return
  if(m_InsideColumnNotes)
  {
    m_ColumnNotes += buf + '\n'; // The readLine removed the newline character but probably left the carriage return character
    return;
  }

  buf = buf.substr(1);                    // remove the '#' charater
  buf = EbsdStringUtils::simplified(buf); // remove leading/trailing white space and multiple white space characters internal to the array

  // now split the array based on spaces
  std::vector<std::string> tokens = EbsdStringUtils::split(buf, ' ');
  std::string word;
  if(!tokens.empty())
  {
    word = tokens.at(0);
  }
  if(word.find_last_of(':') != std::string::npos)
  {
    word = EbsdStringUtils::chop(word, 1);
  }

  if(buf == EbsdLib::Ang::NotesStart)
  {
    m_InsideNotes = true;
    m_Notes = "# " + EbsdLib::Ang::NotesStart + "\r\n";
  }
  else if(buf == EbsdLib::Ang::NotesEnd)
  {
    m_InsideNotes = false;
  }

  if(buf == EbsdLib::Ang::ColumnNotesStart)
  {
    m_InsideColumnNotes = true;
    m_ColumnNotes = "# " + EbsdLib::Ang::ColumnNotesStart + "\r\n";
  }
  else if(buf == EbsdLib::Ang::ColumnNotesEnd)
  {
    m_InsideColumnNotes = false;
  }

  // If the word is "Phase" then we need to construct a "Phase" class and
  // store all the meta data for the phase into that class. When we are done
  // parsing data for the phase then stick the Phase instance into the header
  // map or stick it into a vector<Phase::Pointer> and stick the vector into
  // the map under the "Phase" key
  if(word == EbsdLib::Ang::Phase && !m_InsideNotes)
  {
    m_CurrentPhase = AngPhase::New();
    try
    {
      m_CurrentPhase->setPhaseIndex(std::stoi(tokens.at(1)));
    } catch(std::invalid_argument& e)
    {
      std::cout << e.what() << std::endl;

    } catch(std::out_of_range& e)
    {
      std::cout << e.what() << std::endl;
    }
    // Parsing the phase is complete, now add it to the vector of Phases
    m_PhaseVector.push_back(m_CurrentPhase);
  }
  else if(word == EbsdLib::Ang::MaterialName && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      m_CurrentPhase->parseMaterialName(tokens);
    }
  }
  else if(word == EbsdLib::Ang::Formula && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      m_CurrentPhase->parseFormula(tokens);
    }
  }
  else if(word == EbsdLib::Ang::Symmetry && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      try
      {
        m_CurrentPhase->setSymmetry(std::stoi(tokens.at(1)));
      } catch(std::invalid_argument& e)
      {
        std::cout << e.what() << std::endl;

      } catch(std::out_of_range& e)
      {
        std::cout << e.what() << std::endl;
      }
    }
  }
  else if(word == EbsdLib::Ang::LatticeConstants && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      m_CurrentPhase->parseLatticeConstants(tokens);
    }
  }
  else if(word == EbsdLib::Ang::NumberFamilies && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      try
      {
        m_CurrentPhase->setNumberFamilies(std::stoi(tokens.at(1)));
      } catch(std::invalid_argument& e)
      {
        std::cout << e.what() << std::endl;

      } catch(std::out_of_range& e)
      {
        std::cout << e.what() << std::endl;
      }
    }
  }
  else if(word == EbsdLib::Ang::HKLFamilies && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      m_CurrentPhase->parseHKLFamilies(tokens);
    }
  }
  else if(word.find(EbsdLib::Ang::Categories) == 0 && m_CurrentPhase.get() != nullptr)
  {
    if(tokens.size() > 1)
    {
      m_CurrentPhase->parseCategories(tokens);
    }
  }
  else
  {
    EbsdHeaderEntry::Pointer p = m_HeaderMap[word];
    if(nullptr == p.get())
    {
#if 0
      std::cout << "---------------------------" << std::endl;
      std::cout << "Could not find header entry for key'" << word << "'" << std::endl;
      std::string upper(word);
      std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

      std::cout << "#define ANG_" << upper << "     \"" << word << "\"" << std::endl;
      std::cout << "const std::string " << word << "(ANG_" << upper << ");" << std::endl;
      std::cout << "EBSDHEADER_INSTANCE_PROPERTY(AngHeaderEntry<int>, int, " << word << "EbsdLib::Ang::" << word << ")" << std::endl;
      std::cout << "m_HeaderMap[EbsdLib::Ang::" << word << "] = AngHeaderEntry<float>::NewEbsdHeaderEntry(EbsdLib::Ang::" << word << ");" << std::endl;
#endif
#if 0
      std::cout << "<tr>\n    <td>" << word << "</td>\n    <td>" << "H5T_STRING" << "</td>\n";
      std::cout << "    <td colspan=\"2\"> Contains value for the header entry " << word << "</td>\n</tr>" << std::endl;
#endif
      return;
    }
    if(tokens.size() > 1)
    {
      p->parseValue(tokens[1]);
#if 0
      std::cout << "<tr>\n    <td>" << p->getKey() << "</td>\n    <td>" << p->getHDFType() << "</td>\n";
      std::cout << "    <td colspan=\"2\"> Contains value for the header entry " << p->getKey() << "</td>\n</tr>" << std::endl;
#endif
    }
  }
}

// -----------------------------------------------------------------------------
//  Read the data part of the ANG file
// -----------------------------------------------------------------------------
void AngReader::parseDataLine(std::string& line, size_t i)
{
  /* When reading the data there should be at least 8 cols of data. There may even
   * be 10 columns of data. The column names should be the following:
   * phi1
   * phi
   * phi2
   * x pos
   * y pos
   * image quality
   * confidence index
   * phase
   * SEM Signal
   * Fit of Solution
   *
   * Some TSL ang files do NOT have all 10 columns. Assume these are lacking the last
   * 2 columns and all the other columns are the same as above.
   */
  m_ErrorColumn = 0;
  float p1 = 0.0f;
  float p = 0.0f;
  float p2 = 0.0f;
  float x = -1.0f;
  float y = -1.0f;
  float iqual = -1.0f;
  float conf = -1.0f;
  float semSignal = -1.0f;
  float fit = -1.0f;
  int ph = 0;
  size_t offset = 0;
  line = EbsdStringUtils::simplified(line);
  std::vector<std::string> tokens = EbsdStringUtils::split(line, ' ');
  bool ok = true;
  offset = i;
  if(!tokens.empty())
  {
    p1 = std::stof(tokens[0]);
    if(!ok)
    {
      setErrorCode(-2501);
      m_ErrorColumn = 0;
    }
    m_Phi1[offset] = p1;
  }
  if(tokens.size() >= 2)
  {
    p = std::stof(tokens[1]);
    if(!ok)
    {
      setErrorCode(-2502);
      m_ErrorColumn = 1;
    }
    m_Phi[offset] = p;
  }
  if(tokens.size() >= 3)
  {
    p2 = std::stof(tokens[2]);
    if(!ok)
    {
      setErrorCode(-2503);
      m_ErrorColumn = 2;
    }
    m_Phi2[offset] = p2;
  }
  if(tokens.size() >= 4)
  {
    x = std::stof(tokens[3]);
    if(!ok)
    {
      setErrorCode(-2504);
      m_ErrorColumn = 3;
    }
    m_X[offset] = x;
  }
  if(tokens.size() >= 5)
  {
    y = std::stof(tokens[4]);
    if(!ok)
    {
      setErrorCode(-2505);
      m_ErrorColumn = 4;
    }
    m_Y[offset] = y;
  }
  if(tokens.size() >= 6)
  {
    iqual = std::stof(tokens[5]);
    if(!ok)
    {
      setErrorCode(-2506);
      m_ErrorColumn = 5;
    }
    m_Iq[offset] = iqual;
  }
  if(tokens.size() >= 7)
  {
    conf = std::stof(tokens[6]);
    if(!ok)
    {
      setErrorCode(-2507);
      m_ErrorColumn = 6;
    }
    m_Ci[offset] = conf;
  }
  if(tokens.size() >= 8)
  {
    try
    {
      ph = std::stoi(tokens[7]);
    } catch([[maybe_unused]] const std::invalid_argument& e1)
    {
      setErrorCode(-2508);
      m_ErrorColumn = 7;
      // Some have floats instead of integers so lets try that.
      try
      {
        float f = std::stof(tokens[7]);
        setErrorCode(0);
        ph = static_cast<int32_t>(f);
      } catch([[maybe_unused]] const std::invalid_argument& e2)
      {
        setErrorCode(-2588);
        m_ErrorColumn = 7;
      }
    }
    m_PhaseData[offset] = ph;
  }

  if(tokens.size() >= 9)
  {
    semSignal = std::stof(tokens[8]);
    if(!ok)
    {
      setErrorCode(-2509);
      m_ErrorColumn = 8;
    }
    m_SEMSignal[offset] = semSignal;
  }
  if(tokens.size() >= 10)
  {
    fit = std::stof(tokens[9]);
    if(!ok)
    {
      setErrorCode(-2510);
      m_ErrorColumn = 9;
    }
    m_Fit[offset] = fit;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::pair<int, std::string> AngReader::fixOrderOfData(std::vector<int64_t>& indexMap)
{
  int64_t numCols = getNumOddCols();
  int64_t numRows = getNumRows();
  int64_t numElements = numCols * numRows;

  float* xPosition = getXPositionPointer();
  if(nullptr == xPosition)
  {
    std::stringstream message;
    message << "Error: XPosition Pointer was NULL" << std::endl;
    return {-110, message.str()};
  }
  float* yPosition = getYPositionPointer();
  if(nullptr == yPosition)
  {
    std::stringstream message;
    message << "Error: YPosition Pointer was NULL" << std::endl;
    return {-111, message.str()};
  }

  float xStep = getXStep();
  float yStep = getYStep();

  auto resultX = std::minmax_element(xPosition, xPosition + numElements);
  auto resultY = std::minmax_element(yPosition, yPosition + numElements);
  float xMin = *resultX.first;
  float xMax = *resultX.second;
  float yMin = *resultY.first;
  float yMax = *resultY.second;

  std::array<float, 3> m_Origin = {xMin - 0.5f * xStep, yMin - 0.5f * yStep, 0.0f};
  std::array<float, 3> m_Spacing = {xStep, yStep, 1.0f};
  std::array<size_t, 3> m_Dimensions = {static_cast<size_t>(numCols), static_cast<size_t>(numRows), 1ULL};
  std::array<float, 3> coords = {0.0f, 0.0f, 0.0f};

  if(std::nearbyint((xMax - xMin) / xStep) + 1 != numCols)
  {
    std::stringstream message;
    message << "Error: The calculated number of columns (" << ((xMax - xMin) / xStep) + 1 << ") does not match the actual number of columns (" << numCols << ")" << std::endl;
    return {-100, message.str()};
  }
  if(std::nearbyint((yMax - yMin) / yStep) + 1 != numRows)
  {
    std::stringstream message;
    message << "Error: The calculated number of rows YMax: " << yMax << ", YMin: " << yMin << ", YStep: " << yStep << "  (" << ((yMax - yMin) / yStep) + 1 << ") does not match the actual number of rows (" << numRows + 1
            << ")" << std::endl;
    return {-101101, message.str()};
  }

  indexMap.resize(numElements);

  for(int i = 0; i < numElements; i++)
  {
    coords[0] = xPosition[i];
    coords[1] = yPosition[i];
    auto result = ::GetGridIndex(coords, m_Origin, m_Spacing, m_Dimensions);
    if(result.has_value())
    {
      indexMap[i] = *result;
    }
    else
    {
      std::stringstream message;
      message << "AngReader Error: The calculated index for the X and Y Position " << coords[0] << ", " << coords[1] << " will fall outside of the calculated grid.\n  "
      << "Origin: " << m_Origin[0] <<", " << m_Origin[1] << "\n  "
      << "Spacing: " << m_Spacing[1] << ", " << m_Spacing[1] << "\n  "
      << "Dimensions: " << m_Dimensions[0] << ", " << m_Dimensions[1] << "\n" << std::endl;
      return {-101111, message.str()};
    }
  }
  return {0, ""};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AngReader::getXDimension()
{
  return getNumEvenCols();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngReader::setXDimension(int xdim)
{
  setNumEvenCols(xdim);
  setNumOddCols(xdim);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AngReader::getYDimension()
{
  return getNumRows();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AngReader::setYDimension(int ydim)
{
  setNumRows(ydim);
}
