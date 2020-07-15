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

#include "AngleFileLoader.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "EbsdLib/Core/Orientation.hpp"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AngleFileLoader::AngleFileLoader()
: m_ErrorMessage("")
, m_InputFile("")
, m_AngleRepresentation(AngleFileLoader::EulerAngles)
, m_IgnoreMultipleDelimiters(true)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AngleFileLoader::~AngleFileLoader() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::FloatArrayType::Pointer AngleFileLoader::loadData()
{
  EbsdLib::FloatArrayType::Pointer angles = EbsdLib::FloatArrayType::NullPointer();

  // Make sure the input file variable is not empty
  if(m_InputFile.empty())
  {
    setErrorMessage("Input File Path is empty");
    setErrorCode(-1);
    return angles;
  }
  fs::path ifPath = fs::path(getInputFile());

  // Make sure the file exists on disk
  if(!fs::exists(ifPath))
  {
    std::stringstream ss;
    ss << "Input File does not exist at '" << ifPath.string() << "'\n";
    setErrorMessage(ss.str());
    setErrorCode(-2);
    return angles;
  }

  // Make sure we have a valid angle representation
  if(m_AngleRepresentation != EulerAngles && m_AngleRepresentation != QuaternionAngles && m_AngleRepresentation != RodriguezAngles)
  {
    setErrorMessage("The Angle representation was not set to anything known to this code");
    setErrorCode(-3);
    return angles;
  }

  // The format of the file is quite simple. Comment lines start with a "#" symbol
  // The only Key-Value pair we are looking for is 'Angle Count' which will have
  // the total number of angles that will be read

  int numOrients = 0;
  std::string buf;

  // Open the file and read the first line
  std::ifstream reader(getInputFile(), std::ios_base::in);
  if(!reader.is_open())
  {
    std::string msg = std::string("Angle file could not be opened: ") + getInputFile();
    setErrorCode(-100);
    setErrorMessage(msg);
    return angles;
  }

  std::getline(reader, buf);
  while(buf[0] == '#')
  {
    std::getline(reader, buf);
  }
  buf = EbsdStringUtils::trimmed(buf); // Remove leading and trailing whitespace

  // Split the next line into a pair of tokens delimited by the ":" character
  std::vector<std::string> tokens = EbsdStringUtils::split(buf, ':');
  if(tokens.size() != 2)
  {
    std::stringstream msg;
    msg << "Proper Header was not detected. The file should have a single header line of 'Angle Count:XXXX'";
    setErrorCode(-101);
    setErrorMessage(msg.str());
    return angles;
  }

  if(tokens[0] != "Angle Count")
  {
    std::stringstream msg;
    msg << "Proper Header was not detected. The file should have a single header line of 'Angle Count:XXXX'";
    setErrorCode(-102);
    setErrorMessage(msg.str());
    return angles;
  }
  numOrients = std::stoi(tokens[1]);

  // Allocate enough for the angles
  std::vector<size_t> dims(1, 5);
  angles = EbsdLib::FloatArrayType::CreateArray(numOrients, dims, "EulerAngles_From_File", true);

  for(int i = 0; i < numOrients; i++)
  {
    float weight = 0.0f;
    float sigma = 1.0f;
    std::getline(reader, buf);
    // Skip any lines that start with a '#' character
    if(buf[0] == '#')
    {
      continue;
    }
    buf = EbsdStringUtils::trimmed(buf); // Remove leading and trailing whitespace

    // Remove multiple Delimiters if wanted by the user.
    if(m_IgnoreMultipleDelimiters)
    {
      buf = EbsdStringUtils::simplified(buf); // Remove leading and trailing whitespace
    }
    std::string delimiter = getDelimiter();
    if(delimiter == "\t")
    {
      setDelimiter(" ");
    }
    tokens = EbsdStringUtils::split(buf, (*(getDelimiter().c_str())));

    OrientationF euler(3);
    if(m_AngleRepresentation == EulerAngles)
    {
      euler[0] = std::stof(tokens[0]);
      euler[1] = std::stof(tokens[1]);
      euler[2] = std::stof(tokens[2]);
      weight = std::stof(tokens[3]);
      sigma = std::stof(tokens[4]);
    }
    else if(m_AngleRepresentation == QuaternionAngles)
    {
      QuatF quat(4);

      quat.x() = std::stof(tokens[0]);
      quat.y() = std::stof(tokens[1]);
      quat.z() = std::stof(tokens[2]);
      quat.w() = std::stof(tokens[3]);

      euler = OrientationTransformation::qu2eu<QuatF, OrientationF>(quat);
      weight = std::stof(tokens[4]);
      sigma = std::stof(tokens[5]);
    }
    else if(m_AngleRepresentation == RodriguezAngles)
    {
      Orientation<float> rod(4, 0.0);
      rod[0] = std::stof(tokens[0]);
      rod[1] = std::stof(tokens[1]);
      rod[2] = std::stof(tokens[2]);
      euler = OrientationTransformation::ro2eu<OrientationF, OrientationF>(rod);
      weight = std::stof(tokens[3]);
      sigma = std::stof(tokens[4]);
    }

    // Values in File are in Radians and the user wants them in Degrees
    if(!m_FileAnglesInDegrees && m_OutputAnglesInDegrees)
    {
      euler[0] = euler[0] * EbsdLib::Constants::k_RadToDegF;
      euler[1] = euler[1] * EbsdLib::Constants::k_RadToDegF;
      euler[2] = euler[2] * EbsdLib::Constants::k_RadToDegF;
    }
    // Values are in Degrees but user wants them in Radians
    else if(m_FileAnglesInDegrees && !m_OutputAnglesInDegrees)
    {
      euler[0] = euler[0] * EbsdLib::Constants::k_DegToRadF;
      euler[1] = euler[1] * EbsdLib::Constants::k_DegToRadF;
      euler[2] = euler[2] * EbsdLib::Constants::k_DegToRadF;
    }

    // Store the values into our array
    angles->setComponent(i, 0, euler[0]);
    angles->setComponent(i, 1, euler[1]);
    angles->setComponent(i, 2, euler[2]);
    angles->setComponent(i, 3, weight);
    angles->setComponent(i, 4, sigma);
    //   std::cout << "reading line: " << i ;
  }

  return angles;
}

// -----------------------------------------------------------------------------
AngleFileLoader::Pointer AngleFileLoader::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
AngleFileLoader::Pointer AngleFileLoader::New()
{
  Pointer sharedPtr(new(AngleFileLoader));
  return sharedPtr;
}

// -----------------------------------------------------------------------------
std::string AngleFileLoader::getNameOfClass() const
{
  return std::string("AngleFileLoader");
}

// -----------------------------------------------------------------------------
std::string AngleFileLoader::ClassName()
{
  return std::string("AngleFileLoader");
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setErrorMessage(const std::string& value)
{
  m_ErrorMessage = value;
}

// -----------------------------------------------------------------------------
std::string AngleFileLoader::getErrorMessage() const
{
  return m_ErrorMessage;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setErrorCode(int value)
{
  m_ErrorCode = value;
}

// -----------------------------------------------------------------------------
int AngleFileLoader::getErrorCode() const
{
  return m_ErrorCode;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setInputFile(const std::string& value)
{
  m_InputFile = value;
}

// -----------------------------------------------------------------------------
std::string AngleFileLoader::getInputFile() const
{
  return m_InputFile;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setFileAnglesInDegrees(bool value)
{
  m_FileAnglesInDegrees = value;
}

// -----------------------------------------------------------------------------
bool AngleFileLoader::getFileAnglesInDegrees() const
{
  return m_FileAnglesInDegrees;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setOutputAnglesInDegrees(bool value)
{
  m_OutputAnglesInDegrees = value;
}

// -----------------------------------------------------------------------------
bool AngleFileLoader::getOutputAnglesInDegrees() const
{
  return m_OutputAnglesInDegrees;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setAngleRepresentation(uint32_t value)
{
  m_AngleRepresentation = value;
}

// -----------------------------------------------------------------------------
uint32_t AngleFileLoader::getAngleRepresentation() const
{
  return m_AngleRepresentation;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setDelimiter(const std::string& value)
{
  m_Delimiter = value;
}

// -----------------------------------------------------------------------------
std::string AngleFileLoader::getDelimiter() const
{
  return m_Delimiter;
}

// -----------------------------------------------------------------------------
void AngleFileLoader::setIgnoreMultipleDelimiters(bool value)
{
  m_IgnoreMultipleDelimiters = value;
}

// -----------------------------------------------------------------------------
bool AngleFileLoader::getIgnoreMultipleDelimiters() const
{
  return m_IgnoreMultipleDelimiters;
}
