//
// Created by Michael Jackson on 10/22/24.
//

#include "CprReader.h"

#include "CtfPhase.h"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/inipp.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace ebsdlib;

namespace
{

/*
enum ColumnNames  {
    'X'                  % 1    4 bytes
    'Y'                  % 2       "
    'phi1'               % 3       "
    'Phi'                % 4       "
    'phi2'               % 5       "
    'MAD'                % 6       "
    'BC'                 % 7    1 byte
    'BS'                 % 8       "
    'Unknown'            % 9       "
    'Bands'              % 10      "
    'Error'              % 11      "
    'ReliabilityIndex'   % 12      "
};
*/

// std::array<size_t, 13> k_FieldByteSize = {0, 4, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 1};
// std::array<std::string, 13> k_FieldNames = {"", "X", "Y", "phi1", "Phi", "phi2", "MAD", "BC", "BS", "Unknown", "Bands", "Error", "ReliabilityIndex"};
// std::array<ebsdlib::NumericTypes::Type, 13> k_FieldNumericTypes = {ebsdlib::NumericTypes::Type::UnknownNumType,
//                                                                    ebsdlib::NumericTypes::Type::Float, ebsdlib::NumericTypes::Type::Float, ebsdlib::NumericTypes::Type::Float,
//                                                                    ebsdlib::NumericTypes::Type::Float, ebsdlib::NumericTypes::Type::Float, ebsdlib::NumericTypes::Type::Float,
//                                                                    ebsdlib::NumericTypes::Type::UInt8, ebsdlib::NumericTypes::Type::UInt8, ebsdlib::NumericTypes::Type::UInt8,
//                                                                    ebsdlib::NumericTypes::Type::UInt8, ebsdlib::NumericTypes::Type::UInt8, ebsdlib::NumericTypes::Type::UInt8};
//

// clang-format off
std::array<CrcFieldDefinition, 13> k_FieldDefinitions{CrcFieldDefinition{1, "Phase", ebsdlib::NumericTypes::Type::UInt8},
                                                      CrcFieldDefinition{4, "X", ebsdlib::NumericTypes::Type::Float},
                                                      CrcFieldDefinition{4, "Y", ebsdlib::NumericTypes::Type::Float},
                                                      CrcFieldDefinition{4, "phi1", ebsdlib::NumericTypes::Type::Float},
                                                      CrcFieldDefinition{4, "Phi", ebsdlib::NumericTypes::Type::Float},
                                                      CrcFieldDefinition{4, "phi2", ebsdlib::NumericTypes::Type::Float},
                                                      CrcFieldDefinition{4, "MAD", ebsdlib::NumericTypes::Type::Float},
                                                      CrcFieldDefinition{1, "BC", ebsdlib::NumericTypes::Type::UInt8},
                                                      CrcFieldDefinition{1, "BS", ebsdlib::NumericTypes::Type::UInt8},
                                                      CrcFieldDefinition{1, "Unknown", ebsdlib::NumericTypes::Type::UInt8},
                                                      CrcFieldDefinition{1, "Bands", ebsdlib::NumericTypes::Type::UInt8},
                                                      CrcFieldDefinition{1, "Error", ebsdlib::NumericTypes::Type::UInt8},
                                                      CrcFieldDefinition{4, "ReliabilityIndex", ebsdlib::NumericTypes::Type::Int32}};
// clang-format on

// Read a complete Scan Point
// loop over parserDefinitions->parse(buffer, currentIndex);

template <typename T>
T getFieldValue(inipp::Ini<char>& iniParser, const std::string& section, const std::string& key)
{
  T value;
  inipp::get_value(iniParser.sections[section], key, value);
  return value;
}

std::vector<float> getLatticeConstants(inipp::Ini<char>& iniParser, const std::string& section)
{
  float a;
  float b;
  float c;
  float alpha;
  float beta;
  float gamma;

  inipp::get_value(iniParser.sections[section], "a", a);
  inipp::get_value(iniParser.sections[section], "b", b);
  inipp::get_value(iniParser.sections[section], "c", c);
  inipp::get_value(iniParser.sections[section], "alpha", alpha);
  inipp::get_value(iniParser.sections[section], "beta", beta);
  inipp::get_value(iniParser.sections[section], "gamma", gamma);

  return {a, b, c, alpha, beta, gamma};
}

void ParseAndStoreHeaderEntry(inipp::Ini<char>& iniParser, const std::string& section, const std::string& key, std::map<std::string, EbsdHeaderEntry::Pointer>& m_HeaderMap)
{
  EbsdHeaderEntry::Pointer p1 = m_HeaderMap[key];
  auto value = getFieldValue<std::string>(iniParser, section, key);
  p1->parseValue(value);
}

} // namespace

// -----------------------------------------------------------------------------
CprReader::CprReader()
{

  // Initialize the map of header key to header value
  m_HeaderMap[ebsdlib::Ctf::ChannelTextFile] = CtfStringHeaderEntry::NewEbsdHeaderEntry(ebsdlib::Ctf::ChannelTextFile);
  m_HeaderMap[ebsdlib::Ctf::Prj] = CtfStringHeaderEntry::NewEbsdHeaderEntry(ebsdlib::Ctf::Prj);
  m_HeaderMap[ebsdlib::Ctf::Author] = CtfStringHeaderEntry::NewEbsdHeaderEntry(ebsdlib::Ctf::Author);
  m_HeaderMap[ebsdlib::Ctf::JobMode] = CtfStringHeaderEntry::NewEbsdHeaderEntry(ebsdlib::Ctf::JobMode);

  m_HeaderMap[ebsdlib::Ctf::xCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::XCells);
  m_HeaderMap[ebsdlib::Ctf::yCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::YCells);
  //  m_HeaderMap[ebsdlib::Ctf::ZCells] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::ZCells);
  m_HeaderMap[ebsdlib::Ctf::GridDistX] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::XStep);
  m_HeaderMap[ebsdlib::Ctf::GridDistY] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::YStep);
  //  m_HeaderMap[ebsdlib::Ctf::ZStep] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::ZStep);
  //  m_HeaderMap[ebsdlib::Ctf::AcqE1] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::AcqE1);
  //  m_HeaderMap[ebsdlib::Ctf::AcqE2] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::AcqE2);
  //  m_HeaderMap[ebsdlib::Ctf::AcqE3] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::AcqE3);
  //  m_HeaderMap[ebsdlib::Ctf::Euler] = CtfStringHeaderEntry::NewEbsdHeaderEntry(ebsdlib::Ctf::Euler);

  m_HeaderMap[ebsdlib::Ctf::Magnification] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::Mag);
  m_HeaderMap[ebsdlib::Ctf::Coverage] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::Coverage);
  m_HeaderMap[ebsdlib::Ctf::Device] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::Device);
  m_HeaderMap[ebsdlib::Ctf::kV] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::kV);
  m_HeaderMap[ebsdlib::Ctf::TiltAngle] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::TiltAngle);
  m_HeaderMap[ebsdlib::Ctf::TiltAxis] = CtfHeaderEntry<float, FloatHeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::TiltAxis);
  m_HeaderMap[ebsdlib::Ctf::NumPhases] = CtfHeaderEntry<int, Int32HeaderParser>::NewEbsdHeaderEntry(ebsdlib::Ctf::NumPhases);

  setXCells(0);
  setYCells(0);
  // setZCells(1);
}

// -----------------------------------------------------------------------------
CprReader::~CprReader() = default;

// -----------------------------------------------------------------------------
void* CprReader::getPointerByName(const std::string& featureName)
{
  void* ptr = nullptr;
  if(m_NamePointerMap.find(featureName) != m_NamePointerMap.end())
  {
    ptr = m_NamePointerMap[featureName]->getVoidPointer();
  }
  return ptr;
}

std::vector<std::string> CprReader::getPointerNames() const
{
  std::vector<std::string> names;
  for(const auto& entry : m_NamePointerMap)
  {
    names.push_back(entry.first);
  }
  return names;
}

std::map<std::string, ebsdlib::NumericTypes::Type> CprReader::getPointerTypes() const
{
  std::map<std::string, ebsdlib::NumericTypes::Type> types;
  for(const auto& entry : m_NamePointerMap)
  {
    types[entry.first] = entry.second->getNumericType();
  }
  return types;
}

// -----------------------------------------------------------------------------
ebsdlib::NumericTypes::Type CprReader::getPointerType(const std::string& featureName)
{
  // std::cout << "featureName: " << featureName << std::endl;
  if(featureName == ebsdlib::Ctf::ReliabilityIndex)
  {
    return ebsdlib::NumericTypes::Type::Int32;
  }
  if(featureName == ebsdlib::Ctf::Phase)
  {
    return ebsdlib::NumericTypes::Type::UInt8;
  }
  if(featureName == ebsdlib::Ctf::X)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::Y)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::Z)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::Bands)
  {
    return ebsdlib::NumericTypes::Type::UInt8;
  }
  if(featureName == ebsdlib::Ctf::Error)
  {
    return ebsdlib::NumericTypes::Type::UInt8;
  }
  if(featureName == ebsdlib::Ctf::phi1)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::Phi)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::phi2)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::MAD)
  {
    return ebsdlib::NumericTypes::Type::Float;
  }
  if(featureName == ebsdlib::Ctf::BC)
  {
    return ebsdlib::NumericTypes::Type::UInt8;
  }
  if(featureName == ebsdlib::Ctf::BS)
  {
    return ebsdlib::NumericTypes::Type::UInt8;
  }
  if(featureName == ebsdlib::Ctf::GrainIndex)
  {
    return ebsdlib::NumericTypes::Type::Int32;
  }
  if(featureName == ebsdlib::Ctf::GrainRandomColourR)
  {
    return ebsdlib::NumericTypes::Type::Int32;
  }
  if(featureName == ebsdlib::Ctf::GrainRandomColourG)
  {
    return ebsdlib::NumericTypes::Type::Int32;
  }
  if(featureName == ebsdlib::Ctf::GrainRandomColourB)
  {
    return ebsdlib::NumericTypes::Type::Int32;
  }
  // std::cout << "THIS IS NOT GOOD. Feature name: " << featureName << " was not found in the list" << std::endl;
  return ebsdlib::NumericTypes::Type::UnknownNumType;
}

//
// -----------------------------------------------------------------------------
int CprReader::getTypeSize(const std::string& featureName)
{
  if(featureName == ebsdlib::Ctf::ReliabilityIndex)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::Phase)
  {
    return 1;
  }
  if(featureName == ebsdlib::Ctf::X)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::Y)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::Z)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::Bands)
  {
    return 1;
  }
  if(featureName == ebsdlib::Ctf::Error)
  {
    return 1;
  }
  if(featureName == ebsdlib::Ctf::phi1)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::Phi)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::phi2)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::MAD)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::BC)
  {
    return 1;
  }
  if(featureName == ebsdlib::Ctf::BS)
  {
    return 1;
  }
  if(featureName == ebsdlib::Ctf::GrainIndex)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::GrainRandomColourR)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::GrainRandomColourG)
  {
    return 4;
  }
  if(featureName == ebsdlib::Ctf::GrainRandomColourB)
  {
    return 4;
  }
  return 0;
}

// -----------------------------------------------------------------------------
int CprReader::readHeaderOnly()
{
  int err = 0;

  try
  {
    if(!std::filesystem::exists(getFileName()))
    {
      setErrorMessage(".cpr file does not exist");
      setErrorCode(-11);
      return getErrorCode();
    }
  } catch(std::exception& e)
  {
    setErrorMessage(".cpr std::file_system threw an exception trying to determine if the .cpr file exists.");
    setErrorCode(-10);
    return getErrorCode();
  }
  // Open the file
  std::ifstream is(getFileName());

  // Create an INI parser object and parse the file
  inipp::Ini<char> ini;
  ini.parse(is);

  // Extract out the Phases
  {
    const std::string sectionName("Phases");
    int phaseCount = -1;
    inipp::get_value(ini.sections[sectionName], "Count", phaseCount);
    EbsdHeaderEntry::Pointer p1 = m_HeaderMap[ebsdlib::Ctf::NumPhases];
    auto value = getFieldValue<std::string>(ini, sectionName, ebsdlib::Ctf::Count);
    p1->parseValue(value);
    // std::cout << "Phase Count: " << phaseCount << std::endl;
    for(int phaseIndex = 1; phaseIndex <= phaseCount; phaseIndex++)
    {
      std::stringstream sectionNameStrm;
      sectionNameStrm << "Phase" << phaseIndex;

      CtfPhase::Pointer phase = CtfPhase::New();
      phase->setPhaseIndex(phaseIndex);

      // Phase Name
      phase->setPhaseName(getFieldValue<std::string>(ini, sectionNameStrm.str(), "StructureName"));

      // Lattice Constants
      phase->setLatticeConstants(getLatticeConstants(ini, sectionNameStrm.str()));

      // Laue Group
      phase->setLaueGroup(static_cast<ebsdlib::Ctf::LaueGroupTable>(getFieldValue<int>(ini, sectionNameStrm.str(), "LaueGroup")));

      // Comment
      phase->setComment(getFieldValue<std::string>(ini, sectionNameStrm.str(), "Reference"));

      // Space Group
      phase->setSpaceGroup(getFieldValue<int>(ini, sectionNameStrm.str(), "SpaceGroup"));

      // internal 1
      phase->setInternal1(getFieldValue<std::string>(ini, sectionNameStrm.str(), "ID1"));

      // Internal 2
      phase->setInternal2(getFieldValue<std::string>(ini, sectionNameStrm.str(), "ID2"));

      m_PhaseVector.push_back(phase);
    }
  }

  {
    const std::string sectionName("Job");
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::Magnification, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::Coverage, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::Device, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::kV, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::TiltAngle, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::TiltAxis, m_HeaderMap);

    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::GridDistX, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::GridDistY, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::xCells, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::yCells, m_HeaderMap);

    setNumberOfElements(getXCells() * getYCells());
  }

  {
    const std::string sectionName("General");
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::Author, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::JobMode, m_HeaderMap);
  }

  {
    const std::string sectionName("General");
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::Author, m_HeaderMap);
    ParseAndStoreHeaderEntry(ini, sectionName, ebsdlib::Ctf::JobMode, m_HeaderMap);
  }

  return err;
}

// -----------------------------------------------------------------------------
int CprReader::readFile()
{

  int error = readHeaderOnly();

  if(error < 0)
  {
    return error;
  }
  setErrorCode(0);

  if(getXStep() == 0.0 || getYStep() == 0.0f)
  {
    setErrorMessage("Either the X Step or Y Step was Zero (0.0) which is NOT allowed. Please update the CTF file header with appropriate values.");
    setErrorCode(-102);
    return getErrorCode();
  }

  if(getXCells() == 0 || getYCells() == 0)
  {
    setErrorMessage("Either the X Cells or Y Cells was Zero (0) which is NOT allowed. Please update the CTF file header with appropriate values.");
    setErrorCode(-103);
    return getErrorCode();
  }

  std::vector<CrcDataParser> scanPointParsers = createFieldParsers(getFileName());
  if(scanPointParsers.empty())
  {
    setErrorMessage("The .cpr file did not contain any [Fields] data to parse");
    setErrorCode(-100);
    return getErrorCode();
  }

  size_t bufferSize = 0;
  // Allocate all memory
  size_t totalScanPoints = getNumberOfElements();
  bool didAllocate = false;
  int index = 0;
  for(auto& parser : scanPointParsers)
  {
    bufferSize += parser.FieldDefinition.ByteSize;
    if(m_NamePointerMap.find(parser.FieldDefinition.FieldName) != m_NamePointerMap.end())
    {
      std::stringstream ss;
      ss << "Column Header '" << parser.FieldDefinition.FieldName << "' has been found multiple times in the Header Row. Please check the CTF file for mistakes.";
      setErrorMessage(ss.str());
      setErrorCode(-110);
      return getErrorCode();
    }
    if(ebsdlib::NumericTypes::Type::UInt8 == parser.FieldDefinition.numericType)
    {
      UInt8Parser::Pointer dparser = UInt8Parser::New(nullptr, totalScanPoints, parser.FieldDefinition.FieldName, index);
      didAllocate = dparser->allocateArray(totalScanPoints);
      // Q_ASSERT_X(dparser->getVoidPointer() != nullptr, __FILE__, "Could not allocate memory for Integer data in CTF File.");
      if(didAllocate)
      {
        ::memset(dparser->getVoidPointer(), 0xAB, sizeof(uint8_t) * totalScanPoints);
        m_NamePointerMap[parser.FieldDefinition.FieldName] = dparser;
        parser.destinationPtr = reinterpret_cast<uint8_t*>(dparser->getVoidPointer());
      }
    }
    else if(ebsdlib::NumericTypes::Type::Float == parser.FieldDefinition.numericType)
    {
      FloatParser::Pointer dparser = FloatParser::New(nullptr, totalScanPoints, parser.FieldDefinition.FieldName, index);
      didAllocate = dparser->allocateArray(totalScanPoints);
      // Q_ASSERT_X(dparser->getVoidPointer() != nullptr, __FILE__, "Could not allocate memory for Integer data in CTF File.");
      if(didAllocate)
      {
        ::memset(dparser->getVoidPointer(), 0xAB, sizeof(float) * totalScanPoints);
        m_NamePointerMap[parser.FieldDefinition.FieldName] = dparser;
        parser.destinationPtr = reinterpret_cast<uint8_t*>(dparser->getVoidPointer());
      }
    }
    else if(ebsdlib::NumericTypes::Type::Int32 == parser.FieldDefinition.numericType)
    {
      Int32Parser::Pointer dparser = Int32Parser::New(nullptr, totalScanPoints, parser.FieldDefinition.FieldName, index);
      didAllocate = dparser->allocateArray(totalScanPoints);
      // Q_ASSERT_X(dparser->getVoidPointer() != nullptr, __FILE__, "Could not allocate memory for Integer data in CTF File.");
      if(didAllocate)
      {
        ::memset(dparser->getVoidPointer(), 0xAB, sizeof(int32_t) * totalScanPoints);
        m_NamePointerMap[parser.FieldDefinition.FieldName] = dparser;
        parser.destinationPtr = reinterpret_cast<uint8_t*>(dparser->getVoidPointer());
      }
    }
    else
    {
      std::stringstream ss;
      ss << "Column Header '" << parser.FieldDefinition.FieldName << "' is not a recognized column for this CRC Files. Please recheck your .cpr file and report this error to the DREAM3D developers.";
      setErrorMessage(ss.str());
      setErrorCode(-107);
      return getErrorCode();
    }

    if(!didAllocate)
    {
      std::stringstream ss;
      ss << "The CTF reader could not allocate memory for the data. Check the header for the number of X, Y and Z Cells.";
      ss << "\n X Cells: " << getXCells();
      ss << "\n Y Cells: " << getYCells();
      // ss << "\n Z Cells: " << getzCells();
      ss << "\n Total Scan Points: " << totalScanPoints;
      setErrorMessage(ss.str());
      setErrorCode(-106); // Could not allocate the memory
      return getErrorCode();
    }
    index++;
  }

  // Make a buffer
  std::vector<uint8_t> buffer(bufferSize, 0);

  fs::path filePath(getFileName());
  // Replace the extension with ".crc"
  fs::path crcPath = filePath.replace_extension(".crc");
  // Open the file for reading
  FILE* inputFilePtr = std::fopen(crcPath.string().c_str(), "rb");
  if(inputFilePtr == nullptr)
  {
    std::stringstream ss;
    ss << "The input file '" << crcPath.string() << "' probably does not exist. Nullptr returns when opening the file";
    setErrorMessage(ss.str());
    setErrorCode(-108);
    return getErrorCode();
  }

  // Read through the file
  for(size_t idx = 0; idx < totalScanPoints; idx++)
  {
    size_t elementsRead = std::fread(buffer.data(), 1, bufferSize, inputFilePtr);
    if(elementsRead != buffer.size())
    {
      fclose(inputFilePtr);
      std::stringstream ss;
      ss << "Error when reading the .crc file at file offset " << (idx * bufferSize) << ". We should have read " << bufferSize << " bytes but we only read " << elementsRead << " bytes.";
      setErrorMessage(ss.str());
      setErrorCode(-1666);
      return getErrorCode();
    }
    for(const auto& parser : scanPointParsers)
    {
      parser.parse(buffer.data(), idx);
    }
  }
  fclose(inputFilePtr);

  return getErrorCode();
}

// -----------------------------------------------------------------------------
std::vector<CrcDataParser> CprReader::createFieldParsers(const std::string& filename)
{
  try
  {
    if(!std::filesystem::exists(filename))
    {
      return {};
    }
  } catch(std::exception& e)
  {
    return {};
  }
  // Open the file
  std::ifstream is(filename);

  // Create an INI parser object and parse the file
  inipp::Ini<char> ini;
  ini.parse(is);

  size_t fieldCount = getFieldValue<int>(ini, "Fields", "Count");
  // std::cout << "Field Count: " << fieldCount << std::endl;
  std::vector<CrcDataParser> fieldOrder(fieldCount + 1);

  // Insert the Phase as the first field. I guess?
  fieldOrder[0] = {k_FieldDefinitions[0], nullptr, 0};
  size_t currentOffset = 1;
  size_t edxCurrentOffset = 1;

  for(int i = 1; i <= fieldCount; i++)
  {
    std::stringstream fieldNameStrm;
    fieldNameStrm << "Field" << i;
    auto fieldIndex = getFieldValue<size_t>(ini, "Fields", fieldNameStrm.str());
    if(fieldIndex < 13)
    {
      CrcFieldDefinition fieldDefinition = k_FieldDefinitions[fieldIndex];
      fieldOrder[i] = {fieldDefinition, nullptr, currentOffset};
      currentOffset += fieldDefinition.ByteSize;
    }
    // Handle EDX (XRay EDS) data which seems to have field index values > 100.
    if(fieldIndex > 100)
    {
      fieldNameStrm.str(""); // Clear the stringstream buffer
      fieldNameStrm << "Window" << edxCurrentOffset;
      auto elementName = getFieldValue<std::string>(ini, "EDX Windows", fieldNameStrm.str());
      if(elementName.empty())
      {
        fieldNameStrm.str(""); // Clear the stringstream buffer
        fieldNameStrm << "Field" << i;
        elementName = fieldNameStrm.str();
      }
      fieldOrder[i].FieldDefinition = {4, elementName, ebsdlib::NumericTypes::Type::Float};
      fieldOrder[i].destinationPtr = nullptr;
      fieldOrder[i].readOffset = currentOffset;
      currentOffset += 4;
      edxCurrentOffset++;
    }
  }

  return fieldOrder;
}

#if 0
#define PRINT_HTML_TABLE_ROW(p)                                                                                                                                                                        \
  std::cout << "<tr>\n    <td>" << p->getKey() << "</td>\n    <td>" << p->getHDFType() << "</td>\n";                                                                                                   \
  std::cout << "    <td colspan=\"2\"> Contains value for the header entry " << p->getKey() << "</td>\n</tr>" << std::endl;
#else
#define PRINT_HTML_TABLE_ROW(p)
#endif

// -----------------------------------------------------------------------------
int CprReader::getXDimension()
{
  return getXCells();
}

// -----------------------------------------------------------------------------
void CprReader::setXDimension(int xdim)
{
  setXCells(xdim);
}

// -----------------------------------------------------------------------------
int CprReader::getYDimension()
{
  return getYCells();
}

// -----------------------------------------------------------------------------
void CprReader::setYDimension(int ydim)
{
  setYCells(ydim);
}

// -----------------------------------------------------------------------------
std::vector<std::string> CprReader::getColumnNames()
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
void CprReader::printHeader(std::ostream& out)
{
  std::cout << "-------------------- CprReader Header Values --------------------" << std::endl;
  CTF_PRINT_QSTRING(Channel, out)
  CTF_PRINT_QSTRING(Prj, out)
  CTF_PRINT_QSTRING(Author, out)
  CTF_PRINT_QSTRING(JobMode, out)
  CTF_PRINT_HEADER_VALUE(XCells, out)
  CTF_PRINT_HEADER_VALUE(YCells, out)
  CTF_PRINT_HEADER_VALUE(XStep, out)
  CTF_PRINT_HEADER_VALUE(YStep, out)
  //  CTF_PRINT_HEADER_VALUE(AcqE1, out);
  //  CTF_PRINT_HEADER_VALUE(AcqE2, out);
  //  CTF_PRINT_HEADER_VALUE(AcqE3, out);
  //  CTF_PRINT_QSTRING(Euler, out);
  CTF_PRINT_HEADER_VALUE(Mag, out)
  CTF_PRINT_HEADER_VALUE(Coverage, out)
  CTF_PRINT_HEADER_VALUE(Device, out)
  CTF_PRINT_HEADER_VALUE(KV, out)
  CTF_PRINT_HEADER_VALUE(TiltAngle, out)
  CTF_PRINT_HEADER_VALUE(TiltAxis, out)
  CTF_PRINT_HEADER_VALUE(NumPhases, out)
  int nPhases = getNumPhases();
  for(int p = 0; p < nPhases; ++p)
  {
    out << "### Phase " << p << std::endl;
    m_PhaseVector[p]->printSelf(out);
  }

  std::cout << "----------------------------------------" << std::endl;
}

// -----------------------------------------------------------------------------
std::string CprReader::getNameOfClass() const
{
  return {"CprReader"};
}

// -----------------------------------------------------------------------------
std::string CprReader::ClassName()
{
  return {"CprReader"};
}
