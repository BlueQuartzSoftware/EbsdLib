//
// Created by Michael Jackson on 10/22/24.
//
#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CtfConstants.h"
#include "CtfHeaderEntry.h"
#include "CtfPhase.h"
#include "DataParser.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdSetGetMacros.h"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/IO/EbsdReader.h"

#define CPR_READER_PTR_PROP(name, var, type)                                                                                                                                                           \
  type* get##name##Pointer()                                                                                                                                                                           \
  {                                                                                                                                                                                                    \
    return static_cast<type*>(getPointerByName(#var));                                                                                                                                                 \
  }
namespace ebsdlib
{

struct CrcFieldDefinition
{
  size_t ByteSize = 0;
  std::string FieldName;
  ebsdlib::NumericTypes::Type numericType = ebsdlib::NumericTypes::Type::UnknownNumType;
};

struct CrcDataParser
{
  CrcFieldDefinition FieldDefinition = {};
  uint8_t* destinationPtr = nullptr;
  size_t readOffset = 0;

  void parse(uint8_t* buffer, size_t destinationIndex) const
  {
    // calculate the proper pointer offset
    uint8_t* finalPtr = destinationPtr + (destinationIndex * FieldDefinition.ByteSize);
    // Copy the bytes from the buffer into the final destination
    std::memcpy(finalPtr, buffer + readOffset, FieldDefinition.ByteSize);
  }
};

/**
 * @brief This class can parse an Oxford Instruments .cpr/.crc file combination.
 *
 * This class should successfully be able to read files even all phase types and including EDX data.
 */
class EbsdLib_EXPORT CprReader : public EbsdReader
{
public:
  CprReader();
  ~CprReader() override;

  /**
   * @brief Returns the name of the class for CtfReader
   */
  std::string getNameOfClass() const override;

  /**
   * @brief Returns the name of the class for CtfReader
   */
  static std::string ClassName();

  using CtfIntHeaderType = CtfHeaderEntry<int, Int32HeaderParser>;
  using CtfFloatHeaderType = CtfHeaderEntry<float, FloatHeaderParser>;
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Channel, ebsdlib::Ctf::ChannelTextFile)
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Prj, ebsdlib::Ctf::Prj)
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Author, ebsdlib::Ctf::Author)
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, JobMode, ebsdlib::Ctf::JobMode)

  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, XCells, ebsdlib::Ctf::xCells)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, YCells, ebsdlib::Ctf::yCells)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, ZCells, ebsdlib::Ctf::ZCells)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, XStep, ebsdlib::Ctf::GridDistX)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, YStep, ebsdlib::Ctf::GridDistY)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, ZStep, ebsdlib::Ctf::ZStep)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, AcqE1, ebsdlib::Ctf::AcqE1)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, AcqE2, ebsdlib::Ctf::AcqE2)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, AcqE3, ebsdlib::Ctf::AcqE3)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Euler, ebsdlib::Ctf::Euler)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, Mag, ebsdlib::Ctf::Magnification)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, Coverage, ebsdlib::Ctf::Coverage)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, Device, ebsdlib::Ctf::Device)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, KV, ebsdlib::Ctf::kV)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, TiltAngle, ebsdlib::Ctf::TiltAngle)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, TiltAxis, ebsdlib::Ctf::TiltAxis)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, NumPhases, ebsdlib::Ctf::NumPhases)
  EBSD_INSTANCE_PROPERTY(std::vector<CtfPhase::Pointer>, PhaseVector)

  CPR_READER_PTR_PROP(Phase, Phase, uint8_t)
  CPR_READER_PTR_PROP(X, X, float)
  CPR_READER_PTR_PROP(Y, Y, float)
  // CPR_READER_PTR_PROP(Z, Z, float)
  CPR_READER_PTR_PROP(BandCount, Bands, uint8_t)
  CPR_READER_PTR_PROP(Error, Error, uint8_t)
  CPR_READER_PTR_PROP(Euler1, phi1, float)
  CPR_READER_PTR_PROP(Euler2, Phi, float)
  CPR_READER_PTR_PROP(Euler3, phi2, float)
  CPR_READER_PTR_PROP(MeanAngularDeviation, MAD, float)
  CPR_READER_PTR_PROP(BandContrast, BC, uint8_t)
  CPR_READER_PTR_PROP(BandSlope, BS, uint8_t)
  CPR_READER_PTR_PROP(ReliabilityIndex, ReliabilityIndex, int32_t)

  /**
   * @brief Returns the pointer to the data for a given feature
   * @param featureName The name of the feature to return the pointer to.
   */
  void* getPointerByName(const std::string& featureName) override;

  /**
   * @brief Returns the string names of all the arrays that were allocated during the reading of the file
   * @return
   */
  std::vector<std::string> getPointerNames() const;

  /**
   * @brief Returns the types of data that each array holds.
   * @return
   */
  std::map<std::string, ebsdlib::NumericTypes::Type> getPointerTypes() const;

  /**
   * @brief Returns an enumeration value that depicts the numerical
   * primitive type that the data is stored as (Int, Float, etc).
   * @param featureName The name of the feature.
   */
  ebsdlib::NumericTypes::Type getPointerType(const std::string& featureName) override;

  int getTypeSize(const std::string& featureName);

  // DataParser::Pointer getParser(const std::string& featureName, void* ptr, size_t size);

  std::vector<std::string> getColumnNames();

  /**
   * @brief Reads the complete HKL .ctf file.
   * @return 1 on success
   */
  int readFile() override;

  /**
   * @brief Reads ONLY the header portion of the HKL .ctf file
   * @return 1 on success
   */
  int readHeaderOnly() override;

  // void readOnlySliceIndex(int slice);

  int getXDimension() override;
  void setXDimension(int xdim) override;
  int getYDimension() override;
  void setYDimension(int ydim) override;

  void printHeader(std::ostream& out);

  std::vector<ebsdlib::CrcDataParser> createFieldParsers(const std::string& filename);

private:
  int m_SingleSliceRead = -1;

  std::map<std::string, DataParser::Pointer> m_NamePointerMap;

public:
  CprReader(const CprReader&) = delete;            // Copy Constructor Not Implemented
  CprReader(CprReader&&) = delete;                 // Move Constructor Not Implemented
  CprReader& operator=(const CprReader&) = delete; // Copy Assignment Not Implemented
  CprReader& operator=(CprReader&&) = delete;      // Move Assignment Not Implemented
};
} // namespace ebsdlib
