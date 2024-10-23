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
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Channel, EbsdLib::Ctf::ChannelTextFile)
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Prj, EbsdLib::Ctf::Prj)
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Author, EbsdLib::Ctf::Author)
  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, JobMode, EbsdLib::Ctf::JobMode)

  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, YCells, EbsdLib::Ctf::xCells)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, XCells, EbsdLib::Ctf::yCells)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, ZCells, EbsdLib::Ctf::ZCells)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, XStep, EbsdLib::Ctf::GridDistX)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, YStep, EbsdLib::Ctf::GridDistY)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, ZStep, EbsdLib::Ctf::ZStep)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, AcqE1, EbsdLib::Ctf::AcqE1)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, AcqE2, EbsdLib::Ctf::AcqE2)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, AcqE3, EbsdLib::Ctf::AcqE3)
  //  EBSDHEADER_INSTANCE_PROPERTY(CtfStringHeaderEntry, std::string, Euler, EbsdLib::Ctf::Euler)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, Mag, EbsdLib::Ctf::Magnification)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, Coverage, EbsdLib::Ctf::Coverage)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, Device, EbsdLib::Ctf::Device)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, KV, EbsdLib::Ctf::kV)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, TiltAngle, EbsdLib::Ctf::TiltAngle)
  EBSDHEADER_INSTANCE_PROPERTY(CtfFloatHeaderType, float, TiltAxis, EbsdLib::Ctf::TiltAxis)
  EBSDHEADER_INSTANCE_PROPERTY(CtfIntHeaderType, int, NumPhases, EbsdLib::Ctf::NumPhases)
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
  std::map<std::string, EbsdLib::NumericTypes::Type> getPointerTypes() const;

  /**
   * @brief Returns an enumeration value that depicts the numerical
   * primitive type that the data is stored as (Int, Float, etc).
   * @param featureName The name of the feature.
   */
  EbsdLib::NumericTypes::Type getPointerType(const std::string& featureName) override;

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

private:
  int m_SingleSliceRead = -1;

  std::map<std::string, DataParser::Pointer> m_NamePointerMap;

public:
  CprReader(const CprReader&) = delete;            // Copy Constructor Not Implemented
  CprReader(CprReader&&) = delete;                 // Move Constructor Not Implemented
  CprReader& operator=(const CprReader&) = delete; // Copy Assignment Not Implemented
  CprReader& operator=(CprReader&&) = delete;      // Move Assignment Not Implemented
};
