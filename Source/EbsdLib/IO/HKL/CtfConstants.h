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

#pragma once

#include <string>
#include <vector>

#define DECLARE_STRING_CONST(var) const std::string var(#var);

namespace EbsdLib
{

namespace H5Aztec
{
const std::string Header("Header");
const std::string Phases("Phases");
const std::string OriginalHeader("OriginalHeader");
const std::string OriginalFile("OriginalFile");
const std::string Data("Data");

const std::string FileVersionStr("FileVersion");
const unsigned int FileVersion = 5;
const std::string EbsdLibVersionStr("EbsdLibVersion");
} // namespace H5Aztec

namespace Ctf
{

enum LaueGroupTable
{
  LG_Triclinic = 1,
  LG_Monoclinic = 2,
  LG_Orthorhombic = 3,
  LG_Tetragonal_Low = 4,
  LG_Tetragonal_High = 5,
  LG_Trigonal_Low = 6,
  LG_Trigonal_High = 7,
  LG_Hexagonal_Low = 8,
  LG_Hexagonal_High = 9,
  LG_Cubic_Low = 10,
  LG_Cubic_High = 11,
  UnknownSymmetry = 12
};

class LaueGroupStrings
{
public:
  LaueGroupStrings()
  {
    m_Values.push_back("Unknown(0)");
    m_Values.push_back("Triclinic -1");
    m_Values.push_back("Monoclinic 2/m");
    m_Values.push_back("Orthorhombic mmm");
    m_Values.push_back("Tetragonal-Low 4/m");
    m_Values.push_back("Tetragonal-High 4/mmm");
    m_Values.push_back("Trigonal-Low -3");
    m_Values.push_back("Trigonal-High -3m");
    m_Values.push_back("Hexagonal-Low 6/m");
    m_Values.push_back("Hexagonal-High 6/mmm");
    m_Values.push_back("Cubic-Low m3");
    m_Values.push_back("Cubic-High m3m");
    m_Values.push_back("Unknown(12)");
  }
  virtual ~LaueGroupStrings() = default;

  std::string getString(LaueGroupTable i)
  {
    return m_Values[static_cast<size_t>(i)];
  }

private:
  std::vector<std::string> m_Values;
};

class SpaceGroupQuery
{
public:
  SpaceGroupQuery() = default;
  ~SpaceGroupQuery() = default;

  static bool isCubic(int spaceGroup)
  {
    return (spaceGroup >= 195 && spaceGroup <= 230);
  }
  static bool isHexagonal(int spaceGroup)
  {
    return (spaceGroup >= 168 && spaceGroup <= 194);
  }
};

const std::string FileExt("ctf");

const std::string Manufacturer("HKL");

// These are Header related
const std::string ChannelTextFile("Channel Text File");
DECLARE_STRING_CONST(Prj)
DECLARE_STRING_CONST(Author)
DECLARE_STRING_CONST(JobMode)
DECLARE_STRING_CONST(XCells)
DECLARE_STRING_CONST(YCells)

DECLARE_STRING_CONST(GridDistX)
DECLARE_STRING_CONST(GridDistY)
DECLARE_STRING_CONST(xCells)
DECLARE_STRING_CONST(yCells)
DECLARE_STRING_CONST(Count)

DECLARE_STRING_CONST(ZCells)
DECLARE_STRING_CONST(XStep)
DECLARE_STRING_CONST(YStep)
DECLARE_STRING_CONST(ZStep)
DECLARE_STRING_CONST(AcqE1)
DECLARE_STRING_CONST(AcqE2)
DECLARE_STRING_CONST(AcqE3)
DECLARE_STRING_CONST(Euler)
DECLARE_STRING_CONST(Mag)
DECLARE_STRING_CONST(Magnification)
DECLARE_STRING_CONST(Coverage)
DECLARE_STRING_CONST(Device)
DECLARE_STRING_CONST(KV)
DECLARE_STRING_CONST(kV)
DECLARE_STRING_CONST(TiltAngle)
DECLARE_STRING_CONST(TiltAxis)
const std::string NumPhases("Phases");

// These are phase related
DECLARE_STRING_CONST(LatticeConstants)
DECLARE_STRING_CONST(PhaseName)
DECLARE_STRING_CONST(LaueGroup)
DECLARE_STRING_CONST(SpaceGroup)
DECLARE_STRING_CONST(Internal1)
DECLARE_STRING_CONST(Internal2)
DECLARE_STRING_CONST(Comment)

/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
// These are the names of the Data Columns
// DO NOT CHANGE ANY OF THESE CONSTANTS, IT WILL MESS UP THE CTF Parser
DECLARE_STRING_CONST(ReliabilityIndex)
DECLARE_STRING_CONST(Phase)
DECLARE_STRING_CONST(X)
DECLARE_STRING_CONST(Y)
DECLARE_STRING_CONST(Z)
DECLARE_STRING_CONST(Bands)
DECLARE_STRING_CONST(Error)
DECLARE_STRING_CONST(Euler1)
DECLARE_STRING_CONST(Euler2)
DECLARE_STRING_CONST(Euler3)

DECLARE_STRING_CONST(phi1)
DECLARE_STRING_CONST(Phi)
DECLARE_STRING_CONST(phi2)

DECLARE_STRING_CONST(MAD)
DECLARE_STRING_CONST(BC)         // BC
DECLARE_STRING_CONST(BS)         // BS
DECLARE_STRING_CONST(GrainIndex) // FeatureIndex
DECLARE_STRING_CONST(GrainRandomColourR)
DECLARE_STRING_CONST(GrainRandomColourG)
DECLARE_STRING_CONST(GrainRandomColourB)
/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
/* ******************************************************************************************************** */

} // End namespace Ctf

namespace CtfFile
{
const std::string Phases("Phases");
const std::string EulerAngles("EulerAngles");
const std::string CrystalStructures("CrystalStructures");
const std::string MaterialName("MaterialName");
const std::string LatticeConstants("LatticeConstants");
const std::string BravaisLattice("BravaisLattice");
} // namespace CtfFile

//-----------------------------------------------------------------------------
// https://github.com/oinanoanalysis/h5oina/blob/master/H5OINAFile.md#ebsd-data
//-----------------------------------------------------------------------------
namespace H5OINA
{

const std::string H5FileExt("h5oina");
const std::string Index("Index");
const std::string FormatVersion("Format Version");     // String 2.0 is the earliest that I seem to have
const std::string Manufacturer("Manufacturer");        // String
const std::string SoftwareVersion("Software Version"); // String

const std::string FormatVersion_2("2.0");
const std::string FormatVersion_3("3.0");
const std::string FormatVersion_4("4.0");
const std::string FormatVersion_5("5.0");

//-----------------------------------------------------------------------------
// These are for header names in the hdf5 file
// Format Version: 2
//-----------------------------------------------------------------------------
const std::string Header("Header");
const std::string AcquisitionData("Acquisition Date");
const std::string AcquisitionSpeed("Acquisition Speed");
const std::string AcquisitionTime("Acquisition Time");
const std::string BeamVoltage("Beam Voltage");
const std::string DetectorOrientationEuler("Detector Orientation Euler");
const std::string Magnification("Magnification");
const std::string ProjectFile("Project File");
const std::string ProjectLabel("Project Label");
const std::string ProjectNotes("Project Notes");
const std::string ScanningRotationAngle("Scanning Rotation Angle");
const std::string SiteNotes("Site Notes");
const std::string SpecimenOrientationEuler("Specimen Orientation Euler");
const std::string TiltAngle("Tilt Angle");
const std::string TiltAxis("Tilt Axis");
const std::string XCells("X Cells");
const std::string XStep("X Step");
const std::string YCells("Y Cells");
const std::string YStep("Y Step");
// Phase Headers: Format Version 2
const std::string Phases("Phases");
const std::string Color("Color");
const std::string LatticeAngles("Lattice Angles");
const std::string LatticeDimensions("Lattice Dimensions");
const std::string LaueGroup("Laue Group");
const std::string NumberReflectors("Number Reflectors");
const std::string PhaseName("Phase Name");
const std::string Reference("Reference");
const std::string SpaceGroup("Space Group");
// Data: Format Version 2
const std::string EBSD("EBSD");
const std::string Data("Data");
const std::string BandContrast("Band Contrast");                  // uint8
const std::string BandSlope("Band Slope");                        // uint8
const std::string Bands("Bands");                                 // uint8
const std::string Error("Error");                                 // uint8
const std::string Euler("Euler");                                 // 3xFloat32
const std::string MeanAngularDeviation("Mean Angular Deviation"); // Float
const std::string Phase("Phase");                                 // uint8
const std::string X("X");                                         // uint8
const std::string Y("Y");                                         // uint8

// Data Format Version 5

const std::string UnprocessedPatterns("Unprocessed Patterns");
const std::string ProcessedPatterns("Processed Patterns");

} // namespace H5OINA

} // namespace EbsdLib
