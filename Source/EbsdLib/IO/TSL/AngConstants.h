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
#pragma once

#include <string>

/**
 * @file AngConstants.h
 * @brief This file contains both c style macro definitions and C++ style const
 * declarations of TSL OIM variables and names. If you are dealing with a .ang
 * file from TSL instrumentation then you MUST use the defines from this file.
 */

// -----------------------------------------------------------------------------
//  These defines are needed for the pure "C" parser
// -----------------------------------------------------------------------------
#define ANG_TEM_PIXPERUM "TEM_PIXperUM"
#define ANG_X_STAR "x-star"
#define ANG_Y_STAR "y-star"
#define ANG_Z_STAR "z-star"
#define ANG_PATTERN_CENTER_CALIBRATION "Pattern Center Calibration"
#define ANG_WORKINGDISTANCE "WorkingDistance"
#define ANG_WORKING_DISTANCe "Working Distance"
#define ANG_PHASE "Phase"
#define ANG_PHASE_NAME "PhaseName"
#define ANG_MATERIAL_NAME "MaterialName"
#define ANG_FORMULA "Formula"
#define ANG_INFO "Info"
#define ANG_SYMMETRY "Symmetry"
#define ANG_LATTICE_CONSTANTS "LatticeConstants"
#define ANG_NUMBER_FAMILIES "NumberFamilies"
#define ANG_HKLFAMILIES "hklFamilies"
#define ANG_HKL_FAMILIES "hkl Families"
#define ANG_CATEGORIES "Categories"
#define ANG_GRID "GRID"
#define ANG_GRID_TYPE "Grid Type"
#define ANG_X_STEP "XSTEP"
#define ANG_Y_STEP "YSTEP"
#define ANG_STEP_X "Step X"
#define ANG_STEP_Y "Step Y"
#define ANG_NCOLS_ODD "NCOLS_ODD"
#define ANG_NCOLS_EVEN "NCOLS_EVEN"
#define ANG_NROWS "NROWS"
#define ANG_nColumns "nColumns"
#define ANG_nROWS "nRows"
#define ANG_OPERATOR "OPERATOR"
#define ANG_SAMPLE_ID "SAMPLEID"
#define ANG_SCAN_ID "SCANID"
#define ANG_Scan_ID "Scan ID"

#define ANG_Operator "Operator"
#define ANG_Sample_ID "Sample ID"

#define ANG_SQUARE_GRID "SqrGrid"
#define ANG_HEX_GRID "HexGrid"

#define ANG_PHI1 "Phi1"
#define ANG_PHI "Phi"
#define ANG_PHI2 "Phi2"
#define ANG_IMAGE_QUALITY "Image Quality"
#define ANG_CONFIDENCE_INDEX "Confidence Index"
#define ANG_PHASE_DATA "PhaseData"
#define ANG_PHASES "Phases"
#define ANG_X_POSITION "X Position"
#define ANG_Y_POSITION "Y Position"
#define ANG_SEM_SIGNAL "SEM Signal"
#define ANG_FIT "Fit"
#define ANG_Z_STEP "ZStep"
#define ANG_Z_POS "ZPos"
#define ANG_Z_MAX "ZMax"
#define ANG_CI "CI"
#define ANG_IQ "IQ"
#define ANG_PATTERN_DATA "Pattern"
#define ANG_PATTERN_WIDTH "Pattern Width"
#define ANG_PATTERN_HEIGHT "Pattern Height"

#define ANG_VERSION "VERSION"

#define ANG_NOTES "NOTES"
#define ANG_NOTES_START "NOTES: Start"
#define ANG_NOTES_END "NOTES: End"

#define ANG_COLUMN_NOTES "COLUMN_NOTES"
#define ANG_COLUMN_NOTES_START "COLUMN_NOTES: Start"
#define ANG_COLUMN_NOTES_END "COLUMN_NOTES: End"

#define COLUMN_COUNT "COLUMN_COUNT"
#define COLUMN_HEADERS "COLUMN_HEADERS"
#define COLUMN_UNITS "COLUMN_UNITS"

#define HEADER_START "HEADER: Start"
#define HEADER_END "HEADER: End"

enum ANG_READ_FLAG
{
  ANG_FULL_FILE,
  ANG_HEADER_ONLY
};

namespace EbsdLib
{

namespace H5OIM
{
const std::string EDAX("EDAX");
const std::string Manufacturer(" Manufacturer");
const std::string Version7(" Version");
const std::string Version8("Version");
const std::string H5FileExt("h5");

const std::string OriginalHeader("OriginalHeader");
const std::string OriginalFile("OriginalFile");
const std::string Index("Index");
const std::string Header("Header");
const std::string Phases("Phases");
const std::string Phase("Phase");
const std::string Data("Data");
const std::string EBSD("EBSD");
const std::string SEM_PRIAS_Images("SEM-PRIAS Images");
const std::string PatternCenterCalibration("Pattern Center Calibration");
const std::string SEM("SEM");

const std::string FileVersionStr("FileVersion");
const unsigned int FileVersion = 5;
const std::string EbsdLibVersionStr("EbsdLibVersion");

const std::string OIMAnalysisVersion("OIM Analysis");
const std::string OIMAnalysisVersion7("OIM Analysis 7");
const std::string OIMAnalysisVersion8("OIM Analysis 8");
} // namespace H5OIM

namespace Ang
{
const std::string Manufacturer("TSL");
const std::string EDAX("EDAX");

/* These are courtesy of TSL */
// LAUE Symmetry Identifiers

#define OH 43 // cubic            Oh         a=b=c     a=b=g=90

#define TH 23  // tetrahedral      Th         a=b=c     a=b=g=90
#define D4H 42 // ditetragonal     D4h        a=b!=c    a=b=g=90
#define C4H 4  // tetragonal       C4h        a=b!=c    a=b=g=90

#define D2H 22 // orthrohombic     D2h        a!=b!=c   a=b=g=90

#define C2H_c 2  // monoclinic       C2h        a!=b!=c   a=b=90!=g
#define C2H_b 20 // monoclinic       C2h        a!=b!=c   a=g=90!=b
#define C2H_a 21 // monoclinic       C2h        a!=b!=c   b=g=90!=a

#define D6H 62 // dihexagonal      D6h        a=b!=c    a=b=90 g=120
#define C6H 6  // hexagonal        C6h        a=b! =c   a=b=90 g=120

#define D3D 32 // ditrigonal       D3d        a=b=c     a=b=g!=90
#define C3I 3  // trigonal         C3i        a=b=c     a=b=g!=90

#define CIs 1 // triclinic        Ci         a!=b!=c  a!=b!=g!=90

namespace PhaseSymmetry
{
const unsigned int Cubic = OH;
const unsigned int Tetrahedral = TH;
const unsigned int DiTetragonal = D4H;
const unsigned int Tetragonal = C4H;
const unsigned int Orthorhombic = D2H;
const unsigned int Monoclinic_c = C2H_c;
const unsigned int Monoclinic_b = C2H_b;
const unsigned int Monoclinic_a = C2H_a;
const unsigned int DiHexagonal = D6H;
const unsigned int Hexagonal = C6H;
const unsigned int DiTrigonal = D3D;
const unsigned int Trigonal = C3I;
const unsigned int Triclinic = CIs;
const unsigned int UnknownSymmetry = 999;
} // namespace PhaseSymmetry

const std::string TEMPIXPerUM(ANG_TEM_PIXPERUM);
const std::string XStar(ANG_X_STAR);
const std::string YStar(ANG_Y_STAR);
const std::string ZStar(ANG_Z_STAR);
const std::string WorkingDistance(ANG_WORKINGDISTANCE);
const std::string Working_Distance(ANG_WORKING_DISTANCe);
const std::string Phase(ANG_PHASE);
const std::string PhaseName(ANG_PHASE_NAME);
const std::string MaterialName(ANG_MATERIAL_NAME);
const std::string Formula(ANG_FORMULA);
const std::string Info(ANG_INFO);
const std::string Symmetry(ANG_SYMMETRY);
const std::string LatticeConstants(ANG_LATTICE_CONSTANTS);
const std::string NumberFamilies(ANG_NUMBER_FAMILIES);
const std::string HKLFamilies(ANG_HKLFAMILIES);
const std::string HKL_Families(ANG_HKL_FAMILIES);
const std::string Categories(ANG_CATEGORIES);
const std::string Grid(ANG_GRID);
const std::string GridType(ANG_GRID_TYPE);
const std::string XStep(ANG_X_STEP);
const std::string YStep(ANG_Y_STEP);
const std::string StepX(ANG_STEP_X);
const std::string StepY(ANG_STEP_Y);
const std::string NColsOdd(ANG_NCOLS_ODD);
const std::string NColsEven(ANG_NCOLS_EVEN);
const std::string nColumns(ANG_nColumns);
const std::string NRows(ANG_NROWS);
const std::string nRows(ANG_nROWS);
const std::string Operator(ANG_Operator);
const std::string SampleID(ANG_Sample_ID);
const std::string OPERATOR(ANG_OPERATOR);
const std::string SAMPLEID(ANG_SAMPLE_ID);
const std::string SCANID(ANG_SCAN_ID);
const std::string ScanID(ANG_Scan_ID);
const std::string Phi1(ANG_PHI1);
const std::string Phi(ANG_PHI);
const std::string Phi2(ANG_PHI2);
const std::string ImageQuality(ANG_IMAGE_QUALITY);
const std::string ConfidenceIndex(ANG_CONFIDENCE_INDEX);
const std::string CI(ANG_CI);
const std::string IQ(ANG_IQ);
const std::string PhaseData(ANG_PHASE_DATA);
const std::string XPosition(ANG_X_POSITION);
const std::string YPosition(ANG_Y_POSITION);
const std::string SEMSignal(ANG_SEM_SIGNAL);
const std::string Fit(ANG_FIT);
const std::string PatternData(ANG_PATTERN_DATA);
const std::string PatternWidth(ANG_PATTERN_WIDTH);
const std::string PatternHeight(ANG_PATTERN_HEIGHT);

const std::string SquareGrid(ANG_SQUARE_GRID);
const std::string HexGrid(ANG_HEX_GRID);

const std::string ZStep(ANG_Z_STEP);
const std::string ZPos(ANG_Z_POS);
const std::string ZMax(ANG_Z_MAX);

const std::string Version(ANG_VERSION);

const std::string Notes(ANG_NOTES);
const std::string NotesStart(ANG_NOTES_START);
const std::string NotesEnd(ANG_NOTES_END);

const std::string ColumnNotes(ANG_COLUMN_NOTES);
const std::string ColumnNotesStart(ANG_COLUMN_NOTES_START);
const std::string ColumnNotesEnd(ANG_COLUMN_NOTES_END);

const std::string ColumnCount(COLUMN_COUNT);
const std::string ColumnHeaders(COLUMN_HEADERS);
const std::string ColumnUnits(COLUMN_UNITS);

} // namespace Ang
} // namespace EbsdLib

#if 1
// -----------------------------------------------------------------------------
//  These are the lower case versions of the constants for the ANG file
// -----------------------------------------------------------------------------
#define ANG_TEM_PIXPERUM_LOWER "teEbsdLib::Constants::k_Pixperum"
#define ANG_X_STAR_LOWER "x-star"
#define ANG_Y_STAR_LOWER "y-star"
#define ANG_Z_STAR_LOWER "z-star"
#define ANG_WORKINGDISTANCE_LOWER "workingdistance"

#define ANG_PHASE_LOWER "phase"
#define ANG_MATERIAL_NAME_LOWER "materialname"
#define ANG_FORMULA_LOWER "formula"
#define ANG_INFO_LOWER "info"
#define ANG_SYMMETRY_LOWER "symmetry"
#define ANG_LATTICE_CONSTANTS_LOWER "latticeconstants"
#define ANG_NUMBER_FAMILIES_LOWER "numberfamilies"
#define ANG_HKL_FAMILIES_LOWER "hklfamilies"
#define ANG_CATEGORIES_LOWER "categories"

#define ANG_GRID_LOWER "grid"
#define ANG_X_STEP_LOWER "xstep"
#define ANG_Y_STEP_LOWER "ystep"
#define ANG_NCOLS_ODD_LOWER "ncols_odd"
#define ANG_NCOLS_EVEN_LOWER "ncols_even"
//#define ANG_NCOLS_LOWER "ncols"
#define ANG_NROWS_LOWER "nrows"

#define ANG_OPERATOR_LOWER "operator"
#define ANG_SAMPLE_ID_LOWER "sampleid"
#define ANG_SCAN_ID_LOWER "scanid"
#define ANG_SQUARE_GRID_LOWER "sqrgrid"
#define ANG_HEX_GRID_LOWER "hexgrid"

// Constants  to be used by all classes
#define ANG_PHI1_LOWER "phi1"
#define ANG_PHI_LOWER "phi"
#define ANG_PHI2_LOWER "phi2"
#define ANG_IMAGE_QUALITY_LOWER "image quality"
#define ANG_CONFIDENCE_INDEX_LOWER "confidence index"
#define ANG_PHASE_DATA_LOWER "phase"
#define ANG_X_POSITION_LOWER "x position"
#define ANG_Y_POSITION_LOWER "y position"
#define ANG_SEM_SIGNAL_LOWER "sem signal"
#define ANG_FIT_LOWER "fit"

namespace EbsdLib
{

namespace Ang
{
// These are the Lower Case versions of the constants
const std::string FileExtLower("ang");
const std::string FileExt("ang");
const std::string TEMPIXPerUMLower(ANG_TEM_PIXPERUM_LOWER);
const std::string XStarLower(ANG_X_STAR_LOWER);
const std::string YStarLower(ANG_Y_STAR_LOWER);
const std::string ZStarLower(ANG_Z_STAR_LOWER);
const std::string WorkingDistanceLower(ANG_WORKINGDISTANCE_LOWER);
const std::string PhaseLower(ANG_PHASE_LOWER);
const std::string MaterialNameLower(ANG_MATERIAL_NAME_LOWER);
const std::string FormulaLower(ANG_FORMULA_LOWER);
const std::string InfoLower(ANG_INFO_LOWER);
const std::string SymmetryLower(ANG_SYMMETRY_LOWER);
const std::string LatticeConstantsLower(ANG_LATTICE_CONSTANTS_LOWER);
const std::string NumberFamiliesLower(ANG_NUMBER_FAMILIES_LOWER);
const std::string HKLFamiliesLower(ANG_HKL_FAMILIES_LOWER);
const std::string CategoriesLower(ANG_CATEGORIES_LOWER);
const std::string GridLower(ANG_GRID_LOWER);
const std::string XStepLower(ANG_X_STEP_LOWER);
const std::string YStepLower(ANG_Y_STEP_LOWER);
const std::string NColsOddLower(ANG_NCOLS_ODD_LOWER);
const std::string NColsEvenLower(ANG_NCOLS_EVEN_LOWER);
//   const std::string NColsLower(ANG_NCOLS_LOWER);
const std::string NRowsLower(ANG_NROWS_LOWER);
const std::string OperatorLower(ANG_OPERATOR_LOWER);
const std::string SampleIdLower(ANG_SAMPLE_ID_LOWER);
const std::string ScanIdLower(ANG_SCAN_ID_LOWER);

const std::string Phi1Lower(ANG_PHI1_LOWER);
const std::string PhiLower(ANG_PHI_LOWER);
const std::string Phi2Lower(ANG_PHI2_LOWER);
const std::string ImageQualityLower(ANG_IMAGE_QUALITY_LOWER);
const std::string ConfidenceIndexLower(ANG_CONFIDENCE_INDEX_LOWER);
const std::string PhaseDataLower(ANG_PHASE_DATA_LOWER);
const std::string XPositionLower(ANG_X_POSITION_LOWER);
const std::string YPositionLower(ANG_Y_POSITION_LOWER);
const std::string SEMSignalLower(ANG_SEM_SIGNAL_LOWER);
const std::string FitLower(ANG_FIT_LOWER);

const std::string LatticeConstantA("Lattice Constant a");
const std::string LatticeConstantB("Lattice Constant b");
const std::string LatticeConstantC("Lattice Constant c");
const std::string LatticeConstantAlpha("Lattice Constant alpha");
const std::string LatticeConstantBeta("Lattice Constant beta");
const std::string LatticeConstantGamma("Lattice Constant gamma");
} // namespace Ang

namespace AngFile
{
const std::string Phases("Phases");
const std::string EulerAngles("EulerAngles");
const std::string CrystalStructures("CrystalStructures");
const std::string MaterialName("MaterialName");
const std::string LatticeConstants("LatticeConstants");
const std::string BravaisLattice("BravaisLattice");
} // namespace AngFile
} // namespace EbsdLib
#endif
