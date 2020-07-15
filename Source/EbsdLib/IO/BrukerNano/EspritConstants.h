/* ============================================================================
 * Copyright (c) 2019 BlueQuartz Software, LLC
 * All rights reserved.
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
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include <string>

namespace EbsdLib
{
namespace Esprit
{
const std::string EulerAngles("EulerAngles");
const std::string CrystalStructures("CrystalStructures");
const std::string MaterialName("MaterialName");
const std::string LatticeConstants("LatticeConstants");
const std::string BravaisLattice("BravaisLattice");
//   const std::string PatternData("PatternData");
} // namespace Esprit
namespace H5Esprit
{
const std::string BrukerNano("Bruker Nano");
const std::string Manufacturer("Manufacturer");
const std::string Version("Version");
const std::string H5FileExt("h5");
const std::string Isometric("isometric");
const std::string EBSD("EBSD");
const std::string Header("Header");
const std::string Phases("Phases");
const std::string Data("Data");

// Header Section for EBSD Data
const std::string CameraTilt("CameraTilt");
const std::string GridType("Grid Type");
const std::string KV("KV");
const std::string MADMax("MADMax");
const std::string Magnification("Magnification");
const std::string MapStepFactor("MapStepFactor");
const std::string MaxRadonBandCount("MaxRadonBandCount");
const std::string MinIndexedBands("MinIndexedBands");
const std::string NCOLS("NCOLS");
const std::string NPoints("NPoints");
const std::string NROWS("NROWS");
const std::string OriginalFile("OriginalFile");
const std::string PatternHeight("PatternHeight");
const std::string PatternWidth("PatternWidth");
const std::string PixelByteCount("PixelByteCount");
const std::string SEMImage("SEM Image"); // <=== Actual SEM Image is stored here.
const std::string SEPixelSizeX("SEPixelSizeX");
const std::string SEPixelSizeY("SEPixelSizeY");
const std::string SampleTilt("SampleTilt");
const std::string TopClip("TopClip");
const std::string UnClippedPatternHeight("UnClippedPatternHeight");
const std::string WD("WD");
const std::string XSTEP("XSTEP");
const std::string YSTEP("YSTEP");
const std::string ZOffset("ZOffset");

// Phases section of EBSD/Header
const std::string Formula("Formula");
const std::string IT("IT");
const std::string LatticeConstants("LatticeConstants");
const std::string Name("Name");
const std::string Setting("Setting");
const std::string SpaceGroup("SpaceGroup");

// Data Section for EBSD Data
// const std::string DD("DD");
const std::string MAD("MAD");
// const std::string MADPhase("MADPhase");
const std::string NIndexedBands("NIndexedBands");
// const std::string PCX("PCX");
// const std::string PCY("PCY");
const std::string PHI("PHI");
const std::string Phase("Phase");
const std::string RadonBandCount("RadonBandCount");
const std::string RadonQuality("RadonQuality");
const std::string RawPatterns("RawPatterns");
const std::string XBEAM("X BEAM");
const std::string YBEAM("Y BEAM");
// const std::string XSAMPLE("X SAMPLE");
// const std::string YSAMPLE("Y SAMPLE");
const std::string phi1("phi1");
const std::string phi2("phi2");

using DD_t = float;
using MAD_t = float;
using MADPhase_t = int32_t;
using NIndexedBands_t = int32_t;
using PCX_t = float;
using PCY_t = float;
using PHI_t = float;
using Phase_t = int32_t;
using RadonBandCount_t = int32_t;
using RadonQuality_t = float;
using RawPatterns_t = uint8_t;
using XBEAM_t = int32_t;
using YBEAM_t = int32_t;
using XSAMPLE_t = float;
using YSAMPLE_t = float;
using phi1_t = float;
using phi2_t = float;

const std::string SEM("SEM");
// Header Section for SEM Data
const std::string SEM_IX("SEM IX");
const std::string SEM_IY("SEM IY");
const std::string SEM_Image("SEM Image");
const std::string SEM_ImageHeight("SEM ImageHeight");
const std::string SEM_ImageWidth("SEM ImageWidth");
const std::string SEM_KV("SEM KV");
const std::string SEM_Magnification("SEM Magnification");
const std::string SEM_WD("SEM WD");
const std::string SEM_XResolution("SEM XResolution");
const std::string SEM_YResolution("SEM YResolution");
const std::string SEM_ZOffset("SEM ZOffset");

} // namespace H5Esprit
} // namespace EbsdLib
