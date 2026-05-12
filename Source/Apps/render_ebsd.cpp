/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * render_ebsd
 *
 * Reads a .ang or .ctf EBSD scan and produces, per indexed phase, a 3-PNG set:
 *   - composite pole figure (PoleFigureCompositor)
 *   - IPF map           (per-pixel generateIPFColor)
 *   - IPF triangle legend (LaueOps::generateIPFTriangleLegend)
 *
 * All three outputs honor the user-supplied HexConvention and color-key
 * choice. No legacy Euler pre-processing is applied; the convention plumbing
 * goes through the LaueOps API.
 *
 * Usage:
 *   render_ebsd <input.ang|input.ctf> <output_dir>
 *               [--convention {x_a, x_astar}]            (default x_astar)
 *               [--color-key  {tsl, pucm, nh}]           (default tsl)
 *               [--phase N]                              (default: all phases)
 *               [--ref-dir x,y,z]                        (default 0,0,1)
 *               [--image-dim N] [--lambert-dim N] [--legend-dim N]
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "Apps/render_ebsd.h"

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/CtfPhase.h"
#include "EbsdLib/IO/HKL/CtfReader.h"
#include "EbsdLib/IO/TSL/AngPhase.h"
#include "EbsdLib/IO/TSL/AngReader.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/ImageCrop.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/PUCMColorKey.hpp"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace ebsdlib::render_ebsd
{

namespace
{
struct PhaseScan
{
  int phaseIndex = 0;
  std::string phaseName;
  unsigned int laueOpsIndex = ebsdlib::CrystalStructure::UnknownCrystalStructure;
  ebsdlib::FloatArrayType::Pointer eulers;
  std::vector<int32_t> dims; ///< {width, height} only set for raster outputs (IPF map)
  std::vector<int32_t> rasterPhase;
  std::vector<float> rasterEulers;
};

// ---------------------------------------------------------------------------
// Read .ang. No phi2-30°/90°-z pre-rotation is applied here; convention
// handling is the responsibility of LaueOps via Options::convention.
// ---------------------------------------------------------------------------
std::vector<PhaseScan> readAng(const std::string& filePath)
{
  AngReader reader;
  reader.setFileName(filePath);
  if(reader.readFile() < 0)
  {
    std::cerr << "ERROR: Failed to read .ang file: " << filePath << std::endl;
    return {};
  }

  const size_t totalPoints = reader.getNumberOfElements();
  const int32_t xDim = reader.getXDimension();
  const int32_t yDim = reader.getYDimension();

  float* phi1 = reader.getPhi1Pointer(false);
  float* phi = reader.getPhiPointer(false);
  float* phi2 = reader.getPhi2Pointer(false);
  int* phasePtr = reader.getPhaseDataPointer(false);
  float* ci = reader.getConfidenceIndexPointer(false);

  std::map<int, unsigned int> phaseToLaue;
  std::map<int, std::string> phaseToName;
  for(const auto& phase : reader.getPhaseVector())
  {
    int idx = phase->getPhaseIndex();
    phaseToLaue[idx] = phase->determineOrientationOpsIndex();
    std::string name = phase->getMaterialName();
    if(name.empty())
    {
      name = "Phase_" + std::to_string(idx);
    }
    phaseToName[idx] = name;
  }

  // Per-phase eulers for PF + per-phase rastered IPF coords for the IPF map
  std::map<int, std::vector<float>> phaseEulerMap;
  std::map<int, PhaseScan> phaseScanMap;
  for(auto& [idx, name] : phaseToName)
  {
    PhaseScan& s = phaseScanMap[idx];
    s.phaseIndex = idx;
    s.phaseName = name;
    s.laueOpsIndex = phaseToLaue[idx];
    s.dims = {xDim, yDim};
    s.rasterPhase.assign(totalPoints, 0);
    s.rasterEulers.assign(totalPoints * 3, 0.0F);
  }

  for(size_t i = 0; i < totalPoints; i++)
  {
    int p = phasePtr[i];
    if(p < 1 && phaseToLaue.find(1) != phaseToLaue.end())
    {
      p = 1;
    }
    auto laueIt = phaseToLaue.find(p);
    if(laueIt == phaseToLaue.end() || laueIt->second >= ebsdlib::CrystalStructure::LaueGroupEnd)
    {
      continue;
    }
    PhaseScan& s = phaseScanMap[p];
    s.rasterPhase[i] = 1;
    s.rasterEulers[i * 3] = phi1[i];
    s.rasterEulers[i * 3 + 1] = phi[i];
    s.rasterEulers[i * 3 + 2] = phi2[i];

    if(ci[i] > 0.1F)
    {
      phaseEulerMap[p].push_back(phi1[i]);
      phaseEulerMap[p].push_back(phi[i]);
      phaseEulerMap[p].push_back(phi2[i]);
    }
  }

  std::vector<PhaseScan> result;
  for(auto& [idx, scan] : phaseScanMap)
  {
    auto eulIt = phaseEulerMap.find(idx);
    if(eulIt == phaseEulerMap.end() || eulIt->second.empty())
    {
      continue;
    }
    const size_t numOrientations = eulIt->second.size() / 3;
    std::vector<size_t> cDims = {3};
    scan.eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);
    std::memcpy(scan.eulers->getVoidPointer(0), eulIt->second.data(), eulIt->second.size() * sizeof(float));
    result.push_back(std::move(scan));
  }
  return result;
}

// ---------------------------------------------------------------------------
// Read .ctf. CTF Eulers are in degrees on disk; convert to radians.
// ---------------------------------------------------------------------------
std::vector<PhaseScan> readCtf(const std::string& filePath)
{
  CtfReader reader;
  reader.setFileName(filePath);
  if(reader.readFile() < 0)
  {
    std::cerr << "ERROR: Failed to read .ctf file: " << filePath << std::endl;
    return {};
  }

  const size_t totalPoints = reader.getNumberOfElements();
  const int32_t xDim = reader.getXDimension();
  const int32_t yDim = reader.getYDimension();
  const float degToRad = static_cast<float>(ebsdlib::constants::k_DegToRadD);

  float* e1 = reader.getEuler1Pointer();
  float* e2 = reader.getEuler2Pointer();
  float* e3 = reader.getEuler3Pointer();
  int* phasePtr = reader.getPhasePointer();

  std::map<int, unsigned int> phaseToLaue;
  std::map<int, std::string> phaseToName;
  for(const auto& phase : reader.getPhaseVector())
  {
    int idx = phase->getPhaseIndex();
    phaseToLaue[idx] = phase->determineOrientationOpsIndex();
    std::string name = phase->getPhaseName();
    if(name.empty())
    {
      name = "Phase_" + std::to_string(idx);
    }
    phaseToName[idx] = name;
  }

  std::map<int, std::vector<float>> phaseEulerMap;
  std::map<int, PhaseScan> phaseScanMap;
  for(auto& [idx, name] : phaseToName)
  {
    PhaseScan& s = phaseScanMap[idx];
    s.phaseIndex = idx;
    s.phaseName = name;
    s.laueOpsIndex = phaseToLaue[idx];
    s.dims = {xDim, yDim};
    s.rasterPhase.assign(totalPoints, 0);
    s.rasterEulers.assign(totalPoints * 3, 0.0F);
  }

  for(size_t i = 0; i < totalPoints; i++)
  {
    int p = phasePtr[i];
    if(p < 1 && phaseToLaue.find(1) != phaseToLaue.end())
    {
      p = 1;
    }
    auto laueIt = phaseToLaue.find(p);
    if(laueIt == phaseToLaue.end() || laueIt->second >= ebsdlib::CrystalStructure::LaueGroupEnd)
    {
      continue;
    }
    PhaseScan& s = phaseScanMap[p];
    s.rasterPhase[i] = 1;
    s.rasterEulers[i * 3] = e1[i] * degToRad;
    s.rasterEulers[i * 3 + 1] = e2[i] * degToRad;
    s.rasterEulers[i * 3 + 2] = e3[i] * degToRad;

    phaseEulerMap[p].push_back(e1[i] * degToRad);
    phaseEulerMap[p].push_back(e2[i] * degToRad);
    phaseEulerMap[p].push_back(e3[i] * degToRad);
  }

  std::vector<PhaseScan> result;
  for(auto& [idx, scan] : phaseScanMap)
  {
    auto eulIt = phaseEulerMap.find(idx);
    if(eulIt == phaseEulerMap.end() || eulIt->second.empty())
    {
      continue;
    }
    const size_t numOrientations = eulIt->second.size() / 3;
    std::vector<size_t> cDims = {3};
    scan.eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);
    std::memcpy(scan.eulers->getVoidPointer(0), eulIt->second.data(), eulIt->second.size() * sizeof(float));
    result.push_back(std::move(scan));
  }
  return result;
}

const char* conventionToken(ebsdlib::HexConvention conv)
{
  return conv == ebsdlib::HexConvention::XParallelA ? "x_a" : "x_astar";
}

const char* colorKeyToken(ebsdlib::ColorKeyKind k)
{
  switch(k)
  {
  case ebsdlib::ColorKeyKind::TSL:
    return "tsl";
  case ebsdlib::ColorKeyKind::PUCM:
    return "pucm";
  case ebsdlib::ColorKeyKind::NolzeHielscher:
    return "nh";
  }
  return "tsl";
}

// Sanitize a phase name for filesystem use (lowercase, [a-z0-9_] only).
std::string sanitize(const std::string& name)
{
  std::string out;
  out.reserve(name.size());
  for(char c : name)
  {
    if((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))
    {
      out += c;
    }
    else if(c >= 'A' && c <= 'Z')
    {
      out += static_cast<char>(c - 'A' + 'a');
    }
    else if(c == ' ' || c == '-' || c == '/' || c == '\\')
    {
      out += '_';
    }
  }
  if(out.empty())
  {
    out = "phase";
  }
  return out;
}

std::string makePath(const Options& opts, const PhaseScan& s, const char* kind)
{
  std::ostringstream ss;
  ss << opts.outputDir << "/" << sanitize(s.phaseName) << "_phase" << s.phaseIndex << "_" << conventionToken(opts.convention) << "_" << colorKeyToken(opts.colorKey) << "_" << kind << ".png";
  return ss.str();
}

bool writePoleFigure(const Options& opts, PhaseScan& s, LaueOps::Pointer op, const std::string& outPath)
{
  CompositePoleFigureConfiguration_t config;
  config.eulers = s.eulers.get();
  config.imageDim = opts.imageDim;
  config.lambertDim = opts.lambertDim;
  config.numColors = 32;
  config.minScale = 0.0;
  config.maxScale = 0.0; // 0 => auto
  config.sphereRadius = 1.0F;
  config.discrete = false;
  config.discreteHeatMap = false;
  config.flipFinalImage = true;
  config.layoutType = PoleFigureLayoutType::Horizontal;
  config.laueOpsIndex = s.laueOpsIndex;
  config.phaseName = s.phaseName;
  config.phaseNumber = s.phaseIndex;
  // Don't bake the convention token into the title: the smoke test asserts
  // PF bytes differ between conventions, and we want that difference to come
  // strictly from the rendered disk content (proof the convention plumbing
  // reaches LaueOps), not from rasterized title text. The output FILENAME
  // already stamps the convention.
  config.title = s.phaseName + " (" + op->getSymmetryName() + ")";
  config.hexConvention = opts.convention;

  auto names = op->getDefaultPoleFigureNames(opts.convention);
  config.labels = {names[0], names[1], names[2]};
  config.order = {0, 1, 2};

  PoleFigureCompositor compositor;
  CompositePoleFigureResult result = compositor.generateCompositeImage(config);
  if(result.image == nullptr || result.width <= 0 || result.height <= 0)
  {
    return false;
  }
  auto wr = PngWriter::WriteColorImage(outPath, result.width, result.height, 4, result.image->data());
  return wr.first >= 0;
}

bool writeIpfMap(const Options& opts, const PhaseScan& s, LaueOps::Pointer op, const std::string& outPath)
{
  if(s.dims.size() != 2 || s.dims[0] <= 0 || s.dims[1] <= 0)
  {
    return false;
  }
  const int32_t w = s.dims[0];
  const int32_t h = s.dims[1];
  const size_t total = static_cast<size_t>(w) * static_cast<size_t>(h);
  std::vector<uint8_t> rgb(total * 3, 0);

  double refDir[3] = {opts.refDir[0], opts.refDir[1], opts.refDir[2]};
  double euler[3] = {0.0, 0.0, 0.0};
  for(size_t i = 0; i < total; i++)
  {
    if(s.rasterPhase[i] == 0)
    {
      continue; // not this phase / unindexed -- leave black
    }
    euler[0] = s.rasterEulers[i * 3];
    euler[1] = s.rasterEulers[i * 3 + 1];
    euler[2] = s.rasterEulers[i * 3 + 2];
    Rgb argb = op->generateIPFColor(euler, refDir, false, opts.colorKey);
    rgb[i * 3] = static_cast<uint8_t>(RgbColor::dRed(argb));
    rgb[i * 3 + 1] = static_cast<uint8_t>(RgbColor::dGreen(argb));
    rgb[i * 3 + 2] = static_cast<uint8_t>(RgbColor::dBlue(argb));
  }
  auto wr = PngWriter::WriteColorImage(outPath, w, h, 3, rgb.data());
  return wr.first >= 0;
}

bool writeLegend(const Options& opts, LaueOps::Pointer op, const std::string& outPath)
{
  auto legend = op->generateIPFTriangleLegend(opts.legendImageDim, false, opts.convention, opts.colorKey);
  if(legend == nullptr)
  {
    return false;
  }
  // The legend renderer paints a small SST + label band onto a much larger
  // square canvas (canvasDim x canvasDim). Trim the surrounding whitespace
  // so the output PNG fills with content the way MTEX's legend export does.
  // Padding is a small fixed margin around the painted region.
  constexpr int k_LegendPadding = 12;
  auto cropped = ebsdlib::CropImageToContent(legend.get(), opts.legendImageDim, opts.legendImageDim, /*channels*/ 3, k_LegendPadding);
  if(cropped.image == nullptr)
  {
    return false;
  }
  auto wr = PngWriter::WriteColorImage(outPath, cropped.width, cropped.height, 3, cropped.image->getPointer(0));
  return wr.first >= 0;
}

} // namespace

Result run(const Options& opts)
{
  Result out;

  if(!std::filesystem::exists(opts.inputFile))
  {
    std::cerr << "ERROR: Input file does not exist: " << opts.inputFile << std::endl;
    return out;
  }

  std::filesystem::create_directories(opts.outputDir);

  std::string ext = std::filesystem::path(opts.inputFile).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  std::vector<PhaseScan> scans;
  if(ext == ".ang")
  {
    scans = readAng(opts.inputFile);
  }
  else if(ext == ".ctf")
  {
    scans = readCtf(opts.inputFile);
  }
  else
  {
    std::cerr << "ERROR: Unsupported file extension '" << ext << "'. Use .ang or .ctf" << std::endl;
    return out;
  }

  if(scans.empty())
  {
    std::cerr << "ERROR: No valid phase data found in " << opts.inputFile << std::endl;
    return out;
  }

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  bool allOk = true;
  for(auto& s : scans)
  {
    if(opts.phaseFilter >= 0 && s.phaseIndex != opts.phaseFilter)
    {
      continue;
    }
    if(s.laueOpsIndex >= ops.size())
    {
      continue;
    }
    LaueOps::Pointer op = ops[s.laueOpsIndex];

    PhaseOutput po;
    po.phaseIndex = s.phaseIndex;
    po.phaseName = s.phaseName;
    po.laueOpsIndex = s.laueOpsIndex;
    po.poleFigurePath = makePath(opts, s, "PF");
    po.ipfMapPath = makePath(opts, s, "IPF");
    po.legendPath = makePath(opts, s, "LEGEND");

    const bool pfOk = writePoleFigure(opts, s, op, po.poleFigurePath);
    const bool ipfOk = writeIpfMap(opts, s, op, po.ipfMapPath);
    const bool legOk = writeLegend(opts, op, po.legendPath);
    po.ok = pfOk && ipfOk && legOk;

    if(!po.ok)
    {
      allOk = false;
      std::cerr << "WARNING: phase '" << s.phaseName << "' partial outputs: PF=" << pfOk << " IPF=" << ipfOk << " LEGEND=" << legOk << std::endl;
    }
    out.phases.push_back(std::move(po));
  }

  out.ok = allOk && !out.phases.empty();
  return out;
}

} // namespace ebsdlib::render_ebsd

// CLI entry (`int main`) lives in render_ebsd_main.cpp so this TU can be
// compiled into both the standalone render_ebsd executable and the
// EbsdLibUnitTest binary without the latter colliding with Catch2's main.
