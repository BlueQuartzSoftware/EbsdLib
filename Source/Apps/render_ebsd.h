/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include "EbsdLib/Core/EbsdLibConstants.h"

#include <array>
#include <string>
#include <vector>

namespace ebsdlib::render_ebsd
{

struct Options
{
  std::string inputFile;                                                  ///< .ang or .ctf path
  std::string outputDir;                                                  ///< directory to write PNGs into (created if missing)
  ebsdlib::HexConvention convention = ebsdlib::HexConvention::XParallelA; // TSL/EDAX default
  ebsdlib::ColorKeyKind colorKey = ebsdlib::ColorKeyKind::TSL;
  int phaseFilter = -1;                             ///< -1 = render every indexed phase; otherwise only this phase index
  std::array<float, 3> refDir = {0.0F, 0.0F, 1.0F}; ///< Sample-frame reference direction for IPF map (default = +Z)
  int imageDim = 512;                               ///< Per-pole-figure pixel side
  int lambertDim = 64;                              ///< Lambert square dim used by PoleFigureCompositor
  int legendImageDim = 512;                         ///< Pixel side of the IPF triangle legend
};

struct PhaseOutput
{
  int phaseIndex = 0;
  std::string phaseName;
  unsigned int laueOpsIndex = 0;
  std::string poleFigurePath; ///< Composite pole figure PNG path (one per phase)
  std::string ipfMapPath;     ///< IPF map PNG path (one per phase, sample-frame raster)
  std::string legendPath;     ///< IPF triangle legend PNG path
  bool ok = false;
};

struct Result
{
  std::vector<PhaseOutput> phases;
  bool ok = false; ///< true iff every requested phase produced all three PNGs
};

/**
 * @brief Drive the full render matrix for a single (convention, color-key) cell.
 *
 * Reads the input EBSD scan, then for every indexed phase whose Laue class is
 * recognized (or only `phaseFilter` if that is non-negative) emits three PNGs
 * into `outputDir`:
 *
 *   <phaseName>_phase<N>_<conv>_<colorKey>_PF.png
 *   <phaseName>_phase<N>_<conv>_<colorKey>_IPF.png
 *   <phaseName>_phase<N>_<conv>_<colorKey>_LEGEND.png
 *
 * `<conv>` is `x_a` or `x_astar`; `<colorKey>` is `tsl`, `pucm`, or `nh`.
 *
 * The Euler angles read from the scan are passed to LaueOps untouched -- no
 * legacy phi2 / 90°-z pre-rotation is applied. Convention plumbing flows via
 * `Options::convention` into `PoleFigureCompositor`, `LaueOps::generateIPFColor`,
 * and `LaueOps::generateIPFTriangleLegend`.
 */
Result run(const Options& opts);

} // namespace ebsdlib::render_ebsd
