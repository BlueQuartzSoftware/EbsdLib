/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
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

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Texture/SO3DeLaValleePoussinKernel.h"

#include <array>
#include <cstdint>
#include <vector>

namespace ebsdlib
{
/**
 * @brief Misorientation kernel density estimator on the MDF (misorientation) fundamental-zone bin grid.
 *
 * Accumulates weighted misorientations into the Laue-class MDF-FZ bins (via LaueOps::getMDFFZRod +
 * getMisoBin), then evaluates a symmetrized De la Vallee Poussin kernel density at an arbitrary
 * misorientation. The density is:
 *
 *   f(g) = sum_bins w_bin * ( K(g, c_bin) + K(g, c_bin^-1) ) / ( 2 * |CS|^2 )
 *
 * where each K is summed over the |CS| x |CS| crystal-symmetry pairs (s_i * g * s_j), and the
 * grain-exchange (antipodal) inverse term enforces f(g) == f(g^-1). The forward and inverse terms
 * together span the grain-exchange-extended symmetry orbit (2 * |CS|^2 elements), so the mean-1
 * normalization divides by 2 * |CS|^2. The kernel psi integrates to 1 over SO(3); with weights
 * normalized to sum 1 the density is then a normalized MDF (mean == 1 over SO(3), matching MTEX's
 * mean(mdf) == 1) whose triclinic (|CS| == 1) modal peak height is K(0) / 2. The absolute scale was
 * pinned by a direct numerical cross-check against MTEX 6.1.0 (see computeAngleCurve()).
 *
 * Usage: construct, addMisorientation() for every observation, finalize() once, then evaluate().
 */
class EbsdLib_EXPORT MisorientationKDE
{
public:
  /**
   * @brief Misorientation-angle-distribution curve extracted from the KDE.
   * Angles are in radians (0 .. MaxMisorientationAngle(structure)); Density is the MDF
   * angle distribution; RandomDensity is the uniform (random) reference distribution.
   */
  struct AngleCurve
  {
    std::vector<double> Angles;
    std::vector<double> Density;
    std::vector<double> RandomDensity;
  };

  /**
   * @brief Constructor.
   * @param ops Laue-class symmetry operators for the MDF fundamental zone.
   * @param crystalStructure EbsdLib crystal-structure index of ops. LaueOps has no reverse lookup;
   *        it is stored for Task 4's computeAngleCurve() (the Mackenzie reference) and is not
   *        consumed by this class.
   * @param halfwidthRadians De la Vallee Poussin kernel halfwidth in radians.
   */
  MisorientationKDE(LaueOps::Pointer ops, uint32_t crystalStructure, double halfwidthRadians);

  /**
   * @brief Accumulate a weighted misorientation into its MDF-FZ bin.
   * @param misoQuat Misorientation quaternion.
   * @param weight Non-negative weight (need not be normalized).
   */
  void addMisorientation(const QuatD& misoQuat, double weight);

  /**
   * @brief Normalize accumulated weights to sum 1 and build the center list. Call once, after all adds.
   */
  void finalize();

  /**
   * @brief Sum of all weights passed to addMisorientation() (before normalization).
   */
  double totalWeight() const;

  /**
   * @brief Density at an arbitrary misorientation quaternion; valid after finalize().
   */
  double evaluate(const QuatD& query) const;

  /**
   * @brief MDF-FZ-folded bin-center quaternion for the given miso bin index.
   */
  QuatD binCenter(int binIndex) const;

  /**
   * @brief Serial convenience: evaluate() at every bin center; size getMDFSize().
   */
  std::vector<double> evaluateAtBinCenters() const;

  /**
   * @brief Misorientation-angle-distribution curve: a port of MTEX
   * SO3Fun/@SO3Fun/calcAngleDistribution.m. For each of numPoints angles omega in
   * [0, MaxMisorientationAngle(structure)] the density is the uniform-reference value
   * (random_angle_distribution::Compute) multiplied by the mean of evaluate() over an
   * axis grid on the omega-sphere. Two deliberate deviations from MTEX (validated by the
   * MTEX numerical cross-check in the unit test, tolerance epsilon(0.20)+margin(0.10)):
   *   (1) axes are a Fibonacci full-sphere sampling filtered to MDF-FZ membership (via the
   *       audited getMDFFZRod folds, each misorientation class counted exactly once)
   *       instead of MTEX's fundamental-sector grid with a one-sided FZ check;
   *   (2) the axis-grid count is scaled by 2*|CS| so the post-filter (FZ-only) axis density
   *       matches MTEX's sector-grid density.
   */
  AngleCurve computeAngleCurve(size_t numPoints) const;

private:
  struct Center
  {
    QuatD Quat;
    QuatD QuatInverse;
    double Weight;
  };

  LaueOps::Pointer m_Ops;
  uint32_t m_CrystalStructure;
  SO3DeLaValleePoussinKernel m_Kernel;
  std::vector<QuatD> m_SymQuats;
  std::vector<double> m_BinWeights;
  // Per-bin weight-weighted running sum of the (sign-aligned) fundamental-zone misorientation
  // quaternions {x,y,z,w}. finalize() normalizes each non-empty bin's sum to obtain the bin's
  // representative center. This is the weighted circular mean of the misorientations that fell
  // in the bin, which is far closer to the true data than the geometric bin center and removes
  // the ~5-degree MDF-bin quantization bias from the extracted angle-distribution curve.
  std::vector<std::array<double, 4>> m_BinQuatSum;
  double m_TotalWeight = 0.0;
  std::vector<Center> m_Centers;
};
} // namespace ebsdlib
