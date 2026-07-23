#pragma once

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Texture/SO3DeLaValleePoussinKernel.h"

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
 *   f(g) = sum_bins w_bin * ( K(g, c_bin) + K(g, c_bin^-1) )
 *
 * where each K is averaged over the |CS| x |CS| crystal-symmetry pairs (s_i * g * s_j), and the
 * grain-exchange (antipodal) inverse term enforces f(g) == f(g^-1). getMDFFZRod() already folds
 * grain exchange into the bin assignment, so the inverse term is not additionally halved. The
 * kernel psi integrates to 1 over SO(3), so with weights normalized to sum 1 the density is a
 * normalized MDF (uniform == 1) whose modal peak height is the kernel constant K(0).
 *
 * Usage: construct, addMisorientation() for every observation, finalize() once, then evaluate().
 */
class EbsdLib_EXPORT MisorientationKDE
{
public:
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
  double m_TotalWeight = 0.0;
  std::vector<Center> m_Centers;
};
} // namespace ebsdlib
