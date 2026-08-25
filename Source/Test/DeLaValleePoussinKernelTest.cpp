#include <catch2/catch.hpp>

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Texture/SO3DeLaValleePoussinKernel.h"

#include <cmath>

using namespace ebsdlib;

namespace
{
constexpr double k_DegToRad = ebsdlib::constants::k_PiD / 180.0;
}

// -----------------------------------------------------------------------------
// SO3DeLaValleePoussinKernel is a direct port of MTEX 6.1.0's
// SO3DeLaValleePoussinKernel class (SO3Fun/SO3KernelFunctions/SO3DeLaValleePoussinKernel.m).
// The reference constants below were generated with:
//
//   psi=SO3DeLaValleePoussinKernel('halfwidth',10*degree);
//   fprintf('kappa10=%.12f C10=%.12f\n',psi.kappa,psi.C);
//   psi5=SO3DeLaValleePoussinKernel('halfwidth',5*degree);
//   fprintf('kappa5=%.12f C5=%.12f\n',psi5.kappa,psi5.C);
//   fprintf('eval10_at20deg=%.12f\n', psi.eval(cos(20*degree/2)));
//
// MTEX's eval(psi, co2) takes co2 = cos(omega/2), matching evaluate(cosHalfOmega)
// here, so psi.eval(cos(20*degree/2)) is K(omega=20deg) for the halfwidth=10deg kernel.
TEST_CASE("SO3DeLaValleePoussinKernel matches MTEX", "[DeLaValleePoussinKernel]")
{
  SO3DeLaValleePoussinKernel psi10(10.0 * k_DegToRad);
  REQUIRE(psi10.kappa() == Approx(90.903105993155).epsilon(1.0e-9));
  REQUIRE(psi10.constant() == Approx(1555.219446506688).epsilon(1.0e-9));
  REQUIRE(psi10.evaluate(std::cos(20.0 * k_DegToRad / 2.0)) == Approx(96.171329109334).epsilon(1.0e-9));

  SO3DeLaValleePoussinKernel psi5(5.0 * k_DegToRad);
  REQUIRE(psi5.kappa() == Approx(363.959328000956).epsilon(1.0e-9));
  REQUIRE(psi5.constant() == Approx(12345.110683440258).epsilon(1.0e-9));

  // halfwidth round-trip: hw = 2*acos(0.5^(1/(2*kappa)))
  REQUIRE(2.0 * std::acos(std::pow(0.5, 1.0 / (2.0 * psi10.kappa()))) == Approx(10.0 * k_DegToRad).epsilon(1.0e-12));

  // halfwidth definition: K(hw) == K(0)/2
  REQUIRE(psi10.evaluate(std::cos(10.0 * k_DegToRad / 2.0)) == Approx(psi10.evaluate(1.0) / 2.0).epsilon(1.0e-9));

  // cutoff = min(pi, 3.5*hw)
  REQUIRE(psi10.cutoffAngle() == Approx(3.5 * 10.0 * k_DegToRad));
  SO3DeLaValleePoussinKernel psiWide(60.0 * k_DegToRad);
  REQUIRE(psiWide.cutoffAngle() == Approx(ebsdlib::constants::k_PiD));
}
