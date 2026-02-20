#include <catch2/catch.hpp>

#include "EbsdLib/IO/BrukerNano/EspritPhase.h"
#include "EbsdLib/IO/HKL/CtfPhase.h"
#include "EbsdLib/IO/TSL/AngPhase.h"

#include <string>
#include <vector>

using namespace ebsdlib;

// =============================================================================
// AngPhase Tests
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_Construction", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  REQUIRE(phase != nullptr);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_MaterialName", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  phase->setMaterialName("Iron");
  REQUIRE(phase->getMaterialName() == "Iron");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_Formula", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  phase->setFormula("Fe");
  REQUIRE(phase->getFormula() == "Fe");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_Symmetry", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  phase->setSymmetry(43);
  REQUIRE(phase->getSymmetry() == 43);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_PhaseIndex", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  phase->setPhaseIndex(1);
  REQUIRE(phase->getPhaseIndex() == 1);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_LatticeConstants", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  // Pre-allocate lattice constants vector (the setters index directly)
  phase->setLatticeConstants(std::vector<float>(6, 0.0f));
  phase->setLatticeConstantA(3.6f);
  phase->setLatticeConstantB(3.6f);
  phase->setLatticeConstantC(3.6f);
  phase->setLatticeConstantAlpha(90.0f);
  phase->setLatticeConstantBeta(90.0f);
  phase->setLatticeConstantGamma(90.0f);

  auto lc = phase->getLatticeConstants();
  REQUIRE(lc.size() == 6);
  REQUIRE(lc[0] == Approx(3.6f));
  REQUIRE(lc[1] == Approx(3.6f));
  REQUIRE(lc[2] == Approx(3.6f));
  REQUIRE(lc[3] == Approx(90.0f));
  REQUIRE(lc[4] == Approx(90.0f));
  REQUIRE(lc[5] == Approx(90.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_ParseLatticeConstants", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  std::vector<std::string> tokens = {"LatticeConstants", "3.600", "3.600", "3.600", "90.000", "90.000", "90.000"};
  phase->parseLatticeConstants(tokens);

  auto lc = phase->getLatticeConstants();
  REQUIRE(lc.size() == 6);
  REQUIRE(lc[0] == Approx(3.6f));
  REQUIRE(lc[1] == Approx(3.6f));
  REQUIRE(lc[2] == Approx(3.6f));
  REQUIRE(lc[3] == Approx(90.0f));
  REQUIRE(lc[4] == Approx(90.0f));
  REQUIRE(lc[5] == Approx(90.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::AngPhase_ParseHKLFamilies", "[EbsdLib][PhaseTest]")
{
  auto phase = AngPhase::New();
  std::vector<std::string> tokens = {"hklFamilies", "1", "1", "1", "1", "0.000000", "1"};
  phase->parseHKLFamilies(tokens);

  auto families = phase->getHKLFamilies();
  REQUIRE(families.size() == 1);
  REQUIRE(families[0]->h == 1);
  REQUIRE(families[0]->k == 1);
  REQUIRE(families[0]->l == 1);
}

// =============================================================================
// CtfPhase Tests
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::CtfPhase_Construction", "[EbsdLib][PhaseTest]")
{
  auto phase = CtfPhase::New();
  REQUIRE(phase != nullptr);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::CtfPhase_PhaseName", "[EbsdLib][PhaseTest]")
{
  auto phase = CtfPhase::New();
  phase->setPhaseName("Nickel");
  REQUIRE(phase->getPhaseName() == "Nickel");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::CtfPhase_PhaseIndex", "[EbsdLib][PhaseTest]")
{
  auto phase = CtfPhase::New();
  phase->setPhaseIndex(2);
  REQUIRE(phase->getPhaseIndex() == 2);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::CtfPhase_ParsePhase", "[EbsdLib][PhaseTest]")
{
  auto phase = CtfPhase::New();
  // Format: lattice_constants(;sep) TAB lattice_angles(;sep) TAB name TAB laue_group
  std::string line = "3.524;3.524;3.524\t90.000;90.000;90.000\tNickel\t11";
  phase->parsePhase(line);

  REQUIRE(phase->getPhaseName() == "Nickel");

  auto lc = phase->getLatticeConstants();
  REQUIRE(lc.size() == 6);
  REQUIRE(lc[0] == Approx(3.524f));
  REQUIRE(lc[1] == Approx(3.524f));
  REQUIRE(lc[2] == Approx(3.524f));
  REQUIRE(lc[3] == Approx(90.0f));
  REQUIRE(lc[4] == Approx(90.0f));
  REQUIRE(lc[5] == Approx(90.0f));

  REQUIRE(phase->getLaueGroup() == Ctf::LG_Cubic_High);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::CtfPhase_SpaceGroup", "[EbsdLib][PhaseTest]")
{
  auto phase = CtfPhase::New();
  phase->setSpaceGroup(225);
  REQUIRE(phase->getSpaceGroup() == 225);
}

// =============================================================================
// EspritPhase Tests
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::EspritPhase_Construction", "[EbsdLib][PhaseTest]")
{
  auto phase = EspritPhase::New();
  REQUIRE(phase != nullptr);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::EspritPhase_Name", "[EbsdLib][PhaseTest]")
{
  auto phase = EspritPhase::New();
  phase->setName("Aluminum");
  REQUIRE(phase->getName() == "Aluminum");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::EspritPhase_Formula", "[EbsdLib][PhaseTest]")
{
  auto phase = EspritPhase::New();
  phase->setFormula("Al");
  REQUIRE(phase->getFormula() == "Al");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::EspritPhase_SpaceGroup", "[EbsdLib][PhaseTest]")
{
  auto phase = EspritPhase::New();
  phase->setSpaceGroup("Fm-3m");
  REQUIRE(phase->getSpaceGroup() == "Fm-3m");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PhaseTest::EspritPhase_PhaseIndex", "[EbsdLib][PhaseTest]")
{
  auto phase = EspritPhase::New();
  phase->setPhaseIndex(3);
  REQUIRE(phase->getPhaseIndex() == 3);
}
