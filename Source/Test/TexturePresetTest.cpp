#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Texture/TexturePreset.h"

#include <string>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TexturePresetTest::Construction", "[EbsdLib][TexturePresetTest]")
{
  auto preset = TexturePreset::New(CrystalStructure::Cubic_High, "TestPreset", 10.0, 20.0, 30.0);
  REQUIRE(preset != nullptr);
  REQUIRE(preset->getCrystalStructure() == CrystalStructure::Cubic_High);
  REQUIRE(preset->getName() == "TestPreset");
  REQUIRE(preset->getEuler1() == Approx(10.0));
  REQUIRE(preset->getEuler2() == Approx(20.0));
  REQUIRE(preset->getEuler3() == Approx(30.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TexturePresetTest::SetterGetterRoundTrip", "[EbsdLib][TexturePresetTest]")
{
  auto preset = TexturePreset::New();
  REQUIRE(preset != nullptr);

  preset->setCrystalStructure(CrystalStructure::Hexagonal_High);
  REQUIRE(preset->getCrystalStructure() == CrystalStructure::Hexagonal_High);

  preset->setName("MyPreset");
  REQUIRE(preset->getName() == "MyPreset");

  preset->setEuler1(45.0);
  REQUIRE(preset->getEuler1() == Approx(45.0));

  preset->setEuler2(90.0);
  REQUIRE(preset->getEuler2() == Approx(90.0));

  preset->setEuler3(135.0);
  REQUIRE(preset->getEuler3() == Approx(135.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TexturePresetTest::CubicTexturePresets", "[EbsdLib][TexturePresetTest]")
{
  auto textures = CubicTexturePresets::getTextures();

  // Should return exactly 14 presets
  REQUIRE(textures.size() == 14);

  // All should have Cubic_High crystal structure
  for(const auto& preset : textures)
  {
    REQUIRE(preset->getCrystalStructure() == CrystalStructure::Cubic_High);
  }

  // Verify known names are present
  std::vector<std::string> expectedNames = {"Brass", "S", "Copper", "Goss", "Cube"};
  for(const auto& expectedName : expectedNames)
  {
    bool found = false;
    for(const auto& preset : textures)
    {
      if(preset->getName() == expectedName)
      {
        found = true;
        break;
      }
    }
    CHECK(found);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TexturePresetTest::HexTexturePresets", "[EbsdLib][TexturePresetTest]")
{
  auto textures = HexTexturePresets::getTextures();
  // Currently returns empty vector
  REQUIRE(textures.empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TexturePresetTest::NullPointer", "[EbsdLib][TexturePresetTest]")
{
  auto ptr = TexturePreset::NullPointer();
  REQUIRE(ptr == nullptr);
}
