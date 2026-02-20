#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/ToolTipGenerator.h"

#include <string>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::DefaultConstruction", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  REQUIRE(gen.getRowColorStr() == "#FFFCEA");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::SetGetRowColor", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  gen.setRowColorStr("#FF0000");
  REQUIRE(gen.getRowColorStr() == "#FF0000");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::AddTitle_GenerateHTML", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  gen.addTitle("Test Title");
  std::string html = gen.generateHTML();

  REQUIRE_FALSE(html.empty());
  REQUIRE(html.find("<table") != std::string::npos);
  REQUIRE(html.find("<th") != std::string::npos);
  REQUIRE(html.find("Test Title") != std::string::npos);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::AddValue_GenerateHTML", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  gen.addValue("Width", "100");
  std::string html = gen.generateHTML();

  REQUIRE_FALSE(html.empty());
  REQUIRE(html.find("Width") != std::string::npos);
  REQUIRE(html.find("100") != std::string::npos);
  REQUIRE(html.find("<th") != std::string::npos);
  REQUIRE(html.find("<td>") != std::string::npos);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::AddSpacer_GenerateHTML", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  gen.addSpacer();
  std::string html = gen.generateHTML();

  REQUIRE_FALSE(html.empty());
  // Spacer rows have empty td cells
  REQUIRE(html.find("<td></td><td></td>") != std::string::npos);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::Clear", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  gen.addTitle("Title1");
  gen.addValue("Name", "Value");
  gen.clear();
  std::string html = gen.generateHTML();

  // After clear, only the trailing spacer should be present (no Title1 or Name)
  REQUIRE(html.find("Title1") == std::string::npos);
  REQUIRE(html.find("Name") == std::string::npos);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::Append", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen1;
  gen1.addTitle("First");

  ToolTipGenerator gen2;
  gen2.addTitle("Second");

  gen1.append(gen2);
  std::string html = gen1.generateHTML();

  REQUIRE(html.find("First") != std::string::npos);
  REQUIRE(html.find("Second") != std::string::npos);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ToolTipGeneratorTest::RowColorInHTML", "[EbsdLib][ToolTipGeneratorTest]")
{
  ToolTipGenerator gen;
  gen.setRowColorStr("#AABBCC");
  gen.addTitle("Color Test");
  std::string html = gen.generateHTML();

  REQUIRE(html.find("#AABBCC") != std::string::npos);
}
