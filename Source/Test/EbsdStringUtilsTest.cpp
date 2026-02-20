#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#include <string>
#include <vector>

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitSingleChar", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("a,b,c", ',');
  REQUIRE(tokens.size() == 3);
  REQUIRE(tokens[0] == "a");
  REQUIRE(tokens[1] == "b");
  REQUIRE(tokens[2] == "c");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitNoDelimiter", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("hello", ',');
  REQUIRE(tokens.size() == 1);
  REQUIRE(tokens[0] == "hello");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitEmptyString", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("", ',');
  REQUIRE(tokens.empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitMultipleDelimiters", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("a,b;c", {',', ';'}, false);
  REQUIRE(tokens.size() == 3);
  REQUIRE(tokens[0] == "a");
  REQUIRE(tokens[1] == "b");
  REQUIRE(tokens[2] == "c");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitConsecutiveDelimiters", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("a,,b", {','}, true);
  REQUIRE(tokens.size() == 3);
  REQUIRE(tokens[0] == "a");
  REQUIRE(tokens[1] == "");
  REQUIRE(tokens[2] == "b");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitIgnoreConsecutive", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("a,,b", {','}, false);
  REQUIRE(tokens.size() == 2);
  REQUIRE(tokens[0] == "a");
  REQUIRE(tokens[1] == "b");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SpecificSplitIgnoreEmpty", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::specific_split(",a,,b,", {','}, EbsdStringUtils::IgnoreEmpty);
  REQUIRE(tokens.size() == 2);
  REQUIRE(tokens[0] == "a");
  REQUIRE(tokens[1] == "b");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SpecificSplitAllowAll", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::specific_split(",a,,b,", {','}, EbsdStringUtils::AllowAll);
  REQUIRE(tokens.size() == 5);
  REQUIRE(tokens[0] == "");
  REQUIRE(tokens[1] == "a");
  REQUIRE(tokens[2] == "");
  REQUIRE(tokens[3] == "b");
  REQUIRE(tokens[4] == "");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SpecificSplitOnlyConsecutive", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::specific_split(",a,,b,", {','}, EbsdStringUtils::OnlyConsecutive);
  // Leading/trailing empty stripped, but consecutive empty kept
  REQUIRE(tokens.size() == 3);
  REQUIRE(tokens[0] == "a");
  REQUIRE(tokens[1] == "");
  REQUIRE(tokens[2] == "b");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::Replace", "[EbsdLib][EbsdStringUtilsTest]")
{
  std::string result = EbsdStringUtils::replace("hello world", "world", "there");
  REQUIRE(result == "hello there");

  // Multiple occurrences
  result = EbsdStringUtils::replace("aabaa", "a", "x");
  REQUIRE(result == "xxbxx");

  // No match
  result = EbsdStringUtils::replace("hello", "xyz", "abc");
  REQUIRE(result == "hello");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::LTrim", "[EbsdLib][EbsdStringUtilsTest]")
{
  REQUIRE(EbsdStringUtils::ltrim("  hello") == "hello");
  REQUIRE(EbsdStringUtils::ltrim("\t\nhello") == "hello");
  REQUIRE(EbsdStringUtils::ltrim("hello  ") == "hello  ");
  REQUIRE(EbsdStringUtils::ltrim("") == "");
  REQUIRE(EbsdStringUtils::ltrim("   ") == "");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::RTrim", "[EbsdLib][EbsdStringUtilsTest]")
{
  REQUIRE(EbsdStringUtils::rtrim("hello  ") == "hello");
  REQUIRE(EbsdStringUtils::rtrim("hello\t\n") == "hello");
  REQUIRE(EbsdStringUtils::rtrim("  hello") == "  hello");
  REQUIRE(EbsdStringUtils::rtrim("") == "");
  REQUIRE(EbsdStringUtils::rtrim("   ") == "");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::Trimmed", "[EbsdLib][EbsdStringUtilsTest]")
{
  REQUIRE(EbsdStringUtils::trimmed("  hello  ") == "hello");
  REQUIRE(EbsdStringUtils::trimmed("\t hello \n") == "hello");
  REQUIRE(EbsdStringUtils::trimmed("hello") == "hello");
  REQUIRE(EbsdStringUtils::trimmed("") == "");
  REQUIRE(EbsdStringUtils::trimmed("   ") == "");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::Chop", "[EbsdLib][EbsdStringUtilsTest]")
{
  REQUIRE(EbsdStringUtils::chop("hello", 1) == "hell");
  REQUIRE(EbsdStringUtils::chop("hello", 3) == "he");
  REQUIRE(EbsdStringUtils::chop("hello", 5) == "");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::Number", "[EbsdLib][EbsdStringUtilsTest]")
{
  REQUIRE(EbsdStringUtils::number(42) == "42");
  REQUIRE(EbsdStringUtils::number(-7) == "-7");
  REQUIRE(EbsdStringUtils::number(0) == "0");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::Simplified", "[EbsdLib][EbsdStringUtilsTest]")
{
  // simplified() trims leading/trailing whitespace and replaces interior
  // whitespace chars with spaces (but does not collapse consecutive spaces)
  REQUIRE(EbsdStringUtils::simplified("  hello   world  ") == "hello   world");
  REQUIRE(EbsdStringUtils::simplified("hello") == "hello");
  REQUIRE(EbsdStringUtils::simplified("") == "");
  REQUIRE(EbsdStringUtils::simplified("\thello\tworld\t") == "hello world");
  REQUIRE(EbsdStringUtils::simplified("  a  ") == "a");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitTabDelimited", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("col1\tcol2\tcol3", '\t');
  REQUIRE(tokens.size() == 3);
  REQUIRE(tokens[0] == "col1");
  REQUIRE(tokens[1] == "col2");
  REQUIRE(tokens[2] == "col3");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdStringUtilsTest::SplitSpaceDelimited", "[EbsdLib][EbsdStringUtilsTest]")
{
  auto tokens = EbsdStringUtils::split("1.0 2.0 3.0", ' ');
  REQUIRE(tokens.size() == 3);
  REQUIRE(tokens[0] == "1.0");
  REQUIRE(tokens[1] == "2.0");
  REQUIRE(tokens[2] == "3.0");
}
