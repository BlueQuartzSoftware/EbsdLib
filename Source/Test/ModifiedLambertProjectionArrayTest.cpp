#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/ModifiedLambertProjectionArray.h"

#include <string>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::NewReturnsValid", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  REQUIRE(arr != nullptr);
  REQUIRE(arr->isAllocated() == true);
  REQUIRE(arr->getNumberOfTuples() == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::SetGetName", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->setName("TestArray");
  REQUIRE(arr->getName() == "TestArray");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::SetGetPhase", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->setPhase(42);
  REQUIRE(arr->getPhase() == 42);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::FillArray", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(5);
  REQUIRE(arr->getNumberOfTuples() == 5);

  for(size_t i = 0; i < 5; i++)
  {
    REQUIRE(arr->getModifiedLambertProjection(static_cast<int>(i)) != nullptr);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::SetGetModifiedLambertProjection", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(3);

  auto proj = ModifiedLambertProjection::New();
  arr->setModifiedLambertProjection(1, proj);

  REQUIRE(arr->getModifiedLambertProjection(1) == proj);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::OperatorBracket", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(3);

  auto proj = ModifiedLambertProjection::New();
  arr->setModifiedLambertProjection(2, proj);

  REQUIRE((*arr)[2] == proj);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::ResizeTuples", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(3);
  REQUIRE(arr->getNumberOfTuples() == 3);

  arr->resizeTuples(10);
  REQUIRE(arr->getNumberOfTuples() == 10);

  arr->resizeTuples(2);
  REQUIRE(arr->getNumberOfTuples() == 2);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::CopyTuple", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(4);

  auto proj = ModifiedLambertProjection::New();
  arr->setModifiedLambertProjection(0, proj);

  arr->copyTuple(0, 3);
  REQUIRE(arr->getModifiedLambertProjection(3) == proj);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::EraseTuples", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(5);
  REQUIRE(arr->getNumberOfTuples() == 5);

  std::vector<size_t> idxs = {1, 3};
  int err = arr->eraseTuples(idxs);
  REQUIRE(err == 0);
  REQUIRE(arr->getNumberOfTuples() == 3);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::EraseTuples_InvalidIndex", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(3);

  std::vector<size_t> idxs = {10};
  int err = arr->eraseTuples(idxs);
  REQUIRE(err == -100);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::DeepCopy", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(4);
  arr->setName("Original");

  auto copy = arr->deepCopy();
  REQUIRE(copy != nullptr);
  REQUIRE(copy->getNumberOfTuples() == 4);

  // Verify independence: modify original, check copy is unaffected
  arr->resizeTuples(1);
  REQUIRE(copy->getNumberOfTuples() == 4);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::ClearAll", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  arr->fillArrayWithNewModifiedLambertProjection(5);
  arr->clearAll();
  REQUIRE(arr->getNumberOfTuples() == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::NumberOfComponents", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  REQUIRE(arr->getNumberOfComponents() == 1);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::Rank", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  REQUIRE(arr->getRank() == 1);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::ComponentDimensions", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  std::vector<size_t> dims = arr->getComponentDimensions();
  REQUIRE(dims.size() == 1);
  REQUIRE(dims[0] == 1);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::TypeAsString", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  REQUIRE(arr->getTypeAsString() == "ModifiedLambertProjectionArray");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionArrayTest::NameOfClass", "[EbsdLib][ModifiedLambertProjectionArrayTest]")
{
  auto arr = ModifiedLambertProjectionArray::New();
  REQUIRE(arr->getNameOfClass() == "ModifiedLambertProjectionArray");
  REQUIRE(ModifiedLambertProjectionArray::ClassName() == "ModifiedLambertProjectionArray");
}
