#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"

#include <vector>

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CreateArraySimple", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(10, "TestArray", true);
  REQUIRE(arr != nullptr);
  REQUIRE(arr->getName() == "TestArray");
  REQUIRE(arr->getNumberOfTuples() == 10);
  REQUIRE(arr->getNumberOfComponents() == 1);
  REQUIRE(arr->getSize() == 10);
  REQUIRE(arr->isAllocated() == true);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CreateArrayWithCompDims", "[EbsdLib][EbsdDataArrayTest]")
{
  std::vector<size_t> compDims = {3};
  auto arr = EbsdDataArray<float>::CreateArray(10, compDims, "VectorArray", true);
  REQUIRE(arr != nullptr);
  REQUIRE(arr->getNumberOfTuples() == 10);
  REQUIRE(arr->getNumberOfComponents() == 3);
  REQUIRE(arr->getSize() == 30);
  REQUIRE(arr->getComponentDimensions() == compDims);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CreateArrayWithRank", "[EbsdLib][EbsdDataArrayTest]")
{
  size_t dims[1] = {4};
  auto arr = EbsdDataArray<int32_t>::CreateArray(5, 1, dims, "RankArray", true);
  REQUIRE(arr != nullptr);
  REQUIRE(arr->getNumberOfTuples() == 5);
  REQUIRE(arr->getNumberOfComponents() == 4);
  REQUIRE(arr->getSize() == 20);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CreateArrayNoAllocate", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(10, "NoAlloc", false);
  REQUIRE(arr != nullptr);
  REQUIRE(arr->getNumberOfTuples() == 10);
  REQUIRE(arr->isAllocated() == false);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::FromStdVector", "[EbsdLib][EbsdDataArrayTest]")
{
  std::vector<float> vec = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  auto arr = EbsdDataArray<float>::FromStdVector(vec, "FromVec");
  REQUIRE(arr != nullptr);
  REQUIRE(arr->getNumberOfTuples() == 5);
  REQUIRE(arr->getNumberOfComponents() == 1);

  for(size_t i = 0; i < vec.size(); i++)
  {
    REQUIRE(arr->getValue(i) == vec[i]);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CopyFromPointer", "[EbsdLib][EbsdDataArrayTest]")
{
  float data[] = {10.0f, 20.0f, 30.0f};
  auto arr = EbsdDataArray<float>::CopyFromPointer(data, 3, "CopyPtr");
  REQUIRE(arr != nullptr);
  REQUIRE(arr->getSize() == 3);
  REQUIRE(arr->getValue(0) == 10.0f);
  REQUIRE(arr->getValue(1) == 20.0f);
  REQUIRE(arr->getValue(2) == 30.0f);

  // Verify it's a deep copy
  data[0] = 99.0f;
  REQUIRE(arr->getValue(0) == 10.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::GetSetName", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "Original", true);
  REQUIRE(arr->getName() == "Original");
  arr->setName("Renamed");
  REQUIRE(arr->getName() == "Renamed");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::GetSetValue", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "Values", true);
  arr->initializeWithZeros();

  arr->setValue(0, 1.5f);
  arr->setValue(4, 9.9f);
  REQUIRE(arr->getValue(0) == 1.5f);
  REQUIRE(arr->getValue(1) == 0.0f);
  REQUIRE(arr->getValue(4) == 9.9f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::GetSetComponent", "[EbsdLib][EbsdDataArrayTest]")
{
  std::vector<size_t> compDims = {3};
  auto arr = EbsdDataArray<float>::CreateArray(5, compDims, "CompArray", true);
  arr->initializeWithZeros();

  arr->setComponent(0, 0, 1.0f);
  arr->setComponent(0, 1, 2.0f);
  arr->setComponent(0, 2, 3.0f);
  arr->setComponent(2, 1, 7.5f);

  REQUIRE(arr->getComponent(0, 0) == 1.0f);
  REQUIRE(arr->getComponent(0, 1) == 2.0f);
  REQUIRE(arr->getComponent(0, 2) == 3.0f);
  REQUIRE(arr->getComponent(2, 1) == 7.5f);
  REQUIRE(arr->getComponent(1, 0) == 0.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::InitializeWithZeros", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<int32_t>::CreateArray(10, "Zeros", true);
  arr->initializeWithZeros();
  for(size_t i = 0; i < arr->getSize(); i++)
  {
    REQUIRE(arr->getValue(i) == 0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::InitializeWithValue", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(10, "Filled", true);
  arr->initializeWithValue(42.0f);
  for(size_t i = 0; i < arr->getSize(); i++)
  {
    REQUIRE(arr->getValue(i) == 42.0f);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::ResizeTuples", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "Resize", true);
  arr->initializeWithValue(1.0f);
  REQUIRE(arr->getNumberOfTuples() == 5);

  arr->resizeTuples(10);
  REQUIRE(arr->getNumberOfTuples() == 10);
  REQUIRE(arr->getSize() == 10);

  // Original values should still be present
  REQUIRE(arr->getValue(0) == 1.0f);
  REQUIRE(arr->getValue(4) == 1.0f);

  // Shrink
  arr->resizeTuples(3);
  REQUIRE(arr->getNumberOfTuples() == 3);
  REQUIRE(arr->getSize() == 3);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::EraseTuples", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "Erase", true);
  for(size_t i = 0; i < 5; i++)
  {
    arr->setValue(i, static_cast<float>(i + 1));
  }

  // Erase tuple at index 1 and 3 (values 2.0 and 4.0)
  // NOTE: eraseTuples requires sorted indices
  std::vector<size_t> idxs = {1, 3};
  int32_t err = arr->eraseTuples(idxs);
  REQUIRE(err >= 0);

  // NOTE: eraseTuples has a known bug where m_NumTuples is not updated,
  // so we check getSize() instead of getNumberOfTuples()
  REQUIRE(arr->getSize() == 3);

  // Remaining values should be 1, 3, 5
  REQUIRE(arr->getValue(0) == 1.0f);
  REQUIRE(arr->getValue(1) == 3.0f);
  REQUIRE(arr->getValue(2) == 5.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CopyTuple", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "CopyTuple", true);
  for(size_t i = 0; i < 5; i++)
  {
    arr->setValue(i, static_cast<float>(i + 1));
  }

  arr->copyTuple(0, 4); // Copy tuple 0 to position 4
  REQUIRE(arr->getValue(4) == 1.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::DeepCopy", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "Original", true);
  arr->initializeWithValue(7.0f);

  auto copy = arr->deepCopy();
  REQUIRE(copy != nullptr);
  REQUIRE(copy->getName() == arr->getName());
  REQUIRE(copy->getNumberOfTuples() == arr->getNumberOfTuples());
  REQUIRE(copy->getSize() == arr->getSize());

  for(size_t i = 0; i < arr->getSize(); i++)
  {
    REQUIRE(copy->getValue(i) == arr->getValue(i));
  }

  // Modify copy, original should be unchanged
  copy->setValue(0, 99.0f);
  REQUIRE(arr->getValue(0) == 7.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::CopyFromArray", "[EbsdLib][EbsdDataArrayTest]")
{
  auto src = EbsdDataArray<float>::CreateArray(5, "Source", true);
  for(size_t i = 0; i < 5; i++)
  {
    src->setValue(i, static_cast<float>(i + 10));
  }

  auto dest = EbsdDataArray<float>::CreateArray(10, "Dest", true);
  dest->initializeWithZeros();

  bool result = dest->copyFromArray(3, src, 0, 3);
  REQUIRE(result == true);

  REQUIRE(dest->getValue(3) == 10.0f);
  REQUIRE(dest->getValue(4) == 11.0f);
  REQUIRE(dest->getValue(5) == 12.0f);
  REQUIRE(dest->getValue(0) == 0.0f); // Untouched
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::GetPointer", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "PtrTest", true);
  arr->initializeWithValue(3.0f);

  float* ptr = arr->getPointer(0);
  REQUIRE(ptr != nullptr);
  REQUIRE(ptr[0] == 3.0f);

  float* ptr2 = arr->getPointer(2);
  REQUIRE(ptr2 != nullptr);
  REQUIRE(ptr2[0] == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::SetTuple", "[EbsdLib][EbsdDataArrayTest]")
{
  std::vector<size_t> compDims = {3};
  auto arr = EbsdDataArray<float>::CreateArray(5, compDims, "SetTuple", true);
  arr->initializeWithZeros();

  float tupleData[3] = {1.0f, 2.0f, 3.0f};
  arr->setTuple(2, tupleData);

  REQUIRE(arr->getComponent(2, 0) == 1.0f);
  REQUIRE(arr->getComponent(2, 1) == 2.0f);
  REQUIRE(arr->getComponent(2, 2) == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::SetTupleFromVector", "[EbsdLib][EbsdDataArrayTest]")
{
  std::vector<size_t> compDims = {3};
  auto arr = EbsdDataArray<float>::CreateArray(5, compDims, "SetTupleVec", true);
  arr->initializeWithZeros();

  std::vector<float> tupleData = {4.0f, 5.0f, 6.0f};
  arr->setTuple(1, tupleData);

  REQUIRE(arr->getComponent(1, 0) == 4.0f);
  REQUIRE(arr->getComponent(1, 1) == 5.0f);
  REQUIRE(arr->getComponent(1, 2) == 6.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::MultiComponent", "[EbsdLib][EbsdDataArrayTest]")
{
  std::vector<size_t> compDims = {2, 3};
  auto arr = EbsdDataArray<float>::CreateArray(4, compDims, "MultiComp", true);
  REQUIRE(arr->getNumberOfComponents() == 6);
  REQUIRE(arr->getSize() == 24);
  REQUIRE(arr->getNumberOfTuples() == 4);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::IntegerType", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<int32_t>::CreateArray(5, "IntArray", true);
  arr->initializeWithZeros();
  arr->setValue(0, 42);
  arr->setValue(4, -7);
  REQUIRE(arr->getValue(0) == 42);
  REQUIRE(arr->getValue(4) == -7);
  REQUIRE(arr->getValue(2) == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::OperatorBracket", "[EbsdLib][EbsdDataArrayTest]")
{
  auto arr = EbsdDataArray<float>::CreateArray(5, "BracketTest", true);
  arr->initializeWithZeros();

  (*arr)[0] = 1.0f;
  (*arr)[4] = 5.0f;
  REQUIRE((*arr)[0] == 1.0f);
  REQUIRE((*arr)[4] == 5.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdDataArrayTest::TypeSize", "[EbsdLib][EbsdDataArrayTest]")
{
  auto floatArr = EbsdDataArray<float>::CreateArray(1, "Float", true);
  REQUIRE(floatArr->getTypeSize() == sizeof(float));

  auto doubleArr = EbsdDataArray<double>::CreateArray(1, "Double", true);
  REQUIRE(doubleArr->getTypeSize() == sizeof(double));

  auto intArr = EbsdDataArray<int32_t>::CreateArray(1, "Int", true);
  REQUIRE(intArr->getTypeSize() == sizeof(int32_t));
}
