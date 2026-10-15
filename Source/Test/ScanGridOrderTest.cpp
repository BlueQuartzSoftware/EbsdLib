#include "EbsdLib/IO/ScanGridOrder.h"

#include <catch2/catch.hpp>

#include <array>
#include <cstdint>
#include <vector>

using namespace ebsdlib;

TEST_CASE("ebsdlib::ScanGridOrderTest::IdentityFromIndices", "[EbsdLib][ScanGridOrderTest]")
{
  // 3 columns x 2 rows already in raster order
  const std::array<int32_t, 6> cols = {0, 1, 2, 0, 1, 2};
  const std::array<int32_t, 6> rows = {0, 0, 0, 1, 1, 1};
  const ScanGridOrder order = BuildScanGridOrderFromIndices(cols.data(), rows.data(), 6, 3, 2);
  REQUIRE(order.isValid());
  REQUIRE(order.isIdentity);
  REQUIRE(order.indexMap == std::vector<int64_t>{0, 1, 2, 3, 4, 5});
}

TEST_CASE("ebsdlib::ScanGridOrderTest::SwappedPairFromIndices", "[EbsdLib][ScanGridOrderTest]")
{
  // tuples 1 and 2 are swapped, like the Bruker file at tuples 30/31
  const std::array<int32_t, 6> cols = {0, 2, 1, 0, 1, 2};
  const std::array<int32_t, 6> rows = {0, 0, 0, 1, 1, 1};
  const ScanGridOrder order = BuildScanGridOrderFromIndices(cols.data(), rows.data(), 6, 3, 2);
  REQUIRE(order.isValid());
  REQUIRE_FALSE(order.isIdentity);
  REQUIRE(order.indexMap == std::vector<int64_t>{0, 2, 1, 3, 4, 5});
}

TEST_CASE("ebsdlib::ScanGridOrderTest::RejectsDuplicateRasterSlot", "[EbsdLib][ScanGridOrderTest]")
{
  const std::array<int32_t, 6> cols = {0, 1, 1, 0, 1, 2};
  const std::array<int32_t, 6> rows = {0, 0, 0, 1, 1, 1};
  const ScanGridOrder order = BuildScanGridOrderFromIndices(cols.data(), rows.data(), 6, 3, 2);
  REQUIRE_FALSE(order.isValid());
  REQUIRE(order.errorCode == -101103);
  REQUIRE(order.indexMap.empty());
  REQUIRE(order.errorMessage.find("tuple 2") != std::string::npos);
}

TEST_CASE("ebsdlib::ScanGridOrderTest::RejectsOutOfRangeIndex", "[EbsdLib][ScanGridOrderTest]")
{
  const std::array<int32_t, 6> cols = {0, 1, 3, 0, 1, 2};
  const std::array<int32_t, 6> rows = {0, 0, 0, 1, 1, 1};
  const ScanGridOrder order = BuildScanGridOrderFromIndices(cols.data(), rows.data(), 6, 3, 2);
  REQUIRE(order.errorCode == -101102);
}

TEST_CASE("ebsdlib::ScanGridOrderTest::RejectsTupleCountMismatch", "[EbsdLib][ScanGridOrderTest]")
{
  const std::array<int32_t, 5> cols = {0, 1, 2, 0, 1};
  const std::array<int32_t, 5> rows = {0, 0, 0, 1, 1};
  const ScanGridOrder order = BuildScanGridOrderFromIndices(cols.data(), rows.data(), 5, 3, 2);
  REQUIRE(order.errorCode == -101100);
}

TEST_CASE("ebsdlib::ScanGridOrderTest::RejectsNullPointer", "[EbsdLib][ScanGridOrderTest]")
{
  const std::array<int32_t, 6> rows = {0, 0, 0, 1, 1, 1};
  const ScanGridOrder order = BuildScanGridOrderFromIndices(nullptr, rows.data(), 6, 3, 2);
  REQUIRE(order.errorCode == -101101);
}

TEST_CASE("ebsdlib::ScanGridOrderTest::RoundsPositionsToNearestCell", "[EbsdLib][ScanGridOrderTest]")
{
  // step 0.25, positions carry float noise that floor() would misplace
  const std::array<float, 6> x = {1.75f, 1.9999999f, 2.2500002f, 1.75f, 2.0f, 2.25f};
  const std::array<float, 6> y = {0.25f, 0.25f, 0.25f, 0.4999999f, 0.5f, 0.5f};
  const ScanGridOrder order = BuildScanGridOrderFromPositions(x.data(), y.data(), 6, 3, 2, 0.25f, 0.25f);
  REQUIRE(order.isValid());
  REQUIRE(order.isIdentity);
}

TEST_CASE("ebsdlib::ScanGridOrderTest::ShuffledPositions", "[EbsdLib][ScanGridOrderTest]")
{
  // raster order would be (0,0) (1,0) (2,0) (0,1) (1,1) (2,1) with step 2.0 and origin at 10,20
  const std::array<float, 6> x = {14.0f, 10.0f, 12.0f, 12.0f, 10.0f, 14.0f};
  const std::array<float, 6> y = {22.0f, 20.0f, 20.0f, 22.0f, 22.0f, 20.0f};
  const ScanGridOrder order = BuildScanGridOrderFromPositions(x.data(), y.data(), 6, 3, 2, 2.0f, 2.0f);
  REQUIRE(order.isValid());
  REQUIRE_FALSE(order.isIdentity);
  REQUIRE(order.indexMap == std::vector<int64_t>{5, 0, 1, 4, 3, 2});
}

TEST_CASE("ebsdlib::ScanGridOrderTest::RejectsBadStepAndExtent", "[EbsdLib][ScanGridOrderTest]")
{
  const std::array<float, 6> x = {0.0f, 1.0f, 2.0f, 0.0f, 1.0f, 2.0f};
  const std::array<float, 6> y = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
  REQUIRE(BuildScanGridOrderFromPositions(x.data(), y.data(), 6, 3, 2, 0.0f, 1.0f).errorCode == -101104);
  // extent says 3 columns but caller claims 2 columns x 3 rows
  REQUIRE(BuildScanGridOrderFromPositions(x.data(), y.data(), 6, 2, 3, 1.0f, 1.0f).errorCode == -101105);
}

TEST_CASE("ebsdlib::ScanGridOrderTest::ReordersMultiComponentTuples", "[EbsdLib][ScanGridOrderTest]")
{
  // 3 tuples x 2 components; map sends tuple 0 -> 2, 1 -> 0, 2 -> 1
  std::vector<float> data = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f};
  const std::vector<int64_t> indexMap = {2, 0, 1};
  ReorderScanTuples<float>(data.data(), 3, 2, indexMap);
  REQUIRE(data == std::vector<float>{1.0f, 1.5f, 2.0f, 2.5f, 0.0f, 0.5f});

  std::vector<uint8_t> patterns = {1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
  ReorderScanTuples<uint8_t>(patterns.data(), 3, 4, indexMap);
  REQUIRE(patterns == std::vector<uint8_t>{2, 2, 2, 2, 3, 3, 3, 3, 1, 1, 1, 1});
}

TEST_CASE("ebsdlib::ScanGridOrderTest::ReorderIsNoOpForNullOrMismatchedMap", "[EbsdLib][ScanGridOrderTest]")
{
  std::vector<int32_t> data = {7, 8, 9};
  ReorderScanTuples<int32_t>(nullptr, 3, 1, {2, 0, 1});
  ReorderScanTuples<int32_t>(data.data(), 3, 1, {});
  ReorderScanTuples<int32_t>(data.data(), 3, 1, {1, 0});
  REQUIRE(data == std::vector<int32_t>{7, 8, 9});
}
