#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/MarkerOccupancyGrid.h"

using namespace ebsdlib;

TEST_CASE("ebsdlib::MarkerOccupancyGridTest::Decimation", "[EbsdLib][MarkerOccupancyGridTest]")
{
  // 100x100 area at origin (0,0), 10px cells.
  MarkerOccupancyGrid grid(0.0f, 0.0f, 100.0f, 100.0f, 10.0f);

  // First point in a cell draws; a second point in the same cell does not.
  REQUIRE(grid.shouldDraw(5.0f, 5.0f) == true);
  REQUIRE(grid.shouldDraw(6.0f, 4.0f) == false); // same 10px cell

  // A point in a different cell draws.
  REQUIRE(grid.shouldDraw(55.0f, 55.0f) == true);
  REQUIRE(grid.shouldDraw(59.0f, 51.0f) == false); // same cell as (55,55)

  // Out-of-bounds points never draw.
  REQUIRE(grid.shouldDraw(-1.0f, 5.0f) == false);
  REQUIRE(grid.shouldDraw(5.0f, 1000.0f) == false);
}
