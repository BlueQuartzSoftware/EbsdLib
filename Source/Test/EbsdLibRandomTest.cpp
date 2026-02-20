#include <catch2/catch.hpp>

#include "EbsdLib/Math/EbsdLibRandom.h"

#include <cstdint>
#include <limits>

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::SeededDeterministic", "[EbsdLib][EbsdLibRandomTest]")
{
  // Two generators seeded the same should produce the same sequence
  EbsdLibRandom rg1;
  rg1.init_genrand(12345);

  EbsdLibRandom rg2;
  rg2.init_genrand(12345);

  for(int i = 0; i < 100; i++)
  {
    REQUIRE(rg1.genrand_int32() == rg2.genrand_int32());
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::DifferentSeeds", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg1;
  rg1.init_genrand(12345);

  EbsdLibRandom rg2;
  rg2.init_genrand(54321);

  // Different seeds should produce different sequences (very high probability)
  bool allSame = true;
  for(int i = 0; i < 100; i++)
  {
    if(rg1.genrand_int32() != rg2.genrand_int32())
    {
      allSame = false;
      break;
    }
  }
  REQUIRE(allSame == false);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::GenRandReal1Range", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg;
  rg.init_genrand(42);

  // genrand_real1() should generate values in [0, 1]
  for(int i = 0; i < 1000; i++)
  {
    double val = rg.genrand_real1();
    REQUIRE(val >= 0.0);
    REQUIRE(val <= 1.0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::GenRandReal2Range", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg;
  rg.init_genrand(42);

  // genrand_real2() should generate values in [0, 1)
  for(int i = 0; i < 1000; i++)
  {
    double val = rg.genrand_real2();
    REQUIRE(val >= 0.0);
    REQUIRE(val < 1.0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::GenRandReal3Range", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg;
  rg.init_genrand(42);

  // genrand_real3() should generate values in (0, 1)
  for(int i = 0; i < 1000; i++)
  {
    double val = rg.genrand_real3();
    REQUIRE(val > 0.0);
    REQUIRE(val < 1.0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::GenRandRes53Range", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg;
  rg.init_genrand(42);

  // genrand_res53() should generate values in [0, 1)
  for(int i = 0; i < 1000; i++)
  {
    double val = rg.genrand_res53();
    REQUIRE(val >= 0.0);
    REQUIRE(val < 1.0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::GenRandInt31Range", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg;
  rg.init_genrand(42);

  // genrand_int31() should generate values in [0, 2^31 - 1]
  for(int i = 0; i < 1000; i++)
  {
    long val = rg.genrand_int31();
    REQUIRE(val >= 0);
    REQUIRE(val <= 0x7FFFFFFF);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::InitByArray", "[EbsdLib][EbsdLibRandomTest]")
{
  unsigned long initKey[4] = {0x123, 0x234, 0x345, 0x456};

  EbsdLibRandom rg1;
  rg1.init_by_array(initKey, 4);

  EbsdLibRandom rg2;
  rg2.init_by_array(initKey, 4);

  // Same init_key should produce same sequence
  for(int i = 0; i < 100; i++)
  {
    REQUIRE(rg1.genrand_int32() == rg2.genrand_int32());
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::EbsdLibRandomTest::ZeroSeed", "[EbsdLib][EbsdLibRandomTest]")
{
  EbsdLibRandom rg;
  rg.init_genrand(0);

  // Should still produce valid output
  for(int i = 0; i < 100; i++)
  {
    double val = rg.genrand_real1();
    REQUIRE(val >= 0.0);
    REQUIRE(val <= 1.0);
  }
}
