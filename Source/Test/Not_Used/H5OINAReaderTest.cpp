#include <catch2/catch.hpp>

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/H5OINAReader.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "UnitTestSupport.hpp"

#include <fstream>
#include <string>

using namespace ebsdlib;
#define H5OINA_CHECK_POINTERS(name, def, type)                                                                                                                                                         \
  {                                                                                                                                                                                                    \
    type* ptr0 = reader->get##name##Pointer();                                                                                                                                                         \
    void* ptr1 = reader->getPointerByName(def);                                                                                                                                                        \
    REQUIRE(ptr0 == ptr1);                                                                                                                                                                             \
  }

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::H5OINAReaderTest::TestH5OINAReader", "[EbsdLib][AngleFileLoader]")
{
  const std::string k_HDF5Path = std::string("1");
  H5OINAReader::Pointer reader = H5OINAReader::New();
  reader->setFileName("/Users/mjackson/Desktop/Alumin.h5oina");
  reader->setHDF5Path(k_HDF5Path);
  reader->readHeaderOnly();

  int32_t err = reader->getErrorCode();
  DREAM3D_REQUIRED(err, ==, 0)

  err = reader->readFile();
  DREAM3D_REQUIRED(err, ==, 0)

  H5OINA_CHECK_POINTERS(BandContrast, ebsdlib::H5OINA::BandContrast, uint8_t)
  H5OINA_CHECK_POINTERS(BandSlope, ebsdlib::H5OINA::BandSlope, uint8_t)
  H5OINA_CHECK_POINTERS(Bands, ebsdlib::H5OINA::Bands, uint8_t)
  H5OINA_CHECK_POINTERS(Error, ebsdlib::H5OINA::Error, uint8_t)
  H5OINA_CHECK_POINTERS(Euler, ebsdlib::H5OINA::Euler, float)
  H5OINA_CHECK_POINTERS(MeanAngularDeviation, ebsdlib::H5OINA::MeanAngularDeviation, float)
  H5OINA_CHECK_POINTERS(Phase, ebsdlib::H5OINA::Phase, uint8_t)
  H5OINA_CHECK_POINTERS(X, ebsdlib::H5OINA::X, float)
  H5OINA_CHECK_POINTERS(Y, ebsdlib::H5OINA::Y, float)
}
