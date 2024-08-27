
#include <fstream>
#include <string>

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"
#include "H5Support/H5Utilities.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/IO/HKL/H5OINAReader.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "UnitTestSupport.hpp"

class H5OINAReaderTest
{
  const std::string k_HDF5Path = std::string("1");

public:
  H5OINAReaderTest() = default;
  ~H5OINAReaderTest() = default;

  EBSD_GET_NAME_OF_CLASS_DECL(H5OINAReaderTest)

  // -----------------------------------------------------------------------------
  void RemoveTestFiles()
  {
#if REMOVE_TEST_FILES
    //  fs::remove(UnitTest::H5OINAReaderTest::OutputFile);
#endif
  }

#define H5OINA_CHECK_POINTERS(name, def, type)                                                                                                                                                         \
  {                                                                                                                                                                                                    \
    type* ptr0 = reader->get##name##Pointer();                                                                                                                                                         \
    void* ptr1 = reader->getPointerByName(def);                                                                                                                                                        \
    DREAM3D_REQUIRED(ptr0, ==, ptr1)                                                                                                                                                                   \
  }

  // -----------------------------------------------------------------------------
  void TestH5OINAReader()
  {
    H5OINAReader::Pointer reader = H5OINAReader::New();
    reader->setFileName("/Users/mjackson/Desktop/Alumin.h5oina");
    reader->setHDF5Path(k_HDF5Path);
    reader->readHeaderOnly();

    int32_t err = reader->getErrorCode();
    DREAM3D_REQUIRED(err, ==, 0)

    err = reader->readFile();
    DREAM3D_REQUIRED(err, ==, 0)

    H5OINA_CHECK_POINTERS(BandContrast, EbsdLib::H5OINA::BandContrast, uint8_t)
    H5OINA_CHECK_POINTERS(BandSlope, EbsdLib::H5OINA::BandSlope, uint8_t)
    H5OINA_CHECK_POINTERS(Bands, EbsdLib::H5OINA::Bands, uint8_t)
    H5OINA_CHECK_POINTERS(Error, EbsdLib::H5OINA::Error, uint8_t)
    H5OINA_CHECK_POINTERS(Euler, EbsdLib::H5OINA::Euler, float)
    H5OINA_CHECK_POINTERS(MeanAngularDeviation, EbsdLib::H5OINA::MeanAngularDeviation, float)
    H5OINA_CHECK_POINTERS(Phase, EbsdLib::H5OINA::Phase, uint8_t)
    H5OINA_CHECK_POINTERS(X, EbsdLib::H5OINA::X, float)
    H5OINA_CHECK_POINTERS(Y, EbsdLib::H5OINA::Y, float)
  }

  // -----------------------------------------------------------------------------
  void operator()()
  {
    int err = EXIT_SUCCESS;

    std::cout << "<===== Start " << getNameOfClass() << std::endl;

    DREAM3D_REGISTER_TEST(TestH5OINAReader())

    DREAM3D_REGISTER_TEST(RemoveTestFiles())
  }

public:
  H5OINAReaderTest(const H5OINAReaderTest&) = delete;            // Copy Constructor Not Implemented
  H5OINAReaderTest(H5OINAReaderTest&&) = delete;                 // Move Constructor Not Implemented
  H5OINAReaderTest& operator=(const H5OINAReaderTest&) = delete; // Copy Assignment Not Implemented
  H5OINAReaderTest& operator=(H5OINAReaderTest&&) = delete;      // Move Assignment Not Implemented
};
