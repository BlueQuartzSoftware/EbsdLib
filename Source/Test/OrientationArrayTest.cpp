
#include <cstdlib>
#include <algorithm>
#include <complex>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#if __APPLE__
#include <Accelerate/Accelerate.h>
#endif

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <vector>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/ArrayHelpers.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/EbsdMatrixMath.h"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"

#include "TestPrintFunctions.h"
#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

using namespace EbsdLib::Constants;

class OrientationArrayTest
{
public:
  OrientationArrayTest() = default;
  virtual ~OrientationArrayTest() = default;

  OrientationArrayTest(const OrientationArrayTest&) = delete;            // Copy Constructor Not Implemented
  OrientationArrayTest(OrientationArrayTest&&) = delete;                 // Move Constructor Not Implemented
  OrientationArrayTest& operator=(const OrientationArrayTest&) = delete; // Copy Assignment Not Implemented
  OrientationArrayTest& operator=(OrientationArrayTest&&) = delete;      // Move Assignment Not Implemented

  using FOrientArrayType = Orientation<float>;
  using FloatVectorType = std::vector<float>;
  using FloatQVectorType = std::vector<float>;
  using DoubleVectorType = std::vector<double>;
  using DoubleQVectorType = std::vector<double>;

  EBSD_GET_NAME_OF_CLASS_DECL(OrientationArrayTest)

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T>
  T transfer_sign(T a, T b)
  {
    if(a > 0.0 && b > 0.0)
    {
      return a;
    }
    if(a < 0.0 && b > 0.0)
    {
      return -1 * a;
    }

    if(a < 0.0 && b < 0.0)
    {
      return a;
    }

    return -1 * a;
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void TestRotArray()
  {
    FOrientArrayType ro(3);
    ro[0] = 2;
    ro[1] = 4;
    ro[2] = 6;
    //  float sum = ro.sum();
    //  DREAM3D_REQUIRE_EQUAL(sum, 12);

    //  DREAM3D_REQUIRE_EQUAL(ro.size(), 3);

    //  DREAM3D_REQUIRE_EQUAL(ro.maxval(), 6);
    //  DREAM3D_REQUIRE_EQUAL(ro.minval(), 2);

    //  float pro = ro.product();
    //  DREAM3D_REQUIRE_EQUAL(pro, 48);
    //  sum = result.sum();
    //  float max = result.maxval();
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_eu_check()
  {
    {
      OrientationTransformation::ResultType result;
      using TestType = FOrientArrayType;
      FOrientArrayType eu_r(3);
      eu_r[0] = 0.81585413f;
      eu_r[1] = 3.00f;
      eu_r[2] = 0.8661895f;
      result = OrientationTransformation::eu_check<TestType>(eu_r);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      eu_r[1] = eu_r[1] - EbsdLib::Constants::k_PiF;
      result = OrientationTransformation::eu_check<TestType>(eu_r);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }
    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;
      FloatVectorType eu_v(3);
      eu_v[0] = 1.0F;
      eu_v[1] = 0.4f;
      eu_v[2] = 0.9f;
      result = OrientationTransformation::eu_check<TestType>(eu_v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      eu_v[0] = -1.0;
      result = OrientationTransformation::eu_check<TestType>(eu_v);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
    {
      using TestType = std::vector<float>;
      using FloatQVectorType = std::vector<float>;
      OrientationTransformation::ResultType result;
      FloatQVectorType eu_q(3);
      eu_q[0] = 1.0F;
      eu_q[1] = 0.4f;
      eu_q[2] = 0.9f;
      result = OrientationTransformation::eu_check<TestType>(eu_q);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      eu_q[0] = -1.0;
      result = OrientationTransformation::eu_check<TestType>(eu_q);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ro_check()
  {
    {
      OrientationTransformation::ResultType result;
      FOrientArrayType ro(4);
      using TestType = FOrientArrayType;

      ro[0] = 1.0F;
      ro[1] = 1.0F;
      ro[2] = 1.0F;
      ro[3] = 1.0F;
      EbsdMatrixMath::Normalize3x1(&(ro[0]));
      result = OrientationTransformation::ro_check<TestType>(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ro[3] = -1.0;
      result = OrientationTransformation::ro_check<TestType>(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;

      FloatVectorType ro(4);
      ro[0] = 1.0F;
      ro[1] = 1.0F;
      ro[2] = 1.0F;
      ro[3] = 1.0F;
      EbsdMatrixMath::Normalize3x1(&(ro[0]));
      result = OrientationTransformation::ro_check<TestType>(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ro[3] = -1.0;
      result = OrientationTransformation::ro_check<TestType>(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;
      TestType ro(4);
      ro[0] = 1.0f;
      ro[1] = 1.0f;
      ro[2] = 1.0f;
      ro[3] = 1.0f;
      EbsdMatrixMath::Normalize3x1(&(ro[0]));
      result = OrientationTransformation::ro_check<TestType>(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ro[3] = -1.0;
      result = OrientationTransformation::ro_check<TestType>(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ho_check()
  {
    {
      using TestType = FOrientArrayType;
      OrientationTransformation::ResultType result;
      FOrientArrayType ho(3);
      ho[0] = 0.5f;
      ho[1] = 0.5f;
      ho[2] = 0.5f;
      result = OrientationTransformation::ho_check<TestType>(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ho[2] = 8.0;
      result = OrientationTransformation::ho_check<TestType>(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;

      FloatVectorType ho(3);
      ho[0] = 0.5f;
      ho[1] = 0.5f;
      ho[2] = 0.5f;
      result = OrientationTransformation::ho_check<TestType>(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ho[2] = 8.0;
      result = OrientationTransformation::ho_check<TestType>(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;

      OrientationTransformation::ResultType result;
      TestType ho(3);
      ho[0] = 0.5f;
      ho[1] = 0.5f;
      ho[2] = 0.5f;
      result = OrientationTransformation::ho_check<TestType>(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ho[2] = 8.0;
      result = OrientationTransformation::ho_check<TestType>(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_cu_check()
  {
    {
      using TestType = FOrientArrayType;
      OrientationTransformation::ResultType result;
      FOrientArrayType v(3);
      v[0] = 0.5f;
      v[1] = 0.5f;
      v[2] = 0.5f;
      result = OrientationTransformation::cu_check<TestType>(v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      v[2] = 8.0;
      result = OrientationTransformation::cu_check<TestType>(v);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;
      FloatVectorType v(3);
      v[0] = 0.5f;
      v[1] = 0.5f;
      v[2] = 0.5f;
      result = OrientationTransformation::cu_check<TestType>(v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      v[2] = 8.0;
      result = OrientationTransformation::cu_check<TestType>(v);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;

      OrientationTransformation::ResultType result;
      FloatQVectorType v(3);
      v[0] = 0.5f;
      v[1] = 0.5f;
      v[2] = 0.5f;
      result = OrientationTransformation::cu_check<TestType>(v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      v[2] = 8.0;
      result = OrientationTransformation::cu_check<TestType>(v);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_qu_check()
  {
    QuatF quat(1.0, 1.0, 1.0, 0.0);
    quat = quat.unitQuaternion();

    {
      using TestType = FOrientArrayType;
      OrientationTransformation::ResultType result;
      FOrientArrayType qu(4);
      qu[0] = quat.x();
      qu[1] = quat.y();
      qu[2] = quat.z();
      qu[3] = quat.w();
      EbsdMatrixMath::Normalize3x1(&(qu[0]));
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      qu[0] = 1.5f;
      qu[1] = 3.0f;
      qu[2] = 2.0f;
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      qu[0] = -1.0;
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;
      FloatVectorType qu(4);
      qu[0] = quat.x();
      qu[1] = quat.y();
      qu[2] = quat.z();
      qu[3] = quat.w();
      EbsdMatrixMath::Normalize3x1(&(qu[0]));
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      qu[0] = 1.5f;
      qu[1] = 3.0f;
      qu[2] = 2.0f;
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      qu[0] = -1.0;
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }

    {
      using TestType = std::vector<float>;

      OrientationTransformation::ResultType result;
      TestType qu(4);
      qu[0] = quat.x();
      qu[1] = quat.y();
      qu[2] = quat.z();
      qu[3] = quat.w();
      EbsdMatrixMath::Normalize3x1(&(qu[0]));
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      qu[0] = 1.5f;
      qu[1] = 3.0f;
      qu[2] = 2.0f;
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      qu[0] = -1.0;
      result = OrientationTransformation::qu_check<TestType>(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ax_check()
  {
    {
      using TestType = FOrientArrayType;
      OrientationTransformation::ResultType result;
      FOrientArrayType ax(4);
      ax[0] = 0.0F;
      ax[1] = 0.0F;
      ax[2] = 1.0F;
      ax[3] = EbsdLib::Constants::k_PiF - 0.00001F;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[0] = 1.0;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0F;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;
      FloatVectorType ax(4);
      ax[0] = 0.0F;
      ax[1] = 0.0F;
      ax[2] = 1.0F;
      ax[3] = EbsdLib::Constants::k_PiF - 0.00001F;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[0] = 1.0;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0F;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      using TestType = std::vector<float>;

      OrientationTransformation::ResultType result;
      TestType ax(4);
      ax[0] = 0.0f;
      ax[1] = 0.0f;
      ax[2] = 1.0f;
      ax[3] = EbsdLib::Constants::k_PiF - 0.00001f;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[0] = 1.0F;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0F;
      result = OrientationTransformation::ax_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_om_check()
  {
    {
      using TestType = FOrientArrayType;
      OrientationTransformation::ResultType result;
      FOrientArrayType ax(9);
      ax[0] = 1.0F;
      ax[1] = 0.0F;
      ax[2] = 0.0F;
      ax[3] = 0.0F;
      ax[4] = 1.0F;
      ax[5] = 0.0F;
      ax[6] = 0.0F;
      ax[7] = 0.0F;
      ax[8] = 1.0F;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[1] = -16.0f;
      ax[6] = -12.0f;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -3);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0F;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }

    {
      using TestType = std::vector<float>;
      OrientationTransformation::ResultType result;
      FloatVectorType ax(9);
      ax[0] = 1.0F;
      ax[1] = 0.0F;
      ax[2] = 0.0F;
      ax[3] = 0.0F;
      ax[4] = 1.0F;
      ax[5] = 0.0F;
      ax[6] = 0.0F;
      ax[7] = 0.0F;
      ax[8] = 1.0F;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[1] = -16.0f;
      ax[6] = -12.0f;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -3);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0F;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }

    {
      using TestType = std::vector<float>;

      OrientationTransformation::ResultType result;
      TestType ax(9);
      ax[0] = 1.0f;
      ax[1] = 0.0f;
      ax[2] = 0.0f;
      ax[3] = 0.0f;
      ax[4] = 1.0f;
      ax[5] = 0.0f;
      ax[6] = 0.0f;
      ax[7] = 0.0f;
      ax[8] = 1.0f;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[1] = -16.0f;
      ax[6] = -12.0f;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -3);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0F;
      result = OrientationTransformation::om_check<TestType>(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void GenRotTest(K* in, K omega)
  {
    T eu(3);
    eu[0] = in[0];
    eu[1] = in[1];
    eu[2] = in[2];

    T res(4);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_GenRot()
  {

    std::array<float, 3> eu = {1.0f, 0.0f, 0.0f};
    float omega = EbsdLib::Constants::k_PiOver2F;
    GenRotTest<FOrientArrayType, float>(eu.data(), omega);
    GenRotTest<FloatVectorType, float>(eu.data(), omega);
    GenRotTest<FloatQVectorType, float>(eu.data(), omega);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename FloatType>
  void EU_2_XXX(FloatType* in)
  {

    using TestType = T;

    T eu(3);
    eu[0] = in[0];
    eu[1] = in[1];
    eu[2] = in[2];

    eu[0] = static_cast<FloatType>(std::fmod(eu[0], EbsdLib::Constants::k_2PiD));
    eu[1] = static_cast<FloatType>(std::fmod(eu[1], EbsdLib::Constants::k_PiD));
    eu[2] = static_cast<FloatType>(std::fmod(eu[2], EbsdLib::Constants::k_2PiD));

    T res(9);

    typename OrientationTransformation::ResultType result;
    result = OrientationTransformation::eu_check<TestType>(eu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T ax = OrientationTransformation::eu2ax<T, T>(eu);
    OrientationPrinters::Print_AX<T>(ax);
    result = OrientationTransformation::ax_check<TestType>(ax);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T ro(4);
    ro = OrientationTransformation::eu2ro<T, T>(eu);
    OrientationPrinters::Print_RO<T>(ro);
    result = OrientationTransformation::ro_check<TestType>(ro);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T ho(3);
    ho = OrientationTransformation::eu2ho<T, T>(eu);
    OrientationPrinters::Print_HO<T>(ho);
    result = OrientationTransformation::ho_check<TestType>(ho);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T cu(3);
    cu = OrientationTransformation::eu2cu<T, T>(eu);
    OrientationPrinters::Print_CU<T>(cu);
    result = OrientationTransformation::cu_check<TestType>(cu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    cu = OrientationTransformation::ho2cu<T, T>(ho);
    OrientationPrinters::Print_CU<T>(cu);
    result = OrientationTransformation::cu_check<TestType>(cu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    QuatF qu = OrientationTransformation::eu2qu<T, QuatF>(eu);
    OrientationPrinters::Print_QU<QuatF>(qu);
    result = OrientationTransformation::qu_check<QuatF>(qu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T om(9);
    om = OrientationTransformation::eu2om<T, T>(eu);
    OrientationPrinters::Print_OM<T>(om);
    result = OrientationTransformation::om_check<TestType>(om);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }
  }

  /*
Starting Test ax2eu  eu2ax  -----------------------------------------------------
Total Tuples: 4913
Delta Failed: 1.99037 DataArray: 'ax Difference' Tuple[263] Comp[1] Value:-1.99037
eu
0.0000000000000000   2.9452431201934814   3.1415927410125732
ax
-0.0000000435009078  -0.9951847195625305   0.0980171412229538   3.1415927410125732
0_ax2eu
6.2831850051879883   2.9452428817749023   3.1415922641754150
1_eu2ax
-0.0000000435009078   0.9951847195625305  -0.0980172604322433   3.1415925025939941
------------------------------------------
ax2eu  eu2ax                                                     FAILED
*/

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename K>
  void Test_eu2_XXX()
  {
    using OrientType = Orientation<K>;
    using VectorType = std::vector<K>;
    using QVectorType = std::vector<K>;
    {
      K eu[3] = {static_cast<K>(0.3926990816987242L), static_cast<K>(0.0L), static_cast<K>(0.0L)};
      OrientationPrinters::Print_EU<K*>(eu);
      EU_2_XXX<OrientType, K>(eu);
      EU_2_XXX<VectorType, K>(eu);
      EU_2_XXX<QVectorType, K>(eu);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void OM_2_XXX(K* in)
  {
    std::cout << "   " << std::endl;
    T om(9);
    for(typename T::size_type i = 0; i < 9; i++)
    {
      om[i] = in[i];
    }
    T res(3); // Just size to 9 as we are going to reuse the variable
    // Convert to Euler
    res = OrientationTransformation::om2eu<T, T>(om);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Rodriques
    res = OrientationTransformation::om2ro<T, T>(om);
    OrientationPrinters::Print_RO<T>(res);

    // Convert to Quaternion
    Quaternion<K> quat = OrientationTransformation::om2qu<T, Quaternion<K>>(om);
    OrientationPrinters::Print_QU<Quaternion<K>>(quat);

    // Convert to Axis Angle
    res = OrientationTransformation::om2ax<T, T>(om);
    OrientationPrinters::Print_AX<T>(res);

    // Convert to Homochoric
    res = OrientationTransformation::om2ho<T, T>(om);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::om2cu<T, T>(om);
    OrientationPrinters::Print_CU<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_om2_XXX()
  {
    /*
----------------------------------------------------
Euler angles                     :  90.0000000    0.0000000    0.0000000
Axis angle pair [n; angle]       :  -0.0000000   -0.0000000   -1.0000000 ;   90.0000000
Rodrigues vector                 :      -0.0000000       -0.0000000       -1.0000000
Homochoric representation        :  -0.0000000   -0.0000000   -0.7536693
Cubochoric representation        :   0.0000000    0.0000000   -0.6074544
Quaternion                       :   0.7071068   -0.0000000   -0.0000000   -0.7071068
                                 /  0.0000   1.0000   0.0000 \
Orientation Matrix               : | -1.0000   0.0000   0.0000 |
                                 \  0.0000   0.0000   1.0000 /
*/
    std::cout << "Test_om2_XXX  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    float om[9] = {0.0000, 1.0000, 0.0000, -1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000};
    OrientationPrinters::Print_OM<float*>(om);
    OM_2_XXX<FOrientArrayType>(om);
    OM_2_XXX<FloatVectorType>(om);
    OM_2_XXX<FloatQVectorType>(om);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void RO_2_XXX(K* in)
  {
    T ro(9);
    for(typename T::size_type i = 0; i < 4; i++)
    {
      ro[i] = in[i];
    }

    T res(9); // Just size to 9 as we are going to reuse the variable


    // Convert to Euler
    res = OrientationTransformation::ro2eu<T, T>(ro);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Orientation Matrix
    res = OrientationTransformation::ro2om<T, T>(ro);
    OrientationPrinters::Print_OM<T>(res);

    // Convert to Axis Angle
    res = OrientationTransformation::ro2ax<T, T>(ro);
    OrientationPrinters::Print_AX<T>(res);

    // Convert to Quaternion

    Quaternion<K> quat = OrientationTransformation::ro2qu<T, Quaternion<K>>(ro);
    OrientationPrinters::Print_QU<Quaternion<K>>(quat);

    // Convert to Homochoric
    res = OrientationTransformation::ro2ho<T, T>(ro);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::ro2cu<T, T>(ro);
    OrientationPrinters::Print_CU<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ro2_XXX()
  {
    std::cout << "Test_ro2_XXX  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
    float ro[4] = {0.0f, 0.0f, -1.0f, 1.0f};
    OrientationPrinters::Print_RO<float*>(ro);
    RO_2_XXX<FOrientArrayType>(ro);
    RO_2_XXX<FloatVectorType>(ro);
    RO_2_XXX<FloatQVectorType>(ro);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void AX_2_XXX(K* in)
  {
    T ax(4);
    ax[0] = in[0];
    ax[1] = in[1];
    ax[2] = in[2];
    ax[3] = in[3];

    T res(9);
    // Convert to Orientation Matrix
    res = OrientationTransformation::ax2om<T, T>(ax);
    OrientationPrinters::Print_OM<T>(res);

    // Convert to Axis Angle
    res = OrientationTransformation::ax2eu<T, T>(ax);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Rodriques
    res = OrientationTransformation::ax2ro<T, T>(ax);
    OrientationPrinters::Print_RO<T>(res);

    // Convert to Quaternion
    Quaternion<K> quat = OrientationTransformation::ax2qu<T, Quaternion<K>>(ax);
    OrientationPrinters::Print_QU<Quaternion<K>>(quat);

    // Convert to homochoric
    res = OrientationTransformation::ax2ho<T, T>(ax);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::ax2cu<T, T>(ax);
    OrientationPrinters::Print_CU<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ax2_XXX()
  {
    std::cout << "Test_ax2_XXX  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
    float ax[4] = {0.0F, 0.0F, -1.0F, k_PiOver2F};
    OrientationPrinters::Print_AX<float*>(ax);
    AX_2_XXX<FOrientArrayType>(ax);
    AX_2_XXX<std::vector<float>>(ax);
    AX_2_XXX<FloatQVectorType>(ax);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void QU_2_XXX(K* in, typename Quaternion<K>::Order layout = Quaternion<K>::Order::VectorScalar)
  {
    using QuatType = Quaternion<K>;
    QuatType qu(in[0], in[1], in[2], in[3]);

    // Convert to Orientation Matrix
    T res = OrientationTransformation::qu2om<QuatType, T>(qu, layout);
    OrientationPrinters::Print_OM<T>(res);

    // Convert to Axis Angle
    res = OrientationTransformation::qu2eu<QuatType, T>(qu, layout);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Rodriques
    res = OrientationTransformation::qu2ro<QuatType, T>(qu, layout);
    OrientationPrinters::Print_RO<T>(res);

    // Convert to Quaternion
    res = OrientationTransformation::qu2ax<QuatType, T>(qu, layout);
    OrientationPrinters::Print_AX<T>(res);

    // Convert to Homochoric
    res = OrientationTransformation::qu2ho<QuatType, T>(qu, layout);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::qu2cu<QuatType, T>(qu, layout);
    OrientationPrinters::Print_CU<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_qu2_XXX()
  {
    {
      std::cout << "Test_qu2_XXX  (SCALAR, <X, Y, Z>) ***************************************" << std::endl;
      float qu[4] = {EbsdLib::Constants::k_1OverRoot2F, 0.0f, 0.0f, -EbsdLib::Constants::k_1OverRoot2F};
      OrientationPrinters::Print_QU<QuatD>({qu[0], qu[1], qu[2], qu[3]}, QuatD::Order::ScalarVector);
      QU_2_XXX<FOrientArrayType>(qu, QuatF::Order::ScalarVector);
      //  QU_2_XXX<std::vector<float> >(qu);
      //  QU_2_XXX<FloatQVectorType>(qu);
    }

    {
      std::cout << "Test_qu2_XXX  (<X, Y, Z>, SCALAR) ***************************************" << std::endl;
      float qu[4] = {0.0f, 0.0f, -EbsdLib::Constants::k_1OverRoot2F, EbsdLib::Constants::k_1OverRoot2F};
      OrientationPrinters::Print_QU<QuatD>({qu[0], qu[1], qu[2], qu[3]});
      QU_2_XXX<FOrientArrayType>(qu);
      //  QU_2_XXX<std::vector<float> >(qu);
      //  QU_2_XXX<FloatQVectorType>(qu);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void HO_2_XXX(K* in)
  {
    using QuatType = Quaternion<K>;

    T ho(3);
    ho[0] = in[0];
    ho[1] = in[1];
    ho[2] = in[2];

    T res(9);

    // Convert to Euler
    res = OrientationTransformation::ho2eu<T, T>(ho);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Orientation Matrix
    res = OrientationTransformation::ho2om<T, T>(ho);
    OrientationPrinters::Print_OM<T>(res);

    // Convert to Axis Angle
    res = OrientationTransformation::ho2ax<T, T>(ho);
    OrientationPrinters::Print_AX<T>(res);

    // Convert to Rodriques
    res = OrientationTransformation::ho2ro<T, T>(ho);
    OrientationPrinters::Print_RO<T>(res);

    // Convert to Quaternion
    QuatType quat = OrientationTransformation::ho2qu<T, QuatType>(ho);
    OrientationPrinters::Print_QU<QuatType>(quat);

    // Convert to HomoChoric
    res = OrientationTransformation::ho2cu<T, T>(ho);
    OrientationPrinters::Print_CU<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ho2_XXX()
  {
    std::cout << "Test_ho2_XXX  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
    float ho[3] = {0.000000f, 0.000000f, -0.7536693215f};
    OrientationPrinters::Print_HO<float*>(ho);
    HO_2_XXX<FOrientArrayType>(ho);
    HO_2_XXX<std::vector<float>>(ho);
    HO_2_XXX<FloatQVectorType>(ho);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void TestInputs()
  {
    const float k_PiOver180f = EbsdLib::Constants::k_PiOver180F;
    std::vector<size_t> cDims(1, 3);
    EbsdLib::FloatArrayType::Pointer data = EbsdLib::FloatArrayType::CreateArray(2, cDims, "Eulers", true);
    data->initializeWithZeros();
    float* fPtr = data->getPointer(0);
    fPtr[0] = 90.0F * k_PiOver180f;
    fPtr[1] = 0.0F;
    fPtr[2] = 0.0F;
    fPtr[3] = 90.0F * k_PiOver180f;
    fPtr[4] = 0.0F;
    fPtr[5] = 0.0F;

    //& Notation
    {
      FOrientArrayType eu(&(fPtr[0]), 3); // Wrap the pointer with the &notation
      eu[0] = 45.0F * k_PiOver180f;
      eu[1] = 90.0F * k_PiOver180f;
      eu[2] = 135.0F * k_PiOver180f;

      DREAM3D_REQUIRE_EQUAL(eu[0], fPtr[0]);
      DREAM3D_REQUIRE_EQUAL(eu[1], fPtr[1]);
      DREAM3D_REQUIRE_EQUAL(eu[2], fPtr[2]);
    }

    // Pointer Arithmetic (inputs)
    {
      FOrientArrayType eu(fPtr + 3, 3);
      eu[0] = 135.0F * k_PiOver180f;
      eu[1] = 45.0F * k_PiOver180f;
      eu[2] = 90.0F * k_PiOver180f;

      DREAM3D_REQUIRE_EQUAL(eu[0], fPtr[3]);
      DREAM3D_REQUIRE_EQUAL(eu[1], fPtr[4]);
      DREAM3D_REQUIRE_EQUAL(eu[2], fPtr[5]);
    }

    // Pointer Arithmetic, placing results directly into an array
    {
      FOrientArrayType ax(0.0F, 0.0F, -1.0F, EbsdLib::Constants::k_PiOver2F);
      FOrientArrayType eu = OrientationTransformation::ax2eu<FOrientArrayType, FOrientArrayType>(ax);
      eu.copyInto(fPtr + 3, 3);
      DREAM3D_REQUIRE_EQUAL(eu[0], fPtr[3]);
      DREAM3D_REQUIRE_EQUAL(eu[1], fPtr[4]);
      DREAM3D_REQUIRE_EQUAL(eu[2], fPtr[5]);

      bool b = EbsdLibMath::closeEnough(eu[0], fPtr[3]);
      DREAM3D_REQUIRE_EQUAL(b, true)
      b = EbsdLibMath::closeEnough(eu[1], fPtr[4]);
      DREAM3D_REQUIRE_EQUAL(b, true)
      b = EbsdLibMath::closeEnough(eu[2], fPtr[5]);
      DREAM3D_REQUIRE_EQUAL(b, true)
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void QuatTest()
  {
    QuatF q(-0.5f, -0.5f, -0.5f, 0.5f);
    using VecType = std::vector<float>;
    VecType qVec = {-0.5f, -0.5f, -0.5f, 0.5f};

    VecType gVec = OrientationTransformation::qu2om<VecType, VecType>(qVec);

    OrientationPrinters::Print_OM<std::vector<float>>(gVec);

    float g[3][3];

    g[0][0] = gVec[0];
    g[0][1] = gVec[1];
    g[0][2] = gVec[2];
    g[1][0] = gVec[3];
    g[1][1] = gVec[4];
    g[1][2] = gVec[5];
    g[2][0] = gVec[6];
    g[2][1] = gVec[7];
    g[2][2] = gVec[8];

    for(size_t r = 0; r < 3; r++)
    {
      for(size_t c = 0; c < 3; c++)
      {
        g[c][r] = gVec[3 * r + c];
        std::cout << c << "," << r << "=" << g[c][r] << std::endl;
      }
    }

    std::array<float, 3> vg;
    std::array<float, 3> v = {1.0F, 0.0F, 0.0F};

    // Mathematically correct, results unintuitive
    std::array<float, 3> vq = q.multiplyByVector(v.data());

    std::cout << "vq: " << vq[0] << "," << vq[1] << "," << vq[2] << std::endl;

    EbsdMatrixMath::Multiply3x3with3x1(g, v.data(), vg.data());
    std::cout << "vg: " << vg[0] << "," << vg[1] << "," << vg[2] << std::endl;
  }

  // -----------------------------------------------------------------------------
  void operator()()
  {
    std::cout << "<===== Start " << getNameOfClass() << std::endl;

    int err = EXIT_SUCCESS;
    DREAM3D_REGISTER_TEST(TestRotArray());
    DREAM3D_REGISTER_TEST(Test_eu_check());
    DREAM3D_REGISTER_TEST(Test_ro_check());
    DREAM3D_REGISTER_TEST(Test_ho_check());
    DREAM3D_REGISTER_TEST(Test_cu_check());
    DREAM3D_REGISTER_TEST(Test_qu_check());
    DREAM3D_REGISTER_TEST(Test_ax_check());
    DREAM3D_REGISTER_TEST(Test_om_check());

    DREAM3D_REGISTER_TEST(Test_GenRot())

    DREAM3D_REGISTER_TEST(Test_eu2_XXX<double>());
    DREAM3D_REGISTER_TEST(Test_eu2_XXX<float>());

    DREAM3D_REGISTER_TEST(Test_ax2_XXX());
    DREAM3D_REGISTER_TEST(Test_om2_XXX());
    DREAM3D_REGISTER_TEST(Test_ro2_XXX());
    DREAM3D_REGISTER_TEST(Test_qu2_XXX());
    DREAM3D_REGISTER_TEST(Test_ho2_XXX());

    DREAM3D_REGISTER_TEST(TestInputs());
  }
};
