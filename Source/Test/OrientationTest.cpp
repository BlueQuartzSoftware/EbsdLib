
#include <algorithm>
#include <complex>
#include <cstdlib>
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
#include "EbsdLib/Core/Orientation.hpp"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/ArrayHelpers.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/EbsdMatrixMath.h"

using namespace EbsdLib::Constants;

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "TestPrintFunctions.h"
#include "UnitTestSupport.hpp"

class OrientationTest
{
public:
  OrientationTest() = default;
  virtual ~OrientationTest() = default;

  using FloatVectorType = std::vector<float>;
  using DoubleVectorType = std::vector<double>;

  EBSD_GET_NAME_OF_CLASS_DECL(OrientationTest)

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
    OrientationF ro(3);
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
      OrientationF eu_r(3);
      eu_r[0] = 0.81585413f;
      eu_r[1] = 3.00f;
      eu_r[2] = 0.8661895f;
      result = OrientationTransformation::eu_check(eu_r);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      eu_r[1] = eu_r[1] - EbsdLib::Constants::k_PiF;
      result = OrientationTransformation::eu_check(eu_r);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }
    {
      OrientationTransformation::ResultType result;
      FloatVectorType eu_v(3);
      eu_v[0] = 1.0f;
      eu_v[1] = 0.4f;
      eu_v[2] = 0.9f;
      result = OrientationTransformation::eu_check(eu_v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      eu_v[0] = -1.0;
      result = OrientationTransformation::eu_check(eu_v);
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
      OrientationF ro(4);
      ro[0] = 1.0f;
      ro[1] = 1.0f;
      ro[2] = 1.0f;
      ro[3] = 1.0f;
      EbsdMatrixMath::Normalize3x1(&(ro[0]));
      result = OrientationTransformation::ro_check(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ro[3] = -1.0;
      result = OrientationTransformation::ro_check(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      OrientationTransformation::ResultType result;
      FloatVectorType ro(4);
      ro[0] = 1.0f;
      ro[1] = 1.0f;
      ro[2] = 1.0f;
      ro[3] = 1.0f;
      EbsdMatrixMath::Normalize3x1(&(ro[0]));
      result = OrientationTransformation::ro_check(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ro[3] = -1.0;
      result = OrientationTransformation::ro_check(ro);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ho_check()
  {
    {
      OrientationTransformation::ResultType result;
      OrientationF ho(3);
      ho[0] = 0.5f;
      ho[1] = 0.5f;
      ho[2] = 0.5f;
      result = OrientationTransformation::ho_check(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ho[2] = 8.0;
      result = OrientationTransformation::ho_check(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      OrientationTransformation::ResultType result;
      FloatVectorType ho(3);
      ho[0] = 0.5f;
      ho[1] = 0.5f;
      ho[2] = 0.5f;
      result = OrientationTransformation::ho_check(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      ho[2] = 8.0;
      result = OrientationTransformation::ho_check(ho);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_cu_check()
  {
    {
      OrientationTransformation::ResultType result;
      OrientationF v(3);
      v[0] = 0.5f;
      v[1] = 0.5f;
      v[2] = 0.5f;
      result = OrientationTransformation::cu_check(v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      v[2] = 8.0;
      result = OrientationTransformation::cu_check(v);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      OrientationTransformation::ResultType result;
      FloatVectorType v(3);
      v[0] = 0.5f;
      v[1] = 0.5f;
      v[2] = 0.5f;
      result = OrientationTransformation::cu_check(v);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);
      v[2] = 8.0;
      result = OrientationTransformation::cu_check(v);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_qu_check()
  {
    QuatF quat(1.0, 1.0, 1.0, 0.0);
    quat.unitQuaternion();

    {
      OrientationTransformation::ResultType result;
      OrientationF qu(4);
      qu[0] = quat.x();
      qu[1] = quat.y();
      qu[2] = quat.z();
      qu[3] = quat.w();
      EbsdMatrixMath::Normalize3x1(&(qu[0]));
      result = OrientationTransformation::qu_check(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      qu[0] = 1.5f;
      qu[1] = 3.0f;
      qu[2] = 2.0f;
      result = OrientationTransformation::qu_check(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      qu[0] = -1.0;
      result = OrientationTransformation::qu_check(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }

    {
      OrientationTransformation::ResultType result;
      FloatVectorType qu(4);
      qu[0] = quat.x();
      qu[1] = quat.y();
      qu[2] = quat.z();
      qu[3] = quat.w();
      EbsdMatrixMath::Normalize3x1(&(qu[0]));
      result = OrientationTransformation::qu_check(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      qu[0] = 1.5f;
      qu[1] = 3.0f;
      qu[2] = 2.0f;
      result = OrientationTransformation::qu_check(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      qu[0] = -1.0;
      result = OrientationTransformation::qu_check(qu);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ax_check()
  {
    {

      OrientationTransformation::ResultType result;
      OrientationF ax(4);
      ax[0] = 0.0f;
      ax[1] = 0.0f;
      ax[2] = 1.0f;
      ax[3] = EbsdLib::Constants::k_PiF - 0.00001f;
      result = OrientationTransformation::ax_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[0] = 1.0;
      result = OrientationTransformation::ax_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0f;
      result = OrientationTransformation::ax_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }

    {
      OrientationTransformation::ResultType result;
      FloatVectorType ax(4);
      ax[0] = 0.0f;
      ax[1] = 0.0f;
      ax[2] = 1.0f;
      ax[3] = EbsdLib::Constants::k_PiF - 0.00001f;
      result = OrientationTransformation::ax_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[0] = 1.0;
      result = OrientationTransformation::ax_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0f;
      result = OrientationTransformation::ax_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -1);
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_om_check()
  {
    {

      OrientationTransformation::ResultType result;
      OrientationF ax(9);
      ax[0] = 1.0f;
      ax[1] = 0.0f;
      ax[2] = 0.0f;
      ax[3] = 0.0f;
      ax[4] = 1.0f;
      ax[5] = 0.0f;
      ax[6] = 0.0f;
      ax[7] = 0.0f;
      ax[8] = 1.0f;
      result = OrientationTransformation::om_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[1] = -16.0f;
      ax[6] = -12.0f;
      result = OrientationTransformation::om_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -3);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0f;
      result = OrientationTransformation::om_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -2);
    }

    {
      OrientationTransformation::ResultType result;
      FloatVectorType ax(9);
      ax[0] = 1.0f;
      ax[1] = 0.0f;
      ax[2] = 0.0f;
      ax[3] = 0.0f;
      ax[4] = 1.0f;
      ax[5] = 0.0f;
      ax[6] = 0.0f;
      ax[7] = 0.0f;
      ax[8] = 1.0f;
      result = OrientationTransformation::om_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, 1);

      ax[1] = -16.0f;
      ax[6] = -12.0f;
      result = OrientationTransformation::om_check(ax);
      DREAM3D_REQUIRE_EQUAL(result.result, -3);

      ax[3] = EbsdLib::Constants::k_PiF + 1.0f;
      result = OrientationTransformation::om_check(ax);
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

    float eu[3] = {1.0f, 0.0f, 0.0f};
    float omega = EbsdLib::Constants::k_PiOver2F;
    GenRotTest<OrientationF, float>(eu, omega);
    GenRotTest<FloatVectorType, float>(eu, omega);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void EU_2_XXX(K* in)
  {

    T eu(3);
    eu[0] = in[0];
    eu[1] = in[1];
    eu[2] = in[2];

    eu[0] = static_cast<K>(std::fmod(eu[0], EbsdLib::Constants::k_2PiD));
    eu[1] = static_cast<K>(std::fmod(eu[1], EbsdLib::Constants::k_PiD));
    eu[2] = static_cast<K>(std::fmod(eu[2], EbsdLib::Constants::k_2PiD));

    T res(9);

    OrientationTransformation::ResultType result = OrientationTransformation::eu_check(eu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T ax = OrientationTransformation::eu2ax<T, T>(eu);
    OrientationPrinters::Print_AX<T>(ax);
    result = OrientationTransformation::ax_check(ax);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T ro = OrientationTransformation::eu2ro<T, T>(eu);
    OrientationPrinters::Print_RO<T>(ro);
    result = OrientationTransformation::ro_check(ro);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T ho = OrientationTransformation::eu2ho<T, T>(eu);
    OrientationPrinters::Print_HO<T>(ho);
    result = OrientationTransformation::ho_check(ho);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T cu = OrientationTransformation::eu2cu<T, T>(eu);
    OrientationPrinters::Print_CU<T>(cu);
    result = OrientationTransformation::cu_check(cu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    cu = OrientationTransformation::ho2cu<T, T>(ho);
    OrientationPrinters::Print_CU<T>(cu);
    result = OrientationTransformation::cu_check(cu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    Quaternion<K> qu = OrientationTransformation::eu2qu<T, Quaternion<K>>(eu);
    OrientationPrinters::Print_QU<Quaternion<K>>(qu);
    result = OrientationTransformation::qu_check(qu);
    if(result.result <= 0)
    {
      std::cout << result.msg << std::endl;
    }

    T om = OrientationTransformation::eu2om<T, T>(eu);
    OrientationPrinters::Print_OM<T>(om);
    result = OrientationTransformation::om_check(om);
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
    //  using QVectorType = std::vector<K>;
    {
      K eu[3] = {static_cast<K>(0.3926990816987242L), static_cast<K>(0.0L), static_cast<K>(0.0L)};
      OrientationPrinters::Print_EU<K*>(eu);
      EU_2_XXX<OrientType, K>(eu);
      EU_2_XXX<VectorType, K>(eu);
      //   EU_2_XXX<QVectorType, K>(eu);
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
    for(size_t i = 0; i < 9; i++)
    {
      om[i] = in[i];
    }

    // Convert to Euler
    T res = OrientationTransformation::om2eu<T, T>(om);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Rodriques
    res = OrientationTransformation::om2ro<T, T>(om);
    OrientationPrinters::Print_RO<T>(res);

    // Convert to Quaternion
    Quaternion<K> qres = OrientationTransformation::om2qu<T, Quaternion<K>>(om);
    OrientationPrinters::Print_QU<Quaternion<K>>(qres, Quaternion<K>::Order::ScalarVector);

    // Convert to Axis Angle
    res = OrientationTransformation::om2ax<T, T>(om);
    OrientationPrinters::Print_AX<T>(res);

    // Convert to Homochoric
    res = OrientationTransformation::om2ho<T, T>(om);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::om2cu<T, T>(om);
    OrientationPrinters::Print_CU<T>(res);

    // Convert to Stereographic
    res = OrientationTransformation::om2st<T, T>(om);
    OrientationPrinters::Print_ST<T>(res);
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
    float om[9] = {0.0000f, 1.0000f, 0.0000f, -1.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 1.0000f};
    OrientationPrinters::Print_OM<float*>(om);
    OM_2_XXX<OrientationF>(om);
    OM_2_XXX<FloatVectorType>(om);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void RO_2_XXX(K* in)
  {
    T ro(4);
    for(size_t i = 0; i < 4; i++)
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
    Quaternion<K> qres = OrientationTransformation::ro2qu<T, Quaternion<K>>(ro);
    OrientationPrinters::Print_QU<Quaternion<K>>(qres);

    // Convert to Homochoric
    res = OrientationTransformation::ro2ho<T, T>(ro);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to CuboChoric
    res = OrientationTransformation::ro2cu<T, T>(ro);
    OrientationPrinters::Print_CU<T>(res);

    // Convert to Stereographic
    res = OrientationTransformation::ro2st<T, T>(ro);
    OrientationPrinters::Print_ST<T>(res);

    // Convert to Stereographic and back
    OrientationPrinters::Print_RO<T>(ro);
    OrientationPrinters::Print_ST<T>(res);
    res = OrientationTransformation::st2ro<T, T>(res);
    OrientationPrinters::Print_RO<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ro2_XXX()
  {
    std::cout << "Test_ro2_XXX  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
    float ro[4] = {0.0f, 0.0f, -1.0f, 1.0f};
    OrientationPrinters::Print_RO<float*>(ro);
    RO_2_XXX<OrientationF>(ro);
    RO_2_XXX<FloatVectorType>(ro);
  }

  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void ST_2_XXX(K* in)
  {
    T st(3);
    for(size_t i = 0; i < 3; i++)
    {
      st[i] = in[i];
    }

    T res(9); // Just size to 9 as we are going to reuse the variable

    // Convert to Euler
    res = OrientationTransformation::st2eu<T, T>(st);
    OrientationPrinters::Print_EU<T>(res);

    // Convert to Orientation Matrix
    res = OrientationTransformation::st2om<T, T>(st);
    OrientationPrinters::Print_OM<T>(res);

    // Convert to Axis Angle
    res = OrientationTransformation::st2ax<T, T>(st);
    OrientationPrinters::Print_AX<T>(res);

    // Convert to Quaternion
    Quaternion<K> qres = OrientationTransformation::st2qu<T, Quaternion<K>>(st);
    OrientationPrinters::Print_QU<Quaternion<K>>(qres);

    // Convert to Homochoric
    res = OrientationTransformation::st2ho<T, T>(st);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to CuboChoric
    res = OrientationTransformation::st2cu<T, T>(st);
    OrientationPrinters::Print_CU<T>(res);

    // Convert to Rodrigues
    res = OrientationTransformation::st2ro<T, T>(st);
    OrientationPrinters::Print_RO<T>(res);
  }

  // -----------------------------------------------------------------------------
  /*
Stereographic                    : -0.1989   0.0000   0.0000
Euler angles                     :   0.0000000   45.0000000    0.0000000
                                   /  1.0000   0.0000   0.0000 \
Orientation Matrix               : |  0.0000   0.7071   0.7071  |
                                   \  0.0000  -0.7071   0.7071/
Axis angle pair [n; angle]       :  -1.0000000    0.0000000    0.0000000 ;   45.0000000
Quaternion                       :  <-0.3826834    0.0000000    0.0000000 >  0.9238795
Homochoric representation        :  -0.3886796    0.0000000    0.0000000
Cubochoric representation        :  -0.3132742    0.0000000    0.0000000
Rodrigues vector                 :  -0.4142136    0.0000000    0.0000000

Rotation vector map              : -0.7854   0.0000   0.0000

*/
  void Test_st2_XXX()
  {
    std::cout << "Test_st2_XXX  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
    float st[3] = {0.0F, 0.0F, 0.0F};
    OrientationPrinters::Print_ST<float*>(st);
    ST_2_XXX<OrientationF>(st);
    ST_2_XXX<FloatVectorType>(st);
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
    Quaternion<K> qres = OrientationTransformation::ax2qu<T, Quaternion<K>>(ax);
    OrientationPrinters::Print_QU<Quaternion<K>>(qres);

    // Convert to homochoric
    res = OrientationTransformation::ax2ho<T, T>(ax);
    OrientationPrinters::Print_HO<T>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::ax2cu<T, T>(ax);
    OrientationPrinters::Print_CU<T>(res);

    // Convert to Stereographic
    res = OrientationTransformation::ax2st<T, T>(ax);
    OrientationPrinters::Print_ST<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ax2_XXX()
  {
    std::cout << "Test_ax2_XXX  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
    float ax[4] = {0.0f, 0.0f, -1.0f, k_PiOver2F};
    OrientationPrinters::Print_AX<float*>(ax);
    AX_2_XXX<OrientationF>(ax);
    AX_2_XXX<std::vector<float>>(ax);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T, typename K>
  void QU_2_XXX(K* in, typename Quaternion<K>::Order layout = Quaternion<K>::Order::VectorScalar)
  {
    T qu(4);
    qu[0] = in[0];
    qu[1] = in[1];
    qu[2] = in[2];
    qu[3] = in[3];

    using OrientationType = Orientation<K>;
    OrientationType res(9);

    // Convert to Orientation Matrix
    res = OrientationTransformation::qu2om<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_OM<OrientationType>(res);

    // Convert to Axis Angle
    res = OrientationTransformation::qu2eu<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_EU<OrientationType>(res);

    // Convert to Rodriques
    res = OrientationTransformation::qu2ro<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_RO<OrientationType>(res);

    // Convert to Quaternion
    res = OrientationTransformation::qu2ax<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_AX<OrientationType>(res);

    // Convert to Homochoric
    res = OrientationTransformation::qu2ho<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_HO<OrientationType>(res);

    // Convert to HomoChoric
    res = OrientationTransformation::qu2cu<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_CU<OrientationType>(res);

    // Convert to Stereographic
    res = OrientationTransformation::qu2st<T, OrientationType>(qu, layout);
    OrientationPrinters::Print_ST<OrientationType>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_qu2_XXX()
  {
    {
      std::cout << "Test_qu2_XXX  (SCALAR, <X, Y, Z>) ***************************************" << std::endl;
      float qu[4] = {EbsdLib::Constants::k_1OverRoot2F, 0.0f, 0.0f, -EbsdLib::Constants::k_1OverRoot2F};
      OrientationPrinters::Print_QU<QuatF>({qu[0], qu[1], qu[2], qu[3]}, QuatF::Order::ScalarVector);
      using QuatFType = Quaternion<float>;
      QU_2_XXX<QuatFType, float>(qu, QuatF::Order::ScalarVector);
      //  QU_2_XXX<std::vector<float> >(qu);
      //  QU_2_XXX<FloatQVectorType>(qu);
    }

    {
      std::cout << "Test_qu2_XXX  (<X, Y, Z>, SCALAR) ***************************************" << std::endl;
      float qu[4] = {0.0f, 0.0f, -EbsdLib::Constants::k_1OverRoot2F, EbsdLib::Constants::k_1OverRoot2F};
      OrientationPrinters::Print_QU<QuatF>({qu[0], qu[1], qu[2], qu[3]});
      using QuatFType = Quaternion<float>;
      QU_2_XXX<QuatFType, float>(qu);
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
    Quaternion<K> qres = OrientationTransformation::ho2qu<T, Quaternion<K>>(ho);
    OrientationPrinters::Print_QU<Quaternion<K>>(qres);

    // Convert to HomoChoric
    res = OrientationTransformation::ho2cu<T, T>(ho);
    OrientationPrinters::Print_CU<T>(res);

    // Convert to Stereographic
    res = OrientationTransformation::ho2st<T, T>(ho);
    OrientationPrinters::Print_ST<T>(res);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void Test_ho2_XXX()
  {
    std::cout << "Test_ho2_XXX  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
    float ho[3] = {0.000000f, 0.000000f, -0.7536693215f};
    OrientationPrinters::Print_HO<float*>(ho);
    HO_2_XXX<OrientationF>(ho);
    HO_2_XXX<std::vector<float>>(ho);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void TestInputs()
  {
    std::vector<size_t> cDims(1, 3);
    EbsdLib::FloatArrayType::Pointer data = EbsdLib::FloatArrayType::CreateArray(2, cDims, "Eulers", true);
    data->initializeWithZeros();
    float* fPtr = data->getPointer(0);
    fPtr[0] = 90.0f * EbsdLib::Constants::k_PiOver180F;
    fPtr[1] = 0.0f;
    fPtr[2] = 0.0f;
    fPtr[3] = 90.0f * EbsdLib::Constants::k_PiOver180F;
    fPtr[4] = 0.0f;
    fPtr[5] = 0.0f;

    //& Notation
    {
      OrientationF eu(&(fPtr[0]), 3); // Wrap the pointer with the &notation
      eu[0] = 45.0f * EbsdLib::Constants::k_PiOver180F;
      eu[1] = 90.0f * EbsdLib::Constants::k_PiOver180F;
      eu[2] = 135.0f * EbsdLib::Constants::k_PiOver180F;

      DREAM3D_REQUIRE_EQUAL(eu[0], fPtr[0]);
      DREAM3D_REQUIRE_EQUAL(eu[1], fPtr[1]);
      DREAM3D_REQUIRE_EQUAL(eu[2], fPtr[2]);
    }

    // Pointer Arithmetic (inputs)
    {
      OrientationF eu(fPtr + 3, 3);
      eu[0] = 135.0f * EbsdLib::Constants::k_PiOver180F;
      eu[1] = 45.0f * EbsdLib::Constants::k_PiOver180F;
      eu[2] = 90.0f * EbsdLib::Constants::k_PiOver180F;

      DREAM3D_REQUIRE_EQUAL(eu[0], fPtr[3]);
      DREAM3D_REQUIRE_EQUAL(eu[1], fPtr[4]);
      DREAM3D_REQUIRE_EQUAL(eu[2], fPtr[5]);
    }

    // Pointer Arithmetic, placing results directly into an array
    {
      OrientationF ax(0.0f, 0.0f, -1.0f, EbsdLib::Constants::k_PiOver2F);
      OrientationF eu(fPtr + 3, 3);
      eu = OrientationTransformation::ax2eu<OrientationF, OrientationF>(ax);

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
    using VecFType = std::vector<float>;
    VecFType qVec = {-0.5f, -0.5f, -0.5f, 0.5f};
    VecFType gVec(9, 0);

    gVec = OrientationTransformation::qu2om<VecFType, VecFType>(qVec);

    OrientationPrinters::Print_OM<VecFType>(gVec);

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

    std::array<float, 3> vq;
    std::array<float, 3> vg;
    float v[3] = {1.0f, 0.0f, 0.0f};

    // Mathematically correct, results unintuitive
    vq = q.multiplyByVector(v);

    std::cout << "vq: " << vq[0] << "," << vq[1] << "," << vq[2] << std::endl;

    EbsdMatrixMath::Multiply3x3with3x1(g, v, vg.data());
    std::cout << "vg: " << vg[0] << "," << vg[1] << "," << vg[2] << std::endl;
  }

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

    DREAM3D_REGISTER_TEST(Test_st2_XXX());

    DREAM3D_REGISTER_TEST(TestInputs());
  }

public:
  OrientationTest(const OrientationTest&) = delete;            // Copy Constructor Not Implemented
  OrientationTest(OrientationTest&&) = delete;                 // Move Constructor Not Implemented
  OrientationTest& operator=(const OrientationTest&) = delete; // Copy Assignment Not Implemented
  OrientationTest& operator=(OrientationTest&&) = delete;      // Move Assignment Not Implemented
};
