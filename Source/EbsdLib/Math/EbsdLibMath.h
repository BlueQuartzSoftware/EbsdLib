/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the followig contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once
/** @file EbsdLibMath.h
 * @brief This file performs the necessary including of <math.h> with certain
 * define constants (like M_PI) defined on all platforms.
 */

#if defined(_MSC_VER)
/* [i_a] MSVC again: see the comment from Microsoft's math.h below (MSVC2005).

     Other compilers may also lack M_PI / M_PI_2 which both are used in the
     OpenEXR (test) code.

     Microsoft says:

         Define _USE_MATH_DEFINES before including math.h to expose these macro
         definitions for common math constants.  These are placed under an #ifdef
         since these commonly-defined names are not part of the C/C standards.

     End of quote.

     The other defines have been added for completeness sake (they exist on
     BSD and Linux at least and other code may expect these).

     Microsoft doesn't define M_2PI ever, other compilers may lack some of these
     too, hence the sequence as it is: load math.h, then see what's lacking still.
  */
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 1
#endif

/*
  "It's a known, long-standing bug in the compiler system's headers.  For
  some reason the manufacturer, in its infinite wisdom, chose to #define
  macros min() and max() in violation of the upper-case convention and so
  break any legitimate functions with those names, including those in the
  standard C++ library."
  */
#ifndef NOMINMAX
#define NOMINMAX
#endif

#endif

#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <vector>

#include "EbsdLib/EbsdLib.h"

#ifndef M_E
#define M_E 2.7182818284590452354 /* e */
#endif

#ifndef M_LOG2E
#define M_LOG2E 1.4426950408889634074 /* log_2 e */
#endif

#ifndef M_LOG10E
#define M_LOG10E 0.43429448190325182765 /* log_10 e */
#endif

#ifndef M_LN2
#define M_LN2 0.69314718055994530942 /* log_e 2 */
#endif

#ifndef M_LN10
#define M_LN10 2.30258509299404568402 /* log_e 10 */
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923132169163975144 /* pi/2 */
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309615660845819875721 /* pi/4 */
#endif

#ifndef M_1_PI
#define M_1_PI 0.31830988618379067154 /* 1/pi */
#endif

#ifndef M_2_PI
#define M_2_PI 0.63661977236758134308 /* 2/pi */
#endif

#ifndef M_2_SQRTPI
#define M_2_SQRTPI 1.12837916709551257390 /* 2/sqrt(pi) */
#endif

#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880 /* sqrt(2) */
#endif

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.70710678118654752440 /* 1/sqrt(2) */
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

#ifndef M_2PI
#define M_2PI 6.283185307179586232 /* 2*pi  */
#endif

// This next bit is for MSVC compilers. We need to tell it that we already defined some math constants
#ifndef _MATH_DEFINES_DEFINED
#define _MATH_DEFINES_DEFINED
#endif

namespace EbsdLib::Constants
{
// In C++ 20, can be replaced with <numbers> header

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Pi = static_cast<T>(3.1415926535897932384);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_SqrtPi = static_cast<T>(1.7724538509055159);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_2OverSqrtPi = static_cast<T>(1.1283791670955126);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_HalfOfSqrtPi = static_cast<T>(0.88622692545275794);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_SqrtHalfPi = static_cast<T>(1.2533141373155001);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_2Pi = static_cast<T>(6.2831853071795862);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_1OverPi = static_cast<T>(0.31830988618379069);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_PiOver180 = static_cast<T>(0.017453292519943295);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_360OverPi = static_cast<T>(114.59155902616465);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_180OverPi = static_cast<T>(57.295779513082323);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_PiOver2 = static_cast<T>(1.5707963267948966);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_PiOver3 = static_cast<T>(1.0471975511965976);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_PiOver4 = static_cast<T>(0.78539816339744828);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_PiOver8 = static_cast<T>(0.39269908169872414);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_PiOver12 = static_cast<T>(0.26179938779914941);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Sqrt2 = static_cast<T>(1.4142135623730951);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Sqrt3 = static_cast<T>(1.7320508075688772);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_HalfSqrt2 = static_cast<T>(0.70710678118654757);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_1OverRoot2 = static_cast<T>(0.70710678118654746);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_1OverRoot3 = static_cast<T>(0.57735026918962584);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Root3Over2 = static_cast<T>(0.8660254037844386);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_DegToRad = static_cast<T>(0.017453292519943295);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_RadToDeg = static_cast<T>(57.295779513082323);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_1Point3 = static_cast<T>(1.3333333333333333);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_1Over3 = static_cast<T>(0.33333333333333331);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_ACosNeg1 = k_Pi<T>;

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_ACos1 = static_cast<T>(0.0);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Tan_OneEigthPi = static_cast<T>(0.41421356237309503);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Cos_OneEigthPi = static_cast<T>(0.92387953251128674);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Cos_ThreeEightPi = static_cast<T>(0.38268343236508984);

template <class T, class = std::enable_if_t<std::is_floating_point_v<T>>>
inline constexpr T k_Sin_ThreeEightPi = static_cast<T>(0.92387953251128674);

inline constexpr float k_PiF = k_Pi<float>;
inline constexpr double k_PiD = k_Pi<double>;

inline constexpr float k_SqrtPiF = k_SqrtPi<float>;
inline constexpr double k_SqrtPiD = k_SqrtPi<double>;

inline constexpr float k_2OverSqrtPiF = k_2OverSqrtPi<float>;
inline constexpr double k_2OverSqrtPiD = k_2OverSqrtPi<double>;

inline constexpr float k_HalfOfSqrtPiF = k_HalfOfSqrtPi<float>;
inline constexpr double k_HalfOfSqrtPiD = k_HalfOfSqrtPi<double>;

inline constexpr float k_SqrtHalfPiF = k_SqrtHalfPi<float>;
inline constexpr double k_SqrtHalfPiD = k_SqrtHalfPi<double>;

inline constexpr float k_2PiF = k_2Pi<float>;
inline constexpr double k_2PiD = k_2Pi<double>;

inline constexpr float k_1OverPiF = k_1OverPi<float>;
inline constexpr double k_1OverPiD = k_1OverPi<double>;

inline constexpr float k_PiOver180F = k_PiOver180<float>;
inline constexpr double k_PiOver180D = k_PiOver180<double>;

inline constexpr float k_360OverPiF = k_360OverPi<float>;
inline constexpr double k_360OverPiD = k_360OverPi<double>;

inline constexpr float k_180OverPiF = k_180OverPi<float>;
inline constexpr double k_180OverPiD = k_180OverPi<double>;

inline constexpr float k_PiOver2F = k_PiOver2<float>;
inline constexpr double k_PiOver2D = k_PiOver2<double>;

inline constexpr float k_PiOver3F = k_PiOver3<float>;
inline constexpr double k_PiOver3D = k_PiOver3<double>;

inline constexpr float k_PiOver4F = k_PiOver4<float>;
inline constexpr double k_PiOver4D = k_PiOver4<double>;

inline constexpr float k_PiOver8F = k_PiOver8<float>;
inline constexpr double k_PiOver8D = k_PiOver8<double>;

inline constexpr float k_PiOver12F = k_PiOver12<float>;
inline constexpr double k_PiOver12D = k_PiOver12<double>;

inline constexpr float k_Sqrt2F = k_Sqrt2<float>;
inline constexpr double k_Sqrt2D = k_Sqrt2<double>;

inline constexpr float k_Sqrt3F = k_Sqrt3<float>;
inline constexpr double k_Sqrt3D = k_Sqrt3<double>;

inline constexpr float k_HalfSqrt2F = k_HalfSqrt2<float>;
inline constexpr double k_HalfSqrt2D = k_HalfSqrt2<double>;

inline constexpr float k_1OverRoot2F = k_1OverRoot2<float>;
inline constexpr double k_1OverRoot2D = k_1OverRoot2<double>;

inline constexpr float k_1OverRoot3F = k_1OverRoot3<float>;
inline constexpr double k_1OverRoot3D = k_1OverRoot3<double>;

inline constexpr float k_Root3Over2F = k_Root3Over2<float>;
inline constexpr double k_Root3Over2D = k_Root3Over2<double>;

inline constexpr float k_DegToRadF = k_DegToRad<float>;
inline constexpr double k_DegToRadD = k_DegToRad<double>;

inline constexpr float k_RadToDegF = k_RadToDeg<float>;
inline constexpr double k_RadToDegD = k_RadToDeg<double>;

inline constexpr float k_1Point3F = k_1Point3<float>;
inline constexpr double k_1Point3D = k_1Point3<double>;

inline constexpr float k_1Over3F = k_1Over3<float>;
inline constexpr double k_1Over3D = k_1Over3<double>;

inline constexpr float k_ACosNeg1F = k_ACosNeg1<float>;
inline constexpr double k_ACosNeg1D = k_ACosNeg1<double>;

inline constexpr float k_ACos1F = k_ACos1<float>;
inline constexpr double k_ACos1D = k_ACos1<double>;

inline constexpr float k_Tan_OneEigthPiF = k_Tan_OneEigthPi<float>;
inline constexpr double k_Tan_OneEigthPiD = k_Tan_OneEigthPi<double>;

inline constexpr float k_Cos_OneEigthPiF = k_Cos_OneEigthPi<float>;
inline constexpr double k_Cos_OneEigthPiD = k_Cos_OneEigthPi<double>;

inline constexpr float k_Cos_ThreeEightPiF = k_Cos_ThreeEightPi<float>;
inline constexpr double k_Cos_ThreeEightPiD = k_Cos_ThreeEightPi<double>;

inline constexpr float k_Sin_ThreeEightPiF = k_Sin_ThreeEightPi<float>;
inline constexpr double k_Sin_ThreeEightPiD = k_Sin_ThreeEightPi<double>;

} // namespace EbsdLib::Constants

namespace EbsdLibMath
{
EbsdLib_EXPORT float Gamma(float);

template <typename T>
void bound(T& val, T min, T max)
{
  if(val < min)
  {
    val = min;
  }
  else if(val > max)
  {
    val = max;
  }
}

EbsdLib_EXPORT float erf(float);
EbsdLib_EXPORT float erfc(float);
EbsdLib_EXPORT float gammastirf(float);
EbsdLib_EXPORT float LnGamma(float, float&);
EbsdLib_EXPORT float incompletebeta(float, float, float);
EbsdLib_EXPORT float incompletebetafe(float, float, float, float, float);
EbsdLib_EXPORT float incompletebetafe2(float, float, float, float, float);
EbsdLib_EXPORT float incompletebetaps(float, float, float, float);

/**
 * @brief generates a linearly space array between 2 numbers (inclusive, assumes first number <= second number) [as matlabs linspace]
 * @param first number
 * @param second number
 * @param length of return array
 * @return
 */
EbsdLib_EXPORT std::vector<double> linspace(double first, double second, int length);

/**
 * @brief closeEnough
 * @param a
 * @param b
 * @param epsilon
 * @return
 */
template <typename K>
bool closeEnough(const K& a, const K& b, const K& epsilon = std::numeric_limits<K>::epsilon())
{
  return (epsilon > fabs(a - b));
}

/**
 * @brief transfer_sign
 * @param a
 * @param b
 * @return
 */
template <typename K>
K transfer_sign(K a, K b)
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
}; // namespace EbsdLibMath
