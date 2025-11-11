/* ============================================================================
 * Copyright (c) 2025 BlueQuartz Software, LLC
 * All rights reserved.
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
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Math/Matrix3X3.hpp"

#include <span>
#include <type_traits>

namespace EbsdLib
{

#define REP_SHIM_TO_QUAT_DEF(FROM)                                                                                                                                                                     \
  template <typename T, class InputType>                                                                                                                                                               \
  static Quaternion<T> qu(const InputType& e, typename Quaternion<T>::Order layout = Quaternion<T>::Order::VectorScalar)                                                                               \
  {                                                                                                                                                                                                    \
    Quaternion<T> qu = OrientationTransformation::FROM##2##qu<InputType, Quaternion<T>>(e, layout);                                                                                                    \
    return Orientation<T, QuatRep>(qu[0], qu[1], qu[2], qu[3]);                                                                                                                                        \
  }

#define REP_SHIM_FROM_QUAT_DEF(TO)                                                                                                                                                                     \
  template <typename T, class InputType, class OutputType>                                                                                                                                             \
  static OutputType TO(const InputType& qu, typename Quaternion<T>::Order layout = Quaternion<T>::Order::VectorScalar)                                                                                 \
  {                                                                                                                                                                                                    \
    Quaternion<T> quat(qu[0], qu[1], qu[2], qu[3]);                                                                                                                                                    \
    return OrientationTransformation::qu2##TO<Quaternion<T>, OutputType>(quat, layout);                                                                                                                \
  }

#define REP_SHIM_DEF(FROM, TO)                                                                                                                                                                         \
  template <typename T, class InputType, class OutputType>                                                                                                                                             \
  static OutputType TO(const InputType& orientation)                                                                                                                                                   \
  {                                                                                                                                                                                                    \
    return OrientationTransformation::FROM##2##TO<InputType, OutputType>(orientation);                                                                                                                 \
  }

#define REP_SHIM_NO_OP(NAME)                                                                                                                                                                           \
  template <typename T, class InputType, class OutputType>                                                                                                                                             \
  static OutputType NAME(const InputType& orientation)                                                                                                                                                 \
  {                                                                                                                                                                                                    \
    return orientation;                                                                                                                                                                                \
  }

#define ORIENTATION_REP_TO_METHOD_DEF(NAME, FUNC)                                                                                                                                                      \
  Orientation<T, NAME##Rep> to##NAME() const                                                                                                                                                           \
  {                                                                                                                                                                                                    \
    auto result = RepresentationType::template FUNC<T, StorageType, StorageType>(m_Array);                                                                                                             \
    return Orientation<T, NAME##Rep>(result);                                                                                                                                                          \
  }

struct QuaternionRep
{
  static constexpr size_t s_NumComponents = 4;

  REP_SHIM_FROM_QUAT_DEF(eu);
  REP_SHIM_FROM_QUAT_DEF(om);
  REP_SHIM_NO_OP(qu)
  REP_SHIM_FROM_QUAT_DEF(ax);
  REP_SHIM_FROM_QUAT_DEF(ro);
  REP_SHIM_FROM_QUAT_DEF(ho);
  REP_SHIM_FROM_QUAT_DEF(cu);
  REP_SHIM_FROM_QUAT_DEF(st);
}; // quaternion

struct EulerRep
{
  static constexpr size_t s_NumComponents = 3;
  REP_SHIM_NO_OP(eu);
  REP_SHIM_DEF(eu, om);
  REP_SHIM_DEF(eu, qu)
  REP_SHIM_DEF(eu, ax);
  REP_SHIM_DEF(eu, ro);
  REP_SHIM_DEF(eu, ho);
  REP_SHIM_DEF(eu, cu);
  REP_SHIM_DEF(eu, st);

}; // euler angles

struct OrientationMatrixRep
{
  static constexpr size_t s_NumComponents = 9;
  REP_SHIM_DEF(om, eu);
  REP_SHIM_NO_OP(om);
  REP_SHIM_DEF(om, qu)
  REP_SHIM_DEF(om, ax);
  REP_SHIM_DEF(om, ro);
  REP_SHIM_DEF(om, ho);
  REP_SHIM_DEF(om, cu);
  REP_SHIM_DEF(om, st);

}; // orientation matrix

struct AxisAngleRep
{
  static constexpr size_t s_NumComponents = 4;
  REP_SHIM_DEF(ax, eu);
  REP_SHIM_DEF(ax, om);
  REP_SHIM_DEF(ax, qu)
  REP_SHIM_NO_OP(ax);
  REP_SHIM_DEF(ax, ro);
  REP_SHIM_DEF(ax, ho);
  REP_SHIM_DEF(ax, cu);
  REP_SHIM_DEF(ax, st);
};
struct RodriguesRep
{
  static constexpr size_t s_NumComponents = 4;
  REP_SHIM_DEF(ro, eu);
  REP_SHIM_DEF(ro, om);
  REP_SHIM_DEF(ro, qu)
  REP_SHIM_DEF(ro, ax);
  REP_SHIM_NO_OP(ro);
  REP_SHIM_DEF(ro, ho);
  REP_SHIM_DEF(ro, cu);
  REP_SHIM_DEF(ro, st);
};
struct HomochoricRep
{
  static constexpr size_t s_NumComponents = 3;
  REP_SHIM_DEF(ho, eu);
  REP_SHIM_DEF(ho, om);
  REP_SHIM_DEF(ho, qu)
  REP_SHIM_DEF(ho, ax);
  REP_SHIM_DEF(ho, ro);
  REP_SHIM_NO_OP(ho);
  REP_SHIM_DEF(ho, cu);
  REP_SHIM_DEF(ho, st);
};
struct CubochoricRep
{
  static constexpr size_t s_NumComponents = 3;
  REP_SHIM_DEF(cu, eu);
  REP_SHIM_DEF(cu, om);
  REP_SHIM_DEF(cu, qu)
  REP_SHIM_DEF(cu, ax);
  REP_SHIM_DEF(cu, ro);
  REP_SHIM_DEF(cu, ho);
  REP_SHIM_NO_OP(cu);
  REP_SHIM_DEF(cu, st);
};
struct StereographicRep
{
  static constexpr size_t s_NumComponents = 3;
  REP_SHIM_DEF(st, eu);
  REP_SHIM_DEF(st, om);
  REP_SHIM_DEF(st, qu)
  REP_SHIM_DEF(st, ax);
  REP_SHIM_DEF(st, ro);
  REP_SHIM_DEF(st, ho);
  REP_SHIM_DEF(st, cu);
  REP_SHIM_NO_OP(st);
};

/**
 *
 * @tparam T The primitive type (float or double)
 * @tparam RepresentationType The 'Representation' struct
 */
template <typename T, class RepresentationType>
class Orientation
{

public:
  using StorageType = std::vector<T>;

  using size_type = size_t;
  using value_type = T;
  using reference = T&;

  using QuaternionType = Quaternion<T>;

  Orientation()
  {
    m_Array.resize(RepresentationType::s_NumComponents);
  }

  template <std::enable_if<std::is_same_v<RepresentationType, OrientationMatrixRep>>* = nullptr>
  Orientation(T r0c0, T r0c1, T r0c2, T r1c0, T r1c1, T r1c2, T r2c0, T r2c1, T r2c2)
  : m_Array({
        r0c0,
        r0c1,
        r0c2,
        r1c0,
        r1c1,
        r1c2,
        r2c0,
        r2c1,
        r2c2,
    })
  {
  }

  template <std::enable_if<std::is_same_v<RepresentationType, OrientationMatrixRep>>* = nullptr>
  Orientation(const Matrix3X3<T>& mat)
  : m_Array({mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8]})
  {
  }

  template <std::enable_if<std::is_same_v<RepresentationType, AxisAngleRep> || std::is_same_v<RepresentationType, RodriguesRep> || std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  Orientation(T i, T j, T k, T w)
  : m_Array({i, j, k, w})
  {
  }

  template <std::enable_if<std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  explicit Orientation(const Quaternion<double>& quat)
  : m_Array({quat.x(), quat.y(), quat.z(), quat.w()})
  {
  }

  template <std::enable_if<std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  explicit Orientation(const Quaternion<float>& quat)
  : m_Array({quat.x(), quat.y(), quat.z(), quat.w()})
  {
  }

  template <std::enable_if<std::is_same_v<RepresentationType, EulerRep> || std::is_same_v<RepresentationType, HomochoricRep> || std::is_same_v<RepresentationType, CubochoricRep> ||
                           std::is_same_v<RepresentationType, StereographicRep>>* = nullptr>
  Orientation(T x, T y, T z)
  : m_Array({x, y, z})
  {
  }

  template <std::enable_if<std::is_same_v<RepresentationType, EulerRep> || std::is_same_v<RepresentationType, HomochoricRep> || std::is_same_v<RepresentationType, CubochoricRep> ||
                           std::is_same_v<RepresentationType, StereographicRep>>* = nullptr>
  Orientation(const Matrix3X1<T>& mat)
  : m_Array({mat[0], mat[1], mat[2]})
  {
  }

  Orientation(const Orientation& rhs)
  : m_Array(rhs.m_Array)
  {
  }

  Orientation(Orientation&& rhs)
  : m_Array(std::move(rhs.m_Array))
  {
  }

  Orientation(size_t size)
  : m_Array()
  {
    if(RepresentationType::s_NumComponents != size)
    {
      throw std::runtime_error("Number of components of OrientationRep does not match 'size' argument");
    }
    m_Array.resize(RepresentationType::s_NumComponents);
  }

  Orientation(const StorageType& input)
  : m_Array(input)
  {
    if(RepresentationType::s_NumComponents != m_Array.size())
    {
      throw std::runtime_error("Number of components of OrientationRep does not match 'size' argument");
    }
  }

  Orientation(const T* ptr)
  {
    m_Array.resize(RepresentationType::s_NumComponents);
    m_Array.assign(ptr, ptr + RepresentationType::s_NumComponents);
  }

  Orientation& operator=(const Orientation& rhs) // Copy Assignment
  {
    m_Array = rhs.m_Array;
    return *this;
  }

  Orientation& operator=(Orientation&& rhs) // Move Assignment
  {
    m_Array = std::move(rhs.m_Array);
    return *this;
  }

  reference operator[](size_type index)
  {
    return m_Array[index];
  }

  const T& operator[](size_type index) const
  {
    return m_Array[index];
  }

  size_t size() const
  {
    return RepresentationType::s_NumComponents;
  }

  const T* data() const
  {
    return m_Array.data();
  }

  void copyTo(T* destination) const
  {
    std::copy(m_Array.begin(), m_Array.end(), destination);
  }

  Orientation<T, RepresentationType> operator-(const Orientation<T, RepresentationType>& rhs) const
  {
    StorageType result_vec(m_Array.size());
    std::transform(m_Array.begin(), m_Array.end(), rhs.m_Array.begin(), result_vec.begin(), std::minus<T>());
    return Orientation<T, RepresentationType>(result_vec);
  }

  bool isWithinTolerance(const Orientation<T, RepresentationType>& rhs, T tolerance) const
  {
    auto result = *this - rhs;
    auto it = std::find_if(result.m_Array.begin(), result.m_Array.end(), [tolerance](T num) { return std::abs(num) > tolerance; });
    return it == result.m_Array.end();
  }

  template <std::enable_if<std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  T w() const
  {
    return m_Array[3];
  }
  template <std::enable_if<std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  T x() const
  {
    return m_Array[0];
  }
  template <std::enable_if<std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  T y() const
  {
    return m_Array[1];
  }
  template <std::enable_if<std::is_same_v<RepresentationType, QuaternionRep>>* = nullptr>
  T z() const
  {
    return m_Array[2];
  }

  ORIENTATION_REP_TO_METHOD_DEF(Euler, eu)
  ORIENTATION_REP_TO_METHOD_DEF(OrientationMatrix, om)
  ORIENTATION_REP_TO_METHOD_DEF(Quaternion, qu)
  ORIENTATION_REP_TO_METHOD_DEF(AxisAngle, ax)
  ORIENTATION_REP_TO_METHOD_DEF(Rodrigues, ro)
  ORIENTATION_REP_TO_METHOD_DEF(Homochoric, ho)
  ORIENTATION_REP_TO_METHOD_DEF(Cubochoric, cu)
  ORIENTATION_REP_TO_METHOD_DEF(Stereographic, st)

  QuaternionType toQuat() const
  {
    Orientation<T, EbsdLib::QuaternionRep> quRep = toQuaternion();
    return QuaternionType{quRep[0], quRep[1], quRep[2], quRep[3]};
  }

  template <std::enable_if<std::is_same_v<RepresentationType, OrientationMatrixRep>>* = nullptr>
  Matrix3X3<T> toGMatrixObj() const
  {
    return {m_Array[0], m_Array[1], m_Array[2], m_Array[3], m_Array[4], m_Array[5], m_Array[6], m_Array[7], m_Array[8]};
  }

  template <std::enable_if<std::is_same_v<RepresentationType, OrientationMatrixRep>>* = nullptr>
  Eigen::Matrix<T, 3, 3, Eigen::RowMajor> toEigenGMatrix() const
  {
    Eigen::Matrix<T, 3, 3, Eigen::RowMajor> g1;
    g1(0, 0) = m_Array[0];
    g1(0, 1) = m_Array[1];
    g1(0, 2) = m_Array[2];
    g1(1, 0) = m_Array[3];
    g1(1, 1) = m_Array[4];
    g1(1, 2) = m_Array[5];
    g1(2, 0) = m_Array[6];
    g1(2, 1) = m_Array[7];
    g1(2, 2) = m_Array[8];
    return g1;
  }

private:
  StorageType m_Array;
};

using EulerDType = Orientation<double, EbsdLib::EulerRep>;
using OrientationMatrixDType = Orientation<double, OrientationMatrixRep>;
using QuaternionDType = Orientation<double, QuaternionRep>;
using AxisAngleDType = Orientation<double, AxisAngleRep>;
using RodriguesDType = Orientation<double, RodriguesRep>;
using HomochoricDType = Orientation<double, HomochoricRep>;
using CubochoricDType = Orientation<double, CubochoricRep>;
using StereographicDType = Orientation<double, StereographicRep>;

using EulerFType = Orientation<float, EulerRep>;
using OrientationMatrixFType = Orientation<float, OrientationMatrixRep>;
using QuaternionFType = Orientation<float, QuaternionRep>;
using AxisAngleFType = Orientation<float, AxisAngleRep>;
using RodriguesFType = Orientation<float, RodriguesRep>;
using HomochoricFType = Orientation<float, HomochoricRep>;
using CubochoricFType = Orientation<float, CubochoricRep>;
using StereographicFType = Orientation<float, StereographicRep>;

inline std::ostream& operator<<(std::ostream& os, const EulerDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "EU: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const OrientationMatrixDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "OM: /  " << obj[0] << ", " << obj[1] << ", " << obj[2] << "\\\n";
  os << std::setw(3) << std::setprecision(16) << "OM: |  " << obj[3] << ", " << obj[4] << ", " << obj[5] << "|\n";
  os << std::setw(3) << std::setprecision(16) << "OM: \\  " << obj[6] << ", " << obj[7] << ", " << obj[8] << "/\n";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const QuaternionDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "QU: <" << obj[0] << ", " << obj[1] << ", " << obj[2] << "> " << obj[3];
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const AxisAngleDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "AX: <" << obj[0] << ", " << obj[1] << ", " << obj[2] << "> " << obj[3];
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const RodriguesDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "RO: <" << obj[0] << ", " << obj[1] << ", " << obj[2] << "> " << obj[3];
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const HomochoricDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "HO: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const CubochoricDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "CU: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const StereographicDType& obj)
{
  os << std::setw(3) << std::setprecision(16) << "ST: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}

} // namespace EbsdLib
