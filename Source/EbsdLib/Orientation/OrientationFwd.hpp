// OrientationFwd.h
#pragma once

#include <algorithm>
#include <array>
#include <string>
/**
 * The Orientation codes are written in such a way that the value of -1 indicates
 * an Active Rotation and +1 indicates a passive rotation.
 *
 * DO NOT UNDER ANY CIRCUMSTANCE CHANGE THESE VARIABLES. THERE WILL BE BAD
 * CONSEQUENCES IF THESE ARE CHANGED. EVERY PIECE OF CODE THAT RELIES ON THESE
 * FUNCTIONS WILL BREAK. IN ADDITION, THE QUATERNION ARITHMETIC WILL NO LONGER
 * BE CONSISTENT WITH ROTATION ARITHMETIC.
 *
 * YOU HAVE BEEN WARNED.
 *
 * Adam Morawiec's book uses Passive rotations.
 **/
#ifndef DREAM3D_PASSIVE_ROTATION
#define DREAM3D_PASSIVE_ROTATION 1
// #define DREAM3D_ACTIVE_ROTATION               -1.0
#endif

namespace ebsdlib
{
namespace orientations
{
#if DREAM3D_PASSIVE_ROTATION
constexpr float epsijk = 1.0f;
constexpr double epsijkd = 1.0;
#elif DREAM3D_ACTIVE_ROTATION
static const float epsijk = -1.0f;
static const double epsijkd = -1.0;
#endif

enum class Type : uint8_t
{
  Euler = 0,
  OrientationMatrix,
  Quaternion,
  AxisAngle,
  Rodrigues,
  Homochoric,
  Cubochoric,
  Stereographic,
  Unknown
};
} // namespace orientations

struct ResultType
{
  int result;
  std::string msg;
};

template <typename T>
class Euler;

template <typename T>
class OrientationMatrix;

template <typename T>
class AxisAngle;

template <typename T>
class Rodrigues;

template <typename T>
class Quaternion;

template <typename T>
class Homochoric;

template <typename T>
class Cubochoric;

template <typename T>
class Stereographic;

template <typename T, std::size_t N>
class OrientationBase
{
public:
  using value_type = T;
  static constexpr std::size_t num_elements = N;
  using size_type = size_t;
  using reference = T&;

  OrientationBase() = default;

  explicit OrientationBase(const std::array<T, N>& orientation)
  : m_Array(orientation)
  {
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
    return num_elements;
  }

  const T* data() const
  {
    return m_Array.data();
  }

  std::array<T, N> underlying() const
  {
    return m_Array;
  }

  bool isWithinTolerance(const OrientationBase& rhs, T tolerance) const
  {
    std::array<T, N> result;
    std::transform(m_Array.begin(), m_Array.end(), rhs.m_Array.begin(), result.begin(), std::minus<T>());

    auto position = std::find_if(result.begin(), result.end(), [tolerance](T num) { return std::abs(num) > tolerance; });
    return position == result.end();
  }

  void copyTo(T* destination) const
  {
    std::copy(m_Array.begin(), m_Array.end(), destination);
  }

protected:
  std::array<T, N> m_Array;
};

} // namespace ebsdlib
