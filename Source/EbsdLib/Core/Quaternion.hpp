#pragma once

#include <array>
#include <cmath>
#include <exception>

#include "EbsdLib/Core/Orientation.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/EbsdMatrixMath.h"

template <typename T>
class Quaternion
{
public:
  using EnumType = unsigned int;

  static_assert(std::is_floating_point<T>::value, "T must be a floating point type"); // disallow integers
  static_assert(std::numeric_limits<T>::has_infinity, "T must have infinity");        // must have ieee infinity

  enum class Order : EnumType
  {
    ScalarVector = 0,
    VectorScalar = 1
  };

  using self = Quaternion<T>;
  using size_type = size_t;
  using value_type = T;
  using reference = value_type&;
  using const_reference = const value_type&;
  using pointer = value_type*;

  Quaternion() = default;
  ~Quaternion() = default;

  // Quaternion(const Quaternion&) = default;
  Quaternion(Quaternion&&) noexcept = default;
  //  Quaternion& operator=(const Quaternion&) = default;
  //  Quaternion& operator=(Quaternion&&) noexcept = default;
  //
  Quaternion(const Quaternion<double>& q)
  {
    m_X = static_cast<T>(q.x());
    m_Y = static_cast<T>(q.y());
    m_Z = static_cast<T>(q.z());
    m_W = static_cast<T>(q.w());
  }
  Quaternion(const Quaternion<float>& q)
  {
    m_X = static_cast<T>(q.x());
    m_Y = static_cast<T>(q.y());
    m_Z = static_cast<T>(q.z());
    m_W = static_cast<T>(q.w());
  }

  template <typename FromType>
  Quaternion(const Quaternion<FromType>& in, const Order& o)
  {
    if(o == Order::VectorScalar)
    {
      m_X = static_cast<T>(in[0]);
      m_Y = static_cast<T>(in[1]);
      m_Z = static_cast<T>(in[2]);
      m_W = static_cast<T>(in[3]);
    }
    else
    {
      m_W = static_cast<T>(in[0]);
      m_X = static_cast<T>(in[1]);
      m_Y = static_cast<T>(in[2]);
      m_Z = static_cast<T>(in[3]);
    }
  }

  Quaternion(size_type size)
  {
    if(size != 4)
    {
      throw std::runtime_error("Quaternion Contructors needs argument of '4' for size.");
    }
  }

  Quaternion(T x, T y, T z, T w)
  {
    m_X = x;
    m_Y = y;
    m_Z = z;
    m_W = w;
  }

  Quaternion(T* q, Order o = Order::VectorScalar)
  {
    if(o == Order::VectorScalar)
    {
      m_X = q[0];
      m_Y = q[1];
      m_Z = q[2];
      m_W = q[3];
    }
    else
    {
      m_W = q[0];
      m_X = q[1];
      m_Y = q[2];
      m_Z = q[3];
    }
  }

  reference x()
  {
    return m_X;
  }

  const_reference x() const
  {
    return m_X;
  }

  reference y()
  {
    return m_Y;
  }

  const_reference y() const
  {
    return m_Y;
  }

  reference z()
  {
    return m_Z;
  }

  const_reference z() const
  {
    return m_Z;
  }

  reference w()
  {
    return m_W;
  }

  const_reference w() const
  {
    return m_W;
  }

  /**
   * @brief operator = The data from `rhs` will be copied into this instance.
   * @param rhs
   */
  Quaternion& operator=(const Quaternion<float>& rhs) // Copy Assignment
  {
    m_X = static_cast<T>(rhs.m_X);
    m_Y = static_cast<T>(rhs.m_Y);
    m_Z = static_cast<T>(rhs.m_Z);
    m_W = static_cast<T>(rhs.m_W);
    return *this;
  }

  Quaternion& operator=(const Quaternion<double>& rhs) // Copy Assignment
  {
    m_X = static_cast<T>(rhs.m_X);
    m_Y = static_cast<T>(rhs.m_Y);
    m_Z = static_cast<T>(rhs.m_Z);
    m_W = static_cast<T>(rhs.m_W);
    return *this;
  }

  /**
   * @brief operator = The data from `rhs` will be copied into this instance.
   * @param rhs
   */
  Quaternion& operator=(Quaternion<float>&& rhs) noexcept // Move Assignment
  {
    m_X = static_cast<T>(rhs.m_X);
    m_Y = static_cast<T>(rhs.m_Y);
    m_Z = static_cast<T>(rhs.m_Z);
    m_W = static_cast<T>(rhs.m_W);
    return *this;
  }
  /**
   * @brief operator = The data from `rhs` will be copied into this instance.
   * @param rhs
   */
  Quaternion& operator=(Quaternion<double>&& rhs) noexcept // Move Assignment
  {
    m_X = static_cast<T>(rhs.m_X);
    m_Y = static_cast<T>(rhs.m_Y);
    m_Z = static_cast<T>(rhs.m_Z);
    m_W = static_cast<T>(rhs.m_W);
    return *this;
  }

  /**
   * @brief copyInto
   * @param target
   * @param order
   */
  void copyInto(T* const target, const Order order)
  {
    if(order == Order::VectorScalar)
    {
      target[0] = m_X;
      target[1] = m_Y;
      target[2] = m_Z;
      target[3] = m_W;
    }
    else
    {
      target[0] = m_W;
      target[1] = m_X;
      target[2] = m_Y;
      target[3] = m_Z;
    }
  }

  /**
   * @brief Assumes XYZW data layout
   * @param index
   * @return
   */
  inline reference operator[](size_type index)
  {
    if(index > 3)
    {
      throw std::out_of_range("Index is out of range for the Quaternions.");
    }
    if(index == 0)
    {
      return m_X;
    }
    if(index == 1)
    {
      return m_Y;
    }
    if(index == 2)
    {
      return m_Z;
    }
    return m_W;
  }

  /**
   * @brief Assumes XYZW data layout
   * @param index
   * @return
   */
  inline const T& operator[](size_type index) const
  {
    if(index > 3)
    {
      throw std::out_of_range("Index is out of range for the Quaternions.");
    }
    if(index == 0)
    {
      return m_X;
    }
    if(index == 1)
    {
      return m_Y;
    }
    if(index == 2)
    {
      return m_Z;
    }
    return m_W;
  }

  /**
   * @brief Identity Sets the quaternion q to the identity quaternion (<0,0,0>,1)
   * @param q
   */
  void identity()
  {
    m_X = 0.0;
    m_Y = 0.0;
    m_Z = 0.0;
    m_W = 1.0;
  }

  /**
   * @brief ElementWiseAbs inline assigns the absolute value of each element to itself
   * @param q
   */
  void elementWiseAbs()
  {
    m_X = std::fabs(m_X);
    m_Y = std::fabs(m_Y);
    m_Z = std::fabs(m_Z);
    m_W = std::fabs(m_W);
  }

  /**
   * @brief ScalarMultiply Multiplies each element in the quaternion by the argument v
   * @param v
   */
  void scalarMultiply(T v)
  {
    m_X = m_X * v;
    m_Y = m_Y * v;
    m_Z = m_Z * v;
    m_W = m_W * v;
  }

  /**
   * @brief ScalarDivide Divides each element in the quaternion by the argument v
   * @param v
   */
  void scalarDivide(T v)
  {
    m_X = m_X / v;
    m_Y = m_Y / v;
    m_Z = m_Z / v;
    m_W = m_W / v;
  }

  /**
   * @brief ScalarAdd Adds value to each element of the vector and scalar part of the Quaternion
   * @param v Input Quat to add elements
   */
  void scalarAdd(T v)
  {
    m_X = m_X + v;
    m_Y = m_Y + v;
    m_Z = m_Z + v;
    m_W = m_W + v;
  }

  /**
   * @brief ElementWiseAssign Assigns each element the quaternion
   * @param v Input Quat to add elements
   */
  void elementWiseAssign(T v)
  {
    m_X = v;
    m_Y = v;
    m_Z = v;
    m_W = v;
  }

  /**
   * @brief Negate  −q = (−a, −v) In Place operation
   */
  void negate()
  {
    m_X = m_X * -1.0;
    m_Y = m_Y * -1.0;
    m_Z = m_Z * -1.0;
    m_W = m_W * -1.0;
  }

  /**
   * @brief Add   q1 + q2 = (w1+w2, v1+v2)
   * @param rhs
   * @return out
   */
  self operator+(const Quaternion& rhs) const
  {
    self out;
    out.x() = rhs.x() + m_X;
    out.y() = rhs.y() + m_Y;
    out.z() = rhs.z() + m_Z;
    out.w() = rhs.w() + m_W;
    return out;
  }

  /**
   * @brief Add   q1 - q2 = (w1-w2, v1-v2)
   * @param rhs
   * @return out
   */
  self operator-(const Quaternion& rhs) const
  {
    self out;
    out.x() = m_X - rhs.x();
    out.y() = m_Y - rhs.y();
    out.z() = m_Z - rhs.z();
    out.w() = m_W - rhs.w();
    return out;
  }

  /**
   * @brief Multiply current quaternion by another quaternion returning a third quaternion according to quaternion
   * multiplication. Note that Quaternioin multiplication is NOT cummunitive i.e., A * B != B * A
   * @param rhs Input Quaternion
   * @param out Result
   */
  self operator*(const Quaternion& rhs) const
  {
    self out;
    out.x() = rhs.x() * m_W + rhs.w() * m_X + rhs.z() * m_Y - rhs.y() * m_Z;
    out.y() = rhs.y() * m_W + rhs.w() * m_Y + rhs.x() * m_Z - rhs.z() * m_X;
    out.z() = rhs.z() * m_W + rhs.w() * m_Z + rhs.y() * m_X - rhs.x() * m_Y;
    /* Verified */
    out.w() = rhs.w() * m_W - rhs.x() * m_X - rhs.y() * m_Y - rhs.z() * m_Z;
    return out;
  }

  /**
   * @brief Multiply current quaternion by another quaternion returning a third quaternion according to quaternion
   * multiplication. Note that Quaternioin multiplication is NOT cummunitive i.e., A * B != B * A
   * @param rhs Input Quaternion
   * @param out Result
   */
  self operator*(Quaternion& rhs) const
  {
    self out;
    out.x() = rhs.x() * m_W + rhs.w() * m_X + rhs.z() * m_Y - rhs.y() * m_Z;
    out.y() = rhs.y() * m_W + rhs.w() * m_Y + rhs.x() * m_Z - rhs.z() * m_X;
    out.z() = rhs.z() * m_W + rhs.w() * m_Z + rhs.y() * m_X - rhs.x() * m_Y;
    /* Verified */
    out.w() = rhs.w() * m_W - rhs.x() * m_X - rhs.y() * m_Y - rhs.z() * m_Z;
    return out;
  }
  /**
   * @brief Conjugate Converts quaternion q into its conjugate
   * @return new quaternioin that is the conjugate of the current quaternion
   */
  self conjugate() const
  {
    self out(-m_X, -m_Y, -m_Z, m_W);
    return out;
  }

  /**
   * @brief Norm Computes and returns the "norm" of the quaternion (x^2 + y^2 + z^2 + w^2)
   * @return
   */
  T norm()
  {

    return m_X * m_X + m_Y * m_Y + m_Z * m_Z + m_W * m_W;
  }

  /**
   * @brief Length Computes are returns the "length" of the quaternion which is the square root of the norm. SQRT (x^2 + y^2 + z^2 + w^2)
   * @return
   */
  T length() const
  {
    return std::sqrt(m_X * m_X + m_Y * m_Y + m_Z * m_Z + m_W * m_W);
  }

  /**
   * @brief UnitQuaternion (Normalize) Converts the quaternion into its normalized values (x/L, y/L, z/L, w/L) where "L"
   * is the "length" of the quaternion
   * @return qr
   */
  self unitQuaternion() const
  {
    self out;
    value_type l = length();
    out.x() = static_cast<T>(m_X / l);
    out.y() = static_cast<T>(m_Y / l);
    out.z() = static_cast<T>(m_Z / l);
    out.w() = static_cast<T>(m_W / l);
    return out;
  }

  /**
   * @brief GetMisorientationVector Converts the quaternion into a misorientation vector in the reference frame of the quaternion
   * @return misoVec
   */
  std::array<value_type, 3> getMisorientationVector() const
  {
    std::array<value_type, 3> misoVec;

    value_type qw = m_W;
    EbsdLibMath::bound(qw, static_cast<value_type>(-1.0), static_cast<value_type>(1.0));
    value_type constVal = 0.0;
    if(qw == 1.0 || qw == -1.0)
    {
      constVal = 0.0;
    }
    else
    {
      constVal = 2 * std::acos(qw) / (std::sqrt(1.0 - (qw * qw)));
    }

    misoVec[0] = float(m_X * constVal);
    misoVec[1] = float(m_Y * constVal);
    misoVec[2] = float(m_Z * constVal);
    return misoVec;
  }

  /**
   * @brief MultiplyQuatVec Rotates a 3d vector 'v' by the quaternion 'q'
   * @param q Input Quaternion
   * @param v Input Vector
   * @param out Output Vector
   * SIMPLView uses
   * PASSIVE rotations by default.
   */
  std::array<value_type, 3> multiplyByVector(T* v) const
  {
    value_type qx2 = m_X * m_X;
    value_type qy2 = m_Y * m_Y;
    value_type qz2 = m_Z * m_Z;
    value_type qw2 = m_W * m_W;

    value_type qxy = m_X * m_Y;
    value_type qyz = m_Y * m_Z;
    value_type qzx = m_Z * m_X;

    value_type qxw = m_X * m_W;
    value_type qyw = m_Y * m_W;
    value_type qzw = m_Z * m_W;

    std::array<value_type, 3> out;

    out[0] = v[0] * (qx2 - qy2 - qz2 + qw2) + 2 * (v[1] * (qxy + qzw) + v[2] * (qzx - qyw));
    out[1] = v[1] * (qy2 - qx2 - qz2 + qw2) + 2 * (v[2] * (qyz + qxw) + v[0] * (qxy - qzw));
    out[2] = v[2] * (qz2 - qx2 - qy2 + qw2) + 2 * (v[0] * (qzx + qyw) + v[1] * (qyz - qxw));
    return out;
  }

  /**
   * @brief rotateVector ACTIVELY rotates input vector by this quaternion.
   * @param inputVector Input vector
   * @param passive argument: Passive = 1, Active = -1; Default is Passive
   * @return Output Vector
   */
  std::array<value_type, 3> rotateVector(T* inputVector, int p = 1) const
  {
    std::array<value_type, 3> rotatedVector = {0.0f, 0.0f, 0.0f};
    auto epsijk = static_cast<value_type>(p);

    std::array<value_type, 3> r = {epsijk * m_X, epsijk * m_Y, epsijk * m_Z};
    std::array<value_type, 3> temp = {0.0f, 0.0, 0.0};
    EbsdMatrixMath::CrossProduct(r.data(), inputVector, temp.data());

    temp[0] += m_W * inputVector[0];
    temp[1] += m_W * inputVector[1];
    temp[2] += m_W * inputVector[2];

    std::array<value_type, 3> temp2;
    EbsdMatrixMath::CrossProduct(r.data(), temp.data(), temp2.data());

    rotatedVector[0] = 2.0 * temp2[0] + inputVector[0];
    rotatedVector[1] = 2.0 * temp2[1] + inputVector[1];
    rotatedVector[2] = 2.0 * temp2[2] + inputVector[2];

    return rotatedVector;
  }

private:
  // Order m_Order = Order::VectorScalar;
  value_type m_X = {0.0};
  value_type m_Y = {0.0};
  value_type m_Z = {0.0};
  value_type m_W = {1.0};
};

using QuatType = Quaternion<double>;
using QuatF = Quaternion<float>;
