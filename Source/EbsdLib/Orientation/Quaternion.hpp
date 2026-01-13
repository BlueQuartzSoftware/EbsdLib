// Quaternion.h
#pragma once

#include "EbsdLib/Math/ArrayHelpers.hpp"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"

#include <Eigen/Dense>

#include <array>

namespace ebsdlib
{
/**
 * @brief This class represents an orientation as a Quaternion. The order of the quaternion
 * is stored as "VECTOR-SCALAR"
 *
 * @param T Either double or float
 */
template <typename T = double>
class Quaternion : public OrientationBase<T, 4>
{

public:
  static_assert(std::is_floating_point_v<T>, "T must be a floating point type"); // disallow integers
  static_assert(std::numeric_limits<T>::has_infinity, "T must have infinity");   // must have ieee infinity

  using SuperType = OrientationBase<T, 4>;
  using SelfType = Quaternion<T>;
  using size_type = size_t;
  using value_type = T;

  using EnumType = uint32_t;

  Quaternion() = default;

  /**
   * @brief Constructs a Quaternion Representation which will **ALWAYS** have its layout as Vector-Scalar
   * @param x
   * @param y
   * @param z
   * @param w
   */
  Quaternion(T x, T y, T z, T w)
  : OrientationBase<T, 4>({x, y, z, w})
  {
  }

  /**
   * @brief Constructs a Quaternion Representation which will **ALWAYS** have its layout as Vector-Scalar
   * @param x
   * @param y
   * @param z
   * @param w
   */
  explicit Quaternion(const std::array<T, 4>& d)
  : OrientationBase<T, 4>(d)
  {
  }

  /**
   * @brief Constructs a Quaternion Representation which will **ALWAYS** have its layout as Vector-Scalar
   * @param quat Pointer to a Quaternion in Vector-Scalar order
   */
  explicit Quaternion(const T* quat)
  : OrientationBase<T, 4>({quat[0], quat[1], quat[2], quat[3]})
  {
  }

  /**
   * @brief Constructs a Quaternion Representation which will **ALWAYS** have its layout as Vector-Scalar. The incoming 4x1
   * Eigen matrix should have its Quaternion laid out in vector-scalar order
   * @param eigenQuat
   */
  explicit Quaternion(const Eigen::Matrix<T, 4, 1, 0>& eigenQuat)
  : OrientationBase<T, 4>({eigenQuat[0], eigenQuat[1], eigenQuat[2], eigenQuat[3]})
  {
  }

  const T& x() const
  {
    return (*this)[0];
  }
  T& x()
  {
    return (*this)[0];
  }

  const T& y() const
  {
    return (*this)[1];
  }
  T& y()
  {
    return (*this)[1];
  }

  const T& z() const
  {
    return (*this)[2];
  }
  T& z()
  {
    return (*this)[2];
  }

  const T& w() const
  {
    return (*this)[3];
  }
  T& w()
  {
    return (*this)[3];
  }

  /**
   * @brief Copies the quaternion into a given pointer
   * @param target
   * @param order
   */
  void copyInto(T* target) const
  {
    target[0] = x();
    target[1] = y();
    target[2] = z();
    target[3] = w();
  }

  /**
   * @brief Returns a Quat which is the identity Quaternion (<0,0,0>,1)
   */
  static SelfType identity()
  {
    return {0.0, 0.0, 0.0, 1.0};
  }

  /**
   * @brief ElementWiseAbs inline assigns the absolute value of each element to itself (in place)
   */
  SelfType& elementWiseAbs()
  {
    x() = std::fabs(x());
    y() = std::fabs(y());
    z() = std::fabs(z());
    w() = std::fabs(w());
    return *this;
  }

  /**
   * @brief ScalarMultiply Multiplies each element in the Quat by the argument v (in place)
   * @param v
   */
  SelfType& scalarMultiply(T v)
  {
    x() *= v;
    y() *= v;
    z() *= v;
    w() *= v;
    return *this;
  }

  /**
   * @brief ScalarDivide Divides each element in the Quat by the argument v (in place)
   * @param v
   */
  SelfType& scalarDivide(T v)
  {
    x() /= v;
    y() /= v;
    z() /= v;
    w() /= v;
    return *this;
  }

  /**
   * @brief ScalarAdd Adds value to each element of the vector and scalar part of the Quat (in place)
   * @param v Input Quat to add elements
   */
  SelfType& scalarAdd(T v)
  {
    x() += v;
    y() += v;
    z() += v;
    w() += v;
    return *this;
  }

  /**
   * @brief ElementWiseAssign Assigns each element the Quat (in place)
   * @param v Input Quat to add elements
   */
  SelfType& elementWiseAssign(T v)
  {
    x() = v;
    y() = v;
    z() = v;
    w() = v;
    return *this;
  }

  /**
   * @brief Negate  −q = (−a, −v) In Place operation (in place)
   */
  SelfType& negate()
  {
    x() = -x();
    y() = -y();
    z() = -z();
    w() = -w();
    return *this;
  }

  /**
   * @brief Returns a new Quaternion that is the negative of the current quaternion
   * @return
   */
  SelfType operator-() const
  {
    return {-x(), -y(), -z(), -w()};
  }

  /**
   * @brief Adds q1 + q2 = (w1+w2, v1+v2) and returns a new Quaternion
   * @param rhs
   * @return
   */
  SelfType operator+(const SelfType& rhs) const
  {
    SelfType out;
    out.x() = rhs.x() + x();
    out.y() = rhs.y() + y();
    out.z() = rhs.z() + z();
    out.w() = rhs.w() + w();
    return out;
  }

  /**
   * @brief Add   q1 - q2 = (w1-w2, v1-v2) and returns a new Quaternion
   * @param rhs
   * @return out
   */
  SelfType operator-(const SelfType& rhs) const
  {
    SelfType out;
    out.x() = x() - rhs.x();
    out.y() = y() - rhs.y();
    out.z() = z() - rhs.z();
    out.w() = w() - rhs.w();
    return out;
  }

  /**
   * @brief Multiply current Quat by another Quat returning a third Quat according to Quaternion
   * multiplication. Note that Quaternion multiplication is NOT communicative i.e., A * B != B * A
   * @param rhs Input Quat
   * @return
   */
  SelfType operator*(const SelfType& rhs) const
  {
    SelfType out;
    out.x() = rhs.x() * w() + rhs.w() * x() + rhs.z() * y() - rhs.y() * z();
    out.y() = rhs.y() * w() + rhs.w() * y() + rhs.x() * z() - rhs.z() * x();
    out.z() = rhs.z() * w() + rhs.w() * z() + rhs.y() * x() - rhs.x() * y();
    out.w() = rhs.w() * w() - rhs.x() * x() - rhs.y() * y() - rhs.z() * z();
    return out;
  }

  /**
   * @brief Add q1 + q2 = (w1+w2, v1+v2) (in place)
   * @param rhs
   * @return self
   */
  SelfType& operator+=(const SelfType& rhs)
  {
    x() += rhs.x();
    y() += rhs.y();
    z() += rhs.z();
    w() += rhs.w();
    return *this;
  }

  /**
   * @brief Add q1 - q2 = (w1-w2, v1-v2) (in place)
   * @param rhs
   * @return self
   */
  SelfType& operator-=(const SelfType& rhs)
  {
    x() -= rhs.x();
    y() -= rhs.y();
    z() -= rhs.z();
    w() -= rhs.w();
    return *this;
  }

  /**
   * @brief Multiply current Quat by another Quat (in place)
   * multiplication. Note that Quat multiplication is NOT communicative i.e., A * B != B * A
   * @param rhs Input Quat
   * @return self
   */
  SelfType& operator*=(const SelfType& rhs)
  {
    x() = rhs.x() * w() + rhs.w() * x() + rhs.z() * y() - rhs.y() * z();
    y() = rhs.y() * w() + rhs.w() * y() + rhs.x() * z() - rhs.z() * x();
    z() = rhs.z() * w() + rhs.w() * z() + rhs.y() * x() - rhs.x() * y();
    /* Verified */
    w() = rhs.w() * w() - rhs.x() * x() - rhs.y() * y() - rhs.z() * z();
    return *this;
  }

  /**
   * @brief Computes the dot product between the current Quat and an input Quat
   *
   * @note The value returned could possibly be just outside the theoretical range of -1 to 1 which if the
   * result is then used in the `acos()` function could give unexpected results
   * @param rhs Second Quat to use to compute the dot product
   * @return scalar dot product
   */
  T dotProduct(const SelfType& rhs) const
  {
    T dot = x() * rhs.x() + y() * rhs.y() + z() * rhs.z() + w() * rhs.w();
    return dot;
  }

  /**
   * @brief Conjugate Converts Quat q into its conjugate and returns a new Quaternion<> instance
   * @return new Quat that is the conjugate of the current Quat
   */
  SelfType conjugate() const
  {
    return {-x(), -y(), -z(), w()};
  }

  /**
   * @brief Norm Computes and returns the "norm" of the Quat (x^2 + y^2 + z^2 + w^2)
   * @return
   */
  T norm() const
  {
    return x() * x() + y() * y() + z() * z() + w() * w();
  }

  /**
   * @brief Length Computes are returns the "length" of the Quat which is the square root of the norm. SQRT (x^2 + y^2 + z^2 + w^2)
   * @return
   */
  T length() const
  {
    return std::sqrt(norm());
  }

  /**
   * @brief Computes the inverse of the Quat
   * @return
   */
  SelfType inverse() const
  {
    double normSq = this->norm(); // Use your existing Norm()
    if(normSq == 0.0)
    {
      throw std::runtime_error("Cannot invert a Quat with zero norm.");
    }
    SelfType conj = this->conjugate();
    return SelfType(conj.x() / normSq, conj.y() / normSq, conj.z() / normSq, conj.w() / normSq);
  }

  /**
   * @brief check if this is a unit Quat
   * @param tolerance
   * @return
   */
  bool isUnit(double tolerance = 1e-6) const
  {
    return std::abs(this->norm() - 1.0) < tolerance;
  }

  /**
   * @brief Normalize: Converts the Quat into its normalized values (x/L, y/L, z/L, w/L) where "L"
   * is the "length" of the Quat
   * @return qr
   */
  SelfType normalize() const
  {
    T len = length();
    return {x() / len, y() / len, z() / len, w() / len};
  }

  /**
   * @brief GetMisorientationVector Converts the Quat into a misorientation vector in the reference frame of the Quat
   * @return misoVec
   */
  Matrix3X1<T> getMisorientationVector() const
  {
    T qw = std::clamp(w(), static_cast<T>(-1.0), static_cast<T>(1.0));
    T constVal = 0.0;
    if(qw == 1.0 || qw == -1.0)
    {
      constVal = 0.0;
    }
    else
    {
      constVal = static_cast<T>(2 * std::acos(qw) / (std::sqrt(1.0 - (qw * qw))));
    }

    return {x() * constVal, y() * constVal, z() * constVal};
  }

  /**
   * @brief Rotates a 3d vector 'v' by the Quaternion
   * @param v Input Vector as a pointer.
   */
  Matrix3X1<T> multiplyByVector(const T* v) const
  {
    T qx2 = x() * x();
    T qy2 = y() * y();
    T qz2 = z() * z();
    T qw2 = w() * w();

    T qxy = x() * y();
    T qyz = y() * z();
    T qzx = z() * x();

    T qxw = x() * w();
    T qyw = y() * w();
    T qzw = z() * w();

    Matrix3X1<T> out;

    out[0] = v[0] * (qx2 - qy2 - qz2 + qw2) + 2 * (v[1] * (qxy + qzw) + v[2] * (qzx - qyw));
    out[1] = v[1] * (qy2 - qx2 - qz2 + qw2) + 2 * (v[2] * (qyz + qxw) + v[0] * (qxy - qzw));
    out[2] = v[2] * (qz2 - qx2 - qy2 + qw2) + 2 * (v[0] * (qzx + qyw) + v[1] * (qyz - qxw));
    return out;
  }

  /**
   * @brief rotateVector ACTIVELY rotates input vector by this Quat.
   * @param inputVector Input vector
   * @param p argument: Passive = 1, Active = -1; Default is Passive
   * @return Output Vector
   */
  Matrix3X1<T> rotateVector(const Matrix3X1<T>& inputVector, int32_t p = 1) const
  {
    Matrix3X1<T> rotatedVector = {0.0, 0.0, 0.0};
    T epsijk = static_cast<T>(p);

    Matrix3X1<T> r = {epsijk * x(), epsijk * y(), epsijk * z()};
    Matrix3X1<T> temp = r.cross(inputVector);

    temp[0] += w() * inputVector[0];
    temp[1] += w() * inputVector[1];
    temp[2] += w() * inputVector[2];

    Matrix3X1<T> temp2 = r.cross(temp);

    rotatedVector[0] = static_cast<T>(2.0) * temp2[0] + inputVector[0];
    rotatedVector[1] = static_cast<T>(2.0) * temp2[1] + inputVector[1];
    rotatedVector[2] = static_cast<T>(2.0) * temp2[2] + inputVector[2];

    return rotatedVector;
  }

  /**
   * @brief Returns the magnitude of the vector part of the quaternion.
   *
   * @return
   */
  T magnitude() const
  {
    return std::sqrt(x() * x() + y() * y() + z() * z());
  }

  /**
   * @brief Converts this Quaternion to a different primitive type. This can be useful
   * if you need to convert from a Quaternion<double> to Quaternion<float>
   * @tparam K
   * @return
   */
  template <typename K>
  Quaternion<K> to() const
  {
    return Quaternion<K>(static_cast<K>(x()), static_cast<K>(y()), static_cast<K>(z()), static_cast<K>(w()));
  }

  /* *****************************************************************************
   * THESE ARE THE CRYSTALLOGRAPHIC ORIENTATION CONVERSION METHODS
   * ****************************************************************************/
  /**
   * @brief Ensures this Quat represents an orientation that is located in the Northern Hemisphere.
   *
   * NOTE: This is done IN PLACE!!
   */
  void positiveOrientation()
  {
    if(w() < static_cast<T>(0.0))
    {
      x() = -x();
      y() = -y();
      z() = -z();
      w() = -w();
    }
  }

  /**
   * @brief Returns a new Quat that represents an orientation that is located in the Northern Hemisphere
   * @return Copy of Quat
   */
  SelfType getPositiveOrientation() const
  {
    if(w() < static_cast<T>(0.0))
    {
      return {-x(), -y(), -z(), -w()};
    }
    return {x(), y(), z(), w()};
  }

  ResultType isValid() const
  {
    ResultType res;
    res.result = 1;

    if(w() < 0.0)
    {
      res.msg = "Quaternion Error: quaternion must have positive scalar part";
      res.result = -1;
      return res;
    }

    value_type eps = std::numeric_limits<float>::epsilon();
    value_type r = length();
    if(fabs(r - 1.0) > eps)
    {
      res.msg = "Quaternion Error: quaternion must have unit norm";
      res.result = -2;
    }
    return res;
  }

  Euler<T> toEuler() const
  {
    using OutputType = Euler<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res;
    OutputValueType q12 = 0.0f;
    OutputValueType q03 = 0.0f;
    OutputValueType chi = 0.0f;
    OutputValueType Phi = 0.0f;
    OutputValueType phi1 = 0.0f;
    OutputValueType phi2 = 0.0f;

    SelfType qq(*this);

    q03 = qq.w() * qq.w() + qq.z() * qq.z();
    q12 = qq.x() * qq.x() + qq.y() * qq.y();
    chi = sqrt(q03 * q12);
    if(chi == 0.0)
    {
      if(q12 == 0.0)
      {
        if(ebsdlib::orientations::epsijk == 1.0)
        {
          Phi = 0.0;
          phi2 = 0.0; // arbitrarily due to degeneracy
          phi1 = static_cast<OutputValueType>(atan2(-2.0 * qq.w() * qq.z(), qq.w() * qq.w() - qq.z() * qq.z()));
        }
        else
        {
          Phi = 0.0;
          phi2 = 0.0; // arbitrarily due to degeneracy
          phi1 = static_cast<OutputValueType>(atan2(2.0 * qq.w() * qq.z(), qq.w() * qq.w() - qq.z() * qq.z()));
        }
      }
      else
      {
        Phi = static_cast<OutputValueType>(ebsdlib::constants::k_PiD);
        phi2 = 0.0; // arbitrarily due to degeneracy
        phi1 = static_cast<OutputValueType>(atan2(2.0 * qq.x() * qq.y(), qq.x() * qq.x() - qq.y() * qq.y()));
      }
    }
    else
    {
      if(ebsdlib::orientations::epsijk == 1.0)
      {
        Phi = static_cast<OutputValueType>(atan2(2.0 * chi, q03 - q12));
        chi = static_cast<OutputValueType>(1.0 / chi);
        phi1 = atan2((-qq.w() * qq.y() + qq.x() * qq.z()) * chi, (-qq.w() * qq.x() - qq.y() * qq.z()) * chi);
        phi2 = atan2((qq.w() * qq.y() + qq.x() * qq.z()) * chi, (-qq.w() * qq.x() + qq.y() * qq.z()) * chi);
      }
      else
      {
        Phi = static_cast<OutputValueType>(atan2(2.0 * chi, q03 - q12));
        chi = static_cast<OutputValueType>(1.0 / chi);
        typename OutputType::value_type y1 = (qq.w() * qq.y() + qq.x() * qq.z()) * chi;
        typename OutputType::value_type x1 = (qq.w() * qq.x() - qq.y() * qq.z()) * chi;
        phi1 = atan2(y1, x1);
        y1 = (-qq.w() * qq.y() + qq.x() * qq.z()) * chi;
        x1 = (qq.w() * qq.x() + qq.y() * qq.z()) * chi;
        phi2 = atan2(y1, x1);
      }
    }

    res[0] = phi1;
    res[1] = Phi;
    res[2] = phi2;

    if(res[0] < 0.0)
    {
      res[0] = static_cast<OutputValueType>(fmod(res[0] + 100.0 * ebsdlib::constants::k_PiD, ebsdlib::constants::k_2PiD));
    }
    if(res[1] < 0.0)
    {
      res[1] = static_cast<OutputValueType>(fmod(res[1] + 100.0 * ebsdlib::constants::k_PiD, ebsdlib::constants::k_PiD));
    }
    if(res[2] < 0.0)
    {
      res[2] = static_cast<OutputValueType>(fmod(res[2] + 100.0 * ebsdlib::constants::k_PiD, ebsdlib::constants::k_2PiD));
    }

    return res;
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    using OutputType = OrientationMatrix<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputValueType qq = w() * w() - (x() * x() + y() * y() + z() * z());

    OutputType res;
    res[0] = static_cast<OutputValueType>(qq + 2.0 * x() * x());
    res[4] = static_cast<OutputValueType>(qq + 2.0 * y() * y());
    res[8] = static_cast<OutputValueType>(qq + 2.0 * z() * z());
    res[1] = static_cast<OutputValueType>(2.0 * (x() * y() - w() * z()));
    res[5] = static_cast<OutputValueType>(2.0 * (y() * z() - w() * x()));
    res[6] = static_cast<OutputValueType>(2.0 * (z() * x() - w() * y()));
    res[3] = static_cast<OutputValueType>(2.0 * (y() * x() + w() * z()));
    res[7] = static_cast<OutputValueType>(2.0 * (z() * y() + w() * x()));
    res[2] = static_cast<OutputValueType>(2.0 * (x() * z() + w() * y()));
    if(ebsdlib::orientations::epsijkd == -1.0)
    {
      auto transpose = res.toGMatrix().transpose();
      res = OutputType(transpose);
    }
    return res;
  }

  AxisAngle<T> toAxisAngle() const
  {
    using OutputType = AxisAngle<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputValueType epsijk = ebsdlib::orientations::epsijkd;
    SelfType qo(*this);

    // make sure q[0] is >= 0.0
    typename OutputType::value_type sign = 1.0;
    if(w() < 0.0)
    {
      sign = -1.0;
    }
    qo = qo.scalarMultiply(sign);

    OutputValueType omega = static_cast<OutputValueType>(2.0 * acos(qo.w()));
    // If omega equals zero then return the rotation axis as [001]
    OutputValueType eps = static_cast<OutputValueType>(1.0e-12L);
    OutputType res;
    if(omega < eps)
    {
      res[0] = 0.0;
      res[1] = 0.0;
      res[2] = static_cast<OutputValueType>(1.0 * epsijk);
      res[3] = 0.0;
      return res;
    }

    if(qo.w() != 0.0)
    {
      OutputValueType mag = magnitude();
      if(mag == 0.0)
      {
        res[0] = 0.0;
        res[1] = 0.0;
        res[2] = static_cast<OutputValueType>(1.0 * epsijk);
        res[3] = 0.0;
      }
      else
      {
        mag = static_cast<OutputValueType>(1.0 / mag);
        res[0] = x() * mag;
        res[1] = y() * mag;
        res[2] = z() * mag;
        res[3] = omega;
      }
    }
    else
    {
      res[0] = x();
      res[1] = y();
      res[2] = z();
      res[3] = ebsdlib::constants::k_PiD;
    }

    return res;
  }

  Rodrigues<T> toRodrigues() const
  {
    using OutputType = Rodrigues<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res;

    T thr = static_cast<OutputValueType>(1.0E-8L);
    res[0] = x();
    res[1] = y();
    res[2] = z();
    res[3] = 0.0;

    if(w() < thr)
    {
      res[3] = std::numeric_limits<typename OutputType::value_type>::infinity();
      return res;
    }
    // ValueType s = ebsdlib::EbsdMatrixMath::Magnitude3x1(&(res[0]));
    OutputValueType s = ArrayHelpers<OutputType, OutputValueType>::sqrtSumOfSquares(res);

    if(s < thr)
    {
      res = {0.0, 0.0, ebsdlib::orientations::epsijk, 0.0};
      return res;
    }

    res[0] = res[0] / s;
    res[1] = res[1] / s;
    res[2] = res[2] / s;
    res[3] = tan(acos(w()));
    return res;
  }

  Quaternion<T> toQuaternion() const
  {
    return *this;
  }

  Homochoric<T> toHomochoric() const
  {
    using OutputType = Homochoric<T>;
    using OMHelperType = ArrayHelpers<OutputType, value_type>;

    OutputType res;

    value_type s;
    value_type f;

    value_type omega = static_cast<value_type>(2.0 * std::acos(w()));
    if(omega == 0.0)
    {
      OMHelperType::splat(res, 0.0);
    }
    else
    {
      res[0] = x();
      res[1] = y();
      res[2] = z();
      s = static_cast<value_type>(1.0 / std::sqrt(OMHelperType::sumofSquares(res)));
      OMHelperType::scalarMultiply(res, s);
      f = static_cast<value_type>(0.75 * (omega - std::sin(omega)));
      f = static_cast<value_type>(std::pow(f, 1.0 / 3.0));
      OMHelperType::scalarMultiply(res, f);
    }
    return res;
  }

  Cubochoric<T> toCubochoric() const
  {
    return toHomochoric().toCubochoric();
  }

  Stereographic<T> toStereographic() const
  {
    using OutputType = Stereographic<T>;

    OutputType res = {x(), y(), z()};

    if(w() != 0.0)
    {
      res[0] = x() / (1.0f + w());
      res[1] = y() / (1.0f + w());
      res[2] = z() / (1.0f + w());
    }
    return res;
  }
};

using QuaternionDType = Quaternion<double>;
using QuaternionFType = Quaternion<float>;

using QuatD = Quaternion<double>;
using QuatF = Quaternion<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Quaternion<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "QU: <" << obj[0] << ", " << obj[1] << ", " << obj[2] << "> " << obj[3];
  return os;
}

} // namespace ebsdlib
