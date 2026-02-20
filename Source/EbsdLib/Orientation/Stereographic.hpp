#pragma once

#include "EbsdLib/Math/ArrayHelpers.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"

#include <array>
#include <sstream>

namespace ebsdlib
{

/**
 * @brief This class represents an orientation as a 3D Stereographic coordinate where the
 * coordinate must fall within the unit sphere of radius = 1. This representation
 * can be useful if you wish to visualize orientations as 3D points in space such
 * as a Rodrigues fundamental zone visualization.
 * @tparam T Either double or float
 */
template <typename T = double>
class Stereographic : public OrientationBase<T, 3>
{

public:
  using SuperType = OrientationBase<T, 3>;
  using SelfType = Stereographic<T>;
  using value_type = T;

  Stereographic() = default;

  Stereographic(T x, T y, T z)
  : OrientationBase<T, 3>({x, y, z})
  {
  }

  explicit Stereographic(const std::array<T, 3>& d)
  : OrientationBase<T, 3>(d)
  {
  }

  explicit Stereographic(const T* st)
  : OrientationBase<T, 3>({st[0], st[1], st[2]})
  {
  }

  T& x()
  {
    return (*this)[0];
  }
  const T& x() const
  {
    return (*this)[0];
  }

  T& y()
  {
    return (*this)[1];
  }
  const T& y() const
  {
    return (*this)[1];
  }

  T& z()
  {
    return (*this)[2];
  }
  const T& z() const
  {
    return (*this)[2];
  }

  T dot() const
  {
    return (x() * x() + y() * y() + z() * z());
  }

  T magnitude() const
  {
    return std::sqrt(dot());
  }

  ResultType isValid() const
  {
    using ValueType = value_type;
    ResultType res;
    res.result = 1;

    value_type epsd = 1.0E-15; // std::numeric_limits<float>::epsilon();
    ValueType rd = magnitude();
    if(rd > 1.0 + epsd)
    {
      std::stringstream oss;
      oss << std::setprecision(16) << "Stereographic Error: Stereographic vector must have unit norm <= unity " << rd << " Unity: " << (1.0 + epsd) << " epsd: " << epsd;
      res.result = -2;
      res.msg = oss.str();
    }

    return res;
  }

  Euler<T> toEuler() const
  {
    return toAxisAngle().toEuler();
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    using OutputType = OrientationMatrix<T>;
    using OutputValueType = typename OutputType::value_type;

    auto threshold = static_cast<OutputValueType>(1.0E-5L);
    OutputType res = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // Init with identity matrix

    OutputValueType l = std::sqrt(ArrayHelpers<SelfType, OutputValueType>::sumofSquares(*this));
    if(l > 0.0)
    {
      SelfType tmp(*this);
      ArrayHelpers<SelfType, OutputValueType>::scalarDivide(tmp, l);
      AxisAngle<OutputValueType> ax;
      if(math::closeEnough(l, static_cast<OutputValueType>(1.0L), threshold))
      {
        ax = AxisAngle<OutputValueType>(tmp[0], tmp[1], tmp[2], static_cast<OutputValueType>(constants::k_PiD));
      }
      else
      {
        ax = AxisAngle<OutputValueType>(tmp[0], tmp[1], tmp[2], static_cast<OutputValueType>(4.0 * std::atan(l)));
      }
      res = ax.toOrientationMatrix();
    }
    return res;
  }

  AxisAngle<T> toAxisAngle() const
  {
    using OutputType = AxisAngle<T>;
    using OutputValueType = typename OutputType::value_type;

    auto threshold = static_cast<OutputValueType>(1.0E-5L);
    OutputType res = {0.0, 0.0, 1.0, 0.0};

    OutputValueType l = std::sqrt(ArrayHelpers<SelfType, OutputValueType>::sumofSquares(*this));
    if(l > 0.0)
    {
      SelfType tmp(*this);
      ArrayHelpers<SelfType, OutputValueType>::scalarDivide(tmp, l);
      if(math::closeEnough(l, static_cast<OutputValueType>(1.0L), threshold))
      {
        res = {tmp[0], tmp[1], tmp[2], static_cast<OutputValueType>(constants::k_PiD)};
      }
      else
      {
        res = {tmp[0], tmp[1], tmp[2], static_cast<OutputValueType>(4.0 * std::atan(l))};
      }
    }
    return res;
  }

  Rodrigues<T> toRodrigues() const
  {
    using OutputType = Rodrigues<T>;

    using ValueType = typename OutputType::value_type;
    auto threshold = static_cast<ValueType>(1.0E-5L);
    OutputType res = {0.0, 0.0, 1.0, 0.0};
    ValueType l = std::sqrt(ArrayHelpers<SelfType, ValueType>::sumofSquares(*this));

    if(l > 0.0) // ! not the identity rotation
    {
      SelfType tmp(*this);
      ArrayHelpers<SelfType, ValueType>::scalarDivide(tmp, l);

      if(math::closeEnough(l, static_cast<ValueType>(1.0L), threshold))
      {
        res = {tmp[0], tmp[1], tmp[2], static_cast<ValueType>(std::numeric_limits<ValueType>::infinity())};
      }
      else
      {
        res = {tmp[0], tmp[1], tmp[2], static_cast<ValueType>(std::tan(2.0 * std::atan(l)))};
      }
    }
    return res;
  }

  Quaternion<T> toQuaternion() const
  {
    return toAxisAngle().toQuaternion();
  }

  Homochoric<T> toHomochoric() const
  {
    using OutputType = Homochoric<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res = {0.0, 0.0, 0.0};
    OutputValueType l = std::sqrt(ArrayHelpers<SelfType, OutputValueType>::sumofSquares(*this));

    if(l > 0.0)
    {
      SelfType tmp(*this);
      ArrayHelpers<SelfType, OutputValueType>::scalarDivide(tmp, l);

      OutputValueType angle = 4.0 * std::atan(l);
      OutputValueType temp2 = (3.0 * (angle - std::sin(angle)) / 4.0);
      temp2 = std::pow(temp2, (1.0 / 3.0));

      ArrayHelpers<SelfType, OutputValueType>::scalarMultiply(tmp, temp2);
      res[0] = tmp[0];
      res[1] = tmp[1];
      res[2] = tmp[2];
    }
    return res;
  }

  Cubochoric<T> toCubochoric() const
  {
    using OutputType = Cubochoric<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res = {0.0, 0.0, 0.0};
    OutputValueType l = std::sqrt(ArrayHelpers<SelfType, OutputValueType>::sumofSquares(*this));

    if(l > 0.0)
    {
      SelfType tmp(*this);
      ArrayHelpers<SelfType, OutputValueType>::scalarDivide(tmp, l);

      OutputValueType angle = 4.0 * std::atan(l);
      OutputValueType temp2 = (3.0 * (angle - std::sin(angle)) / 4.0);
      temp2 = std::pow(temp2, (1.0 / 3.0));

      ArrayHelpers<SelfType, OutputValueType>::scalarMultiply(tmp, temp2);
      Homochoric<T> ho(tmp.underlying());
      return ho.toCubochoric();
    }

    return res;
  }

  Stereographic<T> toStereographic() const
  {
    return *this;
  }
};

using StereographicDType = Stereographic<double>;
using StereographicFType = Stereographic<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Stereographic<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "ST: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}

} // namespace ebsdlib
