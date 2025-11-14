// Rodrigues.h
#pragma once

#include "EbsdLib/Math/ArrayHelpers.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"

#include <array>

namespace ebsdlib
{

/**
 * @brief This class represents an orientation as a Rodrigues-Frank vector.
 *
 * @note EbsdLib uses a 4 component Rodrigues vector where the length is stored
 * in the 4th component.
 *
 * @param T Either double or float
 */
template <typename T = double>
class Rodrigues : public OrientationBase<T, 4>
{

public:
  using SuperType = OrientationBase<T, 4>;
  using SelfType = Rodrigues<T>;
  using value_type = T;

  Rodrigues() = default;

  /**
   * @brief Convenience constructor that will convert from a 3 element Rodrigues vector
   * to the 4 element Rodrigues vector that EbsdLib uses.
   *
   * The conversion is:
   * length = sprt(x^2 + y^2 + z^2)
   * x = x / length
   * y = y / length
   * z = z / length
   * l = length
   *
   * @param x
   * @param y
   * @param z
   */
  Rodrigues(T x, T y, T z)
  : OrientationBase<T, 4>({x, y, z, 0.0})
  {
    const T length = sqrtf(x * x + y * y + z * z);
    x() = x / length;
    y() = y / length;
    z() = z / length;
    l() = length;
  }

  /**
   * @brief Constructs a Rodrigues orientation from the 4 components
   * @param x
   * @param y
   * @param z
   * @param l The lenght of the Rodrigues vector
   */
  Rodrigues(T x, T y, T z, T l)
  : OrientationBase<T, 4>({x, y, z, l})
  {
  }
  /**
   * @brief Constructs a Rodrigues orientation from the 4 components
   * @param d
   */
  explicit Rodrigues(const std::array<T, 4>& d)
  : OrientationBase<T, 4>(d)
  {
  }

  /**
   * @brief Constructs a Rodrigues orientation from the 4 components
   * @param rod
   */
  explicit Rodrigues(const T* rod)
  : OrientationBase<T, 4>({rod[0], rod[1], rod[2], rod[3]})
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

  const T& l() const
  {
    return (*this)[3];
  }
  T& l()
  {
    return (*this)[3];
  }

  ResultType isValid() const
  {
    auto eps = static_cast<value_type>(1.0E-6L);
    ResultType res;
    res.result = 1;
    if(l() < 0.0L)
    {
      res.msg = "Rodrigues Error: Rodrigues-Frank vector has negative length: ";
      res.result = -1;
      return res;
    }
    value_type ttl = std::sqrt(x() * x() + y() * y() + z() * z());

    if(std::fabs(ttl - 1.0) > eps)
    {
      res.msg = "Rodrigues Error: Rodrigues-Frank axis vector not normalized";
      res.result = -2;
    }
    return res;
  }

  Euler<T> toEuler() const
  {
    return toOrientationMatrix().toEuler();
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    return toAxisAngle().toOrientationMatrix();
  }

  AxisAngle<T> toAxisAngle() const
  {
    using OutputType = AxisAngle<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res;
    OutputValueType ta = 0.0L;
    OutputValueType angle = 0.0L;
    typename OutputType::value_type threshold = 1.0E-6f;

    ta = l();
    if(math::closeEnough<OutputValueType>(ta, static_cast<OutputValueType>(0.0L), threshold))
    {
      res = {0.0, 0.0, ebsdlib::orientations::epsijk, 0.0};
      return res;
    }

    if(ta == std::numeric_limits<typename OutputType::value_type>::infinity())
    {
      res[0] = x();
      res[1] = y();
      res[2] = z();
      res[3] = static_cast<OutputValueType>(constants::k_PiD);
    }
    else
    {
      angle = static_cast<OutputValueType>(2.0L * atan(ta));
      ta = 1.0L / sqrt(x() * x() + y() * y() + z() * z());
      res[0] = x() * ta;
      res[1] = y() * ta;
      res[2] = z() * ta;
      res[3] = angle;
    }
    return res;
  }

  Rodrigues<T> toRodrigues() const
  {
    return *this;
  }

  Quaternion<T> toQuaternion() const
  {
    return toAxisAngle().toQuaternion();
  }

  Homochoric<T> toHomochoric() const
  {
    using OutputType = Homochoric<T>;
    using OutputValueType = typename OutputType::value_type;
    using SizeType = typename OutputType::size_type;
    using OMHelperType = ArrayHelpers<SelfType, value_type>;

    OutputType res;
    OutputValueType f = 0.0;
    OutputValueType rv = OMHelperType::sumofSquares(*this);
    if(rv == 0.0)
    {
      ArrayHelpers<OutputType, OutputValueType>::splat(res, 0.0);
      return res;
    }
    if(l() == std::numeric_limits<OutputValueType>::infinity())
    {
      f = static_cast<OutputValueType>(0.75 * constants::k_PiD);
    }
    else
    {
      auto t = static_cast<OutputValueType>(2.0 * std::atan(l()));
      f = static_cast<value_type>(0.75 * (t - std::sin(t)));
    }
    f = static_cast<value_type>(pow(f, 1.0 / 3.0));
    res[0] = x() * f;
    res[1] = y() * f;
    res[2] = z() * f;
    return res;
  }

  Cubochoric<T> toCubochoric() const
  {
    return toHomochoric().toCubochoric();
  }

  Stereographic<T> toStereographic() const
  {
    return toQuaternion().toStereographic();
  }
};

using RodriguesDType = Rodrigues<double>;
using RodriguesFType = Rodrigues<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Rodrigues<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "RO: <" << obj[0] << ", " << obj[1] << ", " << obj[2] << "> " << obj[3];
  return os;
}

} // namespace ebsdlib
