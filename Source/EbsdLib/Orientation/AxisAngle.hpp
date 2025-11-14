// AxisAngle.h
#pragma once

#include "EbsdLib/Orientation/Cubochoric.hpp"
#include "EbsdLib/Orientation/Homochoric.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Orientation/Stereographic.hpp"

#include <array>
#include <sstream>

namespace ebsdlib
{

/**
 * @brief This class represents an orientation as an Axis-Angle pair. The data is stored in a
 * flat 4x1 vector where the first three elements are the axis and the last element is
 * the rotation around that axis in radians.
 *
 * @tparam T Either double or float
 */
template <typename T = double>
class AxisAngle : public OrientationBase<T, 4>
{

public:
  using SuperType = OrientationBase<T, 4>;
  using SelfType = AxisAngle<T>;
  using value_type = T;

  AxisAngle() = default;

  AxisAngle(T x, T y, T z, T w)
  : OrientationBase<T, 4>({x, y, z, w})
  {
  }

  explicit AxisAngle(const std::array<T, 4>& d)
  : OrientationBase<T, 4>(d)
  {
  }

  explicit AxisAngle(const T* mat)
  : OrientationBase<T, 4>({mat[0], mat[1], mat[2], mat[3]})
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

  ResultType isValid() const
  {
    ResultType res;
    res.result = 1;
    if((w() < 0.0) || (w() > constants::k_PiD))
    {
      res.msg = "Axis Angle Error: angle must be in range [0,pi]";
      res.result = -1;
      return res;
    }
    T eps = std::numeric_limits<float>::epsilon();

    T r = std::sqrt(x() * x() + y() * y() + z() * z());
    T absv = static_cast<T>(fabs(r - 1.0));

    if(absv > eps)
    {
      std::stringstream oss;
      oss << std::setprecision(16) << "Axis Angle Error: axis-angle axis vector must have unit norm. " << r << " Difference is: " << absv << " eps: " << eps;
      res.msg = oss.str();
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
    using OutputType = OrientationMatrix<T>;

    OutputType res;
    value_type q = 0.0L;
    value_type c = 0.0L;
    value_type s = 0.0L;
    value_type omc = 0.0L;

    c = cos((*this)[3]);
    s = sin((*this)[3]);

    omc = static_cast<value_type>(1.0 - c);

    res[0] = x() * x() * omc + c;
    res[4] = y() * y() * omc + c;
    res[8] = z() * z() * omc + c;
    int _01 = 1;
    int _10 = 3;
    int _12 = 5;
    int _21 = 7;
    int _02 = 2;
    int _20 = 6;
    // Check to see if we need to transpose
    if(ebsdlib::orientations::epsijk == 1.0L)
    {
      _01 = 3;
      _10 = 1;
      _12 = 7;
      _21 = 5;
      _02 = 6;
      _20 = 2;
    }

    q = omc * x() * (*this)[1];
    res[_01] = q + s * (*this)[2];
    res[_10] = q - s * (*this)[2];
    q = omc * y() * (*this)[2];
    res[_12] = q + s * (*this)[0];
    res[_21] = q - s * (*this)[0];
    q = omc * z() * (*this)[0];
    res[_02] = q - s * (*this)[1];
    res[_20] = q + s * (*this)[1];

    return res;
  }

  AxisAngle<T> toAxisAngle() const
  {
    return *this;
  }

  Rodrigues<T> toRodrigues() const
  {
    using OutputType = Rodrigues<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res;

    OutputValueType threshold = 1.0E-7f;
    if(math::closeEnough<OutputValueType>((*this)[3], static_cast<OutputValueType>(0.0L), threshold))
    {
      res = {0.0, 0.0, ebsdlib::orientations::epsijk, 0.0};
      return res;
    }
    res[0] = (*this)[0];
    res[1] = (*this)[1];
    res[2] = (*this)[2];
    if(fabs((*this)[3] - constants::k_PiD) < threshold)
    {
      res[3] = std::numeric_limits<typename OutputType::value_type>::infinity();
    }
    else
    {
      res[3] = static_cast<OutputValueType>(tan((*this)[3] * 0.5));
    }
    return res;
  }

  Quaternion<T> toQuaternion() const
  {
    using OutputType = Quaternion<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res;

    if((*this)[3] == 0.0)
    {
      res.w() = 1.0;
      res.x() = 0.0;
      res.y() = 0.0;
      res.z() = 0.0;
    }
    else
    {
      typename OutputType::value_type c = static_cast<OutputValueType>(cos((*this)[3] * 0.5));
      typename OutputType::value_type s = static_cast<OutputValueType>(sin((*this)[3] * 0.5));
      res.w() = c;
      res.x() = x() * s;
      res.y() = y() * s;
      res.z() = z() * s;
    }
    return res;
  }

  Homochoric<T> toHomochoric() const
  {
    using OutputType = Homochoric<T>;

    OutputType res;
    typename OutputType::value_type f = static_cast<typename OutputType::value_type>(0.75 * ((*this)[3] - sin((*this)[3])));
    f = static_cast<typename OutputType::value_type>(pow(f, (1.0 / 3.0)));
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

using AxisAngleDType = AxisAngle<double>;
using AxisAngleFType = AxisAngle<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const AxisAngle<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "AX: <" << obj[0] << ", " << obj[1] << ", " << obj[2] << "> " << obj[3];
  return os;
}

} // namespace ebsdlib
