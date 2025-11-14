// Cubochoric.h
#pragma once

#include "EbsdLib/Orientation/Homochoric.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Utilities/ModifiedLambertProjection3D.hpp"

#include <array>

namespace ebsdlib
{
/**
 * @brief This class represents an orientation as a Cubochoric representation.
 *
 * @tparam T Either double or float
 */
template <typename T = double>
class Cubochoric : public OrientationBase<T, 3>
{

public:
  using SuperType = OrientationBase<T, 3>;
  using SelfType = Cubochoric<T>;
  using value_type = T;

  Cubochoric() = default;

  Cubochoric(T x, T y, T z)
  : OrientationBase<T, 3>({x, y, z})
  {
  }

  explicit Cubochoric(const std::array<T, 3>& d)
  : OrientationBase<T, 3>(d)
  {
  }

  explicit Cubochoric(const T* cu)
  : OrientationBase<T, 3>({cu[0], cu[1], cu[2]})
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

  ResultType isValid() const
  {
    using ValueType = value_type;
    ResultType res;
    res.result = 1;

    ValueType maxValue = static_cast<ValueType>(LPs::ap / 2.0);
    bool maxValueHit = false;

    if(std::abs(x()) > maxValue)
    {
      maxValueHit = true;
    }
    if(std::abs(y()) > maxValue)
    {
      maxValueHit = true;
    }
    if(std::abs(z()) > maxValue)
    {
      maxValueHit = true;
    }

    if(maxValueHit)
    {
      res.msg = "Cubochoric Error: cubochoric vector outside cube";
      res.result = -1;
    }
    return res;
  }

  Euler<T> toEuler() const
  {
    return toHomochoric().toEuler();
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    return toHomochoric().toOrientationMatrix();
  }

  AxisAngle<T> toAxisAngle() const
  {
    return toHomochoric().toAxisAngle();
  }

  Rodrigues<T> toRodrigues() const
  {
    return toHomochoric().toRodrigues();
  }

  Quaternion<T> toQuaternion() const
  {
    return toHomochoric().toQuaternion();
  }

  Homochoric<T> toHomochoric() const
  {
    int ierr = 0;
    std::vector<T> temp = {x(), y(), z()};
    std::vector<T> res = ModifiedLambertProjection3D<std::vector<T>, typename SelfType::value_type>::LambertCubeToBall(temp, ierr);
    return {res[0], res[1], res[2]};
  }

  Cubochoric<T> toCubochoric() const
  {
    return *this;
  }

  Stereographic<T> toStereographic() const
  {
    return toQuaternion().toStereographic();
  }
};

using CubochoricDType = Cubochoric<double>;
using CubochoricFType = Cubochoric<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Cubochoric<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "CU: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}

} // namespace ebsdlib
