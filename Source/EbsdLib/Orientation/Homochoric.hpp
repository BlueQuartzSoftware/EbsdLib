// Homochoric.h
#pragma once
#include "OrientationFwd.hpp"

#include "EbsdLib/Math/ArrayHelpers.hpp"
#include "EbsdLib/Orientation/AxisAngle.hpp"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/OrientationMatrix.hpp"
#include "EbsdLib/Utilities/ModifiedLambertProjection3D.hpp"

#include <array>

namespace ebsdlib
{
/**
 * @brief This class represents an orientation as a Homochoric representation.
 *
 * @tparam T Either double or float
 */
template <typename T = double>
class Homochoric : public OrientationBase<T, 3>
{

public:
  using SuperType = OrientationBase<T, 3>;
  using SelfType = Homochoric<T>;
  using value_type = T;

  Homochoric() = default;

  Homochoric(T x, T y, T z)
  : OrientationBase<T, 3>({x, y, z})
  {
  }

  explicit Homochoric(const std::array<T, 3>& d)
  : OrientationBase<T, 3>(d)
  {
  }

  explicit Homochoric(const T* ho)
  : OrientationBase<T, 3>({ho[0], ho[1], ho[2]})
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
    ResultType res;
    res.result = 1;

    value_type r = std::sqrt(x() * x() + y() * y() + z() * z());

    if(r > static_cast<float>(LPs::R1))
    {
      res.msg = "Homochoric Error: homochoric vector outside homochoric ball";
      res.result = -1;
    }
    return res;
  }

  Euler<T> toEuler() const
  {
    return toAxisAngle().toEuler();
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    return toAxisAngle().toOrientationMatrix();
  }

  AxisAngle<T> toAxisAngle() const
  {
    using OutputType = AxisAngle<T>;
    using OutputValueType = typename OutputType::value_type;
    using SizeType = typename OutputType::size_type;
    using OMHelperType = ArrayHelpers<SelfType, value_type>;

    OutputType res;

    value_type thr = 1.0E-8f;

    typename OutputType::value_type hmag = ArrayHelpers<SelfType, value_type>::sumofSquares((*this));
    if(hmag == 0.0)
    {
      res[0] = 0.0;
      res[1] = 0.0;
      res[2] = 1.0;
      res[3] = 0.0;
    }
    else
    {
      OutputValueType hm = hmag;
      SelfType hn(*this);
      OutputValueType sqrRtHMag = static_cast<OutputValueType>(1.0 / sqrt(hmag));
      ArrayHelpers<SelfType, typename SelfType::value_type>::scalarMultiply(hn, sqrRtHMag); // In place scalar multiply
      OutputValueType s = static_cast<OutputValueType>(LambertParametersType::tfit[0] + LambertParametersType::tfit[1] * hmag);
      for(int i = 2; i < 16; i++)
      {
        hm = hm * hmag;
        s = static_cast<OutputValueType>(s + LPs::tfit[i] * hm);
      }
      s = static_cast<OutputValueType>(2.0 * acos(s));
      res[0] = hn[0];
      res[1] = hn[1];
      res[2] = hn[2];
      OutputValueType delta = static_cast<OutputValueType>(std::fabs(s - constants::k_PiD));
      if(delta < thr)
      {
        res[3] = static_cast<value_type>(constants::k_PiD);
      }
      else
      {
        res[3] = s;
      }
    }
    return res;
  }

  Rodrigues<T> toRodrigues() const
  {
    return toAxisAngle().toRodrigues();
  }

  Quaternion<T> toQuaternion() const
  {
    return toAxisAngle().toQuaternion();
  }

  Homochoric<T> toHomochoric() const
  {
    return *this;
  }

  Cubochoric<T> toCubochoric() const
  {
    int ierr = -1;
    std::vector<T> temp = {x(), y(), z()};
    std::vector<T> res = ModifiedLambertProjection3D<std::vector<T>, typename SelfType::value_type>::LambertBallToCube(temp, ierr);
    return {res[0], res[1], res[2]};
  }

  Stereographic<T> toStereographic() const
  {
    return toQuaternion().toStereographic();
  }
};

using HomochoricDType = Homochoric<double>;
using HomochoricFType = Homochoric<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Homochoric<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "HO: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}

} // namespace ebsdlib
