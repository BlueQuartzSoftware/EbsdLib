// Euler.h
#pragma once

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Orientation/OrientationFwd.hpp"

#include <array>

namespace ebsdlib
{
/**
 * @brief This class represents an orientation as a Bunge Euler representation (ZXZ).
 *
 * @tparam T Either double or float
 */
template <typename T = double>
class Euler : public OrientationBase<T, 3>
{

public:
  using SuperType = OrientationBase<T, 3>;
  using SelfType = Euler<T>;
  using value_type = T;

  Euler() = default;

  Euler(T phi1, T PHI, T phi2)
  : OrientationBase<T, 3>({phi1, PHI, phi2})
  {
  }

  explicit Euler(const std::array<T, 3>& d)
  : OrientationBase<T, 3>(d)
  {
  }

  explicit Euler(const T* eu)
  : OrientationBase<T, 3>({eu[0], eu[1], eu[2]})
  {
  }

  ResultType isValid() const
  {
    ResultType res;
    res.result = 1;

    if((e0() < 0.0) || (e0() > (constants::k_2PiD)))
    {
      res.msg = "Euler Error: phi1 Euler angle outside of valid range [0,2pi]";
      res.result = -1;
    }
    if((e1() < 0.0) || (e1() > constants::k_PiD))
    {
      res.msg = "Euler Error: Phi Euler angle outside of valid range [0,pi]";
      res.result = -2;
    }
    if((e2() < 0.0) || (e2() > (constants::k_2PiD)))
    {
      res.msg = "Euler Error: phi2 Euler angle outside of valid range [0,2pi]";
      res.result = -3;
    }
    return res;
  }

  T& e0()
  {
    return (*this)[0];
  }
  T& e1()
  {
    return (*this)[1];
  }
  T& e2()
  {
    return (*this)[2];
  }

  const T& e0() const
  {
    return (*this)[0];
  }
  const T& e1() const
  {
    return (*this)[1];
  }
  const T& e2() const
  {
    return (*this)[2];
  }

  SelfType toEuler() const
  {
    return *this;
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    using OutputType = OrientationMatrix<T>;
    using ValueType = typename OutputType::value_type;

    OutputType om;
    ValueType eps = 1.0E-7f;

    ValueType c1 = cos(e0());
    ValueType c = cos(e1());
    ValueType c2 = cos(e2());
    ValueType s1 = sin(e0());
    ValueType s = sin(e1());
    ValueType s2 = sin(e2());
    om[0] = c1 * c2 - s1 * s2 * c;
    om[1] = s1 * c2 + c1 * s2 * c;
    om[2] = s2 * s;
    om[3] = -c1 * s2 - s1 * c2 * c;
    om[4] = -s1 * s2 + c1 * c2 * c;
    om[5] = c2 * s;
    om[6] = s1 * s;
    om[7] = -c1 * s;
    om[8] = c;
    for(size_t i = 0; i < 9; i++)
    {
      if(fabs(om[i]) < eps)
      {
        om[i] = 0.0;
      }
    }
    return om;
  }

  AxisAngle<T> toAxisAngle() const
  {
    using OutputType = AxisAngle<T>;
    using OutputValueType = value_type;

    OutputType res;

    auto thr = static_cast<OutputValueType>(1.0E-6);
    auto alpha = static_cast<OutputValueType>(0.0);
    auto t = static_cast<OutputValueType>(tan(e1() * 0.5));
    auto sig = static_cast<OutputValueType>(0.5 * (e0() + e2()));
    auto del = static_cast<OutputValueType>(0.5 * (e0() - e2()));
    auto tau = static_cast<OutputValueType>(std::sqrt(t * t + sin(sig) * sin(sig)));
    if(math::closeEnough<OutputValueType>(sig, static_cast<OutputValueType>(constants::k_PiOver2D), static_cast<OutputValueType>(1.0E-6L)))
    {
      alpha = static_cast<value_type>(constants::k_PiD);
    }
    else
    {
      alpha = static_cast<value_type>(2.0 * atan(tau / cos(sig))); //! return a default identity axis-angle pair
    }

    if(fabs(alpha) < thr)
    {
      res[0] = 0.0;
      res[1] = 0.0;
      res[2] = 1.0;
      res[3] = 0.0;
    }
    else
    {
      //! passive axis-angle pair so a minus sign in front
      res[0] = static_cast<value_type>(-ebsdlib::orientations::epsijkd * t * cos(del) / tau);
      res[1] = static_cast<value_type>(-ebsdlib::orientations::epsijkd * t * sin(del) / tau);
      res[2] = static_cast<value_type>(-ebsdlib::orientations::epsijkd * sin(sig) / tau);
      res[3] = alpha;

      if(alpha < 0.0)
      {
        res[0] = -res[0];
        res[1] = -res[1];
        res[2] = -res[2];
        res[3] = -res[3];
      }
    }

    return res;
  }

  Rodrigues<T> toRodrigues() const
  {
    using OutputType = Rodrigues<T>;
    using OutputValueType = typename OutputType::value_type;

    typename OutputType::value_type thr = 1.0E-6f;

    AxisAngle res = toAxisAngle();
    using OutputValueType = typename OutputType::value_type;
    typename OutputType::value_type t = res[3];

    if(std::fabs(t - constants::k_PiD) < thr)
    {
      res[3] = std::numeric_limits<typename OutputType::value_type>::infinity();
      return {res[0], res[1], res[2], res[3]};
    }

    if(math::closeEnough<OutputValueType>(t, static_cast<typename OutputType::value_type>(0.0), thr)) // Are we close to Zero
    {
      res = {0.0, 0.0, ebsdlib::orientations::epsijk, 0.0};
    }
    else
    {
      res[3] = static_cast<typename OutputType::value_type>(tan(t * 0.5));
    }

    return {res[0], res[1], res[2], res[3]};
  }

  Quaternion<T> toQuaternion() const
  {
    using OutputType = Quaternion<T>;
    using OutputValueType = value_type;

    std::array<OutputValueType, 3> ee = {0.0f, 0.0f, 0.0f};
    OutputValueType cPhi = 0.0f;
    OutputValueType cp = 0.0f;
    OutputValueType cm = 0.0f;
    OutputValueType sPhi = 0.0f;
    OutputValueType sp = 0.0f;
    OutputValueType sm = 0.0f;

    ee[0] = static_cast<OutputValueType>(0.5 * e0());
    ee[1] = static_cast<OutputValueType>(0.5 * e1());
    ee[2] = static_cast<OutputValueType>(0.5 * e2());

    cPhi = cos(ee[1]);
    sPhi = sin(ee[1]);
    cm = cos(ee[0] - ee[2]);
    sm = sin(ee[0] - ee[2]);
    cp = cos(ee[0] + ee[2]);
    sp = sin(ee[0] + ee[2]);

    Quaternion<T> res;
    res.w() = cPhi * cp;
    res.x() = -ebsdlib::orientations::epsijk * sPhi * cm;
    res.y() = -ebsdlib::orientations::epsijk * sPhi * sm;
    res.z() = -ebsdlib::orientations::epsijk * cPhi * sp;

    if(res.w() < 0.0)
    {
      res = res.negate();
    }

    if(res.x() == -0.0)
    {
      res.x() = -res.x();
    }
    if(res.y() == -0.0)
    {
      res.y() = -res.y();
    }
    if(res.z() == -0.0)
    {
      res.z() = -res.z();
    }

    return res;
  }

  Homochoric<T> toHomochoric() const
  {
    return toAxisAngle().toHomochoric();
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

using EulerDType = Euler<double>;
using EulerFType = Euler<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const Euler<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "EU: " << obj[0] << ", " << obj[1] << ", " << obj[2];
  return os;
}

} // namespace ebsdlib
