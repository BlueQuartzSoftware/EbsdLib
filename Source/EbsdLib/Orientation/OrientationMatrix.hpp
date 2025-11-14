

#pragma once
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Math/Matrix3X3.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"

#include <Eigen/Dense>

#include <array>

namespace ebsdlib
{
/**
 * @brief This class represents an orientation as a 3x3 Matrix typically referred to as
 * an orientation matrix. The data is stored as Row-Major format:
 *
 * @code
 * | 0 1 2 |
 * | 3 4 5 |
 * | 6 7 8 |
 * @endcode
 *
 * @tparam T Either double or float
 */
template <typename T = double>
class OrientationMatrix : public OrientationBase<T, 9>
{

public:
  using SuperType = OrientationBase<T, 9>;
  using SelfType = OrientationMatrix<T>;
  using value_type = T;

  OrientationMatrix() = default;

  /**
   * @brief Construct an Orientation Matrix from a 3x3 matrix that is row major, i.e., the mat is laid out in memory each row is contiguous
   * after the previous row.
   * @param m00
   * @param m01
   * @param m02
   * @param m10
   * @param m11
   * @param m12
   * @param m20
   * @param m21
   * @param m22
   */
  OrientationMatrix(T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21, T m22)
  : OrientationBase<T, 9>({m00, m01, m02, m10, m11, m12, m20, m21, m22})
  {
  }

  /**
   * @brief Construct an Orientation Matrix from a 3x3 matrix that is row major, i.e., the mat is laid out in memory each row is contiguous
   * after the previous row.
   *
   * @note | r0c0, r0c1, r0c2, r1c0 .... |
   * @param mat
   */
  explicit OrientationMatrix(const Matrix3X3<T>& mat)
  : OrientationBase<T, 9>({mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8]})
  {
  }

  /**
   * @brief Construct an Orientation Matrix from a 3x3 matrix that is row major, i.e., the mat is laid out in memory each row is contiguous
   * after the previous row.
   *
   * @note | r0c0, r0c1, r0c2, r1c0 .... |
   * @param mat
   */
  explicit OrientationMatrix(const std::array<T, 9>& d)
  : OrientationBase<T, 9>(d)
  {
  }

  /**
   * @brief Construct an Orientation Matrix from a 3x3 matrix that is row major, i.e., the mat is laid out in memory each row is contiguous
   * after the previous row.
   *
   * @note | r0c0, r0c1, r0c2, r1c0 .... |
   * @param mat
   */
  explicit OrientationMatrix(const T* mat)
  : OrientationBase<T, 9>({mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8]})
  {
  }

  /**
   * @brief Returns the matrix as an EbsdLib::Matrix3x3 which is suitable for matrix calculations.
   * @return
   */
  Matrix3X3<T> toGMatrix() const
  {
    return Matrix3X3<T>{(*this)[0], (*this)[1], (*this)[2], (*this)[3], (*this)[4], (*this)[5], (*this)[6], (*this)[7], (*this)[8]};
  }

  /**
   * @brief Returns the matrix as an Eigen 3x3 Matrix that is RowMajor.
   * @return
   */
  Eigen::Matrix<T, 3, 3, Eigen::RowMajor> toEigenGMatrix() const
  {
    Eigen::Matrix<T, 3, 3, Eigen::RowMajor> g1;
    g1(0, 0) = (*this)[0];
    g1(0, 1) = (*this)[1];
    g1(0, 2) = (*this)[2];
    g1(1, 0) = (*this)[3];
    g1(1, 1) = (*this)[4];
    g1(1, 2) = (*this)[5];
    g1(2, 0) = (*this)[6];
    g1(2, 1) = (*this)[7];
    g1(2, 2) = (*this)[8];
    return g1;
  }

  /**
   * @brief Matrix multiplication of a 3x1 matrix
   * @param vec
   * @return
   */
  Eigen::Matrix<T, 3, 1> operator*(const Eigen::Matrix<T, 3, 1>& vec) const
  {
    return toEigenGMatrix() * vec;
  }

  /**
   * @brief Matrix multiplication of a 3x3 matrix
   * @param rhs
   * @return
   */
  Eigen::Matrix<T, 3, 3> operator*(const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>& rhs) const
  {
    return toEigenGMatrix() * rhs;
  }

  /**
   * @brief Matrix multiplication of a 3x3 matrix
   * @param rhs
   * @return
   */
  SelfType operator*(const SelfType& rhs) const
  {
    Matrix3X3<T> a(this->data());
    Matrix3X3<T> b(rhs.data());
    SelfType res(a * b);
    return res;
  }

  /**
   * @brief returns the orientation matrix transposed. This is useful to turn this into an active transformation matrix
   * @return
   */
  SelfType transpose() const
  {
    const SelfType& in = *this;
    return {in[0], in[3], in[6], in[1], in[4], in[7], in[2], in[5], in[8]};
  }

  /**
   * @brief Checks if this Orientation Matrix is crystallographically valid
   * @return Result type Struct with an error code and error message
   */
  ResultType isValid() const
  {
    using ValueType = T;
    ResultType res;
    res.result = 1;
    auto threshold = static_cast<ValueType>(1.0E-5L);
    using RotationMatrixType = Eigen::Matrix<ValueType, 3, 3, Eigen::RowMajor>;
    using RotationMatrixMapType = Eigen::Map<RotationMatrixType>;
    RotationMatrixMapType omE(const_cast<ValueType*>(this->data()));

    ValueType det = omE.determinant();

    std::stringstream ss;
    if(det < 0.0)
    {
      ss << "OrientationMatrix Error: Determinant of rotation matrix must be positive: " << det;
      res.msg = ss.str();
      res.result = -1;
      return res;
    }

    ValueType r = fabs(det - static_cast<ValueType>(1.0L));
    if(!math::closeEnough<ValueType>(r, static_cast<ValueType>(0.0L), threshold))
    {
      ss << "OrientationMatrix Error: Determinant (" << det << ") of rotation matrix must be unity (1.0)";
      res.msg = ss.str();
      res.result = -2;
      return res;
    }

    RotationMatrixType abv = (omE * omE.transpose()).cwiseAbs();

    RotationMatrixType identity;
    identity.setIdentity();

    identity = identity - abv;
    identity = identity.cwiseAbs();

    for(int c = 0; c < 3; c++)
    {
      for(int rIndex = 0; rIndex < 3; rIndex++)
      {
        if(identity(rIndex, c) > threshold)
        {
          std::stringstream ssError;
          ssError << "OrientationMatrix Error: OrientationMatrix times transpose must be identity matrix: (";
          ssError << rIndex << ", " << c << ") = " << abv(rIndex, c);
          res.msg = ssError.str();
          res.result = -3;
        }
      }
    }

    return res;
  }

  Euler<T> toEuler() const
  {
    using OutputType = Euler<T>;
    using OutputValueType = typename OutputType::value_type;
    OutputType res;
    typename OutputType::value_type zeta = 0.0;
    bool close = math::closeEnough<OutputValueType>(std::fabs((*this)[8]), static_cast<typename OutputType::value_type>(1.0), static_cast<typename OutputType::value_type>(1.0E-6));
    if(!close)
    {
      res[1] = acos((*this)[8]);
      zeta = static_cast<typename OutputType::value_type>(1.0 / sqrt(1.0 - (*this)[8] * (*this)[8]));
      res[0] = atan2((*this)[6] * zeta, -(*this)[7] * zeta);
      res[2] = atan2((*this)[2] * zeta, (*this)[5] * zeta);
    }
    else
    {
      close = math::closeEnough<OutputValueType>((*this)[8], static_cast<typename OutputType::value_type>(1.0), static_cast<typename OutputType::value_type>(1.0E-6));
      if(close)
      {
        res[0] = atan2((*this)[1], (*this)[0]);
        res[1] = 0.0;
        res[2] = 0.0;
      }
      else
      {
        res[0] = static_cast<OutputValueType>(-atan2(-(*this)[1], (*this)[0]));
        res[1] = static_cast<OutputValueType>(constants::k_PiD);
        res[2] = 0.0;
      }
    }

    if(res[0] < 0.0)
    {
      res[0] = static_cast<typename OutputType::value_type>(fmod(res[0] + 100.0 * constants::k_PiD, constants::k_2PiD));
    }
    if(res[1] < 0.0)
    {
      res[1] = static_cast<typename OutputType::value_type>(fmod(res[1] + 100.0 * constants::k_PiD, constants::k_PiD));
    }
    if(res[2] < 0.0)
    {
      res[2] = static_cast<typename OutputType::value_type>(fmod(res[2] + 100.0 * constants::k_PiD, constants::k_2PiD));
    }
    return res;
  }

  OrientationMatrix<T> toOrientationMatrix() const
  {
    return *this;
  }

  AxisAngle<T> toAxisAngle() const
  {
    return toQuaternion().toAxisAngle();
  }

  Rodrigues<T> toRodrigues() const
  {
    return toEuler().toRodrigues();
  }

  Quaternion<T> toQuaternion() const
  {
    using OutputType = Quaternion<T>;
    using OutputValueType = typename OutputType::value_type;

    OutputType res;

    auto thr = static_cast<typename OutputType::value_type>(1.0E-10L);
    if(sizeof(value_type) == 4)
    {
      thr = static_cast<typename OutputType::value_type>(1.0E-6L);
    }
    OutputValueType s = 0.0;
    OutputValueType s1 = 0.0;
    OutputValueType s2 = 0.0;
    OutputValueType s3 = 0.0;

    s = static_cast<OutputValueType>((*this)[0] + (*this)[4] + (*this)[8] + 1.0);
    if(math::closeEnough<OutputValueType>(std::fabs(s), static_cast<typename OutputType::value_type>(0.0), thr)) // Are we close to Zero
    {
      s = 0.0;
    }
    s = sqrt(s);
    s1 = static_cast<OutputValueType>((*this)[0] - (*this)[4] - (*this)[8] + 1.0);
    if(math::closeEnough<OutputValueType>(std::fabs(s1), static_cast<typename OutputType::value_type>(0.0), thr)) // Are we close to Zero
    {
      s1 = 0.0;
    }
    s1 = sqrt(s1);
    s2 = static_cast<OutputValueType>(-(*this)[0] + (*this)[4] - (*this)[8] + 1.0);
    if(math::closeEnough<OutputValueType>(std::fabs(s2), static_cast<typename OutputType::value_type>(0.0), thr)) // Are we close to Zero
    {
      s2 = 0.0;
    }
    s2 = sqrt(s2);
    s3 = static_cast<OutputValueType>(-(*this)[0] - (*this)[4] + (*this)[8] + 1.0);
    if(math::closeEnough<OutputValueType>(std::fabs(s3), static_cast<typename OutputType::value_type>(0.0), thr)) // Are we close to Zero
    {
      s3 = 0.0;
    }
    s3 = sqrt(s3);
    res.w() = static_cast<OutputValueType>(s * 0.5);
    res.x() = static_cast<OutputValueType>(s1 * 0.5);
    res.y() = static_cast<OutputValueType>(s2 * 0.5);
    res.z() = static_cast<OutputValueType>(s3 * 0.5);
    // printf("res[z]: % 3.16f \n", res[z]);

    // verify the signs (q0 always positive)
    if((*this)[7] < (*this)[5])
    {
      res.x() = -ebsdlib::orientations::epsijk * res.x();
    }
    if((*this)[2] < (*this)[6])
    {
      res.y() = -ebsdlib::orientations::epsijk * res.y();
    }
    if((*this)[3] < (*this)[1])
    {
      res.z() = -ebsdlib::orientations::epsijk * res.z();
    }
    // printf("res[z]: % 3.16f \n", res[z]);

    s = res.length();

    if(s != 0.0)
    {
      res = res.scalarDivide(s);
    }

    /* we need to do a quick test here to make sure that the
    ! sign of the vector part is the same as that of the
    ! corresponding vector in the axis-angle representation;
    ! these two can end up being different, presumably due to rounding
    ! issues, but this needs to be further analyzed...
    ! This adds a little bit of computation overhead but for now it
    ! is the easiest way to make sure the signs are correct.
    */
    AxisAngle<T> oax = toEuler().toAxisAngle();

    if(oax[0] * res.x() < 0.0)
    {
      res.x() = -res.x();
    }
    if(oax[1] * res.y() < 0.0)
    {
      res.y() = -res.y();
    }
    if(oax[2] * res.z() < 0.0)
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

using OrientationMatrixDType = OrientationMatrix<double>;
using OrientationMatrixFType = OrientationMatrix<float>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const OrientationMatrix<T>& obj)
{
  os << std::setw(3) << std::setprecision(16) << "OM: / " << obj[0] << ", " << obj[1] << ", " << obj[2] << " \\\n";
  os << std::setw(3) << std::setprecision(16) << "OM: | " << obj[3] << ", " << obj[4] << ", " << obj[5] << " |\n";
  os << std::setw(3) << std::setprecision(16) << "OM: \\ " << obj[6] << ", " << obj[7] << ", " << obj[8] << " /";
  return os;
}

} // namespace ebsdlib
