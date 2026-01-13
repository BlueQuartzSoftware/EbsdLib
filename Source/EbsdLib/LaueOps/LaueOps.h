/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/Matrix3X3.hpp"
#include "EbsdLib/Orientation/AxisAngle.hpp"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

namespace ebsdlib
{
/*
 * @class LaueOps LaueOps.h OrientationLib/LaueOps/LaueOps.h
 * @brief
 */
class EbsdLib_EXPORT LaueOps
{
public:
  using Self = LaueOps;
  using Pointer = std::shared_ptr<Self>;
  using ConstPointer = std::shared_ptr<const Self>;
  using WeakPointer = std::weak_ptr<Self>;
  using ConstWeakPointer = std::weak_ptr<const Self>;
  static Pointer NullPointer();

  /**
   * @brief Returns the name of the class for LaueOps
   */
  virtual std::string getNameOfClass() const;
  /**
   * @brief Returns the name of the class for LaueOps
   */
  static std::string ClassName();

  virtual ~LaueOps();

  /**
   * @brief GetAllOrientationOps This method returns a vector of each type of LaueOps placed such that the
   * index into the vector is the value of the constant at CrystalStructure::***
   * @return Vector of LaueOps subclasses.
   */
  static std::vector<LaueOps::Pointer> GetAllOrientationOps();

  /**
   * @brief GetOrientationOpsFromSpaceGroupNumber
   * @param sgNumber
   * @return
   */
  static Pointer GetOrientationOpsFromSpaceGroupNumber(size_t sgNumber);

  /**
   * @brief GetLaueNames Returns the names of the Laue Classes
   * @return
   */
  static std::vector<std::string> GetLaueNames();

  /**
   * @brief getODFSize Returns the number of elements in the ODF array
   * @return
   */
  virtual size_t getODFSize() const = 0;

  /**
   * @brief getNumSymmetry Returns the internal variables for k_SymSize0, k_SymSize1, k_SymSize2
   * @return
   */
  virtual std::array<int32_t, 3> getNumSymmetry() const = 0;

  /**
   * @brief Returns the number of bins for an MDF Plot assuming 5 degree increments
   * @return
   */
  virtual int getMdfPlotBins() const = 0;

  /**
   * @brief getHasInversion Returns a bool whether the symmetry class is centro-symmetric
   * @return
   */
  virtual bool getHasInversion() const = 0;

  /**
   * @brief getMDFSize Returns the number of elements in the MDF Array
   * @return
   */
  virtual size_t getMDFSize() const = 0;

  /**
   * @brief getNumSymOps Returns the number of symmetry operators
   * @return
   */
  virtual size_t getNumSymOps() const = 0;

  /**
   * @brief getNumRodriguesSymOps Returns the number of Rodrigues symmetry operators
   * @return
   */
  virtual size_t getNumRodriguesSymOps() const = 0;

  /**
   * @brief getSymmetryName Returns the name of the symmetry
   * @return
   */
  virtual std::string getSymmetryName() const = 0;

  /**
   * @brief Returns the Rotation Point Group for the LaueClass.
   * @return
   */
  virtual std::string getRotationPointGroup() const = 0;

  /**
   * @brief returns the value of the crystallographic point group
   * @return
   */
  virtual int getPointGroup() const = 0;

  /**
   * @brief Returns the number of bins in each of the 3 dimensions
   * @return
   */
  virtual std::array<size_t, 3> getOdfNumBins() const = 0;

  /**
   * @brief calculateMisorientation Finds the misorientation between 2 quaternions and returns the result as an Axis Angle value
   * @param q1 Input Quaternion
   * @param q2 Input Quaternion
   * @return Axis Angle Representation
   */
  virtual AxisAngleDType calculateMisorientation(const QuatD& q1, const QuatD& q2) const = 0;

  /**
   * @brief calculateMisorientation Finds the misorientation between 2 quaternions and returns the result as an Axis Angle value
   * @param q1 Input Quaternion
   * @param q2 Input Quaternion
   * @return Axis Angle Representation
   */
  // virtual AxisAngleDType calculateMisorientation(const QuatF& q1, const QuatF& q2) const = 0;

  /**
   * @brief getQuatSymOp Returns the symmetry operator at index i
   * @param i The index into the Symmetry operators array
   * @return The quaternion symmetry operator
   */
  virtual QuatD getQuatSymOp(size_t i) const = 0;

  /**
   * @brief getRodSymOp Returns a Rodrigues vector based on the symmetry operator at index i
   * @param i Index of the symmetry operator
   * @param r Pointer to store the Rodrigues vector into.
   */
  virtual RodriguesDType getRodSymOp(size_t i) const = 0;

  /**
   * @brief Retrieves a specific Symmetry Operator for a giving index
   * @param i The index from the Symmetry Operator Array to retrieve
   * @param g The g matrix
   * @return void or a Matrix3X3 object.
   */
  virtual Matrix3X3F getMatSymOpF(size_t i) const = 0;
  virtual Matrix3X3D getMatSymOpD(size_t i) const = 0;

  /**
   * @brief getODFFZRod
   * @param rod
   * @return
   */
  virtual RodriguesDType getODFFZRod(const RodriguesDType& rod) const = 0;

  /**
   * @brief getMDFFZRod
   * @param rod
   * @return
   */
  virtual RodriguesDType getMDFFZRod(const RodriguesDType& rod) const = 0;

  virtual QuatD getNearestQuat(const QuatD& q1, const QuatD& q2) const = 0;
  virtual QuatF getNearestQuat(const QuatF& q1f, const QuatF& q2f) const = 0;

  /**
   * @brief getFZQuat Returns a Quaternion that lies in the Fundamental Zone (FZ)
   * @param qr Input Quaternion
   * @return
   */
  virtual QuatD getFZQuat(const QuatD& qr) const = 0;

  /**
   * @brief getMisoBin Returns the misorientation bin that the input Rodrigues vector lies in.
   * @param rod
   * @return
   */
  virtual int getMisoBin(const RodriguesDType& rod) const = 0;

  virtual bool inUnitTriangle(double eta, double chi) const = 0;

  virtual EulerDType determineEulerAngles(double random[3], int choose) const = 0;

  virtual EulerDType randomizeEulerAngles(const EulerDType& euler) const = 0;

  virtual size_t getRandomSymmetryOperatorIndex(int numSymOps) const;

  virtual RodriguesDType determineRodriguesVector(double random[3], int choose) const = 0;

  virtual int getOdfBin(const RodriguesDType& rod) const = 0;

  virtual void getSchmidFactorAndSS(double load[3], double& schmidfactor, double angleComps[2], int& slipsys) const = 0;

  virtual void getSchmidFactorAndSS(double load[3], double plane[3], double direction[3], double& schmidfactor, double angleComps[2], int& slipsys) const = 0;

  virtual double getmPrime(const QuatD& q1, const QuatD& q2, double LD[3]) const = 0;

  virtual double getF1(const QuatD& q1, const QuatD& q2, double LD[3], bool maxSF) const = 0;

  virtual double getF1spt(const QuatD& q1, const QuatD& q2, double LD[3], bool maxSF) const = 0;

  virtual double getF7(const QuatD& q1, const QuatD& q2, double LD[3], bool maxSF) const = 0;

  virtual void generateSphereCoordsFromEulers(FloatArrayType* eulers, FloatArrayType* c1, FloatArrayType* c2, FloatArrayType* c3) const = 0;

  static void RodriguesComposition(RodriguesDType sigma, RodriguesDType& rod);

  /**
   * @brief
   * @param eta Optional input value only needed for the "Cubic" Laue classes
   * @return Triplet of etaMin, etaMax, chiMax
   */
  virtual std::array<double, 3> getIpfColorAngleLimits(double eta) const = 0;

  /**
   * @brief generateIPFColor Generates an ARGB Color from an Euler Angle and Reference Direction
   * @param eulers Pointer to the 3 component Euler Angle
   * @param refDir Pointer to the 3 Component Reference Direction
   * @param convertDegrees Are the input angles in Degrees
   * @return rgb [output] The pointer to store the RGB value
   */
  virtual Rgb generateIPFColor(double* eulers, double* refDir, bool convertDegrees) const = 0;

  /**
   * @brief generateIPFColor Generates an ARGB Color from an Euler Angle and Reference Direction
   * @param e0 First component of the Euler Angle
   * @param e1 Second component of the Euler Angle
   * @param e2 Third component of the Euler Angle
   * @param dir0 First component of the Reference Direction
   * @param dir1 Second component of the Reference Direction
   * @param dir2 Third component of the Reference Direction
   * @param convertDegrees Are the input angles in Degrees
   * @return rgb [output] The pointer to store the RGB value
   */
  virtual Rgb generateIPFColor(double e0, double e1, double e2, double dir0, double dir1, double dir2, bool convertDegrees) const = 0;

  /**
   * @brief generateRodriguesColor Generates an RGB Color from a Rodrigues Vector
   * @param r1 First component of the Rodrigues Vector
   * @param r2 Second component of the Rodrigues Vector
   * @param r3 Third component of the Rodrigues Vector
   * @return rgb [output] The pointer to store the RGB value
   */
  virtual Rgb generateRodriguesColor(double r1, double r2, double r3) const = 0;

  /**
   * @brief generateMisorientationColor Generates a color based on the method developed by C. Schuh and S. Patala.
   * @param q A Quaternion representing the crystal direction
   * @param refFrame A Quaternion representing the sample reference direction
   * @return A Rgb value
   */
  virtual Rgb generateMisorientationColor(const QuatD& q, const QuatD& refFrame) const;

  /**
   * @brief generatePoleFigure This method will generate a number of pole figures for this crystal symmetry and the Euler
   * angles that are passed in.
   * @param config The Pole Figure configuration struct
   * @return A std::vector of UInt8ArrayType pointers where each one represents a 2D RGB array that can be used to initialize
   * an image object from other libraries and written out to disk.
   */
  virtual std::vector<UInt8ArrayType::Pointer> generatePoleFigure(PoleFigureConfiguration_t& config) const = 0;

  /**
   * @brief Returns the names for each of the three standard pole figures that are generated. For example
   *<001>, <011> and <111> for a cubic system
   */
  virtual std::array<std::string, 3> getDefaultPoleFigureNames() const = 0;

  /**
   * @brief generateStandardTriangle Generates an RGBA array that is a color "Standard" IPF Triangle Legend used for IPF Color Maps.
   * @return
   */
  virtual UInt8ArrayType::Pointer generateIPFTriangleLegend(int imageDim, bool generateEntirePlane) const = 0;

  enum class FZType : int32_t
  {
    Anorthic = 0, // Triclinic
    Cyclic = 1,
    Dihedral = 2,
    Tetrahedral = 3,
    Octahedral = 4
  };

  enum class AxisOrderingType : int32_t
  {
    None = 0,
    TwoFold = 2,
    ThreeFold = 3,
    FourFold = 4,
    SixFold = 6,
    EightFold = 8,
    TenFold = 10,
    TwelveFold = 12
  };

  /**
   * @brief Returns the given enumeration value as a string
   * @param value
   * @return
   */
  static std::string FZTypeToString(FZType value);

  /**
   * @brief Returns the given enumeration value as a string
   * @param value
   * @return
   */
  static std::string AxisOrderingTypeToString(AxisOrderingType value);

  /**
   * @brief  Returns the Fundamental Zone type
   * @return
   */
  FZType getFZType() const;

  /**
   * @brief Returns the Axis Ordering Type
   * @return
   */
  AxisOrderingType getAxisOrderingType() const;

  /**
   * @brief Determines if the given 4 component Rodrigues Vector is inside a cyclic fundamental zone.
   *
   * Be sure that the AxisOrderingType is NOT NONE otherwise undefined behavior will occur
   * @param rod 4 Component Rodrigues Vector
   * @param fzType
   * @param fzType
   * @param order The Axis Ordering Type
   * @return
   */
  static bool InsideCyclicFZ(const RodriguesDType& rod, FZType fzType, AxisOrderingType order);

  /**
   * @brief Determines if the given 4 component Rodrigues Vector is inside a dihedral fundamental zone
   * @param rod 4 Component Rodrigues Vector
   * @param order The Axis Ordering Type
   * @return
   */
  static bool InsideDihedralFZ(const RodriguesDType& rod, AxisOrderingType order);

  /**
   * @brief Determines if the given 4 component Rodrigues Vector is inside a cubic fundamental zone
   * @param rod 4 Component Rodrigues Vector
   * @param fzType The Fundamental Zone type
   * @return
   */
  static bool InsideCubicFZ(const RodriguesDType& rod, FZType fzType);

  /**
   * @brief Determines if the given 4 component Rodrigues Vector is inside the fundamental zone
   * @param rod 4 Component Rodrigues Vector
   * @param fzType The Fundamental Zone type
   * @param order The Axis Ordering Type
   * @return
   */
  static bool IsInsideFZ(const RodriguesDType& rod, FZType fzType, AxisOrderingType order);

  /**
   * @brief Determines if the given Quaternion Vector is inside the fundamental zone.
   *
   * As part of this check, the passed in Quaternion is normalized before calling the Rodrigues
   * version of this function.
   * @param quat Input Quaternion
   * @param fzType The Fundamental Zone type
   * @param order The Axis Ordering Type
   * @return
   */
  static bool IsInsideFZ(const QuatD& quat, FZType fzType, AxisOrderingType order);

  /**
   * @brief Returns if the given Quaternion is within the Rodrigues Fundamental Zone (RFZ)
   * @param quat Input Quaternion
   * @return
   */
  virtual bool isInsideFZ(const QuatD& quat) const = 0;

  /**
   * @brief Returns if the given Rodrigues vector is within the Rodrigues Fundamental Zone (RFZ)
   * @param rod Input Rodrigues Vector
   * @return
   */
  virtual bool isInsideFZ(const RodriguesDType& rod) const = 0;

protected:
  LaueOps();

  /**
   * @brief calculateMisorientationInternal
   * @param quatsym The Symmetry Quarternion from the specific Laue class
   * @param q1 Input Quaternion 1
   * @param q2 Input Quaternion 2
   * @return Returns Axis-Angle <XYZ>W form.
   */
  virtual AxisAngleDType calculateMisorientationInternal(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const;

  /**
   * @brief
   * @param rodsym
   * @param rod
   * @return
   */
  RodriguesDType _calcRodNearestOrigin(const RodriguesDType& rod) const;

  /**
   * @brief
   * @param quatsym
   * @param q1
   * @param q2
   * @return
   */
  QuatD _calcNearestQuat(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const;

  /**
   * @brief
   * @param dim
   * @param bins
   * @param step
   * @param homochoric
   * @return
   */
  int _calcMisoBin(double dim[3], double bins[3], double step[3], const HomochoricDType& homochoric) const;

  /**
   * @brief
   * @param random
   * @param init
   * @param step
   * @param phi
   * @param r1
   * @param r2
   * @param r3
   */
  void _calcDetermineHomochoricValues(double random[3], double init[3], double step[3], int32_t phi[3], double& r1, double& r2, double& r3) const;

  /**
   * @brief
   * @param dim
   * @param bins
   * @param step
   * @param homochoric
   * @return
   */
  int _calcODFBin(double dim[3], double bins[3], double step[3], const HomochoricDType& homochoric) const;

  /**
   * @brief Generates an IPF Color for a given Euler and Reference Direction. This should be called from the subclass so the
   * specific etaMin, etaMax and ChiMax can be passed in.
   * @param eulers
   * @param refDir
   * @param deg2Rad
   * @return
   */
  Rgb computeIPFColor(double* eulers, double* refDir, bool degToRad) const;

  /**
   * @brief Converts in input Quaternion into a version that is inside the fundamental zone.
   *
   * As part of the algorithm, the Quaternion is normalized before any other orientation
   * transformations are performed. If the scalar part of the Quaternion is negative, then
   * the entire Quaternion is negated.
   * @param quatsym The symmetry operators
   * @param qr The input quaternion. Calling code does not need to normalize this first.
   * @param fzType The type of fundamental zone
   * @param order The Axis orider of the symmetry
   * @return
   */
  static QuatD ConvertToFZ(const std::vector<QuatD>& quatsym, const QuatD& qr, FZType fzType, AxisOrderingType order);

public:
  LaueOps(const LaueOps&) = delete;            // Copy Constructor Not Implemented
  LaueOps(LaueOps&&) = delete;                 // Move Constructor Not Implemented
  LaueOps& operator=(const LaueOps&) = delete; // Copy Assignment Not Implemented
  LaueOps& operator=(LaueOps&&) = delete;      // Move Assignment Not Implemented
};

namespace laue_ops
{

constexpr std::array<LaueOps::FZType, 32> FZtarray = {
    LaueOps::FZType::Anorthic, LaueOps::FZType::Anorthic,    LaueOps::FZType::Cyclic,      LaueOps::FZType::Cyclic,     LaueOps::FZType::Cyclic,   // 0-4
    LaueOps::FZType::Dihedral, LaueOps::FZType::Dihedral,    LaueOps::FZType::Dihedral,    LaueOps::FZType::Cyclic,     LaueOps::FZType::Cyclic,   // 5-9
    LaueOps::FZType::Cyclic,   LaueOps::FZType::Dihedral,    LaueOps::FZType::Dihedral,    LaueOps::FZType::Dihedral,   LaueOps::FZType::Dihedral, // 10-14
    LaueOps::FZType::Cyclic,   LaueOps::FZType::Cyclic,      LaueOps::FZType::Dihedral,    LaueOps::FZType::Dihedral,   LaueOps::FZType::Dihedral, // 15-19
    LaueOps::FZType::Cyclic,   LaueOps::FZType::Cyclic,      LaueOps::FZType::Cyclic,      LaueOps::FZType::Dihedral,   LaueOps::FZType::Dihedral,    LaueOps::FZType::Dihedral,
    LaueOps::FZType::Dihedral, LaueOps::FZType::Tetrahedral, LaueOps::FZType::Tetrahedral, LaueOps::FZType::Octahedral, LaueOps::FZType::Tetrahedral, LaueOps::FZType::Octahedral};

constexpr std::array<LaueOps::AxisOrderingType, 32> FZoarray = {LaueOps::AxisOrderingType::None,      LaueOps::AxisOrderingType::None,      LaueOps::AxisOrderingType::TwoFold,
                                                                LaueOps::AxisOrderingType::TwoFold,   LaueOps::AxisOrderingType::TwoFold, // 0-4
                                                                LaueOps::AxisOrderingType::TwoFold,   LaueOps::AxisOrderingType::TwoFold,   LaueOps::AxisOrderingType::TwoFold,
                                                                LaueOps::AxisOrderingType::FourFold,  LaueOps::AxisOrderingType::FourFold, // 5-9
                                                                LaueOps::AxisOrderingType::FourFold,  LaueOps::AxisOrderingType::FourFold,  LaueOps::AxisOrderingType::FourFold,
                                                                LaueOps::AxisOrderingType::FourFold,  LaueOps::AxisOrderingType::FourFold, // 10-14
                                                                LaueOps::AxisOrderingType::ThreeFold, LaueOps::AxisOrderingType::ThreeFold, LaueOps::AxisOrderingType::ThreeFold,
                                                                LaueOps::AxisOrderingType::ThreeFold, LaueOps::AxisOrderingType::ThreeFold, // 15-19
                                                                LaueOps::AxisOrderingType::SixFold,   LaueOps::AxisOrderingType::SixFold,   LaueOps::AxisOrderingType::SixFold,
                                                                LaueOps::AxisOrderingType::SixFold,   LaueOps::AxisOrderingType::SixFold,   LaueOps::AxisOrderingType::SixFold,
                                                                LaueOps::AxisOrderingType::SixFold,   LaueOps::AxisOrderingType::None,      LaueOps::AxisOrderingType::None,
                                                                LaueOps::AxisOrderingType::None,      LaueOps::AxisOrderingType::None,      LaueOps::AxisOrderingType::None};

} // namespace laue_ops
} // namespace ebsdlib
/*
 * @brief Master Table of Crystallographic Information
 * This is formatted as a MarkDown with LaTeX formatting

|  # | Point Group (H–M) | Rotation Point Group | Space Group No(s). | Schoenflies   | Crystal system | Laue class  | Laue Ops |
| -: | ----------------- | -------------------- | ------------------ | ------------- | -------------- | ----------- | ---------|
|  1 | 1                 | 1                    | 1                  | C₁            | Triclinic      | (\bar{1})   | TriclinicOps |
|  2 | (\bar{1})         | 1                    | 2                  | C(_i)         | Triclinic      | (\bar{1})   |
|  3 | 2                 | 2                    | 3–5                | C₂            | Monoclinic     | 2/m         |
|  4 | m                 | 1                    | 6–9                | C(_s)         | Monoclinic     | 2/m         |
|  5 | 2/m               | 2                    | 10–15              | C(_{2h})      | Monoclinic     | 2/m         | MonoclinicOps |
|  6 | 222               | 222                  | 16–24              | D₂            | Orthorhombic   | mmm         |
|  7 | mm2               | 2                    | 25–46              | C(_{2v})      | Orthorhombic   | mmm         |
|  8 | mmm               | 222                  | 47–74              | D(_{2h})      | Orthorhombic   | mmm         | OrthorhombicOps |
|  9 | 4                 | 4                    | 75–80              | C₄            | Tetragonal     | 4/m         |
| 10 | (\bar{4})         | 2                    | 81–82              | S₄            | Tetragonal     | 4/m         |
| 11 | 4/m               | 4                    | 83–88              | C(_{4h})      | Tetragonal     | 4/m         | TetragonalLowOps |
| 12 | 422               | 422                  | 89–98              | D₄            | Tetragonal     | 4/mmm       |
| 13 | 4mm               | 4                    | 99–110             | C(_{4v})      | Tetragonal     | 4/mmm       |
| 14 | (\bar{4}2m)       | 222                  | 111–122            | D(_{2d})      | Tetragonal     | 4/mmm       |
| 15 | 4/mmm             | 422                  | 123–142            | D(_{4h})      | Tetragonal     | 4/mmm       | TetragonalOps |
| 16 | 3                 | 3                    | 143–146            | C₃            | Trigonal       | (\bar{3})   |
| 17 | (\bar{3})         | 3                    | 147–148            | C(_{3i}) (S₆) | Trigonal       | (\bar{3})   | TrigonalLowOps |
| 18 | 32                | 32                   | 149–155            | D₃            | Trigonal       | (\bar{3}m)  |
| 19 | 3m                | 3                    | 156–161            | C(_{3v})      | Trigonal       | (\bar{3}m)  |
| 20 | (\bar{3}m)        | 32                   | 162–167            | D(_{3d})      | Trigonal       | (\bar{3}m)  | TrigonalOps |
| 21 | 6                 | 6                    | 168–173            | C₆            | Hexagonal      | 6/m         |
| 22 | (\bar{6})         | 3                    | 174                | C(_{3h})      | Hexagonal      | 6/m         |
| 23 | 6/m               | 6                    | 175–176            | C(_{6h})      | Hexagonal      | 6/m         | HexagonalLowOps |
| 24 | 622               | 622                  | 177–182            | D₆            | Hexagonal      | 6/mmm       |
| 25 | 6mm               | 6                    | 183–186            | C(_{6v})      | Hexagonal      | 6/mmm       |
| 26 | (\bar{6}m2)       | 32                   | 187–190            | D(_{3h})      | Hexagonal      | 6/mmm       |
| 27 | 6/mmm             | 622                  | 191–194            | D(_{6h})      | Hexagonal      | 6/mmm       | HexagonalOps |
| 28 | 23                | 23                   | 195–199            | T             | Cubic          | m(\bar{3})  |
| 29 | m(\bar{3})        | 23                   | 200–206            | T(_h)         | Cubic          | m(\bar{3})  | CubicLowOps |
| 30 | 432               | 432                  | 207–214            | O             | Cubic          | m(\bar{3})m |
| 31 | (\bar{4}3m)       | 23                   | 215–220            | T(_d)         | Cubic          | m(\bar{3})m |
| 32 | m(\bar{3})m       | 432                  | 221–230            | O(_h)         | Cubic          | m(\bar{3})m | CubicOps |


*/
