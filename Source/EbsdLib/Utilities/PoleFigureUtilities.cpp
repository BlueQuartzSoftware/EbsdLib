/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
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

#include "PoleFigureUtilities.h"

#include <fstream>
#include <sstream>

#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/LaueOps/OrthoRhombicOps.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ModifiedLambertProjection.h"

#define WRITE_XYZ_SPHERE_COORD_VTK 0
#define WRITE_LAMBERT_SQUARES 0

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
PoleFigureUtilities::PoleFigureUtilities() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
PoleFigureUtilities::~PoleFigureUtilities() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int writeVtkFile(EbsdLib::FloatArrayType* xyz, const std::string& filename)
{

  std::ofstream out(filename, std::ios_base::out);
  if(!out.is_open())
  {
    return -100;
  }

  out << "# vtk DataFile Version 2.0\n";
  out << "data set from DREAM3D\n";
  out << "ASCII";
  out << "\n";

  out << "DATASET UNSTRUCTURED_GRID\nPOINTS " << xyz->getNumberOfTuples() << " float\n";
  size_t count = xyz->getNumberOfTuples();
  for(size_t i = 0; i < count; ++i)
  {
    out << xyz->getComponent(i, 0) << " " << xyz->getComponent(i, 1) << " " << xyz->getComponent(i, 2) << "\n";
  }
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::UInt8ArrayType::Pointer PoleFigureUtilities::CreateColorImage(EbsdLib::DoubleArrayType* data, int width, int height, int nColors, const std::string& name, double min, double max)
{
  std::vector<size_t> dims(1, 4);
  EbsdLib::UInt8ArrayType::Pointer image = EbsdLib::UInt8ArrayType::CreateArray(static_cast<size_t>(width * height), dims, name, true);
  PoleFigureConfiguration_t config;
  config.imageDim = width;
  config.numColors = nColors;
  config.eulers = nullptr;
  config.minScale = min;
  config.maxScale = max;
  config.sphereRadius = 1.0;
  config.discrete = false;
  config.discreteHeatMap = false;
  PoleFigureUtilities::CreateColorImage(data, config, image.get());
  return image;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void PoleFigureUtilities::CreateColorImage(EbsdLib::DoubleArrayType* data, PoleFigureConfiguration_t& config, EbsdLib::UInt8ArrayType* image)
{
  int width = config.imageDim;
  int height = config.imageDim;

  int halfWidth = width / 2;
  int halfHeight = height / 2;

  float xres = 2.0f / static_cast<float>(width);
  float yres = 2.0f / static_cast<float>(height);
  float xtmp, ytmp;

  float max = static_cast<float>(config.maxScale);
  float min = static_cast<float>(config.minScale);

  // Initialize the image with all zeros
  image->initializeWithZeros();
  uint32_t* rgbaPtr = reinterpret_cast<uint32_t*>(image->getPointer(0));

  int numColors = config.numColors;
  std::vector<float> colors(numColors * 3, 0.0f);
  EbsdColorTable::GetColorTable(config.numColors, colors);

  float r = 0.0f, g = 0.0f, b = 0.0f;

  double* dataPtr = data->getPointer(0);
  size_t idx = 0;
  double value;
  int bin;
  for(int64_t y = 0; y < height; y++)
  {
    for(int64_t x = 0; x < width; x++)
    {
      xtmp = float(x - halfWidth) * xres + (xres * 0.5f);
      ytmp = float(y - halfHeight) * yres + (yres * 0.5f);
      idx = (width * y) + x;
      if((xtmp * xtmp + ytmp * ytmp) <= 1.0) // Inside the circle
      {
        value = dataPtr[y * width + x];
        value = (value - min) / (max - min);
        bin = int(value * numColors);
        if(bin > numColors - 1)
        {
          bin = numColors - 1;
        }
        if(bin < 0 || bin >= colors.size())
        {
          r = 0x00;
          b = 0x00;
          g = 0x00;
        }
        else if(!config.discreteHeatMap && config.discrete)
        {
          float frgb = 1.0f;
          if(value > 0.0)
          {
            frgb = 0.0f;
          }
          r = frgb;
          b = frgb;
          g = frgb;
        }
        else
        {
          r = colors[3 * bin];
          g = colors[3 * bin + 1];
          b = colors[3 * bin + 2];
        }

        rgbaPtr[idx] = EbsdLib::RgbColor::dRgb(static_cast<int>(r * 255.0f), static_cast<int>(g * 255.0f), static_cast<int>(b * 255.0f), 255);
      }
      else // Outside the Circle - Set pixel to White
      {
        rgbaPtr[idx] = 0xFFFFFFFF; // White
      }
    }
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void PoleFigureUtilities::GenerateHexPoleFigures(EbsdLib::FloatArrayType* eulers, int lambertDimension, int poleFigureDim, EbsdLib::DoubleArrayType::Pointer& intensity0001,
                                                 EbsdLib::DoubleArrayType::Pointer& intensity1010, EbsdLib::DoubleArrayType::Pointer& intensity1120)
{
  size_t numOrientations = eulers->getNumberOfTuples();

  // Create an Array to hold the XYZ Coordinates which are the coords on the sphere.
  // this is size for HEX ONLY, <0001> Family
  std::vector<size_t> dims(1, 3);
  EbsdLib::FloatArrayType::Pointer xyz0001 = EbsdLib::FloatArrayType::CreateArray(numOrientations * 2, dims, "TEMP_<0001>_xyzCoords", true);
  // this is size for HEX ONLY, <10-10> Family
  EbsdLib::FloatArrayType::Pointer xyz1010 = EbsdLib::FloatArrayType::CreateArray(numOrientations * 6, dims, "TEMP_<1010>_xyzCoords", true);
  // this is size for HEX ONLY, <11-20> Family
  EbsdLib::FloatArrayType::Pointer xyz1120 = EbsdLib::FloatArrayType::CreateArray(numOrientations * 6, dims, "TEMP_<1120>_xyzCoords", true);

  float sphereRadius = 1.0f;

  // Generate the coords on the sphere
  HexagonalOps ops;
  ops.generateSphereCoordsFromEulers(eulers, xyz0001.get(), xyz1010.get(), xyz1120.get());
#if WRITE_XYZ_SPHERE_COORD_VTK
  writeVtkFile(xyz0001.get(), "c:/Users/GroebeMA/Desktop/Sphere_XYZ_FROM_EULER_0001.vtk");
  writeVtkFile(xyz1010.get(), "c:/Users/GroebeMA/Desktop/Sphere_XYZ_FROM_EULER_1010.vtk");
  writeVtkFile(xyz1120.get(), "c:/Users/GroebeMA/Desktop/Sphere_XYZ_FROM_EULER_1120.vtk");
#endif

  // Generate the modified Lambert projection images (Squares, 2 of them, 1 for northern hemisphere, 1 for southern hemisphere
  ModifiedLambertProjection::Pointer lambert = ModifiedLambertProjection::LambertBallToSquare(xyz0001.get(), lambertDimension, sphereRadius);
  // Now create the intensity image that will become the actual Pole figure image
  EbsdLib::DoubleArrayType::Pointer poleFigurePtr = lambert->createStereographicProjection(poleFigureDim);
  poleFigurePtr->setName("PoleFigure_<0001>");
  intensity0001.swap(poleFigurePtr);

#if WRITE_LAMBERT_SQUARES
  size_t dims[3] = {lambert->getDimension(), lambert->getDimension(), 1};
  FloatVec3Type res = {lambert->getStepSize(), lambert->getStepSize(), lambert->getStepSize()};
  EbsdLib::DoubleArrayType::Pointer north = lambert->getNorthSquare();
  EbsdLib::DoubleArrayType::Pointer south = lambert->getSouthSquare();
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/ModifiedLambert_North.vtk", north.get(), dims, res, "double", true);
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/ModifiedLambert_South.vtk", south.get(), dims, res, "double", true);
#endif

  // Generate the <011> pole figure which will generate a new set of Lambert Squares
  lambert = ModifiedLambertProjection::LambertBallToSquare(xyz1010.get(), lambertDimension, sphereRadius);
  poleFigurePtr = lambert->createStereographicProjection(poleFigureDim);
  poleFigurePtr->setName("PoleFigure_<1010>");
  intensity1010.swap(poleFigurePtr);

  // Generate the <111> pole figure which will generate a new set of Lambert Squares
  lambert = ModifiedLambertProjection::LambertBallToSquare(xyz1120.get(), lambertDimension, sphereRadius);
  poleFigurePtr = lambert->createStereographicProjection(poleFigureDim);
  poleFigurePtr->setName("PoleFigure_<1120>");
  intensity1120.swap(poleFigurePtr);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void PoleFigureUtilities::GenerateOrthoPoleFigures(EbsdLib::FloatArrayType* eulers, int lambertDimension, int poleFigureDim, EbsdLib::DoubleArrayType::Pointer& intensity100,
                                                   EbsdLib::DoubleArrayType::Pointer& intensity010, EbsdLib::DoubleArrayType::Pointer& intensity001)
{
  size_t numOrientations = eulers->getNumberOfTuples();

  // Create an Array to hold the XYZ Coordinates which are the coords on the sphere.
  // this is size for ORTHO ONLY, <100> Family
  std::vector<size_t> dims(1, 3);
  EbsdLib::FloatArrayType::Pointer xyz100 = EbsdLib::FloatArrayType::CreateArray(numOrientations * 2, dims, "TEMP_<100>_xyzCoords", true);
  // this is size for ORTHO ONLY, <010> Family
  EbsdLib::FloatArrayType::Pointer xyz010 = EbsdLib::FloatArrayType::CreateArray(numOrientations * 2, dims, "TEMP_<010>_xyzCoords", true);
  // this is size for ORTHO ONLY, <001> Family
  EbsdLib::FloatArrayType::Pointer xyz001 = EbsdLib::FloatArrayType::CreateArray(numOrientations * 2, dims, "TEMP_<001>_xyzCoords", true);

  float sphereRadius = 1.0f;

  // Generate the coords on the sphere
  OrthoRhombicOps ops;
  ops.generateSphereCoordsFromEulers(eulers, xyz100.get(), xyz010.get(), xyz001.get());
#if WRITE_XYZ_SPHERE_COORD_VTK
  writeVtkFile(xyz100.get(), "c:/Users/GroebeMA/Desktop/Sphere_XYZ_FROM_EULER_100.vtk");
  writeVtkFile(xyz010.get(), "c:/Users/GroebeMA/Desktop/Sphere_XYZ_FROM_EULER_010.vtk");
  writeVtkFile(xyz001.get(), "c:/Users/GroebeMA/Desktop/Sphere_XYZ_FROM_EULER_001.vtk");
#endif

  // Generate the modified Lambert projection images (Squares, 2 of them, 1 for northern hemisphere, 1 for southern hemisphere
  ModifiedLambertProjection::Pointer lambert = ModifiedLambertProjection::LambertBallToSquare(xyz100.get(), lambertDimension, sphereRadius);
  // Now create the intensity image that will become the actual Pole figure image
  EbsdLib::DoubleArrayType::Pointer poleFigurePtr = lambert->createStereographicProjection(poleFigureDim);
  poleFigurePtr->setName("PoleFigure_<100>");
  intensity100.swap(poleFigurePtr);

#if WRITE_LAMBERT_SQUARES
  size_t dims[3] = {lambert->getDimension(), lambert->getDimension(), 1};
  FloatVec3Type res = {lambert->getStepSize(), lambert->getStepSize(), lambert->getStepSize()};
  EbsdLib::DoubleArrayType::Pointer north = lambert->getNorthSquare();
  EbsdLib::DoubleArrayType::Pointer south = lambert->getSouthSquare();
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/ModifiedLambert_North.vtk", north.get(), dims, res, "double", true);
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/ModifiedLambert_South.vtk", south.get(), dims, res, "double", true);
#endif

  // Generate the <011> pole figure which will generate a new set of Lambert Squares
  lambert = ModifiedLambertProjection::LambertBallToSquare(xyz010.get(), lambertDimension, sphereRadius);
  poleFigurePtr = lambert->createStereographicProjection(poleFigureDim);
  poleFigurePtr->setName("PoleFigure_<010>");
  intensity010.swap(poleFigurePtr);

  // Generate the <111> pole figure which will generate a new set of Lambert Squares
  lambert = ModifiedLambertProjection::LambertBallToSquare(xyz001.get(), lambertDimension, sphereRadius);
  poleFigurePtr = lambert->createStereographicProjection(poleFigureDim);
  poleFigurePtr->setName("PoleFigure_<001>");
  intensity001.swap(poleFigurePtr);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
GeneratePoleFigureRgbaImageImpl::GeneratePoleFigureRgbaImageImpl() = default;

GeneratePoleFigureRgbaImageImpl::GeneratePoleFigureRgbaImageImpl(EbsdLib::DoubleArrayType* intensity, PoleFigureConfiguration_t* config, EbsdLib::UInt8ArrayType* rgba)
: m_Intensity(intensity)
, m_Config(config)
, m_Rgba(rgba)
{
}
GeneratePoleFigureRgbaImageImpl::~GeneratePoleFigureRgbaImageImpl() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void GeneratePoleFigureRgbaImageImpl::operator()() const
{
  PoleFigureUtilities::CreateColorImage(m_Intensity, *m_Config, m_Rgba);
}
