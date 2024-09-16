#pragma once

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Math/Matrix3X1.hpp"

#include <canvas_ity.hpp>

namespace EbsdLib
{

using Point3DType = EbsdLib::Matrix3X1<float>;

/**
 * @brief
 * @param context
 * @param figureSubtitle
 * @param textOrigin
 * @param fontPtSize
 */
EbsdLib_EXPORT void WriteText(canvas_ity::canvas& context, const std::string& figureSubtitle, std::array<float, 2> textOrigin, int fontPtSize);

/**
* @brief
 * @param context
 * @param xStart
 * @param yStart
 * @param xEnd
 * @param yEnd
 */
EbsdLib_EXPORT void DrawLine(canvas_ity::canvas& context, float xStart, float yStart, float xEnd, float yEnd);

/**
 * @brief Function to generate points on a unit circle on the plane
 * @param direction  This should be in something like a crystallographic direction [110];
 * @param num_points The number of points to generate
 * @return
 */
EbsdLib_EXPORT std::vector<Point3DType> GeneratePointsOnUnitCircle(const Point3DType& direction, int num_points);

/**
 * @brief
 * @param image
 * @param width
 * @param height
 * @return
 */
EbsdLib_EXPORT EbsdLib::UInt8ArrayType::Pointer DrawStandardCubicProjection(EbsdLib::UInt8ArrayType::Pointer image, int width, int height);

/**
 * @brief
 * @param image
 * @param pageWidth
 * @param pageHeight
 * @return
 */
EbsdLib_EXPORT EbsdLib::UInt8ArrayType::Pointer DrawStandardHexagonalProjection(EbsdLib::UInt8ArrayType::Pointer image, int pageWidth, int pageHeight);


/**
* @brief
 * @param context
 * @param directions
 * @param numPoints
 * @param halfWidth
 * @param figureOrigin
 */
EbsdLib_EXPORT void DrawStereographicLines(canvas_ity::canvas& context, const std::vector<EbsdLib::Point3DType>& directions, int numPoints, int halfWidth,
                                      std::array<float, 2> figureOrigin);

// -----------------------------------------------------------------------------
template <typename T>
typename EbsdDataArray<T>::Pointer MirrorImage(EbsdDataArray<T>* src, int imageDim)
{
  typename EbsdDataArray<T>::Pointer converted = EbsdDataArray<T>::CreateArray(imageDim * imageDim, src->getComponentDimensions(), src->getName(), true);
  // We need to flip the image "vertically", which means the bottom row becomes
  // the top row and convert from BGRA to RGBA ordering (This is a Little Endian code)
  // If this is ever compiled on a BIG ENDIAN machine the colors will be off.
  for(int y = 0; y < imageDim; y++)
  {
    const int destY = imageDim - 1 - y;
    for(int x = 0; x < imageDim; x++)
    {
      const size_t indexSrc = y * imageDim + x;
      const size_t indexDest = destY * imageDim + x;

      T* argbPtr = src->getTuplePointer(indexSrc);
      converted->setTuple(indexDest, argbPtr);
    }
  }
  return converted;
}

// -----------------------------------------------------------------------------
template <typename T>
typename EbsdDataArray<T>::Pointer RotateImage90About001(EbsdDataArray<T>* src, int width, int height)
{
  typename EbsdDataArray<T>::Pointer converted = EbsdDataArray<T>::CreateArray(width * height, src->getComponentDimensions(), src->getName(), true);

  int rotWidth = height;
  //int rotHeight = width;

  bool counterClockwise = false;

  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      const size_t indexSrc = y * width + x;
      // Calculate the new position in the rotated image
      int new_x = height - y - 1;
      int new_y = x;

      if(counterClockwise)
      {
        new_x = y;
        new_y = width - x - 1;
      }
      const size_t destIdx = new_y * rotWidth + new_x;

      T* argbPtr = src->getTuplePointer(indexSrc);
      converted->setTuple(destIdx, argbPtr);
    }
  }
  return converted;
}

template <typename T>
typename EbsdDataArray<T>::Pointer ConvertColorOrder(EbsdDataArray<T>* src, int imageDim)
{
  typename EbsdDataArray<T>::Pointer converted = EbsdDataArray<T>::CreateArray(imageDim * imageDim, src->getComponentDimensions(), src->getName(), true);
  // ARGB to RGBA ordering (This is a Little Endian code)
  // If this is ever compiled on a BIG ENDIAN machine the colors will be off.
  size_t numTuples = src->getNumberOfTuples();
  for(size_t tIdx = 0; tIdx < numTuples; tIdx++)
  {
    T* argbPtr = src->getTuplePointer(tIdx);
    T* destPtr = converted->getTuplePointer(tIdx);
    destPtr[0] = argbPtr[2];
    destPtr[1] = argbPtr[1];
    destPtr[2] = argbPtr[0];
    destPtr[3] = argbPtr[3];
  }
  return converted;
}

template <typename T>
typename EbsdDataArray<T>::Pointer RemoveAlphaChannel(EbsdDataArray<T>* src)
{
  typename EbsdDataArray<T>::Pointer converted = EbsdDataArray<T>::CreateArray(src->getNumberOfTuples(), {3ULL}, src->getName(), true);
  // RGBA to RGB ordering (This is a Little Endian code)
  // If this is ever compiled on a BIG ENDIAN machine the colors will be off.
  size_t numTuples = src->getNumberOfTuples();
  for(size_t tIdx = 0; tIdx < numTuples; tIdx++)
  {
    T* argbPtr = src->getTuplePointer(tIdx);
    T* destPtr = converted->getTuplePointer(tIdx);
    destPtr[0] = argbPtr[0];
    destPtr[1] = argbPtr[1];
    destPtr[2] = argbPtr[2];
  }
  return converted;
}

template <typename T>
typename EbsdDataArray<T>::Pointer CropRGBImage(typename EbsdDataArray<T>::Pointer src,
                                                int width, int height,
                                                int colStart, int rowStart,
                                                int numCols, int numRows)
{
  size_t numTuples = numCols * numRows;

  typename EbsdDataArray<T>::Pointer converted = EbsdDataArray<T>::CreateArray(numTuples, src->getComponentDimensions(), src->getName(), true);

  for(size_t y = rowStart; y < rowStart + numRows; y++)
  {
    for(size_t x = colStart; x < colStart + numCols; x++)
    {
      const size_t srcIdx = y * width + x;
      const size_t destIdx = (y -rowStart) * numCols + (x-colStart);

      converted->setTuple(destIdx, src->getTuplePointer(srcIdx));
    }
  }
  return converted;
}



} // namespace EbsdLib
