#pragma once

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdDataArray.hpp"

#include <canvas_ity.hpp>


namespace EbsdLib
{

EbsdLib_EXPORT void WriteText(canvas_ity::canvas& context, const std::string& figureSubtitle, std::array<float, 2> textOrigin, int fontPtSize);

EbsdLib_EXPORT void DrawLine(canvas_ity::canvas& context, float xStart, float yStart, float xEnd, float yEnd);


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

template <typename T>
typename EbsdDataArray<T>::Pointer ConvertColorOrder(EbsdDataArray<T>* src, int imageDim)
{
  typename EbsdDataArray<T>::Pointer converted = EbsdDataArray<T>::CreateArray(imageDim * imageDim, src->getComponentDimensions(), src->getName(), true);
  // BGRA to RGBA ordering (This is a Little Endian code)
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






}
