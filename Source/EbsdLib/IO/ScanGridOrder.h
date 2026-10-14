#pragma once

#include "EbsdLib/EbsdLib.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ebsdlib
{
/**
 * @brief Result of validating the spatial order of the tuples read from a scan file.
 *
 * Vendors do not guarantee that the per-point datasets in a scan file are stored in
 * raster order (X fastest, then Y). This structure carries the map that places tuple i
 * of the file at raster index indexMap[i] = row * numCols + col.
 */
struct EbsdLib_EXPORT ScanGridOrder
{
  /** Negative when the coordinates do not describe exactly one tuple per raster cell. */
  int errorCode = 0;
  /** Human readable reason for a negative errorCode. Empty on success. */
  std::string errorMessage;
  /** indexMap[fileIndex] = rasterIndex. Empty when errorCode < 0. */
  std::vector<int64_t> indexMap;
  /** True when every tuple already sits at its raster index, so no copy is required. */
  bool isIdentity = true;

  bool isValid() const
  {
    return errorCode >= 0;
  }
};

/**
 * @brief Builds the raster map from integer column/row indices such as Bruker 'X BEAM'/'Y BEAM'.
 *
 * Error codes: -101100 numTuples != numCols*numRows; -101101 null pointer;
 * -101102 index outside the grid; -101103 two tuples share one cell.
 */
EbsdLib_EXPORT ScanGridOrder BuildScanGridOrderFromIndices(const int32_t* colIndices, const int32_t* rowIndices, size_t numTuples, size_t numCols, size_t numRows);

/**
 * @brief Builds the raster map from floating point positions and step sizes such as .ang, .ctf or .h5oina X/Y.
 *
 * The cell of a point is llround((position - minimum) / step), so positions carrying float noise
 * resolve to the nearest cell. Error codes: those of BuildScanGridOrderFromIndices plus
 * -101104 step <= 0 and -101105 position extent does not match numCols/numRows.
 */
EbsdLib_EXPORT ScanGridOrder BuildScanGridOrderFromPositions(const float* xPositions, const float* yPositions, size_t numTuples, size_t numCols, size_t numRows, float xStep, float yStep);

/**
 * @brief Moves tuple i of data to position indexMap[i], in place, using one scratch copy of the buffer.
 *
 * Each tuple holds numComponents consecutive values (3 for Euler triplets, width*height for a pattern).
 * Does nothing when data is null or indexMap.size() != numTuples.
 */
template <typename T>
void ReorderScanTuples(T* data, size_t numTuples, size_t numComponents, const std::vector<int64_t>& indexMap)
{
  if(data == nullptr || indexMap.size() != numTuples || numComponents == 0)
  {
    return;
  }
  std::vector<T> buffer(numTuples * numComponents);
  for(size_t fileIndex = 0; fileIndex < numTuples; fileIndex++)
  {
    const size_t rasterIndex = static_cast<size_t>(indexMap[fileIndex]);
    std::copy_n(data + fileIndex * numComponents, numComponents, buffer.data() + rasterIndex * numComponents);
  }
  std::copy(buffer.begin(), buffer.end(), data);
}
} // namespace ebsdlib
