#include "ScanGridOrder.h"

#include <cmath>
#include <sstream>

namespace ebsdlib
{
namespace
{
ScanGridOrder MakeError(int code, const std::string& message)
{
  ScanGridOrder order;
  order.errorCode = code;
  order.errorMessage = message;
  order.isIdentity = false;
  return order;
}

/** Shared tail: validates each (col,row), fills indexMap, detects duplicates. */
ScanGridOrder BuildFromCells(const std::vector<int64_t>& cols, const std::vector<int64_t>& rows, size_t numCols, size_t numRows)
{
  const size_t numTuples = cols.size();
  ScanGridOrder order;
  order.indexMap.resize(numTuples);
  std::vector<int64_t> occupant(numCols * numRows, -1);
  for(size_t i = 0; i < numTuples; i++)
  {
    const int64_t col = cols[i];
    const int64_t row = rows[i];
    if(col < 0 || row < 0 || static_cast<size_t>(col) >= numCols || static_cast<size_t>(row) >= numRows)
    {
      std::stringstream ss;
      ss << "tuple " << i << " has grid cell (col " << col << ", row " << row << ") which is outside the " << numCols << " x " << numRows << " raster grid.";
      return MakeError(-101102, ss.str());
    }
    const int64_t rasterIndex = row * static_cast<int64_t>(numCols) + col;
    if(occupant[static_cast<size_t>(rasterIndex)] >= 0)
    {
      std::stringstream ss;
      ss << "tuple " << i << " and tuple " << occupant[static_cast<size_t>(rasterIndex)] << " both map to grid cell (col " << col << ", row " << row
         << "). Each raster cell must be measured exactly once.";
      return MakeError(-101103, ss.str());
    }
    occupant[static_cast<size_t>(rasterIndex)] = static_cast<int64_t>(i);
    order.indexMap[i] = rasterIndex;
    if(rasterIndex != static_cast<int64_t>(i))
    {
      order.isIdentity = false;
    }
  }
  return order;
}
} // namespace

ScanGridOrder BuildScanGridOrderFromIndices(const int32_t* colIndices, const int32_t* rowIndices, size_t numTuples, size_t numCols, size_t numRows)
{
  if(colIndices == nullptr || rowIndices == nullptr)
  {
    return MakeError(-101101, "the column or row index array is null.");
  }
  if(numTuples != numCols * numRows)
  {
    std::stringstream ss;
    ss << "the file holds " << numTuples << " tuples but the header describes a " << numCols << " x " << numRows << " grid (" << numCols * numRows << " cells).";
    return MakeError(-101100, ss.str());
  }
  std::vector<int64_t> cols(numTuples);
  std::vector<int64_t> rows(numTuples);
  for(size_t i = 0; i < numTuples; i++)
  {
    cols[i] = colIndices[i];
    rows[i] = rowIndices[i];
  }
  return BuildFromCells(cols, rows, numCols, numRows);
}

ScanGridOrder BuildScanGridOrderFromPositions(const float* xPositions, const float* yPositions, size_t numTuples, size_t numCols, size_t numRows, float xStep, float yStep)
{
  if(xPositions == nullptr || yPositions == nullptr)
  {
    return MakeError(-101101, "the X or Y position array is null.");
  }
  if(numTuples != numCols * numRows)
  {
    std::stringstream ss;
    ss << "the file holds " << numTuples << " tuples but the header describes a " << numCols << " x " << numRows << " grid (" << numCols * numRows << " cells).";
    return MakeError(-101100, ss.str());
  }
  if(!(xStep > 0.0f) || !(yStep > 0.0f))
  {
    std::stringstream ss;
    ss << "the step sizes must be positive to place points on the grid (X Step " << xStep << ", Y Step " << yStep << ").";
    return MakeError(-101104, ss.str());
  }
  const auto xRange = std::minmax_element(xPositions, xPositions + numTuples);
  const auto yRange = std::minmax_element(yPositions, yPositions + numTuples);
  const float xMin = *xRange.first;
  const float yMin = *yRange.first;
  const auto spanCols = static_cast<size_t>(std::llround((*xRange.second - xMin) / xStep)) + 1;
  const auto spanRows = static_cast<size_t>(std::llround((*yRange.second - yMin) / yStep)) + 1;
  if(spanCols != numCols || spanRows != numRows)
  {
    std::stringstream ss;
    ss << "the X/Y positions span " << spanCols << " x " << spanRows << " cells (X " << xMin << " to " << *xRange.second << " step " << xStep << ", Y " << yMin << " to " << *yRange.second << " step "
       << yStep << ") but the header describes a " << numCols << " x " << numRows << " grid.";
    return MakeError(-101105, ss.str());
  }
  std::vector<int64_t> cols(numTuples);
  std::vector<int64_t> rows(numTuples);
  for(size_t i = 0; i < numTuples; i++)
  {
    cols[i] = std::llround((xPositions[i] - xMin) / xStep);
    rows[i] = std::llround((yPositions[i] - yMin) / yStep);
  }
  return BuildFromCells(cols, rows, numCols, numRows);
}
} // namespace ebsdlib
