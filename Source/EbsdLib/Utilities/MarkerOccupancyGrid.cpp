#include "EbsdLib/Utilities/MarkerOccupancyGrid.h"

#include <algorithm>
#include <cmath>

namespace ebsdlib
{
MarkerOccupancyGrid::MarkerOccupancyGrid(float originX, float originY, float width, float height, float cellSize)
: m_OriginX(originX)
, m_OriginY(originY)
, m_Right(originX + width)
, m_Bottom(originY + height)
, m_CellSize(std::max(cellSize, 1.0f))
, m_Cols(static_cast<int>(std::ceil(width / std::max(cellSize, 1.0f))) + 1)
, m_Rows(static_cast<int>(std::ceil(height / std::max(cellSize, 1.0f))) + 1)
, m_Occupied(static_cast<size_t>(m_Cols) * static_cast<size_t>(m_Rows), false)
{
}

bool MarkerOccupancyGrid::shouldDraw(float x, float y)
{
  if(x < m_OriginX || x > m_Right || y < m_OriginY || y > m_Bottom)
  {
    return false;
  }
  const int col = static_cast<int>((x - m_OriginX) / m_CellSize);
  const int row = static_cast<int>((y - m_OriginY) / m_CellSize);
  if(col < 0 || col >= m_Cols || row < 0 || row >= m_Rows)
  {
    return false;
  }
  const size_t idx = static_cast<size_t>(row) * static_cast<size_t>(m_Cols) + static_cast<size_t>(col);
  if(m_Occupied[idx])
  {
    return false;
  }
  m_Occupied[idx] = true;
  return true;
}
} // namespace ebsdlib
