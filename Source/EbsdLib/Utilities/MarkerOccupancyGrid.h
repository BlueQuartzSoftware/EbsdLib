#pragma once

#include <vector>

#include "EbsdLib/EbsdLib.h"

namespace ebsdlib
{
/**
 * @brief Screen-space occupancy grid used to decimate overlapping markers.
 *
 * Sized to the marker footprint, only the first point that lands in a given cell
 * is drawn; later points in that cell are skipped. This turns O(N) marker draws
 * into O(area / cell_area) regardless of how many points exist. Modeled on
 * McPlotty's PointDecimationGrid.
 */
class EbsdLib_EXPORT MarkerOccupancyGrid
{
public:
  MarkerOccupancyGrid(float originX, float originY, float width, float height, float cellSize);

  /** @brief True only the first time an in-bounds point lands in a cell (marks it). */
  bool shouldDraw(float x, float y);

private:
  float m_OriginX;
  float m_OriginY;
  float m_Right;
  float m_Bottom;
  float m_CellSize;
  int m_Cols;
  int m_Rows;
  std::vector<bool> m_Occupied;
};
} // namespace ebsdlib
