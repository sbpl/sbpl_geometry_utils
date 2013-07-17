#ifndef SBPL_RASTER_RASTER_H
#define SBPL_RASTER_RASTER_H

#include <cmath>
#include <limits>
#include <ostream>

namespace sbpl
{
    namespace raster
    {

/// @brief Rasterize a line between two cells.
///
/// The rasterized line has its start and end points located in the center of
/// the cells given by (x0, y0) and (y0, y1). The grid is indexed internally as
/// grid[width * y + x] which corresponds to grid[x][y]. The grid has its origin
/// in the upper left corner with the positive x-axis pointing to the right and
/// the positive y-axis pointing downward. The cells in which the line is
/// rasterized are marked by the value 1. Untouched cells are left unaltered.
///
/// @param x0 The x coordinate of the first line endpoint
/// @param y0 The y coordinate of the first line endpoint
/// @param x1 The x coordinate of the second line endpoint
/// @param y1 The y coordinate of the second line endpoint
/// @param grid The grid as a single-dimensional array indexed
/// @param width The width of the grid
/// @param height The height of the grid
void RasterizeLine(int x0, int y0, int x1, int y1, unsigned char* grid, int width, int height);

    } // end namespace raster
} // end namespace sbpl

#endif
