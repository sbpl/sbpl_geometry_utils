//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014, Andrew Dornbush
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

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
