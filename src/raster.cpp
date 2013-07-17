#include <sbpl_geometry_utils/raster.h>
#include <cstdlib>

namespace sbpl
{
    namespace raster
    {

void RasterizeLine(int x0, int y0, int x1, int y1, unsigned char* grid, int width, int height)
{
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    int error = dx / 2;
    int ystep = y0 < y1 ? 1 : -1;
    int y = y0;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            grid[width * x + y] = 1;
        }
        else {
            grid[width * y + x] = 1;
        }
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

    } // end namespace raster
} // end namespace sbpl
