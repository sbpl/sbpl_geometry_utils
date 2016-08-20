//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#ifndef sbpl_geometry_detail_mesh_utils_h
#define sbpl_geometry_detail_mesh_utils_h

#include <cstdio>

namespace sbpl {

template <typename Discretizer>
void CreateIndexedGridMesh(
    const VoxelGrid<Discretizer>& vg,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    // create all the vertices
    const size_t vertex_count = (vg.sizeX() + 1) * (vg.sizeY() + 1) * (vg.sizeZ() + 1);
    vertices.reserve(vertex_count);
    for (int ix = 0; ix < vg.sizeX() + 1; ix++) {
        for (int iy = 0; iy < vg.sizeY() + 1; iy++) {
            for (int iz = 0; iz < vg.sizeZ() + 1; iz++) {
                MemoryCoord mc(ix, iy, iz);
                WorldCoord wc(vg.memoryToWorld(mc));
                Eigen::Vector3d p =
                        Eigen::Vector3d(wc.x, wc.y, wc.z) - 0.5 * vg.res();
                vertices.push_back(p);
            }
        }
    }

    const size_t voxel_count = vg.sizeX() * vg.sizeY() * vg.sizeZ();
    const size_t xplane_count = vg.sizeY() * vg.sizeZ();
    const size_t yplane_count = vg.sizeX() * vg.sizeZ();
    const size_t zplane_count = vg.sizeX() * vg.sizeY();
    indices.reserve(6 * voxel_count * + 2 * xplane_count + 2 * yplane_count + 3 * zplane_count);

    // for every voxel there are 12 triangles
    for (int ix = 0; ix < vg.sizeX(); ++ix) {
        for (int iy = 0; iy < vg.sizeY(); ++iy) {
            for (int iz = 0; iz < vg.sizeZ(); ++iz) {
                MemoryCoord a(ix,     iy,     iz);
                MemoryCoord b(ix,     iy,     iz + 1);
                MemoryCoord c(ix,     iy + 1, iz);
                MemoryCoord d(ix,     iy + 1, iz + 1);
                MemoryCoord e(ix + 1, iy,     iz);
                MemoryCoord f(ix + 1, iy,     iz + 1);
                MemoryCoord g(ix + 1, iy + 1, iz);
                MemoryCoord h(ix + 1, iy + 1, iz + 1);

                auto to_index = [&](const MemoryCoord& c) -> size_t
                {
                    return c.x * (vg.sizeY() + 1) * (vg.sizeZ() + 1) +
                           c.y * (vg.sizeZ() + 1) +
                           c.z;
                };

                // back face
                indices.push_back(to_index(a));
                indices.push_back(to_index(b));
                indices.push_back(to_index(d));
                indices.push_back(to_index(a));
                indices.push_back(to_index(d));
                indices.push_back(to_index(c));

                // left face
                indices.push_back(to_index(a));
                indices.push_back(to_index(f));
                indices.push_back(to_index(b));
                indices.push_back(to_index(a));
                indices.push_back(to_index(e));
                indices.push_back(to_index(f));

                // bottom face
                indices.push_back(to_index(a));
                indices.push_back(to_index(g));
                indices.push_back(to_index(e));
                indices.push_back(to_index(a));
                indices.push_back(to_index(c));
                indices.push_back(to_index(g));

                // front face?
                if (ix == vg.sizeX() - 1) {
                    indices.push_back(to_index(f));
                    indices.push_back(to_index(g));
                    indices.push_back(to_index(e));
                    indices.push_back(to_index(f));
                    indices.push_back(to_index(h));
                    indices.push_back(to_index(g));
                }

                // right face?
                if (iy == vg.sizeY() - 1) {
                    indices.push_back(to_index(h));
                    indices.push_back(to_index(d));
                    indices.push_back(to_index(c));
                    indices.push_back(to_index(h));
                    indices.push_back(to_index(c));
                    indices.push_back(to_index(g));
                }

                // top face?
                if (iz == vg.sizeZ() - 1) {
                    indices.push_back(to_index(b));
                    indices.push_back(to_index(d));
                    indices.push_back(to_index(h));
                    indices.push_back(to_index(b));
                    indices.push_back(to_index(h));
                    indices.push_back(to_index(f));
                }
            }
        }
    }
}

template <typename Discretizer>
void CreateGridMesh(
    const VoxelGrid<Discretizer>& vg,
    std::vector<Eigen::Vector3d>& vertices)
{
    std::vector<Eigen::Vector3d> voxel_mesh;
    CreateBoxMesh(vg.res().x(), vg.res().y(), vg.res().z(), voxel_mesh);

    vertices.reserve(voxel_mesh.size() * vg.sizeX() * vg.sizeY() * vg.sizeZ());

    for (int x = 0; x < vg.sizeX(); ++x) {
        for (int y = 0; y < vg.sizeY(); ++y) {
            for (int z = 0; z < vg.sizeZ(); ++z) {
                const MemoryCoord mc(x, y, z);
                const WorldCoord wc(vg.memoryToWorld(mc));

                Eigen::Vector3d wp(wc.x, wc.y, wc.z);

                std::printf("%d, %d, %d -> %0.3f, %0.3f, %0.3f\n", x, y, z, wc.x, wc.y, wc.z);

                // translate all triangles by the voxel position
                for (const Eigen::Vector3d& v : voxel_mesh) {
                    vertices.push_back(v + wp);
                }
            }
        }
    }
}


} // namespace sbpl

#endif
