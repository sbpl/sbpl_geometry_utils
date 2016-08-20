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

#ifndef sbpl_geometry_detail_voxelize_h
#define sbpl_geometry_detail_voxelize_h

namespace sbpl {

/// \brief Voxelize a triangle
///
/// Based on the algorithm described in:
///
/// 'Huang, Yagel, Filippov, and Kurzion, "An Accurate Method for Voxelizing
/// Polygon Meshes," IEEE Volume Visualization '98, October, 1998, Chapel Hill,
/// North Carolina, USA, pp. 119-126'
template <typename Discretizer>
void VoxelizeTriangle(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    VoxelGrid<Discretizer>& vg)
{
    Eigen::Vector3d p1 = a;
    Eigen::Vector3d p2 = b;
    Eigen::Vector3d p3 = c;

    // check for colinearity and counterclockwiseness
    double det = ((p2 - p1).cross(p3 - p1).norm());
    if (det == 0) {
        return;
    }

    // ensure p1, p2, p3 ccw
    if (det < 0.0) {
        std::swap(p1, p3);
    }

    // thickness parameters

    double rc = sqrt(3.0) * 0.5 * vg.res().x();
//    double rc = sqrt(3.0) * 0.5 * vg.res().x();
    double rc2 = rc * rc;

    // get the normal vector for the triangle
    Eigen::Vector3d u = p2 - p1;
    Eigen::Vector3d v = p3 - p2;
    Eigen::Vector3d w = p1 - p3;
    Eigen::Vector3d n = u.cross(v);
    n.normalize();

    double ca = 1.0;
    double corners[] = {
        Eigen::Vector3d(-0.5774, -0.5774, -0.5774).dot(n),
        Eigen::Vector3d(-0.5774, -0.5774,  0.5774).dot(n),
        Eigen::Vector3d(-0.5774,  0.5774, -0.5774).dot(n),
        Eigen::Vector3d(-0.5774,  0.5774,  0.5774).dot(n),
        Eigen::Vector3d( 0.5774, -0.5774, -0.5774).dot(n),
        Eigen::Vector3d( 0.5774, -0.5774,  0.5774).dot(n),
        Eigen::Vector3d( 0.5774,  0.5774, -0.5774).dot(n),
        Eigen::Vector3d( 0.5774,  0.5774,  0.5774).dot(n)
    };
    ca = *std::max_element(corners, corners + sizeof(corners));

    double t = rc * ca;

    // get the distance from the origin for the triangle plane
    double d = -n.dot(p1);

    // normal to the edge p2 - p1 pointing inwards
    Eigen::Vector3d e1 = -u.cross(n);
    e1.normalize();

    // normal to the edge p3 - p2 pointing inwards
    Eigen::Vector3d e2 = -v.cross(n);
    e2.normalize();

    // normal to the edge p1 - p3 pointing inwards
    Eigen::Vector3d e3 = -w.cross(n);
    e3.normalize();

    // distances of the edge-guard planes from the origin
    double d1 = -e1.dot(p1);
    double d2 = -e2.dot(p2);
    double d3 = -e3.dot(p3);

    Eigen::Vector3d mintri;
    Eigen::Vector3d maxtri;
    ComputeAxisAlignedBoundingBox({ a, b, c }, mintri, maxtri);

    const WorldCoord minwc(mintri.x(), mintri.y(), mintri.z());
    const WorldCoord maxwc(maxtri.x(), maxtri.y(), maxtri.z());
    const GridCoord mingc = vg.worldToGrid(minwc);
    const GridCoord maxgc = vg.worldToGrid(maxwc);

    // consider all voxels that this triangle can voxelize
    for (int gx = mingc.x; gx <= maxgc.x; gx++) {
        for (int gy = mingc.y; gy <= maxgc.y; gy++) {
            for (int gz = mingc.z; gz <= maxgc.z; gz++) {
                const GridCoord gc(gx, gy, gz);
                if (vg[gc]) {
                    continue;
                }

                const WorldCoord wc = vg.gridToWorld(gc);

                // TODO: shortcut based off of distance to triangle plane?

                // check if the voxel point is in the plane of the triangle and
                // within the edges
                const Eigen::Vector3d voxel_p(wc.x, wc.y, wc.z);

                Eigen::Vector3d dx1 = voxel_p - p1;
                Eigen::Vector3d dx2 = voxel_p - p2;
                Eigen::Vector3d dx3 = voxel_p - p3;

                if (dx1.squaredNorm() <= rc2 ||
                    dx2.squaredNorm() <= rc2 ||
                    dx3.squaredNorm() <= rc2)
                {
                    // vertex fills this voxel
                    vg[gc] = 1;
                }
                else if (Distance(p1, p3, rc2, voxel_p) != -1.0 ||
                         Distance(p2, p3, rc2, voxel_p) != -1.0 ||
                         Distance(p3, p1, rc2, voxel_p) != -1.0)
                {
                    // edge fills this voxel
                    vg[gc] = 1;
                }
                else {
                    // then check for inside the triangle
                    if ((
                            // inside triangle thickness
                            utils::Signd(n.dot(voxel_p) + (d + t)) != utils::Signd(n.dot(voxel_p) + (d - t))
                        ) &&
                        (
                            // inside the edge bounding planes
                            (
                                (e1.dot(voxel_p) + d1 > 0.0) &&
                                (e2.dot(voxel_p) + d2 > 0.0) &&
                                (e3.dot(voxel_p) + d3 > 0.0)
                            )
                        ))
                    {
                        vg[gc] = 1;
                    }
                }
            }
        }
    }
}

} // namespace sbpl

#endif
