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

#include <math.h>
#include <stdio.h>

#include <Eigen/Core>

#include <sbpl_geometry_utils/bounding_spheres.h>
#include <sbpl_geometry_utils/Triangle.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <sbpl_geometry_utils/mesh_utils.h>

#define SPHERE_DEBUG 0
#if SPHERE_DEBUG
#define SPHERE_LOG(stuff) stuff
#else
#define SPHERE_LOG(stuff)
#endif

namespace sbpl {

/// \brief Cover the surface of a box with a set of spheres.
///
/// The box has dimensions length x width x height and is centered at the
/// origin. This function will only append sphere centers to the output vector.
void ComputeBoxBoundingSpheres(
    double length, double width, double height,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a sphere with a set of spheres.
///
/// The sphere is centered at the origin. This function will only append sphere
/// centers to the output vector.
void ComputeSphereBoundingSpheres(
    double cradius,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedSphereMesh(cradius, 7, 8, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a cylinder with a set of spheres.
///
/// The cylinder is centered at the origin with the height along the z-axis.
/// This function will only append sphere centers to the output vector.
void ComputeCylinderBoundingSpheres(
    double cradius, double cheight,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedCylinderMesh(cradius, cheight, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a cone with a set of spheres.
///
/// The cone is centered at the origin with the height along the z-axis. This
/// function will only append sphere centers to the output vector.
void ComputeConeBoundingSpheres(
    double cradius, double cheight,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedConeMesh(cradius, cheight, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a mesh with a set of spheres.
///
/// This function will only append sphere centers to the output vector.
void ComputeMeshBoundingSpheres(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    const int triangle_count = indices.size() / 3;

    // for each triangle
    for (int tidx = 0; tidx < triangle_count; ++tidx) {
        const Eigen::Vector3d& a = vertices[indices[3 * tidx]];
        const Eigen::Vector3d& b = vertices[indices[3 * tidx + 1]];
        const Eigen::Vector3d& c = vertices[indices[3 * tidx + 2]];

        SPHERE_LOG(printf("a: %0.3f, %0.3f, %0.3f\n", a.x(), a.y(), a.z());)
        SPHERE_LOG(printf("b: %0.3f, %0.3f, %0.3f\n", b.x(), b.y(), b.z());)
        SPHERE_LOG(printf("c: %0.3f, %0.3f, %0.3f\n", c.x(), c.y(), c.z());)

        SPHERE_LOG(printf("enclosing triangle %d\n", tidx);)

        //  compute the pose of the triangle
        const double a2 = (b - c).squaredNorm();
        const double b2 = (a - c).squaredNorm();
        const double c2 = (a - b).squaredNorm();
        double bc1 = a2 * (b2 + c2 - a2);
        double bc2 = b2 * (c2 + a2 - b2);
        double bc3 = c2 * (a2 + b2 - c2);
        const double s = bc1 + bc2 + bc3;
        bc1 /= s;
        bc2 /= s;
        bc3 /= s;
        Eigen::Vector3d bc(bc1, bc2, bc3);

        SPHERE_LOG(printf("a2 = %0.3f, b2 = %0.3f, c2 = %0.3f\n", a2, b2, c2);)
        SPHERE_LOG(printf("bc1 = %0.3f, bc2 = %0.3f, bc3 = %0.3f\n", bc1, bc2, bc3);)
        SPHERE_LOG(printf("bc: %0.3f, %0.3f, %0.3f\n", bc.x(), bc.y(), bc.z());)

        Eigen::Vector3d p = bc[0] * a + bc[1] * b + bc[2] * c;
        SPHERE_LOG(printf("p: %0.3f, %0.3f, %0.3f\n", p.x(), p.y(), p.z());)

        Eigen::Vector3d z = (c - b).cross(b - a);
        z.normalize();
        Eigen::Vector3d x;
        if (a2 > b2 && a2 > c2) {
            x = b - c;
        }
        else if (b2 > c2) {
            x = a - c;
        }
        else {
            x = a - b;
        }
        x.normalize();
        Eigen::Vector3d y = z.cross(x);
        Eigen::Affine3d T_mesh_triangle;
        T_mesh_triangle(0, 0) = x[0];
        T_mesh_triangle(1, 0) = x[1];
        T_mesh_triangle(2, 0) = x[2];
        T_mesh_triangle(3, 0) = 0.0;

        T_mesh_triangle(0, 1) = y[0];
        T_mesh_triangle(1, 1) = y[1];
        T_mesh_triangle(2, 1) = y[2];
        T_mesh_triangle(3, 1) = 0.0;

        T_mesh_triangle(0, 2) = z[0];
        T_mesh_triangle(1, 2) = z[1];
        T_mesh_triangle(2, 2) = z[2];
        T_mesh_triangle(3, 2) = 0.0;

        T_mesh_triangle(0, 3) = p[0];
        T_mesh_triangle(1, 3) = p[1];
        T_mesh_triangle(2, 3) = p[2];
        T_mesh_triangle(3, 3) = 1.0;

        //  transform the triangle vertices into the triangle frame
        Eigen::Vector3d at = T_mesh_triangle.inverse() * a;
        Eigen::Vector3d bt = T_mesh_triangle.inverse() * b;
        Eigen::Vector3d ct = T_mesh_triangle.inverse() * c;
        SPHERE_LOG(printf("at: %0.3f, %0.3f, %0.3f\n", at.x(), at.y(), at.z());)
        SPHERE_LOG(printf("bt: %0.3f, %0.3f, %0.3f\n", bt.x(), bt.y(), bt.z());)
        SPHERE_LOG(printf("ct: %0.3f, %0.3f, %0.3f\n", ct.x(), ct.y(), ct.z());)

        double minx = std::min(at.x(), std::min(bt.x(), ct.x()));
        double miny = std::min(at.y(), std::min(bt.y(), ct.y()));
        double maxx = std::max(at.x(), std::max(bt.x(), ct.x()));
        double maxy = std::max(at.y(), std::max(bt.y(), ct.y()));

        //  voxelize the triangle
        PivotVoxelGrid vg(
                Eigen::Vector3d(minx, miny, 0.0),
                Eigen::Vector3d(maxx - minx, maxy - miny, 0.0),
                Eigen::Vector3d(radius, radius, radius),
                Eigen::Vector3d::Zero());
        VoxelizeTriangle(at, bt, ct, vg);

        // extract filled voxels and append as sphere centers
        for (int x = 0; x < vg.sizeX(); ++x) {
            for (int y = 0; y < vg.sizeY(); ++y) {
                MemoryCoord mc(x, y, 0);
                if (vg[mc]) {
                    WorldCoord wc = vg.memoryToWorld(mc);
                    centers.push_back(
                            T_mesh_triangle *
                            Eigen::Vector3d(wc.x, wc.y, wc.z));
                }
            }
        }
    }
}

} // namespace sbpl
