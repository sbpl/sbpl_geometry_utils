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

#include <sbpl_geometry_utils/Voxelizer.h>

// standard includes
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>

// project includes
#include <sbpl_geometry_utils/VoxelGrid.h>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl {

//////////////////////////////////
// Static Function Declarations //
//////////////////////////////////

static void VoxelizePlane(
    double a, double b, double c, double d,
    double voxel_size,
    unsigned char* grid,
    int width, int height, int depth);

/// \brief Voxelize a triangle
///
/// Based on the algorithm described in:
///
/// 'Huang, Yagel, Filippov, and Kurzion, "An Accurate Method for Voxelizing
/// Polygon Meshes," IEEE Volume Visualization '98, October, 1998, Chapel Hill,
/// North Carolina, USA, pp. 119-126'
template <typename Discretizer>
static void VoxelizeTriangle(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    VoxelGridBase<Discretizer>& vg);

template <typename Discretizer>
static void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    VoxelGridBase<Discretizer>& vg,
    bool fill = false);

template <typename Discretizer>
static void VoxelizeMeshAwesome(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    VoxelGridBase<Discretizer>& vg);

template <typename Discretizer>
void VoxelizeMeshNaive(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& triangles,
    VoxelGridBase<Discretizer>& vg);

template <typename Discretizer>
void ExtractVoxels(
    const VoxelGridBase<Discretizer>& vg,
    std::vector<Eigen::Vector3d>& voxels);

/// \brief Create a mesh representation of box
/// The box mesh is axis-aligned and located at the origin.
static void CreateIndexedBoxMesh(
    double length,
    double width,
    double height,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices);

/// \brief Create a mesh representation of a sphere
/// The sphere is located at the origin
static void CreateIndexedSphereMesh(
    double radius,
    int lng_count,
    int lat_count,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& triangles);

/// \brief Create a mesh representation of an upright (z-aligned) cylinder
/// The cylinder is located at the origin.
static void CreateIndexedCylinderMesh(
    double radius,
    double height,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices);

/// \brief Create a mesh representation of a grid
static void CreateIndexedGridMesh(
    double res,
    const std::vector<int>& minVoxel,
    const std::vector<int>& maxVoxel,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices);

static void CreateBoxMesh(
    double length,
    double width,
    double height,
    std::vector<Triangle>& trianglesOut);

template <typename Discretizer>
static void CreateGridMesh(
    const VoxelGridBase<Discretizer>& vg,
    std::vector<Triangle>& triangles);

static bool ComputeAxisAlignedBoundingBox(
    const std::vector<Eigen::Vector3d>& vertices,
    Eigen::Vector3d& min,
    Eigen::Vector3d& max);

static bool IsInDiscreteBoundingBox(
    const MemoryCoord& mc,
    const MemoryCoord& minmc,
    const MemoryCoord& maxmc);

static bool Intersects(
    const Triangle& tr1,
    const Triangle& tr2,
    double eps = 1.0e-4);

static bool PointOnTriangle(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& vertex1,
    const Eigen::Vector3d& vertex2,
    const Eigen::Vector3d& vertex3);

/// \brief Fill the interior of a voxel grid via scanning
template <typename Discretizer>
static void ScanFill(VoxelGridBase<Discretizer>& vg);

static void TransformVertices(
    const Eigen::Affine3d& transform,
    std::vector<Eigen::Vector3d>& vertices);

static double Distance(
    double a, double b, double c, double d,
    double x, double y, double z);

static double Distance(
    double p1x, double p1y, double p1z,
    double p2x, double p2y, double p2z,
    double radius_sqrd,
    double x, double y, double z);

static bool CompareX(const Eigen::Vector3d& u, const Eigen::Vector3d& v);
static bool CompareY(const Eigen::Vector3d& u, const Eigen::Vector3d& v);
static bool CompareZ(const Eigen::Vector3d& u, const Eigen::Vector3d& v);

/////////////////////////////////
// Static Function Definitions //
/////////////////////////////////

void VoxelizePlane(
    double a, double b, double c, double d,
    double voxel_size,
    unsigned char* grid,
    int length,
    int width,
    int height)
{
    // FIXME

    d *= -1;
//    double t = 1.0 / 2.0; // TODO: make voxel size
    double t = sqrt(3) / 2;
    for (int z = 0; z < height; z++) {
        for (int y = 0; y < width; y++) {
            for (int x = 0; x < length; x++) {
                // for each voxel at x, y, z
                double voxel_x = x + 0.5;
                double voxel_y = y + 0.5;
                double voxel_z = z + 0.5;
                if (utils::Signd(a * voxel_x + b * voxel_y + c * voxel_z + (d + t)) !=
                    utils::Signd(a * voxel_x + b * voxel_y + c * voxel_z + (d - t)))
                {
                    // voxel lies between two points
                    grid[length * width * z + length * y + x] = 1;
                }
            }
        }
    }
}

template <typename Discretizer>
void VoxelizeTriangle(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    VoxelGridBase<Discretizer>& vg)
{
    Eigen::Vector3d p_1 = a;
    Eigen::Vector3d p_2 = b;
    Eigen::Vector3d p_3 = c;

    // check for colinearity and counterclockwiseness
    double det = ((p_2 - p_1).cross(p_3 - p_1).norm());
    if (det == 0) {
        return;
    }

    // thickness parameters
//    double t = 0.5 * sqrt(3.0);
//    double t2 = 3.0 / 4.0;

    double t = 0.5 * 1.0;
    double t2 = 0.25;

    Eigen::Vector3d mintri;
    Eigen::Vector3d maxtri;
    ComputeAxisAlignedBoundingBox({ a, b, c }, mintri, maxtri);

    const WorldCoord minwc(mintri.x(), mintri.y(), mintri.z());
    const WorldCoord maxwc(maxtri.x(), maxtri.y(), maxtri.z());
    const GridCoord mingc = vg.worldToGrid(minwc);
    const GridCoord maxgc = vg.worldToGrid(maxwc);

    // make p_1, p_2, p_3 ccw
    if (det < 0.0) {
        std::swap(p_1, p_3);
    }

    // get the normal vector for the triangle
    Eigen::Vector3d u = p_2 - p_1;
    Eigen::Vector3d v = p_3 - p_2;
    Eigen::Vector3d w = p_1 - p_3;
    Eigen::Vector3d n = u.cross(v);
    n.normalize();

    // get the distance from the origin for the triangle plane
    double d = -n.dot(p_1);

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
    double d1 = -e1.dot(p_1);
    double d2 = -e2.dot(p_2);
    double d3 = -e3.dot(p_3);

    // consider all voxels that this triangle can voxelize
    for (int gx = mingc.x; gx <= maxgc.x; gx++) {
        for (int gy = mingc.y; gy <= maxgc.y; gy++) {
            for (int gz = mingc.z; gz <= maxgc.z; gz++) {
                const GridCoord gc(gx, gy, gz);
                if (vg[gc]) {
                    continue;
                }

                const WorldCoord wc = vg.gridToWorld(gc);

                // check if the voxel point is in the plane of the triangle and
                // within the edges
                const Eigen::Vector3d voxel_p(wc.x, wc.y, wc.z);

                double dx1 = voxel_p(0) - p_1(0); double dy1 = voxel_p(1) - p_1(1); double dz1 = voxel_p(2) - p_1(2);
                double dx2 = voxel_p(0) - p_2(0); double dy2 = voxel_p(1) - p_2(1); double dz2 = voxel_p(2) - p_2(2);
                double dx3 = voxel_p(0) - p_3(0); double dy3 = voxel_p(1) - p_3(1); double dz3 = voxel_p(2) - p_3(2);

                if ((dx1 * dx1 + dy1 * dy1 + dz1 * dz1) <= t2 ||
                    (dx2 * dx2 + dy2 * dy2 + dz2 * dz2) <= t2 ||
                    (dx3 * dx3 + dy3 * dy3 + dz3 * dz3) <= t2)
                {
                    // check for a vertex filling in this voxel
                    vg[gc] = true;
                }
                else if (Distance(p_1(0), p_1(1), p_1(2), p_2(0), p_2(1), p_2(2), t2, voxel_p(0), voxel_p(1), voxel_p(2)) != -1.0 ||
                         Distance(p_2(0), p_2(1), p_2(2), p_3(0), p_3(1), p_3(2), t2, voxel_p(0), voxel_p(1), voxel_p(2)) != -1.0 ||
                         Distance(p_3(0), p_3(1), p_3(2), p_1(0), p_1(1), p_1(2), t2, voxel_p(0), voxel_p(1), voxel_p(2)) != -1.0)
                {
                    // then check for an edge
                    vg[gc] = true;
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
                        vg[gc] = true;
                    }
                }
            }
        }
    }
}

template <typename Discretizer>
static void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    VoxelGridBase<Discretizer>& vg,
    bool fill)
{
    const bool awesome = true;
    if (awesome) {
        VoxelizeMeshAwesome(vertices, indices, vg);
    }
    else {
        VoxelizeMeshNaive(vertices, indices, vg);
    }

    if (fill) {
        ScanFill(vg);
    }
}

template <typename Discretizer>
void VoxelizeMeshAwesome(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    VoxelGridBase<Discretizer>& vg)
{
    for (int i = 0; i < (int)indices.size() / 3; i++) {
        const Eigen::Vector3d& a = vertices[indices[3 * i + 0]];
        const Eigen::Vector3d& b = vertices[indices[3 * i + 1]];
        const Eigen::Vector3d& c = vertices[indices[3 * i + 2]];
        VoxelizeTriangle(a, b, c, vg);
    }
}

template <typename Discretizer>
void VoxelizeMeshNaive(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& triangles,
    VoxelGridBase<Discretizer>& vg)
{
    // create a triangle mesh for the voxel grid surrounding the mesh
    // TODO: use indexed mesh
    std::vector<Triangle> voxel_mesh;
    CreateGridMesh(vg, voxel_mesh);
    std::cout << "Using " << voxel_mesh.size() * sizeof(Triangle) <<
            " bytes for voxel mesh" << std::endl;

    for (size_t tidx = 0; tidx < triangles.size(); tidx += 3) {
        // get the vertices of the triangle as Point
        const Eigen::Vector3d& pt1 = vertices[triangles[tidx + 0]];
        const Eigen::Vector3d& pt2 = vertices[triangles[tidx + 1]];
        const Eigen::Vector3d& pt3 = vertices[triangles[tidx + 2]];

        // pack those vertices into my Triangle struct
        Triangle triangle(pt1, pt2, pt3);

        // get the bounding box of the triangle
        std::vector<Eigen::Vector3d> triPointV = { pt1, pt2, pt3 };
        Eigen::Vector3d tri_min;
        Eigen::Vector3d tri_max;
        if (!ComputeAxisAlignedBoundingBox(triPointV, tri_min, tri_max)) {
            std::cerr << "Failed to compute AABB of triangle" << std::endl;
            continue; // just skip this triangle; it's bogus
        }

        // compute the bounding voxel grid
        const WorldCoord minwc(tri_min.x(), tri_min.y(), tri_min.z());
        const WorldCoord maxwc(tri_max.x(), tri_max.y(), tri_max.z());
        const MemoryCoord minmc = vg.worldToMemory(minwc);
        const MemoryCoord maxmc = vg.worldToMemory(maxwc);

        // voxels in the voxel mesh are ordered by the memory index
        for (size_t a = 0; a < voxel_mesh.size(); a++) {
            int voxelNum = a / 12; // there are 12 mesh triangles per voxel

            const MemoryIndex mi(voxelNum);
            const MemoryCoord mc = vg.indexToMemory(mi);

            // if not already filled, is in the bounding voxel grid of the
            // triangle, and this voxel mesh triangle Intersects the current
            // triangle, fill in the voxel

            if (!vg[mi] &&
                IsInDiscreteBoundingBox(mc, minmc, maxmc) &&
                Intersects(triangle, voxel_mesh[a]))
            {
                vg[mi] = true;
            }
        }
    }
}

template <typename Discretizer>
void ExtractVoxels(
    const VoxelGridBase<Discretizer>& vg,
    std::vector<Eigen::Vector3d>& voxels)
{
    for (int x = 0; x < vg.sizeX(); x++) {
        for (int y = 0; y < vg.sizeY(); y++) {
            for (int z = 0; z < vg.sizeZ(); z++) {
                MemoryCoord mc(x, y, z);
                if (vg[mc]) {
                    WorldCoord wc = vg.memoryToWorld(mc);
                    voxels.push_back(Eigen::Vector3d(wc.x, wc.y, wc.z));
                }
            }
        }
    }
}

void CreateIndexedBoxMesh(
    double length,
    double width,
    double height,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    vertices.clear();
    indices.clear();

    // l = left, r = right, t = top, b = bottom/back, f = front

    Eigen::Vector3d rtb_corner;
    rtb_corner.x() =  0.5 * length;
    rtb_corner.y() =  0.5 * width;
    rtb_corner.z() = -0.5 * height;

    Eigen::Vector3d ltb_corner;
    ltb_corner.x() = -0.5 * length;
    ltb_corner.y() =  0.5 * width;
    ltb_corner.z() = -0.5 * height;

    Eigen::Vector3d ltf_corner;
    ltf_corner.x() = -0.5 * length;
    ltf_corner.y() =  0.5 * width;
    ltf_corner.z() =  0.5 * height;

    Eigen::Vector3d rtf_corner;
    rtf_corner.x() = 0.5 * length;
    rtf_corner.y() = 0.5 * width;
    rtf_corner.z() = 0.5 * height;

    Eigen::Vector3d rbb_corner;
    rbb_corner.x() =  0.5 * length;
    rbb_corner.y() = -0.5 * width;
    rbb_corner.z() = -0.5 * height;

    Eigen::Vector3d lbb_corner;
    lbb_corner.x() = -0.5 * length;
    lbb_corner.y() = -0.5 * width;
    lbb_corner.z() = -0.5 * height;

    Eigen::Vector3d lbf_corner;
    lbf_corner.x() = -0.5 * length;
    lbf_corner.y() = -0.5 * width;
    lbf_corner.z() =  0.5 * height;

    Eigen::Vector3d rbf_corner;
    rbf_corner.x() =  0.5 * length;
    rbf_corner.y() = -0.5 * width;
    rbf_corner.z() =  0.5 * height;

    vertices.push_back(lbb_corner);
    vertices.push_back(rbb_corner);
    vertices.push_back(ltb_corner);
    vertices.push_back(rtb_corner);
    vertices.push_back(lbf_corner);
    vertices.push_back(rbf_corner);
    vertices.push_back(ltf_corner);
    vertices.push_back(rtf_corner);

    indices.push_back(0); indices.push_back(2); indices.push_back(1); // back faces
    indices.push_back(1); indices.push_back(2); indices.push_back(3);
    indices.push_back(5); indices.push_back(1); indices.push_back(7); // right face
    indices.push_back(1); indices.push_back(3); indices.push_back(7);
    indices.push_back(5); indices.push_back(7); indices.push_back(4); // front face
    indices.push_back(4); indices.push_back(7); indices.push_back(6);
    indices.push_back(4); indices.push_back(6); indices.push_back(0); // left face
    indices.push_back(0); indices.push_back(6); indices.push_back(2);
    indices.push_back(6); indices.push_back(7); indices.push_back(2); // top face
    indices.push_back(7); indices.push_back(3); indices.push_back(2);
    indices.push_back(5); indices.push_back(0); indices.push_back(1); // bottom face
    indices.push_back(4); indices.push_back(5); indices.push_back(0);
}

void CreateIndexedSphereMesh(
    double radius,
    int numLongitudes,
    int numLatitudes,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& triangles)
{
    // TODO: handle the case where there is only one line of longitude and thus
    // there are no quadrilaterals to break up into two triangles and the method
    // for getting the indices of those triangles breaks

    vertices.clear();
    triangles.clear();

    // create the top vertex
    Eigen::Vector3d northPole(0.0, 0.0, radius);
    vertices.push_back(northPole);

    // create the intermediate vertices
    double phiInc = (2.0 * M_PI) / numLatitudes;
    double thetaInc = M_PI / (numLongitudes + 1);
    for (int phi = 0; phi < numLatitudes; phi++) {
        for (int theta = 0; theta < numLongitudes; theta++) {
            double phiCont = phi * phiInc;
            double thetaCont = (theta + 1) * thetaInc;

            Eigen::Vector3d p(
                    radius * sin(thetaCont) * cos(phiCont),
                    radius * sin(thetaCont) * sin(phiCont),
                    radius * cos(thetaCont));
            vertices.push_back(p);
        }
    }

    // create the bottom vertex
    Eigen::Vector3d southPole(0.0, 0.0, -radius);
    vertices.push_back(southPole);

    // add all the triangles with the north pole as a vertex
    for (int i = 0; i < numLatitudes; i++) {
        // add in top triangle
        triangles.push_back(0);
        triangles.push_back(i + 1);
        if (i == numLatitudes - 1) {
            triangles.push_back(1);
        }
        else {
            triangles.push_back(i + 2);
        }
    }

    // add all intermediate triangles
    for (int i = 0; i < numLongitudes - 1; i++) {
        for (int j = 0; j < numLatitudes; j++) {
            // i, j corresponds to one of the generated vertices
            int baseVertexIdx = i * numLatitudes + j + 1;
            int bBaseVertexIdx = baseVertexIdx + numLatitudes;
            int brBaseVertexIdx = bBaseVertexIdx + 1;
            int rBaseVertexIdx = baseVertexIdx + 1;

            if ((brBaseVertexIdx - 1)/numLatitudes != (bBaseVertexIdx - 1)/numLatitudes) {
                brBaseVertexIdx -= numLatitudes;
            }

            if ((rBaseVertexIdx - 1)/numLatitudes != (baseVertexIdx - 1)/numLatitudes) {
                rBaseVertexIdx -= numLatitudes;
            }

            triangles.push_back(baseVertexIdx);
            triangles.push_back(bBaseVertexIdx);
            triangles.push_back(brBaseVertexIdx);

            triangles.push_back(baseVertexIdx);
            triangles.push_back(brBaseVertexIdx);
            triangles.push_back(rBaseVertexIdx);
        }
    }

    // add all the triangles with the south pole as a vertex
    for (int i = 0; i < numLatitudes; i++) {
        triangles.push_back(vertices.size() - 1);
        if (i == 0) {
            triangles.push_back(vertices.size() - 1 - numLatitudes);
        }
        else {
            triangles.push_back(vertices.size() - 1 - i);
        }
        triangles.push_back(vertices.size() - 1 - (i + 1));
    }
}

void CreateIndexedCylinderMesh(
    double radius,
    double length,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    const int numPtsOnRim = 16;

    for (int i = 0; i < numPtsOnRim; i++) {
        double theta = 2.0 * M_PI * (double)i / double(numPtsOnRim);
        Eigen::Vector3d p(
                radius * cos(theta),
                radius * sin(theta),
                0.5 * length);
        vertices.push_back(p);
    }

    for (int i = 0; i < numPtsOnRim; i++) {
        double theta = 2.0 * M_PI * (double)i / double(numPtsOnRim);
        Eigen::Vector3d p(
                radius * cos(theta),
                radius * sin(theta),
                -0.5 * length);
        vertices.push_back(p);
    }

    for (int i = 0; i < numPtsOnRim; i++) {
        indices.push_back(i);
        indices.push_back((i + 1) % numPtsOnRim);
        indices.push_back(i + numPtsOnRim);

        indices.push_back((i + 1) % numPtsOnRim);
        indices.push_back(((i + 1) % numPtsOnRim) + numPtsOnRim);
        indices.push_back(i + numPtsOnRim);
    }
}

void CreateIndexedGridMesh(
    double res,
    const std::vector<int>& minVoxel,
    const std::vector<int>& maxVoxel,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    // TODO: optimize by removing duplicate triangles (hopefully by never
    // generating them in the first place; this will then have to be propogated
    // to calling code to be able to know the voxels that correspond to a given
    // triangle

    assert(minVoxel.size() >= 3 && maxVoxel.size() >= 3);

    vertices.clear();
    indices.clear();

    int numVoxelsX = maxVoxel[0] - minVoxel[0] + 1;
    int numVoxelsY = maxVoxel[1] - minVoxel[1] + 1;
    int numVoxelsZ = maxVoxel[2] - minVoxel[2] + 1;

    // create all the vertices
    for (int i = 0; i < numVoxelsX + 1; i++) {
        for (int j = 0; j < numVoxelsY + 1; j++) {
            for (int k = 0; k < numVoxelsZ + 1; k++) {
                Eigen::Vector3d p(
                        p.x() = (i + minVoxel[0]) * res,
                        p.y() = (j + minVoxel[1]) * res,
                        p.z() = (k + minVoxel[2]) * res);
                vertices.push_back(p);
            }
        }
    }

    // for every voxel there are 12 triangles
    for (int i = 0; i < numVoxelsX; i++) {
        for (int j = 0; j < numVoxelsY; j++) {
            for (int k = 0; k < numVoxelsZ; k++) {
                int lbfIdx = numVoxelsY * numVoxelsZ * i + numVoxelsZ * j + k;
                int lbnIdx = lbfIdx + 1;
                int ltfIdx = lbfIdx + numVoxelsZ + 1;
                int ltnIdx = ltfIdx + 1;
                int rbfIdx = lbfIdx + (numVoxelsZ + 1) * (numVoxelsY + 1);
                int rbnIdx = rbfIdx + 1;
                int rtfIdx = rbfIdx + numVoxelsZ + 1;
                int rtnIdx = rtfIdx + 1;

                // two triangles on the right face
                indices.push_back(rtnIdx);
                indices.push_back(rbnIdx);
                indices.push_back(rtfIdx);

                indices.push_back(rtfIdx);
                indices.push_back(rbnIdx);
                indices.push_back(rbfIdx);

                // two triangles on the front face
                indices.push_back(ltnIdx);
                indices.push_back(lbnIdx);
                indices.push_back(rtnIdx);

                indices.push_back(rtnIdx);
                indices.push_back(lbnIdx);
                indices.push_back(rbnIdx);

                // two triangles on the top face
                indices.push_back(ltfIdx);
                indices.push_back(ltnIdx);
                indices.push_back(rtfIdx);

                indices.push_back(rtfIdx);
                indices.push_back(ltnIdx);
                indices.push_back(rtnIdx);

                // two triangles on the left face
                indices.push_back(ltnIdx);
                indices.push_back(ltfIdx);
                indices.push_back(lbfIdx);

                indices.push_back(ltnIdx);
                indices.push_back(lbfIdx);
                indices.push_back(lbnIdx);

                // two triangles on the back face
                indices.push_back(ltfIdx);
                indices.push_back(rbfIdx);
                indices.push_back(lbfIdx);

                indices.push_back(ltfIdx);
                indices.push_back(rtfIdx);
                indices.push_back(rbfIdx);

                // two triangles on the bottom face
                indices.push_back(lbfIdx);
                indices.push_back(rbnIdx);
                indices.push_back(lbnIdx);

                indices.push_back(lbfIdx);
                indices.push_back(rbfIdx);
                indices.push_back(rbnIdx);
            }
        }
    }
}

void CreateBoxMesh(
    double length,
    double width,
    double height,
    std::vector<Triangle>& trianglesOut)
{
    trianglesOut.clear();

    Eigen::Vector3d rtb_corner;
    rtb_corner.x() =  0.5 * length;
    rtb_corner.y() =  0.5 * width;
    rtb_corner.z() = -0.5 * height;

    Eigen::Vector3d ltb_corner;
    ltb_corner.x() = -0.5 * length;
    ltb_corner.y() =  0.5 * width;
    ltb_corner.z() = -0.5 * height;

    Eigen::Vector3d ltf_corner;
    ltf_corner.x() = -0.5 * length;
    ltf_corner.y() =  0.5 * width;
    ltf_corner.z() =  0.5 * height;

    Eigen::Vector3d rtf_corner;
    rtf_corner.x() = 0.5 * length;
    rtf_corner.y() = 0.5 * width;
    rtf_corner.z() = 0.5 * height;

    Eigen::Vector3d rbb_corner;
    rbb_corner.x() = +0.5 * length;
    rbb_corner.y() =  0.5 * width;
    rbb_corner.z() =  0.5 * height;

    Eigen::Vector3d lbb_corner;
    lbb_corner.x() = -0.5 * length;
    lbb_corner.y() = -0.5 * width;
    lbb_corner.z() = -0.5 * height;

    Eigen::Vector3d lbf_corner;
    lbf_corner.x() = -0.5 * length;
    lbf_corner.y() = -0.5 * width;
    lbf_corner.z() =  0.5 * height;

    Eigen::Vector3d rbf_corner;
    rbf_corner.x() =  0.5 * length;
    rbf_corner.y() = -0.5 * width;
    rbf_corner.z() =  0.5 * height;

    Triangle temp;

    // Front face triangles
    temp.a = lbf_corner;
    temp.b = rbf_corner;
    temp.c = ltf_corner;
    trianglesOut.push_back(temp);

    temp.a = rtf_corner;
    temp.b = ltf_corner;
    temp.c = rbf_corner;
    trianglesOut.push_back(temp);

    // Right face triangles
    temp.a = rbf_corner;
    temp.b = rbb_corner;
    temp.c = rtf_corner;
    trianglesOut.push_back(temp);

    temp.a = rtb_corner;
    temp.b = rtf_corner;
    temp.c = rbb_corner;
    trianglesOut.push_back(temp);

    // Back face triangles
    temp.a = rbb_corner;
    temp.b = lbb_corner;
    temp.c = rtb_corner;
    trianglesOut.push_back(temp);

    temp.a = ltb_corner;
    temp.b = rtb_corner;
    temp.c = lbb_corner;
    trianglesOut.push_back(temp);

    // Left face triangles
    temp.a = lbb_corner;
    temp.b = lbf_corner;
    temp.c = ltb_corner;
    trianglesOut.push_back(temp);

    temp.a = ltf_corner;
    temp.b = ltb_corner;
    temp.c = lbf_corner;
    trianglesOut.push_back(temp);

    // Bottom face triangles
    temp.a = rbb_corner;
    temp.b = rbf_corner;
    temp.c = lbb_corner;
    trianglesOut.push_back(temp);

    temp.a = lbf_corner;
    temp.b = lbb_corner;
    temp.c = rbf_corner;
    trianglesOut.push_back(temp);

    // Top face triangles
    temp.a = rtf_corner;
    temp.b = rtb_corner;
    temp.c = ltf_corner;
    trianglesOut.push_back(temp);

    temp.a = ltb_corner;
    temp.b = ltf_corner;
    temp.c = rtb_corner;
    trianglesOut.push_back(temp);
}

template <typename Discretizer>
static
void CreateGridMesh(
    const VoxelGridBase<Discretizer>& vg,
    std::vector<Triangle>& vg_mesh)
{
    std::vector<Triangle> voxel_mesh;
    CreateBoxMesh(vg.res().x(), vg.res().y(), vg.res().z(), voxel_mesh);

    for (int x = 0; x < vg.sizeX(); x++) {
        for (int y = 0; y < vg.sizeY(); y++) {
            for (int z = 0; z < vg.sizeZ(); z++) {
                const MemoryCoord mc(x, y, z);
                const WorldCoord wc(vg.memoryToWorld(mc));

                Eigen::Vector3d wp(wc.x, wc.y, wc.z);

                // translate all triangles by the voxel position
                for (const Triangle& triangle : voxel_mesh) {
                    Triangle t(
                            triangle.a + wp,
                            triangle.b + wp,
                            triangle.c + wp);
                    vg_mesh.push_back(t);
                }
            }
        }
    }
}

bool ComputeAxisAlignedBoundingBox(
    const std::vector<Eigen::Vector3d>& vertices,
    Eigen::Vector3d& min,
    Eigen::Vector3d& max)
{
    if (vertices.empty()) {
        return false;
    }

    min.x() = max.x() = vertices[0].x();
    min.y() = max.y() = vertices[0].y();
    min.z() = max.z() = vertices[0].z();

    for (const Eigen::Vector3d& vertex : vertices) {
        if (vertex.x() < min.x()) {
            min.x() = vertex.x();
        }
        if (vertex.x() > max.x()) {
            max.x() = vertex.x();
        }
        if (vertex.y() < min.y()) {
            min.y() = vertex.y();
        }
        if (vertex.y() > max.y()) {
            max.y() = vertex.y();
        }
        if (vertex.z() < min.z()) {
            min.z() = vertex.z();
        }
        if (vertex.z() > max.z()) {
            max.z() = vertex.z();
        }
    }

    return true;
}

bool IsInDiscreteBoundingBox(
    const MemoryCoord& mc,
    const MemoryCoord& minmc,
    const MemoryCoord& maxmc)
{
    bool inside = true;
    inside &= mc.x >= minmc.x && mc.x <= maxmc.x;
    inside &= mc.y >= minmc.y && mc.y <= maxmc.y;
    inside &= mc.z >= minmc.z && mc.z <= maxmc.z;
    return inside;
}

bool Intersects(const Triangle& tr1, const Triangle& tr2, double eps)
{
    // Vertices 0, 1, and 2 on triangle 1
    Eigen::Vector3d v10(tr1.a.x(), tr1.a.y(), tr1.a.z());
    Eigen::Vector3d v11(tr1.b.x(), tr1.b.y(), tr1.b.z());
    Eigen::Vector3d v12(tr1.c.x(), tr1.c.y(), tr1.c.z());

    // Vertices 0, 1, and 2 on triangle 2
    Eigen::Vector3d v20(tr2.a.x(), tr2.a.y(), tr2.a.z());
    Eigen::Vector3d v21(tr2.b.x(), tr2.b.y(), tr2.b.z());
    Eigen::Vector3d v22(tr2.c.x(), tr2.c.y(), tr2.c.z());

    ////////////////////////////////////////////////////////////////////////////////
    /// Reject if Triangle 1's vertices are all on the same side of Triangle 2
    ////////////////////////////////////////////////////////////////////////////////

    // Calculate parameters for the plane in which triangle 2 lies
    Eigen::Vector3d n2 = (v21 - v20).cross(v22 - v20);
    n2.normalize();
    double d2 = (-n2).dot(v20);

    // The distances from the vertices of triangle 1 to the plane of triangle 2
    double dv10 = n2.dot(v10) + d2;
    double dv11 = n2.dot(v11) + d2;
    double dv12 = n2.dot(v12) + d2;

    // approx. dv10 != 0.0, dv11 != 0.0, dv12 != 0.0
    if(fabs(dv10) > eps && fabs(dv11) > eps && fabs(dv12) > eps)
    {
        // all points lie to one side of the triangle
        if((dv10 > 0 && dv11 > 0 && dv12 > 0) || (dv10 < 0 && dv11 < 0 && dv12 < 0))
        {
            return false;
        }
    }
    else if(fabs(dv10) < eps && fabs(dv11) < eps && fabs(dv12) < eps)
    {
        ////////////////////////////////////////////////////////////////////////////////
        /// Perform tests to see if coplanar triangles are in collision
        ////////////////////////////////////////////////////////////////////////////////

        // Project the coplanar triangles into the X-Y plane
        std::vector<Eigen::Vector2d> firstTri2DVertices;
        Eigen::Vector2d u1;
        u1[0] = v10[0]; u1[1] = v10[1];
        firstTri2DVertices.push_back(u1);
        u1[0] = v11[0]; u1[1] = v11[1];
        firstTri2DVertices.push_back(u1);
        u1[0] = v12[0]; u1[1] = v12[1];
        firstTri2DVertices.push_back(u1);

        std::vector<Eigen::Vector2d> secondTri2DVertices;
        Eigen::Vector2d u2;
        u2[0] = v20[0]; u2[1] = v20[1];
        secondTri2DVertices.push_back(u2);
        u2[0] = v21[0]; u2[1] = v21[1];
        secondTri2DVertices.push_back(u2);
        u2[0] = v22[0]; u2[1] = v22[1];
        secondTri2DVertices.push_back(u2);

        ////////////////////////////////////////////////////////////////////////////////
        /// Perform collision tests between two-dimensional line segments
        ////////////////////////////////////////////////////////////////////////////////

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                // the points defining the first segment
                const Eigen::Vector2d& pt1 = firstTri2DVertices[i];
                const Eigen::Vector2d& pt2 = firstTri2DVertices[(i + 1) % 3];
                // the points defining the second segment
                const Eigen::Vector2d& pt3 = secondTri2DVertices[j];
                const Eigen::Vector2d& pt4 = secondTri2DVertices[(j + 1) % 3];

                double denom = (pt1[0] - pt2[0]) * (pt3[1] - pt4[1]) - (pt1[1] - pt2[1]) * (pt3[0] - pt4[0]);
                double tmp1 = pt1[0] * pt2[1] - pt1[1] * pt2[0];
                double tmp2 = pt3[0] * pt4[1] - pt3[1] * pt4[0];

                // if lines arent parallel
                if(fabs(denom) > eps)
                {
                    // the point of intersection between the two lines (from the segments)
                    Eigen::Vector2d intersection;
                    intersection[0] = tmp1 * (pt3[0] - pt4[0]) - (pt1[0] - pt2[0]) * tmp2;
                    intersection[0] /= denom;
                    intersection[1] = tmp1 * (pt3[1] - pt4[1]) - (pt1[1] - pt2[1]) * tmp2;
                    intersection[1] /= denom;

                    // check if the point of between the lines intersection lies on the line segments
                    double vx1 = pt2[0] - pt1[0];
                    double vx2 = pt4[0] - pt3[0];
                    if((intersection[0] - pt1[0]) / vx1 >= 0 && (intersection[0] - pt1[0]) / vx1 <= 1.0 &&
                       (intersection[0] - pt3[0]) / vx2 >= 0 && (intersection[0] - pt3[0]) / vx2 <= 1.0)
                    {
                        return true;
                    }
                }
                else
                {
                    // ANDREW: bad assumption, they could be the same line
                    //printf("Line segments are parallel and won't intersect\n");
                }
            }
        }

        ////////////////////////////////////////////////////////////////////////////////
        /// Check for containment of one triangle within another
        ////////////////////////////////////////////////////////////////////////////////

        // compute center point of first triangle
        Eigen::Vector2d firstTriangleCenter(0.0, 0.0);
        for (int i = 0; i < 3; i++) {
            firstTriangleCenter[0] += firstTri2DVertices[i][0];
            firstTriangleCenter[1] += firstTri2DVertices[i][1];
        }
        firstTriangleCenter[0] /= 3;
        firstTriangleCenter[1] /= 3;

        // compute center point of second triangle
        Eigen::Vector2d secondTriangleCenter(0.0, 0.0);
        for (int i = 0; i < 3; i++) {
            secondTriangleCenter[0] += secondTri2DVertices[i][0];
            secondTriangleCenter[1] += secondTri2DVertices[i][1];
        }
        secondTriangleCenter[0] /= 3;
        secondTriangleCenter[1] /= 3;

        Eigen::Vector3d firstTriangleCenterTo3D(firstTriangleCenter[0], firstTriangleCenter[1], 0.0);
        Eigen::Vector3d secondTriangleCenterTo3D(secondTriangleCenter[0], secondTriangleCenter[1], 0.0);
        Eigen::Vector3d u10To3D(firstTri2DVertices[0][0], firstTri2DVertices[0][1], 0.0);
        Eigen::Vector3d u11To3D(firstTri2DVertices[1][0], firstTri2DVertices[1][1], 0.0);
        Eigen::Vector3d u12To3D(firstTri2DVertices[2][0], firstTri2DVertices[2][1], 0.0);
        Eigen::Vector3d u20To3D(secondTri2DVertices[0][0], secondTri2DVertices[0][1], 0.0);
        Eigen::Vector3d u21To3D(secondTri2DVertices[1][0], secondTri2DVertices[1][1], 0.0);
        Eigen::Vector3d u22To3D(secondTri2DVertices[2][0], secondTri2DVertices[2][1], 0.0);

        // Awesome code re-use
        if(PointOnTriangle(firstTriangleCenterTo3D, u20To3D, u21To3D, u22To3D) ||
           PointOnTriangle(secondTriangleCenterTo3D, u10To3D, u11To3D, u12To3D))
        {
            return true;
        }

        return false;
    }
    else
    {
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Reject if Triangle 2's vertices are all on the same side of Triangle 1
    ////////////////////////////////////////////////////////////////////////////////

    // Calculate parameters for the plane in which triangle 1 lies
    Eigen::Vector3d n1 = (v11 - v10).cross(v12 - v10);
    n1.normalize();
    double d1 = (-n1).dot(v10);

    // The distances from the vertices of triangle 2 to the plane of triangle 1
    double dv20 = n1.dot(v20) + d1;
    double dv21 = n1.dot(v21) + d1;
    double dv22 = n1.dot(v22) + d1;

    // approx. dv20 != 0.0, dv21 != 0.0, dv22 != 0.0
    if(fabs(dv20) > eps && fabs(dv21) > eps && fabs(dv22) > eps)
    {
        // all points lie to one side of the triangle
        if((dv20 > 0 && dv21 > 0 && dv22 > 0) || (dv20 < 0 && dv21 < 0 && dv22 < 0))
        {
            return false;
        }
    }
    else
    {
        // ANDREW: should be covered by coplanar case above
        return false;
    }

    // The direction of the line of intersection between the two planes
    Eigen::Vector3d D = n1.cross(n2);

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get the interval on the line where Triangle 1 Intersects
    ////////////////////////////////////////////////////////////////////////////////////////

    double t1;
    double t2;

    double pv10 = D.dot(v10);
    double pv11 = D.dot(v11);
    double pv12 = D.dot(v12);

    // Vertex 0 of Triangle 1 is on the opposite side
    if((dv10 > 0 && dv11 < 0 && dv12 < 0) || (dv10 < 0 && dv11 > 0 && dv12 > 0))
    {
        t1 = pv11 + (pv10 - pv11) * (dv11 / (dv11 - dv10));
        t2 = pv12 + (pv10 - pv12) * (dv12 / (dv12 - dv10));
    }
    // Vertex 1 of Triangle 1 is on the opposite side
    else if((dv11 > 0 && dv10 < 0 && dv12 < 0) || (dv11 < 0 && dv10 > 0 && dv12 > 0))
    {
        t1 = pv10 + (pv11 - pv10) * (dv10 / (dv10 - dv11));
        t2 = pv12 + (pv11 - pv12) * (dv12 / (dv12 - dv11));
    }
    // Vertex 2 of triangle 1 is on the opposite side
    else //if((dv12 > 0 && dv10 < 0 && dv11 < 0) || (dv12 < 0 && dv10 > 0 && dv11 > 0))
    {
        t1 = pv10 + (pv12 - pv10) * (dv10 / (dv10 - dv12));
        t2 = pv11 + (pv12 - pv11) * (dv11 / (dv11 - dv12));
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get the interval on the line where Triangle 2 Intersects
    ////////////////////////////////////////////////////////////////////////////////////////

    double t3;
    double t4;

    double pv20 = D.dot(v20);
    double pv21 = D.dot(v21);
    double pv22 = D.dot(v22);

    // Vertex 0 of Triangle 2 is on the opposite side
    if((dv20 > 0 && dv21 < 0 && dv22 < 0) || (dv20 < 0 && dv21 > 0 && dv22 > 0))
    {
        t3 = pv21 + (pv20 - pv21) * (dv21 / (dv21 - dv20));
        t4 = pv22 + (pv20 - pv22) * (dv22 / (dv22 - dv20));
    }
    // Vertex 1 of Triangle 2 is on the opposite side
    else if((dv21 > 0 && dv20 < 0 && dv22 < 0) || (dv21 < 0 && dv20 > 0 && dv22 > 0))
    {
        t3 = pv20 + (pv21 - pv20) * (dv20 / (dv20 - dv21));
        t4 = pv22 + (pv21 - pv22) * (dv22 / (dv22 - dv21));
    }
    // Vertex 2 of triangle 2 is on the opposite side
    else //if((dv22 > 0 && dv20 < 0 && dv21 < 0) || (dv22 < 0 && dv20 > 0 && dv21 > 0))
    {
        t3 = pv20 + (pv22 - pv20) * (dv20 / (dv20 - dv22));
        t4 = pv21 + (pv22 - pv21) * (dv21 / (dv21 - dv22));
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Test for intersection of the above intervals
    ////////////////////////////////////////////////////////////////////////////////

    double temp = t1;
    t1 = std::min(t1, t2);
    t2 = std::max(temp, t2);

    temp = t3;
    t3 = std::min(t3, t4);
    t4 = std::max(temp, t4);

    // if intervals overlap, triangles intersect
    bool overlap = false;
    overlap |= (t3 <= t2 && t2 <= t4);
    overlap |= (t3 <= t1 && t1 <= t4);
    overlap |= (t1 <= t3 && t3 <= t2);
    overlap |= (t1 <= t4 && t4 <= t2);

    if(overlap)
    {
        return true;
    }

    return false;
}

bool PointOnTriangle(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& vertex1,
    const Eigen::Vector3d& vertex2,
    const Eigen::Vector3d& vertex3)
{
    Eigen::Vector3d center;
    center[0] = (vertex1[0] + vertex2[0] + vertex3[0]) / 3.0;
    center[1] = (vertex1[1] + vertex2[1] + vertex3[1]) / 3.0;
    center[2] = (vertex1[2] + vertex2[2] + vertex3[2]) / 3.0;

    Eigen::Vector3d v1Offset = (vertex1 - center); v1Offset.normalize(); v1Offset *= 0.0;
    Eigen::Vector3d v2Offset = (vertex2 - center); v2Offset.normalize(); v2Offset *= 0.0;
    Eigen::Vector3d v3Offset = (vertex3 - center); v3Offset.normalize(); v3Offset *= 0.0;

    Eigen::Vector3d v1 = vertex1 + v1Offset;
    Eigen::Vector3d v2 = vertex2 + v2Offset;
    Eigen::Vector3d v3 = vertex3 + v3Offset;

    Eigen::Vector3d a1 = v2 - v1; a1.normalize();
    Eigen::Vector3d a2 = v3 - v2; a2.normalize();
    Eigen::Vector3d a3 = v1 - v3; a3.normalize();

    Eigen::Vector3d b = a1.cross(a2); b.normalize();

    Eigen::Vector3d c1 = b.cross(a1); c1.normalize();
    Eigen::Vector3d c2 = b.cross(a2); c2.normalize();
    Eigen::Vector3d c3 = b.cross(a3); c3.normalize();

    bool insideCcw = true, insideCw = true;

    Eigen::Vector3d p1 = v1 - point; p1.normalize();
    Eigen::Vector3d p2 = v2 - point; p2.normalize();
    Eigen::Vector3d p3 = v3 - point; p3.normalize();

    insideCcw &= p1.dot(c1) < 0;
    insideCcw &= p2.dot(c2) < 0;
    insideCcw &= p3.dot(c3) < 0;

    insideCw &= p1.dot(c1) > 0;
    insideCw &= p2.dot(c2) > 0;
    insideCw &= p3.dot(c3) > 0;

    return insideCcw || insideCw;
}

template <typename Discretizer>
void ScanFill(VoxelGridBase<Discretizer>& vg)
{
    for (int x = 0; x < vg.sizeX(); x++) {
        for (int y = 0; y < vg.sizeY(); y++) {
            const int OUTSIDE = 0;
            const int ON_BOUNDARY_FROM_OUTSIDE = 1;
            const int INSIDE = 2;
            const int ON_BOUNDARY_FROM_INSIDE = 4;

            int scan_state = OUTSIDE;

            for (int z = 0; z < vg.sizeZ(); z++) {
                if (scan_state == OUTSIDE && vg[MemoryCoord(x, y, z)]) {
                    scan_state = ON_BOUNDARY_FROM_OUTSIDE;
                }
                else if (scan_state == ON_BOUNDARY_FROM_OUTSIDE &&
                    !vg[MemoryCoord(x, y, z)])
                {
                    bool allEmpty = true;
                    for (int l = z; l < vg.sizeZ(); l++) {
                        allEmpty &= !vg[MemoryCoord(x, y, l)];
                    }
                    if (allEmpty) {
                        scan_state = OUTSIDE;
                    }
                    else {
                        scan_state = INSIDE;
                        vg[MemoryCoord(x, y, z)] = true;
                    }
                }
                else if (scan_state == INSIDE && !vg[MemoryCoord(x, y, z)]) {
                    vg[MemoryCoord(x, y, z)] = true;
                }
                else if (scan_state == INSIDE && vg[MemoryCoord(x, y, z)]) {
                    scan_state = ON_BOUNDARY_FROM_INSIDE;
                }
                else if (scan_state == ON_BOUNDARY_FROM_INSIDE &&
                    !vg[MemoryCoord(x, y, z)])
                {
                    scan_state = OUTSIDE;
                }
            }
        }
    }
}

void TransformVertices(
    const Eigen::Affine3d& transform,
    std::vector<Eigen::Vector3d>& vertices)
{
    for (size_t i = 0; i < vertices.size(); i++) {
        vertices[i] = transform * vertices[i];
    }
}

double Distance(
    double a, double b, double c, double d,
    double x, double y, double z)
{
    double dist = (a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
    return dist;
}

double Distance(
    double p1x, double p1y, double p1z,
    double p2x, double p2y, double p2z,
    double radius_sqrd,
    double x, double y, double z)
{
    double dx = p2x - p1x;
    double dy = p2y - p1y;
    double dz = p2z - p1z;
    double length_sqrd = dx * dx + dy * dy + dz * dz;

    double pdx = x - p1x;
    double pdy = y - p1y;
    double pdz = z - p1z;

    double dot = pdx * dx + pdy * dy + pdz * dz;

    if (dot < 0.0 || dot > length_sqrd) {
        return -1.0;
    }
    else {
        double dsq = pdx * pdx + pdy * pdy + pdz * pdz - dot * dot / length_sqrd;
        if (dsq > radius_sqrd) {
            return -1.0;
        }
        else {
            return dsq;
        }
    }
}

bool CompareX(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    return u.x() < v.x();
}

bool CompareY(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    return u.y() < v.y();
}

bool CompareZ(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    return u.z() < v.z();
}

/////////////////////////////////
// Public Function Definitions //
/////////////////////////////////

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeSphere(
    double radius,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeSphere(
    double radius,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeCylinder(
    double radius,
    double length,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedCylinderMesh(radius, length, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeCylinder(
    double radius,
    double length,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedCylinderMesh(radius, length, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeCylinder(
    double radius, 
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedCylinderMesh(radius, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeCylinder(
    double radius, 
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> triangles;
    CreateIndexedCylinderMesh(radius, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    if (((int)indices.size()) % 3 != 0) {
        std::cerr << "Incorrect indexed triangles format" << std::endl;
        return;
    }

    voxels.clear();

    Eigen::Vector3d min;
    Eigen::Vector3d max;
    if (!ComputeAxisAlignedBoundingBox(vertices, min, max)) {
        std::cerr << "Failed to compute AABB of mesh vertices" << std::endl;
        return;
    }

    const Eigen::Vector3d size = max - min;
    HalfResVoxelGrid vg(min, size, Eigen::Vector3d(res, res, res));

    VoxelizeMesh(vertices, indices, vg, fill);
    ExtractVoxels(vg, voxels);
}

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& triangles,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> v_copy = vertices;
    TransformVertices(pose, v_copy);
    VoxelizeMesh(v_copy, triangles, res, voxels, fill);
}

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& triangles,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    if (((int)triangles.size()) % 3 != 0) {
        std::cerr << "Incorrect indexed triangles format" << std::endl;
        return;
    }

    voxels.clear();

    Eigen::Vector3d min;
    Eigen::Vector3d max;
    if (!ComputeAxisAlignedBoundingBox(vertices, min, max)) {
        std::cerr << "Failed to compute AABB of mesh vertices" << std::endl;
        return;
    }

    const Eigen::Vector3d size = max - min;
    PivotVoxelGrid vg(
            min, size, Eigen::Vector3d(res, res, res),
            Eigen::Vector3d(voxel_origin.x(), voxel_origin.y(), voxel_origin.z()));

    VoxelizeMesh(vertices, triangles, vg, fill);
    ExtractVoxels(vg, voxels);
}

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: implement
}

void VoxelizeSphereList(
    const std::vector<double>& radii,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill)
{
    if (radii.size() != poses.size()) {
        return;
    }

    voxels.clear();

    for (size_t i = 0; i < radii.size(); i++) {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<int> indices;
        CreateIndexedSphereMesh(radii[i], 9, 10, vertices, indices);

        TransformVertices(poses[i], vertices);

        std::vector<Eigen::Vector3d> sphere_voxels;
        VoxelizeMesh(vertices, indices, res, sphere_voxels, fill);

        voxels.insert(voxels.end(), sphere_voxels.begin(), sphere_voxels.end());
    }

    int duplicateIdx = (int)voxels.size();
    for (int i = 0; i < duplicateIdx; i++) {
        for (int j = i + 1; j < duplicateIdx; j++) {
            Eigen::Vector3d dx = voxels[i] = voxels[j];

            // since all voxels are aligned on the same grid, if the distance is
            // greater than half the resolution, it has to be the same voxel
            // (really if distance is just less than the resolution)
            if (dx.squaredNorm() < (res * res) / 4.0) {
                std::swap(voxels[duplicateIdx - 1], voxels[j]);
                duplicateIdx--;
            }
        }
    }

    volume = duplicateIdx * res * res * res;

    if (unique) {
        voxels.resize(duplicateIdx);
    }
}

void VoxelizeSphereListQAD(
    const std::vector<double>& spheres,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,      
    bool fill)
{
//    voxels.clear();
//
//    // compute the continuous bounding box of all spheres
//    double minXc = 1000000000.0;
//    double minYc = 1000000000.0;
//    double minZc = 1000000000.0;
//    double maxXc = -1000000000.0;
//    double maxYc = -1000000000.0;
//    double maxZc = -1000000000.0;
//    for (size_t i = 0; i < spheres.size(); i++) {
//        const double x = spheres[i][0];
//        const double y = spheres[i][1];
//        const double z = spheres[i][2];
//        const double r = spheres[i][3];
//        if (x - r < minXc) minXc = x - r;
//        if (y - r < minYc) minYc = y - r;
//        if (z - r < minZc) minZc = z - r;
//        if (x + r > maxXc) maxXc = x + r;
//        if (y + r > maxYc) maxYc = y + r;
//        if (z + r > maxZc) maxZc = z + r;
//    }
//
//    // compute discrete grid bounds and dimensions
//    HalfResDiscretizer disc(res);
//    const int minXd = disc.discretize(minXc);
//    const int minYd = disc.discretize(minYc);
//    const int minZd = disc.discretize(minZc);
//    const int maxXd = disc.discretize(maxXc);
//    const int maxYd = disc.discretize(maxYc);
//    const int maxZd = disc.discretize(maxZc);
//
//    const int sx = (maxXd - minXd) + 1;
//    const int sy = (maxYd - minYd) + 1;
//    const int sz = (maxZd - minZd) + 1;
//
//    // create an empty voxel grid
//    std::vector<int> grid(sx * sy * sz, 0);
//    Indexer indexer(sx, sy, sz);
//
//    // for each of sphere
//    for (size_t i = 0; i < spheres.size(); i++) {
//        const double x = spheres[i][0];
//        const double y = spheres[i][1];
//        const double z = spheres[i][2];
//        const double r = spheres[i][3];
//        const double r2 = r * r;
//
//        // iterate over grids in discrete bounding box of sphere
//        for (int xd = disc.discretize(x - r);
//            xd <= disc.discretize(x + r); ++xd)
//        {
//            for (int yd = disc.discretize(y - r);
//                yd <= disc.discretize(y + r); ++yd)
//            {
//                for (int zd = disc.discretize(z - r);
//                    zd <= disc.discretize(z + r); ++zd)
//                {
//                    const double dx = disc.continuize(xd) - x;
//                    const double dy = disc.continuize(yd) - y;
//                    const double dz = disc.continuize(zd) - z;
//                    const double d2 = dx * dx + dy * dy + dz * dz;
//                    if (d2 <= r2) {
//                        const int ix = xd - minXd;
//                        const int iy = yd - minYd;
//                        const int iz = zd - minZd;
//                        // count the number of spheres that enclose this cell
//                        // center
//                        ++grid[indexer.to_index(ix, iy, iz)];
//                    }
//                }
//            }
//        }
//    }
//
//    // run through the voxel grid and sum the volume
//    std::vector<double> p(3, 0);
//    volume = 0;
//    const double cellVolume = res * res * res;
//    for (int x = 0; x < sx; x++) {
//        for (int y = 0; y < sy; y++) {
//            for (int z = 0; z < sz; z++) {
//                const int gc = grid[indexer.to_index(x, y, z)];
//                if (gc > 0) {
//                    volume += cellVolume;
//
//                    if (unique) {
//                        p[0] = disc.continuize(minXd + x);
//                        p[1] = disc.continuize(minYd + y);
//                        p[2] = disc.continuize(minZd + z);
//                        voxels.push_back(p);
//                    }
//                    else {
//                        for (int i = 0; i < gc; ++i) {
//                            p[0] = disc.continuize(minXd + x);
//                            p[1] = disc.continuize(minYd + y);
//                            p[2] = disc.continuize(minZd + z);
//                            voxels.push_back(p);
//                        }
//                    }
//                }
//            }
//        }
//    }
}

} // namespace sbpl
