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

#ifndef SBPL_GEOMETRY_UTILS_VOXELIZER_H
#define SBPL_GEOMETRY_UTILS_VOXELIZER_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sbpl_geometry_utils/Triangle.h>

namespace sbpl
{

void VoxelizePlane(double a, double b, double c, double d, double voxel_size,
                   unsigned char* grid, int width, int height, int depth);

/// @brief Voxelize a triangle.
/// @param p1 The first vertex of the triangle (TODO: can vertices be specified in clockwise order?)
/// @param p2 The second vertex of the triangle
/// @param p3 The third vertex of the triangle
/// @param grid The grid in which to voxelize the triangle. The occupied voxels will have a value of 1 while
///             unoccupied voxels are unaffected. The grid spans the continuous world from the origin along the
///             positive axes x, y, z by width, height, depth respectively
/// @param width The width of the voxel grid
/// @param height The height of the voxel grid
/// @param depth The depth of the voxel grid
void VoxelizeTriangle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3,
                      unsigned char* grid, int width, int height, int depth);

void VoxelizeMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& indices,
                  unsigned char* grid, int width, int height, int depth);

/// @brief Voxelize a mesh.
/// @param[in] vertices The vertices of the mesh
/// @param[in] indices Indices into the list of vertices where every three successive indices correspond to a triangle
/// @param[in] cell_size The size of the voxels
/// @param[out] out The resulting voxel grid as a list of points; each element of \out is a 3-dimensional vector
///                 representing the center of a size cell_size^3, axis-aligned voxel.
void VoxelizeMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& indices,
                  double cell_size, std::vector<std::vector<double> >& out);

class Voxelizer
{
public:
    /**
     * @brief Encloses an axis-aligned box with a set of voxels of a given size.
     *
     * Encloses a box with a set of voxels of a given size. The box is assumed to be centered on the origin and
     * axis-aligned.
     *
     * @param[in] xSize,ySize,zSize The lengths of all the sides of the box
     * @param[in] voxelSize The desired length of the generated voxels
     * @param[out] voxels The vector in which to store the voxels; any previous data is overwritten; each element of
     *                    this vector is a vector of doubles with the first three elements as the voxel's x,y,z
     *                    position and the fourth element as the length of a voxel's side
     */
    static void voxelizeBox(double xSize, double ySize, double zSize, double voxelSize,
                            std::vector<std::vector<double> >& voxels, bool fillMesh = false);

    /**
     * @brief Encloses a generic box with a set of voxels of a given size.
     *
     * Encloses a box with a set of voxels of a given size. The box is allowed any generic pose.
     *
     * @param[in] xSize,ySize,zSize The lengths of all the sides of the box
     * @param[in] pose The position and orientation of the box
     * @param[in] voxelSize The desired length of the generated voxels
     * @param[out] voxels The vector in which to store the voxels; any previous data is overwritten; each element of
     *                    this vector is a vector of doubles with the first three elements as the voxel's x,y,z
     *                    position and the fourth element as the length of a voxel's side
     */
    static void voxelizeBox(double xSize, double ySize, double zSize, const geometry_msgs::Pose& pose, double voxelSize,
                            std::vector<std::vector<double> >& voxels, bool fillMesh = false);

	/**
     * @brief Encloses a sphere with a set of voxels of a given size.
     *
     * Encloses a sphere with a set of voxels of a given size. The sphere is assumed to be centered on the origin.
     *
     * @param radius The radius of the sphere
     * @param voxelSize The desired length of the generated voxels
     * @param voxels The vector in which to store the voxels; any previous data is overwritten; each element of
     *               this vector is a vector of doubles with the first three elements as the voxel's x,y,z
     *               position and the fourth element as the length of a voxel's side
     */
    static void voxelizeSphere(double radius, double voxelSize, std::vector<std::vector<double> >& voxels,
                               bool fillMesh = false);

    /**
     * @brief Encloses a sphere with a set of voxels of a given size.
     *
     * Encloses a sphere with a set of voxels of a given size. The sphere is assumed to be centered on the origin.
     *
     * @param radius The radius of the sphere
     * @param pose The position and (irrelevant) rotation
     * @param voxelSize The desired length of the generated voxels
     * @param voxels The vector in which to store the voxels; any previous data is overwritten; each element of
     *               this vector is a vector of doubles with the first three elements as the voxel's x,y,z
     *               position and the fourth element as the length of a voxel's side
     */
    static void voxelizeSphere(double radius, const geometry_msgs::Pose& pose, double voxelSize,
                               std::vector<std::vector<double> >& voxels, bool fillMesh = false);

    /**
     * @brief Encloses an axis-aligned cylinder with a set of voxels of a given size.
     *
     * Encloses a cylinder with a set of voxels of a given size. The cylinder is assumed to be centered on the
     * origin and the cylinder's axis is the z-axis.
     *
     * @param[in] cylinderRadius,length The dimensions of the cylinder
     * @param[in] voxelSize The desired length of the generated voxels
     * @param[out] voxels The vector in which to store the spheres; any previous data is overwritten; each element of
     *                    this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                    position and the fourth element as the radius of the sphere
     */
    static void voxelizeCylinder(double cylinderRadius, double length, double voxelSize,
                                 std::vector<std::vector<double> >& voxels, bool fillMesh = false);

    /**
     * @brief Encloses a generic cylinder with a set of voxels of a given radius
     *
     * Encloses a cylinder with a set of spheres of a given radius. The cylinder is allowed any generic pose.
     *
     * @param[in] cylinderRadius,length The dimensions of the cylinder
     * @param[in] pose The position and rotation of the cylinder
     * @param[in] voxelSize The desired length of the generated voxels
     * @param[out] voxels The vector in which to store the spheres; any previous data is overwritten; each element of
     *                    this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                    position and the fourth element as the radius of the sphere
     */
    static void voxelizeCylinder(double cylinderRadius, double length, const geometry_msgs::Pose& pose,
                                 double voxelSize, std::vector<std::vector<double> >& voxels, bool fillMesh = false);

    /**
     * @brief Encloses a mesh with a set of voxels of a given size
     *
     * Encloses a mesh with a set of voxels of a given size. The generated voxels appear in whatever frame the vertices
     * and triangles of the mesh are defined in.
     *
     * @param[in] vertices The vertices of the mesh
     * @param[in] triangles The triangles of the mesh as a list of indices into the vertex array
     * @param[in] voxelSize The desired length of the generated voxels
     * @param[out] voxels The vector in which to store the spheres; any previous data is overwritten; each element of
     *                    this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                    position and the fourth element as the radius of the sphere
     * @param[in] fillMesh Whether to fill the interior of the mesh with spheres
     * @param[in] maxVoxels An upper bound on the number of generated spheres; will use a larger voxel size than
     *                      $voxelSize if necessary to reduce the number of voxels generated
     */
    static void voxelizeMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                             double voxelSize, std::vector<std::vector<double> >& voxels, bool fillMesh = false,
                             int maxVoxels = 0);

    /**
     * @brief Encloses a mesh with a set of voxels of a given size
     *
     * Encloses a mesh with a set of voxels of a given size. The generated voxels appear in the frame of the vertices
     * transformed by a given pose.
     *
     * @param[in] vertices The vertices of the mesh
     * @param[in] triangles The triangles of the mesh as a list of indices into the vertex array
     * @param[in] pose The position and rotation of the mesh
     * @param[in] voxelSize The desired length of the generated voxels
     * @param[out] voxels The vector in which to store the voxels; any previous data is overwritten; each element of
     *                    this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                    position and the fourth element as the radius of the sphere
     * @param[in] fillMesh Whether to fill the interior of the mesh with spheres
     * @param[in] maxVoxels An upper bound on the number of generated spheres; will use a larger voxel size than
     *                      $voxelSize if necessary to reduce the number of voxels generated
     */
    static void voxelizeMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                             const geometry_msgs::Pose& pose, double voxelSize,
                             std::vector<std::vector<double> >& voxels, bool fillMesh = false, int maxSpheres = 0);

    /**
     * @brief Encloses a list of spheres with a set of voxels of a given size
     *
     * Encloses a list of spheres with a set of voxels of a given size. The generated voxels appear in the frame the
     * spheres are described in.
     *
     * @param[in] spheres The list of spheres to voxelize
     * @param[in] res The resolution of the voxel cells
     * @param[in] removeDuplicates Whether to remove duplicate voxels
     * @param[out] voxels The vector in which to store the voxels
     * @param[out] volume The combined volume of all the spheres
     */
    static void voxelizeSphereList(const std::vector<std::vector<double> >& spheres, double res, bool removeDuplicates,
                                   std::vector<std::vector<double> >& voxels, double& volume, bool fillMesh = false);

    /**
     * @brief a Quick And Dirty (QAD) enclosure of a list of spheres with a set of voxels of a given size
     *
     * Encloses a list of spheres with a set of voxels of a given size. The generated voxels appear in the frame the
     * spheres are described in.
     *
     * @param[in] spheres The list of spheres to voxelize
     * @param[in] res The resolution of the voxel cells
     * @param[in] removeDuplicates Whether to remove duplicate voxels
     * @param[out] voxels The vector in which to store the voxels
     * @param[out] volume The combined volume of all the spheres
     */
    static void voxelizeSphereListQAD(const std::vector<std::vector<double> >& spheres, double res,
                                      bool removeDuplicates, std::vector<std::vector<double> >& voxels,
                                      double& volume, bool fillMesh = false);

private:
    Voxelizer();

    static void createVoxelMesh(double res, const std::vector<int>& minVoxel, const std::vector<int>& maxVoxel,
                                std::vector<geometry_msgs::Point>& vertices, std::vector<int>& indices);
    static void createSphereMesh(const std::vector<double>& sphere, int numLongitudes, int numLatitudes,
                                 std::vector<geometry_msgs::Point>& vertices, std::vector<int>& triangles);
    // if pos does not have size 3, each missing coordinate is assumed to be 0; same goes for dims except each missing
    // dimension is assumed to be 1
    static void createBoxMesh(const std::vector<double>& pos, const std::vector<double>& dims,
                              std::vector<geometry_msgs::Point>& vertices, std::vector<int>& indices);
    // creates a mesh for an upright (axis aligned with z-axis) cylinder
    static void createCylinderMesh(const std::vector<double>& pos, double radius, double length,
                                   std::vector<geometry_msgs::Point>& vertices, std::vector<int>& indices);

    static bool getAxisAlignedBoundingBox(const std::vector<geometry_msgs::Point>& vertices, double& minX,
                                          double& minY, double& minZ, double& maxX, double& maxY, double& maxZ);
    static void createCubeMesh(double x, double y, double z, double length, std::vector<Triangle>& trianglesOut);
    static bool isInDiscreteBoundingBox(int i, int j, int k, int minx, int miny, int minz,
                                        int maxx, int maxy, int maxz);
    static bool intersects(const Triangle& tr1, const Triangle& tr2, double eps = 1.0e-4);
    static bool pointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& vertex1,
                                const Eigen::Vector3d& vertex2, const Eigen::Vector3d& vertex3);
    static void scanFill(std::vector<std::vector<std::vector<bool> > >& voxelGrid);
    static void transformVertices(const Eigen::Affine3d& transform, std::vector<geometry_msgs::Point>& vertices);
};

}

#endif
