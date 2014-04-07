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

#ifndef SBPL_GEOMETRY_UTILS_SPHERE_ENCLOSER_H
#define SBPL_GEOMETRY_UTILS_SPHERE_ENCLOSER_H

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <sbpl_geometry_utils/Sphere.h>
#include <sbpl_geometry_utils/Triangle.h>

namespace sbpl
{

class SphereEncloser
{
public:
    /**
     * @brief Encloses an axis-aligned box with a set of spheres of a given radius.
     *
     * Encloses a box with a set of spheres of a given radius. The box is assumed to be centered on the origin and
     * axis-aligned.
     *
     * @param[in] xSize,ySize,zSize The lengths of all the sides of the box
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten; each element of
     *                     this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                     position and the fourth element as the radius of the sphere
     */
    static void encloseBox(double xSize, double ySize, double zSize, double radius,
                           std::vector<std::vector<double> >& spheres);

    /**
     * @brief Encloses a generic box with a set of spheres of a given radius.
     *
     * Encloses a box with a set of spheres of a given radius. The box is allowed any generic pose.
     *
     * @param[in] xSize,ySize,zSize The lengths of all the sides of the box
     * @param[in] pose The position and orientation of the box
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten; each element of
     *                     this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                     position and the fourth element as the radius of the sphere
     */
    static void encloseBox(double xSize, double ySize, double zSize, const geometry_msgs::Pose& pose, double radius,
                           std::vector<std::vector<double> >& spheres);

    /**
     * @brief Encloses an axis-aligned cylinder with a set of spheres of a given radius.
     *
     * Encloses a cylinder with a set of spheres of a given radius. The cylinder is assumed to be centered on the
     * origin and the cylinder's axis is the z-axis.
     *
     * @param[in] cylinderRadius,length The dimensions of the cylinder
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten; each element of
     *                     this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                     position and the fourth element as the radius of the sphere
     */
    static void encloseCylinder(double cylinderRadius, double length, double radius,
                                std::vector<std::vector<double> >& spheres);

    /**
     * @brief Encloses a generic cylinder with a set of spheres of a given radius.
     *
     * Encloses a cylinder with a set of spheres of a given radius. The cylinder is allowed any generic pose.
     *
     * @param[in] cylinderRadius,length The dimensions of the cylinder
     * @param[in] pose The position and orientation of the cylinder
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten; each element of
     *                     this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                     position and the fourth element as the radius of the sphere
     */
    static void encloseCylinder(double cylinderRadius, double length, const geometry_msgs::Pose& pose, double radius,
                                std::vector<std::vector<double> >& spheres);

    /**
     * @brief Encloses a mesh with a set of spheres of a given radius
     *
     * Encloses a mesh with a set of spheres of a given radius. The generated spheres appear in whatever frame the
     * vertices and triangles of the mesh are defined in.
     *
     * @param[in] vertices The vertices of the mesh
     * @param[in] triangles The triangles of the mesh as a list of indices into the vertex array
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten; each element of
     *                     this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                     position and the fourth element as the radius of the sphere
     * @param[in] fillMesh Whether to fill the interior of the mesh with spheres
     * @param[in] maxSpheres An upper bound on the number of generated spheres; will use a larger radius than $radius
     *                       if necessary to reduce the number of spheres generated
     */
    static void encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                            double radius, std::vector<std::vector<double> >& spheres, bool fillMesh = false,
                            int maxSpheres = 0);

    /**
     * @brief Encloses a generic mesh with a set of spheres of a given radius
     *
     * Encloses a generic mesh with a set of spheres of a given radius. The generated spheres appear in the frame of
     * the vertices transformed by a given pose.
     *
     * @param[in] vertices The vertices of the mesh
     * @param[in] triangles The triangles of the mesh as a list of indices into the vertex array
     * @param[in] pose The position and rotation of the mesh
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten; each element of
     *                     this vector is a vector of doubles with the first three elements as the sphere's x,y,z
     *                     position and the fourth element as the radius of the sphere
     * @param[in] fillMesh Whether to fill the interior of the mesh with spheres
     * @param[in] maxSpheres An upper bound on the number of generated spheres; will use a larger radius than $radius
     *                       if necessary to reduce the number of spheres generated
     */
    static void encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                            const geometry_msgs::Pose& pose, double radius, std::vector<std::vector<double> >& spheres,
                            bool fillMesh = false, int maxSpheres = 0);

private:
    SphereEncloser();

    static bool getAxisAlignedBoundingBox(const std::vector<geometry_msgs::Point>& vertices, double& minX,
                                          double& minY, double& minZ, double& maxX, double& maxY,
                                          double& maxZ);
    static void createCubeMesh(double x, double y, double z, double length, std::vector<Triangle>& trianglesOut);
    static void createVoxelMesh(std::vector<Triangle>& trangles, std::vector<int>& indices);
    static bool isInDiscreteBoundingBox(int i, int j, int k, int minx, int miny, int minz,
                                        int maxx, int maxy, int maxz);
    static bool intersects(const Triangle& tr1, const Triangle& tr2, double eps = 1.0e-4);
    static bool pointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& vertex1,
                                const Eigen::Vector3d& vertex2, const Eigen::Vector3d& vertex3);
};

}

#endif
