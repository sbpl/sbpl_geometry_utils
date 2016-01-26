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

#include <Eigen/Core>

#include <sbpl_geometry_utils/SphereEncloser.h>
#include <sbpl_geometry_utils/Triangle.h>
#include <sbpl_geometry_utils/Voxelizer.h>

namespace sbpl
{

bool ComputeCylinderBoundingSpheres(
    double cylinder_radius,
    double cylinder_height,
    double overestimation,
    std::vector<Sphere>& spheres)
{
    if (overestimation <= 0.0) {
        return false;
    }

    // start by trying to cover whole width of the cylinder with a single sphere
    // that overestimates by the allowable overestimation. this sphere should be
    // placed tightly on one end of the cylinder, so that the edge of the
    // cylinder cap intersects with the sphere. this will be allowed (1) if the
    // sphere does not extend past the cap so much as to extend beyond the
    // allowable overestimation (this condition is somehow derived from the
    // ratio between the cylinder's radius and the maximum overestimation) and
    // (2) if the sphere does not extend past the other cap beyond the allowable
    // overestimation

    // if the above step fails, subdivide the cylinder along its x and y axes,
    // and make the same attempt as before. this step will eventually succeed as
    // the radius of the sphere to the overestimation will decrease and
    // eventually fit the cylinder tightly enough

    int num_subdivs = -1;
    double subdiv_radius;
    double sphere_radius;
    double standoff;
    do {
        ++num_subdivs;
        subdiv_radius = cylinder_radius / pow(2.0, num_subdivs);
        sphere_radius = subdiv_radius + overestimation;
        const double theta = asin(subdiv_radius / sphere_radius);
        standoff = sphere_radius * cos(theta);
    }
    while (standoff > 0.5 * cylinder_height || sphere_radius - standoff > overestimation);

    const double max_span = 2 * standoff;
    const int num_spheres = (int)ceil(cylinder_height / max_span) + 1;
    double adjust_span = (cylinder_height - 2 * standoff) / (num_spheres - 1);
    double subdiv_delta = 2 * cylinder_radius / pow(2.0, num_subdivs);

    for (int ix = 0; ix < (1 << num_subdivs); ++ix) {
        for (int iy = 0; iy < (1 << num_subdivs); ++iy) {
            for (int iz = 0; iz < num_spheres; ++iz) {
                double x = -cylinder_radius + 0.5 * subdiv_delta + ix * subdiv_delta;
                double y = -cylinder_radius + 0.5 * subdiv_delta + iy * subdiv_delta;
                double z = -0.5 * cylinder_height + standoff + iz * adjust_span;
                double r = sphere_radius;
                Sphere s;
                s.center.x = x;
                s.center.y = y;
                s.center.z = z;
                s.radius = r;
                spheres.push_back(s);
            }
        }
    }

    return true;
}

void SphereEncloser::encloseBox(
    double xSize,
    double ySize,
    double zSize,
    double radius,
    std::vector<std::vector<double> >& spheres)
{
    spheres.clear();
    double rootTwo = sqrt(2.0);
    double enclCubeLength = rootTwo * radius;

    int xNumSpheres = ceil(xSize / (enclCubeLength));
    int yNumSpheres = ceil(ySize / (enclCubeLength));
    int zNumSpheres = ceil(zSize / (enclCubeLength));

    // Compute the coordinate of the sphere in the bottom-left-back corner
    double xStart = -1.0 * (xNumSpheres / 2) * enclCubeLength;
    if (xNumSpheres % 2 == 0) xStart += enclCubeLength / 2.0;

    double yStart = -1.0 * (yNumSpheres / 2) * enclCubeLength;
    if (yNumSpheres % 2 == 0) yStart += enclCubeLength / 2.0;

    double zStart = -1.0 * (zNumSpheres / 2) * enclCubeLength;
    if (zNumSpheres % 2 == 0) zStart += enclCubeLength / 2.0;

    // Compute the locations of all the spheres
    for (int x = 0; x < xNumSpheres; x++) {
        for (int y = 0; y < yNumSpheres; y++) {
            for (int z = 0; z < zNumSpheres; z++) {
                std::vector<double> sphere;
                sphere.push_back(xStart + enclCubeLength * x);
                sphere.push_back(yStart + enclCubeLength * y);
                sphere.push_back(zStart + enclCubeLength * z);
                spheres.push_back(sphere);
            }
        }
    }
}

void SphereEncloser::encloseBox(
    double xSize,
    double ySize,
    double zSize,
    const geometry_msgs::Pose& pose,
    double radius,
    std::vector<std::vector<double> >& spheres)
{
    // TODO: transform to be centered at 0,0,0 and axis-aligned; enclose; transform back
    return;
}

void SphereEncloser::encloseCylinder(
    double cylinderRadius,
    double length,
    double radius,
    std::vector<std::vector<double> >& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseCylinder(
    double cylinderRadius,
    double length,
    const geometry_msgs::Pose& pose,
    double radius,
    std::vector<std::vector<double> >& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseMesh(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    double radius,
    std::vector<std::vector<double> >& spheres,
    bool fill,
    int maxSpheres)
{
    // TODO: use the maxSpheres parameter; here's some commented out code from the old encloseMesh function
//    if (maxSpheres > 0 && numVoxelsFilled > maxSpheres) {
//        // we've been trying new radii and nothing can bring it down to the desired amount of spheres
//        if (radius > maxX - minX) {
//            spheres.clear();
//            return;
//        }
//
//        double boundingVolume = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
//        double newVoxelLength = pow(boundingVolume / maxSpheres, 0.333333);
//        double newRadius = newVoxelLength * sqrt(2.0) / 2.0;
//
//        // we calculated the same radius as suggested which means we were too optimistic with our radius and it should increase
//        if (fabs(newRadius - radius) < 1.0e-6) {
//            newRadius = radius * 1.1;
//        }
//        else if (newRadius < radius) {
//            newRadius = radius * 1.1;
//        }
//
//        ROS_INFO("Need a larger radius (%d > %d). Let's try %0.3f instead of %0.3f",
//                numVoxelsFilled, maxSpheres, newRadius, radius);
//
//        spheres.clear();
//        encloseMesh(vertices, triangles, newRadius, spheres, fill, maxSpheres);
//
//        ROS_INFO("Ended up with only %lu spheres.", spheres.size());
//
//        return;
//    }
    double res = sqrt(2.0) * radius;

    std::vector<Eigen::Vector3d> eigen_vertices;
    eigen_vertices.reserve(vertices.size());
    for (const geometry_msgs::Point& vertex : vertices) {
        eigen_vertices.push_back(Eigen::Vector3d(vertex.x, vertex.y, vertex.z));
    }

    std::vector<Eigen::Vector3d> sphere_points;
    VoxelizeMesh(eigen_vertices, triangles, res, sphere_points, fill);

    spheres.reserve(sphere_points.size());
    for (const Eigen::Vector3d& sphere_point : sphere_points) {
        const std::vector<double> sphere =
        {
            sphere_point.x(),
            sphere_point.y(),
            sphere_point.z(),
            radius
        };
        spheres.push_back(sphere);
    }
}

void SphereEncloser::encloseMesh(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    const geometry_msgs::Pose& pose,
    double radius,
    std::vector<std::vector<double> >& spheres,
    bool fillMesh,
    int maxSpheres)
{
    return; // TODO
}

/******************** Private Methods ********************/

SphereEncloser::SphereEncloser()
{
}

}
