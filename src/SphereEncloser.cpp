#include <Eigen/Core>
#include <sbpl_geometry_utils/SphereEncloser.h>
#include <sbpl_geometry_utils/Triangle.h>
#include <sbpl_geometry_utils/Voxelizer.h>

namespace sbpl
{

void SphereEncloser::encloseBox(double xSize, double ySize, double zSize, double radius,
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

void SphereEncloser::encloseBox(double xSize, double ySize, double zSize, const geometry_msgs::Pose& pose,
                                double radius, std::vector<std::vector<double> >& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseCylinder(double cylinderRadius, double length, double radius,
                                     std::vector<std::vector<double> >& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseCylinder(double cylinderRadius, double length, const geometry_msgs::Pose& pose,
                                     double radius, std::vector<std::vector<double> >& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                                 double radius, std::vector<std::vector<double> >& spheres, bool fillMesh,
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
//        encloseMesh(vertices, triangles, newRadius, spheres, fillMesh, maxSpheres);
//
//        ROS_INFO("Ended up with only %lu spheres.", spheres.size());
//
//        return;
//    }
    double voxelLength = sqrt(2.0) * radius;
    Voxelizer::voxelizeMesh(vertices, triangles, voxelLength, spheres, fillMesh, maxSpheres);
    for (int i = 0; i < (int)spheres.size(); i++) {
        spheres[i].push_back(radius);
    }
}

void SphereEncloser::encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                                 const geometry_msgs::Pose& pose, double radius,
                                 std::vector<std::vector<double> >& spheres, bool fillMesh, int maxSpheres)
{
    return; // TODO
}

/******************** Private Methods ********************/

SphereEncloser::SphereEncloser()
{
}

}
