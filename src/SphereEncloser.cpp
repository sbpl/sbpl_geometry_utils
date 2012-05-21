#include <sbpl_geometry_utils/SphereEncloser.h>

namespace sbpl
{

void SphereEncloser::encloseBox(double xSize, double ySize, double zSize, double radius, std::vector<Sphere>& spheres)
{
    spheres.clear();
    Sphere s;
    s.radius = radius;
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
                s.center.x = xStart + enclCubeLength * x;
                s.center.y = yStart + enclCubeLength * y;
                s.center.z = zStart + enclCubeLength * z;
                spheres.push_back(s);
            }
        }
    }
}

void SphereEncloser::encloseBox(double xSize, double ySize, double zSize, const geometry_msgs::Pose& pose,
                                double radius, std::vector<Sphere>& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseCylinder(double cylinderRadius, double length, double radius, std::vector<Sphere>& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseCylinder(double cylinderRadius, double length, const geometry_msgs::Pose& pose,
                                     double radius, std::vector<Sphere>& spheres)
{
    return; // TODO
}

void SphereEncloser::encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                                 double radius, std::vector<Sphere>& spheres, bool fillMesh, int maxSpheres)
{
//    double voxelLength = sqrt(2.0) * radius;
//
//    spheres.clear();
//
//    double minX, minY, minZ;
//    double maxX, maxY, maxZ;
//    if (!getAxisAlignedBoundingBox(vertices, minX, minY, minZ, maxX, maxY, maxZ)) {
//        return;
//    }
//
//    // get the grid coordinates of the bounding voxel grid
//    int minVoxelX = (int)floor(minX / voxelLength);
//    int maxVoxelX = (int)floor(maxX / voxelLength);
//    int minVoxelY = (int)floor(minY / voxelLength);
//    int maxVoxelY = (int)floor(maxY / voxelLength);
//    int minVoxelZ = (int)floor(minZ / voxelLength);
//    int maxVoxelZ = (int)floor(maxZ / voxelLength);
//
//    int numVoxelsX = maxVoxelX - minVoxelX + 1;
//    int numVoxelsY = maxVoxelY - minVoxelY + 1;
//    int numVoxelsZ = maxVoxelZ - minVoxelZ + 1;
//
//    // Initialize the voxel grid
//    std::vector<std::vector<std::vector<bool> > > voxelGrid(numVoxelsX);
//    for (int i = 0; i < numVoxelsX; i++) {
//        voxelGrid[i].resize(numVoxelsY);
//        for (int j = 0; j < numVoxelsY; j++) {
//            voxelGrid[i][j].resize(numVoxelsZ);
//            for (int k = 0; k < numVoxelsZ; k++) {
//                voxelGrid[i][j][k] = false;
//            }
//        }
//    }
//
//    int numVoxelsFilled = 0;
//
//    // create a mesh for the voxel grid surrounding the mesh
//    std::vector<Triangle> entireVoxelMesh;
//    for (int i = 0; i < numVoxelsX; i++) {
//        for (int j = 0; j < numVoxelsY; j++) {
//            for (int k = 0; k < numVoxelsZ; k++) {
//                // Get the world coordinate of the voxel from its indices in the graph
//                double voxelCenterX = (i + minVoxelX) * voxelLength + 0.5 * voxelLength;
//                double voxelCenterY = (j + minVoxelY) * voxelLength + 0.5 * voxelLength;
//                double voxelCenterZ = (k + minVoxelZ) * voxelLength + 0.5 * voxelLength;
//
//                std::vector<Triangle> voxelMesh;
//                createCubeMesh(voxelCenterX, voxelCenterY, voxelCenterZ, voxelLength, voxelMesh);
//                for (unsigned ii = 0; ii < voxelMesh.size(); ii++) {
//                    entireVoxelMesh.push_back(voxelMesh[ii]); // add these triangles to the full voxel mesh
//                }
//            }
//        }
//    }
//
//    if ((int)triangles.size() % 3 != 0) {
//        ROS_ERROR("Mesh triangle list is not well-formed. Triangle list size should be a multiple of three.");
//        return;
//    }
//
//    // for every triangle
//    for (int triangleIdx = 0; triangleIdx < (int)triangles.size(); triangleIdx += 3) {
//        // get the vertices of the triangle as geometry_msgs::Point
//        const geometry_msgs::Point& pt1 = vertices[triangles[triangleIdx + 0]];
//        const geometry_msgs::Point& pt2 = vertices[triangles[triangleIdx + 1]];
//        const geometry_msgs::Point& pt3 = vertices[triangles[triangleIdx + 2]];
//
//        // pack those vertices into my Triangle struct
//        Triangle triangle;
//        triangle.p1.x = pt1.x;
//        triangle.p1.y = pt1.y;
//        triangle.p1.z = pt1.z;
//        triangle.p2.x = pt2.x;
//        triangle.p2.y = pt2.y;
//        triangle.p2.z = pt2.z;
//        triangle.p3.x = pt3.x;
//        triangle.p3.y = pt3.y;
//        triangle.p3.z = pt3.z;
//
//        // get the bounding voxels of the triangle
//        double triMinX, triMinY, triMinZ, triMaxX, triMaxY, triMaxZ;
//        std::vector<geometry_msgs::Point> triPointV;
//        triPointV.push_back(pt1);
//        triPointV.push_back(pt2);
//        triPointV.push_back(pt3);
//        if (!getAxisAlignedBoundingBox(triPointV, triMinX, triMinY, triMinZ, triMaxX, triMaxY, triMaxZ)) {
//            continue; // just skip this triangle; it's bogus
//        }
//        // shift all the voxels over to be aligned with the memory grid
//        int triMinVoxelX = floor(triMinX / voxelLength) - minVoxelX;
//        int triMinVoxelY = floor(triMinY / voxelLength) - minVoxelY;
//        int triMinVoxelZ = floor(triMinZ / voxelLength) - minVoxelZ;
//        int triMaxVoxelX = floor(triMaxX / voxelLength) - minVoxelX;
//        int triMaxVoxelY = floor(triMaxY / voxelLength) - minVoxelY;
//        int triMaxVoxelZ = floor(triMaxZ / voxelLength) - minVoxelZ;
//
//        // for every voxel in the voxel mesh i've constructed
//        for (unsigned a = 0; a < entireVoxelMesh.size(); a++) {
//            int voxelNum = a / 12; // there are 12 mesh triangles per voxel
//            int i = (voxelNum / (numVoxelsZ * numVoxelsY)) % numVoxelsX;
//            int j = (voxelNum / numVoxelsZ) % numVoxelsY;
//            int k = voxelNum % numVoxelsZ;
//            // if not already filled, is in the bounding voxel grid of the triangle, and this voxel mesh
//            // triangle intersects the current triangle, fill in the voxel
//            if (!voxelGrid[i][j][k] && isInDiscreteBoundingBox(i, j, k, triMinVoxelX, triMinVoxelY, triMinVoxelZ,
//                    triMaxVoxelX, triMaxVoxelY, triMaxVoxelZ) && intersects(triangle, entireVoxelMesh[a])) {
//                voxelGrid[i][j][k] = true;
//                numVoxelsFilled++;
//            }
//        }
//    }
//
//    // fill the mesh by scanning lines in the voxel outline grid
//    if (fillMesh) {
//        for (int i = 0; i < numVoxelsX; i++) {
//            for (int j = 0; j < numVoxelsY; j++) {
//                const int OUTSIDE = 0, ON_BOUNDARY_FROM_OUTSIDE = 1, INSIDE = 2, ON_BOUNDARY_FROM_INSIDE = 4;
//                int scanState = OUTSIDE;
//
//                for (int k = 0; k < numVoxelsZ; k++) {
//                    if (scanState == OUTSIDE && voxelGrid[i][j][k]) {
//                        scanState = ON_BOUNDARY_FROM_OUTSIDE;
//                    }
//                    else if (scanState == ON_BOUNDARY_FROM_OUTSIDE && !voxelGrid[i][j][k]) {
//                        bool allEmpty = true;
//                        for (int l = k; l < numVoxelsZ; l++) {
//                            allEmpty &= !voxelGrid[i][j][l];
//                        }
//                        if (allEmpty) {
//                            scanState = OUTSIDE;
//                        }
//                        else {
//                            scanState = INSIDE;
//                            voxelGrid[i][j][k] = true;
//                            numVoxelsFilled++;
//                        }
//                    }
//                    else if (scanState == INSIDE && !voxelGrid[i][j][k]) {
//                        voxelGrid[i][j][k] = true;
//                        numVoxelsFilled++;
//                    }
//                    else if (scanState == INSIDE && voxelGrid[i][j][k]) {
//                        scanState = ON_BOUNDARY_FROM_INSIDE;
//                    }
//                    else if (scanState == ON_BOUNDARY_FROM_INSIDE && !voxelGrid[i][j][k]) {
//                        scanState = OUTSIDE;
//                    }
//                }
//            }
//        }
//    }
//
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
//        getEnclosingSpheresOfMesh(vertices, triangles, newRadius, spheres, fillMesh, maxSpheres);
//
//        ROS_INFO("Ended up with only %lu spheres.", spheres.size());
//
//        return;
//    }
//
//    // push back all the spheres corresponding to filled voxels
//    for (int i = 0; i < numVoxelsX; i++) {
//        for (int j = 0; j < numVoxelsY; j++) {
//            for (int k = 0; k < numVoxelsZ; k++) {
//                if (voxelGrid[i][j][k]) {
//                    Sphere s;
//                    s.p.x = (i + minVoxelX) * voxelLength + 0.5 * voxelLength;
//                    s.p.y = (j + minVoxelY) * voxelLength + 0.5 * voxelLength;
//                    s.p.z = (k + minVoxelZ) * voxelLength + 0.5 * voxelLength;
//                    s.radius = radius;
//                    spheres.push_back(s);
//                }
//            }
//        }
//    }
}

void SphereEncloser::encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                                 const geometry_msgs::Pose& pose, double radius, std::vector<Sphere>& spheres,
                                 bool fillMesh, int maxSpheres)
{
    return; // TODO
}

/******************** Interface functions without Sphere usage ********************/

void SphereEncloser::encloseBox(double xSize, double ySize, double zSize, double radius,
                                std::vector<std::vector<double> >& spheres)
{
    std::vector<Sphere> s;
    encloseBox(xSize, ySize, zSize, radius, s);

    spheres.resize(s.size(), std::vector<double>(4, 0));
    for (std::size_t i = 0; i < s.size(); ++i) {
        spheres[i][0] = s[i].center.x;
        spheres[i][1] = s[i].center.y;
        spheres[i][2] = s[i].center.z;
        spheres[i][3] = s[i].radius;
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
    std::vector<sbpl::Sphere> s;
    encloseMesh(vertices, triangles, radius, s, fillMesh, maxSpheres);

    spheres.resize(s.size(), std::vector<double>(4, 0));
    for (std::size_t i = 0; i < s.size(); ++i) {
        spheres[i][0] = s[i].center.x;
        spheres[i][1] = s[i].center.y;
        spheres[i][2] = s[i].center.z;
        spheres[i][3] = s[i].radius;
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

bool SphereEncloser::getAxisAlignedBoundingBox(const std::vector<geometry_msgs::Point>& vertices, double& minX,
                                               double& minY, double& minZ, double& maxX, double& maxY, double& maxZ)
{
    if (vertices.size() < 3) {
        return false;
    }
    else {
        minX = maxX = vertices[0].x;
        minY = maxY = vertices[0].y;
        minZ = maxZ = vertices[0].z;
    }

    for (std::vector<geometry_msgs::Point>::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
        if (it->x < minX) minX = it->x;
        if (it->x > maxX) maxX = it->x;
        if (it->y < minY) minY = it->y;
        if (it->y > maxY) maxY = it->y;
        if (it->z < minZ) minZ = it->z;
        if (it->z > maxZ) maxZ = it->z;
    }
    return true;
}

}
