#include <Eigen/Core>
#include <sbpl_geometry_utils/SphereEncloser.h>
#include <sbpl_geometry_utils/Triangle.h>

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
    double voxelLength = sqrt(2.0) * radius;

    spheres.clear();

    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    if (!getAxisAlignedBoundingBox(vertices, minX, minY, minZ, maxX, maxY, maxZ)) {
        return;
    }

    // get the grid coordinates of the bounding voxel grid
    int minVoxelX = (int)floor(minX / voxelLength);
    int maxVoxelX = (int)floor(maxX / voxelLength);
    int minVoxelY = (int)floor(minY / voxelLength);
    int maxVoxelY = (int)floor(maxY / voxelLength);
    int minVoxelZ = (int)floor(minZ / voxelLength);
    int maxVoxelZ = (int)floor(maxZ / voxelLength);

    int numVoxelsX = maxVoxelX - minVoxelX + 1;
    int numVoxelsY = maxVoxelY - minVoxelY + 1;
    int numVoxelsZ = maxVoxelZ - minVoxelZ + 1;

    // Initialize the voxel grid
    std::vector<std::vector<std::vector<bool> > > voxelGrid(numVoxelsX);
    for (int i = 0; i < numVoxelsX; i++) {
        voxelGrid[i].resize(numVoxelsY);
        for (int j = 0; j < numVoxelsY; j++) {
            voxelGrid[i][j].resize(numVoxelsZ);
            for (int k = 0; k < numVoxelsZ; k++) {
                voxelGrid[i][j][k] = false;
            }
        }
    }

    int numVoxelsFilled = 0;

    // create a mesh for the voxel grid surrounding the mesh
    std::vector<Triangle> entireVoxelMesh;
    for (int i = 0; i < numVoxelsX; i++) {
        for (int j = 0; j < numVoxelsY; j++) {
            for (int k = 0; k < numVoxelsZ; k++) {
                // Get the world coordinate of the voxel from its indices in the graph
                double voxelCenterX = (i + minVoxelX) * voxelLength + 0.5 * voxelLength;
                double voxelCenterY = (j + minVoxelY) * voxelLength + 0.5 * voxelLength;
                double voxelCenterZ = (k + minVoxelZ) * voxelLength + 0.5 * voxelLength;

                std::vector<Triangle> voxelMesh;
                createCubeMesh(voxelCenterX, voxelCenterY, voxelCenterZ, voxelLength, voxelMesh);
                for (unsigned ii = 0; ii < voxelMesh.size(); ii++) {
                    entireVoxelMesh.push_back(voxelMesh[ii]); // add these triangles to the full voxel mesh
                }
            }
        }
    }

    if ((int)triangles.size() % 3 != 0) {
        ROS_ERROR("Mesh triangle list is not well-formed. Triangle list size should be a multiple of three.");
        return;
    }

    // for every triangle
    for (int triangleIdx = 0; triangleIdx < (int)triangles.size(); triangleIdx += 3) {
        // get the vertices of the triangle as geometry_msgs::Point
        const geometry_msgs::Point& pt1 = vertices[triangles[triangleIdx + 0]];
        const geometry_msgs::Point& pt2 = vertices[triangles[triangleIdx + 1]];
        const geometry_msgs::Point& pt3 = vertices[triangles[triangleIdx + 2]];

        // pack those vertices into my Triangle struct
        Triangle triangle;
        triangle.p1.x = pt1.x;
        triangle.p1.y = pt1.y;
        triangle.p1.z = pt1.z;
        triangle.p2.x = pt2.x;
        triangle.p2.y = pt2.y;
        triangle.p2.z = pt2.z;
        triangle.p3.x = pt3.x;
        triangle.p3.y = pt3.y;
        triangle.p3.z = pt3.z;

        // get the bounding voxels of the triangle
        double triMinX, triMinY, triMinZ, triMaxX, triMaxY, triMaxZ;
        std::vector<geometry_msgs::Point> triPointV;
        triPointV.push_back(pt1);
        triPointV.push_back(pt2);
        triPointV.push_back(pt3);
        if (!getAxisAlignedBoundingBox(triPointV, triMinX, triMinY, triMinZ, triMaxX, triMaxY, triMaxZ)) {
            continue; // just skip this triangle; it's bogus
        }
        // shift all the voxels over to be aligned with the memory grid
        int triMinVoxelX = floor(triMinX / voxelLength) - minVoxelX;
        int triMinVoxelY = floor(triMinY / voxelLength) - minVoxelY;
        int triMinVoxelZ = floor(triMinZ / voxelLength) - minVoxelZ;
        int triMaxVoxelX = floor(triMaxX / voxelLength) - minVoxelX;
        int triMaxVoxelY = floor(triMaxY / voxelLength) - minVoxelY;
        int triMaxVoxelZ = floor(triMaxZ / voxelLength) - minVoxelZ;

        // for every voxel in the voxel mesh i've constructed
        for (unsigned a = 0; a < entireVoxelMesh.size(); a++) {
            int voxelNum = a / 12; // there are 12 mesh triangles per voxel
            int i = (voxelNum / (numVoxelsZ * numVoxelsY)) % numVoxelsX;
            int j = (voxelNum / numVoxelsZ) % numVoxelsY;
            int k = voxelNum % numVoxelsZ;
            // if not already filled, is in the bounding voxel grid of the triangle, and this voxel mesh
            // triangle intersects the current triangle, fill in the voxel
            if (!voxelGrid[i][j][k] && isInDiscreteBoundingBox(i, j, k, triMinVoxelX, triMinVoxelY, triMinVoxelZ,
                    triMaxVoxelX, triMaxVoxelY, triMaxVoxelZ) && intersects(triangle, entireVoxelMesh[a])) {
                voxelGrid[i][j][k] = true;
                numVoxelsFilled++;
            }
        }
    }

    // fill the mesh by scanning lines in the voxel outline grid
    if (fillMesh) {
        for (int i = 0; i < numVoxelsX; i++) {
            for (int j = 0; j < numVoxelsY; j++) {
                const int OUTSIDE = 0, ON_BOUNDARY_FROM_OUTSIDE = 1, INSIDE = 2, ON_BOUNDARY_FROM_INSIDE = 4;
                int scanState = OUTSIDE;

                for (int k = 0; k < numVoxelsZ; k++) {
                    if (scanState == OUTSIDE && voxelGrid[i][j][k]) {
                        scanState = ON_BOUNDARY_FROM_OUTSIDE;
                    }
                    else if (scanState == ON_BOUNDARY_FROM_OUTSIDE && !voxelGrid[i][j][k]) {
                        bool allEmpty = true;
                        for (int l = k; l < numVoxelsZ; l++) {
                            allEmpty &= !voxelGrid[i][j][l];
                        }
                        if (allEmpty) {
                            scanState = OUTSIDE;
                        }
                        else {
                            scanState = INSIDE;
                            voxelGrid[i][j][k] = true;
                            numVoxelsFilled++;
                        }
                    }
                    else if (scanState == INSIDE && !voxelGrid[i][j][k]) {
                        voxelGrid[i][j][k] = true;
                        numVoxelsFilled++;
                    }
                    else if (scanState == INSIDE && voxelGrid[i][j][k]) {
                        scanState = ON_BOUNDARY_FROM_INSIDE;
                    }
                    else if (scanState == ON_BOUNDARY_FROM_INSIDE && !voxelGrid[i][j][k]) {
                        scanState = OUTSIDE;
                    }
                }
            }
        }
    }

    if (maxSpheres > 0 && numVoxelsFilled > maxSpheres) {
        // we've been trying new radii and nothing can bring it down to the desired amount of spheres
        if (radius > maxX - minX) {
            spheres.clear();
            return;
        }

        double boundingVolume = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
        double newVoxelLength = pow(boundingVolume / maxSpheres, 0.333333);
        double newRadius = newVoxelLength * sqrt(2.0) / 2.0;

        // we calculated the same radius as suggested which means we were too optimistic with our radius and it should increase
        if (fabs(newRadius - radius) < 1.0e-6) {
            newRadius = radius * 1.1;
        }
        else if (newRadius < radius) {
            newRadius = radius * 1.1;
        }

        ROS_INFO("Need a larger radius (%d > %d). Let's try %0.3f instead of %0.3f",
                numVoxelsFilled, maxSpheres, newRadius, radius);

        spheres.clear();
        encloseMesh(vertices, triangles, newRadius, spheres, fillMesh, maxSpheres);

        ROS_INFO("Ended up with only %lu spheres.", spheres.size());

        return;
    }

    // push back all the spheres corresponding to filled voxels
    for (int i = 0; i < numVoxelsX; i++) {
        for (int j = 0; j < numVoxelsY; j++) {
            for (int k = 0; k < numVoxelsZ; k++) {
                if (voxelGrid[i][j][k]) {
                    Sphere s;
                    s.center.x = (i + minVoxelX) * voxelLength + 0.5 * voxelLength;
                    s.center.y = (j + minVoxelY) * voxelLength + 0.5 * voxelLength;
                    s.center.z = (k + minVoxelZ) * voxelLength + 0.5 * voxelLength;
                    s.radius = radius;
                    spheres.push_back(s);
                }
            }
        }
    }
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

void SphereEncloser::createCubeMesh(double x, double y, double z, double length, std::vector<Triangle>& trianglesOut)
{
    trianglesOut.clear();

    geometry_msgs::Point rightTopBackCorner;
    rightTopBackCorner.x = x + 0.5 * length;
    rightTopBackCorner.y = y + 0.5 * length;
    rightTopBackCorner.z = z - 0.5 * length;

    geometry_msgs::Point leftTopBackCorner;
    leftTopBackCorner.x = x - 0.5 * length;
    leftTopBackCorner.y = y + 0.5 * length;
    leftTopBackCorner.z = z - 0.5 * length;

    geometry_msgs::Point leftTopFrontCorner;
    leftTopFrontCorner.x = x - 0.5 * length;
    leftTopFrontCorner.y = y + 0.5 * length;
    leftTopFrontCorner.z = z + 0.5 * length;

    geometry_msgs::Point rightTopFrontCorner;
    rightTopFrontCorner.x = x + 0.5 * length;
    rightTopFrontCorner.y = y + 0.5 * length;
    rightTopFrontCorner.z = z + 0.5 * length;

    geometry_msgs::Point rightBottomBackCorner;
    rightBottomBackCorner.x = x + 0.5 * length;
    rightBottomBackCorner.y = y - 0.5 * length;
    rightBottomBackCorner.z = z - 0.5 * length;

    geometry_msgs::Point leftBottomBackCorner;
    leftBottomBackCorner.x = x - 0.5 * length;
    leftBottomBackCorner.y = y - 0.5 * length;
    leftBottomBackCorner.z = z - 0.5 * length;

    geometry_msgs::Point leftBottomFrontCorner;
    leftBottomFrontCorner.x = x - 0.5 * length;
    leftBottomFrontCorner.y = y - 0.5 * length;
    leftBottomFrontCorner.z = z + 0.5 * length;

    geometry_msgs::Point rightBottomFrontCorner;
    rightBottomFrontCorner.x = x + 0.5 * length;
    rightBottomFrontCorner.y = y - 0.5 * length;
    rightBottomFrontCorner.z = z + 0.5 * length;

    Triangle temp;

    // Front face triangles
    temp.p1 = leftBottomFrontCorner;
    temp.p2 = rightBottomFrontCorner;
    temp.p3 = leftTopFrontCorner;
    trianglesOut.push_back(temp);

    temp.p1 = rightTopFrontCorner;
    temp.p2 = leftTopFrontCorner;
    temp.p3 = rightBottomFrontCorner;
    trianglesOut.push_back(temp);

    // Right face triangles
    temp.p1 = rightBottomFrontCorner;
    temp.p2 = rightBottomBackCorner;
    temp.p3 = rightTopFrontCorner;
    trianglesOut.push_back(temp);

    temp.p1 = rightTopBackCorner;
    temp.p2 = rightTopFrontCorner;
    temp.p3 = rightBottomBackCorner;
    trianglesOut.push_back(temp);

    // Back face triangles
    temp.p1 = rightBottomBackCorner;
    temp.p2 = leftBottomBackCorner;
    temp.p3 = rightTopBackCorner;
    trianglesOut.push_back(temp);

    temp.p1 = leftTopBackCorner;
    temp.p2 = rightTopBackCorner;
    temp.p3 = leftBottomBackCorner;
    trianglesOut.push_back(temp);

    // Left face triangles
    temp.p1 = leftBottomBackCorner;
    temp.p2 = leftBottomFrontCorner;
    temp.p3 = leftTopBackCorner;
    trianglesOut.push_back(temp);

    temp.p1 = leftTopFrontCorner;
    temp.p2 = leftTopBackCorner;
    temp.p3 = leftBottomFrontCorner;
    trianglesOut.push_back(temp);

    // Bottom face triangles
    temp.p1 = rightBottomBackCorner;
    temp.p2 = rightBottomFrontCorner;
    temp.p3 = leftBottomBackCorner;
    trianglesOut.push_back(temp);

    temp.p1 = leftBottomFrontCorner;
    temp.p2 = leftBottomBackCorner;
    temp.p3 = rightBottomFrontCorner;
    trianglesOut.push_back(temp);

    // Top face triangles
    temp.p1 = rightTopFrontCorner;
    temp.p2 = rightTopBackCorner;
    temp.p3 = leftTopFrontCorner;
    trianglesOut.push_back(temp);

    temp.p1 = leftTopBackCorner;
    temp.p2 = leftTopFrontCorner;
    temp.p3 = rightTopBackCorner;
    trianglesOut.push_back(temp);
}

void SphereEncloser::createVoxelMesh(std::vector<Triangle>& triangles, std::vector<int>& indices)
{

}

bool SphereEncloser::isInDiscreteBoundingBox(int i, int j, int k, int minx, int miny, int minz,
                                             int maxx, int maxy, int maxz)
{
    bool inside = true;
    inside &= i >= minx && i <= maxx;
    inside &= j >= miny && j <= maxy;
    inside &= k >= minz && k <= maxz;
    return inside;
}

bool SphereEncloser::intersects(const Triangle& tr1, const Triangle& tr2, double eps)
{
    // Vertices 0, 1, and 2 on triangle 1
    Eigen::Vector3d v10(tr1.p1.x, tr1.p1.y, tr1.p1.z);
    Eigen::Vector3d v11(tr1.p2.x, tr1.p2.y, tr1.p2.z);
    Eigen::Vector3d v12(tr1.p3.x, tr1.p3.y, tr1.p3.z);

    // Vertices 0, 1, and 2 on triangle 2
    Eigen::Vector3d v20(tr2.p1.x, tr2.p1.y, tr2.p1.z);
    Eigen::Vector3d v21(tr2.p2.x, tr2.p2.y, tr2.p2.z);
    Eigen::Vector3d v22(tr2.p3.x, tr2.p3.y, tr2.p3.z);

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
        if(pointOnTriangle(firstTriangleCenterTo3D, u20To3D, u21To3D, u22To3D) ||
           pointOnTriangle(secondTriangleCenterTo3D, u10To3D, u11To3D, u12To3D))
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
    /// Get the interval on the line where Triangle 1 intersects
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
    /// Get the interval on the line where Triangle 2 intersects
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

bool SphereEncloser::pointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& vertex1,
                                     const Eigen::Vector3d& vertex2, const Eigen::Vector3d& vertex3)
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

}
