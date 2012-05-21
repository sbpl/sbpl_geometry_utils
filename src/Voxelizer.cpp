#include <sbpl_geometry_utils/Voxelizer.h>

namespace sbpl
{

void Voxelizer::voxelizeBox(double xSize, double ySize, double zSize, double voxelSize,
                            std::vector<std::vector<double> >& voxels)
{
    return; // TODO
}

void Voxelizer::voxelizeBox(double xSize, double ySize, double zSize, const geometry_msgs::Pose& pose, double voxelSize,
                            std::vector<std::vector<double> >& voxels)
{
    return; // TODO
}

void Voxelizer::voxelizeSphere(double radius, double voxelSize, std::vector<std::vector<double> >& voxels)
{
    return; // TODO
}

void Voxelizer::voxelizeSphere(double radius, const geometry_msgs::Pose& pose, double voxelSize,
                               std::vector<std::vector<double> >& voxels)
{
    return; // TODO
}

void Voxelizer::voxelizeCylinder(double cylinderRadius, double length, double voxelSize,
                                 std::vector<std::vector<double> >& voxels)
{
    return; // TODO
}

void Voxelizer::voxelizeCylinder(double cylinderRadius, double length, const geometry_msgs::Pose& pose,
                                 double voxelSize, std::vector<std::vector<double> >& voxels)
{
    return; // TODO
}

void Voxelizer::voxelizeMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                             double voxelSize, std::vector<std::vector<double> >& voxels, bool fillMesh, int maxVoxels)
{
    return; // TODO
}

void Voxelizer::voxelizeMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                             const geometry_msgs::Pose& pose, double voxelSize,
                             std::vector<std::vector<double> >& voxels, bool fillMesh, int maxVoxels)
{
    return; // TODO
}

}
