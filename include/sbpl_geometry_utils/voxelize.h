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

#ifndef sbpl_Voxelizer_h
#define sbpl_Voxelizer_h

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>

// project includes
#include <sbpl_geometry_utils/triangle.h>
#include <sbpl_geometry_utils/voxel_grid.h>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl {

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

void VoxelizePlane(
    double a, double b, double c, double d,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    double res,
    std::vector<Eigen::Vector3d>& voxels);

void VoxelizePlane(
    double a, double b, double c, double d,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels);

void VoxelizeSphereList(
    const std::vector<double>& radii,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill = false);

void VoxelizeSphereListQAD(
    const std::vector<double>& radii,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill = false);

bool ComputeAxisAlignedBoundingBox(
    const std::vector<Eigen::Vector3d>& vertices,
    Eigen::Vector3d& min,
    Eigen::Vector3d& max);

double Distance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& q,
    double radius_sqrd,
    const Eigen::Vector3d& x);

template <typename Discretizer>
void VoxelizeTriangle(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    VoxelGrid<Discretizer>& vg);

} // namespace sbpl

#include "detail/voxelize.h"

#endif
