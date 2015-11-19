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
#include <sbpl_geometry_utils/Triangle.h>

namespace sbpl {

/// \brief Voxelize a box at the origin
void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a box at a given pose
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

/// \brief Voxelize a sphere at the origin
void VoxelizeSphere(
    double radius,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a sphere at a given pose
void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a sphere at the origin using a specified origin for the
///     voxel grid
void VoxelizeSphere(
    double radius,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a sphere at a given pose using a specified origin for the
///     voxel grid
void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a cylinder at the origin
void VoxelizeCylinder(
    double radius, 
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a cylinder at a given pose
void VoxelizeCylinder(
    double radius, 
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a cylinder at the origin using a specified origin for the
///     voxel grid
void VoxelizeCylinder(
    double radius, 
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a cylinder at a given pose using a specified origin for the
///     voxel grid
void VoxelizeCylinder(
    double radius, 
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a mesh at the origin
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a mesh at a given pose
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a mesh at the origin using a specified origin for the voxel
///     grid
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

/// \brief Voxelize a mesh at a given pose using a specified origin for the
///     voxel grid
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<int>& indices,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill = false);

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
void VoxelizeSphereList(
    const std::vector<double>& radii,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill = false);

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
void VoxelizeSphereListQAD(
    const std::vector<double>& radii,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill = false);

} // namespace sbpl

#endif
