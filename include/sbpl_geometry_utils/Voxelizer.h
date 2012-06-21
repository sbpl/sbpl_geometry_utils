#ifndef SBPL_GEOMETRY_UTILS_VOXELIZER_H
#define SBPL_GEOMETRY_UTILS_VOXELIZER_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sbpl_geometry_utils/Triangle.h>

namespace sbpl
{

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
	                        std::vector<std::vector<double> >& voxels);

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
	                        std::vector<std::vector<double> >& voxels);

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
	static void voxelizeSphere(double radius, double voxelSize, std::vector<std::vector<double> >& voxels);

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
	                           std::vector<std::vector<double> >& voxels);

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
	                             std::vector<std::vector<double> >& voxels);

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
	                             double voxelSize, std::vector<std::vector<double> >& voxels);

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
	 * @param[in] spheres
	 * @param[in] res
	 * @param[in] removeDuplicates
	 * @param[out] voxels              The vector in which to store the voxels
	 * @param[out] volume              The combined volume of all the spheres
	 */
	static void voxelizeSphereList(const std::vector<std::vector<double> >& spheres, double res, bool removeDuplicates,
	                               std::vector<std::vector<double> >& voxels, double& volume);
private:
	Voxelizer();

	static void createVoxelMesh(std::vector<Triangle>& triangles, std::vector<int>& indices);
	static void createSphereMesh(const std::vector<double>& sphere, int numLongitudes, int numLatitudes,
                                 std::vector<geometry_msgs::Point>& vertices, std::vector<int>& triangles);
	static bool getAxisAlignedBoundingBox(const std::vector<geometry_msgs::Point>& vertices, double& minX,
                                          double& minY, double& minZ, double& maxX, double& maxY, double& maxZ);
	static void createCubeMesh(double x, double y, double z, double length, std::vector<Triangle>& trianglesOut);
    static bool isInDiscreteBoundingBox(int i, int j, int k, int minx, int miny, int minz,
                                        int maxx, int maxy, int maxz);
    static bool intersects(const Triangle& tr1, const Triangle& tr2, double eps = 1.0e-4);
    static bool pointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& vertex1,
                                const Eigen::Vector3d& vertex2, const Eigen::Vector3d& vertex3);
};

}

#endif
