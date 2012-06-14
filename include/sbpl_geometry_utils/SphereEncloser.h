#ifndef SBPL_GEOMETRY_UTILS_SPHERE_ENCLOSER_H
#define SBPL_GEOMETRY_UTILS_SPHERE_ENCLOSER_H

#include <vector>
#include <sbpl_geometry_utils/Sphere.h>
#include <sbpl_geometry_utils/Triangle.h>
#include <geometry_msgs/Pose.h>

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
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten
     */
    static void encloseBox(double xSize, double ySize, double zSize, double radius, std::vector<Sphere>& spheres);

    /**
     * @brief Encloses a generic box with a set of spheres of a given radius.
     *
     * Encloses a box with a set of spheres of a given radius. The box is allowed any generic pose.
     *
     * @param[in] xSize,ySize,zSize The lengths of all the sides of the box
     * @param[in] pose The position and orientation of the box
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten
     */
    static void encloseBox(double xSize, double ySize, double zSize, const geometry_msgs::Pose& pose, double radius,
                           std::vector<Sphere>& spheres);

    /**
     * @brief Encloses an axis-aligned cylinder with a set of spheres of a given radius.
     *
     * Encloses a cylinder with a set of spheres of a given radius. The cylinder is assumed to be centered on the
     * origin and the cylinder's axis is the z-axis.
     *
     * @param[in] cylinderRadius,length The dimensions of the cylinder
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten
     */
    static void encloseCylinder(double cylinderRadius, double length, double radius, std::vector<Sphere>& spheres);

    /**
     * @brief Encloses a generic cylinder with a set of spheres of a given radius.
     *
     * Encloses a cylinder with a set of spheres of a given radius. The cylinder is allowed any generic pose.
     *
     * @param[in] cylinderRadius,length The dimensions of the cylinder
     * @param[in] pose The position and orientation of the cylinder
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten
     */
    static void encloseCylinder(double cylinderRadius, double length, const geometry_msgs::Pose& pose,
			                    double radius, std::vector<Sphere>& spheres);

    /**
     * @brief Encloses a mesh with a set of spheres of a given radius
     *
     * Encloses a mesh with a set of spheres of a given radius. The generated spheres appear in whatever frame the
     * vertices and triangles of the mesh are defined in.
     *
     * @param[in] vertices The vertices of the mesh
     * @param[in] triangles The triangles of the mesh as a list of indices into the vertex array
     * @param[in] radius The desired radius of the generated spheres
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten
     * @param[in] fillMesh Whether to fill the interior of the mesh with spheres
     * @param[in] maxSpheres An upper bound on the number of generated spheres; will use a larger radius than $radius
     *                       if necessary to reduce the number of spheres generated
     */
    static void encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                            double radius, std::vector<Sphere>& spheres, bool fillMesh = false, int maxSpheres = 0);

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
     * @param[out] spheres The vector in which to store the spheres; any previous data is overwritten
     * @param[in] fillMesh Whether to fill the interior of the mesh with spheres
     * @param[in] maxSpheres An upper bound on the number of generated spheres; will use a larger radius than $radius
     *                       if necessary to reduce the number of spheres generated
     */
    static void encloseMesh(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
	                        const geometry_msgs::Pose& pose, double radius, std::vector<Sphere>& spheres,
	                        bool fillMesh = false, int maxSpheres = 0);

	/******************** Interface functions without Sphere usage ********************/

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
    static bool isInDiscreteBoundingBox(int i, int j, int k, int minx, int miny, int minz,
                                        int maxx, int maxy, int maxz);
    static bool intersects(const Triangle& tr1, const Triangle& tr2, double eps = 1.0e-4);
    static Eigen::Vector3d cross(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    static bool pointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& vertex1,
                                const Eigen::Vector3d& vertex2, const Eigen::Vector3d& vertex3);
};

}

#endif
