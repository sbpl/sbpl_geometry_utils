//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#include <sbpl_geometry_utils/mesh_utils.h>

namespace sbpl {

/// \brief Create a mesh representation of box
/// The box mesh is axis-aligned and located at the origin.
void CreateIndexedBoxMesh(
    double length,
    double width,
    double height,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    vertices.reserve(vertices.size() + 8);
    indices.reserve(indices.size() + 36);

    // l = left, r = right, t = top, b = bottom/back, f = front

    Eigen::Vector3d rtb_corner;
    rtb_corner.x() =  0.5 * length;
    rtb_corner.y() =  0.5 * width;
    rtb_corner.z() = -0.5 * height;

    Eigen::Vector3d ltb_corner;
    ltb_corner.x() = -0.5 * length;
    ltb_corner.y() =  0.5 * width;
    ltb_corner.z() = -0.5 * height;

    Eigen::Vector3d ltf_corner;
    ltf_corner.x() = -0.5 * length;
    ltf_corner.y() =  0.5 * width;
    ltf_corner.z() =  0.5 * height;

    Eigen::Vector3d rtf_corner;
    rtf_corner.x() = 0.5 * length;
    rtf_corner.y() = 0.5 * width;
    rtf_corner.z() = 0.5 * height;

    Eigen::Vector3d rbb_corner;
    rbb_corner.x() =  0.5 * length;
    rbb_corner.y() = -0.5 * width;
    rbb_corner.z() = -0.5 * height;

    Eigen::Vector3d lbb_corner;
    lbb_corner.x() = -0.5 * length;
    lbb_corner.y() = -0.5 * width;
    lbb_corner.z() = -0.5 * height;

    Eigen::Vector3d lbf_corner;
    lbf_corner.x() = -0.5 * length;
    lbf_corner.y() = -0.5 * width;
    lbf_corner.z() =  0.5 * height;

    Eigen::Vector3d rbf_corner;
    rbf_corner.x() =  0.5 * length;
    rbf_corner.y() = -0.5 * width;
    rbf_corner.z() =  0.5 * height;

    vertices.push_back(lbb_corner);
    vertices.push_back(rbb_corner);
    vertices.push_back(ltb_corner);
    vertices.push_back(rtb_corner);
    vertices.push_back(lbf_corner);
    vertices.push_back(rbf_corner);
    vertices.push_back(ltf_corner);
    vertices.push_back(rtf_corner);

    indices.push_back(0); indices.push_back(2); indices.push_back(1); // back faces
    indices.push_back(1); indices.push_back(2); indices.push_back(3);
    indices.push_back(5); indices.push_back(1); indices.push_back(7); // right face
    indices.push_back(1); indices.push_back(3); indices.push_back(7);
    indices.push_back(5); indices.push_back(7); indices.push_back(4); // front face
    indices.push_back(4); indices.push_back(7); indices.push_back(6);
    indices.push_back(4); indices.push_back(6); indices.push_back(0); // left face
    indices.push_back(0); indices.push_back(6); indices.push_back(2);
    indices.push_back(6); indices.push_back(7); indices.push_back(2); // top face
    indices.push_back(7); indices.push_back(3); indices.push_back(2);
    indices.push_back(5); indices.push_back(0); indices.push_back(1); // bottom face
    indices.push_back(4); indices.push_back(5); indices.push_back(0);
}

/// \brief Create a mesh representation of a sphere
/// The sphere is located at the origin
void CreateIndexedSphereMesh(
    double radius,
    int longitude_count,
    int latitude_count,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    // TODO: handle the case where there is only one line of longitude and thus
    // there are no quadrilaterals to break up into two triangles and the method
    // for getting the indices of those triangles breaks

    vertices.reserve(vertices.size() + 2 + latitude_count * longitude_count);
    indices.reserve(indices.size() + 6 * longitude_count + 2 * (latitude_count - 1) * longitude_count);

    // create the top vertex
    Eigen::Vector3d northPole(0.0, 0.0, radius);
    vertices.push_back(northPole);

    // create the intermediate vertices
    double theta_inc = M_PI / (latitude_count + 1);
    double phi_inc = (2.0 * M_PI) / longitude_count;
    for (int tidx = 0; tidx < latitude_count; ++tidx) {
        for (int pidx = 0; pidx < longitude_count; ++pidx) {
            double theta = (tidx + 1) * theta_inc;
            double phi = pidx * phi_inc;

            double x = radius * sin(theta) * cos(phi);
            double y = radius * sin(theta) * sin(phi);
            double z = radius * cos(theta);
            vertices.emplace_back(x, y, z);
        }
    }

    // create the bottom vertex
    Eigen::Vector3d southPole(0.0, 0.0, -radius);
    vertices.push_back(southPole);

    // add all the triangles with the north pole as a vertex
    for (int i = 0; i < longitude_count; i++) {
        // add in top triangle
        indices.push_back(0);
        indices.push_back(i + 1);
        if (i == longitude_count - 1) {
            indices.push_back(1);
        }
        else {
            indices.push_back(i + 2);
        }
    }

    // add all intermediate triangles
    for (int i = 0; i < latitude_count - 1; i++) {
        for (int j = 0; j < longitude_count; j++) {
            // i, j corresponds to one of the generated vertices
            int baseVertexIdx = i * longitude_count + j + 1;
            int bBaseVertexIdx = baseVertexIdx + longitude_count;
            int brBaseVertexIdx = bBaseVertexIdx + 1;
            int rBaseVertexIdx = baseVertexIdx + 1;

            if ((brBaseVertexIdx - 1)/longitude_count != (bBaseVertexIdx - 1)/longitude_count) {
                brBaseVertexIdx -= longitude_count;
            }

            if ((rBaseVertexIdx - 1)/longitude_count != (baseVertexIdx - 1)/longitude_count) {
                rBaseVertexIdx -= longitude_count;
            }

            indices.push_back(baseVertexIdx);
            indices.push_back(bBaseVertexIdx);
            indices.push_back(brBaseVertexIdx);

            indices.push_back(baseVertexIdx);
            indices.push_back(brBaseVertexIdx);
            indices.push_back(rBaseVertexIdx);
        }
    }

    // add all the triangles with the south pole as a vertex
    for (int i = 0; i < longitude_count; i++) {
        indices.push_back(vertices.size() - 1);
        if (i == 0) {
            indices.push_back(vertices.size() - 1 - longitude_count);
        }
        else {
            indices.push_back(vertices.size() - 1 - i);
        }
        indices.push_back(vertices.size() - 1 - (i + 1));
    }
}

/// \brief Create a mesh representation of an upright (z-aligned) cylinder
/// The cylinder is located at the origin.
void CreateIndexedCylinderMesh(
    double radius,
    double length,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    const int rim_count = 16;

    // add vertices for the top cap
    for (int i = 0; i < rim_count; i++) {
        double theta = 2.0 * M_PI * (double)i / double(rim_count);
        Eigen::Vector3d p(
                radius * cos(theta),
                radius * sin(theta),
                0.5 * length);
        vertices.push_back(p);
    }

    // add vertices for the bottom cap
    for (int i = 0; i < rim_count; i++) {
        double theta = 2.0 * M_PI * (double)i / double(rim_count);
        Eigen::Vector3d p(
                radius * cos(theta),
                radius * sin(theta),
                -0.5 * length);
        vertices.push_back(p);
    }

    // add top and bottom cap center vertices
    vertices.emplace_back(0.0, 0.0, 0.5 * length);
    vertices.emplace_back(0.0, 0.0, -0.5 * length);

    // create the sides
    for (int i = 0; i < rim_count; i++) {
        indices.push_back(i);
        indices.push_back((i + 1) % rim_count);
        indices.push_back(i + rim_count);

        indices.push_back((i + 1) % rim_count);
        indices.push_back(((i + 1) % rim_count) + rim_count);
        indices.push_back(i + rim_count);
    }

    // create the top fan
    for (int i = 0; i < rim_count; ++i) {
        indices.push_back(2 * rim_count);
        indices.push_back((i + 1) % rim_count);
        indices.push_back(i);
    }

    // create the bottom fan
    for (int i = 0; i < rim_count; ++i) {
        indices.push_back(2 * rim_count + 1);
        indices.push_back((i + 1) % rim_count + rim_count);
        indices.push_back(i + rim_count);
    }
}

/// \brief Create a mesh representation of an upright (z-aligned) cone
/// The cone is located at the origin
void CreateIndexedConeMesh(
    double radius,
    double height,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    const int rim_count = 16;
    const double bottom_z = -0.5 * height;
    const double top_z = 0.5 * height;

    for (int i = 0; i < rim_count; ++i) {
        double theta = 2.0 * M_PI * (double)i / (double)rim_count;
        const double x = radius * cos(theta);
        const double y = radius * sin(theta);
        vertices.emplace_back(x, y, bottom_z);
    }
    vertices.push_back(Eigen::Vector3d(0.0, 0.0, top_z));
    vertices.emplace_back(0.0, 0.0, bottom_z);

    for (int i = 0; i < rim_count; ++i) {
        indices.push_back(i);
        indices.push_back((i + 1) % rim_count);
        indices.push_back(rim_count);
    }

    // create bottom fan
    for (int i = 0; i < rim_count; ++i) {
        indices.push_back(i);
        indices.push_back((i + 1) % rim_count);
        indices.push_back(rim_count + 1);
    }
}

/// Construct a mesh of a plane, clipped by an axis-aligned bounding box
void CreateIndexedPlaneMesh(
    double a, double b, double c, double d,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<int>& indices)
{
    Eigen::Vector3d corners[8] =
    {
        Eigen::Vector3d(min.x(), min.y(), min.z()),
        Eigen::Vector3d(min.x(), min.y(), max.z()),
        Eigen::Vector3d(min.x(), max.y(), min.z()),
        Eigen::Vector3d(min.x(), max.y(), max.z()),
        Eigen::Vector3d(max.x(), min.y(), min.z()),
        Eigen::Vector3d(max.x(), min.y(), max.z()),
        Eigen::Vector3d(max.x(), max.y(), min.z()),
        Eigen::Vector3d(max.x(), max.y(), max.z()),
    };

    bool corner_isxn[8] =
    {
        false, false, false, false, false, false, false, false
    };

    for (size_t i = 0; i < 8; ++i) {
        double dp = a * corners[i].x() + b * corners[i].y() + c * corners[i].z() + d;
        corner_isxn[i] = (dp == 0.0);
    }

    double x, y, z;

    // length edges

    x = -(b * min.y() + c * min.z() + d) / a;
    if (x >= min.x() && x <= max.x()) {
        vertices.emplace_back(x, min.y(), min.z());
    }

    x = -(b * min.y() + c * max.z() + d) / a;
    if (x >= min.x() && x <= max.x()) {
        vertices.emplace_back(x, min.y(), max.z());
    }

    x = -(b * max.y() + c * min.z() + d) / a;
    if (x >= min.x() && x <= max.x()) {
        vertices.emplace_back(x, max.y(), min.z());
    }

    x = -(b * max.y() + c * max.z() + d) / a;
    if (x >= min.x() && x <= max.x()) {
        vertices.emplace_back(x, max.y(), max.z());
    }

    // width edges

    y = -(a * min.x() + c * min.z() + d) / b;
    if (y >= min.y() && y <= max.y()) {
        vertices.emplace_back(min.x(), y, min.z());
    }

    y = -(a * min.x() + c * max.z() + d) / b;
    if (y >= min.y() && y <= max.y()) {
        vertices.emplace_back(min.x(), y, max.z());
    }

    y = -(a * max.x() + c * min.z() + d) / b;
    if (y >= min.y() && y <= max.y()) {
        vertices.emplace_back(max.x(), y, min.z());
    }

    y = -(a * max.x() + c * max.z() + d) / b;
    if (y >= min.y() && y <= max.y()) {
        vertices.emplace_back(max.x(), y, max.z());
    }

    // height edges

    z = -(a * min.x() + b * min.y() + d) / c;
    if (z >= min.z() && z <= max.z()) {
        vertices.emplace_back(min.x(), min.y(), z);
    }

    z = -(a * min.x() + b * max.y() + d) / c;
    if (z >= min.z() && z <= max.z()) {
        vertices.emplace_back(min.x(), max.y(), z);
    }

    z = -(a * max.x() + b * min.y() + d) / c;
    if (z >= min.z() && z <= max.z()) {
        vertices.emplace_back(max.x(), min.y(), z);
    }

    z = -(a * max.x() + b * max.y() + d) / c;
    if (z >= min.z() && z <= max.z()) {
        vertices.emplace_back(max.x(), max.y(), z);
    }

    if (vertices.size() < 3) {
        return;
    }

    // compute the centroid of all intersection vertices
    Eigen::Vector3d centroid(Eigen::Vector3d::Zero());
    for (const Eigen::Vector3d& v : vertices) {
        centroid += v;
    }
    if (!vertices.empty()) {
        centroid /= (double)vertices.size();
    }

    // define an orientation frame
    Eigen::Vector3d zaxis(a, b, c);
    Eigen::AngleAxisd aa(M_PI / 2.0, Eigen::Vector3d::UnitY());
    Eigen::Vector3d xaxis = aa * zaxis;
    Eigen::Vector3d yaxis = zaxis.cross(xaxis);

    // sort counter clockwise around plane normal rooted at centroid
    std::sort(vertices.begin(), vertices.end(),
        [&](const Eigen::Vector3d& u, const Eigen::Vector3d& v)
        {
            double ux = xaxis.dot(u - centroid);
            double uy = yaxis.dot(u - centroid);
            double vx = xaxis.dot(v - centroid);
            double vy = yaxis.dot(v - centroid);
            double th1 = std::atan2(uy, ux);
            double th2 = std::atan2(vy, vx);
            return th1 < th2;
        });

    // create triangle fan from the first vertex
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);
    for (size_t i = 3; i < vertices.size(); ++i) {
        indices.push_back(0);
        indices.push_back(i- 1);
        indices.push_back(i);
    }
}

/// \brief Create a non-indexed mesh representation of a box
void CreateBoxMesh(
    double length,
    double width,
    double height,
    std::vector<Eigen::Vector3d>& vertices)
{
    vertices.reserve(vertices.size() + 48);

    Eigen::Vector3d a(-0.5 * length, -0.5 * width, -0.5 * height);
    Eigen::Vector3d b(-0.5 * length, -0.5 * width,  0.5 * height);
    Eigen::Vector3d c(-0.5 * length,  0.5 * width, -0.5 * height);
    Eigen::Vector3d d(-0.5 * length,  0.5 * width,  0.5 * height);
    Eigen::Vector3d e( 0.5 * length, -0.5 * width, -0.5 * height);
    Eigen::Vector3d f( 0.5 * length, -0.5 * width,  0.5 * height);
    Eigen::Vector3d g( 0.5 * length,  0.5 * width, -0.5 * height);
    Eigen::Vector3d h( 0.5 * length,  0.5 * width,  0.5 * height);

    // back face
    vertices.push_back(a);
    vertices.push_back(b);
    vertices.push_back(d);
    vertices.push_back(a);
    vertices.push_back(d);
    vertices.push_back(c);

    // left face
    vertices.push_back(a);
    vertices.push_back(f);
    vertices.push_back(b);
    vertices.push_back(a);
    vertices.push_back(e);
    vertices.push_back(f);

    // bottom face
    vertices.push_back(a);
    vertices.push_back(g);
    vertices.push_back(e);
    vertices.push_back(a);
    vertices.push_back(c);
    vertices.push_back(g);

    // front face?
    vertices.push_back(f);
    vertices.push_back(g);
    vertices.push_back(e);
    vertices.push_back(f);
    vertices.push_back(h);
    vertices.push_back(g);

    // right face?
    vertices.push_back(h);
    vertices.push_back(d);
    vertices.push_back(c);
    vertices.push_back(h);
    vertices.push_back(c);
    vertices.push_back(g);

    // top face?
    vertices.push_back(b);
    vertices.push_back(d);
    vertices.push_back(h);
    vertices.push_back(b);
    vertices.push_back(h);
    vertices.push_back(f);
}

} // namespace sbpl
