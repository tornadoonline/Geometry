#include "GeometryMesh.h"
#include <numeric>

namespace Geometry {

    auto CreateBox(double width, double height, double depth)->TriangleMesh 
    {
        // Check width, height, depth.
        if (width <= 0) {
        }
        if (height <= 0) {
        }
        if (depth <= 0) {
        }

        // Vertices.
        std::vector<Eigen::Vector3d> _vertices;
        std::vector<Eigen::Vector3i> _triangles;
        _vertices ={
            {0.0, 0.0, 0.0},
            {width, 0.0, 0.0},
            {0.0, 0.0, depth},
            {width, 0.0, depth},
            {0.0, height, 0.0},
            {width, height, 0.0},
            {0.0, height, depth},
            {width, height, depth}
        };

        // Triangles.
        _triangles = {
            {4, 7, 5},
            {4, 6, 7},
            {0, 2, 4},
            {2, 6, 4},
            {0, 1, 2},
            {1, 3, 2},
            {1, 5, 7},
            {1, 7, 3},
            {2, 3, 7},
            {2, 7, 6},
            {0, 4, 1},
            {1, 4, 5}
        };
        // Mesh.
        TriangleMesh mesh;
        mesh.vertices_.swap(_vertices);
        mesh.triangles_.swap(_triangles);
        return mesh;
    }


    auto CreateSphere(double radius, int resolution)->TriangleMesh
    {
        TriangleMesh mesh;
        if (radius <= 0) {
        }
        if (resolution <= 0) {
        }
        mesh.vertices_.resize(2 * resolution * (resolution - 1) + 2);

        mesh.vertices_[0] = Eigen::Vector3d(0.0, 0.0, radius);
        mesh.vertices_[1] = Eigen::Vector3d(0.0, 0.0, -radius);
        double step = M_PI / (double)resolution;
        for (int i = 1; i < resolution; i++) {
            double alpha = step * i;
            double uv_row = (1.0 / (resolution)) * i;
            int base = 2 + 2 * resolution * (i - 1);
            for (int j = 0; j < 2 * resolution; j++) {
                double theta = step * j;
                double uv_col = (1.0 / (2.0 * resolution)) * j;
                mesh.vertices_[base + j] =
                        Eigen::Vector3d(sin(alpha) * cos(theta),
                                        sin(alpha) * sin(theta), cos(alpha)) *
                        radius;
            }
        }

        // Triangles for poles.
        for (int j = 0; j < 2 * resolution; j++) {
            int j1 = (j + 1) % (2 * resolution);
            int base = 2;
            mesh.triangles_.push_back(Eigen::Vector3i(0, base + j, base + j1));
            base = 2 + 2 * resolution * (resolution - 2);
            mesh.triangles_.push_back(Eigen::Vector3i(1, base + j1, base + j));
        }
  
        // Triangles for non-polar region.
        for (int i = 1; i < resolution - 1; i++) {
            int base1 = 2 + 2 * resolution * (i - 1);
            int base2 = base1 + 2 * resolution;
            for (int j = 0; j < 2 * resolution; j++) {
                int j1 = (j + 1) % (2 * resolution);
                mesh.triangles_.push_back(
                        Eigen::Vector3i(base2 + j, base1 + j1, base1 + j));
                mesh.triangles_.push_back(
                        Eigen::Vector3i(base2 + j, base2 + j1, base1 + j1));
            }
        }
        return mesh;
    }

    auto CreateCylinder(double radius, double height, int resolution, int split)->TriangleMesh
    {
        TriangleMesh mesh;
        if (radius <= 0) {
        }
        if (height <= 0) {
        }
        if (resolution <= 0) {
        }
        if (split <= 0) {
        }
        mesh.vertices_.resize(resolution * (split + 1) + 2);
        mesh.vertices_[0] = Eigen::Vector3d(0.0, 0.0, height * 0.5);
        mesh.vertices_[1] = Eigen::Vector3d(0.0, 0.0, -height * 0.5);
        double step = M_PI * 2.0 / (double)resolution;
        double h_step = height / (double)split;
        for (int i = 0; i <= split; i++) {
            for (int j = 0; j < resolution; j++) {
                double theta = step * j;
                mesh.vertices_[2 + resolution * i + j] =
                        Eigen::Vector3d(cos(theta) * radius, sin(theta) * radius,
                                        height * 0.5 - h_step * i);
            }
        }
        // Triangles for top and bottom face.
        for (int j = 0; j < resolution; j++) {
            int j1 = (j + 1) % resolution;
            int base = 2;
            mesh.triangles_.push_back(Eigen::Vector3i(0, base + j, base + j1));
            base = 2 + resolution * split;
            mesh.triangles_.push_back(Eigen::Vector3i(1, base + j1, base + j));
        }

        // Triangles for cylindrical surface.
        for (int i = 0; i < split; i++) {
            int base1 = 2 + resolution * i;
            int base2 = base1 + resolution;
            for (int j = 0; j < resolution; j++) {
                int j1 = (j + 1) % resolution;
                mesh.triangles_.push_back(
                        Eigen::Vector3i(base2 + j, base1 + j1, base1 + j));
                mesh.triangles_.push_back(
                        Eigen::Vector3i(base2 + j, base2 + j1, base1 + j1));
            }
        }
        return mesh;
    }

    auto CreateCone(double radius, double height, int resolution, int split)->TriangleMesh
    {
        TriangleMesh mesh;
        if (radius <= 0) {
        }
        if (height <= 0) {
        }
        if (resolution <= 0) {
        }
        if (split <= 0) {
        }
        mesh.vertices_.resize(resolution * split + 2);
        mesh.vertices_[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
        mesh.vertices_[1] = Eigen::Vector3d(0.0, 0.0, height);
        double step = M_PI * 2.0 / (double)resolution;
        double h_step = height / (double)split;
        double r_step = radius / (double)split;
        for (int i = 0; i < split; i++) {
            int base = 2 + resolution * i;
            double r = r_step * (split - i);
            for (int j = 0; j < resolution; j++) {
                double theta = step * j;
                mesh.vertices_[base + j] =
                        Eigen::Vector3d(cos(theta) * r, sin(theta) * r, h_step * i);
            }
        }

        for (int j = 0; j < resolution; j++) {
            int j1 = (j + 1) % resolution;
            // Triangles for bottom surface.
            int base = 2;
            mesh.triangles_.push_back(Eigen::Vector3i(0, base + j1, base + j));

            // Triangles for top segment of conical surface.
            base = 2 + resolution * (split - 1);
            mesh.triangles_.push_back(Eigen::Vector3i(1, base + j, base + j1));
        }

        // Triangles for conical surface other than top-segment.
        for (int i = 0; i < split - 1; i++) {
            int base1 = 2 + resolution * i;
            int base2 = base1 + resolution;
            for (int j = 0; j < resolution; j++) {
                int j1 = (j + 1) % resolution;
                mesh.triangles_.push_back(
                        Eigen::Vector3i(base2 + j1, base1 + j, base1 + j1));
                mesh.triangles_.push_back(
                        Eigen::Vector3i(base2 + j1, base2 + j, base1 + j));
            }
        }
        return mesh;
    }


    auto CreateTorus(double torus_radius, double tube_radius, int radial_resolution, int tubular_resolution)->TriangleMesh 
    {
        TriangleMesh mesh;
        if (torus_radius <= 0) {
        }
        if (tube_radius <= 0) {
        }
        if (radial_resolution <= 0) {
        }
        if (tubular_resolution <= 0) {
        }

        mesh.vertices_.resize(radial_resolution * tubular_resolution);
        mesh.triangles_.resize(2 * radial_resolution * tubular_resolution);
        auto vert_idx = [&](int uidx, int vidx) {
            return uidx * tubular_resolution + vidx;
        };
        double u_step = 2 * M_PI / double(radial_resolution);
        double v_step = 2 * M_PI / double(tubular_resolution);
        for (int uidx = 0; uidx < radial_resolution; ++uidx) {
            double u = uidx * u_step;
            Eigen::Vector3d w(cos(u), sin(u), 0);
            for (int vidx = 0; vidx < tubular_resolution; ++vidx) {
                double v = vidx * v_step;
                mesh.vertices_[vert_idx(uidx, vidx)] =
                        torus_radius * w + tube_radius * cos(v) * w +
                        Eigen::Vector3d(0, 0, tube_radius * sin(v));

                int tri_idx = (uidx * tubular_resolution + vidx) * 2;
                mesh.triangles_[tri_idx + 0] = Eigen::Vector3i(
                        vert_idx((uidx + 1) % radial_resolution, vidx),
                        vert_idx((uidx + 1) % radial_resolution,
                                (vidx + 1) % tubular_resolution),
                        vert_idx(uidx, vidx));
                mesh.triangles_[tri_idx + 1] = Eigen::Vector3i(
                        vert_idx(uidx, vidx),
                        vert_idx((uidx + 1) % radial_resolution,
                                (vidx + 1) % tubular_resolution),
                        vert_idx(uidx, (vidx + 1) % tubular_resolution));
            }
        }

        return mesh;
    }

    auto CreateArrow(
        double cylinder_radius, double cone_radius, double cylinder_height, double cone_height,
        int resolution, int cylinder_split, int cone_split)->TriangleMesh
    {
        if (cylinder_radius <= 0) {
        }
        if (cone_radius <= 0) {
        }
        if (cylinder_height <= 0) {
        }
        if (cone_height <= 0) {
        }
        if (resolution <= 0) {
        }
        if (cylinder_split <= 0) {
        }
        if (cone_split <= 0) {
        }
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        auto mesh_cylinder = CreateCylinder(cylinder_radius, cylinder_height,
                                            resolution, cylinder_split);
        transformation(2, 3) = cylinder_height * 0.5;
        mesh_cylinder.Transform(transformation);
        auto mesh_cone =
                CreateCone(cone_radius, cone_height, resolution, cone_split);
        transformation(2, 3) = cylinder_height;
        mesh_cone.Transform(transformation);
        auto mesh_arrow = mesh_cylinder;
        mesh_arrow += mesh_cone;
        return mesh_arrow;
    }

    auto CreateMobius(int length_split, int width_split, int twists,
        double radius, double flatness, double width, double scale)->TriangleMesh
    {
        TriangleMesh mesh;
        if (length_split <= 0) {
        }
        if (width_split <= 0) {
        }
        if (twists < 0) {
        }
        if (radius <= 0) {
        }
        if (flatness == 0) {
        }
        if (width <= 0) {
        }
        if (scale <= 0) {
        }

        mesh.vertices_.resize(length_split * width_split);

        double u_step = 2 * M_PI / length_split;
        double v_step = width / (width_split - 1);
        for (int uidx = 0; uidx < length_split; ++uidx) {
            double u = uidx * u_step;
            double cos_u = std::cos(u);
            double sin_u = std::sin(u);
            for (int vidx = 0; vidx < width_split; ++vidx) {
                int idx = uidx * width_split + vidx;
                double v = -width / 2.0 + vidx * v_step;
                double alpha = twists * 0.5 * u;
                double cos_alpha = std::cos(alpha);
                double sin_alpha = std::sin(alpha);
                mesh.vertices_[idx](0) =
                        scale * ((cos_alpha * cos_u * v) + radius * cos_u);
                mesh.vertices_[idx](1) =
                        scale * ((cos_alpha * sin_u * v) + radius * sin_u);
                mesh.vertices_[idx](2) = scale * sin_alpha * v * flatness;
            }
        }

        for (int uidx = 0; uidx < length_split - 1; ++uidx) {
            for (int vidx = 0; vidx < width_split - 1; ++vidx) {
                if ((uidx + vidx) % 2 == 0) {
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx,
                                            (uidx + 1) * width_split + vidx + 1,
                                            uidx * width_split + vidx + 1));
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx,
                                            (uidx + 1) * width_split + vidx,
                                            (uidx + 1) * width_split + vidx + 1));
                } else {
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx + 1,
                                            uidx * width_split + vidx,
                                            (uidx + 1) * width_split + vidx));
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx + 1,
                                            (uidx + 1) * width_split + vidx,
                                            (uidx + 1) * width_split + vidx + 1));
                }
            }
        }

        int uidx = length_split - 1;
        for (int vidx = 0; vidx < width_split - 1; ++vidx) {
            if (twists % 2 == 1) {
                if ((uidx + vidx) % 2 == 0) {
                    mesh.triangles_.push_back(
                            Eigen::Vector3i((width_split - 1) - (vidx + 1),
                                            uidx * width_split + vidx,
                                            uidx * width_split + vidx + 1));
                    mesh.triangles_.push_back(Eigen::Vector3i(
                            (width_split - 1) - vidx, uidx * width_split + vidx,
                            (width_split - 1) - (vidx + 1)));
                } else {
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx,
                                            uidx * width_split + vidx + 1,
                                            (width_split - 1) - vidx));
                    mesh.triangles_.push_back(Eigen::Vector3i(
                            (width_split - 1) - vidx, uidx * width_split + vidx + 1,
                            (width_split - 1) - (vidx + 1)));
                }
            } else {
                if ((uidx + vidx) % 2 == 0) {
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx, vidx + 1,
                                            uidx * width_split + vidx + 1));
                    mesh.triangles_.push_back(Eigen::Vector3i(
                            uidx * width_split + vidx, vidx, vidx + 1));
                } else {
                    mesh.triangles_.push_back(
                            Eigen::Vector3i(uidx * width_split + vidx, vidx,
                                            uidx * width_split + vidx + 1));
                    mesh.triangles_.push_back(Eigen::Vector3i(
                            uidx * width_split + vidx + 1, vidx, vidx + 1));
                }
            }
        }
        return mesh;
    }

    auto CreateTetrahedron(double radius)->TriangleMesh
    {
        TriangleMesh mesh;
        if (radius <= 0) {
        }

        // Vertices.
        mesh.vertices_.push_back(radius *
                                Eigen::Vector3d(std::sqrt(8. / 9.), 0, -1. / 3.));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-std::sqrt(2. / 9.),
                                                        std::sqrt(2. / 3.),
                                                        -1. / 3.));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-std::sqrt(2. / 9.),
                                                        -std::sqrt(2. / 3.),
                                                        -1. / 3.));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0., 0., 1.));

        // Triangles.
        mesh.triangles_ = {
                {0, 2, 1},
                {0, 3, 2},
                {0, 1, 3},
                {1, 2, 3},
        };
        return mesh;
    }

    auto CreateOctahedron(double radius)->TriangleMesh 
    {
        TriangleMesh mesh;
        if (radius <= 0) {
        }

        // Vertices.
        mesh.vertices_.push_back(radius * Eigen::Vector3d(1, 0, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, 1, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, 0, 1));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-1, 0, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, -1, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, 0, -1));

        // Triangles.
        mesh.triangles_ = {{0, 1, 2}, {1, 3, 2}, {3, 4, 2}, {4, 0, 2},
                            {0, 5, 1}, {1, 5, 3}, {3, 5, 4}, {4, 5, 0}};

        return mesh;
    }

    auto CreateIcosahedron(double radius)->TriangleMesh 
    {
        TriangleMesh mesh;
        if (radius <= 0) {
        }
        const double p = (1. + std::sqrt(5.)) / 2.;

        // Vertices.
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-1, 0, p));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(1, 0, p));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(1, 0, -p));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-1, 0, -p));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, -p, 1));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, p, 1));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, p, -1));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(0, -p, -1));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-p, -1, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(p, -1, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(p, 1, 0));
        mesh.vertices_.push_back(radius * Eigen::Vector3d(-p, 1, 0));

        // Triangles.
        mesh.triangles_ = {{0, 4, 1},  {0, 1, 5},  {1, 4, 9},  {1, 9, 10},
                            {1, 10, 5}, {0, 8, 4},  {0, 11, 8}, {0, 5, 11},
                            {5, 6, 11}, {5, 10, 6}, {4, 8, 7},  {4, 7, 9},
                            {3, 6, 2},  {3, 2, 7},  {2, 6, 10}, {2, 10, 9},
                            {2, 9, 7},  {3, 11, 6}, {3, 8, 11}, {3, 7, 8}};

        return mesh;
    }

}