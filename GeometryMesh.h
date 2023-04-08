#pragma once
#include <Eigen/Core>

#include <vector>
#include <iostream>


namespace Geometry {

	struct TriangleMesh{
		std::vector<Eigen::Vector3d> vertices_;
		std::vector<Eigen::Vector3i> triangles_;

		void Print()
		{
			std::cout <<"Point:\n";
			int idx = 0;
			for(auto& _ver : vertices_){
				for(auto& _p : _ver) std::cout << _p << " " ;
				if(idx%3 == 0)std::cout << "\n";
			}

			std::cout <<"Index:\n";
			idx = 0;
			for(auto& _tri : triangles_){
				for(auto& _p : _tri) std::cout << _p << " " ;
				if(idx%3 == 0)std::cout << "\n";
			}
			std::cout << std::endl;
		}


		void TransformPoints(const Eigen::Matrix4d& transformation, std::vector<Eigen::Vector3d>& points) const {
			for (auto& point : points) {
				Eigen::Vector4d new_point =
						transformation *
						Eigen::Vector4d(point(0), point(1), point(2), 1.0);
				point = new_point.head<3>() / new_point(3);
			}
		}

		TriangleMesh& Transform(const Eigen::Matrix4d &transformation) {
			TransformPoints(transformation, vertices_);
			return *this;
		}

		TriangleMesh& operator+=(const TriangleMesh &mesh) {
			if (mesh.vertices_.size() == 0) return (*this);
			size_t old_vert_num = vertices_.size();
			size_t add_vert_num = mesh.vertices_.size();
			size_t new_vert_num = old_vert_num + add_vert_num;
			vertices_.resize(new_vert_num);
			for (size_t i = 0; i < add_vert_num; i++)
				vertices_[old_vert_num + i] = mesh.vertices_[i];

			
			size_t old_tri_num = triangles_.size();
			size_t add_tri_num = mesh.triangles_.size();
			size_t new_tri_num = old_tri_num + add_tri_num;

			triangles_.resize(triangles_.size() + mesh.triangles_.size());
			Eigen::Vector3i index_shift((int)old_vert_num, (int)old_vert_num,
										(int)old_vert_num);
			for (size_t i = 0; i < add_tri_num; i++) {
				triangles_[old_tri_num + i] = mesh.triangles_[i] + index_shift;
			}
			return (*this);
		}

		template<class T=float>
		auto verticesCast()
		{
			typedef typename Eigen::Vector<T, 3> V3T; 
			std::vector<V3T> _vertices;
			for(auto& _ver : vertices_) {
				_vertices.push_back(_ver.cast<T>());
			}
			return _vertices; 
		}
	};

    /// The left bottom corner on the front will be placed at (0, 0, 0).
    /// \param width is x-directional length.
    /// \param height is y-directional length.
    /// \param depth is z-directional length.
    auto CreateBox(
            double width = 1.0,
            double height = 1.0,
            double depth = 1.0)->TriangleMesh;

    /// The sphere with radius will be centered at (0, 0, 0).
    /// Its axis is aligned with z-axis.
    /// \param radius defines radius of the sphere.
    /// \param resolution defines the resolution of the sphere. The longitudes
    /// will be split into resolution segments (i.e. there are resolution + 1
    /// latitude lines including the north and south pole). The latitudes will
    /// be split into `2 * resolution segments (i.e. there are 2 * resolution
    /// longitude lines.)
    auto CreateSphere(
            double radius = 1.0,
            int resolution = 20)->TriangleMesh;

    /// The axis of the cylinder will be from (0, 0, -height/2) to (0, 0,
    /// height/2). The circle with radius will be split into
    /// resolution segments. The height will be split into split
    /// segments.
    /// \param radius defines the radius of the cylinder.
    /// \param height defines the height of the cylinder.
    /// \param resolution defines that the circle will be split into resolution
    /// segments.
    /// \param split defines that the height will be split into split segments.
    auto CreateCylinder(
            double radius = 1.0,
            double height = 2.0,
            int resolution = 20,
            int split = 4)->TriangleMesh;

    /// The axis of the cone will be from (0, 0, 0) to (0, 0, height).
    /// The circle with radius will be split into resolution
    /// segments. The height will be split into split segments.
    /// \param radius defines the radius of the cone.
    /// \param height defines the height of the cone.
    /// \param resolution defines that the circle will be split into resolution
    /// segments.
    /// \param split defines that the height will be split into split segments.
    auto CreateCone(double radius = 1.0,
					double height = 2.0,
					int resolution = 20,
					int split = 1)->TriangleMesh;

    /// The torus will be centered at (0, 0, 0) and a radius of
    /// torus_radius. The tube of the torus will have a radius of
    /// tube_radius. The number of segments in radial and tubular direction are
    /// radial_resolution and tubular_resolution respectively.
    ///
    /// \param torus_radius defines the radius from the center of the torus to
    /// the center of the tube.
    /// \param tube_radius defines the radius of the torus tube.
    /// \param radial_resolution defines the he number of segments along the
    /// radial direction.
    /// \param tubular_resolution defines the number of segments along the
    /// tubular direction.
    auto CreateTorus(
            double torus_radius = 1.0,
            double tube_radius = 0.5,
            int radial_resolution = 30,
            int tubular_resolution = 20)->TriangleMesh;

    /// The axis of the cone with cone_radius will be along the z-axis.
    /// The cylinder with cylinder_radius is from
    /// (0, 0, 0) to (0, 0, cylinder_height), and
    /// the cone is from (0, 0, cylinder_height)
    /// to (0, 0, cylinder_height + cone_height).
    /// The cone will be split into resolution segments.
    /// The cylinder_height will be split into cylinder_split
    /// segments. The cone_height will be split into cone_split
    /// segments.
    //
    /// \param cylinder_radius defines the radius of the cylinder.
    /// \param cone_radius defines the radius of the cone.
    /// \param cylinder_height defines the height of the cylinder. The cylinder
    /// is from (0, 0, 0) to (0, 0, cylinder_height)
    /// \param cone_height defines the height of the cone. The axis of the cone
    /// will be from (0, 0, cylinder_height) to (0, 0, cylinder_height +
    /// cone_height).
    /// \param resolution defines the cone will be split into resolution
    /// segments.
    /// \param cylinder_split defines the cylinder_height will be split into
    /// cylinder_split segments.
    /// \param cone_split defines the cone_height will be split into cone_split
    /// segments.
    auto CreateArrow(
            double cylinder_radius = 1.0,
            double cone_radius = 1.5,
            double cylinder_height = 5.0,
            double cone_height = 4.0,
            int resolution = 20,
            int cylinder_split = 4,
            int cone_split = 1)->TriangleMesh;

    /// \param length_split defines the number of segments along the Mobius
    /// strip.
    /// \param width_split defines the number of segments along the width
    /// of the Mobius strip.\param twists defines the number of twists of the
    /// strip.
    /// \param radius defines the radius of the Mobius strip.
    /// \param flatness controls the height of the strip.
    /// \param width controls the width of the Mobius strip.
    /// \param scale is used to scale the entire Mobius strip.
    auto CreateMobius(int length_split = 70,
					int width_split = 15,
					int twists = 1,
					double radius = 1,
					double flatness = 1,
					double width = 1,
					double scale = 1)->TriangleMesh;


    /// the mesh centroid will be at (0,0,0) and \p radius defines the
    /// distance from the center to the mesh vertices.
    /// \param radius defines the distance from centroid to mesh vetices.
    auto CreateTetrahedron(double radius = 1.0)->TriangleMesh;

    /// the mesh centroid will be at (0,0,0) and \p radius defines the
    /// distance from the center to the mesh vertices.
    /// \param radius defines the distance from centroid to mesh vetices.
    auto CreateOctahedron(double radius = 1.0)->TriangleMesh;

    /// (trianglemeshfactory.cpp). The mesh centroid will be at (0,0,0) and
    /// \param radius defines the distance from the center to the mesh vertices.
    auto CreateIcosahedron(double radius = 1.0)->TriangleMesh;

}  // namespace geometry