#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <vector>
#include <fstream>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/random_simplify_point_set.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/tags.h>




#include <iostream>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Sphere_3 Sphere;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef std::pair<Point, Vector> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef CGAL::First_of_pair_property_map<PointVectorPair> Point_map;
typedef CGAL::Second_of_pair_property_map<PointVectorPair> Normal_map;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef Kernel::FT FT;


int main (int argc, char** argv)
{
    FT sm_angle = 20.0; // Min triangle angle in degrees.
    FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
    FT sm_distance = 0.375; // Surface Approximation error w.r.t. point set average spacing.

    std::ifstream stream("data/stairs1.ply");
    PointList points;

    if (!stream ||
        !CGAL::read_ply_points(
            stream,
            std::back_inserter(points),
            CGAL::parameters::point_map(Point_map()))
            // CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()))
        )
    {
        std::cerr << "Error: cannot read file "
                << "stairs1.ply" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Points loaded" << std::endl;

    points.erase(CGAL::random_simplify_point_set(points, 90), points.end());

    std::cout << "Filtered points list size:" << points.size() << std::endl;

    std::list<int> indices;

    double L = 1;

    PointList newPoints;

    for (auto i = 0; i < points.size() ; ++i)
    {
        auto point = points[i].first;
        if (point[0] < -L || point[0] > L || point[1] < -L || point[1] > L || point[2] < -L || point[2] > L) 
        {
            indices.push_back(i);
        }
        else
        {
            std::cout << "Added point: " << point << std::endl;
            newPoints.push_back(points[i]);
        }
    }

    std::cout << newPoints.size() << std::endl;

    CGAL::pca_estimate_normals<CGAL::Sequential_tag>
        (newPoints, 3 /* number of neighbours */,
         CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));

    std::cout << newPoints[0].first << std::endl;

    Poisson_reconstruction_function function(newPoints.begin(), newPoints.end(), Point_map(), Normal_map());

    if ( ! function.compute_implicit_function() )
      return EXIT_FAILURE;

    double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(
        newPoints, 6, CGAL::parameters::point_map(Point_map())
    );

    std::cout << "Average spacing" << average_spacing << std::endl;

    Point inner_point = function.get_inner_point();
    Sphere bsphere = function.bounding_sphere();
    FT radius = std::sqrt(bsphere.squared_radius());

    FT sm_sphere_radius = 5.0 * radius;
    FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
                      Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                      sm_dichotomy_error/sm_sphere_radius);

    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius*average_spacing,  // Max triangle size
                                                        sm_distance*average_spacing); // Approximation error

    std::cout << "Surface and criteria ready" << std::endl;

    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh

    std::cout << "Surface generated" << std::endl;

    if(tr.number_of_vertices() == 0)
      return EXIT_FAILURE;

    CGAL::Polyhedron_3<Kernel> output_mesh;
    CGAL::facets_in_complex_2_to_triangle_mesh(c2t3, output_mesh);
    std::ofstream out("stairs_poisson.off");
    out << output_mesh;

    std::cout << "File saved" << std::endl;
    return 0;

    // CGAL::Polyhedron_3<Kernel> output_mesh;

    // if (CGAL::poisson_surface_reconstruction_delaunay
    //     (points.begin(), points.end(),
    //     CGAL::First_of_pair_property_map<PointVectorPair>(),
    //     CGAL::Second_of_pair_property_map<PointVectorPair>(),
    //     output_mesh, average_spacing))
    //     {
    //         std::ofstream out("stairs.off");
    //         out << output_mesh;
    //     }
    // else
    //     return EXIT_FAILURE;
    // return EXIT_SUCCESS;

    return 0;


    // std::vector<Pwn> points;
    // if(!CGAL::IO::read_points("data/kitten.xyz", std::back_inserter(points),
    //                             CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
    //                                             .normal_map(CGAL::Second_of_pair_property_map<Pwn>())))
    // {
    //     std::cerr << "Error: cannot read input file!" << std::endl;
    //     return EXIT_FAILURE;
    // }
    // return 0;
    // Polyhedron output_mesh;
    // double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>
    //     (points, 6, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>()));
    // if (CGAL::poisson_surface_reconstruction_delaunay
    //     (points.begin(), points.end(),
    //     CGAL::First_of_pair_property_map<Pwn>(),
    //     CGAL::Second_of_pair_property_map<Pwn>(),
    //     output_mesh, average_spacing))
    //     {
    //         std::ofstream out("kitten_poisson-20-30-0.375.off");
    //         out << output_mesh;
    //     }
    // else
    //     return EXIT_FAILURE;
    // return EXIT_SUCCESS;
}