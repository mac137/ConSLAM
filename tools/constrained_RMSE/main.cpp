/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Timer.h>
#include <CGAL/bounding_box.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/bounding_box.h>
#include <CGAL/boost/graph/IO/OFF.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>

#include <string>
#include <algorithm>

#include <map>

#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>

#include "CLI11.hpp"

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>


// kernel
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
//using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;
using Point = Kernel::Point_3;
using Plane = Kernel::Plane_3;
using Vector = Kernel::Vector_3;
using Segment = Kernel::Segment_3;
using Triangle = Kernel::Triangle_3;
using Iso_cuboid = Kernel::Iso_cuboid_3;
// surface mesh
using Surface_mesh = CGAL::Surface_mesh<Point>;
// KD tree for K nearest neighbor search


typedef CGAL::Search_traits_3<Kernel> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

template <typename OutputIterator>
double acompute_RMSE(Tree& target_tree, const std::vector<Point> & data, const double threshold, OutputIterator closest_points, bool matching_output = false);

namespace PMP = CGAL::Polygon_mesh_processing;

int main(int argc, char** argv)
{

    // parse command line using CLI ----------------------------------------------
    CLI::App app;
    std::string datatPCFileName;
	std::string targetPCFileName;
	std::string multitPCDirName;
	std::string targetClostestPointsFileName;
	std::string multitPCDOutputFile;
	double dis_threshold = 0.3;
    //parameters
	
    app.description("Compute constrained RMSE. To see the options use --help.");

    //add all the options supported.
    app.add_option("-t,--target, 1", datatPCFileName, "Target PCD file (PLY).")->check(CLI::ExistingFile);
	app.add_option("-d,--data, 2", targetPCFileName, "Data PCD file (PLY)")->check(CLI::ExistingFile);
	app.add_option("-D,--cross_check_dir, 3", multitPCDirName, "Computes RMSE_t over a set of data data PCD files (PLY)")->check(CLI::ExistingDirectory);
	app.add_option("-o,--target_closest, 4", targetClostestPointsFileName, "Target closets PCD file (PLY)");
	app.add_option("-O, --cross_check_output, 5", multitPCDOutputFile, "Output of cross check.");
	app.add_option("-T,--threshold, 6", dis_threshold, "The distance threshold. All nearest neighbors at a distance higher-equal than the threshold are ignored.")->check(CLI::NonNegativeNumber)->default_val(0.3);	

    app.get_formatter()->column_width(40);
    CLI11_PARSE(app, argc, argv);
    // END parse command line using CLI ----------------------------------------------

	if ((!datatPCFileName.empty() || !targetPCFileName.empty()) && !multitPCDirName.empty())
		throw std::runtime_error("Options mismatch: --cross_check_dir cannot be used with --target and/or --data.");

	if (!datatPCFileName.empty() && !targetPCFileName.empty() && !multitPCDirName.empty())
		throw std::runtime_error("You need to provide --cross_check_dir, or --target and --data ");

	if ((!multitPCDirName.empty() && multitPCDOutputFile.empty()) || (multitPCDirName.empty() && !multitPCDOutputFile.empty()))
		throw std::runtime_error("One of the options provided but the related option is not provided, see --cross_check_dir or --cross_check_output.");

	if (!multitPCDirName.empty())
	{
		std::vector<std::string> file_list;
		for (auto& entry : std::experimental::filesystem::directory_iterator(multitPCDirName))
		{
			std::string filename = entry.path().string();
			if (filename.find(".ply") != std::string::npos) {
				file_list.emplace_back(filename);
			}
		}

		std::ofstream output_file;
		output_file.open(multitPCDOutputFile);

		for (int i = 0; i < file_list.size(); i++)
		{
			for (int j = i + 1; j < file_list.size(); j++)
			{

				std::vector<Point> data;
				std::cout << file_list[i] << ", " << file_list[j] << std::endl;
				std::vector<std::vector<double>> closest_points;

				CGAL::IO::read_PLY(file_list[i], std::back_inserter(data));
				std::cout << "data loaded: " << data.size() << " points." << std::endl;

				std::vector<Point> target;
				CGAL::IO::read_PLY(file_list[j], std::back_inserter(target));
				std::cout << "target loaded: " << target.size() << " points." << std::endl;


				// compute KD-tree
				Tree tree(target.begin(), target.end());

				double RMSE = acompute_RMSE(tree, data, dis_threshold, std::back_inserter(closest_points), false);
				output_file << std::experimental::filesystem::path(file_list[i]).filename() << ", " << std::experimental::filesystem::path(file_list[j]).filename() << ", " << RMSE << std::endl;
			}
		}
		output_file.close();
	}
	else {
		// read point-cloud file
		std::vector<Point> data;
		CGAL::IO::read_PLY(datatPCFileName, std::back_inserter(data));
		std::cout << "data loaded: " << data.size() << " points." << std::endl;

		std::vector<Point> target;
		CGAL::IO::read_PLY(targetPCFileName, std::back_inserter(target));
		std::cout << "target loaded: " << target.size() << " points." << std::endl;

		// compute KD-tree
		Tree tree(target.begin(), target.end());

		std::vector<std::vector<double>> closest_points;
		double RMSE = 0;
		if (!targetClostestPointsFileName.empty())
			RMSE = acompute_RMSE(tree, data, dis_threshold, std::back_inserter(closest_points), true);
		else
			RMSE = acompute_RMSE(tree, data, dis_threshold, std::back_inserter(closest_points), false);

		std::cout << "RMSE for threshold: " << dis_threshold << " is : " << RMSE << std::endl;

		if (!targetClostestPointsFileName.empty())
		{
			std::ofstream file_cloud;
			file_cloud.open(targetClostestPointsFileName);
			for (auto& p : closest_points)
			{
				file_cloud << p[0] << ", " << p[1] << ", " << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << ", " << p[6] << std::endl;
			}
			file_cloud.close();
		}
	}


	return EXIT_SUCCESS;
}

template <typename OutputIterator>
double acompute_RMSE(Tree & target_tree, const std::vector<Point> & data, const double threshold, OutputIterator closest_points, bool matching_output)
{
	double RMSE_FINAL = 0;
	long int N_FINAL = 0;
	static tbb::spin_mutex mtx;

	tbb::parallel_for(tbb::blocked_range<int>(0, (int)data.size()), [&](tbb::blocked_range<int> range)
	{

	  double RMSE_PARTIAL = 0;
	  long int N_PARTIAL = 0;
          std::vector<std::vector<double>> closest_points_PARTIAL;
	  for (int i = range.begin(); i < range.end(); ++i)
	  {
		  // compute distance between current sample and nearest point from cloud
		  const Point& query = data[i];

		  Neighbor_search search(target_tree, query, 1); // 1 = one nearest neighbor search
		  Neighbor_search::iterator it = search.begin();

		  // squared distance
		  const FT sd = it->second;
		  if (std::sqrt(sd) < threshold)
		  {
			  N_PARTIAL++;
			  RMSE_PARTIAL += sd;

			  if (matching_output) {
				  std::vector<double> tmp(7);
				  tmp[0] = it->first.x(); tmp[1] = it->first.y(); tmp[2] = it->first.z();
				  tmp[3] = query.x(); tmp[4] = query.y(); tmp[5] = query.z();
				  tmp[6] = std::sqrt(sd);
			          closest_points_PARTIAL.push_back(tmp);
			  }
		  }
	}
         // synchronize, critical section
	{
	       	tbb::spin_mutex::scoped_lock lock(mtx);
		RMSE_FINAL += RMSE_PARTIAL;
		N_FINAL += N_PARTIAL;
		std::copy(closest_points_PARTIAL.begin(), closest_points_PARTIAL.end(), closest_points);	
	}
	});
	if (N_FINAL == 0)
		return -1.0;
	else
		return std::sqrt(RMSE_FINAL / (double)N_FINAL);
}

