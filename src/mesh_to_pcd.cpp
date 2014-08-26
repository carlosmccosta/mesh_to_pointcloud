/**\file mesh_to_pcd.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// external libs includes

// project includes
#include <mesh_to_pointcloud/mesh_to_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


void showUsage(char* program_name) {
	pcl::console::print_info("Usage: %s [path/]input.[3dc|3ds|asc|ac|bsp|dae|dw|dxf|fbx|flt|gem|geo|iv|ive|logo|lwo|lw|lws|md2|obj|ogr|osg|pfb|ply|shp|stl|x|wrl] [path/]output.pcd [-binary 0|1] [-type PointXYZ|PointNormal|PointXYZRGB|PointXYZRGBNormal]\n", program_name);
}


template<typename PointT>
int convertMesh(char* input, char* output, bool binary_output_format, pcl::PointCloud<PointT>& pointcloud) {
	pcl::console::print_highlight("==> Loading %s...\n", input);
	if (mesh_to_pointcloud::loadMeshFromFile(std::string(input), pointcloud)) {
		pcl::console::print_highlight(" +> Loaded %d points\n", (pointcloud.width * pointcloud.height));
		pcl::console::print_highlight(" +> Pointcloud fields: %s\n\n", pcl::getFieldsList(pointcloud).c_str());

		pcl::console::print_highlight("==> Saving pointcloud to %s in %s format...\n", output, (binary_output_format ? "binary" : "ascii"));
		if (pcl::io::savePCDFile(std::string(output), pointcloud, binary_output_format) == 0) {
			pcl::console::print_highlight(" +> Saved %d points in %s\n\n", (pointcloud.width * pointcloud.height), output);
		} else {
			pcl::console::print_error(" !> Failed to save to file %s\n\n", output);
			return (-1);
		}
	} else {
		pcl::console::print_error(" !> Failed to load file %s\n\n", input);
		return (-1);
	}

	return 0;
}

// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	pcl::console::print_info("###################################################################################\n");
	pcl::console::print_info("############################## Mesh to PCD converter ##############################\n");
	pcl::console::print_info("###################################################################################\n\n");

	if (argc < 3) {
		showUsage(argv[0]);
		return (0);
	}

	bool binary_output_format = true;
	pcl::console::parse_argument(argc, argv, "-binary", binary_output_format);

	std::string type("PointNormal");
	pcl::console::parse_argument(argc, argv, "-type", type);
	if (type == "PointXYZ") {
		pcl::PointCloud<pcl::PointXYZ> pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	} else if (type == "PointNormal") {
		pcl::PointCloud<pcl::PointNormal> pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	} else if (type == "PointXYZRGB") {
		pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	} else if (type == "PointXYZRGBNormal") {
		pcl::PointCloud<pcl::PointXYZRGBNormal> pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	}

	return (0);
}
// ###################################################################################   </main>   #############################################################################
