/**\file mesh_to_pointcloud.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <mesh_to_pointcloud/mesh_to_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



namespace mesh_to_pointcloud {

template<typename PointT>
bool loadMeshFromFile(const std::string& filename, pcl::PointCloud<PointT>& pointcloud) {
	std::string::size_type index = filename.rfind(".");
	if (index == std::string::npos) return false;

	std::string extension = filename.substr(index + 1);

	if (extension == "pcd") {
		if (pcl::io::loadPCDFile<PointT>(filename, pointcloud) == 0 && !pointcloud.empty()) return true;
	} else {
		// Supported file types: .3dc .3ds .asc .ac .bsp .dae .dw .dxf .fbx .flt .gem .geo .iv .ive .logo .lwo .lw .lws .md2 .obj .ogr .osg .pfb .ply .shp .stl .x .wrl ...
		// http://trac.openscenegraph.org/projects/osg/browser/OpenSceneGraph/trunk/src/osgPlugins
		// http://trac.openscenegraph.org/projects/osg/wiki/Support/UserGuides/Plugins
		osg::ref_ptr<osg::Node> node(osgDB::readNodeFile(filename));
		if (node) {
			osg::Geode* geode = node->asGeode();
			if (geode && geode->getNumDrawables() > 0) {
				osg::Geometry* geometry = geode->getDrawable(0)->asGeometry();
				if (geometry) {
					return convertMeshToPCLPointCloud(geometry, pointcloud);
				}
			}
		}
	}

	return false;
}


template<typename PointT>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud) {
	insertPoints(geometry, pointcloud);
	return !pointcloud.empty();
}


template<typename PointT>
bool insertPoints(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud) {
	const osg::Vec3Array* vertex_points = (osg::Vec3Array*)geometry->getVertexArray();
	for (osg::Vec3Array::size_type i = 0; i < vertex_points->size(); ++i) {
		PointT point;
		point.x = (*vertex_points)[i][0];
		point.y = (*vertex_points)[i][1];
		point.z = (*vertex_points)[i][2];
		pointcloud.push_back(point);
	}

	pointcloud.is_dense = false;
	pointcloud.width = pointcloud.size();
	pointcloud.height = 1;

	return !pointcloud.empty();
}


template<typename PointT>
bool addNormals(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud) {
	const osg::Vec3Array* vertex_normals = (osg::Vec3Array*)geometry->getNormalArray();
	size_t pointcloud_index = 0;
	size_t pointcloud_normal_stride = (geometry->getNormalBinding() != osg::Geometry::BIND_PER_VERTEX) ? 3 : 1;

	for (osg::Vec3Array::size_type normals_index = 0; normals_index < vertex_normals->size(); ++normals_index) {
		for (size_t i = 0; i < pointcloud_normal_stride && pointcloud_index < pointcloud.size(); ++i) {
			PointT* point = &pointcloud.points[pointcloud_index++];
			point->normal_x = (*vertex_normals)[normals_index][0];
			point->normal_y = (*vertex_normals)[normals_index][1];
			point->normal_z = (*vertex_normals)[normals_index][2];
		}
	}

	return !pointcloud.empty();
}


template<typename PointT>
bool addRGB(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud) {
	const osg::Vec4Array* vertex_colors = (osg::Vec4Array*) geometry->getColorArray();
	size_t pointcloud_index = 0;
	size_t pointcloud_color_stride = (geometry->getColorBinding() != osg::Geometry::BIND_PER_VERTEX) ? 3 : 1;

	for (osg::Vec4Array::size_type colors_index = 0; colors_index < vertex_colors->size(); ++colors_index) {
		for (size_t i = 0; i < pointcloud_color_stride && pointcloud_index < pointcloud.size(); ++i) {
			PointT* point = &pointcloud.points[pointcloud_index++];
			point->r = (*vertex_colors)[colors_index][0] * 256;
			point->g = (*vertex_colors)[colors_index][1] * 256;
			point->b = (*vertex_colors)[colors_index][2] * 256;
		}
	}

	return !pointcloud.empty();
}


template<typename PointT>
bool addRGBA(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud) {
	const osg::Vec4Array* vertex_colors = (osg::Vec4Array*) geometry->getColorArray();
	size_t pointcloud_index = 0;
	size_t pointcloud_color_stride = (geometry->getColorBinding() != osg::Geometry::BIND_PER_VERTEX) ? 3 : 1;

	for (osg::Vec4Array::size_type colors_index = 0; colors_index < vertex_colors->size(); ++colors_index) {
		for (size_t i = 0; i < pointcloud_color_stride && pointcloud_index < pointcloud.size(); ++i) {
			PointT* point = &pointcloud.points[pointcloud_index++];
			point->r = (*vertex_colors)[colors_index][0] * 256;
			point->g = (*vertex_colors)[colors_index][1] * 256;
			point->b = (*vertex_colors)[colors_index][2] * 256;
			point->a = (*vertex_colors)[colors_index][3] * 256;
		}
	}

	return !pointcloud.empty();}

}

