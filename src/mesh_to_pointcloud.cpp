/**\file mesh_to_pointcloud.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <mesh_to_pointcloud/impl/mesh_to_pointcloud.hpp>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifndef DRL_NO_PRECOMPILE

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_MTPloadMeshFromFile(T) template bool mesh_to_pointcloud::loadMeshFromFile<T>(const std::string&, pcl::PointCloud<T>&);
PCL_INSTANTIATE(MTPloadMeshFromFile, MTP_POINT_TYPES)

#define PCL_INSTANTIATE_MTPinsertPoints(T) template bool mesh_to_pointcloud::insertPoints<T>(const osg::Geometry*, pcl::PointCloud<T>&);
PCL_INSTANTIATE(MTPinsertPoints, MTP_POINT_TYPES)

#define PCL_INSTANTIATE_MTPaddNormals(T) template bool mesh_to_pointcloud::addNormals<T>(const osg::Geometry*, pcl::PointCloud<T>&);
PCL_INSTANTIATE(MTPaddNormals, MTP_POINT_TYPES_NORMALS)

#define PCL_INSTANTIATE_MTPaddRGB(T) template bool mesh_to_pointcloud::addRGB<T>(const osg::Geometry*, pcl::PointCloud<T>&);
PCL_INSTANTIATE(MTPaddRGB, MTP_POINT_TYPES_COLOR)

#define PCL_INSTANTIATE_MTPaddRGBA(T) template bool mesh_to_pointcloud::addRGBA<T>(const osg::Geometry*, pcl::PointCloud<T>&);
PCL_INSTANTIATE(MTPaddRGBA, MTP_POINT_TYPES_COLOR)

#endif


namespace mesh_to_pointcloud {

template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
	insertPoints(geometry, pointcloud);
	return !pointcloud.points.empty();
}


template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointXYZI>& pointcloud) {
	insertPoints(geometry, pointcloud);
	return !pointcloud.points.empty();
}


template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud) {
	insertPoints(geometry, pointcloud);
	addRGB(geometry, pointcloud);
	return !pointcloud.points.empty();
}


template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointXYZRGBA>& pointcloud) {
	insertPoints(geometry, pointcloud);
	addRGBA(geometry, pointcloud);
	return !pointcloud.points.empty();
}


template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointNormal>& pointcloud) {
	insertPoints(geometry, pointcloud);
	addNormals(geometry, pointcloud);
	return !pointcloud.points.empty();
}


template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointXYZINormal>& pointcloud) {
	insertPoints(geometry, pointcloud);
	addNormals(geometry, pointcloud);
	return !pointcloud.points.empty();
}


template<>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<pcl::PointXYZRGBNormal>& pointcloud) {
	insertPoints(geometry, pointcloud);
	addRGB(geometry, pointcloud);
	addNormals(geometry, pointcloud);
	return !pointcloud.points.empty();
}

}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

