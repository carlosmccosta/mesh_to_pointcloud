#pragma once

/**\file mesh_to_pointcloud.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define MTP_POINT_TYPES			\
	(pcl::PointXYZ)				\
	(pcl::PointXYZI)			\
	(pcl::PointXYZRGB)			\
	(pcl::PointXYZRGBA)			\
	(pcl::PointNormal)			\
	(pcl::PointXYZINormal)		\
	(pcl::PointXYZRGBNormal)

#define MTP_POINT_TYPES_NORMALS	\
	(pcl::PointNormal)			\
	(pcl::PointXYZINormal)		\
	(pcl::PointXYZRGBNormal)

#define MTP_POINT_TYPES_COLOR	\
	(pcl::PointXYZRGB)			\
	(pcl::PointXYZRGBA)			\
	(pcl::PointXYZRGBNormal)
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>
#include <algorithm>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// OSG includes
#include <osg/ref_ptr>
#include <osg/Array>
#include <osg/Geometry>
#include <osgDB/ReadFile>

// external libs includes

// project includes

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###########################################################################   mesh_to_pointcloud   ##########################################################################
namespace mesh_to_pointcloud {

template <typename PointT>
bool loadMeshFromFile(const std::string& filename, pcl::PointCloud<PointT>& pointcloud);

template <typename PointT>
bool convertMeshToPCLPointCloud(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud);

template <typename PointT>
bool insertPoints(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud);

template <typename PointT>
bool addNormals(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud);

template <typename PointT>
bool addRGB(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud);

template <typename PointT>
bool addRGBA(const osg::Geometry* geometry, pcl::PointCloud<PointT>& pointcloud);

}


#ifdef MTP_NO_PRECOMPILE
#include <mesh_to_pointcloud/impl/mesh_to_pointcloud.hpp>
#endif
