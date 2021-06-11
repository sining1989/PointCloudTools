#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/filters/extract_indices.h>

using namespace std;

typedef pcl::PointXYZRGBA PointTRGBA;
typedef pcl::PointCloud<PointTRGBA> PointCloudTRGBA;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;

class PointCloudFile
{
public:
	PointCloudFile();
	virtual ~PointCloudFile();

public:
	PointCloudTRGBA::Ptr cloud;  //点云指针
	string m_strPointFullFileName;  //点云的全路径文件名
	string m_strSubName;  //点云的文件名
	string m_strDirPath;
	bool m_bVisible;  //点云在 viewer 中是否可见
	PointTRGBA m_minPt;
	PointTRGBA m_maxPt;

};

class PointCloud
{
public:
	PointCloud();
	virtual ~PointCloud();

public:
	PointCloudTRGBA::Ptr cloud;  //点云指针
	string m_strPointName;  //点云名
	bool m_bVisible;  //点云在 viewer 中是否可见
	std::vector<pcl::PointIndices> m_indicesVec;
	PointTRGBA m_minPt;
	PointTRGBA m_maxPt;

};
