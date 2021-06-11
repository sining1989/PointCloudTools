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
	PointCloudTRGBA::Ptr cloud;  //����ָ��
	string m_strPointFullFileName;  //���Ƶ�ȫ·���ļ���
	string m_strSubName;  //���Ƶ��ļ���
	string m_strDirPath;
	bool m_bVisible;  //������ viewer ���Ƿ�ɼ�
	PointTRGBA m_minPt;
	PointTRGBA m_maxPt;

};

class PointCloud
{
public:
	PointCloud();
	virtual ~PointCloud();

public:
	PointCloudTRGBA::Ptr cloud;  //����ָ��
	string m_strPointName;  //������
	bool m_bVisible;  //������ viewer ���Ƿ�ɼ�
	std::vector<pcl::PointIndices> m_indicesVec;
	PointTRGBA m_minPt;
	PointTRGBA m_maxPt;

};
