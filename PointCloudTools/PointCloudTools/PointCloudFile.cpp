#include "PointCloudFile.h"


PointCloudFile::PointCloudFile()
{
	cloud = NULL;  //点云指针
	m_strPointFullFileName = "";
	m_strSubName = "";
	m_strDirPath = "";
	m_bVisible = true;
	m_minPt.x = 0;
	m_minPt.y = 0;
	m_minPt.z = 0;
	m_maxPt.x = 0;
	m_maxPt.y = 0;
	m_maxPt.z = 0;
}


PointCloudFile::~PointCloudFile()
{
}


PointCloud::PointCloud()
{
	cloud = NULL;  //点云指针
	m_strPointName = "";
	m_bVisible = true;
	m_indicesVec.clear();
	m_minPt.x = 0;
	m_minPt.y = 0;
	m_minPt.z = 0;
	m_maxPt.x = 0;
	m_maxPt.y = 0;
	m_maxPt.z = 0;
}


PointCloud::~PointCloud()
{
}
