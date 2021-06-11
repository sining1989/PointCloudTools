# PointCloudTools
PointCloudTools是一款在Windows平台基于VS2017、Qt5.9.5、PCL1.8.1、VTK8.0.0源码编译开发的专门处理点云（.pcd、.ply、.obj等格式）文件的可视化工具。
该工具点云可视化使用的是vtk8.0.0编译生成的QVTKWidget窗口控件，使用PCL可以对点云进行滤波(filter)、特征提取(features)、关键点(keypoint)、
分割(segmentation)、识别(recognition)、可视化(visualization)等操作，可以对所有点云进行WGS84到平面坐标系转换，也包含将经纬度坐标转为UTM坐标的方法。
下载64位PCL1.8.164位下载路径：https://github.com/PointCloudLibrary/pcl/releases或http://unanancyowen.com/en/pcl181
PCL1.8.1对应的VTK版本为8.0.0，下载地址：https://gitlab.kitware.com/vtk/vtk/tree/v8.0.0
