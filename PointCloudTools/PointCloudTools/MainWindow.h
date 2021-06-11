#ifndef MainWindow_H
#define MainWindow_H

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h> 
#include <pcl/common/impl/io.hpp>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <pcl/filters/convolution_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/iss_3d.h>//关键点检测
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <vector>
#include <iostream>

#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QMutex>
#include <QSettings>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <QTextEdit>
#include <QTime>
#include <QMouseEvent> 
#include <QDesktopServices> 
#include <QUrl>
#include <QTranslator>

#include "Common.h"
#include "Conversions.h"
#include "AboutWin.h"
#include "PointCloudFile.h"

#include "ui_MainWindow.h"

class EventThread;
class WorkerThread;

struct PointPicking_Callback_Args
{
	PointCloudTRGBA::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
struct NavStaFix
{
	double dLatitude;
	double dLongitude;
	double dAltitude;

	NavStaFix()
	{
		dLatitude = 0.0;
		dLongitude = 0.0;
		dAltitude = 0.0;
	}
};

struct PointAttribute
{
	int iIndex;
	PointTI pt;
	NavStaFix outNav;
};


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

private:
	Ui::MainWindowClass ui;

	QTranslator* m_translator; //翻译器
	LANGUAGE m_currentLanguage; //当前语言

	EventThread *m_pEvOpenFile;
	WorkerThread *m_workThread;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_mainViewer;
	PointCloudFile m_pointCloudFile;
	std::vector<PointCloudFile> m_pointCloudFileVec;
	PointCloudTI::Ptr m_pointCloudFileTI;

	QString m_strSaveFilename;
	unsigned long m_ulTotalPoints; //Total amount of points in the viewer

	unsigned int m_uRed;
	unsigned int m_uGreen;
	unsigned int m_uBlue;
	int m_uPointSize;

	int m_iThemeId; // 0: Windows theme, 1: Darcula theme

	bool m_bSaveAsBinary;
	QString m_strTimeCost;  // 记录某个动作执行的时间
	PointPicking_Callback_Args m_pointPickingArgs;

	PointCloudT::Ptr cloud_xyz;
	PointCloudT::Ptr cloud_after_filter;
	std::vector<PointTRGBA> m_selectedPointVec;
	boost::mutex cloud_mutex;
	int m_iClickedLastIndex;
	QStringList m_strFileNames;
	QString m_strStatusInfo;
	bool m_bIsOpening;
	bool m_bIsChanging;
	int m_iClickedIndex;
	int m_iAreaPickingIndex;
	int m_iCurPointFileIndex;
	std::vector<int> m_indicesVec;

	std::map<QString, PointCloud> m_pointCloudMap;
	QTreeWidgetItem* m_pCarItem;
	QTreeWidgetItem* m_pTreeItem;
	int m_iSegmentationCarIndex;
	int m_iSegmentationTreeIndex;

	std::vector<PointAttribute> m_pointAttributeVec;
	double m_PLato;
	double m_PLo;

public:
	static void pointPicking_callback(const pcl::visualization::PointPickingEvent& event, void* args);	
	static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
	static void areaPicking_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
	void openRun();
	void changeRun();
	void segmentation();
	void euclideanSegmentation(const PointCloudTRGBA::Ptr &cloud);
	void conditionalEuclideanSegmentation(const PointCloudT::Ptr &cloud);
	static bool customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance);
	void showRealTimePointCloudAfterFilter(const PointCloudT::Ptr &cloud);
	void showRealTimePointCloud(const PointCloudTRGBA::Ptr &cloud);

private:
	void writeSettings();
	void readSettings();
	void changeLanguage(LANGUAGE language);
	double disBetweenAB(PointTRGBA a, PointTRGBA b);
	/***** Slots of QMenuBar and QToolBar *****/
	// File menu slots
	void open();
	void add();
	void clear();
	void save();
	void saveBinary();
	void savemulti();
	void exit();
	// Display menu slots
	void pointcolorChangedData();
	void bgcolorChanged();
	void mainview();
	void leftview();
	void topview();
	// View menu slots
	void dataDock();
	void propertiesDock();
	void consoleDock();
	void rgbDock();
	void typeDock();
	// Generate menu slots
	void cube();
	void createSphere();
	void createCylinder();
	// Process menu slots 
	void convertSurface();  //法线估计、曲面重建、网格面片显示
	void convertPoint();
	void convertWireframe();
	void gridModelConvert();
	void change();
	// Option menu slots
	void windowsTheme();
	void darculaTheme();
	void langEnglish();
	void langChinese();
	// About menu slots
	void about();
	void help();

	/***** Utils Methods ***/
	void initial();
	void showPointcloud();  //显示点云
	void showPointcloudAdd();  //添加给viewer，显示点云

	void setCloudColor(unsigned int r, unsigned int g, unsigned int b);
	void setAlpha(unsigned int a);

	void setPropertyTable(int iIndex);
	void setPointPropertyTable(PointTRGBA pt);
	void setSelectPointPropertyTable(const QString &strName, const PointCloudTRGBA::Ptr &cloud);
	void setPropertyTable();
	void setConsoleTable();
	void setTypeTree();
	void consoleLog(QString operation, QString subname, QString filename, QString note);

	void showPointCloudFile(const int & id);
	void hidePointCloudFile(const int &id);
	void visualize_pcd(PointCloudT::Ptr pcd_src, PointCloudT::Ptr pcd_tgt);
	void angle2Rad(const double& dAng, double& dRad);
	void rad2Angle(const double& dRad, double& dAng);
	NavStaFix getNavStaFixByPointxyz(const PointTI& pt, const NavStaFix& inputNav, const double &angle);
	void BLH2xyz( double latitude, double longitude, int ZoneWide, double *X, double *Y /*double dLat, double dLon, double a, double e1, double e2, int k, double &x, double &y*/);
	void conv_llh2xyz(double latitude, double longitude, double &Y, double &X);
signals:
	void updateSignal(QString);

private slots:
	/***** Slots of RGB widget *****/
	// Change color or size of cloud when slider is released or colorBtn is pressed
	void colorBtnPressed();
	void RGBsliderReleased();
	void pSliderReleased();
	void pSliderChanged(int value);
	void rSliderChanged(int value);
	void gSliderChanged(int value);
	void bSliderChanged(int value);
	// Slots of checkBox
	void cooCbxChecked(int value);
	void bgcCbxChecked(int value);

	/***** Slots of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked
	void itemSelectedData(QTreeWidgetItem*, int);
	// Item in dataTree is right-clicked
	void popMenuData(const QPoint&);
	void hideItemData();
	void showItemData();
	void deleteItemData();

	void popMenuInConsole(const QPoint&);
	void clearConsole();

	void itemSelectedType(QTreeWidgetItem*, int);
	void popMenuType(const QPoint&);
	void hideItemType();
	void showItemType();
	void deleteItemType();
	void pointcolorChangedType();

	bool prepareEventThread();
	void eventThreadStarted();
	void eventThreadFinished();
	bool prepareWorkThread();
	void updateSlot(QString strInfo);
	void workThreadStarted();
	void workThreadFinished();

	void issKeyPoint();
	void siftKeyPoint();
};

class EventThread : public QThread
{
	Q_OBJECT
public:
	EventThread(MainWindow *mainWindow)
		: m_mainWindowEvent(mainWindow) {};
	virtual ~EventThread() {};

	void run() { m_mainWindowEvent->openRun(); }
protected:
	MainWindow *m_mainWindowEvent;

};

class WorkerThread : public QThread
{
	Q_OBJECT
public:
	WorkerThread(MainWindow *mainWindow)
		: m_mainWindowWorker(mainWindow) {};
	virtual ~WorkerThread(){};

	void run() { m_mainWindowWorker->changeRun(); }
protected:
	MainWindow *m_mainWindowWorker;

};


#endif // MainWindow_H