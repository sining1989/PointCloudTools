#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, m_pEvOpenFile(NULL)
	, m_workThread(NULL)
	, m_strSaveFilename("")
	, m_ulTotalPoints(0)
	, m_uRed(0)
	, m_uGreen(0)
	, m_uBlue(0)
	, m_uPointSize(1)
	, m_iThemeId(1)
	, m_bSaveAsBinary(false)
	, m_strTimeCost("")
	, m_iClickedLastIndex(0)
	, m_strStatusInfo("")
	, m_bIsOpening(false)
	, m_bIsChanging(false)
	, m_iClickedIndex(0)
	, m_iAreaPickingIndex(0)
	, m_iCurPointFileIndex(0)
	, m_iSegmentationCarIndex(0)
	, m_iSegmentationTreeIndex(0)
{
	ui.setupUi(this);

	/***** Slots connection of QMenuBar and QToolBar *****/
	// File (connect)
	QObject::connect(ui.openAction, &QAction::triggered, this, &MainWindow::open);
	QObject::connect(ui.addAction, &QAction::triggered, this, &MainWindow::add);
	QObject::connect(ui.clearAction, &QAction::triggered, this, &MainWindow::clear);
	QObject::connect(ui.saveAction, &QAction::triggered, this, &MainWindow::save);
	QObject::connect(ui.saveBinaryAction, &QAction::triggered, this, &MainWindow::saveBinary);
	QObject::connect(ui.exitAction, &QAction::triggered, this, &MainWindow::exit);
	// Display (connect)
	QObject::connect(ui.pointcolorAction, &QAction::triggered, this, &MainWindow::pointcolorChangedData);
	QObject::connect(ui.bgcolorAction, &QAction::triggered, this, &MainWindow::bgcolorChanged);
	QObject::connect(ui.mainviewAction, &QAction::triggered, this, &MainWindow::mainview);
	QObject::connect(ui.leftviewAction, &QAction::triggered, this, &MainWindow::leftview);
	QObject::connect(ui.topviewAction, &QAction::triggered, this, &MainWindow::topview);
	// View (connect)
	QObject::connect(ui.dataAction, &QAction::triggered, this, &MainWindow::dataDock);
	QObject::connect(ui.propertyAction, &QAction::triggered, this, &MainWindow::propertiesDock);
	QObject::connect(ui.consoleAction, &QAction::triggered, this, &MainWindow::consoleDock);
	QObject::connect(ui.RGBAction, &QAction::triggered, this, &MainWindow::rgbDock);
	QObject::connect(ui.typeAction, &QAction::triggered, this, &MainWindow::typeDock);
	// Generate (connect)
	QObject::connect(ui.cubeAction, &QAction::triggered, this, &MainWindow::cube);
	QObject::connect(ui.sphereAction, &QAction::triggered, this, &MainWindow::createSphere);
	QObject::connect(ui.cylinderAction, &QAction::triggered, this, &MainWindow::createCylinder);
	// Process (connect)
	QObject::connect(ui.surfaceAction, &QAction::triggered, this, &MainWindow::convertSurface);
	QObject::connect(ui.pointAction, &QAction::triggered, this, &MainWindow::convertPoint);
	QObject::connect(ui.wireframeAction, &QAction::triggered, this, &MainWindow::convertWireframe);
	QObject::connect(ui.changeAction, &QAction::triggered, this, &MainWindow::change);
	QObject::connect(ui.segmentationAction, &QAction::triggered, this, &MainWindow::segmentation);
	QObject::connect(ui.IssAction, &QAction::triggered, this, &MainWindow::issKeyPoint);
	QObject::connect(ui.SiftAction, &QAction::triggered, this, &MainWindow::siftKeyPoint);
	// Option (connect)
	QObject::connect(ui.windowsThemeAction, &QAction::triggered, this, &MainWindow::windowsTheme);
	QObject::connect(ui.darculaThemeAction, &QAction::triggered, this, &MainWindow::darculaTheme);
	QObject::connect(ui.englishAction, &QAction::triggered, this, &MainWindow::langEnglish);
	QObject::connect(ui.chineseAction, &QAction::triggered, this, &MainWindow::langChinese);
	// About (connect)
	QObject::connect(ui.aboutAction, &QAction::triggered, this, &MainWindow::about);
	QObject::connect(ui.helpAction, &QAction::triggered, this, &MainWindow::help);

	/***** Slots connection of RGB widget *****/
	// Random color (connect)
	connect(ui.colorBtn, SIGNAL(clicked()), this, SLOT(colorBtnPressed()));
	// Connection between RGB slider and RGB value (connect)
	connect(ui.rSlider, SIGNAL(valueChanged(int)), this, SLOT(rSliderChanged(int)));
	connect(ui.gSlider, SIGNAL(valueChanged(int)), this, SLOT(gSliderChanged(int)));
	connect(ui.bSlider, SIGNAL(valueChanged(int)), this, SLOT(bSliderChanged(int)));
	// RGB slider released (connect)
	connect(ui.rSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(ui.gSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(ui.bSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	// Change size of cloud (connect)
	connect(ui.pSlider, SIGNAL(valueChanged(int)), this, SLOT(pSliderChanged(int)));
	connect(ui.pSlider, SIGNAL(sliderReleased()), this, SLOT(pSliderReleased()));
	// Checkbox for coordinate and background color (connect)
	connect(ui.cooCbx, SIGNAL(stateChanged(int)), this, SLOT(cooCbxChecked(int)));
	connect(ui.bgcCbx, SIGNAL(stateChanged(int)), this, SLOT(bgcCbxChecked(int)));

	/***** Slots connection of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked (connect)
	connect(ui.dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelectedData(QTreeWidgetItem*, int)));
	// Item in dataTree is right-clicked
	connect(ui.dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenuData(const QPoint&)));
	connect(ui.consoleTable, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenuInConsole(const QPoint&)));
	connect(ui.typeTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelectedType(QTreeWidgetItem*, int)));
	connect(ui.typeTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenuType(const QPoint&)));
	connect(this, SIGNAL(updateSignal(QString)), this, SLOT(updateSlot(QString)), Qt::QueuedConnection);
	// Initialization
	initial();

	qDebug("This point cloud tools had initial!");
}

MainWindow::~MainWindow()
{
	if (m_pEvOpenFile)
	{
		if (m_pEvOpenFile->wait(1000) == false)
			m_pEvOpenFile->terminate();
		delete m_pEvOpenFile;
		m_pEvOpenFile = NULL;
	}

	if (m_workThread)
	{
		if (m_workThread->wait(1000) == false)
			m_workThread->terminate();
		delete m_workThread;
		m_workThread = NULL;
	}

	// save window state
	writeSettings();
}

//初始化
void MainWindow::initial()
{
	//界面初始化
	setWindowIcon(QIcon(":/Resources/images/icon.png"));
	setWindowTitle(tr("Point Cloud Tools"));  //更新窗口标题

	// 设置默认主题
	QFile qss(":/Resources/qss/Darcula.qss");
	qss.open(QFile::ReadOnly);
	qApp->setStyleSheet(qss.readAll());
	qss.close();

	m_pointCloudFileVec.clear();
	m_pointCloudMap.clear();
	//点云初始化
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	m_pointCloudFile.cloud->resize(1);
	m_pointCloudFileTI.reset(new PointCloudTI);

	m_mainViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	m_mainViewer->initCameraParameters();   //初始化相机参数
	ui.screen->SetRenderWindow(m_mainViewer->getRenderWindow());
	m_mainViewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());

	// Add point picking callback to viewer:
	cloud_mutex.lock();
	PointCloudTRGBA::Ptr clicked_points_3d(new PointCloudTRGBA);
	m_pointPickingArgs.clicked_points_3d = clicked_points_3d;
	m_pointPickingArgs.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(m_mainViewer);
	m_mainViewer->registerPointPickingCallback(pointPicking_callback, (void*)this);
	m_mainViewer->registerKeyboardCallback(keyboardEventOccurred, (void*)this);
	m_mainViewer->registerAreaPickingCallback(areaPicking_callback, (void*)this);
	ui.screen->update();
	cloud_mutex.unlock();

	setPropertyTable();
	ui.dataTree->setSelectionMode(QAbstractItemView::ExtendedSelection); // 允许 dataTree 进行多选
	setConsoleTable();
	setTypeTree();

	// 输出窗口
	consoleLog(tr("Software start"), tr("Point cloud tools"), tr("Welcome to use point cloud tools"), tr("Tools"));

	// 设置背景颜色为 dark
	m_mainViewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);

	readSettings();

	// create the event thread
	prepareEventThread();

	// init worker Thread
	prepareWorkThread();
}

void MainWindow::writeSettings()
{
	// saves window positions and states
	QSettings settings("PCL", "Poing cloud Tools");

	settings.beginGroup("mainWindow");
	settings.setValue("geometry", saveGeometry());
	settings.setValue("state", saveState());
	settings.endGroup();
}

void MainWindow::readSettings()
{
	// reads window positions and states
	QSettings settings("PCL", "Poing cloud Tools");

	settings.beginGroup("mainWindow");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("state").toByteArray());
	settings.endGroup();
}


void MainWindow::changeLanguage(LANGUAGE language)
{
	if (m_currentLanguage == language)
	{
		return;
	}
	m_currentLanguage = language;

	if (m_translator != NULL)
	{
		qApp->removeTranslator(m_translator);
	}

	QTranslator *l_translator = new QTranslator(qApp);
	switch (language)
	{
	case UI_EN:
		l_translator->load(QString(":/Resources/qm/lang_en"));
		break;

	case UI_ZH:
		l_translator->load(QString(":/Resources/qm/lang_zh_CN"));
		break;

	default:
		l_translator->load(QString(":/Resources/qm/lang_zh_CN"));
	}
	qApp->installTranslator(l_translator);
	ui.retranslateUi(this);
	m_translator = l_translator;
}

bool MainWindow::prepareEventThread()
{
	if (!m_pEvOpenFile)
		m_pEvOpenFile = new EventThread(this);
	if (m_pEvOpenFile->isRunning())
		return false; // Not
	connect(m_pEvOpenFile, SIGNAL(started()), this, SLOT(eventThreadStarted()), Qt::QueuedConnection);
	connect(m_pEvOpenFile, SIGNAL(finished()), this, SLOT(eventThreadFinished()), Qt::QueuedConnection);
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	connect(m_workThread, SIGNAL(terminated()), this, SLOT(workThreadFinished()), Qt::QueuedConnection);
#endif
	return true;
}

void MainWindow::eventThreadStarted()
{
	m_bIsOpening = true;
}

void MainWindow::eventThreadFinished()
{
	m_bIsOpening = false;

	// time of
	m_strTimeCost = timeOff();
	//输出窗口
	consoleLog(tr("Open"), tr("Point cloud files"), "", tr("Time cost: ") + m_strTimeCost + tr(" s"));

	ui.statusBar->showMessage(tr("Point cloud file open over"));
	showPointcloudAdd();

	setPropertyTable(m_pointCloudFileVec.size() - 1);
	m_iCurPointFileIndex = m_pointCloudFileVec.size() - 1;

}

bool MainWindow::prepareWorkThread()
{
	if (!m_workThread)
		m_workThread = new WorkerThread(this);
	if (m_workThread->isRunning())
		return false; // Not
	connect(m_workThread, SIGNAL(started()), this, SLOT(workThreadStarted()), Qt::QueuedConnection);
	connect(m_workThread, SIGNAL(finished()), this, SLOT(workThreadFinished()), Qt::QueuedConnection);
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	connect(m_workThread, SIGNAL(terminated()), this, SLOT(workThreadFinished()), Qt::QueuedConnection);
#endif
	return true;
}

void MainWindow::updateSlot(QString strInfo)
{
	ui.statusBar->showMessage(strInfo);
}

void MainWindow::workThreadStarted()
{
	m_bIsChanging = true;
}

void MainWindow::workThreadFinished()
{
	m_bIsChanging = false;
	showRealTimePointCloudAfterFilter(cloud_after_filter);
	ui.statusBar->showMessage(tr("Point cloud filter sucess"));
	consoleLog(tr("Filter"), tr("All point clouds"), QString::fromLocal8Bit(m_pointCloudFile.m_strPointFullFileName.c_str()), "");
	showPointcloudAdd();
	setPropertyTable(m_pointCloudFileVec.size() - 1);

}

// Open point cloud
void MainWindow::open()
{
	m_strFileNames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(m_pointCloudFile.m_strDirPath.c_str()), tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));
	//Return if filenames is empty
	if (m_strFileNames.isEmpty())
		return;

	// Clear cache
	m_pointCloudFileVec.clear();
	m_ulTotalPoints = 0;
	ui.dataTree->clear();
	m_mainViewer->removeAllPointClouds();

	m_pEvOpenFile->start();
}

// Add Point Cloud
void MainWindow::add()
{
	m_strFileNames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(m_pointCloudFile.m_strDirPath.c_str()), tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));
	if (m_strFileNames.isEmpty())
		return;

	/* start the event threads */
	m_pEvOpenFile->start();

}

void MainWindow::openRun()
{
	// time start
	timeStart();
	
	for (int i = 0; i != m_strFileNames.size(); i++)
	{
		PointCloudTI::Ptr pointCloudFileTITemp;
		pointCloudFileTITemp.reset(new PointCloudTI);
		QString strFileName = m_strFileNames[i];
		m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
		std::string file_name = strFileName.toStdString();
		std::string subname = getFileName(file_name);

		//更新状态栏
		emit updateSignal(QString::fromLocal8Bit(subname.c_str()) + ": " + QString::number(i) + "/" + QString::number(m_strFileNames.size()) + " point cloud loading...");
		int status = -1;
		if (strFileName.endsWith(".pcd", Qt::CaseInsensitive))
		{
			//status = pcl::io::loadPCDFile(file_name, *(m_pointCloudFile.cloud));
			//if (m_pointCloudFile.cloud->points[0].r == 0 && m_pointCloudFile.cloud->points[0].g == 0 && m_pointCloudFile.cloud->points[0].b == 0)
			//{
			//	setCloudColor(255, 255, 255);
			//}
			status = pcl::io::loadPCDFile(file_name, *pointCloudFileTITemp);
			m_pointCloudFileTI->resize(pointCloudFileTITemp->size());
			//std::vector<float> intensityVec;
			//std::map<float, int> intensityMap;
			//intensityVec.clear();
			//intensityMap.clear();
			//for (UINT i = 0; i < pointCloudFileTITemp->size(); ++i)
			//{
			//	//intensityVec.push_back(pointCloudFileTITemp->points[i].intensity);
			//	//intensityMap.insert(std::make_pair(pointCloudFileTITemp->points[i].intensity, i));
			//	//从lego_loam坐标系转为右手坐标系
			//	m_pointCloudFileTI->points[i].x = pointCloudFileTITemp->points[i].z;
			//	m_pointCloudFileTI->points[i].y = pointCloudFileTITemp->points[i].x;
			//	m_pointCloudFileTI->points[i].z = pointCloudFileTITemp->points[i].y;
			//	m_pointCloudFileTI->points[i].intensity = pointCloudFileTITemp->points[i].intensity;
			//}
			//int iCount = intensityMap.size();
			//绕Z轴旋转GPRMC给出的航向角的弧度值
			//float theta = 153 * M_PI / 180; // 弧度角 63根据两个经纬度算出的方位角
			//Eigen::Affine3f transform = Eigen::Affine3f::Identity();

			//// 和前面一样的旋转; Z 轴上旋转 theta 弧度
			//transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
			////PointCloudTI::Ptr transformed_cloud(new PointCloudTI);
			//pcl::transformPointCloud(*m_pointCloudFileTI, *m_pointCloudFileTI, transform);
		}
		else if (strFileName.endsWith(".ply", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPLYFile(file_name, *(m_pointCloudFile.cloud));
			if (m_pointCloudFile.cloud->points[0].r == 0 && m_pointCloudFile.cloud->points[0].g == 0 && m_pointCloudFile.cloud->points[0].b == 0)
			{
			setCloudColor(255, 255, 255);
			}
		}
		else if (strFileName.endsWith(".obj", Qt::CaseInsensitive))
		{
		status = pcl::io::loadOBJFile(file_name, *(m_pointCloudFile.cloud));
		if (m_pointCloudFile.cloud->points[0].r == 0 && m_pointCloudFile.cloud->points[0].g == 0 && m_pointCloudFile.cloud->points[0].b == 0)
		{
			setCloudColor(255, 255, 255);
		}
		}
		else
		{
		//提示：无法读取除了.ply .pcd .obj以外的文件
		emit updateSignal(tr("File format error"));
		return;
		}
		//提示：后缀没问题，但文件内容无法读取
		if (status != 0)
		{
			emit updateSignal(tr("Reading file error"));
			return;
		}

		//从点云中移除NAN点也就是无效点
		std::vector<int> indices;
		indices.clear();
		pcl::removeNaNFromPointCloud(*pointCloudFileTITemp, *m_pointCloudFileTI, indices);

		////22.6519171283,114.04355726,75.3121337891,-64.9127807617
		NavStaFix inputNav;
		inputNav.dLatitude = 22.663340000000002;
		inputNav.dLongitude = 114.04108833333333;
		inputNav.dAltitude = 55.681337891;

		QString rootPath = QCoreApplication::applicationDirPath();
		QDateTime curTime = QDateTime::currentDateTime();
		QString dateTime = curTime.toString("yyyy-MM-dd_hh-mm-ss");
		QString strParaFileName = QString("\\parameter%1.txt").arg(dateTime);

		rootPath += strParaFileName;
		ofstream outfile(rootPath.toStdString(), ios::trunc);
		outfile.setf(ios::fixed, ios::floatfield);
		outfile.precision(12);

		////获取原点在高斯坐标系下的值
		double dOriginX = 0.0, dOriginY = 0.0;
		double dOriginUTMNorth = 0.0, dOriginUTMEast = 0.0;
		std::string strUtmZone = "";
		BLH2xyz(inputNav.dLatitude, inputNav.dLongitude, 6, &dOriginY, &dOriginX);
		LLtoUTM(inputNav.dLatitude, inputNav.dLongitude, dOriginUTMNorth, dOriginUTMEast, strUtmZone);
		outfile << "UTMNorth:" << dOriginUTMNorth << " UTMEast:" << dOriginUTMEast << "\n";
		outfile << "Origin Longitude:" << inputNav.dLongitude << " Origin Latitude:" << inputNav.dLatitude << " Origin Altitude:" << inputNav.dAltitude << "\n";
		long dOriginXTemp = (long)dOriginX;
		long dOriginYTemp = (long)dOriginY;
		float fX = dOriginX - dOriginXTemp;
		float fY = dOriginY - dOriginYTemp;
		outfile << "Origin X:" << dOriginXTemp << " Origin Y:" << dOriginYTemp << " Origin Z:" << inputNav.dAltitude << "\n";
		outfile << "Voxel Size:" << m_pointCloudFileTI->size() << "\n";
		//取得点云坐标极值
		PointTI minPt, maxPt;
		pcl::getMinMax3D(*m_pointCloudFileTI, minPt, maxPt);
		outfile << "Min X:" << minPt.x << " Min Y:" << minPt.y << " Min Z:" << minPt.z << "\n";
		outfile << "Max X:" << maxPt.x << " Max Y:" << maxPt.y << " Max Z:" << maxPt.z << "\n";

		//m_pointAttributeVec.clear();
		//for (size_t i = 0; i < m_pointCloudFileTI->size(); ++i)
		//{
		//	PointAttribute pta;
		//	pta.iIndex = i;

		//	pta.pt = m_pointCloudFileTI->points[i];

		//	//减一个统一的坐标原点
		//	m_pointCloudFileTI->points[i].x = static_cast<float>(pta.pt.x + fX);			
		//	m_pointCloudFileTI->points[i].y = static_cast<float>(pta.pt.z + fY);
		//	m_pointCloudFileTI->points[i].z = static_cast<float>(pta.pt.y + inputNav.dAltitude);
		//	m_pointAttributeVec.push_back(pta);
		//}

		//点云坐标转为WGS84
		//原点经纬度、海拔高度、方位角
		//for (size_t i = 0; i < m_pointCloudFileTI->size(); ++i)
		//{
		//	NavStaFix outNav = getNavStaFixByPointxyz(m_pointCloudFileTI->points[i], inputNav, 0);
		//	outfile << outNav.dLongitude << "," << outNav.dLatitude << "," << outNav.dAltitude << "\n";
		//}

		outfile.close();
		pcl::copyPointCloud(*m_pointCloudFileTI, *m_pointCloudFile.cloud);

		//取得点云坐标极值
		pcl::getMinMax3D(*(m_pointCloudFile.cloud), m_pointCloudFile.m_minPt, m_pointCloudFile.m_maxPt);
		setAlpha(200);  //设置点云为不透明255
		m_pointCloudFile.m_strPointFullFileName = file_name;
		m_pointCloudFile.m_strSubName = subname;
		m_pointCloudFile.m_strDirPath = file_name.substr(0, file_name.size() - subname.size());
		m_pointCloudFileVec.push_back(m_pointCloudFile);  //将点云导入点云容器

		//设置资源管理器
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(subname.c_str()));
		cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		ui.dataTree->addTopLevelItem(cloudName);
		m_ulTotalPoints += m_pointCloudFile.cloud->points.size();
	}
}

void MainWindow::BLH2xyz( double latitude, double longitude, int ZoneWide, double *Y, double *X /*double dLat, double dLon, double a, double e1, double e2, int k, double &x, double &y*/)
{
	////公式精确到0.001米
	//double a0, a2, a4, a6, a8;
	//double m0, m2, m4, m6, m8;
	//double yita2, t, N;
	//double X;//子午线弧长
	//double l, L0;//经差以及中央子午线
	//double n;// 辅助量
	//double daihao;//带号

	//if (k == 1)//6度带
	//{
	//	if ((int)dLon % 6 == 0)
	//	{
	//		daihao = (int)(dLon / 6);
	//		L0 = 6 * daihao - 3;
	//	}
	//	else
	//	{
	//		daihao = (int)(dLon / 6) + 1;
	//		L0 = 6 * daihao - 3;
	//	}
	//}
	//else //3度带
	//{
	//	if ((int)(dLon - 1.5) % 3 == 0)
	//	{
	//		daihao = (int)((dLon - 1.5) / 3);//int 强制类型转换返回最接近0的整数部分，-0.1 返回 0，0.1 返回 0
	//		L0 = 3 * daihao;
	//	}
	//	else
	//	{
	//		daihao = (int)floor((dLon - 1.5) / 3) + 1;//math.floor 返回原类型小于原数值的整数部分-0.1 返回 -1， 0.1 返回 0
	//		L0 = 3 * daihao;
	//	}
	//}

	//l = dmsToDeg(dLon) - dmsToDeg(L0);
	//dLat = dmsToDeg(dLat);

	////计算子午线弧长
	//m0 = a * (1 - e1);
	//m2 = 3 * e1 * m0 / 2;
	//m4 = 5 * e1 * m2 / 4;
	//m6 = 7 * e1 * m4 / 6;
	//m8 = 9 * e1 * m6 / 8;

	//a0 = m0 + m2 / 2 + 3 * m4 / 8 + 5 * m6 / 16 + 35 * m8 / 128;
	//a2 = m2 / 2 + m4 / 2 + 15 * m6 / 32 + 7 * m8 / 16;
	//a4 = m4 / 8 + 3 * m6 / 16 + 7 * m8 / 32;
	//a6 = m6 / 32 + m8 / 16;
	//a8 = m8 / 128;
	////子午线弧长计算公式2
	//X = a0 * dLat - a2 * sin(2 * dLat) / 2 + a4 * sin(4 * dLat) / 4 - a6 * sin(6 * dLat) / 6 + a8 * sin(8 * dLat) / 8;
	//yita2 = e2 * cos(dLat) * cos(dLat);//η平方
	//t = tan(dLat);
	//N = a / sqrt(1 - e1 * sin(dLat) * sin(dLat));
	////计算高斯平面坐标
	//n = cos(dLat) * l;
	//x = X + N * t * (n * n / 2 + pow(n, 4) * (5 - t * t + 9 * yita2 + 4 * yita2 * yita2) / 24 + pow(n, 6) * (61 - 58 * t * t + pow(t, 4)) / 720);
	//y = N * (n + pow(n, 3) * (1 - t * t + yita2) / 6 + pow(n, 5) * (5 - 18 * t * t + pow(t, 4) + 14 * yita2 - 58 * yita2 * t * t) / 120);
	//y = y + 500000 + daihao * 1000000;//计算高斯通用坐标

	int ProjNo = 0;
	double longitude1, latitude1, longitude0, latitude0, X0, Y0, xval, yval;
	double a, f, e2, ee, NN, T, C, A, M, iPI;
	iPI = 0.0174532925199433; ////3.1415926535898/180.0; 
	a = 6378137; //WGS1984坐标系参数 
	f = 1.0 / 298.2572235635;
	ProjNo = (int)(longitude / ZoneWide);
	longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
	longitude0 = longitude0 * iPI;
	latitude0 = 0;
	longitude1 = longitude * iPI; //经度转换为弧度
	latitude1 = latitude * iPI; //纬度转换为弧度
	e2 = 2 * f - f * f;
	ee = e2 * (1.0 - e2);
	NN = a / sqrt(1.0 - e2 * sin(latitude1)*sin(latitude1));
	T = tan(latitude1)*tan(latitude1);
	C = ee * cos(latitude1)*cos(latitude1);
	A = (longitude1 - longitude0)*cos(latitude1);

	M = a * ((1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256)*latitude1 - (3 * e2 / 8 + 3 * e2*e2 / 32 + 45 * e2*e2*e2 / 1024)*sin(2 * latitude1)
		+ (15 * e2*e2 / 256 + 45 * e2*e2*e2 / 1024)*sin(4 * latitude1) - (35 * e2*e2*e2 / 3072)*sin(6 * latitude1));
	xval = NN * (A + (1 - T + C)*A*A*A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee)*A*A*A*A*A / 120);
	yval = M + NN * tan(latitude1)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
		+ (61 - 58 * T + T * T + 600 * C - 330 * ee)*A*A*A*A*A*A / 720);

	X0 = 0;
	Y0 = 1000000L * (ProjNo + 1) + 500000L;
	yval = yval + X0;
	xval = xval + Y0;
	*X = xval;
	*Y = yval;
}

// Clear all point clouds
void MainWindow::clear()
{
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	m_pointCloudFileVec.clear();  //从点云容器中移除所有点云
	m_pointCloudMap.clear();
	m_mainViewer->removeAllPointClouds();  //从viewer中移除所有点云
	m_mainViewer->removeAllShapes(); //这个remove更彻底

	//if (1 == GetKeyState(0x58))
	{
		::keybd_event(0x58, 0, 0, 0);
		Sleep(20);
		::keybd_event(0x58, 0, KEYEVENTF_KEYUP, 0);
	}

	ui.dataTree->clear();  //将dataTree清空
	ui.typeTree->clear();
	m_pCarItem = new QTreeWidgetItem(ui.typeTree, QStringList() << tr("Car"));
	m_pTreeItem = new QTreeWidgetItem(ui.typeTree, QStringList() << tr("Tree"));
	m_pCarItem->setHidden(true);
	m_pTreeItem->setHidden(true);
	ui.propertyTable->clearContents();
	cloud_xyz.reset(new PointCloudT);
	cloud_after_filter.reset(new PointCloudT);
	m_selectedPointVec.clear();
	m_iClickedLastIndex = 0;
	m_iClickedIndex = 0;
	m_iAreaPickingIndex = 0;
	m_indicesVec.clear();
	m_iSegmentationCarIndex = 0;
	m_iSegmentationTreeIndex = 0;
	//输出窗口
	consoleLog(tr("Clear"), tr("All point clouds"), "", "");

	showPointcloud();  //更新显示
}

// Save point cloud
void MainWindow::save()
{
	m_strSaveFilename = QFileDialog::getSaveFileName(this, tr("Save point cloud"),
		QString::fromLocal8Bit(m_pointCloudFile.m_strDirPath.c_str()), tr("Point cloud data(*.pcd *.ply);;Allfile(*.*)"));
	//文件名为空直接返回
	if (m_strSaveFilename.isEmpty())
		return;

	std::string file_name = m_strSaveFilename.toStdString();
	std::string subname = getFileName(file_name);

	if (m_pointCloudFileVec.size() > 1)
	{
		savemulti();
		return;
	}
	QApplication::processEvents();
	int status = -1;
	if (m_strSaveFilename.endsWith(".pcd", Qt::CaseInsensitive))
	{
		status = pcl::io::savePCDFile(file_name, *m_pointCloudFileTI/*(m_pointCloudFile.cloud)*/);//二进制为1，ASCII码为0
		status = pcl::io::savePCDFile(file_name, *(m_pointCloudFile.cloud));
	}
	else if (m_strSaveFilename.endsWith(".ply", Qt::CaseInsensitive))
	{
		status = pcl::io::savePLYFile(file_name, *(m_pointCloudFile.cloud));
	}
	else //提示：无法保存为除了.ply .pcd以外的文件
	{
		QMessageBox::information(this, tr("File format error"), tr("Can't save files except .ply .pcd"));
		return;
	}
	//提示：后缀没问题，但是无法保存
	if (status != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"), tr("Can not save the file"));
		return;
	}

	//输出窗口
	consoleLog(tr("Save"), QString::fromLocal8Bit(subname.c_str()), m_strSaveFilename, tr("Single save"));

	setWindowTitle(m_strSaveFilename);
	QMessageBox::information(this, tr("save point cloud file"),
		QString::fromLocal8Bit(("Save " + subname + " successfully!").c_str()));
}

// Save point cloud as binary file
void MainWindow::saveBinary()
{
	m_strSaveFilename = QFileDialog::getSaveFileName(this, tr("Save point cloud as binary file"),
		QString::fromLocal8Bit(m_pointCloudFile.m_strDirPath.c_str()), tr("Point cloud data(*.pcd *.ply);;Allfile(*.*)"));
	//文件名为空直接返回
	if (m_strSaveFilename.isEmpty())
		return;
	std::string file_name = m_strSaveFilename.toStdString();
	std::string subname = getFileName(file_name);

	if (m_pointCloudFileVec.size() > 1)
	{
		savemulti();
		return;
	}

	int status = -1;
	if (m_strSaveFilename.endsWith(".pcd", Qt::CaseInsensitive))
	{
		status = pcl::io::savePCDFileBinary(file_name, *(m_pointCloudFile.cloud));
	}
	else if (m_strSaveFilename.endsWith(".ply", Qt::CaseInsensitive))
	{
		status = pcl::io::savePLYFileBinary(file_name, *(m_pointCloudFile.cloud));
	}
	else //提示：无法保存为除了.ply .pcd以外的文件
	{
		QMessageBox::information(this, tr("File format error"), tr("Can't save files except .ply .pcd"));
		return;
	}
	//提示：后缀没问题，但是无法保存
	if (status != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"), tr("Can not save the file"));
		return;
	}

	//输出窗口
	consoleLog(tr("Save as binary"), QString::fromLocal8Bit(subname.c_str()), m_strSaveFilename, tr("Single save (binary)"));

	setWindowTitle(m_strSaveFilename);
	ui.statusBar->showMessage(tr("Save ") + subname.c_str() + tr(" successfully!"));

}

// Save multi point cloud
void MainWindow::savemulti()
{
	std::string subname = getFileName(m_strSaveFilename.toStdString());
	PointCloudTRGBA::Ptr multi_cloud;
	multi_cloud.reset(new PointCloudTRGBA);
	multi_cloud->height = 1;
	int sum = 0;
	for (auto c : m_pointCloudFileVec)
	{
		sum += c.cloud->points.size();
	}
	multi_cloud->width = sum;
	multi_cloud->resize(multi_cloud->height * multi_cloud->width);
	int k = 0;
	for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	{
		for (int j = 0; j != m_pointCloudFileVec[i].cloud->points.size(); j++) //注意cloudvec[i]->points.size()和cloudvec[i]->size()的区别
		{
			multi_cloud->points[k].x = m_pointCloudFileVec[i].cloud->points[j].x;
			multi_cloud->points[k].y = m_pointCloudFileVec[i].cloud->points[j].y;
			multi_cloud->points[k].z = m_pointCloudFileVec[i].cloud->points[j].z;
			multi_cloud->points[k].r = m_pointCloudFileVec[i].cloud->points[j].r;
			multi_cloud->points[k].g = m_pointCloudFileVec[i].cloud->points[j].g;
			multi_cloud->points[k].b = m_pointCloudFileVec[i].cloud->points[j].b;
			k++;
		}
	}
	//保存multi_cloud
	int status = -1;
	if (m_strSaveFilename.endsWith(".pcd", Qt::CaseInsensitive))
	{
		if (m_bSaveAsBinary)
		{
			status = pcl::io::savePCDFileBinary(m_strSaveFilename.toStdString(), *multi_cloud);
		}
		else
		{
			status = pcl::io::savePCDFile(m_strSaveFilename.toStdString(), *multi_cloud);
		}

	}
	else if (m_strSaveFilename.endsWith(".ply", Qt::CaseInsensitive))
	{
		if (m_bSaveAsBinary)
		{
			status = pcl::io::savePLYFileBinary(m_strSaveFilename.toStdString(), *multi_cloud);
		}
		else
		{
			status = pcl::io::savePLYFile(m_strSaveFilename.toStdString(), *multi_cloud);
		}
	}
	else //提示：无法保存为除了.ply .pcd以外的文件
	{
		QMessageBox::information(this, tr("File format error"), tr("Can't save files except .ply .pcd"));
		return;
	}

	//提示：后缀没问题，但是无法保存
	if (status != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"), tr("Can not save the file"));
		return;
	}

	// 输出窗口
	if (m_bSaveAsBinary)
	{
		consoleLog(tr("Save as binary"), QString::fromLocal8Bit(subname.c_str()), m_strSaveFilename, tr("Multi save (binary)"));
	}
	else
	{
		consoleLog(tr("Save"), QString::fromLocal8Bit(subname.c_str()), m_strSaveFilename, tr("Multi save"));
	}

	m_bSaveAsBinary = false;
	//将保存后的 multi_cloud 设置为当前 m_pointCloudFile,以便保存之后直接进行操作
	m_pointCloudFile.cloud = multi_cloud;
	multi_cloud.reset(new PointCloudTRGBA);
	m_pointCloudFile.m_strPointFullFileName = m_strSaveFilename.toStdString();
	m_pointCloudFile.m_strSubName = subname;

	setWindowTitle(m_strSaveFilename);
	ui.statusBar->showMessage(tr("Save ") + subname.c_str() + tr(" successfully!"));

}

//退出程序
void MainWindow::exit()
{
	qDebug("This point cloud tools had exited!");
	this->close();
	qApp->quit();

}

// Generate cube
void MainWindow::cube()
{
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	m_ulTotalPoints = 0;
	//ui.dataTree->clear();  //清空资源管理器的item
	m_mainViewer->removeAllPointClouds();  //从viewer中移除所有点云
	//m_pointCloudFileVec.clear();  //清空点云容器

	m_pointCloudFile.cloud->width = 50000;         // 设置点云宽
	m_pointCloudFile.cloud->height = 1;            // 设置点云高，高为1，说明为无组织点云
	m_pointCloudFile.cloud->is_dense = false;
	m_pointCloudFile.cloud->resize(m_pointCloudFile.cloud->width * m_pointCloudFile.cloud->height);     // 重置点云大小
	for (size_t i = 0; i < m_pointCloudFile.cloud->size(); ++i)
	{
		m_pointCloudFile.cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		m_pointCloudFile.cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		m_pointCloudFile.cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		m_pointCloudFile.cloud->points[i].r = m_uRed;
		m_pointCloudFile.cloud->points[i].g = m_uGreen;
		m_pointCloudFile.cloud->points[i].b = m_uBlue;
	}
	//设置资源管理器
	QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("cube"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);

	// 输出窗口
	consoleLog(tr("Generate cube"), tr("cube"), tr("cube"), "");

	m_pointCloudFileVec.push_back(m_pointCloudFile);
	showPointcloudAdd();
}

void MainWindow::pointPicking_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct MainWindow* thisT = (struct MainWindow *)args;

	if (event.getPointIndex() == -1 || thisT->m_iClickedLastIndex == event.getPointIndex())
		return;
	thisT->m_iClickedLastIndex = event.getPointIndex();

	PointTRGBA curPoint;
	event.getPoint(curPoint.x, curPoint.y, curPoint.z);
	thisT->m_pointPickingArgs.clicked_points_3d->points.push_back(curPoint);
	thisT->cloud_mutex.lock();
	// Draw clicked points in blue:
	pcl::visualization::PointCloudColorHandlerCustom<PointTRGBA> blue(thisT->m_pointPickingArgs.clicked_points_3d, 0, 0, 255);
	if (thisT->m_iClickedIndex > 2)
		thisT->m_pointPickingArgs.viewerPtr->removePointCloud("clicked_points" + QString::number(thisT->m_iClickedIndex - 2).toStdString());
	thisT->m_pointPickingArgs.viewerPtr->addPointCloud(thisT->m_pointPickingArgs.clicked_points_3d, blue, "clicked_points" + QString::number(thisT->m_iClickedIndex).toStdString());
	thisT->m_pointPickingArgs.viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clicked_points" + QString::number(thisT->m_iClickedIndex).toStdString());
	thisT->m_iClickedIndex++;

	thisT->setPointPropertyTable(curPoint);
	thisT->m_selectedPointVec.push_back(curPoint);
	int iCount = thisT->m_selectedPointVec.size();
	if (iCount >= 2)
	{
		double dDis = thisT->disBetweenAB(thisT->m_selectedPointVec[iCount - 2], thisT->m_selectedPointVec[iCount - 1]);
		if (0 == dDis)
		{
			thisT->ui.statusBar->showMessage(tr("Please choose a different point"));
		}
		else
		{
			QString strTips = tr("Distance:");
			QString strInfo = QString("%1%2").arg(strTips).arg(dDis);
			thisT->ui.statusBar->showMessage(strInfo);
		}
		thisT->m_selectedPointVec.clear();
		thisT->m_selectedPointVec.push_back(curPoint);
	}
	thisT->cloud_mutex.unlock();
	thisT->ui.screen->update();
}

void MainWindow::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	struct MainWindow* thisT = (struct MainWindow *)viewer_void;
	if (event.getKeySym() == "r" && event.keyDown())
	{
		PointCloudTRGBA::iterator it = thisT->m_pointCloudFile.cloud->end();
		for (int i = thisT->m_indicesVec.size() - 1; i >= 0; i--)
		{
			//if (thisT->m_indicesVec[i] > thisT->m_pointCloudFile.cloud->size())
			//	continue;
			it = thisT->m_pointCloudFile.cloud->begin() + thisT->m_indicesVec[i];
			thisT->m_pointCloudFile.cloud->erase(it);
		}

		thisT->m_mainViewer->removeShape("area_picking" + QString::number(thisT->m_iAreaPickingIndex).toStdString());
		thisT->m_mainViewer->removePointCloud("area_picking" + QString::number(thisT->m_iAreaPickingIndex).toStdString());

		thisT->showRealTimePointCloud(thisT->m_pointCloudFile.cloud);
	}

}

void MainWindow::areaPicking_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	struct MainWindow* thisT = (struct MainWindow *)args;

	thisT->m_indicesVec.clear();
	if (!event.getPointsIndices(thisT->m_indicesVec))
		return;

	PointCloudTRGBA::Ptr cloud(new PointCloudTRGBA);
	for (int i = 0; i < thisT->m_indicesVec.size(); ++i)
	{
		cloud->points.push_back(thisT->m_pointCloudFile.cloud->points.at(thisT->m_indicesVec[i]));
	}
	pcl::visualization::PointCloudColorHandlerCustom<PointTRGBA> red(cloud, 255, 0, 0);
	thisT->m_iAreaPickingIndex++;
	thisT->m_mainViewer->addPointCloud(cloud, red, "area_picking" + QString::number(thisT->m_iAreaPickingIndex).toStdString());
	thisT->m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "area_picking" + QString::number(thisT->m_iAreaPickingIndex).toStdString());

	thisT->ui.screen->update();
}

double MainWindow::disBetweenAB(PointTRGBA a, PointTRGBA b)
{
	const double ab = sqrt(pow((a.x - b.x), 2.0) + pow((a.y - b.y), 2.0) + pow((a.z - b.z), 2.0));
	return ab;
}

//显示点云，不重置相机角度
void MainWindow::showPointcloud()
{
	for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	{
		m_mainViewer->updatePointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
	}
	//m_mainViewer->resetCamera();
	ui.screen->update();
}

//添加点云到viewer,并显示点云
void MainWindow::showPointcloudAdd()
{
	for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	{
		m_mainViewer->addPointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
		m_mainViewer->updatePointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
	}
	m_mainViewer->resetCamera();
	ui.screen->update();
}

void MainWindow::setCloudColor(unsigned int r, unsigned int g, unsigned int b)
{
	// Set the new color
	for (size_t i = 0; i < m_pointCloudFile.cloud->size(); i++)
	{
		m_pointCloudFile.cloud->points[i].r = r;
		m_pointCloudFile.cloud->points[i].g = g;
		m_pointCloudFile.cloud->points[i].b = b;
		m_pointCloudFile.cloud->points[i].a = 255;
	}
}

void MainWindow::setAlpha(unsigned int a)
{
	for (size_t i = 0; i < m_pointCloudFile.cloud->size(); i++)
	{
		m_pointCloudFile.cloud->points[i].a = a;
	}
}

//设置停靠窗口的显示与隐藏
void MainWindow::dataDock()
{
	if (ui.dataAction->isChecked())
	{
		ui.dataDock->setVisible(true);
	}
	else
	{
		ui.dataDock->setVisible(false);
	}
}

void MainWindow::propertiesDock()
{
	if (ui.propertyAction->isChecked())
	{
		ui.propertyDock->setVisible(true);
	}
	else
	{
		ui.propertyDock->setVisible(false);
	}
}

void MainWindow::consoleDock()
{
	if (ui.consoleAction->isChecked())
	{
		ui.consoleDock->setVisible(true);
	}
	else
	{
		ui.consoleDock->setVisible(false);
	}
}

void MainWindow::rgbDock()
{
	if (ui.RGBAction->isChecked())
	{
		ui.RGBDock->setVisible(true);
	}
	else
	{
		ui.RGBDock->setVisible(false);
	}
}

void MainWindow::typeDock()
{
	if (ui.typeAction->isChecked())
	{
		ui.typeDock->setVisible(true);
	}
	else
	{
		ui.typeDock->setVisible(false);
	}
}

//绘制基本图形
void MainWindow::createSphere()
{
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	//ui.dataTree->clear();  //清空资源管理器的item
	m_mainViewer->removeAllShapes();
	//m_pointCloudFileVec.clear();  //清空点云容器

	pcl::PointXYZ p;
	p.x = 0; p.y = 0; p.z = 0;
	m_mainViewer->addSphere(p, 100, "sphere");

	m_mainViewer->resetCamera();
	ui.screen->update();

	// 输出窗口
	consoleLog(tr("Create sphere"), tr("Sphere"), "", tr("Succeeded"));
}

void MainWindow::createCylinder()
{
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	//ui.dataTree->clear();  //清空资源管理器的item
	m_mainViewer->removeAllShapes();
	//m_pointCloudFileVec.clear();  //清空点云容器

	m_mainViewer->addCylinder(*(new pcl::ModelCoefficients()), "cylinder");

	m_mainViewer->resetCamera();
	ui.screen->update();

	// 输出窗口
	consoleLog(tr("Create cylinder"), tr("Cylinder"), "", tr("Failed"));

}

//格式转换
void MainWindow::change()
{
	if (m_bIsChanging)
	{
		ui.statusBar->showMessage(tr("This point cloud file filtering"));
		return;
	}
	cloud_after_filter.reset(new PointCloudT);
	m_workThread->start();
}

void MainWindow::changeRun()
{
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud);

	/////*方法一：直通滤波器对点云进行处理*/
	//PointCloudT::Ptr cloud_after_PassThrough(new PointCloudT);
	////取得点云坐标极值
	//pcl::PointXYZ minPt, maxPt;
	//pcl::getMinMax3D(*cloud, minPt, maxPt);
	//pcl::PassThrough<pcl::PointXYZ> passthrough;
	//passthrough.setInputCloud(cloud);//输入点云
	//passthrough.setFilterFieldName("y");//对z轴进行操作
	//passthrough.setFilterLimits(-1.0, 2/*maxPt.z - 10.0*/);//设置直通滤波器操作范围
	//passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
	//passthrough.filter(*cloud_after_filter);//执行滤波，过滤结果保存在 cloud_after_PassThrough

	//emit updateSignal(QString("%1%2").arg(tr("Straight after filtering point cloud data points:")).arg(cloud_after_filter->points.size()));
	//return;
	///////****************************************************////////////////////

	///////****************************************************////////////////////
	/*方法二：体素滤波器实现下采样*/
	PointCloudT::Ptr cloud_after_voxelgrid(new PointCloudT);
	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setInputCloud(cloud);//输入点云数据
	voxelgrid.setLeafSize(0.01f, 0.01f, 0.01f);//AABB长宽高
	voxelgrid.filter(*cloud_after_voxelgrid);

	emit updateSignal(QString("%1%2").arg(tr("Voxel grid method after the point cloud data points:")).arg(cloud_after_voxelgrid->points.size()));

	/////////****************************************************////////////////////

	/////////****************************************************////////////////////
	///*方法三：统计滤波器滤波*/
	PointCloudT::Ptr cloud_after_StatisticalRemoval(new PointCloudT);

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	// 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;//创建滤波器对象
	Statistical.setInputCloud(cloud_after_voxelgrid);//设置待滤波的点云
	Statistical.setMeanK(50);//设置在进行统计时考虑查询点临近点数
	Statistical.setStddevMulThresh(1.0); //设置判断是否为离群点的阀值
	Statistical.filter(*cloud_after_StatisticalRemoval);

	//outlier
	Statistical.setNegative(true);
	Statistical.filter(*cloud_after_filter);

	emit updateSignal(QString("%1%2").arg(tr("After statistical analysis filtering point cloud data points:")).arg(cloud_after_StatisticalRemoval->points.size()));

	///////****************************************************////////////////////

	/////////****************************************************////////////////////
	///*方法四：条件滤波器*/
	//PointCloudT::Ptr cloud_after_Condition(new PointCloudT);

	//pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
	//range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
	//	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));  //GT表示大于等于
	//range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
	//	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));  //LT表示小于等于

	//pcl::ConditionalRemoval<pcl::PointXYZ> condition;
	//condition.setCondition(range_condition);
	//condition.setInputCloud(cloud);                   //输入点云
	//condition.setKeepOrganized(true);

	//condition.filter(*cloud_after_Condition);

	//emit updateSignal(QString("%1%2").arg(tr("After filtering under the condition point cloud data points:")).arg(cloud_after_Condition->points.size()));

	////showRealTimePointCloudAfterFilter(cloud_after_Condition);

	/////////****************************************************////////////////////

	/////////****************************************************////////////////////
	///*方法五：半径滤波器*/
	//PointCloudT::Ptr cloud_after_Radius(new PointCloudT);

	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

	//radiusoutlier.setInputCloud(cloud);    //设置输入点云
	//radiusoutlier.setRadiusSearch(100);     //设置半径为100的范围内找临近点
	//radiusoutlier.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除

	//radiusoutlier.filter(*cloud_after_Radius);

	//emit updateSignal(QString("%1%2").arg(tr("Radius after filtering point cloud data points:")).arg(cloud_after_Radius->points.size()));

	///////****************************************************////////////////////
	/*方法六：分割器*/
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);//设置分割模型
	seg.setMethodType(pcl::SAC_RANSAC);//设置采样一致性估计方法
	// 距离阈值 单位m
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(1);//设置距离阈值
	seg.setInputCloud(cloud_after_StatisticalRemoval);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return;
	}

	// 提取地面
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_after_StatisticalRemoval);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_after_filter);

	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("E:\\VTK\\B1_ground2222.pcd", *cloud_after_filter, false);
	emit updateSignal(QString("%1%2").arg(tr("After the split point cloud data points:")).arg(cloud_after_filter->points.size()));
	//// 提取除地面外的物体
	//extract.setNegative(true);
	//extract.filter(*cloud_after_filter);


	return;

}

void MainWindow::segmentation()
{
	if (0 == m_indicesVec.size() || 0 == m_pointCloudFile.cloud->points.size())
	{
		ui.statusBar->showMessage(tr("The point cloud count is 0"));
		return;
	}
	PointCloudTRGBA::Ptr cloud(new PointCloudTRGBA);
	if (0 != m_indicesVec.size())
		pcl::copyPointCloud(*m_pointCloudFile.cloud, m_indicesVec, *cloud);
	else if (0 == m_indicesVec.size() && 0 != m_pointCloudFile.cloud->points.size())
		pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud);
	euclideanSegmentation(cloud);

	ui.statusBar->showMessage(tr("This point cloud file segmentation over"));
}

//标准的欧几里德分割
void MainWindow::euclideanSegmentation(const PointCloudTRGBA::Ptr &cloud)
{
	PointCloud point;
	point.cloud.reset(new PointCloudTRGBA);
	// 建立kd-tree对象用来搜索 .
	pcl::search::KdTree<PointTRGBA>::Ptr kdtree(new pcl::search::KdTree<PointTRGBA>);
	kdtree->setInputCloud(cloud);

	// Euclidean 聚类对象.
	pcl::EuclideanClusterExtraction<PointTRGBA> clustering;
	// 设置聚类的最小值 40cm (small values may cause objects to be divided
	// in several clusters, whereas big values may join objects in a same cluster).
	clustering.setClusterTolerance(0.5);
	// 设置聚类的小点数和最大点云数
	clustering.setMinClusterSize(100);//100
	clustering.setMaxClusterSize(35000);//25000
	clustering.setSearchMethod(kdtree);
	clustering.setInputCloud(cloud);
	std::vector<pcl::PointIndices> clustersIndices;
	clustering.extract(clustersIndices);

	point.m_indicesVec = clustersIndices;

	// For every cluster...
	for (std::vector<pcl::PointIndices>::const_iterator i = clustersIndices.begin(); i != clustersIndices.end(); ++i)
	{
		//添加所有的点云到一个新的点云中
		PointCloudTRGBA::Ptr cluster(new PointCloudTRGBA);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// 保存
		if (cluster->points.size() <= 0)
			break;

		*point.cloud += *cluster;
	}

	m_mainViewer->removeShape("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());
	m_mainViewer->removePointCloud("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());

	// 节点还可以添加子节点
	m_pCarItem->setHidden(false);
	m_pTreeItem->setHidden(false);
	QString strCar = tr("Car_") + QString::number(m_iSegmentationCarIndex);
	QTreeWidgetItem *cloudCarItem = new QTreeWidgetItem(QStringList() << strCar);
	cloudCarItem->setIcon(0, QIcon(":/Resources/images/icon.png"));
	m_pCarItem->addChild(cloudCarItem);

	QString strTree = tr("Tree_") + QString::number(m_iSegmentationTreeIndex);
	QTreeWidgetItem *cloudTreeItem = new QTreeWidgetItem(QStringList() << strTree);
	cloudTreeItem->setIcon(0, QIcon(":/Resources/images/icon.png"));
	m_pTreeItem->addChild(cloudTreeItem);

	ui.typeTree->addTopLevelItem(m_pCarItem);
	ui.typeTree->addTopLevelItem(m_pTreeItem);
	point.m_strPointName = strCar.toStdString();
	m_pointCloudMap.insert(std::make_pair(strCar, point));
	pcl::visualization::PointCloudColorHandlerCustom<PointTRGBA> green(point.cloud, 0, 255, 0);
	m_mainViewer->addPointCloud(point.cloud, green, strCar.toStdString());
	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, strCar.toStdString());
	// 将cloud_id所对应的点云设置成透明
	m_iSegmentationCarIndex++;

	setSelectPointPropertyTable(strCar, cloud);
	ui.screen->update();
}

//条件欧几里德分割
void MainWindow::conditionalEuclideanSegmentation(const PointCloudT::Ptr &cloud)
{
	PointCloudT::Ptr add_cloud(new PointCloudT);
	// 建立kd-tree对象用来搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	// Euclidean 聚类对象.
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	clustering.setClusterTolerance(0.4);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	//设置每次检测一对点云时的函数
	clustering.setConditionFunction(&customCondition);
	std::vector<pcl::PointIndices> clusters;
	clustering.segment(clusters);

	// For every cluster...
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//添加所有的点云到一个新的点云中
		PointCloudT::Ptr cluster(new PointCloudT);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// 保存
		if (cluster->points.size() <= 0)
			break;

		*add_cloud += *cluster;
	}
	//删除红色框选的点云
	m_mainViewer->removeShape("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());
	m_mainViewer->removePointCloud("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());

	static int iSegmentationIndex = 1;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(add_cloud, 0, 0, 255);
	m_mainViewer->removeShape("segmentation" + QString::number(iSegmentationIndex - 1).toStdString());
	m_mainViewer->addPointCloud(add_cloud, blue, "segmentation" + QString::number(iSegmentationIndex).toStdString());
	iSegmentationIndex++;

	ui.screen->update();
}

// 如果这个函数返回的是真，这这个候选点将会被加入聚类中
bool MainWindow::customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// Do whatever you want here.做你想做的条件的筛选
	if (candidatePoint.y < seedPoint.y)  //如果候选点的Y的值小于种子点的Y值（就是之前被选择为聚类的点），则不满足条件，返回假
		return false;

	return true;
}

void MainWindow::issKeyPoint()
{
	clock_t start = clock();
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud);
	//iss关键点提取
	PointCloudT::Ptr cloud_src_is(new PointCloudT);

	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_det;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());

	//参数设置
	iss_det.setSearchMethod(kdTree);
	iss_det.setSalientRadius(2.4);
	iss_det.setNonMaxRadius(1.6);
	iss_det.setThreshold21(0.975);
	iss_det.setThreshold32(0.975);
	iss_det.setMinNeighbors(5);
	iss_det.setNumberOfThreads(4);
	iss_det.setInputCloud(cloud);
	iss_det.compute(*cloud_src_is);

	clock_t end = clock();
	QString strTimeCost = QString::number((double)(end - start) / CLOCKS_PER_SEC);
	// 输出窗口
	consoleLog(tr("Key point extract"), tr("iss"), "", tr("Time cost: ") + strTimeCost + tr(" s"));

	PointCloudT::Ptr cloud_src(new PointCloudT);
	pcl::copyPointCloud(*cloud_src_is, *cloud_src);

	//可视化
	visualize_pcd(cloud, cloud_src);
}

void MainWindow::siftKeyPoint()
{
	clock_t start = clock();
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud);
	//设定参数值
	const float min_scale = 0.002f; //the standard deviation of the smallest scale in the scale space
	const int n_octaves = 3;//尺度空间层数,小、关键点多
	const int n_scales_per_octave = 3;//the number of scales to compute within each octave
	const float min_contrast = 0.0001f;//根据点云，设置大小，越小关键点越多

	//sift关键点检测
	//pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale > sift_src;
	//pcl::PointCloud<pcl::PointWithScale> result_src;
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
	//sift_src.setSearchMethod(tree_src);
	//sift_src.setScales(min_scale, n_octaves, n_scales_per_octave);
	//sift_src.setMinimumContrast(min_contrast);
	//sift_src.setInputCloud(cloud);
	//sift_src.compute(result_src);

	//clock_t end = clock();
	//QString strTimeCost = QString::number((double)(end - start) / CLOCKS_PER_SEC);
	//// 输出窗口
	//consoleLog(tr("Key point extract"), tr("sift"), "", tr("Time cost: ") + strTimeCost + tr(" s"));

	//PointCloudT::Ptr cloud_src(new PointCloudT);
	//pcl::copyPointCloud(result_src, *cloud_src);


	////可视化
	//visualize_pcd(cloud, cloud_src);
}

void MainWindow::visualize_pcd(PointCloudT::Ptr pcd_src, PointCloudT::Ptr pcd_tgt)
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);

	m_mainViewer->addPointCloud(pcd_src, src_h, "source cloud");
	m_mainViewer->addPointCloud(pcd_tgt, tgt_h, "tgt cloud");

	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "tgt cloud");

}

void MainWindow::showRealTimePointCloudAfterFilter(const PointCloudT::Ptr &cloud)
{
	//从点云中移除NAN点也就是无效点
	std::vector<int> indices;
	indices.clear();
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	//设置资源管理器
	string strFileName = m_pointCloudFile.m_strSubName.substr(0, m_pointCloudFile.m_strSubName.size() - 4);
	strFileName = strFileName + "_filter.pcd";
	QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(strFileName.c_str()));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);
	m_ulTotalPoints += m_pointCloudFile.cloud->points.size();

	//m_mainViewer->addPointCloud(cloud, "cloud_realTime_filter");
	//m_mainViewer->updatePointCloud(cloud, "cloud_realTime_filter");

	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	pcl::copyPointCloud(*cloud, *m_pointCloudFile.cloud);
	if (m_pointCloudFile.cloud->points[0].r == 0 && m_pointCloudFile.cloud->points[0].g == 0 && m_pointCloudFile.cloud->points[0].b == 0)
	{
		setCloudColor(255, 255, 255);
	}
	m_pointCloudFile.m_strSubName = strFileName;
	setAlpha(200);  //设置点云为不透明255
	m_pointCloudFileVec.push_back(m_pointCloudFile);  //将点云导入点云容器

	setPropertyTable(m_pointCloudFileVec.size() - 1);
	m_iCurPointFileIndex = m_pointCloudFileVec.size() - 1;

	ui.screen->update();

}

void MainWindow::showRealTimePointCloud(const PointCloudTRGBA::Ptr &cloud)
{
	m_mainViewer->removeAllPointClouds();  //从viewer中移除所有点云
	m_mainViewer->removeAllShapes(); //这个remove更彻底
	::keybd_event(0x58, 0, 0, 0);
	Sleep(20);
	::keybd_event(0x58, 0, KEYEVENTF_KEYUP, 0);
	m_mainViewer->addPointCloud(cloud, "cloud_realTime");
	m_mainViewer->updatePointCloud(cloud, "cloud_realTime");

	ui.screen->update();

}

// Change theme: Windows/Darcula
void MainWindow::windowsTheme()
{
	//加载QSS样式表
	QFile qss(":/Resources/qss/Windows.qss");
	qss.open(QFile::ReadOnly);
	qApp->setStyleSheet(qss.readAll());
	qss.close();

	// 输出窗口
	consoleLog(tr("Change theme"), tr("Windows theme"), "", "");

	m_iThemeId = 0;
}

void MainWindow::darculaTheme()
{
	QFile qss(":/Resources/qss/Darcula.qss");
	qss.open(QFile::ReadOnly);
	qApp->setStyleSheet(qss.readAll());
	qss.close();

	// 输出窗口
	consoleLog(tr("Change theme"), tr("Darcula theme"), "", "");

	m_iThemeId = 1;
}

// Change language: English/Chinese
void MainWindow::langEnglish()
{
	// 输出窗口
	consoleLog(tr("Change language"), tr("English"), "", "");
	changeLanguage(UI_EN);
}

void MainWindow::langChinese()
{
	// 输出窗口
	consoleLog(tr("Change language"), tr("Chinese"), "", "");
	changeLanguage(UI_ZH);
}

//关于
void MainWindow::about()
{
	AboutWin *aboutwin = new AboutWin(this);
	aboutwin->setModal(true);
	aboutwin->show();

	// 输出窗口
	consoleLog(tr("About"), tr("Point cloud tools about"), "http://www.baidu.com", tr("Welcome to my blog!"));
}

//帮助
void MainWindow::help()
{
	QDesktopServices::openUrl(QUrl(QLatin1String("https://github.com/Ewenwan/MVision/tree/master/PCL_APP")));

	// 输出窗口
	consoleLog(tr("Help"), tr("Point cloud tools help"), "https://github.com/Ewenwan/MVision/tree/master/PCL_APP", "");

}


/*********************************************/
/*****************界面槽函数*****************/
/********************************************/
void MainWindow::colorBtnPressed()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	// 如果未选中任何点云，则对视图窗口中的所有点云进行着色
	if (selected_item_count == 0)
	{
		for (int i = 0; i != m_pointCloudFileVec.size(); i++)
		{
			for (int j = 0; j != m_pointCloudFileVec[i].cloud->points.size(); j++)
			{
				m_pointCloudFileVec[i].cloud->points[j].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				m_pointCloudFileVec[i].cloud->points[j].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				m_pointCloudFileVec[i].cloud->points[j].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			}
		}

		// 输出窗口
		consoleLog(tr("Random color"), tr("All point clous"), "", "");

	}
	else
	{
		for (int i = 0; i != selected_item_count; i++)
		{
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			for (int j = 0; j != m_pointCloudFileVec[cloud_id].cloud->size(); j++)
			{
				m_pointCloudFileVec[cloud_id].cloud->points[j].r = m_uRed;
				m_pointCloudFileVec[cloud_id].cloud->points[j].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				m_pointCloudFileVec[cloud_id].cloud->points[j].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			}
		}

		// 输出窗口
		consoleLog(tr("Random color"), tr("Point clouds selected"), "", "");
	}
	showPointcloud();
}

void MainWindow::RGBsliderReleased()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	// 如果未选中任何点云，则对视图窗口中的所有点云进行着色
	if (selected_item_count == 0)
	{
		for (int i = 0; i != m_pointCloudFileVec.size(); i++)
		{
			for (int j = 0; j != m_pointCloudFileVec[i].cloud->points.size(); j++)
			{
				m_pointCloudFileVec[i].cloud->points[j].r = m_uRed;
				m_pointCloudFileVec[i].cloud->points[j].g = m_uGreen;
				m_pointCloudFileVec[i].cloud->points[j].b = m_uBlue;
			}
		}

		// 输出窗口
		consoleLog(tr("Change cloud color"), tr("All point clouds"), QString::number(m_uRed) + " " + QString::number(m_uGreen) + " " + QString::number(m_uBlue), "");
	}
	else
	{
		for (int i = 0; i != selected_item_count; i++)
		{
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			for (int j = 0; j != m_pointCloudFileVec[cloud_id].cloud->size(); j++)
			{
				m_pointCloudFileVec[cloud_id].cloud->points[j].r = m_uRed;
				m_pointCloudFileVec[cloud_id].cloud->points[j].g = m_uGreen;
				m_pointCloudFileVec[cloud_id].cloud->points[j].b = m_uBlue;
			}
		}
		// 输出窗口
		consoleLog(tr("Change cloud color"), tr("Point clouds selected"), QString::number(m_uRed) + " " + QString::number(m_uGreen) + " " + QString::number(m_uBlue), "");
	}
	showPointcloud();
}

//设置所有点云的尺寸
void MainWindow::pSliderReleased()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0)
	{
		for (int i = 0; i != m_pointCloudFileVec.size(); i++)
		{
			m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				m_uPointSize, "cloud" + QString::number(i).toStdString());
		}
		// 输出窗口
		consoleLog(tr("Change cloud size"), tr("All point clouds"), tr("Size:") + QString::number(m_uPointSize), "");
	}
	else
	{
		for (int i = 0; i != selected_item_count; i++)
		{
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				m_uPointSize, "cloud" + QString::number(cloud_id).toStdString());
		}
		// 输出窗口
		consoleLog(tr("Change cloud size"), tr("Point clouds selected"), tr("Size:") + QString::number(m_uPointSize), "");
	}
	ui.screen->update();
}

void MainWindow::pSliderChanged(int value)
{
	m_uPointSize = value;
	ui.sizeLCD->display(value);
	pSliderReleased();
}

void MainWindow::rSliderChanged(int value)
{
	m_uRed = value;
	ui.rLCD->display(value);
	//RGBsliderReleased();
}

void MainWindow::gSliderChanged(int value)
{
	m_uGreen = value;
	ui.gLCD->display(value);
	//RGBsliderReleased();
}

void MainWindow::bSliderChanged(int value)
{
	m_uBlue = value;
	ui.bLCD->display(value);
	//RGBsliderReleased();
}

void MainWindow::cooCbxChecked(int value)
{
	switch (value)
	{
	case 0:
		m_mainViewer->removeCoordinateSystem(0);  //移除坐标系
		// 输出窗口
		consoleLog(tr("Remove coordinate system"), tr("Remove"), "", "");
		break;
	case 2:
		m_mainViewer->addCoordinateSystem(10.0, 0);  //添加坐标系
		// 输出窗口
		consoleLog(tr("Add coordinate system"), tr("Add"), "", "");
		ui.statusBar->showMessage(tr("Red is the X axis, green is the Y axis, blue is Z axis")); //红色是X轴，绿色是Y轴，蓝色是Z
		break;
	}

	ui.screen->update();
}

void MainWindow::bgcCbxChecked(int value)
{
	switch (value)
	{
	case 0:
		m_mainViewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
		// 输出窗口
		consoleLog(tr("Change bg color"), tr("Background"), "30 30 30", "");
		break;
	case 2:
		// 注意：setBackgroundColor()接收的是0-1的double型参数
		m_mainViewer->setBackgroundColor(255 / 255.0, 255 / 255.0, 255 / 255.0);
		// 输出窗口
		consoleLog(tr("Change bg color"), tr("Background"), "255 255 255", "");
		break;
	}

	ui.screen->update();
}

//通过颜色对话框改变点云颜色
void MainWindow::pointcolorChangedData()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("Select color for point cloud"));

	if (color.isValid()) //判断所选的颜色是否有效
	{
		QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
		int selected_item_count = ui.dataTree->selectedItems().size();
		if (selected_item_count == 0)
		{
			for (int i = 0; i != m_pointCloudFileVec.size(); i++)
			{
				for (int j = 0; j != m_pointCloudFileVec[i].cloud->points.size(); j++)
				{
					m_pointCloudFileVec[i].cloud->points[j].r = color.red();
					m_pointCloudFileVec[i].cloud->points[j].g = color.green();
					m_pointCloudFileVec[i].cloud->points[j].b = color.blue();
				}
			}
			// 输出窗口
			consoleLog(tr("Change cloud color"), tr("All point clouds"), QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
		}
		else
		{
			for (int i = 0; i != selected_item_count; i++)
			{
				int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
				for (int j = 0; j != m_pointCloudFileVec[cloud_id].cloud->size(); j++)
				{
					m_pointCloudFileVec[cloud_id].cloud->points[j].r = color.red();
					m_pointCloudFileVec[cloud_id].cloud->points[j].g = color.green();
					m_pointCloudFileVec[cloud_id].cloud->points[j].b = color.blue();
				}
			}
			// 输出窗口
			consoleLog(tr("Change cloud color"), tr("Point clouds selected"), QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
		}
		//颜色的改变同步至RGB停靠窗口
		ui.rSlider->setValue(color.red());
		ui.gSlider->setValue(color.green());
		ui.bSlider->setValue(color.blue());

		showPointcloud();
	}
}

//通过颜色对话框改变背景颜色
void MainWindow::bgcolorChanged()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("Select color for point cloud"));
	if (color.isValid())
	{
		m_mainViewer->setBackgroundColor(color.red() / 255.0,
			color.green() / 255.0, color.blue() / 255.0);
		// 输出窗口
		consoleLog(tr("Change bg color"), tr("Background"), QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
		showPointcloud();
	}
}

//三视图
void MainWindow::mainview()
{
	m_mainViewer->setCameraPosition(0, -1, 0, 0.5, 0.5, 0.5, 0, 0, 1);
	ui.screen->update();
}

void MainWindow::leftview()
{
	m_mainViewer->setCameraPosition(-1, 0, 0, 0, 0, 0, 0, 0, 1);
	ui.screen->update();
}

void MainWindow::topview()
{
	m_mainViewer->setCameraPosition(0, 0, 1, 0, 0, 0, 0, 1, 0);
	ui.screen->update();
}

//设置属性管理窗口
void MainWindow::setPropertyTable(int iIndex)
{
	ui.propertyTable->clearContents();

	ui.propertyTable->setItem(0, 0, new QTableWidgetItem(tr("Clouds")));
	ui.propertyTable->setItem(1, 0, new QTableWidgetItem(tr("Points")));
	ui.propertyTable->setItem(2, 0, new QTableWidgetItem(tr("Total points")));
	ui.propertyTable->setItem(3, 0, new QTableWidgetItem(tr("RGB")));
	ui.propertyTable->setItem(4, 0, new QTableWidgetItem(tr("Min XYZ")));
	ui.propertyTable->setItem(5, 0, new QTableWidgetItem(tr("Max XYZ")));
	//提取当前点云的RGB,点云数量等信息
	int cloud_size = m_pointCloudFileVec[iIndex].cloud->points.size();
	UINT32 iTotal = 0;
	for (UINT i = 0; i < m_pointCloudFileVec.size(); ++i)
	{
		iTotal += m_pointCloudFileVec[i].cloud->points.size();
	}
	unsigned int cloud_r = m_pointCloudFileVec[iIndex].cloud->points[0].r;
	unsigned int cloud_g = m_pointCloudFileVec[iIndex].cloud->points[0].g;
	unsigned int cloud_b = m_pointCloudFileVec[iIndex].cloud->points[0].b;

	bool multi_color = true;
	if (m_pointCloudFileVec[iIndex].cloud->points.begin()->r == (m_pointCloudFileVec[iIndex].cloud->points.end() - 1)->r) //判断点云单色多色的条件（不是很严谨）
		multi_color = false;

	ui.propertyTable->setItem(0, 1, new QTableWidgetItem(QString::number(iIndex)));
	ui.propertyTable->setItem(1, 1, new QTableWidgetItem(QString::number(cloud_size)));
	ui.propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(iTotal)));
	ui.propertyTable->setItem(3, 1, new QTableWidgetItem(multi_color ? "Multi Color" : (QString::number(cloud_r) + " " + QString::number(cloud_g) + " " + QString::number(cloud_b))));
	ui.propertyTable->setItem(4, 1, new QTableWidgetItem(QString::number(m_pointCloudFileVec[iIndex].m_minPt.x) + " " + QString::number(m_pointCloudFileVec[iIndex].m_minPt.y) + " " + QString::number(m_pointCloudFileVec[iIndex].m_minPt.z)));
	ui.propertyTable->setItem(5, 1, new QTableWidgetItem(QString::number(m_pointCloudFileVec[iIndex].m_maxPt.x) + " " + QString::number(m_pointCloudFileVec[iIndex].m_maxPt.y) + " " + QString::number(m_pointCloudFileVec[iIndex].m_maxPt.z)));

}

void MainWindow::setPointPropertyTable(PointTRGBA pt)
{
	ui.propertyTable->clearContents();
	ui.propertyTable->setItem(0, 0, new QTableWidgetItem("X"));
	ui.propertyTable->setItem(1, 0, new QTableWidgetItem("Y"));
	ui.propertyTable->setItem(2, 0, new QTableWidgetItem("Z"));
	ui.propertyTable->setItem(3, 0, new QTableWidgetItem("Index"));
	ui.propertyTable->setItem(4, 0, new QTableWidgetItem("Longitude"));
	ui.propertyTable->setItem(5, 0, new QTableWidgetItem("Latitude"));

	ui.propertyTable->setItem(0, 1, new QTableWidgetItem(QString::number(pt.x)));
	ui.propertyTable->setItem(1, 1, new QTableWidgetItem(QString::number(pt.y)));
	ui.propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(pt.z)));
	std::vector<PointAttribute>::const_iterator it = m_pointAttributeVec.cbegin();
	for (; it != m_pointAttributeVec.cend(); ++it)
	{
		if ((*it).pt.x == pt.x && (*it).pt.y == pt.y && (*it).pt.z == pt.z)
		{
			ui.propertyTable->setItem(3, 1, new QTableWidgetItem(QString::number((*it).iIndex)));
			ui.propertyTable->setItem(4, 1, new QTableWidgetItem(QString::number((*it).outNav.dLongitude)));
			ui.propertyTable->setItem(5, 1, new QTableWidgetItem(QString::number((*it).outNav.dLatitude)));
			break;
		}
	}

}

void MainWindow::setSelectPointPropertyTable(const QString &strName, const PointCloudTRGBA::Ptr &cloud)
{
	ui.propertyTable->clearContents();

	ui.propertyTable->setItem(0, 0, new QTableWidgetItem(tr("Name")));
	ui.propertyTable->setItem(1, 0, new QTableWidgetItem(tr("Points")));
	ui.propertyTable->setItem(2, 0, new QTableWidgetItem(tr("Min XYZ")));
	ui.propertyTable->setItem(3, 0, new QTableWidgetItem(tr("Max XYZ")));

	//取得点云坐标极值
	PointTRGBA minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	ui.propertyTable->setItem(0, 1, new QTableWidgetItem(strName));
	ui.propertyTable->setItem(1, 1, new QTableWidgetItem(QString::number(cloud->points.size())));
	ui.propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(minPt.x) + " " + QString::number(minPt.y) + " " + QString::number(minPt.z)));
	ui.propertyTable->setItem(3, 1, new QTableWidgetItem(QString::number(maxPt.x) + " " + QString::number(maxPt.y) + " " + QString::number(maxPt.z)));

}

void MainWindow::setPropertyTable()
{
	QStringList header;
	header << tr("Property") << tr("Value");
	ui.propertyTable->setHorizontalHeaderLabels(header);
	ui.propertyTable->setSelectionMode(QAbstractItemView::NoSelection); // 禁止点击属性管理器的 item
}

void MainWindow::setConsoleTable()
{
	// 设置输出窗口
	QStringList header2;
	header2 << tr("Time") << tr("Operation") << tr("Operation obeject") << tr("Details") << tr("Note");
	ui.consoleTable->setHorizontalHeaderLabels(header2);
	ui.consoleTable->setColumnWidth(0, 200);
	ui.consoleTable->setColumnWidth(1, 200);
	ui.consoleTable->setColumnWidth(2, 200);
	ui.consoleTable->setColumnWidth(3, 250);

	//ui.consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); //设置不可编辑
	ui.consoleTable->verticalHeader()->setDefaultSectionSize(22); //设置行距

	ui.consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
	ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);  // 禁止点击输出窗口的 item
}

void MainWindow::setTypeTree()
{
	////设置资源管理器
	ui.typeTree->setColumnCount(1);
	// 设置表头
	ui.typeTree->setHeaderLabels(QStringList() << tr("Point Cloud"));
	ui.typeTree->setSelectionMode(QAbstractItemView::ExtendedSelection);
	// 添加一级节点
	m_pCarItem = new QTreeWidgetItem(ui.typeTree, QStringList() << tr("Car"));
	m_pTreeItem = new QTreeWidgetItem(ui.typeTree, QStringList() << tr("Tree"));
	m_pCarItem->setHidden(true);
	m_pTreeItem->setHidden(true);
}

void MainWindow::consoleLog(QString operation, QString subname, QString filename, QString note)
{
	int rows = ui.consoleTable->rowCount();
	ui.consoleTable->setRowCount(++rows);
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString strTime = time.toString("yyyy-MM-dd hh:mm:ss"); //设置显示格式
	ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(strTime));
	ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
	ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(subname));
	ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(filename));
	ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(note));

	ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
	qDebug() << operation;
}

//QTreeWidget的item的点击相应函数
void MainWindow::itemSelectedData(QTreeWidgetItem* item, int count)
{
	count = ui.dataTree->indexOfTopLevelItem(item);  //获取item的行号
	m_iCurPointFileIndex = count;
	//for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	//{
	//	m_mainViewer->updatePointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
	//	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud" + QString::number(i).toStdString());
	//}

	////选中item所对应的点云尺寸变大
	//QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	//int selected_item_count = ui.dataTree->selectedItems().size();
	//for (int i = 0; i != selected_item_count; i++)
	//{
	//	int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
	//	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud" + QString::number(cloud_id).toStdString());
	//}
	m_pointCloudFile = m_pointCloudFileVec[count];
	
	setPropertyTable(count);
	ui.screen->update();
}

//QTreeWidget的item的右击响应函数
void MainWindow::popMenuData(const QPoint&)
{
	QTreeWidgetItem* curItem = ui.dataTree->currentItem(); //获取当前被点击的节点
	if (curItem == NULL)return;           //这种情况是右键的位置不在treeItem的范围内，即在空白位置右击
	QString name = curItem->text(0);
	int id = ui.dataTree->indexOfTopLevelItem(curItem);
	string cloud_id = "cloud" + QString::number(id).toStdString();

	QAction hideItemAction(tr("Hide"), this);
	QAction showItemAction(tr("Show"), this);
	QAction deleteItemAction(tr("Delete"), this);
	QAction changeColorAction(tr("Change color"), this);

	connect(&hideItemAction, &QAction::triggered, this, &MainWindow::hideItemData);
	connect(&showItemAction, &QAction::triggered, this, &MainWindow::showItemData);
	connect(&deleteItemAction, &QAction::triggered, this, &MainWindow::deleteItemData);
	connect(&changeColorAction, &QAction::triggered, this, &MainWindow::pointcolorChangedData);

	QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&hideItemAction);
	menu.addAction(&showItemAction);
	menu.addAction(&deleteItemAction);
	menu.addAction(&changeColorAction);

	if (m_pointCloudFileVec[id].m_bVisible == true)
	{
		menu.actions()[1]->setVisible(false);
		menu.actions()[0]->setVisible(true);
	}
	else
	{
		menu.actions()[1]->setVisible(true);
		menu.actions()[0]->setVisible(false);
	}

	menu.exec(QCursor::pos()); //在当前鼠标位置显示
}

// consoleTable 右击响应事件
void MainWindow::popMenuInConsole(const QPoint&)
{
	QAction clearConsoleAction(tr("Clear console"), this);

	connect(&clearConsoleAction, &QAction::triggered, this, &MainWindow::clearConsole);

	QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&clearConsoleAction);

	menu.exec(QCursor::pos()); //在当前鼠标位置显示
}

// 清空 consoleTable
void MainWindow::clearConsole()
{
	ui.consoleTable->clearContents();
	ui.consoleTable->setRowCount(0);
}

//QTreeWidget的item的点击相应函数
void MainWindow::itemSelectedType(QTreeWidgetItem* item, int count)
{
	count = ui.dataTree->indexOfTopLevelItem(item);  //获取item的行号
	QTreeWidgetItem* curItem = ui.typeTree->currentItem(); //获取当前被点击的节点
	if (curItem == NULL)
		return;

	//选中item所对应的点云尺寸变大
	QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
	int selected_item_count = ui.typeTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++)
	{
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		if ("Car" == name || "Tree" == name)
			return;
		//m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name.toStdString());
		setSelectPointPropertyTable(name, m_pointCloudMap[name].cloud);
	}
	ui.screen->update();
}

//QTreeWidget的item的右击响应函数
void MainWindow::popMenuType(const QPoint&)
{
	QTreeWidgetItem* curItem = ui.typeTree->currentItem(); //获取当前被点击的节点
	if (curItem == NULL)return;           //这种情况是右键的位置不在treeItem的范围内，即在空白位置右击
	QString name = curItem->text(0);
	int id = ui.typeTree->indexOfTopLevelItem(curItem);

	QAction hideItemAction(tr("Hide"), this);
	QAction showItemAction(tr("Show"), this);
	//QAction deleteItemAction(tr("Delete"), this);
	QAction changeColorAction(tr("Change color"), this);

	connect(&hideItemAction, &QAction::triggered, this, &MainWindow::hideItemType);
	connect(&showItemAction, &QAction::triggered, this, &MainWindow::showItemType);
	//connect(&deleteItemAction, &QAction::triggered, this, &MainWindow::deleteItemType);
	connect(&changeColorAction, &QAction::triggered, this, &MainWindow::pointcolorChangedType);

	QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&hideItemAction);
	menu.addAction(&showItemAction);
	//menu.addAction(&deleteItemAction);
	menu.addAction(&changeColorAction);

	if (m_pointCloudMap[name].m_bVisible == true)
	{
		menu.actions()[1]->setVisible(false);
		menu.actions()[0]->setVisible(true);
	}
	else
	{
		menu.actions()[1]->setVisible(true);
		menu.actions()[0]->setVisible(false);
	}

	menu.exec(QCursor::pos()); //在当前鼠标位置显示
}


void MainWindow::hideItemData()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	for (int i = 0; i != ui.dataTree->selectedItems().size(); i++)
	{
		//TODO hide之后，item变成灰色，再次右击item时，“hideItem” 选项变成 “showItem”
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		const int id = ui.dataTree->indexOfTopLevelItem(curItem);
		hidePointCloudFile(id);
		QColor item_color = QColor(112, 122, 132, 255);
		curItem->setTextColor(0, item_color);
		m_pointCloudFileVec[id].m_bVisible = false;
	}

	// 输出窗口
	consoleLog(tr("Hide point clouds"), tr("Point cloud files selected"), "", "");

	ui.screen->update(); //刷新视图窗口，不能省略
}

void MainWindow::hidePointCloudFile(const int &id)
{
	string cloud_id = "cloud" + QString::number(id).toStdString();

	// 将cloud_id所对应的点云设置成透明
	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.0, cloud_id, 0);

}

void MainWindow::showItemData()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	for (int i = 0; i != ui.dataTree->selectedItems().size(); i++)
	{
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui.dataTree->indexOfTopLevelItem(curItem);
		showPointCloudFile(id);
		QColor item_color;
		if (m_iThemeId == 0)
		{
			item_color = QColor(0, 0, 0, 255);
		}
		else
		{
			item_color = QColor(241, 241, 241, 255);
		}
		curItem->setTextColor(0, item_color);
		m_pointCloudFileVec[id].m_bVisible = true;
	}

	// 输出窗口
	consoleLog(tr("Show point clouds"), tr("Point cloud files selected"), "", "");

	ui.screen->update(); //刷新视图窗口，不能省略

}

void MainWindow::showPointCloudFile(const int & id)
{
	string cloud_id = "cloud" + QString::number(id).toStdString();
	// 将cloud_id所对应的点云设置成透明
	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, cloud_id, 0);
}

void MainWindow::deleteItemData()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	const int selected_item_count = ui.dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++)
	{
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui.dataTree->indexOfTopLevelItem(curItem);
		auto it = m_pointCloudFileVec.begin() + ui.dataTree->indexOfTopLevelItem(curItem);
		// 删除点云之前，将其点的数目保存
		int delete_points = (*it).cloud->points.size();
		it = m_pointCloudFileVec.erase(it);

		m_ulTotalPoints -= delete_points;
		if (m_pointCloudFileVec.size() > 0)
		{
			setPropertyTable(0);
		}
		else
		{
			ui.propertyTable->clearContents();
		}

		ui.dataTree->takeTopLevelItem(ui.dataTree->indexOfTopLevelItem(curItem));
	}

	// 移除之后再添加，避免 id 和资源管理树行号不一致的情况
	m_mainViewer->removeAllPointClouds();
	for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	{
		m_mainViewer->addPointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
		m_mainViewer->updatePointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
		if (m_pointCloudFileVec[i].m_bVisible)
		{
			showPointCloudFile(i);
		}
		else
		{
			hidePointCloudFile(i);
		}
	}

	// 输出窗口
	consoleLog(tr("Delete point clouds"), tr("Point clouds file selected"), "", "");

	ui.screen->update();
}

////type
void MainWindow::hideItemType()
{
	QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
	for (int i = 0; i != ui.typeTree->selectedItems().size(); i++)
	{
		//TODO hide之后，item变成灰色，再次右击item时，“hideItem” 选项变成 “showItem”
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);

		// 将原有点云文件设置成透明
		m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.0, name.toStdString(), 0);

		QColor item_color = QColor(112, 122, 132, 255);
		curItem->setTextColor(0, item_color);
		m_pointCloudMap[name].m_bVisible = false;
	}

	ui.screen->update();
}

void MainWindow::showItemType()
{
	QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
	for (int i = 0; i != ui.typeTree->selectedItems().size(); i++)
	{
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		// 将cloud_id所对应的点云设置成透明
		m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, name.toStdString(), 0);
		QColor item_color;
		if (m_iThemeId == 0)
		{
			item_color = QColor(0, 0, 0, 255);
		}
		else
		{
			item_color = QColor(241, 241, 241, 255);
		}
		curItem->setTextColor(0, item_color);
		m_pointCloudMap[name].m_bVisible = true;
	}

	ui.screen->update(); //刷新视图窗口，不能省略

}

void MainWindow::deleteItemType()
{
	QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
	int selected_item_count = ui.typeTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++)
	{
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);

		//先将所选的颜色图层删除
		m_mainViewer->removeShape(name.toStdString());

		//再将原始底图中所选的删除
		std::vector<int> indices;
		indices.clear();
		for (std::vector<pcl::PointIndices>::const_iterator i = m_pointCloudMap[name].m_indicesVec.begin(); i != m_pointCloudMap[name].m_indicesVec.end(); ++i)
		{
			//添加所有的点云到一个新的点云中
			PointCloudT::Ptr cluster(new PointCloudT);
			for (std::vector<int>::const_iterator it = i->indices.begin(); it != i->indices.end(); ++it)
				indices.push_back(*it);
		}

		//PointCloudTRGBA::Ptr pointCloud(new PointCloudTRGBA);
		//pointCloud = m_pointCloudFile.cloud;
		PointCloudTRGBA::iterator it = m_pointCloudFile.cloud->end();
		for (int i = indices.size() - 1; i >= 0; i--)
		{
			it = m_pointCloudFile.cloud->begin() + indices[i];
			m_pointCloudFile.cloud->erase(it);
		}
		m_pointCloudFileVec[m_iCurPointFileIndex] = m_pointCloudFile;

		//m_pointCloudMap.erase(name);

		std::map<QString, PointCloud>::iterator key = m_pointCloudMap.find(name);
		if (key != m_pointCloudMap.end())
		{
			m_pointCloudMap.erase(key);
		}

		if (m_pointCloudMap.size() > 0)
		{
			std::map<QString, PointCloud>::const_iterator it = m_pointCloudMap.cbegin();
			const QString &strName = it->first;
			setSelectPointPropertyTable(strName, m_pointCloudMap[strName].cloud);
		}
		else
		{
			ui.propertyTable->clearContents();
		}
	}
	showRealTimePointCloud(m_pointCloudFile.cloud);

}

void MainWindow::pointcolorChangedType()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("Select color for point cloud"));

	if (color.isValid()) //判断所选的颜色是否有效
	{
		QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
		int selected_item_count = ui.typeTree->selectedItems().size();
		if (selected_item_count != 0)
		{
			for (int i = 0; i != selected_item_count; i++)
			{
				QTreeWidgetItem* curItem = itemList[i];
				QString name = curItem->text(0);
				for (int j = 0; j != m_pointCloudMap[name].cloud->size(); j++)
				{
					m_pointCloudMap[name].cloud->points[j].r = color.red();
					m_pointCloudMap[name].cloud->points[j].g = color.green();
					m_pointCloudMap[name].cloud->points[j].b = color.blue();
				}
				m_mainViewer->updatePointCloud(m_pointCloudMap[name].cloud, name.toStdString());
			}
		}

		ui.screen->update();
	}
}


//法线估计、曲面重建、网格面片显示
void MainWindow::convertSurface()
{
	gridModelConvert();
	//设置网格模型显示模式
	m_mainViewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示
	ui.statusBar->showMessage(tr("Surface reconstruction finished"));
	// 输出窗口
	consoleLog(tr("Convert surface"), tr("surface"), "", "");

}

void MainWindow::convertPoint()
{
	gridModelConvert();
	//设置网格模型显示模式
	m_mainViewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示
	ui.statusBar->showMessage(tr("Point reconstruction finished"));
	// 输出窗口
	consoleLog(tr("Convert point"), tr("point"), "", "");
}

void MainWindow::convertWireframe()
{
	gridModelConvert();
	//设置网格模型显示模式
	m_mainViewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示
	ui.statusBar->showMessage(tr("Wireframe reconstruction finished"));
	// 输出窗口
	consoleLog(tr("Convert wireframe"), tr("wireframe"), "", "");

}

void MainWindow::gridModelConvert()
{
	pcl::PointXYZ point;
	cloud_xyz.reset(new PointCloudT);
	pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud_xyz);

	if (!cloud_xyz)
	{
		return;
	}

	/****** 法向估计模块 ******/
	//创建法线估计对象 n
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//创建法向数据指针 normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//创建 kdtree 用于法向计算时近邻搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_xyz); //为 kdtree 输入点云
	n.setInputCloud(cloud_xyz); //为法向估计对象输入点云
	n.setSearchMethod(tree);  //设置法向估计时所采取的搜索方式为kdtree
	n.setKSearch(20); //设置法向估计时，k近邻搜索的点数
	n.compute(*normals); //进行法向估计

	ui.statusBar->showMessage(tr("Normal estimation finished"));

	/****** 点云数据与法向数据拼接 ******/
	//创建同时包含点和法线的数据结构的指针
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	//将已获得的点数据和法向数据拼接
	pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals); //这里编译出错，与cloud的类型有关？改成PointXYZ的点云就没有报错了

	//创建另一个kdtree用于重建
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//为kdtree输入点云数据，该点云数据类型为点和法向
	tree2->setInputCloud(cloud_with_normals);

	/****** 曲面重建模块 ******/
	//创建贪婪三角形投影重建对象
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//创建多边形网格对象，用来存储重建结果
	pcl::PolygonMesh triangles;
	//设置参数
	gp3.setSearchRadius(25); //设置连接点之间最大距离，用于确定k近邻的球半径
	gp3.setMu(2.5); //设置最近邻距离的乘子，以得到每个点的最终搜索半径
	gp3.setMaximumNearestNeighbors(100); //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 2); //45度 最大平面角
	gp3.setMinimumAngle(M_PI / 18); //10度 每个三角的最大角度？
	gp3.setMaximumAngle(2 * M_PI / 3); //120度
	gp3.setNormalConsistency(false); //若法向量一致，设为true
	//设置点云数据和搜索方式
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	// 开始重建
	gp3.reconstruct(triangles);

	//将重建结果保存到硬盘文件中，重建结果以VTK格式存储
	pcl::io::saveVTKFile("mymesh.vtk", triangles);

	/****** 图形显示模块 ******/
	m_mainViewer->addPolygonMesh(triangles, "convert"); //设置要显示的网格对象
}

void MainWindow::angle2Rad(const double& dAng, double& dRad)
{
	dRad = dAng * M_PI / 180;
}

void MainWindow::rad2Angle(const double& dRad, double& dAng)
{
	dAng = dRad * 180 / M_PI;
}

NavStaFix MainWindow::getNavStaFixByPointxyz(const PointTI& pt, const NavStaFix& inputNav, const double &angle)
{
	NavStaFix outNav;
	const double dRad = atan2(pt.x, pt.z);
	const double dAngle = angle * M_PI / 180 + dRad;

	double ph1 = 0.0, ph2 = 0.0;
	double lamda1 = 0.0, lamda2 = 0.0;
	angle2Rad(inputNav.dLatitude, ph1);
	angle2Rad(inputNav.dLongitude, lamda1);

	double dDis = sqrt(pow((pt.x), 2.0) /*+ pow((pt.y), 2.0)*/ + pow((pt.z), 2.0));
	ph2 = asin(sin(ph1)*cos(dDis / R) + cos(ph1)*sin(dDis / R)*cos(dAngle));
	lamda2 = lamda1 + atan2(sin(dAngle)*sin(dDis / R)*cos(ph1), cos(dDis / R) - sin(ph1)*sin(ph2));

	outNav.dLatitude = ph2 * 180 / M_PI;
	outNav.dLongitude = lamda2 * 180 / M_PI;
	outNav.dAltitude = inputNav.dAltitude;

	return outNav;
}