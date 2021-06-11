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

//��ʼ��
void MainWindow::initial()
{
	//�����ʼ��
	setWindowIcon(QIcon(":/Resources/images/icon.png"));
	setWindowTitle(tr("Point Cloud Tools"));  //���´��ڱ���

	// ����Ĭ������
	QFile qss(":/Resources/qss/Darcula.qss");
	qss.open(QFile::ReadOnly);
	qApp->setStyleSheet(qss.readAll());
	qss.close();

	m_pointCloudFileVec.clear();
	m_pointCloudMap.clear();
	//���Ƴ�ʼ��
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	m_pointCloudFile.cloud->resize(1);
	m_pointCloudFileTI.reset(new PointCloudTI);

	m_mainViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	m_mainViewer->initCameraParameters();   //��ʼ���������
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
	ui.dataTree->setSelectionMode(QAbstractItemView::ExtendedSelection); // ���� dataTree ���ж�ѡ
	setConsoleTable();
	setTypeTree();

	// �������
	consoleLog(tr("Software start"), tr("Point cloud tools"), tr("Welcome to use point cloud tools"), tr("Tools"));

	// ���ñ�����ɫΪ dark
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
	//�������
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

		//����״̬��
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
			//	//��lego_loam����ϵתΪ��������ϵ
			//	m_pointCloudFileTI->points[i].x = pointCloudFileTITemp->points[i].z;
			//	m_pointCloudFileTI->points[i].y = pointCloudFileTITemp->points[i].x;
			//	m_pointCloudFileTI->points[i].z = pointCloudFileTITemp->points[i].y;
			//	m_pointCloudFileTI->points[i].intensity = pointCloudFileTITemp->points[i].intensity;
			//}
			//int iCount = intensityMap.size();
			//��Z����תGPRMC�����ĺ���ǵĻ���ֵ
			//float theta = 153 * M_PI / 180; // ���Ƚ� 63����������γ������ķ�λ��
			//Eigen::Affine3f transform = Eigen::Affine3f::Identity();

			//// ��ǰ��һ������ת; Z ������ת theta ����
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
		//��ʾ���޷���ȡ����.ply .pcd .obj������ļ�
		emit updateSignal(tr("File format error"));
		return;
		}
		//��ʾ����׺û���⣬���ļ������޷���ȡ
		if (status != 0)
		{
			emit updateSignal(tr("Reading file error"));
			return;
		}

		//�ӵ������Ƴ�NAN��Ҳ������Ч��
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

		////��ȡԭ���ڸ�˹����ϵ�µ�ֵ
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
		//ȡ�õ������꼫ֵ
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

		//	//��һ��ͳһ������ԭ��
		//	m_pointCloudFileTI->points[i].x = static_cast<float>(pta.pt.x + fX);			
		//	m_pointCloudFileTI->points[i].y = static_cast<float>(pta.pt.z + fY);
		//	m_pointCloudFileTI->points[i].z = static_cast<float>(pta.pt.y + inputNav.dAltitude);
		//	m_pointAttributeVec.push_back(pta);
		//}

		//��������תΪWGS84
		//ԭ�㾭γ�ȡ����θ߶ȡ���λ��
		//for (size_t i = 0; i < m_pointCloudFileTI->size(); ++i)
		//{
		//	NavStaFix outNav = getNavStaFixByPointxyz(m_pointCloudFileTI->points[i], inputNav, 0);
		//	outfile << outNav.dLongitude << "," << outNav.dLatitude << "," << outNav.dAltitude << "\n";
		//}

		outfile.close();
		pcl::copyPointCloud(*m_pointCloudFileTI, *m_pointCloudFile.cloud);

		//ȡ�õ������꼫ֵ
		pcl::getMinMax3D(*(m_pointCloudFile.cloud), m_pointCloudFile.m_minPt, m_pointCloudFile.m_maxPt);
		setAlpha(200);  //���õ���Ϊ��͸��255
		m_pointCloudFile.m_strPointFullFileName = file_name;
		m_pointCloudFile.m_strSubName = subname;
		m_pointCloudFile.m_strDirPath = file_name.substr(0, file_name.size() - subname.size());
		m_pointCloudFileVec.push_back(m_pointCloudFile);  //�����Ƶ����������

		//������Դ������
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(subname.c_str()));
		cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		ui.dataTree->addTopLevelItem(cloudName);
		m_ulTotalPoints += m_pointCloudFile.cloud->points.size();
	}
}

void MainWindow::BLH2xyz( double latitude, double longitude, int ZoneWide, double *Y, double *X /*double dLat, double dLon, double a, double e1, double e2, int k, double &x, double &y*/)
{
	////��ʽ��ȷ��0.001��
	//double a0, a2, a4, a6, a8;
	//double m0, m2, m4, m6, m8;
	//double yita2, t, N;
	//double X;//�����߻���
	//double l, L0;//�����Լ�����������
	//double n;// ������
	//double daihao;//����

	//if (k == 1)//6�ȴ�
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
	//else //3�ȴ�
	//{
	//	if ((int)(dLon - 1.5) % 3 == 0)
	//	{
	//		daihao = (int)((dLon - 1.5) / 3);//int ǿ������ת��������ӽ�0���������֣�-0.1 ���� 0��0.1 ���� 0
	//		L0 = 3 * daihao;
	//	}
	//	else
	//	{
	//		daihao = (int)floor((dLon - 1.5) / 3) + 1;//math.floor ����ԭ����С��ԭ��ֵ����������-0.1 ���� -1�� 0.1 ���� 0
	//		L0 = 3 * daihao;
	//	}
	//}

	//l = dmsToDeg(dLon) - dmsToDeg(L0);
	//dLat = dmsToDeg(dLat);

	////���������߻���
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
	////�����߻������㹫ʽ2
	//X = a0 * dLat - a2 * sin(2 * dLat) / 2 + a4 * sin(4 * dLat) / 4 - a6 * sin(6 * dLat) / 6 + a8 * sin(8 * dLat) / 8;
	//yita2 = e2 * cos(dLat) * cos(dLat);//��ƽ��
	//t = tan(dLat);
	//N = a / sqrt(1 - e1 * sin(dLat) * sin(dLat));
	////�����˹ƽ������
	//n = cos(dLat) * l;
	//x = X + N * t * (n * n / 2 + pow(n, 4) * (5 - t * t + 9 * yita2 + 4 * yita2 * yita2) / 24 + pow(n, 6) * (61 - 58 * t * t + pow(t, 4)) / 720);
	//y = N * (n + pow(n, 3) * (1 - t * t + yita2) / 6 + pow(n, 5) * (5 - 18 * t * t + pow(t, 4) + 14 * yita2 - 58 * yita2 * t * t) / 120);
	//y = y + 500000 + daihao * 1000000;//�����˹ͨ������

	int ProjNo = 0;
	double longitude1, latitude1, longitude0, latitude0, X0, Y0, xval, yval;
	double a, f, e2, ee, NN, T, C, A, M, iPI;
	iPI = 0.0174532925199433; ////3.1415926535898/180.0; 
	a = 6378137; //WGS1984����ϵ���� 
	f = 1.0 / 298.2572235635;
	ProjNo = (int)(longitude / ZoneWide);
	longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
	longitude0 = longitude0 * iPI;
	latitude0 = 0;
	longitude1 = longitude * iPI; //����ת��Ϊ����
	latitude1 = latitude * iPI; //γ��ת��Ϊ����
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
	m_pointCloudFileVec.clear();  //�ӵ����������Ƴ����е���
	m_pointCloudMap.clear();
	m_mainViewer->removeAllPointClouds();  //��viewer���Ƴ����е���
	m_mainViewer->removeAllShapes(); //���remove������

	//if (1 == GetKeyState(0x58))
	{
		::keybd_event(0x58, 0, 0, 0);
		Sleep(20);
		::keybd_event(0x58, 0, KEYEVENTF_KEYUP, 0);
	}

	ui.dataTree->clear();  //��dataTree���
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
	//�������
	consoleLog(tr("Clear"), tr("All point clouds"), "", "");

	showPointcloud();  //������ʾ
}

// Save point cloud
void MainWindow::save()
{
	m_strSaveFilename = QFileDialog::getSaveFileName(this, tr("Save point cloud"),
		QString::fromLocal8Bit(m_pointCloudFile.m_strDirPath.c_str()), tr("Point cloud data(*.pcd *.ply);;Allfile(*.*)"));
	//�ļ���Ϊ��ֱ�ӷ���
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
		status = pcl::io::savePCDFile(file_name, *m_pointCloudFileTI/*(m_pointCloudFile.cloud)*/);//������Ϊ1��ASCII��Ϊ0
		status = pcl::io::savePCDFile(file_name, *(m_pointCloudFile.cloud));
	}
	else if (m_strSaveFilename.endsWith(".ply", Qt::CaseInsensitive))
	{
		status = pcl::io::savePLYFile(file_name, *(m_pointCloudFile.cloud));
	}
	else //��ʾ���޷�����Ϊ����.ply .pcd������ļ�
	{
		QMessageBox::information(this, tr("File format error"), tr("Can't save files except .ply .pcd"));
		return;
	}
	//��ʾ����׺û���⣬�����޷�����
	if (status != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"), tr("Can not save the file"));
		return;
	}

	//�������
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
	//�ļ���Ϊ��ֱ�ӷ���
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
	else //��ʾ���޷�����Ϊ����.ply .pcd������ļ�
	{
		QMessageBox::information(this, tr("File format error"), tr("Can't save files except .ply .pcd"));
		return;
	}
	//��ʾ����׺û���⣬�����޷�����
	if (status != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"), tr("Can not save the file"));
		return;
	}

	//�������
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
		for (int j = 0; j != m_pointCloudFileVec[i].cloud->points.size(); j++) //ע��cloudvec[i]->points.size()��cloudvec[i]->size()������
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
	//����multi_cloud
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
	else //��ʾ���޷�����Ϊ����.ply .pcd������ļ�
	{
		QMessageBox::information(this, tr("File format error"), tr("Can't save files except .ply .pcd"));
		return;
	}

	//��ʾ����׺û���⣬�����޷�����
	if (status != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"), tr("Can not save the file"));
		return;
	}

	// �������
	if (m_bSaveAsBinary)
	{
		consoleLog(tr("Save as binary"), QString::fromLocal8Bit(subname.c_str()), m_strSaveFilename, tr("Multi save (binary)"));
	}
	else
	{
		consoleLog(tr("Save"), QString::fromLocal8Bit(subname.c_str()), m_strSaveFilename, tr("Multi save"));
	}

	m_bSaveAsBinary = false;
	//�������� multi_cloud ����Ϊ��ǰ m_pointCloudFile,�Ա㱣��֮��ֱ�ӽ��в���
	m_pointCloudFile.cloud = multi_cloud;
	multi_cloud.reset(new PointCloudTRGBA);
	m_pointCloudFile.m_strPointFullFileName = m_strSaveFilename.toStdString();
	m_pointCloudFile.m_strSubName = subname;

	setWindowTitle(m_strSaveFilename);
	ui.statusBar->showMessage(tr("Save ") + subname.c_str() + tr(" successfully!"));

}

//�˳�����
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
	//ui.dataTree->clear();  //�����Դ��������item
	m_mainViewer->removeAllPointClouds();  //��viewer���Ƴ����е���
	//m_pointCloudFileVec.clear();  //��յ�������

	m_pointCloudFile.cloud->width = 50000;         // ���õ��ƿ�
	m_pointCloudFile.cloud->height = 1;            // ���õ��Ƹߣ���Ϊ1��˵��Ϊ����֯����
	m_pointCloudFile.cloud->is_dense = false;
	m_pointCloudFile.cloud->resize(m_pointCloudFile.cloud->width * m_pointCloudFile.cloud->height);     // ���õ��ƴ�С
	for (size_t i = 0; i < m_pointCloudFile.cloud->size(); ++i)
	{
		m_pointCloudFile.cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		m_pointCloudFile.cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		m_pointCloudFile.cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		m_pointCloudFile.cloud->points[i].r = m_uRed;
		m_pointCloudFile.cloud->points[i].g = m_uGreen;
		m_pointCloudFile.cloud->points[i].b = m_uBlue;
	}
	//������Դ������
	QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("cube"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);

	// �������
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

//��ʾ���ƣ�����������Ƕ�
void MainWindow::showPointcloud()
{
	for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	{
		m_mainViewer->updatePointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
	}
	//m_mainViewer->resetCamera();
	ui.screen->update();
}

//��ӵ��Ƶ�viewer,����ʾ����
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

//����ͣ�����ڵ���ʾ������
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

//���ƻ���ͼ��
void MainWindow::createSphere()
{
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	//ui.dataTree->clear();  //�����Դ��������item
	m_mainViewer->removeAllShapes();
	//m_pointCloudFileVec.clear();  //��յ�������

	pcl::PointXYZ p;
	p.x = 0; p.y = 0; p.z = 0;
	m_mainViewer->addSphere(p, 100, "sphere");

	m_mainViewer->resetCamera();
	ui.screen->update();

	// �������
	consoleLog(tr("Create sphere"), tr("Sphere"), "", tr("Succeeded"));
}

void MainWindow::createCylinder()
{
	m_pointCloudFile.cloud.reset(new PointCloudTRGBA);
	//ui.dataTree->clear();  //�����Դ��������item
	m_mainViewer->removeAllShapes();
	//m_pointCloudFileVec.clear();  //��յ�������

	m_mainViewer->addCylinder(*(new pcl::ModelCoefficients()), "cylinder");

	m_mainViewer->resetCamera();
	ui.screen->update();

	// �������
	consoleLog(tr("Create cylinder"), tr("Cylinder"), "", tr("Failed"));

}

//��ʽת��
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

	/////*����һ��ֱͨ�˲����Ե��ƽ��д���*/
	//PointCloudT::Ptr cloud_after_PassThrough(new PointCloudT);
	////ȡ�õ������꼫ֵ
	//pcl::PointXYZ minPt, maxPt;
	//pcl::getMinMax3D(*cloud, minPt, maxPt);
	//pcl::PassThrough<pcl::PointXYZ> passthrough;
	//passthrough.setInputCloud(cloud);//�������
	//passthrough.setFilterFieldName("y");//��z����в���
	//passthrough.setFilterLimits(-1.0, 2/*maxPt.z - 10.0*/);//����ֱͨ�˲���������Χ
	//passthrough.setFilterLimitsNegative(true);//true��ʾ������Χ�ڣ�false��ʾ������Χ��
	//passthrough.filter(*cloud_after_filter);//ִ���˲������˽�������� cloud_after_PassThrough

	//emit updateSignal(QString("%1%2").arg(tr("Straight after filtering point cloud data points:")).arg(cloud_after_filter->points.size()));
	//return;
	///////****************************************************////////////////////

	///////****************************************************////////////////////
	/*�������������˲���ʵ���²���*/
	PointCloudT::Ptr cloud_after_voxelgrid(new PointCloudT);
	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setInputCloud(cloud);//�����������
	voxelgrid.setLeafSize(0.01f, 0.01f, 0.01f);//AABB�����
	voxelgrid.filter(*cloud_after_voxelgrid);

	emit updateSignal(QString("%1%2").arg(tr("Voxel grid method after the point cloud data points:")).arg(cloud_after_voxelgrid->points.size()));

	/////////****************************************************////////////////////

	/////////****************************************************////////////////////
	///*��������ͳ���˲����˲�*/
	PointCloudT::Ptr cloud_after_StatisticalRemoval(new PointCloudT);

	// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
	// ����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;//�����˲�������
	Statistical.setInputCloud(cloud_after_voxelgrid);//���ô��˲��ĵ���
	Statistical.setMeanK(50);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	Statistical.setStddevMulThresh(1.0); //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
	Statistical.filter(*cloud_after_StatisticalRemoval);

	//outlier
	Statistical.setNegative(true);
	Statistical.filter(*cloud_after_filter);

	emit updateSignal(QString("%1%2").arg(tr("After statistical analysis filtering point cloud data points:")).arg(cloud_after_StatisticalRemoval->points.size()));

	///////****************************************************////////////////////

	/////////****************************************************////////////////////
	///*�����ģ������˲���*/
	//PointCloudT::Ptr cloud_after_Condition(new PointCloudT);

	//pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
	//range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
	//	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));  //GT��ʾ���ڵ���
	//range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
	//	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));  //LT��ʾС�ڵ���

	//pcl::ConditionalRemoval<pcl::PointXYZ> condition;
	//condition.setCondition(range_condition);
	//condition.setInputCloud(cloud);                   //�������
	//condition.setKeepOrganized(true);

	//condition.filter(*cloud_after_Condition);

	//emit updateSignal(QString("%1%2").arg(tr("After filtering under the condition point cloud data points:")).arg(cloud_after_Condition->points.size()));

	////showRealTimePointCloudAfterFilter(cloud_after_Condition);

	/////////****************************************************////////////////////

	/////////****************************************************////////////////////
	///*�����壺�뾶�˲���*/
	//PointCloudT::Ptr cloud_after_Radius(new PointCloudT);

	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //�����˲���

	//radiusoutlier.setInputCloud(cloud);    //�����������
	//radiusoutlier.setRadiusSearch(100);     //���ð뾶Ϊ100�ķ�Χ�����ٽ���
	//radiusoutlier.setMinNeighborsInRadius(2); //���ò�ѯ�������㼯��С��2��ɾ��

	//radiusoutlier.filter(*cloud_after_Radius);

	//emit updateSignal(QString("%1%2").arg(tr("Radius after filtering point cloud data points:")).arg(cloud_after_Radius->points.size()));

	///////****************************************************////////////////////
	/*���������ָ���*/
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);//���÷ָ�ģ��
	seg.setMethodType(pcl::SAC_RANSAC);//���ò���һ���Թ��Ʒ���
	// ������ֵ ��λm
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(1);//���þ�����ֵ
	seg.setInputCloud(cloud_after_StatisticalRemoval);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return;
	}

	// ��ȡ����
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_after_StatisticalRemoval);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_after_filter);

	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("E:\\VTK\\B1_ground2222.pcd", *cloud_after_filter, false);
	emit updateSignal(QString("%1%2").arg(tr("After the split point cloud data points:")).arg(cloud_after_filter->points.size()));
	//// ��ȡ�������������
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

//��׼��ŷ����·ָ�
void MainWindow::euclideanSegmentation(const PointCloudTRGBA::Ptr &cloud)
{
	PointCloud point;
	point.cloud.reset(new PointCloudTRGBA);
	// ����kd-tree������������ .
	pcl::search::KdTree<PointTRGBA>::Ptr kdtree(new pcl::search::KdTree<PointTRGBA>);
	kdtree->setInputCloud(cloud);

	// Euclidean �������.
	pcl::EuclideanClusterExtraction<PointTRGBA> clustering;
	// ���þ������Сֵ 40cm (small values may cause objects to be divided
	// in several clusters, whereas big values may join objects in a same cluster).
	clustering.setClusterTolerance(0.5);
	// ���þ����С��������������
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
		//������еĵ��Ƶ�һ���µĵ�����
		PointCloudTRGBA::Ptr cluster(new PointCloudTRGBA);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ����
		if (cluster->points.size() <= 0)
			break;

		*point.cloud += *cluster;
	}

	m_mainViewer->removeShape("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());
	m_mainViewer->removePointCloud("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());

	// �ڵ㻹��������ӽڵ�
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
	// ��cloud_id����Ӧ�ĵ������ó�͸��
	m_iSegmentationCarIndex++;

	setSelectPointPropertyTable(strCar, cloud);
	ui.screen->update();
}

//����ŷ����·ָ�
void MainWindow::conditionalEuclideanSegmentation(const PointCloudT::Ptr &cloud)
{
	PointCloudT::Ptr add_cloud(new PointCloudT);
	// ����kd-tree������������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	// Euclidean �������.
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	clustering.setClusterTolerance(0.4);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	//����ÿ�μ��һ�Ե���ʱ�ĺ���
	clustering.setConditionFunction(&customCondition);
	std::vector<pcl::PointIndices> clusters;
	clustering.segment(clusters);

	// For every cluster...
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//������еĵ��Ƶ�һ���µĵ�����
		PointCloudT::Ptr cluster(new PointCloudT);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ����
		if (cluster->points.size() <= 0)
			break;

		*add_cloud += *cluster;
	}
	//ɾ����ɫ��ѡ�ĵ���
	m_mainViewer->removeShape("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());
	m_mainViewer->removePointCloud("area_picking" + QString::number(m_iAreaPickingIndex).toStdString());

	static int iSegmentationIndex = 1;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(add_cloud, 0, 0, 255);
	m_mainViewer->removeShape("segmentation" + QString::number(iSegmentationIndex - 1).toStdString());
	m_mainViewer->addPointCloud(add_cloud, blue, "segmentation" + QString::number(iSegmentationIndex).toStdString());
	iSegmentationIndex++;

	ui.screen->update();
}

// �������������ص����棬�������ѡ�㽫�ᱻ���������
bool MainWindow::customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// Do whatever you want here.����������������ɸѡ
	if (candidatePoint.y < seedPoint.y)  //�����ѡ���Y��ֵС�����ӵ��Yֵ������֮ǰ��ѡ��Ϊ����ĵ㣩�����������������ؼ�
		return false;

	return true;
}

void MainWindow::issKeyPoint()
{
	clock_t start = clock();
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud);
	//iss�ؼ�����ȡ
	PointCloudT::Ptr cloud_src_is(new PointCloudT);

	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_det;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());

	//��������
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
	// �������
	consoleLog(tr("Key point extract"), tr("iss"), "", tr("Time cost: ") + strTimeCost + tr(" s"));

	PointCloudT::Ptr cloud_src(new PointCloudT);
	pcl::copyPointCloud(*cloud_src_is, *cloud_src);

	//���ӻ�
	visualize_pcd(cloud, cloud_src);
}

void MainWindow::siftKeyPoint()
{
	clock_t start = clock();
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::copyPointCloud(*m_pointCloudFile.cloud, *cloud);
	//�趨����ֵ
	const float min_scale = 0.002f; //the standard deviation of the smallest scale in the scale space
	const int n_octaves = 3;//�߶ȿռ����,С���ؼ����
	const int n_scales_per_octave = 3;//the number of scales to compute within each octave
	const float min_contrast = 0.0001f;//���ݵ��ƣ����ô�С��ԽС�ؼ���Խ��

	//sift�ؼ�����
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
	//// �������
	//consoleLog(tr("Key point extract"), tr("sift"), "", tr("Time cost: ") + strTimeCost + tr(" s"));

	//PointCloudT::Ptr cloud_src(new PointCloudT);
	//pcl::copyPointCloud(result_src, *cloud_src);


	////���ӻ�
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
	//�ӵ������Ƴ�NAN��Ҳ������Ч��
	std::vector<int> indices;
	indices.clear();
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	//������Դ������
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
	setAlpha(200);  //���õ���Ϊ��͸��255
	m_pointCloudFileVec.push_back(m_pointCloudFile);  //�����Ƶ����������

	setPropertyTable(m_pointCloudFileVec.size() - 1);
	m_iCurPointFileIndex = m_pointCloudFileVec.size() - 1;

	ui.screen->update();

}

void MainWindow::showRealTimePointCloud(const PointCloudTRGBA::Ptr &cloud)
{
	m_mainViewer->removeAllPointClouds();  //��viewer���Ƴ����е���
	m_mainViewer->removeAllShapes(); //���remove������
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
	//����QSS��ʽ��
	QFile qss(":/Resources/qss/Windows.qss");
	qss.open(QFile::ReadOnly);
	qApp->setStyleSheet(qss.readAll());
	qss.close();

	// �������
	consoleLog(tr("Change theme"), tr("Windows theme"), "", "");

	m_iThemeId = 0;
}

void MainWindow::darculaTheme()
{
	QFile qss(":/Resources/qss/Darcula.qss");
	qss.open(QFile::ReadOnly);
	qApp->setStyleSheet(qss.readAll());
	qss.close();

	// �������
	consoleLog(tr("Change theme"), tr("Darcula theme"), "", "");

	m_iThemeId = 1;
}

// Change language: English/Chinese
void MainWindow::langEnglish()
{
	// �������
	consoleLog(tr("Change language"), tr("English"), "", "");
	changeLanguage(UI_EN);
}

void MainWindow::langChinese()
{
	// �������
	consoleLog(tr("Change language"), tr("Chinese"), "", "");
	changeLanguage(UI_ZH);
}

//����
void MainWindow::about()
{
	AboutWin *aboutwin = new AboutWin(this);
	aboutwin->setModal(true);
	aboutwin->show();

	// �������
	consoleLog(tr("About"), tr("Point cloud tools about"), "http://www.baidu.com", tr("Welcome to my blog!"));
}

//����
void MainWindow::help()
{
	QDesktopServices::openUrl(QUrl(QLatin1String("https://github.com/Ewenwan/MVision/tree/master/PCL_APP")));

	// �������
	consoleLog(tr("Help"), tr("Point cloud tools help"), "https://github.com/Ewenwan/MVision/tree/master/PCL_APP", "");

}


/*********************************************/
/*****************����ۺ���*****************/
/********************************************/
void MainWindow::colorBtnPressed()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	// ���δѡ���κε��ƣ������ͼ�����е����е��ƽ�����ɫ
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

		// �������
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

		// �������
		consoleLog(tr("Random color"), tr("Point clouds selected"), "", "");
	}
	showPointcloud();
}

void MainWindow::RGBsliderReleased()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	// ���δѡ���κε��ƣ������ͼ�����е����е��ƽ�����ɫ
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

		// �������
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
		// �������
		consoleLog(tr("Change cloud color"), tr("Point clouds selected"), QString::number(m_uRed) + " " + QString::number(m_uGreen) + " " + QString::number(m_uBlue), "");
	}
	showPointcloud();
}

//�������е��Ƶĳߴ�
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
		// �������
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
		// �������
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
		m_mainViewer->removeCoordinateSystem(0);  //�Ƴ�����ϵ
		// �������
		consoleLog(tr("Remove coordinate system"), tr("Remove"), "", "");
		break;
	case 2:
		m_mainViewer->addCoordinateSystem(10.0, 0);  //�������ϵ
		// �������
		consoleLog(tr("Add coordinate system"), tr("Add"), "", "");
		ui.statusBar->showMessage(tr("Red is the X axis, green is the Y axis, blue is Z axis")); //��ɫ��X�ᣬ��ɫ��Y�ᣬ��ɫ��Z
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
		// �������
		consoleLog(tr("Change bg color"), tr("Background"), "30 30 30", "");
		break;
	case 2:
		// ע�⣺setBackgroundColor()���յ���0-1��double�Ͳ���
		m_mainViewer->setBackgroundColor(255 / 255.0, 255 / 255.0, 255 / 255.0);
		// �������
		consoleLog(tr("Change bg color"), tr("Background"), "255 255 255", "");
		break;
	}

	ui.screen->update();
}

//ͨ����ɫ�Ի���ı������ɫ
void MainWindow::pointcolorChangedData()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("Select color for point cloud"));

	if (color.isValid()) //�ж���ѡ����ɫ�Ƿ���Ч
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
			// �������
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
			// �������
			consoleLog(tr("Change cloud color"), tr("Point clouds selected"), QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
		}
		//��ɫ�ĸı�ͬ����RGBͣ������
		ui.rSlider->setValue(color.red());
		ui.gSlider->setValue(color.green());
		ui.bSlider->setValue(color.blue());

		showPointcloud();
	}
}

//ͨ����ɫ�Ի���ı䱳����ɫ
void MainWindow::bgcolorChanged()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("Select color for point cloud"));
	if (color.isValid())
	{
		m_mainViewer->setBackgroundColor(color.red() / 255.0,
			color.green() / 255.0, color.blue() / 255.0);
		// �������
		consoleLog(tr("Change bg color"), tr("Background"), QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
		showPointcloud();
	}
}

//����ͼ
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

//�������Թ�����
void MainWindow::setPropertyTable(int iIndex)
{
	ui.propertyTable->clearContents();

	ui.propertyTable->setItem(0, 0, new QTableWidgetItem(tr("Clouds")));
	ui.propertyTable->setItem(1, 0, new QTableWidgetItem(tr("Points")));
	ui.propertyTable->setItem(2, 0, new QTableWidgetItem(tr("Total points")));
	ui.propertyTable->setItem(3, 0, new QTableWidgetItem(tr("RGB")));
	ui.propertyTable->setItem(4, 0, new QTableWidgetItem(tr("Min XYZ")));
	ui.propertyTable->setItem(5, 0, new QTableWidgetItem(tr("Max XYZ")));
	//��ȡ��ǰ���Ƶ�RGB,������������Ϣ
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
	if (m_pointCloudFileVec[iIndex].cloud->points.begin()->r == (m_pointCloudFileVec[iIndex].cloud->points.end() - 1)->r) //�жϵ��Ƶ�ɫ��ɫ�����������Ǻ��Ͻ���
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

	//ȡ�õ������꼫ֵ
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
	ui.propertyTable->setSelectionMode(QAbstractItemView::NoSelection); // ��ֹ������Թ������� item
}

void MainWindow::setConsoleTable()
{
	// �����������
	QStringList header2;
	header2 << tr("Time") << tr("Operation") << tr("Operation obeject") << tr("Details") << tr("Note");
	ui.consoleTable->setHorizontalHeaderLabels(header2);
	ui.consoleTable->setColumnWidth(0, 200);
	ui.consoleTable->setColumnWidth(1, 200);
	ui.consoleTable->setColumnWidth(2, 200);
	ui.consoleTable->setColumnWidth(3, 250);

	//ui.consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); //���ò��ɱ༭
	ui.consoleTable->verticalHeader()->setDefaultSectionSize(22); //�����о�

	ui.consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
	ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);  // ��ֹ���������ڵ� item
}

void MainWindow::setTypeTree()
{
	////������Դ������
	ui.typeTree->setColumnCount(1);
	// ���ñ�ͷ
	ui.typeTree->setHeaderLabels(QStringList() << tr("Point Cloud"));
	ui.typeTree->setSelectionMode(QAbstractItemView::ExtendedSelection);
	// ���һ���ڵ�
	m_pCarItem = new QTreeWidgetItem(ui.typeTree, QStringList() << tr("Car"));
	m_pTreeItem = new QTreeWidgetItem(ui.typeTree, QStringList() << tr("Tree"));
	m_pCarItem->setHidden(true);
	m_pTreeItem->setHidden(true);
}

void MainWindow::consoleLog(QString operation, QString subname, QString filename, QString note)
{
	int rows = ui.consoleTable->rowCount();
	ui.consoleTable->setRowCount(++rows);
	QDateTime time = QDateTime::currentDateTime();//��ȡϵͳ���ڵ�ʱ��
	QString strTime = time.toString("yyyy-MM-dd hh:mm:ss"); //������ʾ��ʽ
	ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(strTime));
	ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
	ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(subname));
	ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(filename));
	ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(note));

	ui.consoleTable->scrollToBottom(); // �����Զ�������ײ�
	qDebug() << operation;
}

//QTreeWidget��item�ĵ����Ӧ����
void MainWindow::itemSelectedData(QTreeWidgetItem* item, int count)
{
	count = ui.dataTree->indexOfTopLevelItem(item);  //��ȡitem���к�
	m_iCurPointFileIndex = count;
	//for (int i = 0; i != m_pointCloudFileVec.size(); i++)
	//{
	//	m_mainViewer->updatePointCloud(m_pointCloudFileVec[i].cloud, "cloud" + QString::number(i).toStdString());
	//	m_mainViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud" + QString::number(i).toStdString());
	//}

	////ѡ��item����Ӧ�ĵ��Ƴߴ���
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

//QTreeWidget��item���һ���Ӧ����
void MainWindow::popMenuData(const QPoint&)
{
	QTreeWidgetItem* curItem = ui.dataTree->currentItem(); //��ȡ��ǰ������Ľڵ�
	if (curItem == NULL)return;           //����������Ҽ���λ�ò���treeItem�ķ�Χ�ڣ����ڿհ�λ���һ�
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

	menu.exec(QCursor::pos()); //�ڵ�ǰ���λ����ʾ
}

// consoleTable �һ���Ӧ�¼�
void MainWindow::popMenuInConsole(const QPoint&)
{
	QAction clearConsoleAction(tr("Clear console"), this);

	connect(&clearConsoleAction, &QAction::triggered, this, &MainWindow::clearConsole);

	QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&clearConsoleAction);

	menu.exec(QCursor::pos()); //�ڵ�ǰ���λ����ʾ
}

// ��� consoleTable
void MainWindow::clearConsole()
{
	ui.consoleTable->clearContents();
	ui.consoleTable->setRowCount(0);
}

//QTreeWidget��item�ĵ����Ӧ����
void MainWindow::itemSelectedType(QTreeWidgetItem* item, int count)
{
	count = ui.dataTree->indexOfTopLevelItem(item);  //��ȡitem���к�
	QTreeWidgetItem* curItem = ui.typeTree->currentItem(); //��ȡ��ǰ������Ľڵ�
	if (curItem == NULL)
		return;

	//ѡ��item����Ӧ�ĵ��Ƴߴ���
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

//QTreeWidget��item���һ���Ӧ����
void MainWindow::popMenuType(const QPoint&)
{
	QTreeWidgetItem* curItem = ui.typeTree->currentItem(); //��ȡ��ǰ������Ľڵ�
	if (curItem == NULL)return;           //����������Ҽ���λ�ò���treeItem�ķ�Χ�ڣ����ڿհ�λ���һ�
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

	menu.exec(QCursor::pos()); //�ڵ�ǰ���λ����ʾ
}


void MainWindow::hideItemData()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	for (int i = 0; i != ui.dataTree->selectedItems().size(); i++)
	{
		//TODO hide֮��item��ɻ�ɫ���ٴ��һ�itemʱ����hideItem�� ѡ���� ��showItem��
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		const int id = ui.dataTree->indexOfTopLevelItem(curItem);
		hidePointCloudFile(id);
		QColor item_color = QColor(112, 122, 132, 255);
		curItem->setTextColor(0, item_color);
		m_pointCloudFileVec[id].m_bVisible = false;
	}

	// �������
	consoleLog(tr("Hide point clouds"), tr("Point cloud files selected"), "", "");

	ui.screen->update(); //ˢ����ͼ���ڣ�����ʡ��
}

void MainWindow::hidePointCloudFile(const int &id)
{
	string cloud_id = "cloud" + QString::number(id).toStdString();

	// ��cloud_id����Ӧ�ĵ������ó�͸��
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

	// �������
	consoleLog(tr("Show point clouds"), tr("Point cloud files selected"), "", "");

	ui.screen->update(); //ˢ����ͼ���ڣ�����ʡ��

}

void MainWindow::showPointCloudFile(const int & id)
{
	string cloud_id = "cloud" + QString::number(id).toStdString();
	// ��cloud_id����Ӧ�ĵ������ó�͸��
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
		// ɾ������֮ǰ����������Ŀ����
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

	// �Ƴ�֮������ӣ����� id ����Դ�������кŲ�һ�µ����
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

	// �������
	consoleLog(tr("Delete point clouds"), tr("Point clouds file selected"), "", "");

	ui.screen->update();
}

////type
void MainWindow::hideItemType()
{
	QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
	for (int i = 0; i != ui.typeTree->selectedItems().size(); i++)
	{
		//TODO hide֮��item��ɻ�ɫ���ٴ��һ�itemʱ����hideItem�� ѡ���� ��showItem��
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);

		// ��ԭ�е����ļ����ó�͸��
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
		// ��cloud_id����Ӧ�ĵ������ó�͸��
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

	ui.screen->update(); //ˢ����ͼ���ڣ�����ʡ��

}

void MainWindow::deleteItemType()
{
	QList<QTreeWidgetItem*> itemList = ui.typeTree->selectedItems();
	int selected_item_count = ui.typeTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++)
	{
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);

		//�Ƚ���ѡ����ɫͼ��ɾ��
		m_mainViewer->removeShape(name.toStdString());

		//�ٽ�ԭʼ��ͼ����ѡ��ɾ��
		std::vector<int> indices;
		indices.clear();
		for (std::vector<pcl::PointIndices>::const_iterator i = m_pointCloudMap[name].m_indicesVec.begin(); i != m_pointCloudMap[name].m_indicesVec.end(); ++i)
		{
			//������еĵ��Ƶ�һ���µĵ�����
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

	if (color.isValid()) //�ж���ѡ����ɫ�Ƿ���Ч
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


//���߹��ơ������ؽ���������Ƭ��ʾ
void MainWindow::convertSurface()
{
	gridModelConvert();
	//��������ģ����ʾģʽ
	m_mainViewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ
	ui.statusBar->showMessage(tr("Surface reconstruction finished"));
	// �������
	consoleLog(tr("Convert surface"), tr("surface"), "", "");

}

void MainWindow::convertPoint()
{
	gridModelConvert();
	//��������ģ����ʾģʽ
	m_mainViewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ
	ui.statusBar->showMessage(tr("Point reconstruction finished"));
	// �������
	consoleLog(tr("Convert point"), tr("point"), "", "");
}

void MainWindow::convertWireframe()
{
	gridModelConvert();
	//��������ģ����ʾģʽ
	m_mainViewer->setRepresentationToWireframeForAllActors(); //����ģ�����߿�ͼģʽ��ʾ
	ui.statusBar->showMessage(tr("Wireframe reconstruction finished"));
	// �������
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

	/****** �������ģ�� ******/
	//�������߹��ƶ��� n
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//������������ָ�� normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//���� kdtree ���ڷ������ʱ��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_xyz); //Ϊ kdtree �������
	n.setInputCloud(cloud_xyz); //Ϊ������ƶ����������
	n.setSearchMethod(tree);  //���÷������ʱ����ȡ��������ʽΪkdtree
	n.setKSearch(20); //���÷������ʱ��k���������ĵ���
	n.compute(*normals); //���з������

	ui.statusBar->showMessage(tr("Normal estimation finished"));

	/****** ���������뷨������ƴ�� ******/
	//����ͬʱ������ͷ��ߵ����ݽṹ��ָ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	//���ѻ�õĵ����ݺͷ�������ƴ��
	pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals); //������������cloud�������йأ��ĳ�PointXYZ�ĵ��ƾ�û�б�����

	//������һ��kdtree�����ؽ�
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//Ϊkdtree����������ݣ��õ�����������Ϊ��ͷ���
	tree2->setInputCloud(cloud_with_normals);

	/****** �����ؽ�ģ�� ******/
	//����̰��������ͶӰ�ؽ�����
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//���������������������洢�ؽ����
	pcl::PolygonMesh triangles;
	//���ò���
	gp3.setSearchRadius(25); //�������ӵ�֮�������룬����ȷ��k���ڵ���뾶
	gp3.setMu(2.5); //��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶
	gp3.setMaximumNearestNeighbors(100); //��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 2); //45�� ���ƽ���
	gp3.setMinimumAngle(M_PI / 18); //10�� ÿ�����ǵ����Ƕȣ�
	gp3.setMaximumAngle(2 * M_PI / 3); //120��
	gp3.setNormalConsistency(false); //��������һ�£���Ϊtrue
	//���õ������ݺ�������ʽ
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	// ��ʼ�ؽ�
	gp3.reconstruct(triangles);

	//���ؽ�������浽Ӳ���ļ��У��ؽ������VTK��ʽ�洢
	pcl::io::saveVTKFile("mymesh.vtk", triangles);

	/****** ͼ����ʾģ�� ******/
	m_mainViewer->addPolygonMesh(triangles, "convert"); //����Ҫ��ʾ���������
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