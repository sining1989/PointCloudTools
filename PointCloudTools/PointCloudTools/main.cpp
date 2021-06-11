#include <QApplication>
#include <QTextCodec>
#include <QTranslator>
#include <QMutex>
#include <QFile>
#include <QDir>
#include <QSharedMemory>

#include "MainWindow.h"

#pragma once
#pragma warning ( disable : 4099 )

void outputMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	static QMutex mutex;
	mutex.lock();
	QString text;
	switch (type)
	{
	case QtDebugMsg:
		text = QString("Debug:");
		break;

	case QtWarningMsg:
		text = QString("Warning:");
		break;

	case QtCriticalMsg:
		text = QString("Critical:");
		break;

	case QtFatalMsg:
		text = QString("Fatal:");
	}
	QString context_info = QString("Func:%1 Line:%2").arg(context.function).arg(context.line);
	QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss ddd");
	QString current_date = QString("(%1)").arg(current_date_time);
	QString message = QString("%1 %2 %3 %4").arg(current_date).arg(text).arg(context_info).arg(msg);
	QString rootPath = QCoreApplication::applicationDirPath();
	QFile file(rootPath + "/log/log.txt");
	if (!file.exists())
	{
		QDir path(rootPath + "/log");
		if (!path.exists())
		{
			path.mkdir(rootPath + "/log");
		}
	}
	file.open(QIODevice::WriteOnly | QIODevice::Append);
	QTextStream text_stream(&file);
	text_stream << message << "\r\n";
	file.flush();
	file.close();
	mutex.unlock();
}

int main(int argc, char *argv[])
{
	Q_UNUSED(argc);
	Q_UNUSED(argv);
	::CreateMutex(NULL, NULL, L"Point Cloud Tools");

	if (::GetLastError() == ERROR_ALREADY_EXISTS)
		return 0;

	QApplication app(argc, argv);
#if QT_VERSION < QT_VERSION_CHECK(5,0,0)  
#if defined(_MSC_VER) && (_MSC_VER < 1600)  
	QTextCodec::setCodecForTr(QTextCodec::codecForName("GB18030-0"));
#else  
	QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
#endif  
#endif  

#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")
#endif
	qInstallMessageHandler(outputMessage);
	QTextCodec *codec = QTextCodec::codecForName("UTF-8");
	QTextCodec::setCodecForLocale(codec);

	QTranslator translator;
	bool bFlag = translator.load(QString(":/Resources/qm/lang_zh_CN"));
	app.installTranslator(&translator);
	   
	QSharedMemory shared_memory;
	shared_memory.setKey(QString("PointCloudTools"));

	if (shared_memory.attach())
	{
		qWarning("This point cloud tools existing!");
		return 0;
	}

	if (shared_memory.create(1))
	{
		MainWindow main_window;
		main_window.show();

		return app.exec();
	}

}





