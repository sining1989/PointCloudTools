#include "common.h"


QTime qtime;

bool isDirExist(QString fullPath)
{
	QDir dir(fullPath);
	if (dir.exists())
	{
		return true;
	}
	else
	{
		bool ok = dir.mkdir(fullPath);//ֻ����һ����Ŀ¼�������뱣֤�ϼ�Ŀ¼����
		return ok;
	}
}

char ConvertHexChar(char ch)
{
	if ((ch >= '0') && (ch <= '9'))
		return ch - 0x30;
	else if ((ch >= 'A') && (ch <= 'F'))
		return ch - 'A' + 10;
	else if ((ch >= 'a') && (ch <= 'f'))
		return ch - 'a' + 10;
	//        else return (-1);
	else return ch - ch;//����0-f��Χ�ڵĻᷢ�ͳ�0
}

void StringToHex(QString str, QByteArray & senddata)  //�ַ���ת����16��������0-F
{
	int hexdata, lowhexdata;
	int hexdatalen = 0;
	int len = str.length();
	senddata.resize(len / 2);
	char lstr, hstr;
	for (int i = 0; i < len; )
	{
		//char lstr,
		hstr = str[i].toLatin1();
		if (hstr == ' ')
		{
			i++;
			continue;
		}
		i++;
		if (i >= len)
			break;
		lstr = str[i].toLatin1();
		hexdata = ConvertHexChar(hstr);
		lowhexdata = ConvertHexChar(lstr);
		if ((hexdata == 16) || (lowhexdata == 16))
			break;
		else
			hexdata = hexdata * 16 + lowhexdata;
		i++;
		senddata[hexdatalen] = (char)hexdata;
		hexdatalen++;
	}
	senddata.resize(hexdatalen);
}

//��ȡȫ·���е��ļ���������׺��
std::string getFileName(std::string file_name)
{
	std::string subname;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subname.insert(subname.begin(), *i);
	}
	return subname;
}

//��ʱ��ʼ
void timeStart()
{
	qtime.start();
}

//��ʱ����
QString timeOff()
{
	int timediff = qtime.elapsed();   //���ش��ϴ�start()��restart()��ʼ������ʱ����λms
	float f = timediff / 1000.0;
	QString tr_timediff = QString("%1").arg(f);  //float->QString
	return tr_timediff;
}

// QString(Unicode) -> std::string (GBK)
static string FromUnicode(const QString& qstr)
{
	QTextCodec* pCodec = QTextCodec::codecForName("gb2312");
	if (!pCodec) return "";

	QByteArray arr = pCodec->fromUnicode(qstr);
	string cstr = arr.data();
	return cstr;
}

// std::string (GBK) -> QString(Unicode)
static QString ToUnicode(const string& cstr)
{
	QTextCodec* pCodec = QTextCodec::codecForName("gb2312");
	if (!pCodec) return "";

	QString qstr = pCodec->toUnicode(cstr.c_str(), cstr.length());
	return qstr;
}

//byteתint  Azimuth
int bytes2ToInt(byte* bytes)
{
	int addr = bytes[0] & 0xFF;
	addr |= (bytes[1] << 8 & 0xFF00);
	return addr;
}

int bytes1ToInt(byte* bytes)
{
	int addr = bytes[0] & 0xFF;
	return addr;
}

//byteתint  Azimuth
long bytes4ToInt(byte* bytes)
{
	long addr = bytes[0] & 0xFF;
	addr |= (bytes[1] << 8 & 0xFF00);
	addr |= ((bytes[2] << 16) & 0xFF0000);
	addr |= ((bytes[3] << 24) & 0xFF000000);
	return addr;
}

double dmsToDeg(double dms)
{
	double d, m, s;
	int i = 1;
	if (dms < 0)
	{
		i = -1;
		dms = abs(dms);
	}
	d = floor(dms);
	m = floor((dms - d) * 100);
	s = ((dms - d) * 100 - m) * 100;
	return i * (d + m / 60 + s / 3600) * PI / 180;
}
