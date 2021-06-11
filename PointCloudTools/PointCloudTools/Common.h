#ifndef COMMON_H
#define COMMON_H

#include <QString>
#include <QDir>
#include <string>
#include <iostream>  
#include <QTime>
#include <windows.h> 
#include <math.h>
#include <QTextCodec>

using std::string;
using namespace std;

#pragma once
#pragma warning ( disable : 4099 )


#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0) 
#define M_PI       3.14159265358979323846 
#define PI acos(-1)
#define R 6371393 //m

typedef enum{
	UI_EN,
	UI_ZH
}LANGUAGE;


//枚举按钮的几种状态
typedef enum{
	NORMALSTU,
	ENTER,
	PRESS,
	NOSTATUS
}ButtonStatus;


extern bool isDirExist(QString fullPath);
extern void StringToHex(QString str, QByteArray & senddata);
extern char ConvertHexChar(char ch);
extern std::string getFileName(std::string file_name);
extern void timeStart();
extern QString timeOff();
extern double dmsToDeg(double dms);
#endif // COMMON_H
