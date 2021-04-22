#define _CRT_SECURE_NO_WARNINGS
#pragma once

#define WIN32_LEAN_AND_MEAN             // 从 Windows 头文件中排除极少使用的内容
// Windows 头文件
#include <windows.h>

// 在此处引用程序需要的其他标头
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <corecrt_math_defines.h>
#include <sys/timeb.h>
#include <tchar.h>
#include <atlstr.h>
#include <io.h>
#include <direct.h>

inline std::string getCurrentDir() {
    TCHAR szFilePath[MAX_PATH + 1] = { 0 };
    GetModuleFileName((HINSTANCE)&__ImageBase, szFilePath, MAX_PATH);
    (_tcsrchr(szFilePath, _T('\\')))[1] = 0;
    CString curdir = szFilePath;
    std::string curdirStr = std::string(CT2A(curdir.GetString()));
    return curdirStr; 
}

inline std::string getNowTimeString(std::string fmtStr) {
    time_t timep;
    time(&timep);
    char tmp[256];
    strftime(tmp, sizeof(tmp), fmtStr.c_str(), localtime(&timep));
    return tmp;
}

void DrawPoint(float x_raw, float y_raw, cv::Mat img);

UINT64 getCurrentTimeMillis()
{
    using namespace std;

    timeb now;
    ftime(&now);
    std::stringstream milliStream;
    milliStream << setw(3) << setfill('0') << right << now.millitm;

    stringstream secStream;
    secStream << now.time;
    string timeStr(secStream.str());
    timeStr.append(milliStream.str());

    UINT64 timeLong;
    stringstream transStream(timeStr);
    transStream >> timeLong;

    return timeLong;
}


