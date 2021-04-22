#define _CRT_SECURE_NO_WARNINGS
// stdafx.h: 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 项目特定的包含文件
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // 从 Windows 头文件中排除极少使用的内容
// Windows 头文件
#include <windows.h>

#ifndef VRMEDS_EXPORT
#define VRMEDS_EXPORT __declspec(dllexport)
#endif

// 在此处引用程序需要的其他标头
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <WS2tcpip.h>
#include <corecrt_math_defines.h>
#include <sys/timeb.h>
#include <tchar.h>
#include <atlstr.h>
#include <io.h>
#include <direct.h>

#include "k4a/k4a.hpp"
#include "k4abt.hpp"

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
