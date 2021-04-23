#define _CRT_SECURE_NO_WARNINGS
#pragma once

#ifndef __VRMeds_h__
#define __VRMeds_h__

#include <SDKDDKVer.h>

#define WIN32_LEAN_AND_MEAN

#include <windows.h>

#ifndef VRMEDS_EXPORT
#define VRMEDS_EXPORT __declspec(dllexport)
#endif

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <WS2tcpip.h>
#include <corecrt_math_defines.h>
#include <corecrt.h>
#include <corecrt_io.h>
#include <sys/timeb.h>
#include <tchar.h>
#include <atlstr.h>
#include <io.h>
#include <direct.h>
#include <cstdlib>

#include "k4a/k4a.h"

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

typedef enum
{
    SPINEBASE_MAIN = 0,
    SPINEMID_MAIN,
    NECK_MAIN,
    HEAD_MAIN,
    SHOULDERLEFT_MAIN,
    ELBOWLEFT_MAIN,
    WRISTLEFT_MAIN,
    HANDLEFT_MAIN,
    SHOULDERRIGHT_MAIN,
    ELBOWRIGHT_MAIN,
    WRISTRIGHT_MAIN,
    HANDRIGHT_MAIN,
    HIPLEFT_MAIN,
    KNEELEFT_MAIN,
    ANKLELEFT_MAIN,
    FOOTLEFT_MAIN,
    HIPRIGHT_MAIN,
    KNEERIGHT_MAIN,
    ANKLERIGHT_MAIN,
    FOOTRIGHT_MAIN,
    SPINESHOULDER_MAIN,
    HANDTIPLEFT_MAIN,
    THUMBLEFT_MAIN,
    HANDTIPRIGHT_MAIN,
    THUMBRIGHT_MAIN,
    JOINT_COUNT_MAIN,
} joint_type_main;


typedef enum
{
    NOSE_MEDIAPIPE = 0,
    LEFTEYE_IN_MEDIAPIPE,
    LEFTEYE_MEDIAPIPE,
    LEFTEYE_OUT_MEDIAPIPE,
    RIGHTEYE_IN_MEDIAPIPE,
    RIGHTEYE_MEDIAPIPE,
    RIGHTEYE_OUT_MEDIAPIPE,
    LEFTEAR_MEDIAPIPE,
    RIGHTEAR_MEDIAPIPE,
    MOUTH_LEFT_MEDIAPIPE,
    MOUTH_RIGHT_MEDIAPIPE,
    SHOULDERLEFT_MEDIAPIPE,
    SHOULDERRIGHT_MEDIAPIPE,
    ELBOWLEFT_MEDIAPIPE,
    ELBOWRIGHT_MEDIAPIPE,
    WRISTLEFT_MEDIAPIPE,
    WRISTRIGHT_MEDIAPIPE,
    HIPLEFT_MEDIAPIPE,
    HIPRIGHT_MEDIAPIPE,
    KNEELEFT_MEDIAPIPE,
    KNEERIGHT_MEDIAPIPE,
    ANKLELEFT_MEDIAPIPE,
    ANKLERIGHT_MEDIAPIPE,
    HEELLEFT_MEDIAPIPE,
    HEELRIGHT_MEDIAPIPE,
    FOOTLEFT_MEDIAPIPE,
    FOOTRIGHT_MEDIAPIPE,
    THUMBLEFT_MEDIAPIPE,
    HANDLEFT_MEDIAPIPE,
    HANDTIPLEFT_MEDIAPIPE,
    THUMBRIGHT_MEDIAPIPE,
    HANDRIGHT_MEDIAPIPE,
    HANDTIPRIGHT_MEDIAPIPE,
    JOINT_COUNT_MEDIAPIPE,
} joint_type_mediapipe;


typedef struct _joints{
    k4a_float3_t jointsPos[JOINT_COUNT_MAIN];
    unsigned long long timeStamp;  /**< TimeStamp of this Skeleton. */
    bool valid = false;  /**< Whether this skeleton is valid. */
    int reuseCount = 0;  /**< How Many times this skeleton has been reused. */
} skeleton_main;


std::string getCurrentTimeString();

UINT64 getCurrentTimeMillis();

int SendJoints(skeleton_main skeleton);
#endif




