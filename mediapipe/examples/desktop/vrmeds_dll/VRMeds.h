#define _CRT_SECURE_NO_WARNINGS
#pragma once

#ifndef __VRMeds_h__
#define __VRMeds_h__

// 序列化并发送身体关节点位置
int SendBodyJoints(k4abt_frame_t body_frame);

// 序列化并发送身体关节点朝向
int SendBodyJointOrientations(k4abt_frame_t body_frame);

// 序列化并发送身体关节点位置和朝向信息
int SendMergedBodyData(k4abt_frame_t body_frame);

// 发送数据流结束码，防止前置机阻塞
void SendDataEnd();

std::string getCurrentTimeString();

// 类似Java中System.currentTimeMillis函数
// 获取当前系统时间到1970年1月1日0
UINT64 getCurrentTimeMillis();

// 从关节点采样数组中求平均值并发送
int CalculateAndSendSkeletonAvg(k4abt_skeleton_t *skeleton_buffer, int sampleNum, uint32_t selectedID);

// 从关节点采样数组中求滑动平均值并发送
int CalculateAndSendSkeletonMA(k4abt_skeleton_t *skeleton_buffer, int sampleNum, float beta, int mvcount, k4a_float3_t *lastBodyAvg, uint32_t selectedID);
#endif




