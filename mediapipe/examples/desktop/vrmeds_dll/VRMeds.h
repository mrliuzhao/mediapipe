#define _CRT_SECURE_NO_WARNINGS
#pragma once

#ifndef __VRMeds_h__
#define __VRMeds_h__

// ���л�����������ؽڵ�λ��
int SendBodyJoints(k4abt_frame_t body_frame);

// ���л�����������ؽڵ㳯��
int SendBodyJointOrientations(k4abt_frame_t body_frame);

// ���л�����������ؽڵ�λ�úͳ�����Ϣ
int SendMergedBodyData(k4abt_frame_t body_frame);

// ���������������룬��ֹǰ�û�����
void SendDataEnd();

std::string getCurrentTimeString();

// ����Java��System.currentTimeMillis����
// ��ȡ��ǰϵͳʱ�䵽1970��1��1��0
UINT64 getCurrentTimeMillis();

// �ӹؽڵ������������ƽ��ֵ������
int CalculateAndSendSkeletonAvg(k4abt_skeleton_t *skeleton_buffer, int sampleNum, uint32_t selectedID);

// �ӹؽڵ�����������󻬶�ƽ��ֵ������
int CalculateAndSendSkeletonMA(k4abt_skeleton_t *skeleton_buffer, int sampleNum, float beta, int mvcount, k4a_float3_t *lastBodyAvg, uint32_t selectedID);
#endif




