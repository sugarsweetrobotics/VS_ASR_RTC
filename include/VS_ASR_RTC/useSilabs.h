//useSilabs.h
#ifndef __USESILABS_H__
#define __USESILABS_H__

//���w�萔�̎g�p
#define _USE_MATH_DEFINES

//include�t�@�C��
#include	<Windows.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	<iostream>

//CP2110/4 HID USB-to-UART �C���^�[�t�F�[�X���C�u�����̃C���N���[�h�t�@�C��
#include	<SLABCP2110.h>
#include	<SLABHIDtoUART.h>
#include	<CP2114_Common.h>

//CP2110/4 HID USB-to-UART �C���^�[�t�F�[�X���C�u���� �ւ̃����N
#pragma	comment(lib,"SLABHIDtoUART.LIB")
#pragma	comment(lib,"winmm.LIB")

//SCARA���{�b�g��Vender ID
#define	VID	(0x10C4)

//SCARA���{�b�g��Product ID
#define	PID	(0xEA80)

//���֐߂̎��ԋ����imm�j
#define	AXISLEN_A	(80.0)
//���֐߂̎��ԋ����imm�j
#define	AXISLEN_B	(80.0)

//��1�����[�^�̉��͈�
#define	Angle1_LimitMax	(1200)
#define	Angle1_LimitMin	(-1200)
//��2�����[�^�̉��͈�
#define	Angle2_LimitMax	(1400)
#define	Angle2_LimitMin	(-1400)
//��3�����[�^�̉��͈�
#define	Angle3_LimitMax	(1100)
#define	Angle3_LimitMin	(-1100)
//��4�����[�^�̉��͈�
#define	Angle4_LimitMax	(1500)
#define	Angle4_LimitMin	(-1500)
//��5�����[�^�̉��͈�
#define	Angle5_LimitMax	(400)
#define	Angle5_LimitMin	(-400)

//X�����͈�[mm]
#define X_LimitMax		(160.0)
#define X_LimitMin		(-160.0)
//Y�����͈�[mm]
#define Y_LimitMax		(160.0)
#define Y_LimitMin		(-160.0)
//Z�����͈�[mm]
#define Z_LimitMax		(60.0)
#define Z_LimitMin		(0.0)

//�n���h���͈�[mm]
#define Hand_LimitMax		(30.0)
#define Hand_LimitMin		(10.0)

//���쎞�ԌW��
#define MoveTime		(80)

//�O���[�o���ϐ�
extern int hr;//�Ԃ�l
extern HID_UART_DEVICE dev;//�ʐM�n���h��
//extern int MoveTime;
extern short MotorPosition[5];
extern short StartMotorPosition[5];
extern short HomeMotorPosition[5];
extern int servoNum;//�T�[�{��
extern int CartesianSpeed;
extern int JointSpeed;

struct CLimit{
	double Upper;
	double Lower;
};

struct JLimit{
	short Upper;
	short Lower;
};

struct Catesian{
	CLimit x;
	CLimit y;
	CLimit z;
};
struct Offset{
	double x;//[m]
	double y;//[m]
	double z;//[m]
	double Rz;//[rad]
};

extern Catesian CatesianLimit;//[mm]
extern JLimit JointLimit[5];//���[�^�w�ߒl�ő��
extern Offset BaseOffset;

//�֐��̃v���g�^�C�v�錾
extern int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num) ;
extern int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
extern int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num );
extern int ReadLocalEcho( HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len );
extern int CartesianLimitJudgement( double x , double y , double z );
extern int JointLimitJudgement( short *JointAngle );

#endif//__DEFRETURNID_H__