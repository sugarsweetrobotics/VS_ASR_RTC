//useSilabs.h
#ifndef __USESILABS_H__
#define __USESILABS_H__

//数学定数の使用
#define _USE_MATH_DEFINES

//includeファイル
#include	<Windows.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	<iostream>

//CP2110/4 HID USB-to-UART インターフェースライブラリのインクルードファイル
#include	<SLABCP2110.h>
#include	<SLABHIDtoUART.h>
#include	<CP2114_Common.h>

//CP2110/4 HID USB-to-UART インターフェースライブラリ へのリンク
#pragma	comment(lib,"SLABHIDtoUART.LIB")
#pragma	comment(lib,"winmm.LIB")

//SCARAロボットのVender ID
#define	VID	(0x10C4)

//SCARAロボットのProduct ID
#define	PID	(0xEA80)

//第一関節の軸間距離（mm）
#define	AXISLEN_A	(80.0)
//第二関節の軸間距離（mm）
#define	AXISLEN_B	(80.0)

//第1軸モータの可動範囲
#define	Angle1_LimitMax	(1200)
#define	Angle1_LimitMin	(-1200)
//第2軸モータの可動範囲
#define	Angle2_LimitMax	(1400)
#define	Angle2_LimitMin	(-1400)
//第3軸モータの可動範囲
#define	Angle3_LimitMax	(1100)
#define	Angle3_LimitMin	(-1100)
//第4軸モータの可動範囲
#define	Angle4_LimitMax	(1500)
#define	Angle4_LimitMin	(-1500)
//第5軸モータの可動範囲
#define	Angle5_LimitMax	(400)
#define	Angle5_LimitMin	(-400)

//X軸可動範囲[mm]
#define X_LimitMax		(160.0)
#define X_LimitMin		(-160.0)
//Y軸可動範囲[mm]
#define Y_LimitMax		(160.0)
#define Y_LimitMin		(-160.0)
//Z軸可動範囲[mm]
#define Z_LimitMax		(60.0)
#define Z_LimitMin		(0.0)

//ハンド可動範囲[mm]
#define Hand_LimitMax		(30.0)
#define Hand_LimitMin		(10.0)

//動作時間係数
#define MoveTime		(80)

//グローバル変数
extern int hr;//返り値
extern HID_UART_DEVICE dev;//通信ハンドル
//extern int MoveTime;
extern short MotorPosition[5];
extern short StartMotorPosition[5];
extern short HomeMotorPosition[5];
extern int servoNum;//サーボ数
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
extern JLimit JointLimit[5];//モータ指令値で代入
extern Offset BaseOffset;

//関数のプロトタイプ宣言
extern int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num) ;
extern int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
extern int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num );
extern int ReadLocalEcho( HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len );
extern int CartesianLimitJudgement( double x , double y , double z );
extern int JointLimitJudgement( short *JointAngle );

#endif//__DEFRETURNID_H__