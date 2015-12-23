// -*-C++-*-
/*!
 * @file  ManipulatorCommonInterface_CommonSVC_impl.cpp
 * @brief Service implementation code of ManipulatorCommonInterface_Common.idl
 *
 */

#include "ManipulatorCommonInterface_CommonSVC_impl.h"
#include "useSilabs.h"
#include "defreturnid.h"

/*
 * Example implementational code for IDL interface JARA_ARM::ManipulatorCommonInterface_Common
 */
ManipulatorCommonInterface_CommonSVC_impl::ManipulatorCommonInterface_CommonSVC_impl()
{
  // Please add extra constructor code here.
}


ManipulatorCommonInterface_CommonSVC_impl::~ManipulatorCommonInterface_CommonSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::clearAlarms()
{
	std::cout<<"ClearAlarms"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::getActiveAlarm(JARA_ARM::AlarmSeq_out alarms)
{
	std::cout<<"GetActiveAlarm"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::getFeedbackPosJoint(JARA_ARM::JointPos_out pos)
{
	std::cout<<"GetFeedbackPosJoint"<<std::endl;

	//現在接続されたモータ数だけ現在位置を取得
	for(int i=0;i<servoNum;i++){
		hr = RSGetAngle(dev,i+1,&MotorPosition[i]);
		if(hr != TRUE){
			std::cout<<"ERROR : RSGetAngle()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Motor["<<i+1<<"] : "<<MotorPosition[i]<<std::endl;
		}
	}

	//モータ値を[rad],[m]に変換して代入
	pos=new JARA_ARM::JointPos;
	pos->length(servoNum);//グリッパも含める
	
	(*pos)[0]=(MotorPosition[0]/10.0)*M_PI/180.0;//[rad]
	(*pos)[1]=(MotorPosition[1]/10.0)*M_PI/180.0;//[rad]
	(*pos)[2]=((Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin))/1000.0;//[m]
	if(servoNum == 5){
		(*pos)[3]=(MotorPosition[3]/10.0)*M_PI/180.0;//[rad]
		(*pos)[4]=((Hand_LimitMax-Hand_LimitMin)*(MotorPosition[4]-Angle5_LimitMin)/(Angle5_LimitMax-Angle5_LimitMin))/1000.0;//[m]
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::getManipInfo(JARA_ARM::ManipInfo_out mInfo)
{
	std::cout<<"GetManipInfo"<<std::endl;

	mInfo=new JARA_ARM::ManipInfo;
	mInfo->manufactur="VSTONE";
	mInfo->type="SCARA Robot";
	mInfo->cmdCycle=20;
	if(servoNum == 5){
		mInfo->axisNum=4;
		mInfo->isGripper=true;
	}else{
		mInfo->axisNum=3;
		mInfo->isGripper=false;
	}

	std::cout<<" manufactur : "<<mInfo->manufactur<<std::endl;
	std::cout<<" type       : "<<mInfo->type<<std::endl;
	std::cout<<" axisNum    : "<<mInfo->axisNum<<std::endl;
	std::cout<<" cmdCycle   : "<<mInfo->cmdCycle<<std::endl;
	std::cout<<" isGripper  : "<<mInfo->isGripper<<std::endl;

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::getSoftLimitJoint(JARA_ARM::LimitSeq_out softLimit)
{
	std::cout<<"SetSoftLimitJoint"<<std::endl;

	softLimit=new JARA_ARM::LimitSeq;
	softLimit->length(servoNum);

	(*softLimit)[0].upper=(double)JointLimit[0].Upper*M_PI/1800.0;
	(*softLimit)[0].lower=(double)JointLimit[0].Lower*M_PI/1800.0;
	(*softLimit)[1].upper=(double)JointLimit[1].Upper*M_PI/1800.0;
	(*softLimit)[1].lower=(double)JointLimit[1].Lower*M_PI/1800.0;
	(*softLimit)[2].upper=((double)JointLimit[2].Upper-Angle3_LimitMin)*(Z_LimitMax-Z_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin)/1000.0;
	(*softLimit)[2].lower=((double)JointLimit[2].Lower-Angle3_LimitMin)*(Z_LimitMax-Z_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin)/1000.0;

	if(servoNum == 5){
		(*softLimit)[3].upper=(double)JointLimit[3].Upper*M_PI/1800.0;
		(*softLimit)[3].lower=(double)JointLimit[3].Lower*M_PI/1800.0;
		(*softLimit)[4].upper=((double)JointLimit[4].Upper-Angle5_LimitMin)*(Hand_LimitMax-Hand_LimitMin)/(Angle5_LimitMax-Angle5_LimitMin)/1000.0;
		(*softLimit)[4].lower=((double)JointLimit[4].Lower-Angle5_LimitMin)*(Hand_LimitMax-Hand_LimitMin)/(Angle5_LimitMax-Angle5_LimitMin)/1000.0;
	}

	std::cout<<"Success"<<std::endl<<std::endl;
	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::getState(ULONG& state)
{
	std::cout<<"GetState"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::servoOFF()
{
	std::cout<<"ServoOFF (SERVO_OFF)"<<std::endl;

	hr = RSTorqueOnOff(dev,0,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSTorqueOnOff()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::servoON()
{
	std::cout<<"ServoON (SERVO_ON)"<<std::endl;

	hr = RSTorqueOnOff(dev,1,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSTorqueOnOff()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_CommonSVC_impl::setSoftLimitJoint(const JARA_ARM::LimitSeq& softLimit)
{
	std::cout<<"SetSoftLimitJoint"<<std::endl;

	if((Angle1_LimitMax<=softLimit[0].upper*1800.0/M_PI)&&
		(Angle1_LimitMin>=softLimit[0].lower*1800.0/M_PI)&&
		(Angle2_LimitMax<=softLimit[1].upper*1800.0/M_PI)&&
		(Angle2_LimitMin>=softLimit[1].lower*1800.0/M_PI)&&
		(Angle3_LimitMax<=Angle3_LimitMin+(softLimit[2].upper*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin))&&
		(Angle3_LimitMin>=Angle3_LimitMin+(softLimit[2].lower*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin))){

			if(servoNum == 5){
				if((Angle4_LimitMax<=softLimit[3].upper*1800.0/M_PI)&&
					(Angle4_LimitMin>=softLimit[3].lower*1800.0/M_PI)&&
					(Angle5_LimitMax<=Angle5_LimitMin+(softLimit[4].upper*1000.0)*(Angle5_LimitMax-Angle5_LimitMin)/(Hand_LimitMax-Hand_LimitMin))&&
					(Angle5_LimitMin>=Angle5_LimitMin+(softLimit[4].lower*1000.0)*(Angle5_LimitMax-Angle5_LimitMin)/(Hand_LimitMax-Hand_LimitMin))){
						
						JointLimit[3].Upper = (short)(softLimit[3].upper*1800.0/M_PI);
						JointLimit[3].Lower = (short)(softLimit[3].lower*1800.0/M_PI);
						JointLimit[4].Upper = (short)(Angle5_LimitMin+(softLimit[4].upper*1000.0)*(Angle5_LimitMax-Angle5_LimitMin)/(Hand_LimitMax-Hand_LimitMin));
						JointLimit[4].Lower = (short)(Angle5_LimitMin+(softLimit[4].lower*1000.0)*(Angle5_LimitMax-Angle5_LimitMin)/(Hand_LimitMax-Hand_LimitMin));
				}else{
					std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
					RETURNID_VALUE_ERR;
				}
			}

			JointLimit[0].Upper = (short)(softLimit[0].upper*1800.0/M_PI);
			JointLimit[0].Lower = (short)(softLimit[0].lower*1800.0/M_PI);
			JointLimit[1].Upper = (short)(softLimit[1].upper*1800.0/M_PI);
			JointLimit[1].Lower = (short)(softLimit[1].lower*1800.0/M_PI);
			JointLimit[2].Upper = (short)(Angle3_LimitMin+(softLimit[2].upper*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));
			JointLimit[2].Lower = (short)(Angle3_LimitMin+(softLimit[2].lower*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));

	}else{
		std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
		RETURNID_VALUE_ERR;
	}

	std::cout<<"Success"<<std::endl<<std::endl;
	RETURNID_OK;
}



// End of example implementational code



