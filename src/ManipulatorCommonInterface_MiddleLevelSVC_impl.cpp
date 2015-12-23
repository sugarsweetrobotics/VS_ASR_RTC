// -*-C++-*-
/*!
 * @file  ManipulatorCommonInterface_MiddleLevelSVC_impl.cpp
 * @brief Service implementation code of ManipulatorCommonInterface_MiddleLevel.idl
 *
 */

#include "ManipulatorCommonInterface_MiddleLevelSVC_impl.h"
#include "useSilabs.h"
#include "defreturnid.h"

/*
 * Example implementational code for IDL interface JARA_ARM::ManipulatorCommonInterface_Middle
 */
ManipulatorCommonInterface_MiddleSVC_impl::ManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra constructor code here.
}


ManipulatorCommonInterface_MiddleSVC_impl::~ManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::closeGripper()
{
	std::cout<<"CloseGripper (HAND_CLOSE)"<<std::endl;

	if(servoNum == 5){
		MotorPosition[4]=Angle5_LimitMax;

		hr = RSMove(dev,&MotorPosition[4],MoveTime,5,1);

		if(hr != TRUE){
			std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Success"<<std::endl<<std::endl;
		}
		RETURNID_OK;
	}else{
		std::cout<<"ERROR : No Gripper"<<std::endl<<std::endl;
		RETURNID_NG;
	}

}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM::HgMatrix offset)
{
	std::cout<<"GetBaseOffset"<<std::endl;

	for(int i=0 ; i<3 ;i++){
		for(int j=0 ; j<3 ;j++){
			offset[i][j] = 0.0;
		}
		offset[i][i] = 1.0;
	}

	offset[0][0] = cos(BaseOffset.Rz);		offset[0][1] = -sin(BaseOffset.Rz);				offset[0][3] = BaseOffset.x;
	offset[1][0] = sin(BaseOffset.Rz);		offset[1][1] = cos(BaseOffset.Rz);				offset[1][3] = BaseOffset.y;
																							offset[2][3] = BaseOffset.z;

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM::CarPosWithElbow& pos)
{
	std::cout<<"GetFeedbackPosCartesian"<<std::endl;

	//���ݐڑ����ꂽ���[�^���������݈ʒu(�A�[�����W�n)���擾
	for(int i=0;i<servoNum;i++){
		hr = RSGetAngle(dev,i+1,&MotorPosition[i]);
		if(hr != TRUE){
			std::cout<<"ERROR : RSGetAngle()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Motor["<<i+1<<"] : "<<MotorPosition[i]<<std::endl;
		}
	}

	if(servoNum != 3){
		MotorPosition[3] = 0;
	}

	//���[�^�l���猻�݈ʒu���Z�o
	double x = AXISLEN_A*cos(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*cos((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//[m]
	double y = AXISLEN_A*sin(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*sin((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//[m]
	double z = (Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin);//[m]
	double Rz = (MotorPosition[0]+MotorPosition[1]+MotorPosition[3])*M_PI/1800.0;//[rad]

	//�I�t�Z�b�g�l��(�A�[�����W�n�˃��{�b�g���W�n�ϊ�)
	pos.carPos[0][0]=cos(Rz + BaseOffset.Rz);
	pos.carPos[0][1]=-sin(Rz + BaseOffset.Rz);
	pos.carPos[0][2]=0.0;
	pos.carPos[0][3]=(x + BaseOffset.x)/1000.0;//x[m]

	pos.carPos[1][0]=sin(Rz + BaseOffset.Rz);
	pos.carPos[1][1]=cos(Rz + BaseOffset.Rz);
	pos.carPos[1][2]=0.0;
	pos.carPos[1][3]=(y + BaseOffset.y)/1000.0;//y[m]
	
	pos.carPos[2][0]=0.0;
	pos.carPos[2][1]=0.0;
	pos.carPos[2][2]=1.0;
	pos.carPos[2][3]=(z + BaseOffset.z)/1000.0;//z[m]

	if(MotorPosition[1] >= 0){
		//�E��
		pos.structFlag = 1;
	}else{
		//����
		pos.structFlag = 2;
	}

	std::cout<<" x  : "<<pos.carPos[0][3]*1000.0<<"[mm]"<<std::endl;
	std::cout<<" y  : "<<pos.carPos[1][3]*1000.0<<std::endl;
	std::cout<<" z  : "<<pos.carPos[2][3]*1000.0<<std::endl;
	std::cout<<" Rz : "<<(MotorPosition[0]+MotorPosition[1]+MotorPosition[3])/10.0<<"[deg]"<<std::endl;
	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM::CartesianSpeed& speed)
{
	std::cout<<"GetMaxSpeedCartesian"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM::DoubleSeq_out speed)
{
	std::cout<<"GetMaxSpeedJoint"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(CORBA::Double& aclTime)
{
	std::cout<<"GetMinAccelTimeCartesian"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(CORBA::Double& aclTime)
{
	std::cout<<"GetMinAccelTimeJoint"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM::LimitValue& xLimit, JARA_ARM::LimitValue& yLimit, JARA_ARM::LimitValue& zLimit)
{
	std::cout<<"GetSoftLimitCartesian"<<std::endl;

	xLimit.upper = CatesianLimit.x.Upper/1000.0;//[m](���{�b�g���W�n)
	xLimit.lower = CatesianLimit.x.Lower/1000.0;
	yLimit.upper = CatesianLimit.y.Upper/1000.0;
	yLimit.lower = CatesianLimit.y.Lower/1000.0;
	zLimit.upper = CatesianLimit.z.Upper/1000.0;
	zLimit.lower = CatesianLimit.z.Lower/1000.0;

	std::cout<<"Success"<<std::endl<<std::endl;
	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::moveGripper(ULONG angleRatio)
{
	std::cout<<"MoveGripper (HAND_MOV)"<<std::endl;

	if(servoNum == 5){
		MotorPosition[4]=(short)((Angle5_LimitMin-Angle5_LimitMax)*((angleRatio)/100.0)+Angle5_LimitMax);//�O���b�p�̒l��[%]����ϊ����đ�����邱��!

		hr = RSMove(dev,&MotorPosition[4],MoveTime,5,1);

		if(hr != TRUE){
			std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Success"<<std::endl<<std::endl;
		}
		RETURNID_OK;
	}else{
		std::cout<<"ERROR : No Gripper"<<std::endl<<std::endl;
		RETURNID_NG;
	}
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)
{
	std::cout<<"MoveLinearCartesianAbs (CMVS)"<<std::endl;

	//CartesianSpeed����
	if((CartesianSpeed <= 0)||(CartesianSpeed > 100)){
		std::cout<<"ERROR : Wrong Cartesian Speed"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//�^����ꂽ���{�b�g���W�n�̖ڕW�l��C�\�t�g���~�b�g����
	hr = CartesianLimitJudgement( carPoint.carPos[0][3]*1000.0 , carPoint.carPos[1][3]*1000.0 , carPoint.carPos[2][3]*1000.0 );
	if(hr != TRUE){
		std::cout<<"ERROR : Cartesian Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//���ݐڑ����ꂽ���[�^���������[�^�l���擾
	for(int i=0;i<servoNum;i++){
		hr = RSGetAngle(dev,i+1,&MotorPosition[i]);
		if(hr != TRUE){
			std::cout<<"ERROR : RSGetAngle()"<<std::endl<<std::endl;
			RETURNID_NG;
		}
	}

	//���[�^�l���猻�݈ʒu(�A�[�����W�n)���Z�o
	double startPos[4];//x,y,z,rotz
	startPos[3]=(MotorPosition[0]+MotorPosition[1]+MotorPosition[3])/10.0;//Rz[deg]
	startPos[0]=AXISLEN_A*cos(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*cos((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//x[mm]
	startPos[1]=AXISLEN_A*sin(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*sin((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//y[mm]
	startPos[2]=((Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin));//z[mm]

	double viaPos[4];
	int StrideNum=100 * 100 / CartesianSpeed;
	double Buf[2];

	//�I�t�Z�b�g�l��(�^����ꂽ���{�b�g���W�n�̖ڕW�l�˃A�[�����W�n�ϊ�)�A[m]->[mm]�ϊ��A[rad]->[deg]�ϊ�
	double x = (cos(BaseOffset.Rz)*carPoint.carPos[0][3] + sin(BaseOffset.Rz)*carPoint.carPos[1][3] +BaseOffset.x)*1000.0;
	double y = (-sin(BaseOffset.Rz)*carPoint.carPos[0][3] + cos(BaseOffset.Rz)*carPoint.carPos[1][3] + BaseOffset.y)*1000.0;
	double z = (carPoint.carPos[2][3] + BaseOffset.z)*1000.0;
	double Rz = (atan2(carPoint.carPos[0][1],carPoint.carPos[0][0]) + BaseOffset.Rz)*180.0/M_PI;

	for(int i=0; i<=StrideNum ; i++){
		viaPos[0]=(x-startPos[0])*(double)i/(double)StrideNum+startPos[0];
		viaPos[1]=(y-startPos[1])*(double)i/(double)StrideNum+startPos[1];
		viaPos[2]=(z-startPos[2])*(double)i/(double)StrideNum+startPos[2];
		viaPos[3]=(Rz-startPos[3])*(double)i/(double)StrideNum+startPos[3];
	
		//�t�^���w
		MotorPosition[2]=(short)(Angle3_LimitMin+viaPos[2]*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));

		Buf[0] = (viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]+AXISLEN_A*AXISLEN_A-AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_A);
		Buf[1] = (viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-AXISLEN_A*AXISLEN_A+AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_B);

		if(carPoint.structFlag == 2){//Left
			MotorPosition[0] = (short)((atan2(viaPos[1],viaPos[0])+atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
			MotorPosition[1] = (short)((-atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0])-atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
		}else{//Right
			MotorPosition[0] = (short)((atan2(viaPos[1],viaPos[0])-atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
			MotorPosition[1] = (short)((atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0])+atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
		}
		
		MotorPosition[3]=(short)(viaPos[3]*10.0-MotorPosition[0]-MotorPosition[1]);

		//���[�^�w�ߒl��J�\�t�g���~�b�g����
		hr = JointLimitJudgement( MotorPosition );
		if(hr != TRUE){
			std::cout<<"ERROR : Joint Soft Limit Over"<<std::endl<<std::endl;
			RETURNID_NG;
		}

		hr = RSMove(dev,MotorPosition,20,1,servoNum);
	
		if(hr != TRUE){
			std::cout<<"ERROR : RSMove()"<<std::endl;
			RETURNID_NG;
		}
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)
{
	std::cout<<"MoveLinearCartesianRel"<<std::endl;

	//CartesianSpeed����
	if((CartesianSpeed <= 0)||(CartesianSpeed > 100)){
		std::cout<<"ERROR : Wrong Cartesian Speed"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//���ݐڑ����ꂽ���[�^���������[�^�l���擾
	for(int i=0;i<servoNum;i++){
		hr = RSGetAngle(dev,i+1,&MotorPosition[i]);
		if(hr != TRUE){
			std::cout<<"ERROR : RSGetAngle()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			//std::cout<<"Motor["<<i+1<<"] : "<<MotorPosition[i]<<std::endl;
		}
	}

	//���[�^�l���猻�݈ʒu(�A�[�����W�n)���Z�o
	double startPos[4];//x,y,z,rotz
	startPos[3]=(MotorPosition[0]+MotorPosition[1]+MotorPosition[3])/10.0;//Rz[deg]
	startPos[0]=AXISLEN_A*cos(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*cos((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//x[mm]
	startPos[1]=AXISLEN_A*sin(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*sin((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//y[mm]
	startPos[2]=((Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin));//z[mm]

	//�ڕW�l�𑊑΍��W�����΍��W�ɕϊ�
	double Abs_x = carPoint.carPos[0][3]*1000.0 + startPos[0];//[mm]
	double Abs_y = carPoint.carPos[1][3]*1000.0 + startPos[1];//[mm]
	double Abs_z = carPoint.carPos[2][3]*1000.0 + startPos[2];//[mm]



	//�^����ꂽ���{�b�g���W�n�̖ڕW�l��C�\�t�g���~�b�g����
	hr = CartesianLimitJudgement( Abs_x , Abs_y , Abs_z );
	if(hr != TRUE){
		std::cout<<"ERROR : Cartesian Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}



	double viaPos[4];
	int StrideNum=100 * 100 / CartesianSpeed;
	double Buf[2];

	//�I�t�Z�b�g�l��(�^����ꂽ���{�b�g���W�n�̖ڕW�l�˃A�[�����W�n�ϊ�)�A[m]->[mm]�ϊ��A[rad]->[deg]�ϊ�
	double x = cos(BaseOffset.Rz)*Abs_x + sin(BaseOffset.Rz)*Abs_y + (BaseOffset.x)*1000.0;
	double y = -sin(BaseOffset.Rz)*Abs_x + cos(BaseOffset.Rz)*Abs_y + (BaseOffset.y)*1000.0;
	double z = Abs_z + (BaseOffset.z)*1000.0;
	double Rz = (atan2(carPoint.carPos[0][1],carPoint.carPos[0][0]) + BaseOffset.Rz)*180.0/M_PI +startPos[3];


	for(int i=0; i<=StrideNum ; i++){
		viaPos[0]=(x-startPos[0])*(double)i/(double)StrideNum+startPos[0];
		viaPos[1]=(y-startPos[1])*(double)i/(double)StrideNum+startPos[1];
		viaPos[2]=(z-startPos[2])*(double)i/(double)StrideNum+startPos[2];
		viaPos[3]=(Rz-startPos[3])*(double)i/(double)StrideNum+startPos[3];
	
		//�t�^���w
		MotorPosition[2]=(short)(Angle3_LimitMin+viaPos[2]*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));

		Buf[0] = (viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]+AXISLEN_A*AXISLEN_A-AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_A);
		Buf[1] = (viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-AXISLEN_A*AXISLEN_A+AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_B);

		if(carPoint.structFlag == 2){//Left
			MotorPosition[0] = (short)((atan2(viaPos[1],viaPos[0])+atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
			MotorPosition[1] = (short)((-atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0])-atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
		}else{//Right
			MotorPosition[0] = (short)((atan2(viaPos[1],viaPos[0])-atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
			MotorPosition[1] = (short)((atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[0]*Buf[0]),Buf[0])+atan2(sqrt(viaPos[0]*viaPos[0]+viaPos[1]*viaPos[1]-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
		}
		
		MotorPosition[3]=(short)(viaPos[3]*10.0-MotorPosition[0]-MotorPosition[1]);

		//���[�^�w�ߒl��J�\�t�g���~�b�g����
		hr = JointLimitJudgement( MotorPosition );
		if(hr != TRUE){
			std::cout<<"ERROR : Joint Soft Limit Over"<<std::endl<<std::endl;
			RETURNID_NG;
		}

		hr = RSMove(dev,MotorPosition,20,1,servoNum);
	
		if(hr != TRUE){
			std::cout<<"ERROR : RSMove()"<<std::endl;
			RETURNID_NG;
		}
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)
{
	std::cout<<"MovePTPCartesianAbs (CMOV)"<<std::endl;

	//CartesianSpeed����
	if((CartesianSpeed <= 0)||(CartesianSpeed > 100)){
		std::cout<<"ERROR : Wrong Cartesian Speed"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//�^����ꂽ���{�b�g���W�n�̖ڕW�l��C�\�t�g���~�b�g����
	hr = CartesianLimitJudgement( carPoint.carPos[0][3]*1000.0 , carPoint.carPos[1][3]*1000.0 , carPoint.carPos[2][3]*1000.0 );
	if(hr != TRUE){
		std::cout<<"ERROR : Cartesian Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//�I�t�Z�b�g�l��(�^����ꂽ���{�b�g���W�n�̖ڕW�l�˃A�[�����W�n�ϊ�)�A[m]->[mm]�ϊ�]
	double x = (cos(BaseOffset.Rz)*carPoint.carPos[0][3] - sin(BaseOffset.Rz)*carPoint.carPos[1][3] + BaseOffset.x)*1000.0;
	double y = (sin(BaseOffset.Rz)*carPoint.carPos[0][3] + cos(BaseOffset.Rz)*carPoint.carPos[1][3] + BaseOffset.y)*1000.0;
	double z = (carPoint.carPos[2][3] + BaseOffset.z)*1000.0;

	//�t�^���w
	double Buf[2];
	MotorPosition[2]=(short)(Angle3_LimitMin+z*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));

	Buf[0] = (x*x+y*y+AXISLEN_A*AXISLEN_A-AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_A);
	Buf[1] = (x*x+y*y-AXISLEN_A*AXISLEN_A+AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_B);

	if(carPoint.structFlag == 2){//Left
		MotorPosition[0] = (short)((atan2(y,x)+atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
		MotorPosition[1] = (short)((-atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0])-atan2(sqrt(x*x+y*y-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
	}else{//Right
		MotorPosition[0] = (short)((atan2(y,x)-atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
		MotorPosition[1] = (short)((atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0])+atan2(sqrt(x*x+y*y-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
	}

	double Buffer = asin(-carPoint.carPos[0][2]);//B
	if(fabs(M_PI_2-fabs(Buffer)) <= DBL_EPSILON){
		//�������s��
		RETURNID_VALUE_ERR;
	}else{
		Buffer=atan2(-carPoint.carPos[0][1],carPoint.carPos[0][0])*180.0/M_PI;//C
	}
	MotorPosition[3]=(short)(Buffer*10.0-MotorPosition[0]-MotorPosition[1] - BaseOffset.Rz*1800.0/M_PI);

	//���[�^�w�ߒl��J�\�t�g���~�b�g����
	hr = JointLimitJudgement( MotorPosition );

	if(hr != TRUE){
		std::cout<<"ERROR : Joint Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	hr = RSMove(dev,MotorPosition,MoveTime * 100 / CartesianSpeed,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)//����
{
	std::cout<<"movePTPCartesianRel"<<std::endl;

	//CartesianSpeed����
	if((CartesianSpeed <= 0)||(CartesianSpeed > 100)){
		std::cout<<"ERROR : Wrong Cartesian Speed"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//���ݐڑ����ꂽ���[�^���������[�^�l���擾
	for(int i=0;i<servoNum;i++){
		hr = RSGetAngle(dev,i+1,&StartMotorPosition[i]);
		if(hr != TRUE){
			std::cout<<"ERROR : RSGetAngle()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Motor["<<i+1<<"] : "<<StartMotorPosition[i]<<std::endl;
		}
	}

	//���[�^�l���猻�݈ʒu(�A�[�����W�n)���Z�o
	double startPos[4];//x,y,z,rotz
	startPos[3]=(MotorPosition[0]+MotorPosition[1]+MotorPosition[3])/10.0;//Rz[deg]
	startPos[0]=AXISLEN_A*cos(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*cos((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//x[mm]
	startPos[1]=AXISLEN_A*sin(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*sin((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//y[mm]
	startPos[2]=((Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin));//z[mm]


	//�ڕW�l�𑊑΍��W�����΍��W�ɕϊ�
	double Abs_x = carPoint.carPos[0][3]*1000.0 + startPos[0];//[mm]
	double Abs_y = carPoint.carPos[1][3]*1000.0 + startPos[1];//[mm]
	double Abs_z = carPoint.carPos[2][3]*1000.0 + startPos[2];//[mm]

	//�^����ꂽ���{�b�g���W�n�̖ڕW�l��C�\�t�g���~�b�g����
	hr = CartesianLimitJudgement( Abs_x , Abs_y , Abs_z );
	if(hr != TRUE){
		std::cout<<"ERROR : Cartesian Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//�I�t�Z�b�g�l��(�^����ꂽ���{�b�g���W�n�̖ڕW�l�˃A�[�����W�n�ϊ�)�A[m]->[mm]�ϊ�
	double x = cos(BaseOffset.Rz)*Abs_x + sin(BaseOffset.Rz)*Abs_y + (BaseOffset.x)*1000.0;
	double y = -sin(BaseOffset.Rz)*Abs_x + cos(BaseOffset.Rz)*Abs_y + (BaseOffset.y)*1000.0;
	double z = Abs_z + (BaseOffset.z)*1000.0;

	//�t�^���w
	double Buf[2];
	MotorPosition[2]=(short)(Angle3_LimitMin+z*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));

	Buf[0] = (x*x+y*y+AXISLEN_A*AXISLEN_A-AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_A);
	Buf[1] = (x*x+y*y-AXISLEN_A*AXISLEN_A+AXISLEN_B*AXISLEN_B)/(2.0*AXISLEN_B);

	if(carPoint.structFlag == 2){//Left
		MotorPosition[0] = (short)((atan2(y,x)+atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
		MotorPosition[1] = (short)((-atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0])-atan2(sqrt(x*x+y*y-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
	}else{//Right
		MotorPosition[0] = (short)((atan2(y,x)-atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0]))*1800.0/M_PI);
		MotorPosition[1] = (short)((atan2(sqrt(x*x+y*y-Buf[0]*Buf[0]),Buf[0])+atan2(sqrt(x*x+y*y-Buf[1]*Buf[1]),Buf[1]))*1800.0/M_PI);
	}

	double Buffer = asin(-carPoint.carPos[0][2]);//B
	if(fabs(M_PI_2-fabs(Buffer)) <= DBL_EPSILON){
		//�������s��
		RETURNID_VALUE_ERR;
	}else{
		Buffer=atan2(-carPoint.carPos[0][1],carPoint.carPos[0][0])*180.0/M_PI + startPos[3];//C
	}
	MotorPosition[3]=(short)(Buffer*10.0-MotorPosition[0]-MotorPosition[1] - BaseOffset.Rz*1800.0/M_PI);

	//���[�^�w�ߒl��J�\�t�g���~�b�g����
	hr = JointLimitJudgement( MotorPosition );
	if(hr != TRUE){
		std::cout<<"ERROR : Joint Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	hr = RSMove(dev,MotorPosition,MoveTime * 100 / CartesianSpeed,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}
	
	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM::JointPos& jointPoints)
{
	std::cout<<"movePTPJointAbs (JMOV)"<<std::endl;

	//JointSpeed����
	if((JointSpeed <= 0)||(JointSpeed > 100)){
		std::cout<<"ERROR : Wrong Joint Speed"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	MotorPosition[0]=(short)(jointPoints[0]*1800/M_PI);//���[�^�l�������邱��!
	MotorPosition[1]=(short)(jointPoints[1]*1800/M_PI);
	MotorPosition[2]=(short)(Angle3_LimitMin+(jointPoints[2]*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));
	if(servoNum == 5){
		MotorPosition[3]=(short)(jointPoints[3]*1800/M_PI);
	}

	//���[�^�w�ߒl��J�\�t�g���~�b�g����
	hr = JointLimitJudgement( MotorPosition );
	if(hr != TRUE){
		std::cout<<"ERROR : Joint Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//���[�^�l����ڕW�ʒu(�A�[�����W�n)���Z�o
	double targetPos_Arm[3],targetPos[3];//x,y,z
	targetPos_Arm[0]=AXISLEN_A*cos(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*cos((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//x[mm]
	targetPos_Arm[1]=AXISLEN_A*sin(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*sin((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//y[mm]
	targetPos_Arm[2]=((Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin));//z[mm]

	//�x�[�X�I�t�Z�b�g�l��(�A�[�����W�n�����{�b�g���W�n�ɕϊ�)
	targetPos[0]= cos(BaseOffset.Rz)*targetPos_Arm[0] - sin(BaseOffset.Rz)*targetPos_Arm[1] - BaseOffset.x;
	targetPos[1]= sin(BaseOffset.Rz)*targetPos_Arm[0] + cos(BaseOffset.Rz)*targetPos_Arm[1] - BaseOffset.y;
	targetPos[2]= targetPos_Arm[2] - BaseOffset.z;


	//�ڕW�ʒu��C�\�t�g���~�b�g����
	hr = CartesianLimitJudgement( targetPos[0] , targetPos[1] , targetPos[2] );
	if(hr != TRUE){
		std::cout<<"ERROR : Cartesian Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	hr = RSMove(dev,MotorPosition,MoveTime * 100 / JointSpeed,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}
	
	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM::JointPos& jointPoints)
{
	std::cout<<"movePTPJointRel"<<std::endl;

	//JointSpeed����
	if((JointSpeed <= 0)||(JointSpeed > 100)){
		std::cout<<"ERROR : Wrong Joint Speed"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//���ݐڑ����ꂽ���[�^���������[�^�l���擾
	for(int i=0;i<servoNum;i++){
		hr = RSGetAngle(dev,i+1,&StartMotorPosition[i]);
		if(hr != TRUE){
			std::cout<<"ERROR : RSGetAngle()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Motor["<<i+1<<"] : "<<StartMotorPosition[i]<<std::endl;
		}
	}

	MotorPosition[0]=(short)(jointPoints[0]*1800/M_PI) + StartMotorPosition[0];//���[�^�l�������邱��!
	MotorPosition[1]=(short)(jointPoints[1]*1800/M_PI) + StartMotorPosition[1];
	MotorPosition[2]=(short)(Angle3_LimitMin+(jointPoints[2]*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin)) + StartMotorPosition[2];
	if(servoNum == 5){
		MotorPosition[3]=(short)(jointPoints[3]*1800/M_PI) + StartMotorPosition[3];
	}

	//���[�^�w�ߒl��J�\�t�g���~�b�g����
	hr = JointLimitJudgement( MotorPosition );
	if(hr != TRUE){
		std::cout<<"ERROR : Joint Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	//���[�^�l����ڕW�ʒu(�A�[�����W�n)���Z�o
	double targetPos_Arm[3],targetPos[3];//x,y,z
	targetPos_Arm[0]=AXISLEN_A*cos(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*cos((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//x[mm]
	targetPos_Arm[1]=AXISLEN_A*sin(MotorPosition[0]*M_PI/1800.0)+AXISLEN_B*sin((MotorPosition[0]+MotorPosition[1])*M_PI/1800.0);//y[mm]
	targetPos_Arm[2]=((Z_LimitMax-Z_LimitMin)*(MotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin));//z[mm]

	//�x�[�X�I�t�Z�b�g�l��(�A�[�����W�n�����{�b�g���W�n�ɕϊ�)
	targetPos[0]= cos(BaseOffset.Rz)*targetPos_Arm[0] - sin(BaseOffset.Rz)*targetPos_Arm[1] - BaseOffset.x;
	targetPos[1]= sin(BaseOffset.Rz)*targetPos_Arm[0] + cos(BaseOffset.Rz)*targetPos_Arm[1] - BaseOffset.y;
	targetPos[2]= targetPos_Arm[2] - BaseOffset.z;


	//�ڕW�ʒu��C�\�t�g���~�b�g����
	hr = CartesianLimitJudgement( targetPos[0] , targetPos[1] , targetPos[2] );
	if(hr != TRUE){
		std::cout<<"ERROR : Cartesian Soft Limit Over"<<std::endl<<std::endl;
		RETURNID_NG;
	}

	hr = RSMove(dev,MotorPosition,MoveTime * 100 / JointSpeed,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}
	
	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::openGripper()
{
	std::cout<<"OpenGripper (HAND_OPEN)"<<std::endl;

	if(servoNum == 5){
		MotorPosition[4]=Angle5_LimitMin;

		hr = RSMove(dev,&MotorPosition[4],MoveTime,5,1);

		if(hr != TRUE){
			std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
			RETURNID_NG;
		}else{
			std::cout<<"Success"<<std::endl<<std::endl;
		}
		RETURNID_OK;
	}else{
		std::cout<<"ERROR : No Gripper"<<std::endl<<std::endl;
		RETURNID_NG;
	}
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::pause()
{
	std::cout<<"Pause"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::resume()
{
	std::cout<<"Resume"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::stop()
{
	std::cout<<"Stop"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(CORBA::Double aclTime)
{
	std::cout<<"SetAccelTimeCartesian"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(CORBA::Double aclTime)
{
	std::cout<<"SetAccelTimeJoint"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM::HgMatrix offset)
{
	std::cout<<"SetBaseOffset"<<std::endl;

	BaseOffset.x = offset[0][3];
	BaseOffset.y = offset[1][3];
	BaseOffset.z = offset[2][3];

	if(fabs(offset[0][0]) <= DBL_EPSILON){//Rz=�}90[deg]
		if(fabs(offset[1][0]-1.0) <= DBL_EPSILON){
			BaseOffset.Rz = M_PI_2;
		}else if(fabs(offset[1][0]+1.0) <= DBL_EPSILON){
			BaseOffset.Rz = -M_PI_2;
		}else{
			std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
			RETURNID_VALUE_ERR;
		}
	}else{
		BaseOffset.Rz = atan2(offset[1][0],offset[0][0]);
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM::HgMatrix offset)
{
	std::cout<<"SetControlPointOffset"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM::CartesianSpeed& speed)
{
	std::cout<<"SetMaxSpeedCartesian"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM::DoubleSeq& speed)
{
	std::cout<<"SetMaxSpeedJoint"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(CORBA::Double aclTime)
{
	std::cout<<"SetMinAccelTimeCartesian"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(CORBA::Double aclTime)
{
	std::cout<<"SetMinAccelTimeJoint"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM::LimitValue& xLimit, const JARA_ARM::LimitValue& yLimit, const JARA_ARM::LimitValue& zLimit)
{
	std::cout<<"SetSoftLimitCartesian"<<std::endl;

	if((xLimit.upper*1000 <= X_LimitMax) && 
		(xLimit.lower*1000 >= X_LimitMin) &&
		(yLimit.upper*1000 <= Y_LimitMax) && 
		(yLimit.lower*1000 >= Y_LimitMin) &&
		(zLimit.upper*1000 <= Z_LimitMax) && 
		(zLimit.lower*1000 >= Z_LimitMin) 
		){
			CatesianLimit.x.Upper = xLimit.upper*1000;
			CatesianLimit.x.Lower = xLimit.lower*1000;
			CatesianLimit.y.Upper = yLimit.upper*1000;
			CatesianLimit.y.Lower = yLimit.lower*1000;
			CatesianLimit.z.Upper = zLimit.upper*1000;
			CatesianLimit.z.Lower = zLimit.lower*1000;

	}else{
		std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
		RETURNID_VALUE_ERR;
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(ULONG spdRatio)
{
	std::cout<<"SetSpeedCartesian"<<std::endl;

	if(spdRatio >= 0 && spdRatio <= 100){
		CartesianSpeed = spdRatio;
	}else{
		std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
		RETURNID_VALUE_ERR;
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(ULONG spdRatio)
{
	std::cout<<"SetSpeedJoint"<<std::endl;

	if(spdRatio >= 0 && spdRatio <= 100){
		JointSpeed = spdRatio;
	}else{
		std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
		RETURNID_VALUE_ERR;
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)
{
	std::cout<<"MoveCircularCartesianAbs"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)
{
	std::cout<<"MoveCircularCartesianRel"<<std::endl;
	std::cout<<"ERROR : Not Inplemented"<<std::endl<<std::endl;
	RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM::JointPos& jointPoint)
{
	std::cout<<"SetHome"<<std::endl;

	HomeMotorPosition[0] = (short)(jointPoint[0]*1800.0/M_PI);
	HomeMotorPosition[1] = (short)(jointPoint[1]*1800.0/M_PI);
	HomeMotorPosition[2] = (short)(Angle3_LimitMin+(jointPoint[2]*1000.0)*(Angle3_LimitMax-Angle3_LimitMin)/(Z_LimitMax-Z_LimitMin));
	if(servoNum == 5){
		HomeMotorPosition[3] = (short)(jointPoint[3]*1800.0/M_PI);
		HomeMotorPosition[4] = (short)(Angle5_LimitMin+(jointPoint[4]*1000.0)*(Angle5_LimitMax-Angle5_LimitMin)/(Hand_LimitMax-Hand_LimitMin));
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM::JointPos_out jointPoint)
{
	std::cout<<"GetHome"<<std::endl;

	jointPoint=new JARA_ARM::JointPos;
	jointPoint->length(servoNum);
	(*jointPoint)[0]=HomeMotorPosition[0]*M_PI/1800.0;
	(*jointPoint)[1]=HomeMotorPosition[1]*M_PI/1800.0;
	(*jointPoint)[2]=((Z_LimitMax-Z_LimitMin)*(HomeMotorPosition[2]-Angle3_LimitMin)/(Angle3_LimitMax-Angle3_LimitMin))/1000.0;//[m]
	if(servoNum == 5){
		(*jointPoint)[3]=HomeMotorPosition[3]*M_PI/1800.0;
		(*jointPoint)[4]=((Hand_LimitMax-Hand_LimitMin)*(HomeMotorPosition[4]-Angle5_LimitMin)/(Angle5_LimitMax-Angle5_LimitMin))/1000.0;//[m]
	}

	std::cout<<"Success"<<std::endl<<std::endl;

	RETURNID_OK;
}

JARA_ARM::RETURN_ID* ManipulatorCommonInterface_MiddleSVC_impl::goHome()
{
	std::cout<<"GoHome"<<std::endl;

	hr = RSMove(dev,HomeMotorPosition,MoveTime,1,servoNum);

	if(hr != TRUE){
		std::cout<<"ERROR : RSMove()"<<std::endl<<std::endl;
		RETURNID_NG;
	}else{
		std::cout<<"Success"<<std::endl<<std::endl;
	}

	RETURNID_OK;
}



// End of example implementational code



