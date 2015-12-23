// -*- C++ -*-
/*!
 * @file  VS_ASR_RTC.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "VS_ASR_RTC.h"
#include "useSilabs.h"

// Module specification
// <rtc-template block="module_spec">
static const char* vs_asr_rtc_spec[] =
  {
    "implementation_id", "VS_ASR_RTC",
    "type_name",         "VS_ASR_RTC",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "Saitama Univ. Design Lab.",
    "category",          "Industrial Robot",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.servoNum", "5",
    // Widget
    "conf.__widget__.servoNum", "text",
    // Constraints
    "conf.__constraints__.servoNum", "(3,5)",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
VS_ASR_RTC::VS_ASR_RTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_ManipulatorCommonInterface_CommonPort("ManipulatorCommonInterface_Common"),
    m_ManipulatorCommonInterface_MiddlePort("ManipulatorCommonInterface_Middle")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
VS_ASR_RTC::~VS_ASR_RTC()
{
}



RTC::ReturnCode_t VS_ASR_RTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_ManipulatorCommonInterface_CommonPort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_JARA_ARM_ManipulatorCommonInterface_Common);
  m_ManipulatorCommonInterface_MiddlePort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_JARA_ARM_ManipulatorCommonInterface_Middle);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_ManipulatorCommonInterface_CommonPort);
  addPort(m_ManipulatorCommonInterface_MiddlePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("servoNum", m_servoNum, "5");
  
  // </rtc-template>
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VS_ASR_RTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VS_ASR_RTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VS_ASR_RTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t VS_ASR_RTC::onActivated(RTC::UniqueId ec_id)
{
	dev=0;	//通信ハンドル初期化
	DWORD numDevice=0;

	std::cout<<"SCARA Robot Control RTC"<<std::endl<<std::endl;

	std::cout<<"******************************"<<std::endl;
	std::cout<<"           Activate           "<<std::endl;
	std::cout<<"******************************"<<std::endl<<std::endl;

	//コンフィギュレーション値の設定
	std::cout<<"ServoNum : "<<m_servoNum<<std::endl<<std::endl;
	if(m_servoNum == 3 || m_servoNum == 5){
		servoNum = m_servoNum;
	}else{
		std::cout<<"ServoNum Error"<<std::endl<<std::endl;
		return RTC::RTC_ERROR;
	}

	//現在PCに接続されているSCARAロボットの数を取得
	HidUart_GetNumDevices(&numDevice,VID,PID);
	std::cout<<"Device(s) : "<<numDevice<<std::endl<<std::endl;

	//1台も接続されていない場合はERROR状態に遷移
	if(numDevice==0){
		std::cout<<"Device Not Found"<<std::endl<<std::endl;
		return RTC::RTC_ERROR;
	}

	//1台目のロボットに接続
	if(HidUart_Open(&dev,0,VID,PID)!=HID_UART_SUCCESS){
		//接続が正しく行われていない場合はERROR状態に遷移
		std::cout<<"Cannot Open Device"<<std::endl<<std::endl;
		return RTC::RTC_ERROR;
	}else{

		std::cout<<"******************************"<<std::endl;
		std::cout<<"           Execute            "<<std::endl;
		std::cout<<"******************************"<<std::endl<<std::endl;
		
	}
	
	return RTC::RTC_OK;
}


RTC::ReturnCode_t VS_ASR_RTC::onDeactivated(RTC::UniqueId ec_id)
{
	std::cout<<"******************************"<<std::endl;
	std::cout<<"          Deactivate          "<<std::endl;
	std::cout<<"******************************"<<std::endl<<std::endl;

	//サーボOFF
	hr = RSTorqueOnOff(dev,0,1,servoNum);
	//ロボットとの通信を切断
	HidUart_Close(dev);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t VS_ASR_RTC::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VS_ASR_RTC::onAborting(RTC::UniqueId ec_id)
{
	std::cout<<"******************************"<<std::endl;
	std::cout<<"            Error             "<<std::endl;
	std::cout<<"******************************"<<std::endl<<std::endl;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VS_ASR_RTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VS_ASR_RTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VS_ASR_RTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VS_ASR_RTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void VS_ASR_RTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(vs_asr_rtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<VS_ASR_RTC>,
                             RTC::Delete<VS_ASR_RTC>);
  }
  
};


