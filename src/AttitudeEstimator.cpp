// -*- C++ -*-
/*!
 * @file  AttitudeEstimator.cpp
 * @brief AttitudeEstimator filter component
 * $Date$
 *
 * $Id$
 */

#include <hrpsys-state-observation/AttitudeEstimator.h>
#include <hrpsys-state-observation/VectorConvert.h>
#include <hrpUtil/Eigen3d.h>

// Module specification
// <rtc-template block="module_spec">
static const char* AttitudeEstimator_spec[] =
  {
    "implementation_id", "AttitudeEstimator",
    "type_name",         "AttitudeEstimator",
    "description",       "Attitude Estimator component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.compensateMode", "1",
    "conf.default.offset", "0,0,0",
    "conf.default.Q", "0.01",
    "conf.default.Qbw", "0.001",
    "conf.default.R", "0.1",
    "conf.default.Tgsens", "0.05",
    "conf.default.filter_order", "0",
    "conf.default.debugLevel", "0",

    ""
  };
// </rtc-template>

AttitudeEstimator::AttitudeEstimator(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_UseGsensFilter(false),
    // <rtc-template block="initializer">
    m_accIn("acc", m_acc),
    m_accRefIn("accRef", m_accRef),
    m_rateIn("rate", m_rate),
    m_rpyOut("rpy", m_rpy),

    // </rtc-template>
	dummy(0)
{
}

AttitudeEstimator::~AttitudeEstimator()
{

}


RTC::ReturnCode_t AttitudeEstimator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("acc", m_accIn);
  addInPort("accRef", m_accRefIn);
  addInPort("rate", m_rateIn);

  // Set OutPort buffer
  addOutPort("rpy", m_rpyOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("compensateMode", m_compensateMode, "1");
  bindParameter("offset", m_offset, "0,0,0");
  bindParameter("Q", m_Q, "0.01");
  bindParameter("Qbw", m_Qbw, "0.001");
  bindParameter("R", m_R, "0.1");
  bindParameter("Tgsens", m_Tgsens, "0.05");
  bindParameter("filter_order", m_filter_order, "0");
  bindParameter("debugLevel", m_debugLevel, "0");

  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());



  m_acc.data.ax = m_acc.data.ay = m_acc.data.az = 0.0;
  m_rate.data.avx = m_rate.data.avy = m_rate.data.avz = 0.0;
  m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t AttitudeEstimator::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AttitudeEstimator::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AttitudeEstimator::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AttitudeEstimator::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "AttitudeEstimator::onActivated(" << ec_id << ")" << std::endl;

  std::cout << "AttitudeEstimator: Q = " << m_Q << ", Q_bw = " << m_Qbw
            << ", R = " << m_R << ", dt = " << m_dt << ", filter order = "
	    << m_filter_order << ", Tgsens = " << m_Tgsens << std::endl;

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AttitudeEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t AttitudeEstimator::onExecute(RTC::UniqueId ec_id)
{
  coil::TimeValue coiltm(coil::gettimeofday());
  Time tm;
  tm.sec  = coiltm.sec();
  tm.nsec = coiltm.usec() * 1000;

  //std::cout << "AttitudeEstimator::onExecute(" << ec_id << ")" << std::endl;
  // input from InPorts
  if (m_accIn.isNew()) m_accIn.read();
  if (m_rateIn.isNew()) m_rateIn.read();

  if (m_accRefIn.isNew()){
      m_accRefIn.read();
  }else{
      m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
  }

  // processing
  double acc[3];
  if (m_compensateMode){
      acc[0] = m_acc.data.ax - m_accRef.data.ax;
      acc[1] = m_acc.data.ay - m_accRef.data.ay;
      acc[2] = m_acc.data.az - m_accRef.data.az;
  }else{
      acc[0] = m_acc.data.ax;
      acc[1] = m_acc.data.ay;
      acc[2] = m_acc.data.az;
  }

  bool ret = true;
  if (!m_UseGsensFilter){
      }else{
    // UseGsensFilter
    // by s.kajita 2010 Jan.7
  }

  // output to OutPorts
  m_rpy.tm = tm;
  m_rpy.data.r = 0;
  m_rpy.data.p = 0;
  m_rpy.data.y = 0;
  m_rpyOut.write();
  if (m_debugLevel > 0){
    printf("acc:%6.3f %6.3f %6.3f, rate:%6.3f %6.3f %6.3f, rpy:%6.3f %6.3f %6.3f \n",
	   acc[0], acc[1], acc[2],
	   m_rate.data.avx, m_rate.data.avy, m_rate.data.avz,
	   m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);
  }

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t AttitudeEstimator::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AttitudeEstimator::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AttitudeEstimator::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AttitudeEstimator::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AttitudeEstimator::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void AttitudeEstimatorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(AttitudeEstimator_spec);
    manager->registerFactory(profile,
                             RTC::Create<AttitudeEstimator>,
                             RTC::Delete<AttitudeEstimator>);
  }

};


