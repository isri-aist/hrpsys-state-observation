// -*- C++ -*-
/*!
 * @file  TiltEstimator.cpp
 * @brief TiltEstimator filter component
 * $Date$
 *
 * $Id$
 */

#include <hrpUtil/Eigen3d.h>
#include <Eigen/Geometry>
#include <hrpModel/ModelLoaderUtil.h>

#include <hrpsys-state-observation/TiltEstimator.h>

namespace so=stateObservation;

const double alpha_const = 200;
const double beta_const = 15;

const double sampling_time_const = 0.002;

// Module specification
// <rtc-template block="module_spec">
static const char* TiltEstimator_spec[] =
{
  "implementation_id", "TiltEstimator",
  "type_name",         "TiltEstimator",
  "description",       "Tilt Estimator component",
  "version",           "1.0",
  "vendor",            "AIST",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  // Configuration variables
  ""
};
// </rtc-template>



TiltEstimator::TiltEstimator(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
  // <rtc-template block="initializer">
  m_accIn("acc", m_acc),
  m_rateIn("rate", m_rate),
  m_qIn("q", m_q),

  // </rtc-template>
  estimator_(alpha_const, beta_const),
  dt_(sampling_time_const),
  firstSample_(true)
{
}



TiltEstimator::~TiltEstimator()
{
}



RTC::ReturnCode_t TiltEstimator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">

  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;

  // Set InPort buffers
  addInPort("acc", m_accIn);
  addInPort("rate", m_rateIn);
  addInPort("q", m_qIn);

  // Set OutPort buffers

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

  // </rtc-template>

  // Parameters for CORBA

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  
  int comPos = nameServer.find(",");
  if (comPos < 0) {
    comPos = nameServer.length();
  }
  
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  
  // Parameters for the internal robot model

  m_robot = hrp::BodyPtr(new hrp::Body());
 
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()))) {
    std::cerr << "Failed to load model [" << prop["model"] << "] in "
              << m_profile.instance_name << std::endl;
    return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t TiltEstimator::onFinalize()
{
}
*/

/*
RTC::ReturnCode_t TiltEstimator::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TiltEstimator::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t TiltEstimator::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": onActivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}



RTC::ReturnCode_t TiltEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": onDeactivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}



RTC::ReturnCode_t TiltEstimator::onExecute(RTC::UniqueId ec_id)
{
  
  if (m_qIn.isNew()) {
    m_qIn.read();

    for (unsigned i = 0; i < m_robot->numJoints(); i++) {

      if (firstSample_) {
        dq_[i] = 0;
        firstSample_ = false;
      }
      else
        dq_[i] = (m_q.data[i] - q_[i]) / dt_;
      
      q_[i] = m_q.data[i];
    }

    // m_robot->setPosture(q_, dq_, )

  }
  
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t TiltEstimator::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TiltEstimator::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TiltEstimator::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TiltEstimator::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TiltEstimator::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{

  void TiltEstimatorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(TiltEstimator_spec);
    manager->registerFactory(profile,
                             RTC::Create<TiltEstimator>,
                             RTC::Delete<TiltEstimator>);
  }

};
