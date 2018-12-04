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
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>

#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <hrpsys-state-observation/TiltEstimator.h>

namespace so=stateObservation;

const double alpha_const = 200;
const double beta_const = 5;
const double gamma_const = 15;

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
  m_rpyBRefIn("rpyBRef", m_rpyBRef),
  m_pBRefIn("pBRef", m_pBRef),
  m_tiltOut("tilt", m_tilt),

  // </rtc-template>
  estimator_(alpha_const, beta_const, gamma_const),
  dt_(sampling_time_const),
  firstSample_(true)
{
  estimator_.setSamplingTime(dt_);

  xk_.resize(9);
  xk_ << so::Vector3::Zero(), so::Vector3::Zero(), so::Vector3(0.49198, 0.66976, 0.55622);
  estimator_.setState(xk_, 0);
  std::cout << "Tilt Estimator constructor" << std::endl;
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
  addInPort("rpyBRef", m_rpyBRefIn);
  addInPort("pBRef", m_pBRefIn);

  // Set OutPort buffers
  addOutPort("tilt", m_tiltOut);
  
  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

  estimator_.setSamplingTime(dt_);

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

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

  return RTC::RTC_OK;
}



RTC::ReturnCode_t TiltEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": onDeactivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}



RTC::ReturnCode_t TiltEstimator::onExecute(RTC::UniqueId ec_id)
{
  so::Vector3 ya, yg;
  
  if (m_accIn.isNew()) {
    m_accIn.read();
    ya << m_acc.data.ax, m_acc.data.ay, m_acc.data.az;
  }
  
  if (m_rateIn.isNew()) {
    m_rateIn.read();
    yg << m_rate.data.avx, m_rate.data.avy, m_rate.data.avz;
  }
  
  if (m_rpyBRefIn.isNew()) {

    m_rpyBRefIn.read();
    
    so::Matrix3 R = so::kine::rollPitchYawToRotationMatrix(m_rpyBRef.data.r, m_rpyBRef.data.p, m_rpyBRef.data.y);

    so::Vector3 wBRef;
    if (firstSample_)
      wBRef.setZero();
    else {
      wBRef = so::kine::rotationMatrixToRotationVector(R * m_robot->rootLink()->R.transpose()) / dt_;
    }
    
    m_robot->rootLink()->R = R;
    m_robot->rootLink()->w = wBRef;
  }

  if (m_pBRefIn.isNew()) {

    m_pBRefIn.read();

    so::Vector3 p;
    p << m_pBRef.data.x, m_pBRef.data.y, m_pBRef.data.z;

    so::Vector3 vBRef;
    if (firstSample_)
      vBRef.setZero();
    else
      vBRef = (p - m_robot->rootLink()->p) / dt_;

    m_robot->rootLink()->p = p;
    m_robot->rootLink()->v = vBRef;
  }
  
  if (m_qIn.isNew()) {
    
    m_qIn.read();

    so::Vector dq;
    dq.setZero(m_robot->numJoints());

    for (unsigned i = 0; i < m_robot->numJoints(); i++) {

      hrp::Link* test = m_robot->joint(i);
      test->q;

      if (firstSample_)
        dq[i] = 0;
      else
        dq[i] = (m_q.data[i] - m_robot->joint(i)->q) / dt_;
      
      m_robot->joint(i)->q = m_q.data[i];
      m_robot->joint(i)->dq = dq[i];
    }
  }

  firstSample_ = false;

  m_robot->calcForwardKinematics(true); 

  hrp::Sensor *accel = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
  hrp::Sensor *gyro  = m_robot->sensor(hrp::Sensor::RATE_GYRO, 0);

  so::Vector3 b = gyro->link->R * gyro->localPos;
  
  so::Vector3 p_S_C = gyro->link->p + b;
  so::Matrix3 R_S_C = gyro->link->R * gyro->localR;
  
  so::Vector3 v_S_C = gyro->link->v + gyro->link->w.cross(b);
  so::Vector3 w_S_C = gyro->link->w;

  estimator_.setSensorPositionInC(p_S_C);
  estimator_.setSensorOrientationInC(R_S_C);
  estimator_.setSensorLinearVelocityInC(v_S_C);
  estimator_.setSensorAngularVelocityInC(w_S_C);

  int k = estimator_.getCurrentTime();

  estimator_.setMeasurement(ya, yg, k + 1);
  
  xk_ = estimator_.getEstimatedState(k + 1);

  so::Vector3 tilt = xk_.tail(3);
  
  m_tilt.tm = m_q.tm;
  m_tilt.data.x = tilt[0];
  m_tilt.data.y = tilt[1];
  m_tilt.data.z = tilt[2];

  m_tiltOut.write();
  
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
