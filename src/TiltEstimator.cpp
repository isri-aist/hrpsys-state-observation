// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
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

namespace so = stateObservation;

const double alpha_const = 200;
const double beta_const = 5;
const double gamma_const = 15;

const double sampling_time_const = 0.002;

static const std::string alpha_char = so::tools::toString(alpha_const);
static const std::string beta_char  = so::tools::toString(beta_const);
static const std::string gamma_char = so::tools::toString(gamma_const);

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
  "conf.default.alpha", alpha_char.c_str(),
  "conf.default.beta", beta_char.c_str(),
  "conf.default.gamma", gamma_char.c_str(),
  ""
};
// </rtc-template>



TiltEstimator::TiltEstimator(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
  // <rtc-template block="initializer">
  m_accIn("acc", m_acc),
  m_rateIn("rate", m_rate),
  m_qIn("q", m_q),
  m_rpyBEstIn("rpyBEst", m_rpyBEst),
  m_pBEstIn("pBEst", m_pBEst),
  m_rpyFEstIn("rpyFEst", m_rpyFEst),
  m_pFEstIn("pFEst", m_pFEst),
  m_rpySOut("rpyS", m_rpyS),
  // Aditional data
  m_pSCOut("pSC", m_pSC),
  m_rpySCOut("rpySC", m_rpySC),
  m_vSCOut("vSC", m_vSC),
  m_wSCOut("wSC", m_wSC),
  m_vCOut("vC", m_vC),
  m_xkOut("xk", m_xk),
    
  // </rtc-template>
  m_alpha(alpha_const),
  m_beta(beta_const),
  m_gamma(gamma_const),
  estimator_(m_alpha, m_beta, m_gamma),
  m_pF_prev(so::Vector3::Zero()),
  dt_(sampling_time_const),
  firstSample_(true),
  ya_(so::Vector3::Zero()),
  yg_(so::Vector3::Zero()),
  v_C_(so::Vector3::Zero()),
  p_S_C_(so::Vector3::Zero()),
  R_S_C_(so::Matrix3::Identity()),
  v_S_C_(so::Vector3::Zero()),
  w_S_C_(so::Vector3::Zero())
{
  estimator_.setSamplingTime(dt_);

  xk_.resize(9);
  xk_ << so::Vector3::Zero(), so::Vector3::Zero(), so::Vector3(0, 0, 1); // so::Vector3(0.49198, 0.66976, 0.55622);
  // std::cout << "Rafa, in the constructor of TiltEstimator, xk_ = " << xk_.transpose() << std::endl;
  estimator_.setState(xk_, 0);

  m_xk.data.length(xk_.size());
  
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
  addInPort("rpyBEst", m_rpyBEstIn);
  addInPort("pBEst", m_pBEstIn);
  addInPort("rpyFEst", m_rpyFEstIn);
  addInPort("pFEst", m_pFEstIn);

  // Set OutPort buffers
  addOutPort("rpyS", m_rpySOut);
  // Additional data
  addOutPort("pSC", m_pSCOut);
  addOutPort("rpySC", m_rpySCOut);
  addOutPort("vSC", m_vSCOut);
  addOutPort("wSC", m_wSCOut);
  addOutPort("vC", m_vCOut);
  addOutPort("xk", m_xkOut);
  
  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("alpha", m_alpha, alpha_char.c_str());
  bindParameter("beta",  m_beta,  beta_char.c_str());
  bindParameter("gamma", m_gamma, gamma_char.c_str());

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

  m_acc.data.ax   = m_acc.data.ay   = m_acc.data.az   = 0.0;
  m_rate.data.avx = m_rate.data.avy = m_rate.data.avz = 0.0;

  dof_ = m_robot->numJoints();
  m_q.data.length(dof_);
  
  for (size_t i = 0; i < dof_; i++)
    m_q.data[i] = 0.0;

  m_rpyBEst.data.r = m_rpyBEst.data.p = m_rpyBEst.data.y = 0.0;
  m_pBEst.data.x   = m_pBEst.data.y   = m_pBEst.data.z   = 0.0;

  m_rpyFEst.data.r = m_rpyFEst.data.p = m_rpyFEst.data.y = 0.0;
  m_pFEst.data.x   = m_pFEst.data.y   = m_pFEst.data.z   = 0.0;
  
  return RTC::RTC_OK;
}



RTC::ReturnCode_t TiltEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": onDeactivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}



RTC::ReturnCode_t TiltEstimator::onExecute(RTC::UniqueId ec_id)
{
  estimator_.setAlpha(m_alpha);
  estimator_.setBeta(m_beta);
  estimator_.setGamma(m_gamma);
  
  // so::Vector3 ya  = so::Vector3::Zero();
  // so::Vector3 yg  = so::Vector3::Zero();
  // so::Vector3 v_C = so::Vector3::Zero();

  if (m_accIn.isNew()) {
    m_accIn.read();
    ya_ = so::Vector3(m_acc.data.ax, m_acc.data.ay, m_acc.data.az);
  }
  
  if (m_rateIn.isNew()) {
    m_rateIn.read();
    yg_ = so::Vector3(m_rate.data.avx, m_rate.data.avy, m_rate.data.avz);
  }

  // std::cout << "Rafa, in TiltEstimator::onExecute, ya_ = " << ya_.transpose() << std::endl;
  // std::cout << "Rafa, in TiltEstimator::onExecute, yg_ = " << yg_.transpose() << std::endl;
  
  if (m_rpyBEstIn.isNew() && m_rpyFEstIn.isNew() && m_pBEstIn.isNew() && m_pFEstIn.isNew() && m_qIn.isNew()) {

    m_rpyBEstIn.read();
    m_rpyFEstIn.read();

    so::Matrix3 RB = so::kine::rollPitchYawToRotationMatrix(m_rpyBEst.data.r, m_rpyBEst.data.p, m_rpyBEst.data.y);
    so::Matrix3 RF = so::kine::rollPitchYawToRotationMatrix(m_rpyFEst.data.r, m_rpyFEst.data.p, m_rpyFEst.data.y);
    
    so::Matrix3 R = RF.transpose() * RB;
    
    so::Vector3 w;
    if (firstSample_)
      w.setZero();
    else {
      w = so::kine::rotationMatrixToRotationVector(R * m_robot->rootLink()->R.transpose()) / dt_;
    }
    
    m_robot->rootLink()->R = R;
    m_robot->rootLink()->w = w;

    m_pBEstIn.read();
    m_pFEstIn.read();

    so::Vector3 pB(m_pBEst.data.x, m_pBEst.data.y, m_pBEst.data.z);
    so::Vector3 pF(m_pFEst.data.x, m_pFEst.data.y, m_pFEst.data.z);

    so::Vector3 p = RF.transpose()*(pB - pF);

    so::Vector3 v;
    if (firstSample_)
      v.setZero();
    else
      v = (p - m_robot->rootLink()->p) / dt_;

    m_robot->rootLink()->p = p;
    m_robot->rootLink()->v = v;

    if (firstSample_)
      v_C_.setZero();
    else
      v_C_ = RF.transpose() * (pF - m_pF_prev) / dt_;
    
    m_pF_prev = pF;
    
    m_qIn.read();

    so::Vector dq;
    dq.setZero(dof_);

    for (unsigned i = 0; i < dof_; i++) {

      if (firstSample_)
        dq[i] = 0;
      else
        dq[i] = (m_q.data[i] - m_robot->joint(i)->q) / dt_;
      
      m_robot->joint(i)->q = m_q.data[i];
      m_robot->joint(i)->dq = dq[i];
    }

    m_robot->calcForwardKinematics(true); 
    
    hrp::Sensor *accel = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
    hrp::Sensor *gyro  = m_robot->sensor(hrp::Sensor::RATE_GYRO, 0);
    
    so::Vector3 b = gyro->link->R * gyro->localPos;
    
    p_S_C_ = gyro->link->p + b;
    R_S_C_ = gyro->link->R * gyro->localR;
    
    v_S_C_ = gyro->link->v + gyro->link->w.cross(b);
    w_S_C_ = gyro->link->w;
  }

  firstSample_ = false;

  // std::cout << "Rafa, in TiltEstimator::onExecute, p_S_C_ = " << p_S_C_.transpose() << std::endl;
  // std::cout << "Rafa, in TiltEstimator::onExecute, v_S_C_ = " << v_S_C_.transpose() << std::endl;
  // std::cout << "Rafa, in TiltEstimator::onExecute, w_S_C_ = " << w_S_C_.transpose() << std::endl;
  // std::cout << "Rafa, in TiltEstimator::onExecute, v_C_ = " << v_C_.transpose() << std::endl;
  
  estimator_.setSensorPositionInC(p_S_C_);
  estimator_.setSensorOrientationInC(R_S_C_);
  estimator_.setSensorLinearVelocityInC(v_S_C_);
  estimator_.setSensorAngularVelocityInC(w_S_C_);
  estimator_.setControlOriginVelocityInW(v_C_);

  int k = estimator_.getCurrentTime();

  estimator_.setMeasurement(ya_, yg_, k + 1);
  
  xk_ = estimator_.getEstimatedState(k + 1);

  // std::cout << "Rafa, in TiltEstimator::onExecute, xk_ = " << xk_.transpose() << std::endl << std::endl;

  so::Vector3 tilt = xk_.tail(3);

  so::Vector3 rpyS = so::kine::rotationMatrixToRollPitchYaw(
                      so::kine::mergeTiltWithYaw(
                        tilt, so::kine::rollPitchYawToRotationMatrix(
                                              m_rpyBEst.data.r, m_rpyBEst.data.p, m_rpyBEst.data.y)));

  m_rpyS.data.r = rpyS(0);
  m_rpyS.data.p = rpyS(1);
  m_rpyS.data.y = rpyS(2);

  m_rpyS.tm = m_q.tm;

  m_rpySOut.write();

  // Additional data

  m_pSC.data.x = p_S_C_.x();
  m_pSC.data.y = p_S_C_.y();
  m_pSC.data.z = p_S_C_.z();
  
  m_pSC.tm = m_q.tm;

  m_pSCOut.write();

  so::Vector3 rpySC = so::kine::rotationMatrixToRollPitchYaw(R_S_C_);
  m_rpySC.data.r = rpySC(0);
  m_rpySC.data.p = rpySC(1);
  m_rpySC.data.y = rpySC(2);

  m_rpySC.tm = m_q.tm;

  m_rpySCOut.write();

  m_vSC.data.x = v_S_C_.x();
  m_vSC.data.y = v_S_C_.y();
  m_vSC.data.z = v_S_C_.z();

  m_vSC.tm = m_q.tm;

  m_vSCOut.write();

  m_wSC.data.avx = w_S_C_.x();
  m_wSC.data.avy = w_S_C_.y();
  m_wSC.data.avz = w_S_C_.z();

  m_wSC.tm = m_q.tm;

  m_wSCOut.write();

  m_vC.data.x = v_C_.x();
  m_vC.data.y = v_C_.y();
  m_vC.data.z = v_C_.z();

  m_vC.tm = m_q.tm;

  m_vCOut.write();  

  for (size_t i = 0; i < xk_.size(); i++) {
    m_xk.data[i] = xk_(i);
  }

  m_xk.tm = m_q.tm;

  m_xkOut.write();
  
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
