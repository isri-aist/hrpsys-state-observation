// -*- C++ -*-
/*!
 * @file  AttitudeEstimator.cpp
 * @brief AttitudeEstimator filter component
 * $Date$
 *
 * $Id$
 */

#include <hrpUtil/Eigen3d.h>
#include <Eigen/Geometry>

#include <hrpsys-state-observation/AttitudeEstimator.h>
#include <hrpsys-state-observation/VectorConvert.h>

namespace so=stateObservation;

const int statesize=18;
const int inputsize=6;

const double acc_cov_const=1e-4;
const double gyr_cov_const=1e-10;
const double ori_acc_const=1e0;
const double state_cov_const=3e-13;

const double sampling_time_const = 0.002;

static const std::string acc_cov_char  =so::tools::toString(acc_cov_const);
static const std::string gyr_cov_char  =so::tools::toString(gyr_cov_const);
static const std::string ori_acc_char  =so::tools::toString(ori_acc_const);
static const std::string state_cov_char=so::tools::toString(state_cov_const);

static const std::string sampling_time_char=so::tools::toString(sampling_time_const);


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
  "conf.default.acc_cov", acc_cov_char.c_str(),
  "conf.default.gyr_cov", gyr_cov_char.c_str(),
  "conf.default.ori_acc_cov", ori_acc_char.c_str(),
  "conf.default.state_cov", state_cov_char.c_str(),
  "conf.default.debug_level", "0",
  "conf.default.sampling_time", sampling_time_char.c_str(),
  ""
};
// </rtc-template>

AttitudeEstimator::AttitudeEstimator(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_accIn("acc", m_acc),
    m_accRefIn("accRef", m_accRef),
    m_rateIn("rate", m_rate),
    m_rpyOut("rpy", m_rpy),

    // </rtc-template>
    filter_(stateSize_, measurementSize_, inputSize_, false),
    dt_(sampling_time_const),
    q_(so::Matrix::Identity(stateSize_,stateSize_)*state_cov_const),
    r_(so::Matrix::Identity(measurementSize_,measurementSize_)*acc_cov_const),
    uk_(inputsize),
    xk_(statesize),
    m_compensateMode(true),
    m_acceleroCovariance(acc_cov_const),
    m_gyroCovariance(gyr_cov_const),
    m_orientationAccCov(ori_acc_const),
    m_stateCov(state_cov_const)
{
  q_(9,9)=q_(10,10)=q_(11,11)=ori_acc_const;

  r_(3,3)=r_(4,4)=r_(5,5)=gyr_cov_const;

  ///initialization of the extended Kalman filter
  imuFunctor_.setSamplingPeriod(dt_);
  filter_.setFunctor(& imuFunctor_);

  filter_.setQ(q_);
  filter_.setR(r_);
  xk_.setZero();
  uk_.setZero();
  filter_.setState(xk_,0);
  filter_.setStateCovariance(q_);

  Kpt_<<-20,0,0,
       0,-20,0,
       0,0,-20;
  Kdt_<<-10,0,0,
       0,-10,0,
       0,0,-10;
  Kpo_<<-0.0,0,0,
       0,-0.0,0,
       0,0,-10;
  Kdo_<<-0.0,0,0,
       0,-0.0,0,
       0,0,-10;

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
  bindParameter("acc_cov",m_acceleroCovariance, acc_cov_char.c_str());
  bindParameter("gyr_cov", m_gyroCovariance, gyr_cov_char.c_str());
  bindParameter("ori_acc_cov", m_orientationAccCov, ori_acc_char.c_str());
  bindParameter("state_cov", m_stateCov, state_cov_char.c_str());
  bindParameter("debug_level", m_debugLevel, "0");

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

  bindParameter("sampling_time", dt_, prop["dt"].c_str());

  // </rtc-template>



  m_acc.data.ax = m_acc.data.ay = m_acc.data.az = 0.0;
  m_rate.data.avx = m_rate.data.avy = m_rate.data.avz = 0.0;
  m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;

  return RTC::RTC_OK;
}




RTC::ReturnCode_t AttitudeEstimator::onFinalize()
{
  if (log_>0)
  {
    sensorLog_.writeInFile("/tmp/ae-sensor.log");
    stateLog_.writeInFile("/tmp/ae-state.log");
    inputLog_.writeInFile("/tmp/ae-input.log");
    outputLog_.writeInFile("/tmp/ae-output.log");
  }
  return RTC::RTC_OK;
}


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

//  std::cout << "AttitudeEstimator: Q = " << m_Q << ", Q_bw = " << m_Qbw
//            << ", R = " << m_R << ", dt = " << m_dt << ", filter order = "
//            << m_filter_order << ", Tgsens = " << m_Tgsens << std::endl;

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

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
  log_=1;

  imuFunctor_.setSamplingPeriod(dt_);

  if (m_debugLevel>0)
  {
    std::cout << "AttitudeEstimator::onExecute(" << ec_id << ")" << std::endl;
  }

  q_.noalias()=so::Matrix::Identity(stateSize_,stateSize_)*m_stateCov;
  r_.noalias()=so::Matrix::Identity(measurementSize_,measurementSize_)*m_acceleroCovariance;
  q_(9,9)=q_(10,10)=q_(11,11)=m_orientationAccCov;
  r_(3,3)=r_(4,4)=r_(5,5)=m_gyroCovariance;

  filter_.setQ(q_);
  filter_.setR(r_);

  coil::TimeValue coiltm(coil::gettimeofday());
  Time tm;
  tm.sec  = coiltm.sec();
  tm.nsec = coiltm.usec() * 1000;


  // input from InPorts
  if (m_accIn.isNew()) m_accIn.read();
  if (m_rateIn.isNew()) m_rateIn.read();

  if (m_accRefIn.isNew())
  {
    m_accRefIn.read();
  }
  else
  {
    m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
  }

  // processing
  so::Vector6 measurement;
  if (m_compensateMode)
  {
    measurement[0] = m_acc.data.ax - m_accRef.data.ax;
    measurement[1] = m_acc.data.ay - m_accRef.data.ay;
    measurement[2] = m_acc.data.az - m_accRef.data.az;
  }
  else
  {
    measurement[0] = m_acc.data.ax;
    measurement[1] = m_acc.data.ay;
    measurement[2] = m_acc.data.az;
  }

  measurement[3]=m_rate.data.avx;
  measurement[4]=m_rate.data.avy;
  measurement[5]=m_rate.data.avz;

  int time=filter_.getCurrentTime();

  ///damped linear and angular spring
  uk_.head<3>()=Kpt_*xk_.segment<3>(so::kine::pos)
                +Kdt_*xk_.segment<3>(so::kine::linVel);
  uk_.tail<3>()=Kpo_*xk_.segment<3>(so::kine::ori)
                +Kdo_*xk_.segment<3>(so::kine::angVel);

  filter_.setInput(uk_,time);

  filter_.setMeasurement(measurement,time+1);


  ///set the derivation step for the finite difference method
  so::Vector dx=filter_.stateVectorConstant(1)*1e-8;


  so::Matrix a=filter_.getAMatrixFD(dx);
  so::Matrix c= filter_.getCMatrixFD(dx);

  filter_.setA(a);
  filter_.setC(c);


  ///get the estimation and give it to the array
  xk_=filter_.getEstimatedState(time+1);

  so::Vector3 orientation(xk_.segment<3>(so::kine::ori));


  so::Matrix3 mat(so::kine::rotationVectorToRotationMatrix(orientation));

  so::Vector3 euler(so::kine::rotationMatrixToRollPitchYaw(mat));

  so::Vector3 offset(m_offset[0],m_offset[1],m_offset[2]);

  so::Vector3 output(euler+offset);


  if (m_debugLevel > 0)
  {
    printf("acc:%6.3f %6.3f %6.3f, rate:%6.3f %6.3f %6.3f, rpy:%6.3f %6.3f %6.3f \n",
           measurement[0], measurement[1], measurement[2],
           measurement[3], measurement[4], measurement[5],
           m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);
  }

  if (log_)
  {
    outputLog_.pushBack(output);

    sensorLog_.pushBack(measurement);
    stateLog_.pushBack(xk_);
    inputLog_.pushBack(uk_);
  }

  // output to OutPorts
  m_rpy.tm = tm;
  m_rpy.data.r = output[0];
  m_rpy.data.p = output[1];
  m_rpy.data.y = output[2];
  m_rpyOut.write();

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

/*Ķ
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


