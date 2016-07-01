// -*- C++ -*-
/*!
 * @file  KineticsObserver.cpp
 * @brief KineticsObserver filter component
 * $Date$
 *
 * $Id$
 */

#include <hrpUtil/Eigen3d.h>
#include <Eigen/Geometry>

#include <hrpsys-state-observation/KineticsObserver.h>
#include <hrpsys-state-observation/VectorConvert.h>

namespace so=stateObservation;

namespace fest=so::flexibilityEstimation;
typedef fest::IMUElasticLocalFrameDynamicalSystem::input input;


const int statesize=18;
const int inputsize=6;

const double acc_cov_const=1e-4;
const double gyr_cov_const=1e-10;
const double ori_acc_const=3e-6;
const double state_cov_const=1e-12;

const double mass=80.;

static const std::string acc_cov_char  =so::tools::toString(acc_cov_const);
static const std::string gyr_cov_char  =so::tools::toString(gyr_cov_const);
static const std::string ori_acc_char  =so::tools::toString(ori_acc_const);
static const std::string state_cov_char=so::tools::toString(state_cov_const);


// Module specification
// <rtc-template block="module_spec">
static const char* KineticsObserver_spec[] =
{
  "implementation_id", "KineticsObserver",
  "type_name",         "KineticsObserver",
  "description",       "Kinetics Observer component",
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
  "conf.default.debugLevel", "0",
  ""
};
// </rtc-template>

//ctor
KineticsObserver::KineticsObserver(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_accIn("acc", m_acc),
    m_accRefIn("accRef", m_accRef),
    m_rateIn("rate", m_rate),
    m_rfforceIn("lfforce",m_rfforce),
    m_lfforceIn("rfforce",m_lfforce),
    m_rpyOut("rpy", m_rpy),

    // </rtc-template>
    dt_(0.005),
    q_(so::Matrix::Identity(stateSize_,stateSize_)*state_cov_const),
    r_(so::Matrix::Identity(measurementSize_,measurementSize_)*acc_cov_const),
    uk_(inputsize),
    xk_(statesize),
    m_compensateMode(true),
    m_acceleroCovariance(acc_cov_const),
    m_gyroCovariance(gyr_cov_const),
    m_orientationAccCov(ori_acc_const),
    m_stateCov(state_cov_const),
    contactNbr_(0)
{

  ///initialization of the extended Kalman filter
  estimator_.setSamplingPeriod(dt_);

  xk_.setZero();
  uk_.setZero();
  estimator_.setFlexibilityGuess(xk_);
  estimator_.setFlexibilityCovariance(q_);
  estimator_.setMass(mass);
}

KineticsObserver::~KineticsObserver()
{
}


RTC::ReturnCode_t KineticsObserver::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("acc", m_accIn);
  addInPort("accRef", m_accRefIn);
  addInPort("rate", m_rateIn);
  addInPort("lfforce", m_lfforceIn);
  addInPort("rfforce", m_rfforceIn);

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
  bindParameter("acc_cov",m_acceleroCovariance, "");
  bindParameter("gyr_cov", m_gyroCovariance, "0.01");
  bindParameter("ori_acc_cov", m_orientationAccCov, "0.01");
  bindParameter("state_cov", m_stateCov, "0.01");
  bindParameter("debugLevel", m_debugLevel, "0");

  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

  m_acc.data.ax = m_acc.data.ay = m_acc.data.az = 0.0;
  m_rate.data.avx = m_rate.data.avy = m_rate.data.avz = 0.0;
  m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;

  return RTC::RTC_OK;
}




RTC::ReturnCode_t KineticsObserver::onFinalize()
{
}


/*
RTC::ReturnCode_t KineticsObserver::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KineticsObserver::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t KineticsObserver::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "KineticsObserver::onActivated(" << ec_id << ")" << std::endl;

//  std::cout << "KineticsObserver: Q = " << m_Q << ", Q_bw = " << m_Qbw
//            << ", R = " << m_R << ", dt = " << m_dt << ", filter order = "
//            << m_filter_order << ", Tgsens = " << m_Tgsens << std::endl;

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KineticsObserver::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t KineticsObserver::onExecute(RTC::UniqueId ec_id)
{

  q_.noalias()=so::Matrix::Identity(stateSize_,stateSize_)*m_stateCov;
  r_.noalias()=so::Matrix::Identity(measurementSize_,measurementSize_)*m_acceleroCovariance;
  q_(9,9)=q_(10,10)=q_(11,11)=m_orientationAccCov;
  r_(3,3)=r_(4,4)=r_(5,5)=m_gyroCovariance;

  estimator_.setProcessNoiseCovariance(q_);
  estimator_.setMeasurementNoiseCovariance(r_);

  coil::TimeValue coiltm(coil::gettimeofday());
  Time tm;
  tm.sec  = coiltm.sec();
  tm.nsec = coiltm.usec() * 1000;

  //std::cout << "AttitudeEstimator::onExecute(" << ec_id << ")" << std::endl;
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

  bool withForce=false;
  contactNbr_=2;

  if (m_lfforceIn.isNew())
  {
    m_lfforceIn.read();

    if (m_rfforceIn.isNew())
    {
      m_rfforceIn.read();
      withForce=true;
      contactNbr_=0;
    }
  }



  so::Vector6 measurement;

  if (withForce)
    measurement.resize(18);
  else
    measurement.resize(6);


  measurement.head<6>() << m_acc.data.ax,   ///------------
                   m_acc.data.ay,   /// accelerometer
                   m_acc.data.az,   ///-------------
                   m_rate.data.avx, ///-------------
                   m_rate.data.avy, /// gyrometer
                   m_rate.data.avz; ///------------
  int measurementIndex=6;

  if (withForce)
  {

    if (m_lfforce.data[2]>mass*0.05)
    {
      ++contactNbr_;
      measurement.segment<6>(measurementIndex)
                       << m_lfforce.data[0],
                          m_lfforce.data[1],
                          m_lfforce.data[2],
                          m_lfforce.data[3],
                          m_lfforce.data[4],
                          m_lfforce.data[5];
      measurementIndex+=6;

    }
    if (m_rfforce.data[2]>mass*0.05)
    {
      ++contactNbr_;
      measurement.segment<6>(measurementIndex)
                       << m_rfforce.data[0],
                          m_rfforce.data[1],
                          m_rfforce.data[2],
                          m_rfforce.data[3],
                          m_rfforce.data[4],
                          m_rfforce.data[5];
      measurementIndex+=6;
    }
  }


  estimator_.setContactsNumber(contactNbr_);


  estimator_.setMeasurement(measurement);

  uk_.resize(estimator_.getInputSize());

  uk_.segment<3> (input::posCom)<<0,0,0.9;
  uk_.segment<3> (input::velCom)<<0,0,0;
  uk_.segment<3> (input::accCom)<<0,0,0;
  uk_.segment<6> (input::inertia)<<217.6,212.,20.,0,0,0;
  uk_.segment<3> (input::angMoment)<<0,0,0;
  uk_.segment<6> (input::dotInertia)<<0,0,0,0,0,0;
  uk_.segment<3> (input::dotAngMoment)<<0,0,0;
  uk_.segment<3> (input::posIMU)<<0,0,1.2;
  uk_.segment<3> (input::oriIMU)<<0,0,0;
  uk_.segment<3> (input::linVelIMU)<<0,0,0;
  uk_.segment<3> (input::angVelIMU)<<0,0,0;
  uk_.segment<3> (input::linAccIMU)<<0,0,0;
  uk_.segment<12>(input::contacts)<<0,+0.19,0,0,0,0,
                                    0,-0.19,0,0,0,0;





  estimator_.setInput(uk_);


  ///get the estimation and give it to the array
  xk_=estimator_.getFlexibilityVector();

  so::Vector3 orientation(xk_.segment<3>(so::kine::ori));

  so::Matrix3 mat(so::kine::rotationVectorToRotationMatrix(orientation));

  so::Vector3 euler(so::kine::rotationMatrixToRollPitchYaw(mat));

  so::Vector3 offset(m_offset[0],m_offset[1],m_offset[2]);

  so::Vector3 output(euler+offset);

  // output to OutPorts
  m_rpy.tm = tm;
  m_rpy.data.r = output[0];
  m_rpy.data.p = output[1];
  m_rpy.data.y = output[2];
  m_rpyOut.write();
  if (m_debugLevel > 0)
  {
    printf("acc:%6.3f %6.3f %6.3f, rate:%6.3f %6.3f %6.3f, rpy:%6.3f %6.3f %6.3f \n",
           measurement[0], measurement[1], measurement[2],
           measurement[3], measurement[4], measurement[5],
           m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);



  }


    sensorLog.pushBack(measurement);
    orientationLog.pushBack(orientation);
    offsetLog.pushBack(offset);
    eulerLog.pushBack(euler);
    myOutLog.pushBack(output);

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t KineticsObserver::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KineticsObserver::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*Ķ
RTC::ReturnCode_t KineticsObserver::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KineticsObserver::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KineticsObserver::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void KineticsObserverInit(RTC::Manager* manager)
  {
    RTC::Properties profile(KineticsObserver_spec);
    manager->registerFactory(profile,
                             RTC::Create<KineticsObserver>,
                             RTC::Delete<KineticsObserver>);
  }

};


