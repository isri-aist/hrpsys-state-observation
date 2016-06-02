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


const int statesize=18;
const int inputsize=6;



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
	dummy(0),
  filter_(stateSize_, measurementSize_, inputSize_, false),
  dt_(0.005),
  q_(stateObservation::Matrix::Identity(stateSize_,stateSize_)*1e-12),
  r_(stateObservation::Matrix::Identity(measurementSize_,measurementSize_)*1e-4),
  uk_(inputsize),
  xk_(statesize)
{
    q_(9,9)=q_(10,10)=q_(11,11)=1e-6;

    r_(3,3)=r_(4,4)=r_(5,5)=1e-10;

    ///initialization of the extended Kalman filter
    imuFunctor_.setSamplingPeriod(dt_);
    filter_.setFunctor(& imuFunctor_);

    filter_.setQ(q_);
    filter_.setR(r_);
    xk_.setZero();
    uk_.setZero();
    filter_.setState(xk_,0);
    filter_.setStateCovariance(q_);

    Kpt_<<-10,0,0,
           0,-10,0,
           0,0,-10;
    Kdt_<<-1,0,0,
           0,-1,0,
           0,0,-1;
    Kpo_<<-0.01,0,0,
           0,-0.01,0,
           0,0,-10;
    Kdo_<<-0.01,0,0,
           0,-0.01,0,
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




RTC::ReturnCode_t AttitudeEstimator::onFinalize()
{
  sensorLog.writeInFile("/home/benallegue/tmp/sensor.log");
  orientationLog.writeInFile("/home/benallegue/tmp/ori.log");
  eulerLog.writeInFile("/home/benallegue/tmp/euler.log");
  offsetLog.writeInFile("/home/benallegue/tmp/offset.log");
  myOutLog.writeInFile("/home/benallegue/tmp/myout.log");
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
  stateObservation::Vector6 measurement;
  if (m_compensateMode){
      measurement[0] = m_acc.data.ax - m_accRef.data.ax;
      measurement[1] = m_acc.data.ay - m_accRef.data.ay;
      measurement[2] = m_acc.data.az - m_accRef.data.az;
  }else{
      measurement[0] = m_acc.data.ax;
      measurement[1] = m_acc.data.ay;
      measurement[2] = m_acc.data.az;
  }

  measurement[3]=m_rate.data.avx;
  measurement[5]=m_rate.data.avz;
  measurement[4]=m_rate.data.avy;

  int time=filter_.getCurrentTime();

  ///damped linear and angular spring
  uk_.head<3>()=Kpt_*xk_.segment<3>(stateObservation::kine::pos)
              +Kdt_*xk_.segment<3>(stateObservation::kine::linVel);
  uk_.tail<3>()=Kpo_*xk_.segment<3>(stateObservation::kine::ori)
              +Kdo_*xk_.segment<3>(stateObservation::kine::angVel);



  filter_.setInput(uk_,time);

  filter_.setMeasurement(measurement,time+1);


  ///set the derivation step for the finite difference method
  stateObservation::Vector dx=filter_.stateVectorConstant(1)*1e-8;


  stateObservation::Matrix a=filter_.getAMatrixFD(dx);
  stateObservation::Matrix c= filter_.getCMatrixFD(dx);

  filter_.setA(a);
  filter_.setC(c);


  ///get the estimation and give it to the array
  xk_=filter_.getEstimatedState(time+1);

  stateObservation::Vector3 orientation(xk_.segment<3>(stateObservation::kine::ori));


  stateObservation::Matrix3 mat(
    stateObservation::kine::rotationVectorToRotationMatrix(orientation));

  stateObservation::Vector3 euler(
    stateObservation::kine::rotationMatrixToRollPitchYaw(mat));

  std::cout<< orientation.transpose() << "    "<<euler.transpose() << std::endl;

  stateObservation::Vector3 offset(m_offset[0],m_offset[1],m_offset[2]);

  stateObservation::Vector3 output(euler+offset);




  // output to OutPorts
  m_rpy.tm = tm;
  m_rpy.data.r = output[0];
  m_rpy.data.p = output[1];
  m_rpy.data.y = output[2];
  m_rpyOut.write();
  if (m_debugLevel > 0){
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


