// -*- C++ -*-
/*!
 * @file  KineticsObserver.cpp
 * @brief KineticsObserver filter component
 * $Date$
 *
 * $Id$
 */
#include <hrpsys-state-observation/KineticsObserver.h>
#include <hrpUtil/Eigen3d.h>
#include <Eigen/Geometry>

#include <rtm/CorbaNaming.h>
#include <hrpModel/Sensor.h>

using namespace motion_generator;


#include <hrpsys-state-observation/VectorConvert.h>

namespace so=stateObservation;

namespace fest=so::flexibilityEstimation;
typedef fest::IMUElasticLocalFrameDynamicalSystem::state state;
typedef fest::IMUElasticLocalFrameDynamicalSystem::input Input;
typedef so::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::contactModel contactModel;


const int inputsize=6;

const double acc_cov_const=1e-4;
const double gyr_cov_const=1e-10;
const double state_fc_const=1e-0;
const double state_cov_const=1e-12;

const double mass=80.;

const double dt_const=0.002;

static const std::string acc_cov_char  =so::tools::toString(acc_cov_const);
static const std::string gyr_cov_char  =so::tools::toString(gyr_cov_const);
static const std::string state_fc_char  =so::tools::toString(state_fc_const);
static const std::string state_cov_char=so::tools::toString(state_cov_const);
typedef so::kine::indexes<so::kine::rotationVector> indexes;


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
  "conf.default.ori_acc_cov", state_fc_char.c_str(),
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
    m_qIn("q", m_q),
    m_qEncoderIn("qEncoder",m_qEncoder),
    m_pRefIn("pRef", m_pRef),
    m_rpyRefIn("rpyRef", m_rpyRef),

    // </rtc-template>

    dt_(dt_const),
    Q_(so::Matrix::Identity(stateSize_,stateSize_)*state_cov_const),
    R_(so::Matrix::Identity(measurementSize_,measurementSize_)*acc_cov_const),
    uk_(inputsize),
    xk_(stateSize_),
    m_compensateMode(true),
    m_acceleroCovariance(acc_cov_const),
    m_gyroCovariance(gyr_cov_const),
    m_stateForceCov(state_fc_const),
    m_stateCov(state_cov_const),
    contactNbr_(0),
    firstPostureSample_(true)
{

  ///initialization of the extended Kalman filter
  estimator_.setSamplingPeriod(dt_);

  xk_.setZero();
  uk_.setZero();
  estimator_.setFlexibilityGuess(xk_);
  estimator_.setFlexibilityCovariance(Q_);
  estimator_.setRobotMass(mass);

  logger_.setPath("/home/benallegue/tmp/");

  pRef_.resize(3);
  pRef_.setZero();
  oriRef_.resize(3);
  oriRef_.setZero();


  if (m_debugLevel>0)
  {
    logger_.record(yk_,"ko-sensor.log");
    logger_.record(xk_,"ko-state.log");
    logger_.record(output_,"ko-output.log");
    logger_.record(uk_,"ko-input.log");
    logger_.record(contactNbr_,"ko-contactNbr.log");
    logger_.record(pRef_,"ko-position-ref.log");
    logger_.record(oriRef_,"ko-orientation-ref.log");
    logger_.record(q_,"ko-posture.log");
    logger_.record(qEncoder_,"ko-encoder.log");
    logger_.record(dq_,"ko-derivative.log");
  }
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

  addInPort("q", m_qIn);
  addInPort("qEncoder",m_qEncoderIn);

  addInPort("pRef", m_pRefIn);
  addInPort("rpyRef", m_rpyRefIn);

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
  bindParameter("ori_acc_cov", m_stateForceCov, "0.01");
  bindParameter("state_cov", m_stateCov, "0.01");
  bindParameter("debugLevel", m_debugLevel, "0");

  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt_, prop["dt"].c_str());

  estimator_.setSamplingPeriod(dt_);

  m_acc.data.ax = m_acc.data.ay = m_acc.data.az = 0.0;
  m_rate.data.avx = m_rate.data.avy = m_rate.data.avz = 0.0;
  m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;

  // load robot model
  m_body = HumanoidBodyPtr(new HumanoidBody());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  RTC::CorbaNaming naming(rtcManager.getORB(), "localhost:2809");
  CORBA::Object_ptr ml = naming.resolve("ModelLoader");
  if (!CORBA::is_nil(ml))
  {
    std::cout << "found ModelLoader on localhost:2809" << std::endl;
  }
  else
  {
    int comPos = nameServer.find(",");
    if (comPos < 0)
    {
      comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    naming.init(nameServer.c_str());
  }

  if (!loadHumanoidBodyFromModelLoader(m_body, prop["model"].c_str(),
                                       CosNaming::NamingContext::_duplicate(naming.getRootContext()), true))
  {
    std::cerr << "failed to load model[" << prop["model"] << "]"
              << std::endl;
  }

  m_body->getPosture(q_);
  qEncoder_=q_;
  dq_=0*q_;




  std::cout << "m_body FIRST " << m_body<< std::endl;

  return RTC::RTC_OK;
}




RTC::ReturnCode_t KineticsObserver::onFinalize()
{
  if (m_debugLevel>0)
  {
    logger_.save();
  }
  return RTC::RTC_OK;
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
  m_debugLevel=1;


  if (m_debugLevel > 0)
  {
    std::cout << "KineticsObserver::onExecute(" << ec_id << ")" << std::endl;
  }



  R_.noalias()=so::Matrix::Identity(measurementSize_,measurementSize_)*m_acceleroCovariance;
  R_(3,3)=R_(4,4)=R_(5,5)=m_gyroCovariance;
  Q_.noalias()=so::Matrix::Identity(stateSize_,stateSize_)*m_stateCov;
  Q_(state::fc,state::fc)
    =Q_(state::fc+1,state::fc+1)
     =Q_(state::fc+2,state::fc+2)
      =Q_(state::fc+3,state::fc+3)
       =Q_(state::fc+4,state::fc+4)
        =Q_(state::fc+5,state::fc+5)
         =Q_(state::fc+6,state::fc+6)
          =Q_(state::fc+7,state::fc+7)
           =Q_(state::fc+8,state::fc+8)
            =Q_(state::fc+9,state::fc+9)
             =Q_(state::fc+10,state::fc+10)
              =Q_(state::fc+11,state::fc+11)
               =m_stateForceCov;

  estimator_.setProcessNoiseCovariance(Q_);
  estimator_.setMeasurementNoiseCovariance(R_);

  estimator_.setForceVariance(1e1);

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

  bool withForce=false;
  contactNbr_=2;


  if (m_lfforceIn.isNew())
  {
    m_lfforceIn.read();

    if (m_rfforceIn.isNew())
    {
      m_rfforceIn.read();
      withForce=true;
    }
  }

  estimator_.setWithForcesMeasurements(withForce);
  estimator_.setContactModel(contactModel::elasticContact);

  if (withForce)
    yk_.resize(18);
  else
    yk_.resize(6);


  yk_.head<6>() << m_acc.data.ax,   ///------------
           m_acc.data.ay,   /// accelerometer
           m_acc.data.az,   ///-------------
           m_rate.data.avx, ///-------------
           m_rate.data.avy, /// gyrometer
           m_rate.data.avz; ///------------
  int measurementIndex=6;

  if (withForce)
  {
    contactNbr_=0;
    if (m_lfforce.data[2]>mass*9.8*0.01)
    {
      ++contactNbr_;
      yk_.segment<6>(measurementIndex)
          << m_lfforce.data[0],
          m_lfforce.data[1],
          m_lfforce.data[2],
          m_lfforce.data[3],
          m_lfforce.data[4],
          m_lfforce.data[5];
      measurementIndex+=6;

    }
    if (m_rfforce.data[2]>mass*9.8*0.01)
    {
      ++contactNbr_;
      yk_.segment<6>(measurementIndex)
          << m_rfforce.data[0],
          m_rfforce.data[1],
          m_rfforce.data[2],
          m_rfforce.data[3],
          m_rfforce.data[4],
          m_rfforce.data[5];
      measurementIndex+=6;
      rightFootIn_=true;
    }
    else
      rightFootIn_=false;
  }


  estimator_.setContactsNumber(contactNbr_);
  yk_.conservativeResize(estimator_.getMeasurementSize());



  estimator_.setMeasurement(yk_);

  uk_.resize(estimator_.getInputSize());

  uk_.segment<3> (Input::posCom)<<0,0,0.9;
  uk_.segment<3> (Input::velCom)<<0,0,0;
  uk_.segment<3> (Input::accCom)<<0,0,0;
  uk_.segment<6> (Input::inertia)<<217.6,212.,20.,0,0,0;
  uk_.segment<3> (Input::angMoment)<<0,0,0;
  uk_.segment<6> (Input::dotInertia)<<0,0,0,0,0,0;
  uk_.segment<3> (Input::dotAngMoment)<<0,0,0;
  uk_.segment<3> (Input::posIMU)<<0,0,1.2;
  uk_.segment<3> (Input::oriIMU)<<0,0,0;
  uk_.segment<3> (Input::linVelIMU)<<0,0,0;
  uk_.segment<3> (Input::angVelIMU)<<0,0,0;
  uk_.segment<3> (Input::linAccIMU)<<0,0,0;

  if (contactNbr_>0)
  {
    if (rightFootIn_)
      uk_.segment<12>(Input::contacts   )<<0,-0.19,0,0,0,0,0,0,0,0,0,0;
    else
      uk_.segment<12>(Input::contacts   )<<0,-0.19,0,0,0,0,0,0,0,0,0,0;
    if (contactNbr_>1)
      uk_.segment<12>(Input::contacts+12)<<0,-0.19,0,0,0,0,0,0,0,0,0,0;
  }

  hrp::Matrix33 sensorR;

  if (m_pRefIn.isNew())
  {
    m_pRefIn.read();
    pRef_[0]=m_pRef.data.x;
    pRef_[1]=m_pRef.data.y;
    pRef_[2]=m_pRef.data.z;
    std::cout << "PRef " << pRef_.transpose()<< std::endl;
  }

  if (m_rpyRefIn.isNew())
  {
    m_rpyRefIn.read();
    oriRef_[0]=m_rpyRef.data.r;
    oriRef_[1]=m_rpyRef.data.p;
    oriRef_[2]=m_rpyRef.data.y;

    std::cout << "rpyRef " << oriRef_.transpose() << std::endl;
  }

  if (m_qEncoderIn.isNew())
  {
    m_qEncoderIn.read();

    for (unsigned int i=0; i<m_body->numJoints(); i++)
    {
      qEncoder_[i] = m_qEncoder.data[i];
    }
  }


  if (m_qIn.isNew())
  {
    m_qIn.read();



    for (unsigned int i=0; i<m_body->numJoints(); i++)
    {
      if (firstPostureSample_)
      {
        dq_[i]=0;
      }
      else
      {
        dq_[i] = (m_q.data[i] - q_[i])/dt_;
      }
      firstPostureSample_=false;
      q_[i] = m_q.data[i];
    }

    m_body->setPosture(q_, dq_);
    m_body->calcForwardKinematics(true);



    //m_body->

    estimator_.setRobotMass(m_body->totalMass());



    // Position and orientation of the IMU.
    hrp::Sensor *gyro = m_body->sensor(hrp::Sensor::RATE_GYRO, 0);
    sensorR = gyro->link->attitude()*gyro->localR;
    hrp::Vector3 sensorP = gyro->link->p + sensorR*gyro->localPos;
    //std::cout << "gyro position = " << sensorP.transpose() << std::endl;
    //std::cout << "gyro orientation = " << sensorR << std::endl;

    uk_.segment<3> (Input::posIMU)<<sensorP;
    uk_.segment<3> (Input::oriIMU)<<so::kine::rotationMatrixToRotationVector(sensorR);
    uk_.segment<3> (Input::linVelIMU)<<0,0,0;
    uk_.segment<3> (Input::angVelIMU)<<0,0,0;
    uk_.segment<3> (Input::linAccIMU)<<0,0,0;

    // The angular and linear momenta
    hrp::Vector3 P, L;
    m_body->calcTotalMomentum(P, L);
    //std::cout << "angular momentum = " << L.transpose() << std::endl;

    // Position of the CoM.
    uk_.segment<3> (Input::posCom)=m_body->calcCM();
    uk_.segment<3> (Input::velCom)=L/m_body->totalMass(); //velocity of the CoM
    std::cout << "CoM = " << uk_.segment<3>(Input::posCom).transpose() << std::endl;
    //std::cout << "q_ " << m_qOld.head<6>().transpose() << std::endl;


    // Position of the end effectors.
    hrp::Link *rhand = m_body->wristLink[0];
    hrp::Link *lhand = m_body->wristLink[1];
    hrp::Link *rfoot = m_body->ankleLink[0];
    hrp::Link *lfoot = m_body->ankleLink[1];
    hrp::Link *waistP = m_body->waistPjoint;




    if (contactNbr_>0)
    {
      if (rightFootIn_)
        uk_.segment<12>(Input::contacts)<<rfoot->p,so::kine::rotationMatrixToRotationVector(rfoot->attitude()),0,0,0,0,0,0;
      else
        uk_.segment<12>(Input::contacts)<<lfoot->p,so::kine::rotationMatrixToRotationVector(lfoot->attitude()),0,0,0,0,0,0;
      if (contactNbr_>1)
        uk_.segment<12>(Input::contacts+12)<<rfoot->p,so::kine::rotationMatrixToRotationVector(rfoot->attitude()),0,0,0,0,0,0;
    }

    std::cout << "right foot position = " << rfoot->p.transpose() << " Orientation "  << so::kine::rotationMatrixToRotationVector(rfoot->attitude()).transpose() << std::endl;
    std::cout << "left  foot position = " << lfoot->p.transpose() << " Orientation "  << so::kine::rotationMatrixToRotationVector(lfoot->attitude()).transpose() << std::endl;
    //std::cout << "right hand orientation = " << rhand->attitude() << std::endl;

    std::cout << "waist " << waistP->p.transpose() << std::endl;

    // The matrix of inertia
    hrp::dmatrix M;
    m_body->calcMassMatrix(M);
    //std::cout << "mass matrix = " << M << std::endl;

    // Mass of the robot
    //std::cout << "mass = " << m_body->totalMass() << "[kg]" << std::endl;
  }
  else
  {
    firstPostureSample_=true;
  }







  estimator_.setMeasurementInput(uk_);

  ///get the estimation and give it to the array
  xk_=estimator_.getFlexibilityVector();

  so::Vector3 orientation(xk_.segment<3>(indexes::ori));

  so::Matrix3 mat(so::kine::rotationVectorToRotationMatrix(orientation)*sensorR);


  so::Vector3 euler(so::kine::rotationMatrixToRollPitchYaw(mat));

  so::Vector3 offset(m_offset[0],m_offset[1],m_offset[2]);

  output_.noalias()=euler+offset;



  // output to OutPorts
  m_rpy.tm = tm;
  m_rpy.data.r = output_[0];
  m_rpy.data.p = output_[1];
  m_rpy.data.y = output_[2];
  m_rpyOut.write();


  if (m_debugLevel > 2)
  {
    printf("acc:%6.3f %6.3f %6.3f, rate:%6.3f %6.3f %6.3f, rpy:%6.3f %6.3f %6.3f \n",
           yk_[0], yk_[1], yk_[2],
           yk_[3], yk_[4], yk_[5],
           m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);


    std::cout << "Contact Number " << contactNbr_ << " withForce "<< withForce << std::endl;
    std::cout << "U " << uk_.transpose()<< std::endl;
    std::cout << "K" << std::endl << estimator_.getEKF().getLastGain() << std::endl;
  }

  if (m_debugLevel > 0)
  {
    logger_.push();


  }


  std::cout << "result" << output_.transpose()<<std::endl;



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


