// -*- C++ -*-
/*!
 * @file  KineticsObserver.h
 * @brief KineticsObserver component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef KineticsObserver_H
#define KineticsObserver_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <state-observation/flexibility-estimation/model-base-ekf-flex-estimator-imu.hpp>
// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class KineticsObserver
  : public RTC::DataFlowComponentBase
{
 public:
  KineticsObserver(RTC::Manager* manager);
  virtual ~KineticsObserver();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
 virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  bool m_compensateMode;
  double m_acceleroCovariance;
  double m_gyroCovariance;
  double m_orientationAccCov;
  double m_stateCov;
  std::vector<double> m_offset;
  bool m_debugLevel;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_rfforce;
  InPort<TimedDoubleSeq> m_rfforceIn;
  TimedDoubleSeq m_lfforce;
  InPort<TimedDoubleSeq> m_lfforceIn;

  TimedAcceleration3D m_acc;
  InPort<TimedAcceleration3D> m_accIn;
  TimedAcceleration3D m_accRef;
  InPort<TimedAcceleration3D> m_accRefIn;
  TimedAngularVelocity3D m_rate;
  InPort<TimedAngularVelocity3D> m_rateIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedOrientation3D m_rpy;
  OutPort<TimedOrientation3D> m_rpyOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>


  ///Sizes of the states for the state, the measurement, and the input vector
  const unsigned stateSize_=18;
  const unsigned measurementSize_=6;
  const unsigned inputSize_=6;

  double dt_;

  ///initalization of the functor
  stateObservation::flexibilityEstimation::
          ModelBaseEKFFlexEstimatorIMU estimator_;

  stateObservation::Vector xk_;
  stateObservation::Vector uk_;

  stateObservation::Matrix q_;
  stateObservation::Matrix r_;

  stateObservation::IndexedMatrixArray sensorLog;
  stateObservation::IndexedMatrixArray stateLog;
  stateObservation::IndexedMatrixArray inputLog;


  int contactNbr_;

};


extern "C"
{
  void KineticsObserverInit(RTC::Manager* manager);
};

#endif // KineticsObserver_H
