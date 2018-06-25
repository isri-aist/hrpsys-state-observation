// -*- C++ -*-
/*!
 * @file  TiltEstimator.h
 * @brief TiltEstimator component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TiltEstimator_H
#define TiltEstimator_H

#include <rtm/CorbaNaming.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>

#include <state-observation/observer/tilt-estimator.hpp>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class TiltEstimator : public RTC::DataFlowComponentBase
{
 public:
  
  TiltEstimator(RTC::Manager* manager);
  virtual ~TiltEstimator();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

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
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

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

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
 protected:
  
  // Configuration variable declaration
  // <rtc-template block="config_declare">

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">

  TimedAcceleration3D m_acc;
  InPort<TimedAcceleration3D> m_accIn;
  TimedAngularVelocity3D m_rate;
  InPort<TimedAngularVelocity3D> m_rateIn;
  
  TimedDoubleSeq m_q;
  InPort<TimedDoubleSeq> m_qIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">

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

  
  double dt_;

  /// Instance of the Tilt Estimator
  stateObservation::TiltEstimator estimator_;
  
  hrp::BodyPtr m_robot;

  stateObservation::Vector q_;
  stateObservation::Vector dq_;

  bool firstSample_;
};


extern "C"
{
  void TiltEstimatorInit(RTC::Manager* manager);
}

#endif // TiltEstimator_H
