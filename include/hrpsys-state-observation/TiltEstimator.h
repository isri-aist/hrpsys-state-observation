// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
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
#include <rtm/idl/ExtendedDataTypes.hh>
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
 protected:
  
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  double m_alpha;
  double m_beta;
  double m_gamma;
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">

  TimedAcceleration3D m_acc;
  InPort<TimedAcceleration3D> m_accIn;
  
  TimedAngularVelocity3D m_rate;
  InPort<TimedAngularVelocity3D> m_rateIn;
  
  TimedDoubleSeq m_q;
  InPort<TimedDoubleSeq> m_qIn;

  // The RPY angles for the estimated orientation of the waist ({B})
  TimedOrientation3D m_rpyBEst;
  InPort<TimedOrientation3D> m_rpyBEstIn;

  // The estimated position of the waist ({B})
  TimedPoint3D m_pBEst;
  InPort<TimedPoint3D> m_pBEstIn;

  // The RPY angles for the estimated orientation of the control frame ({F})
  TimedOrientation3D m_rpyFEst;
  InPort<TimedOrientation3D> m_rpyFEstIn;

  // The estimated position of the control frame ({F})
  TimedPoint3D m_pFEst;
  InPort<TimedPoint3D> m_pFEstIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">

  // The observed tilt of the sensor ({S})
  TimedOrientation3D m_rpyS;
  OutPort<TimedOrientation3D> m_rpySOut;

  // Additional data
  TimedPoint3D m_pSC;
  OutPort<TimedPoint3D> m_pSCOut;

  TimedOrientation3D m_rpySC;
  OutPort<TimedOrientation3D> m_rpySCOut;

  TimedVector3D m_vSC;
  OutPort<TimedVector3D> m_vSCOut;

  TimedAngularVelocity3D m_wSC;
  OutPort<TimedAngularVelocity3D> m_wSCOut;

  TimedVector3D m_vC;
  OutPort<TimedVector3D> m_vCOut;

  TimedDoubleSeq m_xk;
  OutPort<TimedDoubleSeq> m_xkOut;
  
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

  stateObservation::Vector3 m_pF_prev;
  double dt_;

  /// Instance of the Tilt Estimator
  stateObservation::TiltEstimator estimator_;
  
  hrp::BodyPtr m_robot;
  size_t dof_;

  stateObservation::Vector xk_;

  bool firstSample_;

  /// Auxiliar variables

  stateObservation::Vector3 ya_;
  stateObservation::Vector3 yg_;
  stateObservation::Vector3 v_C_;
  stateObservation::Vector3 p_S_C_;
  stateObservation::Matrix3 R_S_C_;
  stateObservation::Vector3 v_S_C_;
  stateObservation::Vector3 w_S_C_;
};


extern "C"
{
  void TiltEstimatorInit(RTC::Manager* manager);
}

#endif // TiltEstimator_H
