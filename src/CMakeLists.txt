#AttitudeEstimator
set(comp_sources_ae AttitudeEstimator.cpp)
add_library(AttitudeEstimator SHARED ${comp_sources_ae} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(AttitudeEstimator ${Boost_LIBRARIES} ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES}  ${HMC2_LIBRARIES})
set_target_properties(AttitudeEstimator PROPERTIES PREFIX "")

PKG_CONFIG_USE_DEPENDENCY(AttitudeEstimator state-observation)

add_executable(AttitudeEstimatorComp AttitudeEstimatorComp.cpp ${comp_sources_ae} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(AttitudeEstimatorComp ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES} ${HMC2_LIBRARIES})

PKG_CONFIG_USE_DEPENDENCY(AttitudeEstimatorComp state-observation)

#KineticsObserver
set(comp_sources_kinetic KineticsObserver.cpp)
add_library(KineticsObserver SHARED ${comp_sources_kinetic} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(KineticsObserver ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES} ${HMC2_LIBRARIES} )
set_target_properties(KineticsObserver PROPERTIES PREFIX "")

PKG_CONFIG_USE_DEPENDENCY(KineticsObserver state-observation)

add_executable(KineticsObserverComp KineticsObserverComp.cpp ${comp_sources_kinetic} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(KineticsObserverComp ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES} ${HMC2_LIBRARIES})

PKG_CONFIG_USE_DEPENDENCY(KineticsObserverComp state-observation)

#TiltEstimator
if(BUILD_TILT_ESTIMATOR)

  set(comp_sources_tilt TiltEstimator.cpp)
  add_library(TiltEstimator SHARED ${comp_sources_tilt} ${${PROJECT_NAME}_HEADERS})
  target_link_libraries(TiltEstimator ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES})
  set_target_properties(TiltEstimator PROPERTIES PREFIX "")

  PKG_CONFIG_USE_DEPENDENCY(TiltEstimator state-observation)

  add_executable(TiltEstimatorComp TiltEstimatorComp.cpp ${comp_sources_tilt} ${${PROJECT_NAME}_HEADERS})
  target_link_libraries(TiltEstimatorComp ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES})

  PKG_CONFIG_USE_DEPENDENCY(TiltEstimatorComp state-observation)

endif()

set(target KineticsObserver KineticsObserverComp AttitudeEstimator AttitudeEstimatorComp)

if(BUILD_TILT_ESTIMATOR)
  set(target ${target} TiltEstimator TiltEstimatorComp)
endif()

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
