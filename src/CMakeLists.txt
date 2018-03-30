#AttitudeEstimator
set(comp_sources_ae AttitudeEstimator.cpp)
add_library(AttitudeEstimator SHARED ${comp_sources_ae} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(AttitudeEstimator ${Boost_LIBRARIES} ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES}  ${HMC2_LIBRARIES})
set_target_properties(AttitudeEstimator PROPERTIES PREFIX "")

PKG_CONFIG_USE_DEPENDENCY(AttitudeEstimator state-observation)

add_executable(AttitudeEstimatorComp AttitudeEstimatorComp.cpp ${comp_sources_ae} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(AttitudeEstimatorComp ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES}  ${HMC2_LIBRARIES})

PKG_CONFIG_USE_DEPENDENCY(AttitudeEstimatorComp state-observation)

#KineticsObserver
set(comp_sources_kinetic KineticsObserver.cpp)
add_library(KineticsObserver SHARED ${comp_sources_kinetic} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(KineticsObserver ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES}  ${HMC2_LIBRARIES} )
set_target_properties(KineticsObserver PROPERTIES PREFIX "")

PKG_CONFIG_USE_DEPENDENCY(KineticsObserver state-observation)

add_executable(KineticsObserverComp KineticsObserverComp.cpp ${comp_sources_kinetic} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(KineticsObserverComp ${Boost_LIBRARIES} ${OPENHRP_LIBRARIES}  ${HMC2_LIBRARIES})

PKG_CONFIG_USE_DEPENDENCY(KineticsObserverComp state-observation)

set(target KineticsObserver KineticsObserverComp AttitudeEstimator AttitudeEstimatorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
