set(comp_sources AttitudeEstimator.cpp)
add_library(AttitudeEstimator SHARED ${comp_sources} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(AttitudeEstimator ${OPENHRP_LIBRARIES} )
set_target_properties(AttitudeEstimator PROPERTIES PREFIX "")

PKG_CONFIG_USE_DEPENDENCY(AttitudeEstimator state-observation)

add_executable(AttitudeEstimatorComp AttitudeEstimatorComp.cpp ${comp_sources} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(AttitudeEstimatorComp ${OPENHRP_LIBRARIES})

PKG_CONFIG_USE_DEPENDENCY(AttitudeEstimatorComp state-observation)

set(target AttitudeEstimator AttitudeEstimatorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
