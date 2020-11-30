
# Copyright 2016, Mehdi Benallegue, AIST
#
# This file is part of hrpsys-state-observation.
# 

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/boost.cmake)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

SET(PROJECT_NAME hrpsys-state-observation)
SET(PROJECT_DESCRIPTION "Robot's state observation integration in hrpsys-environment")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

#set compilation options to c++11
set(CMAKE_CXX_STANDARD 11)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

SET(BOOST_COMPONENTS filesystem system thread program_options unit_test_framework timer)
SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("state-observation")


# OpenRTM
set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
if(UNIX)
  if(NOT OPENRTM_DIR)
    pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
  endif()
elseif(MSVC)
  if(NOT OPENRTM_DIR)
    message(FATAL_ERROR "Please specify the directory of OpenRTM-aist to OPENRTM_DIR.")
  endif()
endif()

if(OPENRTM_DIR)

  set(OPENRTM_PREFIX ${OPENRTM_DIR})

  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include 
    ${OPENRTM_DIR}/include/coil-1.1
    ${OPENRTM_DIR}/include/openrtm-1.1 
    ${OPENRTM_DIR}/include/openrtm-1.1/rtm/idl
    ${OPENRTM_DIR}/rtm # for the Windows installer version
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    ${OPENRTM_DIR}
    )

  if(MSVC)
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/bin)
  else()
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/lib)
  endif()

  if(UNIX)
    set(OPENRTM_LIBRARIES RTC coil)
  elseif(MSVC)
    file(GLOB librtc RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/RTC???.lib")
    get_filename_component(librtc ${librtc} NAME_WE)
    file(GLOB libcoil RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/coil???.lib")
    get_filename_component(libcoil ${libcoil} NAME_WE)
    set(OPENRTM_LIBRARIES 
      optimized ${librtc} debug ${librtc}d
      optimized ${libcoil} debug ${libcoil}d)
    message(STATUS "bin dir = ${OPENRTM_DIR}/bin, lib dir = ${OPENRTM_LIBRARY_DIRS}, rtm libs = ${librtc}, ${libcoil}")
    install_external_libraries(${OPENRTM_DIR}/bin ${OPENRTM_LIBRARY_DIRS} ${librtc} ${libcoil})
  endif()
endif()

# Make the following variables accessible from other directories
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS} CACHE INTERNAL "OpenRTM's include directories")
set(OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS} CACHE INTERNAL "OpenRTM's library directories")
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} CACHE INTERNAL "OpenRTM's library files")

#add_definitions(${OPENRTM_CFLAGS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS} ${HRPSYSBASE_PREFIX}/lib)


# OpenHRP
find_package(OpenHRP REQUIRED)

include_directories(
  ${OPENHRP_DIR}/include/OpenHRP-3.1
  ${OPENHRP_DIR}/include/OpenHRP-3.1/hrpCorba
  )
 
include_directories(${CMAKE_BINARY_DIR}/idl)

pkg_check_modules(HRPSYSBASE hrpsys-base)
include_directories(
  ${HRPSYSBASE_PREFIX}/include
  ${HRPSYSBASE_PREFIX}/include/hrpsys
  ${HRPSYSBASE_PREFIX}/include/hrpsys/idl
  )
link_directories(${HRPSYSBASE_LIBRARY_DIRS})


pkg_check_modules(HMC2 hmc2)
include_directories(
  ${HMC2_PREFIX}/include/hmc2
  ${HMC2_PREFIX}/include/hmc2/Model
  )
link_directories(${HMC2_LIBRARY_DIRS})

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

set(CPACK_PACKAGE_NAME hrpsys-private)
set(CPACK_PACKAGE_VENDOR "AIST")
set(CPACK_PACKAGE_CONTACT "Fumio Kanehiro <f-kanehiro@aist.go.jp>")
set(CPACK_PACKAGE_VERSION_MAJOR 3)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 4)
execute_process(
  COMMAND date +%Y%m%d%H%M
  OUTPUT_VARIABLE CPACK_PACKAGE_DATE
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}-${CPACK_PACKAGE_DATE})
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE-en.txt")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}${OPENHRP_VERSION_ADD_TAIL}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "openhrp3.1")
if(UNIX)
  set(CPACK_PACKAGING_INSTALL_PREFIX "/usr" CACHE PATH "The root directory of the package")
endif()
include(CPack)


SET(${PROJECT_NAME}_HEADERS
	${PROJECT_SOURCE_DIR}/include/hrpsys-state-observation/AttitudeEstimator.h
	${PROJECT_SOURCE_DIR}/include/hrpsys-state-observation/KineticsObserver.h
        ${PROJECT_SOURCE_DIR}/include/hrpsys-state-observation/TiltEstimator.h
)


# Add subdirectories.
ADD_SUBDIRECTORY(src)


SETUP_PROJECT_FINALIZE()
