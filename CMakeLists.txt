# Copyright 2014, Andrea Del Prete, LAAS/CNRS
#
# This file is part of sot-torque-control.
# sot-torque-control is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-torque-control is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-torque-control. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME sot-torque-control)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aiming at the implementation of torque control on HRP-2.")
SET(PROJECT_URL "https://github.com/andreadelprete/sot-torque-control.git")

SET(CUSTOM_HEADER_DIR "sot/torque-control")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
#add_compile_options(-std=c++11) # CMake 2.8.12 or newer

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

PKG_CONFIG_APPEND_LIBS("sot-torque-control")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS thread filesystem program_options unit_test_framework system regex )
SEARCH_FOR_BOOST()

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2")
ADD_REQUIRED_DEPENDENCY("PinInvDyn")

# METAPOD configuration
ADD_REQUIRED_DEPENDENCY("metapod >= 1.0.7")
SET(robot_name "robot_hrp2_14")
SET(lib_robot_name "metapod_hrp2_14")
#SET(METAPODFROMURDF_EXECUTABLE $ENV{ROS_WORKSPACE}/install/bin/metapodfromurdf)
#SET(dir_robot ${PROJECT_BINARY_DIR}/include/metapod/models/hrp2_14)
SET(robot_name_sources
  include/metapod/models/hrp2_14/config.hh
  include/metapod/models/hrp2_14/hrp2_14.hh
  src/hrp2_14.cc
  )
#SET(data_robot_dir ${PROJECT_SOURCE_DIR}/share/metapod/data/hrp2_14)
#SET(config_file_robot  ${data_robot_dir}/hrp2_14.config)
#SET(license_file_robot ${data_robot_dir}/hrp2_14_license_file.txt)
#SET(urdf_file_robot $ENV{ROS_WORKSPACE}/stacks/hrp2/hrp2_14_description/urdf/hrp2_14_reduced_no_fixed.urdf)
#message (STATUS ${METAPODFROMURDF_EXECUTABLE} " --name " hrp2_14 " --libname " ${lib_robot_name} " --directory " ${dir_robot} " --config-file " ${config_file_robot} " --license-file " ${license_file_robot} " " ${urdf_file_robot})
ADD_LIBRARY(${lib_robot_name} SHARED ${robot_name_sources})
# Install the generated files
INSTALL(FILES ${robot_name_sources} DESTINATION include/metapod/models/hrp2_14)
INSTALL(TARGETS ${lib_robot_name} DESTINATION lib)
# END of METAPOD configuration

SET(SOTTORQUECONTROL_LIB_NAME ${PROJECT_NAME})
SET(LIBRARY_NAME ${SOTTORQUECONTROL_LIB_NAME})

SET(${LIBRARY_NAME}_HEADERS
  include/sot/torque_control/joint-torque-controller.hh
  include/sot/torque_control/force-torque-estimator.hh
  include/sot/torque_control/motor-model.hh
  include/sot/torque_control/joint-trajectory-generator.hh
  include/sot/torque_control/nd-trajectory-generator.hh
  include/sot/torque_control/free-flyer-locator.hh
  include/sot/torque_control/inverse-dynamics-controller.hh
  include/sot/torque_control/inverse-dynamics-balance-controller.hh
  include/sot/torque_control/position-controller.hh
  include/sot/torque_control/admittance-controller.hh
  include/sot/torque_control/control-manager.hh
  include/sot/torque_control/hrp2-common.hh
  include/sot/torque_control/hrp2-device-pos-ctrl.hh
  #include/sot/torque_control/hrp2-device-torque-ctrl.hh
  include/sot/torque_control/commands-helper.hh
  include/sot/torque_control/signal-helper.hh
  include/sot/torque_control/utils/logger.hh
  include/sot/torque_control/utils/trajectory-generators.hh
  include/sot/torque_control/utils/lin-estimator.hh
  include/sot/torque_control/utils/poly-estimator.hh
  include/sot/torque_control/utils/quad-estimator.hh
  include/sot/torque_control/utils/Stdafx.hh
  include/sot/torque_control/utils/stop-watch.hh
  include/sot/torque_control/utils/vector-conversions.hh
  include/sot/torque_control/utils/metapod-helper.hh
  )

#INSTALL(FILES ${${LIBRARY_NAME}_HEADERS}
#  DESTINATION include/sot/torque-control
#  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

SET(${LIBRARY_NAME}_SOURCES ${${LIBRARY_NAME}_HEADERS}
    src/logger.cpp
    src/trajectory-generators.cpp
    src/lin-estimator.cpp
    src/poly-estimator.cpp
    src/quad-estimator.cpp
    src/stop-watch.cpp
    src/hrp2-common.cpp
    src/motor-model.cpp
)

SET(${LIBRARY_NAME}_PYTHON_FILES python/*.py)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} metapod)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} PinInvDyn)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} metapod_hrp2_14)

IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

FINDPYTHON()
INSTALL(FILES python/test_estimator.py
              python/create_entities_utils.py
              python/test_estimator_sim.py
              python/hrp2_motors_parameters.py
              python/hrp2_joint_pos_ctrl_gains.py
              python/identification_utils.py
              python/test_balance_ctrl_sim_conf.py
              python/balance_ctrl_conf.py
         DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/)

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
