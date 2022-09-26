cmake_minimum_required(VERSION 3.5)
project(romea_imu_localisation_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_imu REQUIRED)
find_package(romea_core_localisation REQUIRED)
find_package(romea_core_localisation_imu REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_localisation_msgs REQUIRED)
find_package(romea_localisation_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
 src/imu_localisation_plugin.cpp
 src/imu_parameters.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
 PRIVATE "ROMEA_LOCALISATION_IMU_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 romea_core_common
 romea_core_localisation
 romea_core_localisation_imu
 romea_common_utils
 romea_localisation_msgs
 romea_localisation_utils
 sensor_msgs
 nav_msgs
 )

rclcpp_components_register_nodes(${PROJECT_NAME}
 "romea::ImuLocalisationPlugin"
 )

install(TARGETS
 ${PROJECT_NAME}
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin)


add_executable(${PROJECT_NAME}_node src/imu_localisation_plugin_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} rclcpp)

install(TARGETS
 ${PROJECT_NAME}_node
 DESTINATION lib/${PROJECT_NAME})


#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)
#ament_export_dependencies(geometry_msgs)
#ament_export_dependencies(romea_corecommon)
#ament_export_dependencies(romea_common_utils)
#ament_export_dependencies(four_wheel_steering_msgs)
#ament_export_dependencies(ackermann_msgs)
#ament_export_dependencies(romea_mobile_base)
#ament_export_dependencies(romea_mobile_base_msgs)
#ament_export_dependencies(romea_mobile_base_utils)
#ament_export_dependencies(romea_localisation_msgs)

#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

install(TARGETS
 ${PROJECT_NAME}
 DESTINATION lib/${PROJECT_NAME})

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

#install(
#  TARGETS ${PROJECT_NAME}
#  EXPORT export_${PROJECT_NAME}
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)

#    find_package(ament_lint_auto REQUIRED)
#    ament_lint_auto_find_test_dependencies()
#    add_subdirectory(test)

#endif()

ament_package()


#cmake_minimum_required(VERSION 3.0.2)
#project(romea_localisation_imu_plugin)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

########################
### Find dependencies ##
########################
#set(${PROJECT_NAME}_CATKIN_DEPS
#    roscpp
#    roslib
#    nodelet
#    romea_common_utils
#    romea_localisation_utils)

#find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
#find_package(romea_core_common REQUIRED)
#find_package(romea_core_imu REQUIRED)
#find_package(romea_core_localisation REQUIRED)
#find_package(romea_core_localisation_imu REQUIRED)

#find_package( PkgConfig )
#pkg_check_modules( EIGEN3 REQUIRED eigen3 )

####################################
### catkin specific configuration ##
####################################
#catkin_package(
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

#include_directories(include
# ${catkin_INCLUDE_DIRS}
# ${EIGEN3_INCLUDE_DIRS}
# ${romea_core_common_INCLUDE_DIRS}
# ${romea_core_imu_INCLUDE_DIRS}
# ${romea_core_localisation_INCLUDE_DIRS}
# ${romea_core_localisation_imu_INCLUDE_DIRS}
#)

#set(romea_LIBRARIES
# ${romea_core_common_LIBRARIES}
# ${romea_core_imu_LIBRARIES}
# ${romea_core_localisation_LIBRARIES}
# ${romea_core_localisation_imu_LIBRARIES}
#)

#add_library(imu_localisation_plugin_nodelet
#  src/imu_localisation_plugin_nodelet.cpp
#  src/ImuFactory.cpp)

#add_executable(imu_localisation_plugin_node
#  src/imu_localisation_plugin_node.cpp)

#target_link_libraries(imu_localisation_plugin_nodelet
#  ${catkin_LIBRARIES}
#  ${romea_LIBRARIES}
#  )

#target_link_libraries(imu_localisation_plugin_node
#  ${catkin_LIBRARIES})

