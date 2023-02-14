cmake_minimum_required(VERSION 3.0.2)
project(polhemus_ros_driver)

add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)

pkg_check_modules(libusb-1.0 libusb-1.0 QUIET)

if(NOT ${libusb_1_FOUND})
  message(FATAL_ERROR "Could not find libusb1")
endif()

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  geometry_msgs
  actionlib_msgs
  tf2
  tf2_ros
  roscpp
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  calibrate.srv
  set_source.srv
  persist.srv
  Publish.srv
)

add_action_files(
  FILES
  Calibrate.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polhemus
  CATKIN_DEPENDS std_msgs geometry_msgs actionlib_msgs tf2 tf2_ros roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_1_INCLUDE_DIRS}
)

add_executable(polhemus_tf_broadcaster src/polhemus_ros_driver/polhemus_tf_broadcaster.cpp)
add_dependencies(polhemus_tf_broadcaster polhemus_ros_driver_generate_messages_cpp)

add_library(polhemus
  src/polhemus_ros_driver/liberty.cpp
  src/polhemus_ros_driver/viper.cpp
  src/polhemus_ros_driver/polhemus.cpp
)

add_dependencies(polhemus polhemus_ros_driver_generate_messages_cpp)

target_link_libraries(polhemus_tf_broadcaster
  ${catkin_LIBRARIES}
  ${LIBUSB_1_LIBRARIES}
  usb-1.0
  polhemus
)

install(DIRECTORY include/polhemus_ros_driver
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS polhemus_tf_broadcaster polhemus
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/start.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
