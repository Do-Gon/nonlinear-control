cmake_minimum_required(VERSION 3.8)
project(mycobot_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

set(MYCOBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../MyCobotCppLib")
set(MYCOBOT_INCLUDE_DIR "${MYCOBOT_SDK_ROOT}/include")
set(MYCOBOT_LIB "${MYCOBOT_SDK_ROOT}/build/lib/libmyCobotCpp.so")

if(NOT EXISTS "${MYCOBOT_INCLUDE_DIR}/mycobot/MyCobot.hpp")
  message(FATAL_ERROR "Cannot find mycobot/MyCobot.hpp under ${MYCOBOT_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${MYCOBOT_LIB}")
  message(FATAL_ERROR "Cannot find libmyCobotCpp.so at ${MYCOBOT_LIB}")
endif()

message(STATUS "Using SDK include: ${MYCOBOT_INCLUDE_DIR}")
message(STATUS "Using SDK lib: ${MYCOBOT_LIB}")


# --- Executables ---
add_executable(mycobot_motion src/mycobot_motion.cpp)
target_compile_definitions(mycobot_motion PRIVATE ROBOT_MYCOBOT)
target_include_directories(mycobot_motion PRIVATE
  ${MYCOBOT_INCLUDE_DIR} ${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS})
target_link_libraries(mycobot_motion ${MYCOBOT_LIB} Qt5::Core Qt5::SerialPort)
ament_target_dependencies(mycobot_motion rclcpp std_msgs)

add_executable(mycobot_gripper src/gripper_only.cpp)
target_compile_definitions(mycobot_gripper PRIVATE ROBOT_MYCOBOT)
target_include_directories(mycobot_gripper PRIVATE
  ${MYCOBOT_INCLUDE_DIR} ${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS})
target_link_libraries(mycobot_gripper ${MYCOBOT_LIB} Qt5::Core Qt5::SerialPort)
ament_target_dependencies(mycobot_gripper rclcpp std_msgs)

add_executable(mycobot_jog src/jog_j1_until.cpp)
target_compile_definitions(mycobot_jog PRIVATE ROBOT_MYCOBOT)
target_include_directories(mycobot_jog PRIVATE
  ${MYCOBOT_INCLUDE_DIR} ${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS})
target_link_libraries(mycobot_jog ${MYCOBOT_LIB} Qt5::Core Qt5::SerialPort)
ament_target_dependencies(mycobot_jog rclcpp std_msgs)

add_executable(mycobot_joint_limits src/joint_limit.cpp)
target_compile_definitions(mycobot_joint_limits PRIVATE ROBOT_MYCOBOT)
target_include_directories(mycobot_joint_limits PRIVATE
  ${MYCOBOT_INCLUDE_DIR} ${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS})
target_link_libraries(mycobot_joint_limits ${MYCOBOT_LIB} Qt5::Core Qt5::SerialPort)
ament_target_dependencies(mycobot_joint_limits rclcpp std_msgs)

add_executable(mycobot_pd_main src/mycobot_pd_main.cpp)
target_compile_definitions(mycobot_pd_main PRIVATE ROBOT_MYCOBOT)
target_include_directories(mycobot_pd_main PRIVATE
  ${MYCOBOT_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5SerialPort_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mycobot_pd_main ${MYCOBOT_LIB} Qt5::Core Qt5::SerialPort)
ament_target_dependencies(mycobot_pd_main rclcpp std_msgs)

install(FILES ${MYCOBOT_LIB} DESTINATION lib/${PROJECT_NAME})

set(_sdk_libdir "${MYCOBOT_SDK_ROOT}/build/lib")
set_target_properties(mycobot_motion mycobot_gripper mycobot_jog mycobot_joint_limits mycobot_pd_main PROPERTIES
  BUILD_RPATH "${_sdk_libdir}"
  INSTALL_RPATH "\$ORIGIN"
)

install(TARGETS
  mycobot_motion
  mycobot_gripper
  mycobot_jog
  mycobot_joint_limits
  mycobot_pd_main
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install public headers (pd_controller.hpp, etc.) ---
install(DIRECTORY include/
  DESTINATION include)

ament_package()
