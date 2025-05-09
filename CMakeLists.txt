cmake_minimum_required(VERSION 3.5)
project(als_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_components
  cv_bridge
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav2_msgs
  nav_msgs
  # rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  yaml_cpp_vendor
)

foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES als
# #  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions
# #  DEPENDS system_lib
# )

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(mcl src/mcl.cpp)
# target_link_libraries(mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})
ament_target_dependencies(mcl ${dependencies})
install(TARGETS mcl DESTINATION lib/${PROJECT_NAME})

add_executable(sm src/sm.cpp)
# target_link_libraries(sm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})
ament_target_dependencies(sm ${dependencies})
install(TARGETS sm DESTINATION lib/${PROJECT_NAME})

add_executable(mrf_failure_detector src/mrf_failure_detector.cpp)
# target_link_libraries(mrf_failure_detector ${catkin_LIBRARIES})
ament_target_dependencies(mrf_failure_detector ${dependencies})
install(TARGETS mrf_failure_detector DESTINATION lib/${PROJECT_NAME})

add_executable(gl_pose_sampler src/gl_pose_sampler.cpp)
target_link_libraries(gl_pose_sampler ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(gl_pose_sampler ${dependencies})
install(TARGETS gl_pose_sampler DESTINATION lib/${PROJECT_NAME})

add_executable(classifier_dataset_generator src/classifier_dataset_generator.cpp)
# target_link_libraries(classifier_dataset_generator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
ament_target_dependencies(classifier_dataset_generator ${dependencies})
install(TARGETS classifier_dataset_generator DESTINATION lib/${PROJECT_NAME})

add_executable(mae_classifier_learning src/mae_classifier_learning.cpp)
# target_link_libraries(mae_classifier_learning ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})
ament_target_dependencies(mae_classifier_learning ${dependencies})
install(TARGETS mae_classifier_learning DESTINATION lib/${PROJECT_NAME})

add_executable(evaluator src/evaluator.cpp)
# target_link_libraries(evaluator ${catkin_LIBRARIES})
ament_target_dependencies(evaluator ${dependencies})
install(TARGETS evaluator DESTINATION lib/${PROJECT_NAME})

add_executable(scan2pc src/scan2pc.cpp)
# target_link_libraries(scan2pc ${catkin_LIBRARIES})
ament_target_dependencies(scan2pc ${dependencies})
install(TARGETS scan2pc DESTINATION lib/${PROJECT_NAME})

add_executable(slamer src/slamer.cpp)
# target_link_libraries(slamer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PCL_LIBRARY_DIRS})
ament_target_dependencies(slamer ${dependencies})
install(TARGETS slamer DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
