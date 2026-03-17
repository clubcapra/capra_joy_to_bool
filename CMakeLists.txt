cmake_minimum_required(VERSION 3.8)
project(capra_joy_to_bool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PROJECT_DEPEND
  rclcpp
  std_msgs
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PROJECT_DEPEND})
  find_package(${dep} REQUIRED)
endforeach(dep)

add_library(joy_to_bool_node src/joy_to_bool_node.cpp)
target_include_directories(joy_to_bool_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joy_to_bool_node
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(joy_to_bool_node PUBLIC cxx_std_20)

add_executable(joy_to_bool src/joy_to_bool_main.cpp)
target_link_libraries(joy_to_bool joy_to_bool_node)
ament_target_dependencies(joy_to_bool
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(joy_to_bool PRIVATE cxx_std_20)

install(TARGETS
  joy_to_bool_node
  joy_to_bool
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joy_to_bool_node test/test_joy_to_bool_node.cpp)
  if(TARGET test_joy_to_bool_node)
    target_link_libraries(test_joy_to_bool_node joy_to_bool_node)
    ament_target_dependencies(test_joy_to_bool_node ${THIS_PROJECT_DEPEND})
    target_include_directories(test_joy_to_bool_node PRIVATE include)
    target_compile_features(test_joy_to_bool_node PRIVATE cxx_std_20)
  endif()
endif()


ament_package()
