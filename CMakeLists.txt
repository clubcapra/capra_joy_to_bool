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

add_executable(joy_to_bool src/joy_to_bool_node.cpp)
ament_target_dependencies(joy_to_bool
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(joy_to_bool PRIVATE cxx_std_20)

install(TARGETS
  joy_to_bool
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   config include
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
