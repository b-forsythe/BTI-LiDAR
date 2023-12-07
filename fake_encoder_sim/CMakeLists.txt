cmake_minimum_required(VERSION 3.8)
project(fake_encoder_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

set(msg_files
  "msg/FakeEncoder.msg"
  )
  
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
  )
  
  
add_executable(
  fake_encoder_sim_node src/fake_encoder_sim_node.cpp
)
ament_target_dependencies(fake_encoder_sim_node rclcpp)  


install(TARGETS
  fake_encoder_sim_node
 DESTINATION lib/${PROJECT_NAME})
 
 
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
    
target_link_libraries(fake_encoder_sim_node "${cpp_typesupport_target}")
  
ament_export_dependencies(rosidl_default_runtime)  

ament_package()
