cmake_minimum_required(VERSION 3.5)
project(unitree_api)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Official Unitree wire definitions, unchanged. Connext-specific IDL is not
# needed for the Humble Fast DDS / CycloneDDS typesupport used here.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Request.msg"
  "msg/RequestHeader.msg"
  "msg/RequestIdentity.msg"
  "msg/RequestLease.msg"
  "msg/RequestPolicy.msg"
  "msg/Response.msg"
  "msg/ResponseHeader.msg"
  "msg/ResponseStatus.msg"
)
ament_export_dependencies(rosidl_default_runtime)
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})
ament_package()
