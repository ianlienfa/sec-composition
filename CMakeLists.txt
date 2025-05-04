cmake_minimum_required(VERSION 3.8)
project(mycomposition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# wasmer dependencies
include_directories("/home/ian/wasmer/package/include") # self-compiled version
link_directories("/home/ian/wasmer/package/lib") # self-compiled version
# include_directories("/home/ian/.wasmer/include")
# link_directories("/home/ian/.wasmer/lib")

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# talker_component
add_library(talker_component SHARED src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(talker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(talker_component 
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_node(
    talker_component
    PLUGIN "composition::Talker"
    EXECUTABLE talker
)
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")
ament_export_targets(export_talker_rclcpp_components)
  
# listener_component
add_library(listener_component SHARED src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(listener_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(listener_component 
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_node(
    listener_component
    PLUGIN "composition::Listener"
    EXECUTABLE listener
)
set(node_plugins "${node_plugins}composition::Listener;$<TARGET_FILE:listener_component>\n")
ament_export_targets(export_listener_rclcpp_components)

# attacker_component
add_library(attacker_component SHARED src/attacker_component.cpp)
target_compile_definitions(attacker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(attacker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(attacker_component 
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_node(
    attacker_component
    PLUGIN "composition::Attacker"
    EXECUTABLE attacker
)
set(node_plugins "${node_plugins}composition::Attacker;$<TARGET_FILE:attacker_component>\n")
ament_export_targets(export_attacker_rclcpp_components)

# wasm_component
add_library(wasm_component SHARED src/wasm_component.cpp)
target_compile_definitions(wasm_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(wasm_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wasm_component 
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_node(
  wasm_component
    PLUGIN "composition::WasmNode"
    EXECUTABLE wasmnode
)
set(node_plugins "${node_plugins}composition::WasmNode;$<TARGET_FILE:wasm_component>\n")
ament_export_targets(export_wasm_component_rclcpp_components)
target_link_libraries(wasm_component "wasmer")

# wasm binary dependency for wasm_component 
add_custom_command(TARGET wasm_component PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E echo "Building wasm_mod_attacker.wasm"  
  COMMAND em++ -std=c++20 -O2 -sSTANDALONE_WASM -Wl,--allow-undefined -Wl,--no-entry -o ${CMAKE_CURRENT_SOURCE_DIR}/wasm_build/wasm_mod_attacker.wasm ${CMAKE_CURRENT_SOURCE_DIR}/src/wasm_mod_attacker.cpp
  COMMAND wasm2wat ${CMAKE_CURRENT_SOURCE_DIR}/wasm_build/wasm_mod_attacker.wasm -o ${CMAKE_CURRENT_SOURCE_DIR}/wasm_build/wasm_mod_attacker.wat
)

# manual_compose
add_executable(manual_compose "src/manual_compose.cpp")
target_link_libraries(manual_compose 
  talker_component wasm_component attacker_component
)
ament_target_dependencies(manual_compose
  "rclcpp")

# dds_listener
add_executable(dds_listener "src/dds_listener.cpp")
ament_target_dependencies(dds_listener
  "rclcpp"
  "std_msgs"
)
  

install(TARGETS listener_component
        EXPORT export_listener_rclcpp_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS talker_component
        EXPORT export_talker_rclcpp_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS attacker_component
        EXPORT export_attacker_rclcpp_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS wasm_component
        EXPORT export_wasm_component_rclcpp_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS
  manual_compose
  dds_listener
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
