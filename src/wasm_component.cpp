// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "composition/wasm_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "wasmer.h"

using namespace std::chrono_literals;

namespace composition
{
#define own
// Use the last_error API to retrieve error messages
void print_wasmer_error()
{
    int error_len = wasmer_last_error_length();
    if (error_len > 0) {
      printf("Error len: `%d`\n", error_len);
      char *error_str = (char*)malloc(error_len);
      wasmer_last_error_message(error_str, error_len);
      printf("Error str: `%s`\n", error_str);
    }
}  

WasmNode::WasmNode(const rclcpp::NodeOptions & options)
: Node("WasmNode", options) // future option, take filename and nodename
{
    std::string filepath = "/home/ian/ros2_ws/src/mycomposition/wasm_build/wasm_mod_attacker.wasm"; // fixed filepath for now
    RCLCPP_INFO(this->get_logger(), "Initializing...\n");
    wasm_engine_t* engine = wasm_engine_new();
    mod = new Wasm_Mod("WasmNode", engine, false /*std_inherit*/);
    mod->wasmmod_load_module_from_file(filepath);    
    mod->nowasi_wasmmod_build_instance(); 

    // export wasm_publish function
    // examine the function defined in wasm
    get_wasm_callbacks();
}


void WasmNode::get_wasm_callbacks(){
    // go through the exported functions, 
    // inspect the defined function and their usages
    // dynamically create subscriptions based on function names
    // own wasm_functype_t* get_msg = wasm_functype_new_1_1(wasm_valtype_new_i32(), wasm_valtype_new_i32());
    wasm_func_t* mod_subscriber_callback = mod->wasmmod_get_export_func(1);
    const wasm_name_t* mod_export_name_1 = mod->wasmmod_get_export_name(1);
    printf("function name: %s\n", mod_export_name_1->data);
    if(!mod_subscriber_callback){
      printf("> failed retreiving `mod_subscriber_callback` function!\n");      
    }

    // subscribers 

    // callback wrapper, calls wasm function
    auto callback =
    [this, mod_subscriber_callback](std_msgs::msg::Int32::ConstSharedPtr msg) -> void
    {

      (void)(msg);
      wasm_val_vec_t args = WASM_EMPTY_VEC;
      wasm_val_vec_t results = WASM_EMPTY_VEC;
      if (wasm_func_call(mod_subscriber_callback, &args, &results)) {
        print_wasmer_error();
        printf("> Error calling the `mod_subscriber_callback` function!\n");            
      }
    };
    sub_ = create_subscription<std_msgs::msg::Int32>("chatter", 10, callback);    
}


}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::WasmNode)
