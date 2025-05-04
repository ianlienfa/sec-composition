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

// the write_and_publish interface for ROS
// everytime this is called, allocate a unique pointer and write the given value into it
wasm_trap_t* WasmNode::wasm_write_and_publish(void* env_arg, const wasm_val_vec_t* args, wasm_val_vec_t* results) {
  auto* self = static_cast<WasmNode*>(env_arg);
  auto msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = static_cast<int>(args->data[0].of.i32);
  if((!self) || (!self->pub_)){
    results->data[0].of.i64 = 1;
  }
  else{
    self->pub_->publish(std::move(msg));
    results->data[0].of.i64 = 0;
  }
  return NULL;
}

WasmNode::WasmNode(const rclcpp::NodeOptions & options)
: Node("WasmNode", options) // future option, take filename and nodename
{
    // ROS specific init (to be automated)
    timer_ = create_wall_timer(50ms, [this]() {return this->on_timer();});
    pub_ = create_publisher<std_msgs::msg::Int32>("rand", 10);

    // wasm specific init
    std::string filepath = "/home/ian/ros2_ws/src/mycomposition/wasm_build/wasm_mod_attacker.wasm"; // fixed filepath for now
    RCLCPP_INFO(this->get_logger(), "Initializing...\n");
    wasm_engine_t* engine = wasm_engine_new();
    mod = new Wasm_Mod("WasmNode", engine, false /*std_inherit*/);
    mod->wasmmod_load_module_from_file(filepath);    

    // process import before initializing instance
    own wasm_functype_t* write_and_publish_type = wasm_functype_new_1_1(wasm_valtype_new_i32(), wasm_valtype_new_i32());
    own wasm_func_t* write_and_publish_func = wasm_func_new_with_env(mod->store, write_and_publish_type, wasm_write_and_publish, this, NULL);
    wasm_functype_delete(write_and_publish_type);
    wasm_extern_t* externs[] = {
      wasm_func_as_extern(write_and_publish_func)
    };
    mod->imports = WASM_ARRAY_VEC(externs);

    // build instance after import
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

    wasm_func_t* mod_on_timer_callback = mod->wasmmod_get_export_func(2);
    const wasm_name_t* mod_export_name_2 = mod->wasmmod_get_export_name(2);    
    printf("function name: %s\n", mod_export_name_2->data);
    if(!mod_subscriber_callback){
      printf("> failed retreiving `on_timer_callback` function!\n");      
    }

    auto wasm_on_timer_callback =
    [this, mod_on_timer_callback]() -> void
    {      
      wasm_val_vec_t args = WASM_EMPTY_VEC;
      wasm_val_t results_val[1] = { WASM_INIT_VAL };
      wasm_val_vec_t results = WASM_ARRAY_VEC(results_val);
      if (wasm_func_call(mod_on_timer_callback, &args, &results)) {
        // print_wasmer_error();
        // printf("> Error calling the `wasm_on_timer_callback` function!\n");            
      }
    };

    this->on_timer_callback = wasm_on_timer_callback;
}

void WasmNode::on_timer()
{
  this->on_timer_callback();
}


}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::WasmNode)
