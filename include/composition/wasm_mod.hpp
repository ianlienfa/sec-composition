
#ifndef WASM_MOD_HPP
#define WASM_MOD_HPP

#include "wasmer.h"
#define wasm_own

struct Wasm_Mod {  
    wasm_engine_t* engine = nullptr;
    wasm_store_t* store = nullptr;
    wasm_module_t* module = nullptr;
    wasi_config_t* config = nullptr;
    wasi_env_t* wasi_env = nullptr;
    wasm_instance_t* instance = nullptr;
    wasm_extern_vec_t imports;
    wasm_extern_vec_t exports;
    wasm_exporttype_vec_t export_types;
    std::vector<wasm_func_t*> func_exposed;
    std::string mod_name;
  
    Wasm_Mod(std::string mod_name, wasm_engine_t* engine, bool std_inherit=true){    
      this->mod_name = mod_name;
      this->engine = engine;  
      wasm_extern_vec_new_empty(&imports);
      wasm_extern_vec_new_empty(&exports);
      wasm_exporttype_vec_new_empty(&export_types);
      store = wasm_store_new(engine);
      config = wasi_config_new(mod_name.c_str());
      if(std_inherit){
        wasi_config_inherit_stdout(config);
        wasi_config_inherit_stdin(config);
        wasi_config_inherit_stderr(config);
      }
    }  
  
    ~Wasm_Mod(){
      if(wasi_env)wasi_env_delete(wasi_env);
      if(module)wasm_module_delete(module);
      if(instance)wasm_instance_delete(instance);
      if(store)wasm_store_delete(store);
      if(exports.size)wasm_extern_vec_delete(&exports);
      if(export_types.size)wasm_exporttype_vec_delete(&export_types);    
      for(auto f: func_exposed){
        wasm_func_delete(f);
      }
    }
  
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

    wasm_byte_vec_t load_wasm_binary(std::string filepath){
      printf("Loading binary...\n");
      FILE* file = fopen(filepath.c_str(), "rb");
      if (!file) {
        throw std::runtime_error("> Error loading module!\n");
        return wasm_byte_vec_t();
      }
      fseek(file, 0L, SEEK_END);
      size_t file_size = ftell(file);
      fseek(file, 0L, SEEK_SET);
      wasm_byte_vec_t binary;
      wasm_byte_vec_new_uninitialized(&binary, file_size);
      if (fread(binary.data, file_size, 1, file) != 1) {
        throw std::runtime_error("> Error initializing module!\n");
        return wasm_byte_vec_t();
      }
      fclose(file);
      return binary;
    }
    
    void wasmmod_build_instance(){
      if (!wasi_get_imports(store, wasi_env, module, &imports)) {
        print_wasmer_error();
        throw std::runtime_error("> Error getting WASI imports!\n");
        }
      wasm_own instance =
        wasm_instance_new(store, module, &imports, NULL);
      if (!instance) {
        throw std::runtime_error("> Error instantiating module!\n");
        print_wasmer_error();
        }
      if (!wasi_env_initialize_instance(wasi_env, store, instance)) {
        print_wasmer_error();
        throw std::runtime_error("> Error initializing wasi env memory!\n");
        }
    } 
  
    void nowasi_wasmmod_build_instance(){
      wasm_own instance =
        wasm_instance_new(store, module, &imports, NULL);
      if (!instance) {
        throw std::runtime_error("> Error instantiating module!\n");
        print_wasmer_error();
        }
    } 
  
    // this deletes the binary vec 
    void wasmmod_load_module_from_bytes(wasm_byte_vec_t *binary){
      printf("Compiling module...\n");
      wasm_own module = wasm_module_new(store, binary);
      if (!module) {
        throw std::runtime_error("> Error compiling module!\n");
      }
      wasm_byte_vec_delete(binary);
  
      // create wasi env
      wasi_env = wasi_env_new(store, config);
      if (!wasi_env) {
        print_wasmer_error();
        throw std::runtime_error("> Error building WASI env!\n");
        return ;
      }
  
      // populate import too
      if(!(module && wasi_env && store)){      
        if(!module) throw std::runtime_error("> module is NULL!\n");
        if(!wasi_env) throw std::runtime_error("> wasi_env is NULL!\n");
        if(!store) throw std::runtime_error("> store is NULL!\n");
      }
    }
  
    void nowasi_wasmmod_load_module_from_bytes(wasm_byte_vec_t *binary){
      printf("Compiling module...\n");
      wasm_own module = wasm_module_new(store, binary);
      if (!module) {
        throw std::runtime_error("> Error compiling module!\n");
      }
      wasm_byte_vec_delete(binary);
  
      // populate import too
      if(!(module && store)){      
        if(!module) throw std::runtime_error("> module is NULL!\n");
        if(!store) throw std::runtime_error("> store is NULL!\n");
      }
  
      // populate export types
      wasm_module_exports(module, &export_types);
    }
  
    void wasmmod_load_module_from_str(const char* wat_string){
      wasm_own wasm_byte_vec_t wat;
      wasm_byte_vec_new(&wat, strlen(wat_string), wat_string);
      wasm_own wasm_byte_vec_t wasm_bytes;
      wat2wasm(&wat, &wasm_bytes);
      wasm_byte_vec_delete(&wat); // wasm_own wasm_byte_vec_t wat
      nowasi_wasmmod_load_module_from_bytes(&wasm_bytes); // wasm_ownership trans
    }
  
    void wasmmod_load_module_from_file(std::string filepath){
      wasm_byte_vec_t binary = load_wasm_binary(filepath);
      nowasi_wasmmod_load_module_from_bytes(&binary); // wasm_ownership trans
    }
    
    int wasmmod_run_start(){
      if(!instance){
        printf("> Instance not initialized!\n");
        return 1;
      }
      wasm_func_t* run_func = wasi_get_start_function(instance);
      func_exposed.push_back(run_func);
      if (run_func == NULL) {
        printf("> Error accessing export!\n");
        print_wasmer_error();
        return 1;
      }
      wasm_val_vec_t args = WASM_EMPTY_VEC;
      wasm_val_vec_t res = WASM_EMPTY_VEC;
      printf("=============== %s: Start module function call=============\n", mod_name.c_str());
      if (wasm_func_call(run_func, &args, &res)) {
        printf("> Error calling function!\n");    
        return 1;
      }
      printf("===================Call completed===========================\n");
      return 0;
    }
  
    // func/arg mem will automatically be cleaned up after call
    wasm_func_t* wasmmod_get_export_func(int func_index, std::string func_name = ""){
      wasm_instance_exports(instance, &exports);
      if (exports.size == 0) {
        printf("> Error accessing exports!\n");
        return NULL;
      }
      printf("export size: %ld\n", exports.size);
      printf("Retrieving function %s at index %d.\n", func_name.c_str(), func_index);
  
      wasm_func_t* func = wasm_extern_as_func(exports.data[func_index]);
      if (func == NULL) {
          printf("> Failed to get the `%s` function!\n", func_name.c_str());
          return NULL;
      }
      return func;
    }
  
    const wasm_name_t* wasmmod_get_export_name(int func_index){
      if (export_types.size == 0) {
        printf("> Error accessing exports!\n, call nowasi_wasmmod_load_module_from_bytes before processing name!");
        return NULL;
      }
      printf("export_types size: %ld\n", exports.size);
  
      wasm_exporttype_t* export_type = export_types.data[func_index];
      if (export_type == NULL) {
        printf("> Failed to get the export_type at index %d!\n", func_index);
        return NULL;
      }
      
      const wasm_name_t* export_name = wasm_exporttype_name(export_type);
      if (export_name == NULL) {
          printf("> Failed to get the name at %d!\n", func_index);
          return NULL;
      }
      return export_name;
    }
  
    const wasm_externtype_t* wasmmod_get_export_type(int func_index){
      if (export_types.size == 0) {
        printf("> Error accessing exports!\n, call nowasi_wasmmod_load_module_from_bytes before processing name!");
        return NULL;
      }
      printf("export_types size: %ld\n", exports.size);
  
      wasm_exporttype_t* export_type = export_types.data[func_index];
      if (export_type == NULL) {
        printf("> Failed to get the export_type at index %d!\n", func_index);
        return NULL;
      }
      const wasm_externtype_t* extern_type = wasm_exporttype_type(export_type);
      if (extern_type == NULL) {
          printf("> Failed to get the type at %d!\n", func_index);
          return NULL;
      }
      return extern_type;
    }
  } ;

#endif