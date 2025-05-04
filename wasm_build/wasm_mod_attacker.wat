(module
  (type (;0;) (func (result i32)))
  (type (;1;) (func (param i32) (result i32)))
  (type (;2;) (func))
  (type (;3;) (func (param i32)))
  (import "env" "write_and_publish" (func (;0;) (type 1)))
  (func (;1;) (type 2))
  (func (;2;) (type 0) (result i32)
    (local i64)
    i32.const 1024
    i32.const 1024
    i64.load
    i64.const 6364136223846793005
    i64.mul
    i64.const 1
    i64.add
    local.tee 0
    i64.store
    local.get 0
    i64.const 33
    i64.shr_u
    i32.wrap_i64
    call 0
    drop
    i32.const 0)
  (func (;3;) (type 3) (param i32)
    local.get 0
    global.set 0)
  (func (;4;) (type 0) (result i32)
    global.get 0)
  (table (;0;) 1 1 funcref)
  (memory (;0;) 258 258)
  (global (;0;) (mut i32) (i32.const 66576))
  (export "memory" (memory 0))
  (export "subscriber_callback" (func 1))
  (export "on_timer_callback" (func 2))
  (export "__indirect_function_table" (table 0))
  (export "_emscripten_stack_restore" (func 3))
  (export "emscripten_stack_get_current" (func 4)))
