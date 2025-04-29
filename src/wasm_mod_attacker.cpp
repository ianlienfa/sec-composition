// #include <unistd.h>
// using namespace std;
#include <stdlib.h>
#include <string.h>
#ifdef __cplusplus
extern "C" {  // Prevent name mangling for C++ code
#endif

__attribute__((used, visibility("default"))) void subscriber_callback();

#ifdef __cplusplus
}  // End extern "C" block
#endif

int double_free(int i){
    int* cur = (int*)(malloc(sizeof(int)));
    int c = i;    
    free(cur);
    free(cur); // double free
    (*cur++); // accessing that pointer
    return (c+1);
}

void subscriber_callback(){
    double_free(1);
}
