// #include <unistd.h>
// using namespace std;
#include <stdlib.h>
#include <string.h>
#ifdef __cplusplus
extern "C" {  // Prevent name mangling for C++ code
#endif

__attribute__((used, visibility("default"))) void subscriber_callback();
__attribute__((used, visibility("default"))) int on_timer_callback();

int write_and_publish(int i);

#ifdef __cplusplus
}  // End extern "C" block
#endif

int double_free(int i){
    int* cur = (int*)(malloc(sizeof(int)));
    int c = i;    
    free(cur);
    free(cur); // double free
    (void)(*cur++); // accessing that pointer    
    return (c+1);
}

void subscriber_callback(){
    // do nothing
    int c = 1;
    c++;
}


int random_fill(){
    return (int)rand();
}

int on_timer_callback(){
    write_and_publish(random_fill());
    return 0;
}