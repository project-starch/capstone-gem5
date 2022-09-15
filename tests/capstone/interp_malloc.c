#include<stddef.h>
#include<stdio.h>
#include<sys/syscall.h>
#include<unistd.h>

#define SYSCALL_NOTIFYMALLOC 3000
#define SYSCALL_NOTIFYFREE 3001

void* __real_malloc(size_t size);
void __real_free(void* ptr);

void* __wrap_malloc(size_t size) {
    void* obj = __real_malloc(size);
    if(obj) {
        return (void*)syscall(SYSCALL_NOTIFYMALLOC, obj, size);
    }
    return NULL;
}

void __wrap_free(void* ptr) {
    __real_free(ptr);
    if(ptr) {
        syscall(SYSCALL_NOTIFYFREE, ptr);
    }
}


