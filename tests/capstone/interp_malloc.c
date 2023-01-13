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
        void* res = obj;
        asm(".insn r 0x5B, 0x0, 0x5, %0, %1, %2" 
                : "=r" (res)
                : "r" (obj), "r"(size));
        return res;
    }
    return NULL;
}

void __wrap_free(void* ptr) {
    __real_free(ptr);
    if(ptr) {
        asm(".insn r 0x5B, 0x0, 0x6, x0, %0, x0"
                : : "r" (ptr));
    }
}


