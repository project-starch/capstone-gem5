CROSS_PREFIX=riscv64-unknown-linux-gnu-
CC=$(CROSS_PREFIX)gcc
LD=$(CROSS_PREFIX)ld
AR=$(CROSS_PREFIX)ar

all: hello libinterp.a

libinterp.a: interp_malloc.o
	$(AR) rc libinterp.a interp_malloc.o

hello: interp_malloc.o hello.o
	$(CC) -Wl,--wrap=malloc -Wl,--wrap=free -static -o hello interp_malloc.o hello.o

interp_malloc.o: interp_malloc.c
	$(CC) -o interp_malloc.o  -c interp_malloc.c

hello.o: hello.c
	$(CC) -o hello.o  -c hello.c


.PHONY: clean all

clean:
	rm -rf hello interp_malloc.o hello.o libinterp.a

