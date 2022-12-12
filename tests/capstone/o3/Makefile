GCC=riscv64-unknown-elf-gcc


all: test

.PHONY: all clean

test: exit.o test.o
	$(GCC) -o test -nostdlib -static exit.o test.o

exit.o: exit.S
	$(GCC) -c -o exit.o exit.S
	
test.o: test.S
	$(GCC) -c -o test.o test.S

clean:
	rm -rf *.o test


