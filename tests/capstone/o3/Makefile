ASM_SOURCES=$(wildcard *.S)
STATIC_LIB=exit
HEADERS=asm_insn.h
OBJECTS=$(patsubst %.S,%.o,$(ASM_SOURCES))
TARGETS=$(filter-out $(STATIC_LIB),$(patsubst %.S,%,$(ASM_SOURCES)))

GCC=riscv64-unknown-elf-gcc

all: $(TARGETS)

$(TARGETS):%:$(STATIC_LIB).o %.o
	$(GCC) -o $@ -nostdlib -static $^

$(OBJECTS):%.o:%.S $(HEADERS)
	$(GCC) -c -o $@ $<

clean:
	rm -rf $(TARGETS) $(OBJECTS)

.PHONY: all clean


