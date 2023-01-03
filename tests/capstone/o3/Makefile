ASM_SOURCES=$(wildcard *.S)
STATIC_LIB=exit
HEADERS=asm_insn.h
OBJECTS=$(patsubst %.S,%.o,$(ASM_SOURCES))
TARGETS=$(filter-out $(STATIC_LIB),$(patsubst %.S,%,$(ASM_SOURCES)))
GEM5?=gem5.opt
GEM5_FLAGS?=
GEM5_CONFIG?=config.py
GEM5_CONFIG_FLAGS?=

GCC=riscv64-unknown-elf-gcc

all: $(TARGETS)

$(TARGETS):%:$(STATIC_LIB).o %.o
	$(GCC) -o $@ -nostdlib -static $^

$(OBJECTS):%.o:%.S $(HEADERS)
	$(GCC) -c -o $@ $<

run: $(TARGETS)
	$(foreach testcase,$(TARGETS), \
		echo "Running testcase $(testcase)"; \
		$(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $(testcase); \
	)

clean:
	rm -rf $(TARGETS) $(OBJECTS)

.PHONY: all clean run


