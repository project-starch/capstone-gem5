SHELL:=/bin/bash

ASM_SOURCES=$(wildcard *.S)
STATIC_LIB=exit
HEADERS=asm_insn.h
OBJECTS=$(patsubst %.S,%.o,$(ASM_SOURCES))
TARGETS=$(filter-out $(STATIC_LIB),$(patsubst %.S,%,$(ASM_SOURCES)))
RUN_TARGETS=$(patsubst %,run-%,$(TARGETS))
DEBUG_TARGETS=$(patsubst %,debug-%,$(TARGETS))
MEMCHECK_TARGETS=$(patsubst %,memcheck-%,$(TARGETS))
GEM5?=gem5.opt
GEM5_FLAGS?=
GEM5_CONFIG?=config.py
GEM5_CONFIG_FLAGS?=
GEM5_TEST_TIMEOUT?=20
CAPSTONE_TESTS_CFLAGS?=

GCC=riscv64-unknown-linux-gnu-gcc

all: $(TARGETS)

$(TARGETS):%:$(STATIC_LIB).o %.o
	$(GCC) -o $@ -nostdlib -static $^

$(OBJECTS):%.o:%.S $(HEADERS)
	$(GCC) $(CAPSTONE_TESTS_CFLAGS) -c -o $@ $<

$(RUN_TARGETS):run-%:%
	$(info Running testcase $^)
	@tmpfile=$$(mktemp); \
	timeout $(GEM5_TEST_TIMEOUT) $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^ 2>&1 | tee $$tmpfile; \
	outcode=$${PIPESTATUS[0]}; \
	if [ -f $^.code ]; then \
		expected="$$(cat $^.code)"; \
	else \
		expected=0; \
	fi; \
	if [ -f $^.out ]; then \
		grep -q "$$(cat $^.out)" $$tmpfile; \
	else \
		test $$outcode -eq "$$expected"; \
	fi;
	
$(DEBUG_TARGETS):debug-%:%
	gdb --args $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^

$(MEMCHECK_TARGETS):memcheck-%:%
	valgrind $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^

run: $(RUN_TARGETS)

clean:
	rm -rf $(TARGETS) $(OBJECTS)

.PHONY: all clean run $(RUN_TARGETS) $(DEBUG_TARGETS)


