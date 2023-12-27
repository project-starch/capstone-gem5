SHELL:=/bin/bash

SRC_DIR=./src/capstone

ASM_SOURCES=$(wildcard $(SRC_DIR)/*.S)
STATIC_LIB=$(SRC_DIR)/exit
HEADERS=$(SRC_DIR)/asm_insn.h
OBJECTS=$(patsubst %.S,%.o,$(ASM_SOURCES))
TARGETS=$(filter-out $(STATIC_LIB),$(patsubst %.S,%,$(ASM_SOURCES)))
TARGET_NAMES=$(patsubst $(SRC_DIR)/%,%,$(TARGETS))
RUN_TARGETS=$(patsubst %,run-%,$(TARGET_NAMES))
DEBUG_TARGETS=$(patsubst %,debug-%,$(TARGET_NAMES))
MEMCHECK_TARGETS=$(patsubst %,memcheck-%,$(TARGET_NAMES))
RECORD_TARGETS=$(patsubst %,rr-%,$(TARGET_NAMES))
GEM5?=../../../build/RISCVCapstone/gem5.debug
GEM5_FLAGS?=
GEM5_CONFIG?=../../../configs/capstone/fast-forward.py
GEM5_CONFIG_FLAGS?=--cpu=o3
GEM5_TEST_TIMEOUT?=20
CAPSTONE_TESTS_CFLAGS?=

GCC=riscv64-unknown-linux-gnu-gcc

all: $(TARGETS)

$(TARGETS):%:$(STATIC_LIB).o %.o
	$(GCC) -o $@ -nostdlib -static -mno-relax $^

$(TARGET_NAMES):%:$(SRC_DIR)/%

$(OBJECTS):%.o:%.S $(HEADERS)
	$(GCC) $(CAPSTONE_TESTS_CFLAGS) -c -o $@ $<

$(RUN_TARGETS):run-%:$(SRC_DIR)/%
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
	
$(DEBUG_TARGETS):debug-%:$(SRC_DIR)/%
	gdb --args $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^

$(MEMCHECK_TARGETS):memcheck-%:$(SRC_DIR)/%
	valgrind $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^

$(RECORD_TARGETS):rr-%:$(SRC_DIR)/%
	rr record $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^

run: $(RUN_TARGETS)

clean:
	rm -rf $(TARGETS) $(OBJECTS)

.PHONY: all clean run $(RUN_TARGETS) $(DEBUG_TARGETS) $(TARGET_NAMES)


