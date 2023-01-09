ASM_SOURCES=$(wildcard *.S)
STATIC_LIB=exit
HEADERS=asm_insn.h
OBJECTS=$(patsubst %.S,%.o,$(ASM_SOURCES))
TARGETS=$(filter-out $(STATIC_LIB),$(patsubst %.S,%,$(ASM_SOURCES)))
RUN_TARGETS=$(patsubst %,run-%,$(TARGETS))
DEBUG_TARGETS=$(patsubst %,debug-%,$(TARGETS))
GEM5?=gem5.opt
GEM5_FLAGS?=
GEM5_CONFIG?=config.py
GEM5_CONFIG_FLAGS?=
GEM5_TEST_TIMEOUT?=5

GCC=riscv64-unknown-linux-gnu-gcc

all: $(TARGETS)

$(TARGETS):%:$(STATIC_LIB).o %.o
	$(GCC) -o $@ -nostdlib -static $^

$(OBJECTS):%.o:%.S $(HEADERS)
	$(GCC) -c -o $@ $<

$(RUN_TARGETS):run-%:%
	@echo "Running testcase $^"; \
		if [ -f $^.out ]; then \
			echo "Testing against expected output"; \
			TMP_FILE=$$(mktemp); \
			(timeout $(GEM5_TEST_TIMEOUT) $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^ 2>&1 | tee $$TMP_FILE); \
			grep -q "$$(cat $^.out)" $$TMP_FILE; \
		else \
			echo "Testing against exit code"; \
			timeout $(GEM5_TEST_TIMEOUT) $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^ 2>&1 | tee $$TEMP_FILE; \
		fi
			

$(DEBUG_TARGETS):debug-%:%
	gdb --args $(GEM5) $(GEM5_FLAGS) $(GEM5_CONFIG) $(GEM5_CONFIG_FLAGS) -- $^

run: $(RUN_TARGETS)

clean:
	rm -rf $(TARGETS) $(OBJECTS)

.PHONY: all clean run $(RUN_TARGETS) $(DEBUG_TARGETS)


