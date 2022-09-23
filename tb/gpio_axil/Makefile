TOPLEVEL_LANG = verilog

SIM ?= icarus
WAVES ?= 1

COCOTB_HDL_TIMEUNIT = 1ns
COCOTB_HDL_TIMEPRECISION = 1ps

DUT      = gpio_axil
TOPLEVEL = $(DUT)
MODULE   = test_$(DUT)

RTL_DIR = $(shell pwd)/../../rtl

VERILOG_FILES = $(DUT).v

VERILOG_SOURCES = $(patsubst %, $(RTL_DIR)/%, $(VERILOG_FILES))

export PARAM_NUM_GPIO ?= 32
export PARAM_AXIL_ADDR_WIDTH ?= 16
export PARAM_AXIL_ADDR_BASE ?= 0
export PARAM_RB_NEXT_PTR ?= 0

ifeq ($(SIM), icarus)
	PLUSARGS += -fst

	COMPILE_ARGS += -P $(TOPLEVEL).NUM_GPIO=$(PARAM_NUM_GPIO)
	COMPILE_ARGS += -P $(TOPLEVEL).AXIL_ADDR_WIDTH=$(PARAM_AXIL_ADDR_WIDTH)
	COMPILE_ARGS += -P $(TOPLEVEL).AXIL_ADDR_BASE=$(PARAM_AXIL_ADDR_BASE)
	COMPILE_ARGS += -P $(TOPLEVEL).RB_NEXT_PTR=$(PARAM_RB_NEXT_PTR)

	ifeq ($(WAVES), 1)
		VERILOG_SOURCES += iverilog_dump.v
		COMPILE_ARGS += -s iverilog_dump
	endif
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

iverilog_dump.v:
	echo 'module iverilog_dump();' > $@
	echo 'initial begin' >> $@
	echo '    $$dumpfile("$(TOPLEVEL).fst");' >> $@
	echo '    $$dumpvars(0, $(TOPLEVEL));' >> $@
	echo 'end' >> $@
	echo 'endmodule' >> $@

clean::
	@rm -rf iverilog_dump.v
	@rm -rf dump.fst $(TOPLEVEL).fst
	@rm -rf results.xml
