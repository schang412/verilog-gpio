# Verilog GPIO

[![Regression Tests](https://github.com/schang412/verilog-gpio/actions/workflows/regression-tests.yml/badge.svg)](https://github.com/schang412/verilog-gpio/actions/workflows/regression-tests.yml)

GitHub repository: https://github.com/schang412/verilog-gpio

## Introduction

GPIO interface components written in Verilog with cocotb testbenches.

## Documentation

### gpio_axil module

#### Parameters
 - NUM_GPIO: the number of GPIOs
 - AXIL_ADDR_WIDTH: the address width of the AXIL interface (16, 32, 64)
 - AXIL_ADDR_BASE: the base address that the contents are offset from
 - RB_NEXT_PTR: the pointer to the next block

### Source Files
```
rtl/gpio_axil.v : GPIO module (32-bit AXI lite slave)
```

## Testing

Running the included testbenches requires [cocotb](https://github.com/cocotb/cocotb), [cocotbext-axi](https://github.com/alexforencich/cocotbext-axi), and [Icarus Verilog](http://iverilog.icarus.com/).  The testbenches can be run with pytest directly (requires [cocotb-test](https://github.com/themperek/cocotb-test)), pytest via tox, or via cocotb makefiles.