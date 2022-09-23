# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2022 Spencer Chang
import os
import logging

import cocotb_test.simulator
import pytest

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, Event

from cocotbext.axi import AxiLiteMaster, AxiLiteBus


class TB:
    def __init__(self, dut):
        self.dut = dut
        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        cocotb.start_soon(Clock(dut.clk, 4, units='ns').start())
        self.axil_master = AxiLiteMaster(AxiLiteBus.from_prefix(dut, 's_axil'), dut.clk, dut.rst)

    async def reset(self):
        self.dut.rst.setimmediatevalue(0)
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.rst.value = 1
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.rst.value = 0
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)


A32V_GPIO_ID            = 0x294EC110
A32V_GPIO_REV           = 0x00000100

A32V_GPIO_ID_OFFSET     = 0x00
A32V_GPIO_REV_OFFSET    = 0x04
A32V_GPIO_PNT_OFFSET    = 0x08
A32V_GPIO_RESETR_OFFSET = 0x10
A32V_GPIO_RESET_VECTOR  = 0x0A
A32V_GPIO_DDR_OFFSET    = 0x24
A32V_GPIO_OUT_OFFSET    = 0x28
A32V_GPIO_IN_OFFSET     = 0x2C


async def do_soft_rst(tb, baseaddr):
    await tb.axil_master.write_dword(baseaddr + A32V_GPIO_RESETR_OFFSET, A32V_GPIO_RESET_VECTOR)


@cocotb.test()
async def run_test(dut):
    tb = TB(dut)
    await tb.reset()

    baseaddr = int(os.environ["PARAM_AXIL_ADDR_BASE"])
    num_gpio = int(os.environ["PARAM_NUM_GPIO"])
    test_vectors = [0xffff_ffff, 0xaaaa_aaaa, 0x00aa00aa, 0xaa00aa00, 0x55aa55aa, 0xaa55aa55]
    bmask = (2 ** num_gpio) - 1

    # test symmetric ddr read/write
    await tb.axil_master.write_dword(baseaddr + A32V_GPIO_DDR_OFFSET, 0xAA)
    ddr = await tb.axil_master.read_dword(baseaddr + A32V_GPIO_DDR_OFFSET)
    assert ddr == 0xAA

    # test software reset
    await do_soft_rst(tb, baseaddr)
    ddr = await tb.axil_master.read_dword(baseaddr + A32V_GPIO_DDR_OFFSET)
    assert ddr == 0x00

    # test output
    await tb.axil_master.write_dword(baseaddr + A32V_GPIO_OUT_OFFSET, 0xffff_ffff)
    for test_vector in test_vectors:
        v = bmask & test_vector
        await tb.axil_master.write_dword(baseaddr + A32V_GPIO_OUT_OFFSET, v)

        for i in range(num_gpio):
            pin_output = int(dut.gpio_o.value) & ~(int(dut.gpio_t.value))
            assert (pin_output & (1 << i)) == (v & (1 << i))

    await do_soft_rst(tb, baseaddr)

    # test input
    for test_vector in test_vectors:
        v = bmask & test_vector
        dut.gpio_i.value = v

        result = await tb.axil_master.read_dword(baseaddr + A32V_GPIO_IN_OFFSET)
        assert result == v


tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '../../rtl'))


@pytest.mark.parametrize("baseaddr", [0, 0xff00])
def test_gpio_axil(request, baseaddr):
    dut = "gpio_axil"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_files = [
        f"{dut}.v",
    ]
    verilog_sources = [os.path.join(rtl_dir, x) for x in verilog_files]

    parameters = {}
    parameters["NUM_GPIO"] = 32
    parameters["AXIL_ADDR_WIDTH"] = 16
    parameters["AXIL_ADDR_BASE"] = baseaddr

    extra_env = {f'PARAM_{k}': str(v) for k,v in parameters.items()}

    sim_build = os.path.join(tests_dir, "sim_build",
                             request.node.name.replace('[', '-').replace(']', ''))

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=toplevel,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )
