// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2022 Spencer Chang

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * SPI master AXI lite wrapper
 */
module gpio_axil #
(
    parameter NUM_GPIO = 1, // 1-32 allowed

    parameter AXIL_ADDR_WIDTH = 16,
    parameter AXIL_ADDR_BASE = 0,
    parameter RB_NEXT_PTR = 0
)
(
    input  wire                         clk,
    input  wire                         rst,

    /*
     * Host interface
     */
    input  wire [AXIL_ADDR_WIDTH-1:0]   s_axil_awaddr,
    input  wire [2:0]                   s_axil_awprot,
    input  wire                         s_axil_awvalid,
    output wire                         s_axil_awready,
    input  wire [31:0]                  s_axil_wdata,
    input  wire [3:0]                   s_axil_wstrb,
    input  wire                         s_axil_wvalid,
    output wire                         s_axil_wready,
    output wire [1:0]                   s_axil_bresp,
    output wire                         s_axil_bvalid,
    input  wire                         s_axil_bready,
    input  wire [AXIL_ADDR_WIDTH-1:0]   s_axil_araddr,
    input  wire [2:0]                   s_axil_arprot,
    input  wire                         s_axil_arvalid,
    output wire                         s_axil_arready,
    output wire [31:0]                  s_axil_rdata,
    output wire [1:0]                   s_axil_rresp,
    output wire                         s_axil_rvalid,
    input  wire                         s_axil_rready,

    /*
     * GPIO interface
     */
    output wire                         irq,
    input  wire [NUM_GPIO-1:0]          gpio_i,
    output wire [NUM_GPIO-1:0]          gpio_t,
    output wire [NUM_GPIO-1:0]          gpio_o
);
/*

ID Register: (AXIL_ADDR_BASE + 0x00) (RO)

Identifies that this register section contains an GPIO block: 0x294E_C110


-----------------------------------------------------------------------
Revision Register: (AXIL_ADDR_BASE + 0x04) (RO)

Identifies the revision of the HDL code: 0x0000_0100


-----------------------------------------------------------------------
Pointer Register: (AXIL_ADDR_BASE + 0x08) (RO)

Contains the data specified by the parameter RB_NEXT_PTR. The intended
use is for the main driver to automatically load the respective sub-drivers
using the ID and the pointer to the next driver to be loaded.

Default: 0 (settable by parameter RB_NEXT_PTR)

-----------------------------------------------------------------------
Software Reset Register: (AXIL_ADDR_BASE + 0x10) (WO)

Write 0x0000_000A to perform a software reset

-----------------------------------------------------------------------
GPIO Info Register: (AXIL_ADDR_BASE + 0x14) (RO)

The number of GPIOs available to use.

-----------------------------------------------------------------------
GPIO Data Direction Register: (AXIL_ADDR_BASE + 0x20) (RW)

The data direction register of length N (determined by NUM_GPIO). A set bit
indicates that that GPIO should be an output. This register is 0x00 on reset.

Bit  31-N: Reserved
Bit N-1:0: A 1 bit indicates that the corresponding GPIO is an output.

-----------------------------------------------------------------------
GPIO Data Out Register: (AXIL_ADDR_BASE + 0x24) (RW)

The data output register of length N (determined by NUM_GPIO). When both the
DDR and OUT bit is set, a data high is asserted on the pin. If the DDR is set
while the OUT bit is unset, a data low is asserted on the corresponding pin.

-----------------------------------------------------------------------
GPIO Data In Register: (AXIL_ADDR_BASE + 0x28) (RO)

The data input register of length N (determined by NUM_GPIO). When a bit is set,
this indicates that a logic 1 is asserted on the pin. If the bit is unset, a
logic 0 is asserted on the pin.

*/

// AXIL
reg s_axil_awready_reg = 1'b0, s_axil_awready_next;
reg s_axil_wready_reg = 1'b0, s_axil_wready_next;
reg s_axil_bvalid_reg = 1'b0, s_axil_bvalid_next;
reg s_axil_arready_reg = 1'b0, s_axil_arready_next;
reg [31:0] s_axil_rdata_reg = 32'd0, s_axil_rdata_next;
reg s_axil_rvalid_reg = 1'b0, s_axil_rvalid_next;
reg do_axil_write, do_axil_read;

assign s_axil_awready = s_axil_awready_reg;
assign s_axil_wready = s_axil_wready_reg;
assign s_axil_bresp = 2'b00;
assign s_axil_bvalid = s_axil_bvalid_reg;

assign s_axil_arready = s_axil_arready_reg;
assign s_axil_rdata = s_axil_rdata_reg;
assign s_axil_rresp = 2'b00;
assign s_axil_rvalid = s_axil_rvalid_reg;

// AXIS - For Transmitting
reg [31:0] axis_write_tdata = 32'd0;
reg axis_write_tvalid = 1'b0, axis_write_tvalid_next;
wire axis_write_tready;

wire [31:0] axis_write_ext_tdata;
wire axis_write_ext_tvalid;
wire axis_write_ext_tready;

// AXIS - For Receiving
wire [31:0] axis_read_tdata;
wire axis_read_tvalid;
reg  axis_read_tready = 1'b0;

wire [31:0] axis_read_ext_tdata;
wire axis_read_ext_tvalid;
wire axis_read_ext_tready;

// software reset
reg software_rst = 0;


reg [31:0] data_direct_reg = 32'b0;
reg [31:0] data_output_reg = 32'b0;
reg [31:0]  data_input_reg = 32'b0;

assign gpio_o = data_output_reg[NUM_GPIO-1:0];
assign gpio_t = ~data_output_reg[NUM_GPIO-1:0];


/*
 * AXIL Write Transaction
 */

always @* begin

    axis_write_tvalid_next = axis_write_tready ? 1'b0 : axis_write_tvalid;

    // axil write transaction
    do_axil_write = 1'b0;

    s_axil_awready_next = 1'b0;
    s_axil_wready_next = 1'b0;
    s_axil_bvalid_next = s_axil_bvalid_reg && !s_axil_bready;

    if (s_axil_awvalid && s_axil_wvalid && (!s_axil_bvalid || s_axil_bready) && (!s_axil_awready && !s_axil_wready)) begin
        s_axil_awready_next = 1'b1;
        s_axil_wready_next = 1'b1;
        s_axil_bvalid_next = 1'b1;

        do_axil_write = 1'b1;
    end
end
always @(posedge clk) begin
    if (rst || software_rst) begin
        s_axil_awready_reg <= 1'b0;
        s_axil_wready_reg <= 1'b0;
        s_axil_bvalid_reg <= 1'b0;
        software_rst <= 1'b0;
        axis_write_tvalid <= 1'b0;

        // restore defaults
        data_output_reg <= 32'b0;
        data_input_reg <= 32'b0;
        data_direct_reg <= 32'b0;

    end else begin
        axis_write_tvalid <= axis_write_tvalid_next;

        s_axil_awready_reg <= s_axil_awready_next;
        s_axil_wready_reg <= s_axil_wready_next;
        s_axil_bvalid_reg <= s_axil_bvalid_next;

        data_input_reg[NUM_GPIO-1:0] <= gpio_i;

        if (do_axil_write) begin
            case ({s_axil_awaddr >> 2, 2'b00})

                // Software Reset Register
                AXIL_ADDR_BASE+8'h10: begin
                    if (s_axil_wdata == 32'h0000000A) begin
                        software_rst <= 1'b1;
                    end
                end

                // GPIO Data Direction Register
                AXIL_ADDR_BASE+8'h20: begin
                    if (s_axil_wstrb[0]) begin
                        data_direct_reg[7:0] <= s_axil_wdata[7:0];
                    end
                    if (s_axil_wstrb[1]) begin
                        data_direct_reg[15:8] <= s_axil_wdata[15:8];
                    end
                    if (s_axil_wstrb[2]) begin
                        data_direct_reg[23:16] <= s_axil_wdata[23:16];
                    end
                    if (s_axil_wstrb[3]) begin
                        data_direct_reg[31:24] <= s_axil_wdata[31:24];
                    end
                end

                // GPIO Data Output Register
                AXIL_ADDR_BASE+8'h24: begin
                    if (s_axil_wstrb[0]) begin
                        data_output_reg[7:0] <= s_axil_wdata[7:0];
                    end
                    if (s_axil_wstrb[1]) begin
                        data_output_reg[15:8] <= s_axil_wdata[15:8];
                    end
                    if (s_axil_wstrb[2]) begin
                        data_output_reg[23:16] <= s_axil_wdata[23:16];
                    end
                    if (s_axil_wstrb[3]) begin
                        data_output_reg[31:24] <= s_axil_wdata[31:24];
                    end
                end

                default: ;
            endcase
        end // do_axil_write
    end
end

/*
 * AXIL Read Transaction
 */
always @* begin
    do_axil_read = 1'b0;

    s_axil_arready_next = 1'b0;
    s_axil_rvalid_next = s_axil_rvalid_reg && !s_axil_rready;

    if (s_axil_arvalid && (!s_axil_rvalid || s_axil_rready) && (!s_axil_arready)) begin
        s_axil_arready_next = 1'b1;
        s_axil_rvalid_next = 1'b1;

        do_axil_read = 1'b1;
    end
end
always @(posedge clk) begin
    if (rst || software_rst) begin
        s_axil_arready_reg <= 1'b0;
        s_axil_rvalid_reg <= 1'b0;
    end else begin
        axis_read_tready <= 1'b0;

        s_axil_arready_reg <= s_axil_arready_next;
        s_axil_rvalid_reg <= s_axil_rvalid_next;
        s_axil_rdata_reg <= 32'd0;

        if (do_axil_read) begin
            case ({s_axil_araddr >> 2, 2'b00})
                // ID Register
                AXIL_ADDR_BASE+8'h00: s_axil_rdata_reg <= 32'h294EC110;

                // Revision Register
                AXIL_ADDR_BASE+8'h04: s_axil_rdata_reg <= 32'h00000100;

                // Pointer Register
                AXIL_ADDR_BASE+8'h08: s_axil_rdata_reg <= RB_NEXT_PTR;

                // GPIO Info Register
                AXIL_ADDR_BASE+8'h14: s_axil_rdata_reg <= NUM_GPIO;

                // GPIO Data Direction Register
                AXIL_ADDR_BASE+8'h20: s_axil_rdata_reg <= data_direct_reg;

                // GPIO Data Output Register
                AXIL_ADDR_BASE+8'h24: s_axil_rdata_reg <= data_output_reg;

                // GPIO Data Input Register
                AXIL_ADDR_BASE+8'h28: s_axil_rdata_reg <= data_input_reg;

                default: ;
            endcase
        end // do_axil_read
    end
end

endmodule
`resetall