/*
 * Copyright (C) 2013 Vlad Lazarenko <vlad@lazarenko.me>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

module a2gx_top
  (// Clock & Reset Signals
   input            clk_top_100_p,
   input            clk_top_125_p,
   input            global_reset_n,
   // PCI Express Signals
	input            pcie_rstn,
	input            pcie_refclk,
	input            pcie_rx_in0,
	input            pcie_rx_in1,
	input            pcie_rx_in2,
	input            pcie_rx_in3,
	output           pcie_tx_out0,
	output           pcie_tx_out1,
	output           pcie_tx_out2,
	output           pcie_tx_out3,
   output reg [3:0] pcie_led,
   // Ethernet Signals
   input            eth_rx_clk,
   input            eth_rx_ctrl,
   output           eth_tx_clk,
   output           eth_tx_ctrl,
   input [3:0]      eth_rgmii_in,
   output [3:0]     eth_rgmii_out,
   output           eth_reset_n,
   output           eth_mdc,
   inout            eth_mdio);

   wire   clk_50;
   wire   clk_125;
   wire   reconfig_pll_lock;

   wire [16:0] reconfig_fromgxb;
   wire [3:0]  reconfig_togxb;
   wire        altgx_reconfig_busy;

   wire        eth_mdio_in;
   wire        eth_mdio_out;
   wire        eth_mdio_oen;

	wire [63:0] test_out_icm;
	wire [39:0] test_in;

   assign eth_reset_n = global_reset_n;

   assign eth_mdio = eth_mdio_oen ? 1'bz : eth_mdio_out;
   assign eth_mdio_in = eth_mdio;

   reconfig_pll reconfig_pll
     (.inclk0(clk_top_100_p),
      .c0(clk_50),
      .c1(clk_125),
      .locked(reconfig_pll_lock));

   altgx_reconfig altgx_reconfig
     (.offset_cancellation_reset(!reconfig_pll_lock),
      .reconfig_clk(clk_50),
      .reconfig_fromgxb(reconfig_fromgxb),
      .busy(altgx_reconfig_busy),
      .reconfig_togxb(reconfig_togxb));

   a2gx_qsys a2gx_qsys
     (.clk_50_clk(clk_50),
      .clk_125_clk(clk_top_125_p),
      .reset_50_reset_n(global_reset_n),
      .reset_125_reset_n(global_reset_n),
      .pcie_refclk_export(pcie_refclk),
      .pcie_fixedclk_clk(clk_125),
		.pcie_rx_in_rx_datain_0(pcie_rx_in0),
      .pcie_rx_in_rx_datain_1(pcie_rx_in1),
      .pcie_rx_in_rx_datain_2(pcie_rx_in2),
      .pcie_rx_in_rx_datain_3(pcie_rx_in3),
      .pcie_tx_out_tx_dataout_0(pcie_tx_out0),
      .pcie_tx_out_tx_dataout_1(pcie_tx_out1),
      .pcie_tx_out_tx_dataout_2(pcie_tx_out2),
      .pcie_tx_out_tx_dataout_3(pcie_tx_out3),
      .pcie_reconfig_busy_busy_altgxb_reconfig(altgx_reconfig_busy),
      .pcie_reconfig_togxb_data(reconfig_togxb),
      .pcie_reconfig_fromgxb_0_data(reconfig_fromgxb),
      .pcie_pcie_rstn_export(pcie_rstn),
      .pcie_test_in_test_in(test_in),
      .pcie_test_out_test_out(test_out_icm),
      .ethernet_conduit_connection_rgmii_in(eth_rgmii_in),
      .ethernet_conduit_connection_rgmii_out(eth_rgmii_out),
      .ethernet_conduit_connection_rx_control(eth_rx_ctrl),
      .ethernet_conduit_connection_tx_control(eth_tx_ctrl),
      .ethernet_conduit_connection_tx_clk(clk_125),
      .ethernet_conduit_connection_rx_clk(eth_rx_clk),
      .ethernet_conduit_connection_set_10(1'b0),
      .ethernet_conduit_connection_set_1000(1'b1),
      .ethernet_conduit_connection_ena_10(1'b0),
      .ethernet_conduit_connection_eth_mode(),
      .ethernet_conduit_connection_mdio_out(eth_mdio_out),
      .ethernet_conduit_connection_mdio_oen(eth_mdio_oen),
      .ethernet_conduit_connection_mdio_in(eth_mdio_in),
      .ethernet_conduit_connection_mdc(eth_mdc));

	altddio_out	eth_tx_clk_ddio
     (.aclr(!global_reset_n),
		.datain_h(1'b1),
		.datain_l(1'b0),
		.oe(1'b1),
		.outclock(clk_125),
		.outclocken(1'b1),
		.dataout(eth_tx_clk),
		.aset(1'b0),
		.oe_out(),
		.sclr(1'b0),
		.sset(1'b0));
	defparam
	  eth_tx_clk_ddio.extend_oe_disable = "OFF",
	  eth_tx_clk_ddio.intended_device_family = "Arria II GX",
	  eth_tx_clk_ddio.invert_output = "OFF",
	  eth_tx_clk_ddio.lpm_hint = "UNUSED",
	  eth_tx_clk_ddio.lpm_type = "altddio_out",
	  eth_tx_clk_ddio.oe_reg = "UNREGISTERED",
	  eth_tx_clk_ddio.power_up_high = "OFF",
	  eth_tx_clk_ddio.width = 1;

   always @(posedge clk_125 or negedge global_reset_n) begin
      if (global_reset_n == 1'b0) begin
         pcie_led[3:0] <= 4'b0;
      end else begin
         pcie_led[3:0] <= ~(test_out_icm[28:25]);
      end
   end

endmodule
