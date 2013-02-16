#
# Copyright (C) 2013 Vlad Lazarenko <vlad@lazarenko.me>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

set PROJECT_NAME $env(PROJECT_NAME)

project_new ${PROJECT_NAME} -overwrite

set_global_assignment -name TOP_LEVEL_ENTITY ${PROJECT_NAME}_top

set_global_assignment -name FAMILY "Arria II GX"
set_global_assignment -name DEVICE EP2AGX260FF35I3

set_global_assignment -name MIN_CORE_JUNCTION_TEMP "0"
set_global_assignment -name MAX_CORE_JUNCTION_TEMP "85"
set_global_assignment -name DEVICE_FILTER_PACKAGE FBGA
set_global_assignment -name OPTIMIZATION_TECHNIQUE SPEED

set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "2.5 V"
set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP "AS INPUT TRI-STATED"
set_global_assignment -name POWER_PRESET_COOLING_SOLUTION "23 MM HEAT SINK WITH 200 LFPM AIRFLOW"
set_global_assignment -name POWER_BOARD_THERMAL_MODEL "NONE (CONSERVATIVE)"

set_global_assignment -name AUTO_CLOCK_ENABLE_RECOGNITION On
set_global_assignment -name REMOVE_DUPLICATE_REGISTERS On
set_global_assignment -name ENABLE_IP_DEBUG Off
set_global_assignment -name STATE_MACHINE_PROCESSING Auto
set_global_assignment -name EXTRACT_VERILOG_STATE_MACHINES On
set_global_assignment -name VERILOG_CONSTANT_LOOP_LIMIT 5000
set_global_assignment -name VERILOG_NON_CONSTANT_LOOP_LIMIT 250
set_global_assignment -name PARALLEL_SYNTHESIS On
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL

set_global_assignment -name PHYSICAL_SYNTHESIS_ASYNCHRONOUS_SIGNAL_PIPELINING ON
set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION ON
set_global_assignment -name OPTIMIZE_HOLD_TIMING "ALL PATHS"
set_global_assignment -name FITTER_EFFORT "STANDARD FIT"
set_global_assignment -name SMART_RECOMPILE ON

### Setup timing constraints.
set_global_assignment -name SDC_FILE ${PROJECT_NAME}.sdc

### Source files..
set RTL_FILES $env(RTL_FILES)
foreach f ${RTL_FILES} {
    set_global_assignment -name VERILOG_FILE ${f}
}

### QIP files...
set QIP_FILES $env(QIP_FILES)
foreach f ${QIP_FILES} {
    set_global_assignment -name QIP_FILE ${f}
}

### PIN assignments ###

# 100 MHz Clock
set_location_assignment PIN_AJ19 -to clk_top_100_p
set_location_assignment PIN_AK19 -to clk_top_100_n
set_instance_assignment -name IO_STANDARD LVDS -to clk_top_100_p

# 125 MHz Clock
set_location_assignment PIN_F18 -to clkin_top_p
set_instance_assignment -name IO_STANDARD LVDS -to clkin_top_p

# Reset button (PB1, active low)
set_location_assignment PIN_AL7 -to global_reset_n
set_instance_assignment -name IO_STANDARD "1.8 V" -to global_reset_n
set_instance_assignment -name IO_MAXIMUM_TOGGLE_RATE "0 MHz" -to global_reset_n

# Ethernet RGMII
set_location_assignment PIN_D25 -to eth_tx_clk
set_location_assignment PIN_V6 -to eth_rx_clk
set_location_assignment PIN_D17 -to eth_rx_ctrl
set_location_assignment PIN_G20 -to eth_tx_ctrl
set_location_assignment PIN_M20 -to eth_reset_n
set_location_assignment PIN_E21 -to eth_rgmii_in[0]
set_location_assignment PIN_E24 -to eth_rgmii_in[1]
set_location_assignment PIN_E22 -to eth_rgmii_in[2]
set_location_assignment PIN_F24 -to eth_rgmii_in[3]
set_location_assignment PIN_J20 -to eth_rgmii_out[0]
set_location_assignment PIN_C25 -to eth_rgmii_out[1]
set_location_assignment PIN_G22 -to eth_rgmii_out[2]
set_location_assignment PIN_G21 -to eth_rgmii_out[3]

set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_rx_ctrl
set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_tx_ctrl
set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_tx_clk
set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_rx_clk
set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_rgmii_in
set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_reset_n
set_instance_assignment -name IO_STANDARD "2.5 V" -to eth_rgmii_out

# Ethernet MDIO
set_location_assignment PIN_K20 -to eth_mdc
set_location_assignment PIN_N20 -to eth_mdio

project_close
