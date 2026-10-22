# GTKWave Tcl script for tb_i2c_smbus_timeout_fastmode.vcd
# Load with: gtkwave tb_i2c_smbus_timeout_fastmode.vcd -S add_tb_i2c_smbus_timeout_fastmode_signals.tcl
#
# Note: this GTKWave build's Tcl API does not expose setWindowPrefix /
# getCurrentDump / setNewRadix (confirmed via `info commands gtkwave::*`),
# so this script sticks to commands that actually exist in it:
# addSignalsFromList, highlightSignalsFromList, and the Zoom_Full menu action.

# Top-level bus and testbench control signals
gtkwave::addSignalsFromList [list \
    tb_i2c_smbus_timeout_fastmode.clk \
    tb_i2c_smbus_timeout_fastmode.rst_n \
    tb_i2c_smbus_timeout_fastmode.scl \
    tb_i2c_smbus_timeout_fastmode.sda \
    tb_i2c_smbus_timeout_fastmode.tb_scl_force_low \
    tb_i2c_smbus_timeout_fastmode.tb_sda_force_low \
    tb_i2c_smbus_timeout_fastmode.mst_busy \
    tb_i2c_smbus_timeout_fastmode.mst_done \
    tb_i2c_smbus_timeout_fastmode.mst_ack_error \
    tb_i2c_smbus_timeout_fastmode.mst_arb_lost \
    tb_i2c_smbus_timeout_fastmode.mst_smbus_timeout \
    tb_i2c_smbus_timeout_fastmode.mst_addr \
    tb_i2c_smbus_timeout_fastmode.mst_rw \
    tb_i2c_smbus_timeout_fastmode.mst_data_tx \
    tb_i2c_smbus_timeout_fastmode.mst_data_rx \
    tb_i2c_smbus_timeout_fastmode.mst_num_bytes \
    tb_i2c_smbus_timeout_fastmode.slv_addressed \
    tb_i2c_smbus_timeout_fastmode.slv_dir_rw \
    tb_i2c_smbus_timeout_fastmode.slv_busy \
    tb_i2c_smbus_timeout_fastmode.slv_rx_data \
    tb_i2c_smbus_timeout_fastmode.slv_rx_valid \
    tb_i2c_smbus_timeout_fastmode.slv_rx_ready \
    tb_i2c_smbus_timeout_fastmode.slv_tx_data \
    tb_i2c_smbus_timeout_fastmode.slv_tx_valid \
    tb_i2c_smbus_timeout_fastmode.slv_tx_ready \
]

# Master FSM and data-path signals
gtkwave::addSignalsFromList [list \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.state \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.addr \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.rw \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.data_tx \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.data_rx \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.bit_cnt \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.byte_cnt \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.busy \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.done \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.ack_error \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.arbitration_lost \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.shift_reg \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.scl_oe \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.sda_oe \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.smbus_timeout \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.pec_error \
]

# Slave FSM, stretch, ARP, PEC, and SMBALERT# signals
gtkwave::addSignalsFromList [list \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.state \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.rx_data \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.rx_valid \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.tx_ready \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.addressed \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.dir_rw \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.busy \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.bit_pos \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.shift_reg \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.stretch_active \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.scl_oe_slv \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.sda_oe \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.my_addr \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.ack_sent \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.arp_get_udid_pending \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.arp_cmd \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.assigned_addr \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.av \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.def_addr \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.own_arp_write \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.ara_addressed \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.pec_error \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.smbalert_asserted \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.alert_pending \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.start_detected \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.stop_detected \
]

# Highlight the timeout/stretch/alert signals for easy identification
gtkwave::highlightSignalsFromList [list \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_master.smbus_timeout \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.stretch_active \
    tb_i2c_smbus_timeout_fastmode.u_i2c_top.u_i2c_slave.smbalert_asserted \
]

# Zoom to fit all data
gtkwave::/Time/Zoom/Zoom_Full
