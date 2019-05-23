# The emergency button for the high voltage is triggered.
bool high_voltage_emergency_button_triggered

# The current consumed by the power distribution board.
float64 power_distribution_board_current

# The total current of all high voltage nets combined.
float64 high_voltage_net_current

# The request of the power distribution board to shutdown the master.
bool master_shutdown_requested

LowVoltageNet[] low_voltage_nets
HighVoltageNet[] high_voltage_nets

