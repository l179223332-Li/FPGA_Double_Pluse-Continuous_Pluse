################## 设备配置电压 ##################
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

# LED输出（锁定状态指示）
set_property PACKAGE_PIN F19 [get_ports led_out1]
set_property IOSTANDARD LVCMOS33 [get_ports led_out1]
set_property PACKAGE_PIN E21 [get_ports led_out2]
set_property IOSTANDARD LVCMOS33 [get_ports led_out2]
set_property PACKAGE_PIN D20 [get_ports led_out3]
set_property IOSTANDARD LVCMOS33 [get_ports led_out3]
set_property PACKAGE_PIN C20 [get_ports led_out4]
set_property IOSTANDARD LVCMOS33 [get_ports led_out4]

set_property PACKAGE_PIN Y18 [get_ports sys_clk]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk]
set_property IOSTANDARD LVCMOS33 [get_ports key1]
set_property PACKAGE_PIN M13 [get_ports key1]
set_property PACKAGE_PIN K14 [get_ports key2]
set_property IOSTANDARD LVCMOS33 [get_ports key2]

set_property PACKAGE_PIN K13 [get_ports reset_key]
set_property IOSTANDARD LVCMOS33 [get_ports reset_key]
set_property PACKAGE_PIN F20 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

set_property IOSTANDARD LVCMOS33 [get_ports PWM1]
set_property IOSTANDARD LVCMOS33 [get_ports PWM2]
set_property IOSTANDARD LVCMOS33 [get_ports PWM3]
set_property IOSTANDARD LVCMOS33 [get_ports PWM4]
set_property PACKAGE_PIN C15 [get_ports PWM1]
set_property PACKAGE_PIN C14 [get_ports PWM2]
set_property PACKAGE_PIN A16 [get_ports PWM3]
set_property PACKAGE_PIN A15 [get_ports PWM4]

set_property PACKAGE_PIN B18 [get_ports st1_off]
set_property IOSTANDARD LVCMOS33 [get_ports st1_off]
set_property PACKAGE_PIN B17 [get_ports st2_off]
set_property IOSTANDARD LVCMOS33 [get_ports st2_off]
set_property PACKAGE_PIN A19 [get_ports st3_off]
set_property IOSTANDARD LVCMOS33 [get_ports st3_off]
set_property PACKAGE_PIN A18 [get_ports st4_off]
set_property IOSTANDARD LVCMOS33 [get_ports st4_off]

set_property PACKAGE_PIN C19 [get_ports st1_on]
set_property IOSTANDARD LVCMOS33 [get_ports st1_on]
set_property PACKAGE_PIN C18 [get_ports st2_on]
set_property IOSTANDARD LVCMOS33 [get_ports st2_on]
set_property PACKAGE_PIN A20 [get_ports st3_on]
set_property IOSTANDARD LVCMOS33 [get_ports st3_on]
set_property PACKAGE_PIN B20 [get_ports st4_on]
set_property IOSTANDARD LVCMOS33 [get_ports st4_on]

#设置IO口快速响应
set_property SLEW FAST [get_ports PWM1]
set_property SLEW FAST [get_ports PWM2]
set_property SLEW FAST [get_ports PWM3]
set_property SLEW FAST [get_ports PWM4]

#############SPI Configurate Setting##################
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
