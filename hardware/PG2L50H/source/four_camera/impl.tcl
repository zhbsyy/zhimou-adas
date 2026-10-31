set_arch -family Logos2 -device PG2L50H -speedgrade -6 -package FBG484
compile -top_module image_pcie_capture
synthesize -ads -selected_syn_tool_opt 2
dev_map
pnr
report_timing
report_power
gen_netlist
gen_bit_stream -compress_bitstream true -master_configuration_clock_frequency {40M}
set_arch -family Logos2 -device PG2L50H -speedgrade -6 -package FBG484
compile -top_module image_pcie_capture
synthesize -ads -selected_syn_tool_opt 2 
dev_map 
pnr -gplace_seed 2 -groute_seed 2 -optimize_multi_corner_timing 
report_timing 
report_power 
gen_netlist 
gen_bit_stream -compress_bitstream true -master_configuration_clock_frequency {40M} 
