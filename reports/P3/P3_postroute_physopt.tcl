# P3 post-route timing optimization (Vivado 2023.2)
# Source design: existing routed impl_1
# Changes: in-memory physical optimization only (NO RTL changes).
# Baseline impl_1 run/checkpoint is NOT reset or overwritten.

set report_dir "D:/2026PRJ/fpga-ai-accelerator-main/fpga-ai-accelerator-main/reports/P3/physopt"
file mkdir $report_dir

# Open the baseline routed design.
open_run impl_1

puts "INFO: Saving baseline routed design and timing reports..."
write_checkpoint -force [file join $report_dir p3_before_physopt.dcp]
report_timing_summary -delay_type min_max -file [file join $report_dir timing_summary_before.rpt]
report_timing -delay_type max -max_paths 20 -path_type full \
    -file [file join $report_dir setup_paths_before.rpt]

# Physically optimize the already-routed design.
# Vivado reroutes any changed routes as part of post-route phys_opt_design.
puts "INFO: Running post-route physical optimization..."
phys_opt_design -directive AggressiveExplore

# Preserve the optimized result independently from impl_1.
write_checkpoint -force [file join $report_dir p3_after_physopt.dcp]

# Check both setup and hold after optimization.
report_timing_summary -delay_type min_max \
    -file [file join $report_dir timing_summary_after.rpt]
report_timing -delay_type max -max_paths 20 -path_type full \
    -file [file join $report_dir setup_paths_after.rpt]
report_timing -delay_type min -max_paths 20 -path_type full \
    -file [file join $report_dir hold_paths_after.rpt]
report_utilization -file [file join $report_dir utilization_after.rpt]
report_route_status -file [file join $report_dir route_status_after.rpt]

puts "=================================================="
puts "P3 post-route physical optimization completed."
puts "Baseline run: impl_1 (unchanged)"
puts "Outputs: $report_dir"
puts "Compare timing_summary_before.rpt and timing_summary_after.rpt"
puts "PASS criteria: Setup WNS >= 0, TNS = 0; Hold WHS >= 0, THS = 0"
puts "=================================================="
