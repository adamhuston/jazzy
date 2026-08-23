# Aggregate framework status published on the alive loop.

uint8 INIT=0
uint8 READY=1
uint8 ACTIVE=2
uint8 DEGRADED=3
uint8 FAULT=4
uint8 SHUTDOWN=5

builtin_interfaces/Time stamp    # heartbeat timestamp
uint8 state                      # framework state constant above
string state_label               # textual state
uint32 loop_count                # alive loop iterations since activation
float64 loop_period_ms           # configured alive loop period
float64 loop_jitter_ms           # last measured loop jitter
PluginStatus[] plugins           # per-plugin health
