# Input Status Registers (command 2, Read Discrete Inputs)

Returns a BIT value (on/off) for the following configuration items:

10001|TMPOL
10002|SHNTOL
10003|SHNTUL
10004|BUSOL
10005|BUSUL
10006|POL
10007|Temperature compensation
10008|ADC Range 0 = ±163.84 mV, 1 = ± 40.96 mV
10009|Relay Trigger on TMPOL
10010|Relay Trigger on SHNTOL
10011|Relay Trigger on SHNTUL
10012|Relay Trigger on BUSOL
10013|Relay Trigger on BUSUL
10014|Relay Trigger on POL
10015|Existing Relay state

# Holding Registers (command 4)

Returns 16 bit word value for the following configuration items.  Some items are split over two registers and will need to recombined before use.
For valid results you must read both registers as part of the same request, otherwise invalid/corrupt data will be returned.

40001|Voltage (4 byte double)
40002|Voltage
40003|Current (4 byte double)
40004|Current
40005|amphour_out (4 byte double)
40006|amphour_out
40007|amphour_in (4 byte double)
40008|amphour_in
40009|temperature (signed int16)
40010|Watchdog timer trigger count (like error counter) (unsigned int16)
40011|Power (4 byte double)
40012|Power
40013|Shunt mV (4 byte double)
40014|Shunt mV
40015|CURRENT_LSB (4 byte double)
40016|CURRENT_LSB
40017|shunt_resistance (4 byte double)
40018|shunt_resistance
40019|shunt_max_current  (unsigned int16)
40020|shunt_millivolt  (unsigned int16)
40021|INA_REGISTER::SHUNT_CAL (unsigned int16)
40022|Temperature limit (signed int16)
40023|Bus Overvoltage (overvoltage protection)(4 byte double)
40024|Bus Overvoltage (overvoltage protection)
40025|BusUnderVolt (4 byte double)
40026|BusUnderVolt
40027|Shunt Over Voltage Limit (current limit) (4 byte double)
40028|Shunt Over Voltage Limit (current limit) 
40029|Shunt UNDER Voltage Limit (under current limit) (4 byte double)
40030|Shunt UNDER Voltage Limit (under current limit)
40031|Shunt Over POWER LIMIT (4 byte double)
40032|Shunt Over POWER LIMIT

