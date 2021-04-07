# Input Status Registers (command 2, Read Discrete Inputs)

Returns a BIT value (on/off) for the following configuration items:

10001|TMPOL|Read only
10002|SHNTOL|Read only
10003|SHNTUL|Read only
10004|BUSOL|Read only
10005|BUSUL|Read only
10006|POL|Read only
10007|Temperature compensation enabled|Read write
10008|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only
10009|Relay Trigger on TMPOL|Read write
10010|Relay Trigger on SHNTOL|Read write
10011|Relay Trigger on SHNTUL|Read write
10012|Relay Trigger on BUSOL|Read write
10013|Relay Trigger on BUSUL|Read write
10014|Relay Trigger on POL|Read write
10015|Existing Relay state (0=off)|Read write
10016|Factory reset bit (always 0 when read)|Read write

Registers 1 to 6 indicate the current alert state of the INA228 chip, zero means not alerted.  The RED LED will light when any of the alert states are non-zero.

Registers 9 to 14 control which alerts will trigger the relay to turn on.  Value of 1 means the alert is active/enabled.

Register 15 returns the current relay state, and allows the relay to be manually controlled

Register 16 always returns zero when read, set to 1 to perform a factory reset on the monitor, returning all values to defaults.


# Holding Registers (command 3)

Returns 16 bit word value for the following configuration items.  Some items are split over two registers and will need to recombined before use.
For valid results you must read both registers as part of the same request, otherwise invalid/corrupt data will be returned.

All registers are read only, unless also specified in "Write Registers" later on

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
40033|Shunt Temperature Coefficient (SHUNT_TEMPCO) (unsigned int16)
40034|INAXXX chip model number (should always be 0x0228)
40035|GITHUB version
40036|GITHUB version
40037|COMPILE_DATE_TIME_EPOCH
40038|COMPILE_DATE_TIME_EPOCH
40039|Spare (not used)
40040|DEBUG CONFIG
40041|DEBUG ADC_CONFIG
40042|DEBUG SHUNT_CAL
40043|DEBUG SHUNT_TEMPCO
40044|DEBUG DIAG_ALRT
40045|DEBUG SOVL
40046|DEBUG SUVL
40047|DEBUG BOVL
40049|DEBUG BUVL
40050|DEBUG TEMP_LIMIT
40051|DEBUG PWR_LIMIT
40052|DEBUG DIETEMP


# 0x16 Write Multiple Registers

40005/40006|amphour_out|Set to zero to reset
40007/40008|amphour_in|Set to zero to reset
40010|Watchdog timer trigger count (like error counter) (unsigned int16)

40019|shunt_max_current  (unsigned int16) |e.g. 150
40020|shunt_millivolt  (unsigned int16) |e.g. 50
40021|SHUNT_CAL (unsigned int16) **

40022|Temperature limit (signed int16)|degrees C)
40023/40024|Bus Overvoltage (overvoltage protection)(4 byte double)|Volts
40025/40026|Bus Under Volt (4 byte double) |Volts
40027/40028|Shunt Over Voltage Limit (current limit) (4 byte double)|Current
40029/40030|Shunt UNDER Voltage Limit (under current limit) (4 byte double)|Current
40031/40032|Shunt Over POWER LIMIT (4 byte double)|Power

** For normal configuration, just set the shunt maximum current and millivolt registers.  Shunt_Cal is calculated as needed based on those values.  For fine tuning calibration, the SHUNT_CAL register can be written to/adjusted.  Once a register is set, it is stored in EEPROM and used when shunt is powered up.

