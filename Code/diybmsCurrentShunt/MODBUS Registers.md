
# Holding Registers (command 3)

Returns 16 bit word value for the following configuration items.  Some items are split over two registers and will need to recombined before use.
For valid results you must read both registers as part of the same request, otherwise invalid/corrupt data will be returned.

All registers are read only, unless also specified in "Write Registers" later on

|Register|Description|
|--------|-----------|
|40001|Voltage (4 byte double)
|40002|Voltage
|40003|Current (4 byte double)
|40004|Current
|40005|milliamphour_out (4 byte unsigned long uint32_t)
|40006|milliamphour_out
|40007|milliamphour_in (4 byte  unsigned long uint32_t)
|40008|milliamphour_in
|40009|temperature (signed int16)
|40010|Various status flags (see below)
|40011|Power (4 byte double)
|40012|Power
|40013|Shunt mV (4 byte double)
|40014|Shunt mV
|40015|CURRENT_LSB (4 byte double)
|40016|CURRENT_LSB
|40017|shunt_resistance (4 byte double)
|40018|shunt_resistance
|40019|shunt_max_current  (unsigned int16)
|40020|shunt_millivolt  (unsigned int16)
|40021|Battery Capacity (ah)  (unsigned int16)
|40022|Fully charged voltage (4 byte double)
|40023|Fully charged voltage
|40024|Tail current (Amps) (4 byte double)
|40025|Tail current (Amps)
|40026|Charge efficiency factor % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 9561 = 95.61%)
|40027|State of charge % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 8012 = 80.12%, 100 = 1.00%)
|40028|INA_REGISTER::SHUNT_CAL (unsigned int16)
|40029|Temperature limit (signed int16)
|40030|Bus Overvoltage (overvoltage protection)(4 byte double)
|40031|Bus Overvoltage (overvoltage protection)
|40032|BusUnderVolt (4 byte double)
|40033|BusUnderVolt
|40034|Shunt Over Voltage Limit (current limit) (4 byte double)
|40035|Shunt Over Voltage Limit (current limit) 
|40036|Shunt UNDER Voltage Limit (under current limit) (4 byte double)
|40037|Shunt UNDER Voltage Limit (under current limit)
|40038|Shunt Over POWER LIMIT (4 byte double)
|40039|Shunt Over POWER LIMIT
|40040|Shunt Temperature Coefficient (SHUNT_TEMPCO) (unsigned int16)
|40041|INAXXX chip model number (should always be 0x0228)
|40042|GITHUB version
|40043|GITHUB version
|40044|COMPILE_DATE_TIME_EPOCH
|40045|COMPILE_DATE_TIME_EPOCH
|40046|Watchdog timer trigger count (like error counter)(unsigned int16)
|40047|DEBUG CONFIG (unsigned int16)
|40048|DEBUG ADC_CONFIG (unsigned int16)
|40049|DEBUG SHUNT_CAL (unsigned int16)
|40050|DEBUG SHUNT_TEMPCO
|40051|DEBUG DIAG_ALRT
|40052|DEBUG SOVL
|40053|DEBUG SUVL
|40054|DEBUG BOVL
|40055|DEBUG BUVL
|40056|DEBUG TEMP_LIMIT
|40057|DEBUG PWR_LIMIT
|40058|DEBUG DIETEMP

# 0x16 Write Multiple Registers

|Register|Description|Example|
|--------|-----------|-------|
|40005/40006|amphour_out|Set to zero to reset
|40007/40008|amphour_in|Set to zero to reset
|40010|Watchdog timer trigger count (like error counter) (unsigned int16)|
|40019|shunt_max_current  (unsigned int16) |e.g. 150
|40020|shunt_millivolt  (unsigned int16) |e.g. 50
|40021/40022|Battery Capacity (ah)  (unsigned int16)
|40023/40024|Fully charged voltage
|40025|Tail current (Amps)
|40026|Charge efficiency factor % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 9561 = 95.61%)
|40028|SHUNT_CAL (unsigned int16) **
|40029|Temperature limit (signed int16)|degrees C)
|40030/40031|Bus Overvoltage (overvoltage protection)(4 byte double)|Volts
|40032/40033|Bus Under Volt (4 byte double) |Volts
|40034/40035|Shunt Over Voltage Limit (current limit) (4 byte double)|Current
|40036/40037|Shunt UNDER Voltage Limit (under current limit) (4 byte double)|Current
|40038/40039|Shunt Over POWER LIMIT (4 byte double)|Power

** For normal configuration, just set the shunt maximum current and millivolt registers.  Shunt_Cal is calculated as needed based on those values.  For fine tuning calibration, the SHUNT_CAL register can be written to/adjusted.  Once a register is set, it is stored in EEPROM and used when shunt is powered up.

# Bit values for Register 40017 (status flags)

Returns a BIT value (on/off) for the following configuration items:

(First byte)
|Bit|Description|Read write|
|--------|-----------|-------|
16|TMPOL|Read only
15|SHNTOL|Read only
14|SHNTUL|Read only
13|BUSOL|Read only
12|BUSUL|Read only
11|POL|Read only
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only

(Second byte)
|Bit|Description|Read write|
|--------|-----------|-------|
8|Relay Trigger on TMPOL|Read write
7|Relay Trigger on SHNTOL|Read write
6|Relay Trigger on SHNTUL|Read write
5|Relay Trigger on BUSOL|Read write
4|Relay Trigger on BUSUL|Read write
3|Relay Trigger on POL|Read write
2|Existing Relay state (0=off)|Read write
1|Factory reset bit (always 0 when read)|Read write

Registers 1 to 6 indicate the current alert state of the INA228 chip, zero means not alerted.  The RED LED will light when any of the alert states are non-zero.

Registers 9 to 14 control which alerts will trigger the relay to turn on.  Value of 1 means the alert is active/enabled.

Register 15 returns the current relay state (and allows the relay to be manually controlled - TBC)

Register 16 always returns zero when read (set to 1 to perform a factory reset on the monitor, returning all values to defaults - TBC)
