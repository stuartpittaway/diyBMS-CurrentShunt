#include "SimpleModbusSlave.h"

#define BUFFER_SIZE (64 * 2 + 10)

// frame[] is used to recieve and transmit packages.
uint8_t frame[BUFFER_SIZE];
const uint16_t holdingRegsSize = 55; // size of the register array

bool broadcastFlag;

uint8_t function;
uint16_t errorCount;
uint16_t T1_5; // inter character time out
uint16_t T3_5; // frame delay
UartClass *ModbusPort;

// function definitions
void exceptionResponse(unsigned char exception);
uint16_t calculateCRC(unsigned char bufferSize);
void sendPacket(unsigned char bufferSize);

void modbus_configure(UartClass *SerialPort, long baud)
{
	ModbusPort = SerialPort;

	//holdingRegsSize = 55;
	errorCount = 0; // initialize errorCount

	//(*ModbusPort).begin(baud, byteFormat);
	//slaveID = _slaveID;

	// Modbus states that a baud rate higher than 19200 must use a fixed 750 us
	// for inter character time out and 1.75 ms for a frame delay for baud rates
	// below 19200 the timing is more critical and has to be calculated.
	// E.g. 9600 baud in a 10 bit packet is 960 characters per second
	// In milliseconds this will be 960characters per 1000ms. So for 1 character
	// 1000ms/960characters is 1.04167ms per character and finally modbus states
	// an inter-character must be 1.5T or 1.5 times longer than a character. Thus
	// 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.

	if (baud > 19200)
	{
		T1_5 = 750;
		T3_5 = 1750;
	}
	else
	{
		T1_5 = 15000000 / baud; // 1T * 1.5 = T1.5
		T3_5 = 35000000 / baud; // 1T * 3.5 = T3.5
	}
}

unsigned int modbus_update()
{

	if ((*ModbusPort).available())
	{
		unsigned char buffer = 0;
		unsigned char overflow = 0;

		while ((*ModbusPort).available())
		{
			// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the
			// serial buffer will be read untill all the data is cleared from the receive buffer.
			if (overflow)
				(*ModbusPort).read();
			else
			{
				if (buffer == BUFFER_SIZE)
					overflow = 1;
				frame[buffer] = (*ModbusPort).read();
				buffer++;
			}
			delayMicroseconds(T1_5); // inter character time out
		}

		// If an overflow occurred increment the errorCount
		// variable and return to the main sketch without
		// responding to the request i.e. force a timeout
		if (overflow)
			return errorCount++;

		// The minimum request packet is 8 bytes for function 3 & 16
		if (buffer > 7)
		{
			uint8_t id = frame[0];

			broadcastFlag = false;

			if (id == 0)
				broadcastFlag = true;

			if (id == ModbusSlaveAddress || broadcastFlag) // if the recieved ID matches the slaveID or broadcasting id (0), continue
			{
				uint16_t crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]); // combine the crc Low & High bytes

				if (calculateCRC(buffer - 2) == crc) // if the calculated crc matches the recieved crc continue
				{
					GreenLED(true);

					function = frame[1];
					uint16_t startingAddress = ((frame[2] << 8) | frame[3]); // combine the starting address bytes
					uint16_t no_of_registers = ((frame[4] << 8) | frame[5]); // combine the number of register bytes
					uint16_t maxData = startingAddress + no_of_registers;

					uint8_t address;
					uint16_t crc16;

					/*
					// broadcasting is not supported for function 2
					if (!broadcastFlag && (function == 2))
					{
						if (startingAddress <= 16 && (startingAddress + no_of_registers) <= 16) // check exception 2 ILLEGAL DATA ADDRESS
						{
							if ((maxData / 8) <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
							{
								//Clear output buffer
								memset(frame, 0, sizeof(frame));

								uint8_t noOfBytes = ReadDiscreteInputs(startingAddress, no_of_registers, &frame[3]);
								uint8_t responseFrameSize = 5 + noOfBytes;
								frame[0] = ModbusSlaveAddress;
								frame[1] = function;
								frame[2] = noOfBytes;

								crc16 = calculateCRC(responseFrameSize - 2);
								frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
								frame[responseFrameSize - 1] = crc16 & 0xFF;
								sendPacket(responseFrameSize);
							}
							else
								exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
						}
						else
							exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
					else
*/
					// broadcasting is not supported for function 3
					if (!broadcastFlag && (function == 3))
					{
						if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						{
							if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
							{
								//Clear output buffer
								memset(frame, 0, sizeof(frame));
								unsigned char noOfBytes = no_of_registers * 2;
								// ID, function, noOfBytes, (dataLo + dataHi)*number of registers,
								//  crcLo, crcHi
								unsigned char responseFrameSize = 5 + noOfBytes;
								frame[0] = ModbusSlaveAddress;
								frame[1] = function;
								frame[2] = noOfBytes;
								address = 3; // PDU starts at the 4th byte
								unsigned int temp;

								for (uint8_t index = startingAddress; index < maxData; index++)
								{
									//temp = regs[index];
									temp = ReadHoldingRegister(index);
									frame[address] = temp >> 8; // split the register into 2 bytes
									address++;
									frame[address] = temp & 0xFF;
									address++;
								}

								crc16 = calculateCRC(responseFrameSize - 2);
								frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
								frame[responseFrameSize - 1] = crc16 & 0xFF;
								sendPacket(responseFrameSize);
							}
							else
								exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
						}
						else
							exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
					else if (function == 6)
					{
						if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						{
							//Value
							uint16_t newValue = ((frame[4] << 8) | frame[5]); // the 4th and 5th elements in frame is the 16 bit data value

							if (SetRegister(startingAddress, newValue))
							{

								//frame[0] = ModbusSlaveAddress;
								//frame[1] = function;
								//frame[2] = noOfBytes;

								// only the first 6 bytes are used for CRC calculation
								crc16 = calculateCRC(6);
								frame[6] = crc16 >> 8; // split crc into 2 bytes
								frame[7] = crc16 & 0xFF;

								// a function 16 response is an echo of the first 6 bytes from
								// the request + 2 crc bytes
								if (!broadcastFlag) // don't respond if it's a broadcast message
									sendPacket(8);
							}
							else
							{
								exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
							}
						}
						else
							exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
					else if (function == 16)
					{
						// Check if the recieved number of bytes matches the calculated bytes
						// minus the request bytes.
						// id + function + (2 * address bytes) + (2 * no of register bytes) +
						// byte count + (2 * CRC bytes) = 9 bytes
						if (frame[6] == (buffer - 9))
						{
							if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
							{
								if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
								{
									address = 7; // start at the 8th byte in the frame

									for (uint16_t index = startingAddress; index < maxData; index++)
									{
										uint16_t newValue = ((frame[address] << 8) | frame[address + 1]);
										SetRegister(index, newValue);
										address += 2;
									}

									// only the first 6 bytes are used for CRC calculation
									crc16 = calculateCRC(6);
									frame[6] = crc16 >> 8; // split crc into 2 bytes
									frame[7] = crc16 & 0xFF;

									// a function 16 response is an echo of the first 6 bytes from
									// the request + 2 crc bytes
									if (!broadcastFlag) // don't respond if it's a broadcast message
									{
										sendPacket(8);
									}
								}
								else
								{
									// exception 3 ILLEGAL DATA VALUE
									exceptionResponse(3);
								}
							}
							else
							{
								exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
							}
						}
						else
						{
							errorCount++; // corrupted packet
						}
					}
					else
					{
						exceptionResponse(1); // exception 1 ILLEGAL FUNCTION
					}
				}
				else
				{
					// checksum failed
					errorCount++;
				}

				GreenLED(false);
			} // incorrect id
		}
		else if (buffer > 0 && buffer < 8)
		{
			errorCount++; // corrupted packet
		}
	}
	return errorCount;
}

void exceptionResponse(uint8_t exception)
{
	// each call to exceptionResponse() will increment the errorCount
	errorCount++;
	if (!broadcastFlag) // don't respond if its a broadcast message
	{
		frame[0] = ModbusSlaveAddress;
		frame[1] = (function | 0x80); // set MSB bit high, informs the master of an exception
		frame[2] = exception;
		unsigned int crc16 = calculateCRC(3); // ID, function|0x80, exception code
		frame[3] = crc16 >> 8;
		frame[4] = crc16 & 0xFF;
		// exception response is always 5 bytes
		// ID, function + 0x80, exception code, 2 bytes crc
		sendPacket(5);
	}
}

uint16_t calculateCRC(uint8_t bufferSize)
{
	uint16_t temp = CRC16.modbus(frame, bufferSize);
	return (temp << 8) | (temp >> 8);
}

/*
uint16_t calculateCRC(uint8_t bufferSize)
{
	unsigned int temp, temp2, flag;
	temp = 0xFFFF;
	for (unsigned char i = 0; i < bufferSize; i++)
	{
		temp = temp ^ frame[i];
		for (unsigned char j = 1; j <= 8; j++)
		{
			flag = temp & 0x0001;
			temp >>= 1;
			if (flag)
				temp ^= 0xA001;
		}
	}
	// Reverse byte order.
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF;
	// the returned value is already swapped
	// crcLo byte is first & crcHi byte is last
	return temp;
}
*/

void sendPacket(unsigned char bufferSize)
{
	//ATTINY1614 has automatic control of TxEnablePin via hardware
	//for RS485 control

	//digitalWrite(TxEnablePin, HIGH);

	for (unsigned char i = 0; i < bufferSize; i++)
	{
		(*ModbusPort).write(frame[i]);
	}

	(*ModbusPort).flush();

	// allow a frame delay to indicate end of transmission
	delayMicroseconds(T3_5);

	//digitalWrite(TxEnablePin, LOW);
}