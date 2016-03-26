#ifndef KamRead_h
#define KamRead_h

#include <inttypes.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

#include <AltSoftSerial.h>

#define ERROR_TIMEOUT 1
#define ERROR_RECIEVE 2
#define ERROR_CRC 3
#define ERROR_DECODE 4

class KamRead {
	public:
		KamRead(long baud);
		void Setup(int KamData[], unsigned int kregs[], int NrKregs);
		void Loop();
	private:
		int PrepareTransmit();
		int KamDecode(unsigned short const kreg, byte const *msg);
		long crc_1021(byte const *inmsg, unsigned int len);
};

#endif