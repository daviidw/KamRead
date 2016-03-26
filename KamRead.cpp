#include "KamRead.h"

#define TIMEOUT 1500

typedef enum State
{
	Init,
	PrepareSend,
	Transmit,
	FinishTransmit,
	Wait,
	Recieve,
	Decode1,
	Decode2,
	Decode3,
	Done
}State;

long Baudrate = 1200;
byte TX_msg[20];
int TX_Size, TX_msg_Counter;
byte RX_Data[50];
unsigned long RX_Index;
byte Recv_msg[40];
unsigned long Starttime=0;
int DelayCount = 0;

//unsigned long MeasureStart, MeasureStop, MeasureDelta;

//unsigned int K_Register[20];	// Kamstrup register numbers 
unsigned int *kregnums;
int Nr_Regs;		// Number of registers to read (max 20)
int *Kam_Data;
int Current_Kreg;
int j;
bool Error;
int ErrorCounter=0;

State CurrentState = Init;
// Kamstrup optical IR serial
AltSoftSerial kamSer(8, 9, false);	// AltSoftware serial
//SoftwareSerial kamSer(8,9,false);


KamRead::KamRead(long baud)
{
	Baudrate = baud;
}

void KamRead::Setup(int KamData[], unsigned int kregs[], int NrKregs)
{
	Nr_Regs = NrKregs;
	kregnums = kregs;
	Kam_Data = KamData;
	kamSer.begin(Baudrate);
}

void KamRead::Loop()
{
	switch (CurrentState)
	{
		case Init:
			for (int i=0;i<Nr_Regs;i++)	//Reset data
				Kam_Data[i] = 0;
			ErrorCounter = 0;
			Current_Kreg = 0;
			CurrentState = PrepareSend;
			break;
		case PrepareSend:
			TX_Size = PrepareTransmit();
			TX_msg_Counter=0;
			CurrentState = Transmit;
			break;
		case Transmit:
			// send to serial interface
			for (TX_msg_Counter = 0; TX_msg_Counter < TX_Size; TX_msg_Counter++) 
			{
				kamSer.write(TX_msg[TX_msg_Counter]);
			}
			CurrentState = FinishTransmit;
			break;
		case FinishTransmit:
			if ((kamSer.TX_available() == 0) && !kamSer.TX_State())
			{
				kamSer.flushInput();  // flush serial buffer - might contain noise
				DelayCount = 0;
				CurrentState = Wait;
			}
			break;			
		case Wait:
			if (DelayCount < 7)
			{
				delay(1);		//Wait for some reason for TX to finish
				DelayCount++;
			}
			else
			{
				//Prepare for recieve
				RX_Index = 0;
				Starttime = millis();
				CurrentState = Recieve;
			}
			break;
		case Recieve:
			
			byte r;
			
			// handle incoming data
			if (kamSer.available()) 
			{
				// receive byte
				r = kamSer.read();
				if(r != 0x40)   // don't append if we see the start marker
				{
					// append data
					RX_Data[RX_Index] = r;
					RX_Index++;
				}
			}
			if (millis() - Starttime > TIMEOUT)		//Restart if timeout
			{
				Kam_Data[Nr_Regs] = ERROR_TIMEOUT;
				ErrorCounter++;
				if (ErrorCounter > 3)
					CurrentState = Init;
				else
					CurrentState = PrepareSend;
			}
			if (r == 0x0d)				// loop until EOL received
				CurrentState = Decode1;
			break;
		case Decode1:
			Error = false;
			// remove escape markers from received data
			j = 0;
			for (unsigned short i = 0; i < RX_Index -1; i++) 
			{
				if (RX_Data[i] == 0x1b) 
				{
					byte v = RX_Data[i+1] ^ 0xff;
					if (v != 0x06 and v != 0x0d and v != 0x1b and v != 0x40 and v != 0x80)
					{
						Error = true;	//Missing escape
					}
					Recv_msg[j] = v;
					i++; // skip
				} 
				else 
				{
					Recv_msg[j] = RX_Data[i];
				}
				j++;
			}
			
			if((RX_Index != 0) && !Error){
				// Check the received message
				CurrentState = Decode2;
			}
			else
			{
				Kam_Data[Nr_Regs] = ERROR_RECIEVE;
				ErrorCounter++;
				if (ErrorCounter > 3)
					CurrentState = Init;	//Error in reception, reset everything
				else
					CurrentState = PrepareSend;		//Retry
			}
			break;
		case Decode2:
			// check CRC
			if (crc_1021(Recv_msg,j)) 
			{
				Kam_Data[Nr_Regs] = ERROR_CRC;
				ErrorCounter++;
				if (ErrorCounter > 3)
					CurrentState = Init;	// CRC error, reset everything
				else
					CurrentState = PrepareSend;		//Retry
			}	
			else		
				CurrentState = Decode3;
			break;
		case Decode3:
			Kam_Data[Current_Kreg] = KamDecode(Current_Kreg, Recv_msg);
			CurrentState = Done;
			break;
		case Done:
			Current_Kreg++;
			if (Current_Kreg > (Nr_Regs - 1))
				Current_Kreg = 0;
			ErrorCounter = 0;
			CurrentState = PrepareSend;
			break;
		default:
			CurrentState = Init;
	}
}


// PrepareTransmit - prepare data to be sent
int KamRead::PrepareTransmit() {
	int tx_size = 1;
	int msgsize = 5;
	// prepare message to send and send it
	byte sendmsg[] = { 0x3f, 0x10, 0x01, (kregnums[Current_Kreg] >> 8), (kregnums[Current_Kreg] & 0xff) };

	// append checksum bytes to message
	byte newmsg[msgsize+2];
	for (int i = 0; i < msgsize; i++) { newmsg[i] = sendmsg[i]; }
	newmsg[msgsize++] = 0x00;
	newmsg[msgsize++] = 0x00;
	int c = crc_1021(newmsg, msgsize);
	newmsg[msgsize-2] = (c >> 8);
	newmsg[msgsize-1] = c & 0xff;

	// build final transmit message - escape various bytes
	TX_msg[0] = { 0x80 };   // prefix
	tx_size = 1;
	for (int i = 0; i < msgsize; i++) {
		if (newmsg[i] == 0x06 or newmsg[i] == 0x0d or newmsg[i] == 0x1b or newmsg[i] == 0x40 or newmsg[i] == 0x80) {
			TX_msg[tx_size++] = 0x1b;
			TX_msg[tx_size++] = newmsg[i] ^ 0xff;
			} else {
			TX_msg[tx_size++] = newmsg[i];
		}
	}
	TX_msg[tx_size++] = 0x0d;  // EOF
	
	return tx_size;
}

// kamDecode - decodes received data
int KamRead::KamDecode(unsigned short const kreg, byte const *msg) {
	float Result;
	int Result_i;	

	// skip if message is not valid
	if (msg[0] != 0x3f or msg[1] != 0x10) {
		Kam_Data[Nr_Regs] = ERROR_DECODE;
		return 0;
	}
	if (msg[2] != (kregnums[kreg] >> 8) or msg[3] != (kregnums[kreg] & 0xff)) {
		Kam_Data[Nr_Regs] = ERROR_DECODE;
		return 0;
	}
	
	// decode the mantissa
	long x = 0;
	for (int i = 0; i < msg[5]; i++) {
		x <<= 8;
		x |= msg[i + 7];
	}
	
	// decode the exponent
	int i = msg[6] & 0x3f;
	if (msg[6] & 0x40) {
		i = -i;
	};
	float ifl = pow(10,i);
	if (msg[6] & 0x80) {
		ifl = -ifl;
	}
	Result =x * ifl * 10;		// * 10 for saia decimal
	//Serial.println(Result);
	//Result = Result *10;
	Result_i=(int)(Result + 0.5);
	// return final value
	return Result_i;
}

// crc_1021 - calculate crc16
long KamRead::crc_1021(byte const *inmsg, unsigned int len){
	long creg = 0x0000;
	for(unsigned int i = 0; i < len; i++) {
		int mask = 0x80;
		while(mask > 0) {
			creg <<= 1;
			if (inmsg[i] & mask){
				creg |= 1;
			}
			mask>>=1;
			if (creg & 0x10000) {
				creg &= 0xffff;
				creg ^= 0x1021;
			}
		}
	}
	return creg;
}