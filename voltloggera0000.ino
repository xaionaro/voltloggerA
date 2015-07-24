#include <SPI.h>
#include <EthernetV2_0.h>
#include <TimerOne.h>

#define FASTADC 1


byte mac[] = {
	0xfe, 0x00, 0x00, 0x00, 0x01, 0x00
};

unsigned int port_local  = 30319;

IPAddress ip_remote;

unsigned int port_remote = 30319;

EthernetClient eth;
EthernetUDP    udp;

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define SS    10   //Gadgeteer PIN 6
#define nRST  8  //Gadgeteer PIN 4
#define nPWDN 9  //Gadgeteer PIN 5
#define nINT 3  //Gadgeteer PIN 3

extern volatile unsigned long timer0_overflow_count;
unsigned long ticks()
{
	return timer0_overflow_count;
}

//#define UDP_BUFSIZE UDP_TX_PACKET_MAX_SIZE
#define UDP_BUFSIZE 800

unsigned char WriteBuffer[UDP_BUFSIZE];
int WriteBufferFilled;

void pushValue() {
	if (WriteBufferFilled >= UDP_BUFSIZE-3) {
		return;
	}

	uint16_t value;
	uint16_t timestamp;

	value     = analogRead(5);
	timestamp = micros();

	WriteBuffer[WriteBufferFilled++] = timestamp;
	WriteBuffer[WriteBufferFilled++] = timestamp >> 8;
	WriteBuffer[WriteBufferFilled++] = value;
	WriteBuffer[WriteBufferFilled++] = value >> 8;
}

void setup()
{
        SPI.setClockDivider(SPI_CLOCK_DIV2);


	Serial.begin(9600);
#if FASTADC
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	cbi(ADCSRA, ADPS0);
#endif
	pinMode(4,	OUTPUT);
	digitalWrite(4,	HIGH);		// disable SD card if one in the slot

	pinMode(SS,	OUTPUT);
	pinMode(nRST,	OUTPUT);
	pinMode(nPWDN,	OUTPUT);
	pinMode(nINT,	INPUT);
	digitalWrite(nPWDN,	LOW);	// enable power
	digitalWrite(nRST,	LOW);	// reset W5200
	delay(10);
	Serial.println("Starting VoltloggerA0000");
	digitalWrite(nRST,	HIGH);
	delay(200);			// wait W5200 work
	Serial.println("Trying to get an IP-address");

	while(!Ethernet.begin(mac)) {
		Serial.println("Retring DHCP");
	}

	Serial.print("Local address: ");
	Serial.print(Ethernet.localIP());
	Serial.print(":");
	Serial.println(port_local);
	ip_remote = Ethernet.localIP();
	ip_remote[3] = 2;
	Serial.print("Remote address: ");
	Serial.print(ip_remote);
	Serial.print(":");
	Serial.println(port_remote);
	udp.begin(port_local);

	WriteBufferFilled = 0;
	Timer1.initialize(100);
	Timer1.attachInterrupt(pushValue); 
}

#define BYTES_PER_PACKET (UDP_BUFSIZE << 2)

void loop()
{
	udp.beginPacket(ip_remote, port_remote);
	int i = 0;

//	while (WriteBufferFilled < UDP_BUFSIZE-3)
//		pushValue();

	udp.write(WriteBuffer, WriteBufferFilled);
	WriteBufferFilled=0;
	udp.endPacket();
}

/*** Interrupt routine ADC ready *** /
ISR(ADC_vect) {
  int aval = ADCL;    // store lower byte ADC
  aval += ADCH << 8;  // store higher bytes ADC
  if (sendStatus == 0) {
    if (startDelay < 10000) { // do nothing the first x samples
      startDelay++;
    }
    else {
      valueBin[aval] += 1; // increase value bin
      if (valueBin[aval] == 255) { // stop if a bin is full
        sendStatus = 1;
      }
    }
  }
}
*/
