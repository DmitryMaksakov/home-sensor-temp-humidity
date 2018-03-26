#include <Arduino.h>
#include <EEPROM.h>
#include <VirtualWire.h>
#include <EasyTransferVirtualWire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "DHT.h"
#include "sha1.h"

// Constants
const unsigned int AWAKE_SLEEP_INTERVAL_COUNT = 4;
const int LED_POWER_PIN = 13;
const int TRANSMIT_POWER_PIN = 7;
const int TEMP_DATA_PIN = 3;
const int TEMP_POWER_PIN = 4;
const int TRANSMIT_POWER_PIN = 8;
const unsigned char* SECRET = "QHcGpCh?mAzQ7vCW#4SZnZ5-2-r%2kfL";
const unsigned int SECRET_LENGTH = 32;

// Device ID
unsigned int uniqueDeviceId = 0;

// Current packet number
unsigned int packetNumber = 0;

// EasyTransfer lib object
EasyTransferVirtualWire ET;

// DHT lib object
DHT dht(TEMP_DATA_PIN, DHT22);

// Eight second done sleep intervals count
unsigned int doneSleepIntervalsCount = 0;

// Packet structure
struct SEND_DATA_STRUCTURE {
  unsigned int sourceId; //4 bytes
  unsigned int packetNumber; //4 bytes
  byte commandType; // 1 byte
  float data; // 4 bytes
  byte hmac0; // 1 byte
  byte hmac2; // 1 byte
  byte hmac3; // 1 byte
  byte hmac4; // 1 byte
  byte hmac5; // 1 byte
  byte hmac6; // 1 byte
  byte hmac8; // 1 byte
  byte hmac11; // 1 byte
  byte hmac13; // 1 byte
  byte hmac15; // 1 byte
  byte hmac17; // 1 byte
  byte hmac18; // 1 byte
  byte hmac19; // 1 byte
};

// Current packet
SEND_DATA_STRUCTURE packet;

// Write EEPROM API
void EEPROMWriteInt(int address, unsigned int value)
{
	byte lowByte = ((value >> 0) & 0xFF);
	byte highByte = ((value >> 8) & 0xFF);

	EEPROM.write(address, lowByte);
	EEPROM.write(address + 1, highByte);
}

// Read EEPROM API
unsigned int EEPROMReadInt(int address)
{
	byte lowByte = EEPROM.read(address);
	byte highByte = EEPROM.read(address + 1);

	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

// Returns unique unsigned int Device ID
unsigned int getDeviceId()
{
	unsigned int uniqueDeviceId = 0;

	Serial.print("Getting Device ID...");
	uniqueDeviceId = EEPROMReadInt(0);

	if (uniqueDeviceId < 10000 || uniqueDeviceId > 60000) {
		Serial.print("N/A, updating... ");
		randomSeed(analogRead(0));
		uniqueDeviceId = random(10000, 60000);
		EEPROMWriteInt(0, uniqueDeviceId);
	}

	Serial.println(uniqueDeviceId);

	return uniqueDeviceId;
}

// Blinks Arduino led with {ms} interval {count} times
void blink(int ms = 100, int count = 1)
{
	for(int i = 0; i < count; i++)
	{
		digitalWrite(LED_POWER_PIN, HIGH);
		delay(ms);
		digitalWrite(LED_POWER_PIN, LOW);
		delay(100);
	}
}

// Sends temp and humidity
void sendData()
{
  unsigned char *hash;

  // Reading temp
  float temp = dht.readTemperature();

  // Calculating hash
  Sha1.initHmac(SECRET, SECRET_LENGTH);
  Sha1.print(uniqueDeviceId);
  Sha1.print(packetNumber);
  Sha1.print(1);
  Sha1.print(temp);

  hash = Sha1.resultHmac();

  // Constructing packet
	packet.sourceId = uniqueDeviceId;
	packet.packetNumber = packetNumber;
	packet.commandType = 1;
	packet.data = temp;
  packet.hmac0 = hash[0];
  packet.hmac2 = hash[2];
  packet.hmac3 = hash[3];
  packet.hmac4 = hash[4];
  packet.hmac5 = hash[5];
  packet.hmac6 = hash[6];
  packet.hmac8 = hash[8];
  packet.hmac11 = hash[11];
  packet.hmac13 = hash[13];
  packet.hmac15 = hash[15];
  packet.hmac17 = hash[17];
  packet.hmac18 = hash[18];
  packet.hmac19 = hash[19];

  Serial.println(packet.packetNumber);
  Serial.println(packet.data);
  Serial.println();

  // Increasing current packet number
  packetNumber++;

	digitalWrite(LED_POWER_PIN, HIGH);
	ET.sendData();

  delay(25);

  // Reading humidity
  float humidity = dht.readHumidity();

  // Calculating hash
  Sha1.initHmac(SECRET, SECRET_LENGTH);
  Sha1.print(uniqueDeviceId);
  Sha1.print(packetNumber);
  Sha1.print(2);
  Sha1.print(humidity);

  hash = Sha1.resultHmac();

  // Constructing packet
	packet.sourceId = uniqueDeviceId;
	packet.packetNumber = packetNumber;
	packet.commandType = 2;
	packet.data = humidity;
  packet.hmac0 = hash[0];
  packet.hmac2 = hash[2];
  packet.hmac3 = hash[3];
  packet.hmac4 = hash[4];
  packet.hmac5 = hash[5];
  packet.hmac6 = hash[6];
  packet.hmac8 = hash[8];
  packet.hmac11 = hash[11];
  packet.hmac13 = hash[13];
  packet.hmac15 = hash[15];
  packet.hmac17 = hash[17];
  packet.hmac18 = hash[18];
  packet.hmac19 = hash[19];

  Serial.println(packet.packetNumber);
  Serial.println(packet.data);
  Serial.println();

  // Increasing current packet number
  packetNumber++;

	digitalWrite(LED_POWER_PIN, HIGH);
	ET.sendData();
	digitalWrite(LED_POWER_PIN, LOW);
}

// Watchdog interrupt
ISR (WDT_vect)
{
   wdt_disable(); // disable watchdog
}

// Enables sleep mode
void sleep()
{
	ADCSRA = 0;
	MCUSR = 0;
	WDTCSR = bit (WDCE) | bit (WDE);
	WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);
	wdt_reset();
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	MCUCR = bit (BODS) | bit (BODSE);
	MCUCR = bit (BODS);
	sleep_cpu ();
	sleep_disable();
}

// Switches power for periphery (transmitter, sensors, etc.)
void switchPeriphery(bool isOn)
{
	digitalWrite(TEMP_POWER_PIN, isOn ? HIGH : LOW);
  digitalWrite(TRANSMIT_POWER_PIN, isOn ? HIGH : LOW);
}

// Initialization part
void setup()
{
  // Pins initialization
  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(TEMP_POWER_PIN, OUTPUT);
  pinMode(TRANSMIT_POWER_PIN, OUTPUT);
  pinMode(TEMP_DATA_PIN, INPUT);
  digitalWrite(TEMP_DATA_PIN, HIGH);

  // Starting serial
  Serial.begin(9600); // Debugging only

  // Init DHT Library
  dht.begin();

  // Init Easy Transfer lib
  ET.begin(details(packet));
	vw_set_tx_pin(TRANSMIT_POWER_PIN);
	vw_setup(2000); // 2k bits ber second

  // Obtaining Device ID
	uniqueDeviceId = getDeviceId();

  // Indicating Device is ready
	blink(100, 3);
}

// Main loop
void loop()
{
	doneSleepIntervalsCount++;

  // Arduino wakes up after 8s, so real action will be after 8s * AWAKE_SLEEP_INTERVAL_COUNT
	if (doneSleepIntervalsCount >= AWAKE_SLEEP_INTERVAL_COUNT) {
		// Power on for sensors and transmitter
		switchPeriphery(1);
		delay(100);

		// Init DHT Library
    dht.begin();
    delay(500);

    // Send data
		sendData();

		// Power off for sensors and transmitter
		switchPeriphery(0);

		// Reset intervals counter
		doneSleepIntervalsCount = 0;
	}

  // Sleep again
	sleep();
}
