///////////////////////////////////////////////////////////////////////////////
//
// LLAP very low power example - RFu-328 only
//
///////////////////////////////////////////////////////////////////////////////
//
// Target:       Ciseco RFu-328
// version:      0.1
// date:         25 August 2013
//
// copyright/left, limitations to use etc. statement to go here
//
//////////////////////////////////////////////////////////////////////////////
//
// Reads the tempurature from a DS1820 sensor
// Connections:
// DS18B20 Vdd: pin A5
// DS18B20 Data: pin D3
// DS18B20 Gnd: pin GND
// 4K7 pull up resistor : DS18B20 Vdd -> DS18B20 Data
//
/////////////////////////////////////////////////////////////////////////////
//
// RFu-328 specific AT commands (requires firmware RFu-328 V0.84 or better)
//    The following RFu specific AT commands are supported in V0.84 or later
//		Added a new sleep mode (ATSM3) Wake after timed interval
//			The duration is specified by a new command ATSD, units are mS
//			and the range is 32bit (max approx. 49 days)
//				e.g. ATSD5265C00 is one day (hex 5265C00 is decimal 86400000)
//				ATSD36EE80 is one hour
//				ATSD493E0 is five minutes
//		This sleep mode can be used to wake the 328 after a timed interval
//			by connecting RFu-328 pin 7 to pin 19, D3(INT1) or 20, D2(INT0).
//		Using this technique means that a RFu-328 can sleep for a timed
//			period consuming about 0.6uA, instead of using the AVR328 watchdog (for timing)
//			which uses around 7uA.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 emonTH low power temperature & humidity node
 ============================================

 Ambient humidity & temperature (DHT22 on-board)
 Multiple remote temperature (DS18B20) - Support for other onewire sensors such as the MAX31850K may be added at a later date. (If you want to do this yourself and contribute, go for it!)

 Provides the following inputs to emonCMS:

 1. Battery voltage
 2. Humidity (with DHT22 on-board sensor, otherwise zero)
 3. Ambient temperature (with DHT22 on-board sensor, otherwise zero)
 4. External temperature 1 (first DS18B20)
 5. External temperature 2 (second DS18B20)
 6. ... and so on. Should automatically detect any DS18B20 connected to the one wire bus. (Up to 60 sensors ordered by address.)

 Notes - If you connect additional DS18B20 sensors after node has been set up, the sensor order may change.
       - Check your inputs in emoncms after adding additional sensors and reassign feeds accordingly.
       -
       - This sketch will also draw more power with more connected sensors and is optimised for getting good data rather than low power consumption.
       - If you have a large number of sensors it is reccomended that you run your node off an external power supply.

 -----------------------------------------------------------------------------------------------------------
 Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH

 Part of the openenergymonitor.org project
 Licence: GNU GPL V3

 Authors: Dave McCraw (original creator), Marshall Scholz (added autimatic onewire scanning)

 Based on the emonTH_DHT22_DS18B20 sketch by Glyn Hudson and the DallasTemperature Tester sketch.

 THIS SKETCH REQUIRES:

 Libraries in the standard arduino libraries folder:
   - RFu JeeLib           https://github.com/openenergymonitor/RFu_jeelib   - to work with CISECO RFu328 module
   - DHT22 Sensor Library https://github.com/adafruit/DHT-sensor-library    - be sure to rename the sketch folder to remove the '-'
   - OneWire library      http://www.pjrc.com/teensy/td_libs_OneWire.html   - DS18B20 sensors
   - DallasTemperature    http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip - DS18B20 sensors
 */

#include <Arduino.h>
#include <LLAPSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Prototypes
uint8_t setupSRF();
uint8_t enterCommandMode();
uint8_t sendCommand(char* lpszCommand);
uint8_t checkOK(int timeout);

void dodelay(unsigned int ms);
long readVcc();
void printAddress(DeviceAddress deviceAddress);
void led_error();

void initialise_DS18B20();
void start_ds18b20_reading();
float take_ds18b20_reading(int i);

boolean temperature_in_range(float temp);

// configaturation
#define LED_PIN 6
#define NODE    "T1"

// Onewire bus configaturation
// ======================================================================================================================================
const int DS18B20_PWR = A5;  // Default A5
const int ONE_WIRE_BUS = 3;  // Default 3

const int ASYNC_DELAY           = 750; // Default 375 - Delay for onewire sensors to respond
const int TEMPERATURE_PRECISION = 12;  // Default 12  - Onewire temperature sensor precisionn. Details found below.
 /*
  NOTE: - There is a trade off between power consumption and sensor resolution.
        - A higher resolution will keep the processor awake longer - Approximate values found below.

  - DS18B20 temperature precision:
      9bit: 0.5C,  10bit: 0.25C,  11bit: 0.1125C, 12bit: 0.0625C
  - Required delay when reading DS18B20
      9bit: 95ms,  10bit: 187ms,  11bit: 375ms,   12bit: 750ms

  More info can be found here: http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/
 */

// SRF pin configaturation
// ======================================================================================================================================
const int RADIO_WAKE = 4;
const int RADIO_ENABLE = 8;
const int RADIO_IRQ = 2;

// Global variables
// ======================================================================================================================================

boolean debug = false; // variable to store is debug is avalable or not.

enum
{
  INIT,
  POWER_ON_SENSORS,
  BRING_UP_SENSORS,
  READ_TEMP,
  POWER_OFF_SENSORS,
  POWER_ON_RADIO,
  SEND_DATA,
  POWER_OFF_RADIO,
} state;

// OneWire for DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found onewire device address

#define MaxOnewire 60  // Maximum number of sensors on the onewire bus - too big of a number may create too big of a data packet (max packet size is 128 bytes) - defiult "60" to allow for (almost) full 128 byte packet length. 61 would make it full

void setup()
{
  pinMode(LED_PIN, OUTPUT);           // pin 4 controls the radio sleep
  digitalWrite(LED_PIN, HIGH);         // wake the radio

  Serial.begin(115200);         // Start the serial port

  LLAP.init(NODE);                  // Initialise the LLAPSerial library and the device identity

  pinMode(RADIO_ENABLE, OUTPUT);           // pin 8 controls the radio
  digitalWrite(RADIO_ENABLE, HIGH);        // select the radio

  pinMode(RADIO_WAKE, OUTPUT);           // pin 4 controls the radio sleep
  digitalWrite(RADIO_WAKE, LOW);         // wake the radio

  delay(400);						// let everything start up

  // set up sleep mode 3 (low = awake)
  uint8_t val;
  while ((val = setupSRF()) != 5)
  {
    LLAP.sendInt("ERR",val); // Diagnostic
    led_error();
    dodelay(5000);	// try again in 5 seconds
  }

  // Initialise sensors output pin
  pinMode(DS18B20_PWR, OUTPUT);

  // Did we find any sensors?
  while (0 == numberOfDevices)
  {
    // Find & Initialise sensors
    initialise_DS18B20();
    if (numberOfDevices > 0) {
      break;
    }

    LLAP.sendInt("ERR", -1); // Diagnostic
    led_error();
    dodelay(5000);
  }

  LLAP.sendMessage(F("STARTED"));  // send the usual "started message
  dodelay(50);
  pinMode(RADIO_WAKE, INPUT);             // sleep the radio

  digitalWrite(LED_PIN, LOW);

  state = INIT;
}

#define NAP_TIME 10

volatile int sleep = 0;
volatile int nap = 0;

float temp;
long batt;

void loop()
{
  // Do we need to sleep
  if (debug)
  {
    Serial.print((String)"sleep=" + sleep); delay(50);
    Serial.print((String)", nap=" + nap); delay(50);
    Serial.println((String)", state=" + (int)state); delay(50);
  }

  if (sleep > 0)
  {
    --sleep;
    pinMode(RADIO_WAKE, INPUT);
    LLAP.sleep(RADIO_IRQ, RISING, false);		// sleep until woken on pin 2, no pullup (low power)

    // Signal the SRF that we are awake
    pinMode(RADIO_WAKE, OUTPUT);
    return;
  }
  // do we need to nap
  if (nap > 0)
  {
    nap -= NAP_TIME;
    dodelay(NAP_TIME);
    return;
  }

  switch (state)
  {
    case INIT:
      state = POWER_ON_SENSORS;
      break;

    case POWER_ON_SENSORS:
      // Power up
      digitalWrite(DS18B20_PWR, HIGH);
      state = BRING_UP_SENSORS;
      nap = 50;
      break;

    case BRING_UP_SENSORS:
      start_ds18b20_reading();

      // Read the battery info (assume the Vcc)
      batt = readVcc();

      state = READ_TEMP;
      // nap = ASYNC_DELAY;
      sleep = 1;
      break;

    case READ_TEMP:
      // Take temperature readings
      // TODO: How can we send information about the temp sensor number? #Jeremy
      temp = take_ds18b20_reading(0);

      state = POWER_OFF_SENSORS;
      break;

    case POWER_OFF_SENSORS:
      // Power up
      digitalWrite(DS18B20_PWR, LOW);
      state = POWER_ON_RADIO;
      break;

    case POWER_ON_RADIO:
      // wake the radio
      //pinMode(RADIO_WAKE, OUTPUT);
      digitalWrite(LED_PIN, HIGH);

      // Send the sensor values
      LLAP.sendIntWithDP("BATT", batt, 3);
      if (temperature_in_range(temp)) {
        LLAP.sendIntWithDP("TEMP", temp * 100, 2);
      }

      // sleep the radio
      //pinMode(RADIO_WAKE, INPUT);
      state = INIT;
      sleep = 10;
      digitalWrite(LED_PIN, LOW);

      break;
  }
}


/////////////////////////////////////////////////////////
// SRF AT command handling
/////////////////////////////////////////////////////////

uint8_t setupSRF()	// set Sleep mode 3
{
  if (!enterCommandMode())	// if failed once then try again
  {
    if (!enterCommandMode()) return 1;
  }

  //if (!sendCommand("ATSD49E30")) return 2; // 5 minutes
  //if (!sendCommand("ATSDEA60")) return 2;	// 1 minute
  //if (!sendCommand("ATSD4E20")) return 2;	// 20 seconds
  //if (!sendCommand("ATSD1388")) return 2;	// 5 seconds
  if (!sendCommand("ATSD3E8")) return 2;	// 1 second

  if (!sendCommand("ATSM3")) return 3;
  if (!sendCommand("ATSD")) return 3;
  if (!sendCommand("ATSM")) return 3;
  if (!sendCommand("ATVR")) return 3;
  if (!sendCommand("ATDN")) return 3;
  return 5;
}

uint8_t enterCommandMode()
{
  delay(1200);
  Serial.print("+++");
  delay(500);
  while (Serial.available()) Serial.read();  // flush serial in
  delay(500);
  return checkOK(500);
}

uint8_t sendCommand(char* lpszCommand)
{
  Serial.print(lpszCommand);
  Serial.write('\r');
  return checkOK(100);
}

uint8_t checkOK(int timeout)
{
  uint32_t time = millis();
  while (millis() - time < timeout)
  {
    if (Serial.available() >= 3)
    {
      if (Serial.read() != 'O') continue;
      if (Serial.read() != 'K') continue;
      if (Serial.read() != '\r') continue;
      return 1;
    }
  }
  return 0;
}

//////////////////////////////////////////////////
/**
* Find the expected DS18B20 sensors
*
* Automatically scans the entire onewire bus for sensors and stores their adressess in the device adresses array.
*
* Should support up to 60 sensors. (limited by size of data packet) Currently tested up to four sensors.
*/
void initialise_DS18B20()
{
  // Switch on
  digitalWrite(DS18B20_PWR, HIGH);
  dodelay(50);

  // start up oonewire library
  sensors.begin();

  // Disable automatic temperature conversion to reduce time spent awake, instead we sleep for ASYNC_DELAY
  // see http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/
  sensors.setWaitForConversion(false);

  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();

  // Detect if too many devices are connected
  if (numberOfDevices > MaxOnewire)
  {
    numberOfDevices = MaxOnewire; // Set number of devices to maximum allowed sensors to prevent the extra sensors from being included.
  }

  // bus info
  if (debug)
  {
    // locate devices on the bus
    Serial.print("Locating devices...");
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");

    if (numberOfDevices > MaxOnewire) {
      Serial.println("Too many devices!!!");
      Serial.print("maximum allowed number of devices is ");
      Serial.println(MaxOnewire);
    }

    // report parasite power requirements
    Serial.print("Parasite power is: ");
    if (sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
  }

  // Loop through each device, print out address
  for (int i = 0; i<numberOfDevices; i++)
  {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      if (debug) {
        Serial.print("Found device ");
        Serial.print(i, DEC);
        Serial.print(" with address: ");
        printAddress(tempDeviceAddress); // just prints sensor adress on serial bus
        Serial.println();

        Serial.print("Setting resolution to ");
        Serial.println(TEMPERATURE_PRECISION, DEC);
      }

      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

      if (debug) {
        Serial.print("Resolution actually set to: ");
        Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
        Serial.println();
      }
    }
    else{

      if (debug) {
        Serial.print("Found ghost device at ");
        Serial.print(i, DEC);
        Serial.println(" but could not detect address. Check power and cabling");
      }
    }
  }

  // Switch off for now
  digitalWrite(DS18B20_PWR, LOW);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


void start_ds18b20_reading()
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  if (debug) {
    Serial.println("DONE");
  }
}

//////////////////////////////////////////////////
/**
* Convenience method; read from all DS18B20s
*
*/
float take_ds18b20_reading(int i)
{
  float temp1 = -9999;

  // Search the wire for address
  if (sensors.getAddress(tempDeviceAddress, i))
  {
    sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

    // Get readings. We must wait for ASYNC_DELAY due to power-saving (waitForConversion = false)
    temp1 = (sensors.getTempC(tempDeviceAddress));
  }

  return temp1;
}


//////////////////////////////////////////////////
/**
* validate that the provided temperature is within acceptable bounds.
*/
boolean temperature_in_range(float temp)
{
  // Only accept the reading if it's within a desired range.
  float minimumTemp = -40.0;
  float maximumTemp = 125.0;

  return temp > minimumTemp && temp < maximumTemp;
}



//////////////////////////////////////////////////
/**
* Power-friendly delay
*/
void dodelay(unsigned int ms)
{
  byte oldADCSRA = ADCSRA;
  byte oldADCSRB = ADCSRB;
  byte oldADMUX = ADMUX;

  LLAP.sleepForaWhile(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)

  ADCSRA = oldADCSRA;         // restore ADC state
  ADCSRB = oldADCSRB;
  ADMUX = oldADMUX;
}

// From https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
long readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void led_error()
{
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW); delay(100);
  }
}
