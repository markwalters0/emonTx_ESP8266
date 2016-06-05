#define emonTxV3                                                        // Tell emonLib this is the emonTx V3 - don't read Vcc assume Vcc = 3.3V as is always the case on emonTx V3 eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/
//#define RF69_COMPAT 1                                                 // Set to 1 if using RFM69CW or 0 is using RFM12B
//#include <JeeLib.h>                                                   //https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14
//ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption

#include "EmonLib_ESP.h"                                                //modified library for SPI based ADC (MCP3008)                    // Include EmonLib energy monitoring library https://github.com/openenergymonitor/EmonLib

#include <MCP3008.h>

//#include <OneWire.h>                                                  //http://www.pjrc.com/teensy/td_libs_OneWire.html
//#include <DallasTemperature.h>                                        //http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip


EnergyMonitor vt1, ct1, ct2, ct3, ct4, ct5, ct6, ct7;                   // MCP3008 is 8 channel ADC. VT is connected to pin 0, pins 1-7 are CTs. ************CHECK THIS***************

const byte version = 23;         // firmware version divided by 10 e,g 16 = V1.6
//----------------------------emonTx V3 Settings---------------------------------------------------------------------------------------------------------------
const byte Vrms =                  240;                              // Vrms for apparent power readings (when no AC-AC voltage sample is present)
const byte TIME_BETWEEN_READINGS = 10;            //Time between readings

//http://openenergymonitor.org/emon/buildingblocks/calibration

const float Ical1 =                90.9;                                // (2000 turns / 22 Ohm burden) = 90.9
const float Ical2 =                90.9;                                // (2000 turns / 22 Ohm burden) = 90.9
const float Ical3 =                90.9;                                // (2000 turns / 22 Ohm burden) = 90.9
const float Ical4 =                16.67;                              // (2000 turns / 120 Ohm burden) = 16.67
const float Ical5 =                90.9;                                // (2000 turns / 22 Ohm burden) = 90.9
const float Ical6 =                90.9;                                // (2000 turns / 22 Ohm burden) = 90.9
const float Ical7 =                16.67;                              // (2000 turns / 120 Ohm burden) = 16.67

float Vcal =                       145.389189;                        // (230V x 13) / (9V x 1.2) = 276.9 Calibration for UK AC-AC adapter 77DB-06-09
//float Vcal=276.9;
//const float Vcal=               260;                             //  Calibration for EU AC-AC adapter 77DE-06-09
const float Vcal_USA =             130.0;                            //Calibration for US AC-AC adapter 77DA-10-09
boolean USA = false;

const float phase_shift =          1.7;
const int no_of_samples =          800;
const int no_of_half_wavelengths = 30;
const int timeout =                500;                              //emonLib timeout
const int ACAC_DETECTION_LEVEL =   90;
//const byte min_pulsewidth= 110;                                // minimum width of interrupt pulse (default pulse output meters = 100ms)
//const int TEMPERATURE_PRECISION=  11;                          //9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to resplution of 0.5C, 0.25C, 0.125C and 0.0625C
//const byte MaxOnewire=             6;
//#define ASYNC_DELAY 375                                          // DS18B20 conversion delay - 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------


//----------------------------emonTx V3 hard-wired connections---------------------------------------------------------------------------------------------------------------
//const byte LEDpin =                 6;                             // emonTx V3 LED
//const byte DS18B20_PWR=            19;                             // DS18B20 Power
//const byte DIP_switch1=            8;                              // Voltage selection 230 / 110 V AC (default switch off 230V)  - switch off D8 is HIGH from internal pullup
//const byte DIP_switch2=            9;                              // RF node ID (default no chance in node ID, switch on for nodeID -1) switch off D9 is HIGH from internal pullup
//const byte battery_voltage_pin =    7;                             // Battery Voltage sample from 3 x AA
//const byte pulse_countINT =         1;                             // INT 1 / Dig 3 Terminal Block / RJ45 Pulse counting pin(emonTx V3.4) - (INT0 / Dig2 emonTx V3.2)
//const byte pulse_count_pin =        3;                             // INT 1 / Dig 3 Terminal Block / RJ45 Pulse counting pin(emonTx V3.4) - (INT0 / Dig2 emonTx V3.2)
//#define ONE_WIRE_BUS               5                               // DS18B20 Data
//-------------------------------------------------------------------------------------------------------------------------------------------

//Setup DS128B20
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);
//byte allAddress [MaxOnewire][8];  // 8 bytes per address
//byte numSensors;
//-------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------RFM12B / RFM69CW SETTINGS----------------------------------------------------------------------------------------------------
//#define RF_freq RF12_433MHZ                                              // Frequency of RF69CW module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
//byte nodeID = 8;                                                        // emonTx RFM12B node ID
//const int networkGroup = 210;

typedef struct {
  int power1, power2, power3, power4, power5, power6, power7, Vrms;
  //  unsigned long pulseCount;
} PayloadTX;     // create structure - a neat way of packaging data for RF comms

PayloadTX emontx;

//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------

//Random Variables
//boolean settled = false;
boolean CT1, CT2, CT3, CT4, CT5, CT6, CT7, ACAC, debug;
byte CT_count = 0;

//volatile byte pulseCount = 0;
//unsigned long pulsetime=0;                                    // Record time of interrupt pulse

//..........................MCP3008 Config.........................................................
//this is the hardware SPI pins.
//to change these, you need to change the emonTx library, and here!
#define CS_PIN 15 //GPIO 15
#define CLOCK_PIN 12 //GPIO 12
#define MOSI_PIN 13 //GPIO 13, master out, slave in goes to Din
#define MISO_PIN 14 //GPIO14, Dout on MCP3008

// put pins inside MCP3008 constructor
MCP3008 adc1(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);



double calc_rms(int pin, int samples)
{
  unsigned long sum = 0;
  for (int i = 0; i < samples; i++) // 178 samples takes about 20ms
  {
    int raw = (adc1.readADC(pin) - 512);
    sum += (unsigned long)raw * raw;
  }
  double rms = sqrt((double)sum / samples);
  return rms;
}



void setupEmonTX() {
  //  pinMode(LEDpin, OUTPUT);
  //  pinMode(DS18B20_PWR, OUTPUT);

  //  pinMode(pulse_count_pin, INPUT_PULLUP);                     // Set emonTx V3.4 interrupt pulse counting pin as input (Dig 3 / INT1)
  //  emontx.pulseCount=0;                                        // Make sure pulse count starts at zero

  //  digitalWrite(LEDpin,HIGH);

  Serial.begin(115200);

  Serial.print("emonTx V3.4 Discrete Sampling V"); Serial.print(version * 0.1);
  //  #if (RF69_COMPAT)
  //    Serial.println(" RFM69CW");
  //  #else
  //    Serial.println(" RFM12B");
  //  #endif
  Serial.println("OpenEnergyMonitor.org");
  //Serial.println("POST.....wait 10s");

  //READ DIP SWITCH POSITIONS
  //  pinMode(DIP_switch1, INPUT_PULLUP);
  //  pinMode(DIP_switch2, INPUT_PULLUP);
  //  if (digitalRead(DIP_switch1)==LOW) nodeID--;                            // IF DIP switch 1 is switched on then subtract 1 from nodeID
  //  if (digitalRead(DIP_switch2)==LOW) USA=true;                            // IF DIP switch 2 is switched on then activate USA mode


  if (USA == true) {                                                      // if USA mode is true
    Vcal = Vcal_USA;                                                      // Assume USA AC/AC adatper is being used, set calibration accordingly
  }

  //delay(10);
  //  rf12_initialize(nodeID, RF_freq, networkGroup);                         // initialize RFM12B/rfm69CW
  for (int i = 10; i >= 0; i--)                                           // Send RF test sequence (for factory testing)
  {
    emontx.power1 = i;
    //    rf12_sendNow(0, &emontx, sizeof emontx);
    //delay(100);
  }
  //  rf12_sendWait(2);
  emontx.power1 = 0;

  //
  //  for (int pin = 0, pin <=7, Pin++){
  //   double rms_check = calc_rms(pin, no_of_samples); //need to have a better think about CT detection. I think I used some digital pins connected to the stereo jack tip to either pull up or down.
  //
  //
  //  }

  if (adc1.readADC(0) > 0) {
    CT7 = 1;
    CT_count++;
  } else CT7 = 0;            // check to see if CT is connected to CT0 input, if so enable that channel
  if (adc1.readADC(1) > 0) {
    CT1 = 1;
    CT_count++;
  } else CT1 = 0;            // check to see if CT is connected to CT1 input, if so enable that channel
  if (adc1.readADC(2) > 0) {
    CT2 = 1;
    CT_count++;
  } else CT2 = 0;            // check to see if CT is connected to CT2 input, if so enable that channel
  if (adc1.readADC(3) > 0) {
    CT3 = 1;
    CT_count++;
  } else CT3 = 0;            // check to see if CT is connected to CT3 input, if so enable that channel
  if (adc1.readADC(4) > 0) {
    CT4 = 1;
    CT_count++;
  } else CT4 = 0;            // check to see if CT is connected to CT4 input, if so enable that channel
  if (adc1.readADC(5) > 0) {
    CT5 = 1;
    CT_count++;
  } else CT5 = 0;            // check to see if CT is connected to CT2 input, if so enable that channel
  if (adc1.readADC(6) > 0) {
    CT6 = 1;
    CT_count++;
  } else CT6 = 0;            // check to see if CT is connected to CT3 input, if so enable that channel


  if ( CT_count == 0) CT1 = 1;                                           // If no CT's are connect ed CT1-4 then by default read from CT1

  // Quick check to see if there is a voltage waveform present on the ACAC Voltage input
  // Check consists of calculating the RMS from 100 samples of the voltage input.
  //  Sleepy::loseSomeTime(10000);            //wait for settle
  //  digitalWrite(LEDpin, LOW);

  // Calculate if there is an ACAC adapter on analog input 0
  //double vrms = calc_rms(0,1780) * (Vcal * (3.3/1024) );
  double vrms = calc_rms(0, no_of_samples) * (Vcal * (3.3 / 1024));
  if (vrms > ACAC_DETECTION_LEVEL) ACAC = 1; else ACAC = 0;

  //  if (ACAC)
  //  {
  //    for (int i = 0; i < 10; i++)                                          // indicate AC has been detected by flashing LED 10 times
  //    {
  ////      digitalWrite(LEDpin, HIGH); delay(200);
  ////      digitalWrite(LEDpin, LOW); delay(300);
  //    }
  //  }
  //  else
  //  {
  ////    delay(1000);
  ////    digitalWrite(LEDpin, HIGH); delay(2000); digitalWrite(LEDpin, LOW);   // indicate DC power has been detected by turing LED on then off
  //  }


  //################################################################################################################################
  //Setup and for presence of DS18B20
  //################################################################################################################################
  //  digitalWrite(DS18B20_PWR, HIGH); delay(100);
  //  sensors.begin();
  //  sensors.setWaitForConversion(false);             // disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping
  //                                                   // http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/
  //  numSensors=(sensors.getDeviceCount());
  //  if (numSensors > MaxOnewire) numSensors=MaxOnewire;   //Limit number of sensors to max number of sensors
  //
  //  byte j=0;                                        // search for one wire devices and
  //                                                   // copy to device address arrays.
  //  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  //
  //  delay(500);
  //  digitalWrite(DS18B20_PWR, LOW);
  //
  //  if (numSensors==0) DS18B20_STATUS=0;
  //    else DS18B20_STATUS=1;

  //################################################################################################################################

  if (Serial) debug = 1; else debug = 0;        // if serial UART to USB is connected show debug O/P. If not then disable serial
  if (debug == 1)
  {
    Serial.print("CT 1 Cal "); Serial.println(Ical1);
    Serial.print("CT 2 Cal "); Serial.println(Ical2);
    Serial.print("CT 3 Cal "); Serial.println(Ical3);
    Serial.print("CT 4 Cal "); Serial.println(Ical4);
    Serial.print("CT 5 Cal "); Serial.println(Ical5);
    Serial.print("CT 6 Cal "); Serial.println(Ical6);
    Serial.print("CT 7 Cal "); Serial.println(Ical7);
    //delay(1000);

    Serial.print("RMS Voltage on AC-AC  is: ~");
    Serial.print(vrms, 0); Serial.println("V");

    if (ACAC) {
      Serial.println("AC-AC detected - Real Power measure enabled");
      Serial.println("assuming pwr from AC-AC (jumper closed)");
      if (USA == true) Serial.println("USA mode active");
      Serial.print("Vcal: "); Serial.println(Vcal);
      Serial.print("Phase Shift: "); Serial.println(phase_shift);
    } else {
      Serial.println("AC-AC NOT detected - Apparent Pwr measure enabled");
      Serial.print("Assuming VRMS: "); Serial.print(Vrms); Serial.println("V");
      Serial.println("Assuming power from batt / 5V USB - power save enabled");
    }

    if (CT_count == 0) {
      Serial.println("NO CT's detected");
    } else {
      if (CT1) Serial.println("CT 1 detected");
      if (CT2) Serial.println("CT 2 detected");
      if (CT3) Serial.println("CT 3 detected");
      if (CT4) Serial.println("CT 4 detected");
      if (CT5) Serial.println("CT 5 detected");
      if (CT6) Serial.println("CT 6 detected");
      if (CT7) Serial.println("CT 7 detected");
    }

    //    if (DS18B20_STATUS==1) {
    //      Serial.print("Detected Temp Sensors:  ");
    //      Serial.println(numSensors);
    //    } else {
    //      Serial.println("No temperature sensor");
    //    }
    //
    //    #if (RF69_COMPAT)
    //       Serial.println("RFM69CW");
    //    #else
    //      Serial.println("RFM12B");
    //    #endif
    //
    //    Serial.print("Node: "); Serial.print(nodeID);
    ////    Serial.print(" Freq: ");
    ////    if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
    ////    if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
    //////    if (RF_freq == RF12_915MHZ) Serial.print("915Mhz");
    ////    Serial.print(" Network: "); Serial.println(networkGroup);

    Serial.print("VT1 CT1 CT2 CT3 CT4 CT5 CT6 CT7 VRMS/BATT PULSE");
    //    if (DS18B20_STATUS==1){Serial.print(" Temperature 1-"); Serial.print(numSensors);}
    Serial.println(" ");
    //delay(500);

  }
  else
  {
    Serial.end();
  }


  if (CT1) ct1.current(1, Ical1);             // CT ADC channel 1, calibration.  calibration (2000 turns / 22 Ohm burden resistor = 90.909)
  if (CT2) ct2.current(2, Ical2);             // CT ADC channel 2, calibration.
  if (CT3) ct3.current(3, Ical3);             // CT ADC channel 3, calibration.
  if (CT4) ct4.current(4, Ical4);             // CT ADC channel 4, calibration.  calibration (2000 turns / 120 Ohm burden resistor = 16.66) high accuracy @ low power -  4.5kW Max @ 240V
  if (CT5) ct2.current(5, Ical5);             // CT ADC channel 2, calibration.
  if (CT6) ct3.current(6, Ical6);             // CT ADC channel 3, calibration.
  if (CT7) ct4.current(7, Ical7);             // CT ADC channel 4, calibration.  calibration (2000 turns / 120 Ohm burden resistor = 16.66) high accuracy @ low power -  4.5kW Max @ 240V

  if (ACAC)
  {
    if (CT1) ct1.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT2) ct2.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT3) ct3.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT4) ct4.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT5) ct5.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT6) ct6.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT7) ct7.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
  }

  //  attachInterrupt(pulse_countINT, onPulse, FALLING);     // Attach pulse counting interrupt pulse counting

  //  for(byte j=0;j<MaxOnewire;j++)
  //      emontx.temp[j] = 3000;                             // If no temp sensors connected default to status code 3000
  //                                                         // will appear as 300 once multipled by 0.1 in emonhub
} //end SETUP


char* measure_power() {
  unsigned long start = millis();
  Serial.println("Start collecting data");
  if (ACAC) {
    //delay(200);                         //if powering from AC-AC allow time for power supply to settle
    emontx.Vrms = 0;                    //Set Vrms to zero, this will be overwirtten by CT 1-4
  }

  emontx.power1 = 1;
  emontx.power2 = 1;
  emontx.power3 = 1;
  emontx.power4 = 1;
  emontx.power5 = 1;
  emontx.power6 = 1;
  emontx.power7 = 1;

  if (CT1) {
    if (ACAC) {
      ct1.calcVI(no_of_half_wavelengths, timeout);
      emontx.power1 = ct1.realPower;
      emontx.Vrms = ct1.Vrms * 100;
    } else {
      emontx.power1 = ct1.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of sample
    }
    Serial.print("Power 1: ");
    Serial.println(emontx.power1);
  }

  if (CT2) {
    if (ACAC) {
      ct2.calcVI(no_of_half_wavelengths, timeout);
      emontx.power2 = ct2.realPower;
      emontx.Vrms = ct2.Vrms * 100;
    } else {
      emontx.power2 = ct2.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of samples
    }
    Serial.print("Power 2: ");
    Serial.println(emontx.power2);
  }

  if (CT3) {
    if (ACAC) {
      ct3.calcVI(no_of_half_wavelengths, timeout);
      emontx.power3 = ct3.realPower;
      emontx.Vrms = ct3.Vrms * 100;
    } else {
      emontx.power3 = ct3.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of samples
    }
    Serial.print("Power 3: ");
    Serial.println(emontx.power3);
  }

  if (CT4) {
    if (ACAC) {
      ct4.calcVI(no_of_half_wavelengths, timeout);
      emontx.power4 = ct4.realPower;
      emontx.Vrms = ct4.Vrms * 100;
    } else {
      emontx.power4 = ct4.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of samples
    }
    Serial.print("Power 4: ");
    Serial.println(emontx.power4);
  }


  if (CT5) {
    if (ACAC) {
      ct5.calcVI(no_of_half_wavelengths, timeout);
      emontx.power5 = ct5.realPower;
      emontx.Vrms = ct5.Vrms * 100;
    } else {
      emontx.power5 = ct5.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of samples
    }
    Serial.print("Power 5: ");
    Serial.println(emontx.power5);
  }

  if (CT6) {
    if (ACAC) {
      ct6.calcVI(no_of_half_wavelengths, timeout);
      emontx.power6 = ct6.realPower;
      emontx.Vrms = ct6.Vrms * 100;
    } else {
      emontx.power6 = ct6.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of samples
    }
    Serial.print("Power 6: ");
    Serial.println(emontx.power6);
  }

  if (CT7) {
    if (ACAC) {
      ct7.calcVI(no_of_half_wavelengths, timeout);
      emontx.power7 = ct7.realPower;
      emontx.Vrms = ct7.Vrms * 100;
    } else {
      emontx.power7 = ct7.calcIrms(no_of_samples) * Vrms;                             // Calculate Apparent Power 1  1480 is  number of samples
    }
    Serial.print("Power 7: ");
    Serial.println(emontx.power7);
  }

  //  if (!ACAC) {                                                                        // read battery voltage if powered by DC
  //    int battery_voltage = analogRead(battery_voltage_pin) * 0.681322727;              // 6.6V battery = 3.3V input = 1024 ADC
  //    emontx.Vrms = battery_voltage;
  //  }

  //  if (pulseCount)                                                                     // if the ISR has counted some pulses, update the total count
  //  {
  //    cli();                                                                            // Disable interrupt just in case pulse comes in while we are updating the count
  //    emontx.pulseCount += pulseCount;
  //    pulseCount = 0;
  //    sei();                                                                            // Re-enable interrupts
  //  }
  //
  if (debug == 1) {
    Serial.print(emontx.power1); Serial.print(" ");
    Serial.print(emontx.power2); Serial.print(" ");
    Serial.print(emontx.power3); Serial.print(" ");
    Serial.print(emontx.power4); Serial.print(" ");
    Serial.print(emontx.power5); Serial.print(" ");
    Serial.print(emontx.power6); Serial.print(" ");
    Serial.print(emontx.power7); Serial.print(" ");
    Serial.print(emontx.Vrms); Serial.print(" ");
    //    Serial.print(emontx.pulseCount); Serial.print(" ");
    //    if (DS18B20_STATUS==1){
    //      for(byte j=0;j<numSensors;j++){
    //        Serial.print(emontx.temp[j]);
    //       Serial.print(" ");
    //      }
    //    }
    Serial.println("");
    //delay(50);
  }

  unsigned long runtime = millis() - start;
  unsigned long sleeptime = (TIME_BETWEEN_READINGS * 1000) - runtime - 100;

  //  if (ACAC) {                                                               // If powered by AC-AC adaper (mains power) then delay instead of sleep
  //    delay(sleeptime);
  //  } else {                                                                  // if powered by battery then sleep rather than dealy and disable LED to lower energy consumption
  //    // lose an additional 500ms here (measured timing)
  //    //    Sleepy::loseSomeTime(sleeptime-500);                                    // sleep or delay in seconds
  //  }
  //}

  // sleep will need to move to the main loop if we need it

  // need to arrange results into a string to MQTT out.
  //power result will be a double.
  Serial.println("Assembling data to transmit");

  //http://www.cplusplus.com/reference/cstdio/snprintf/
  //int snprintf ( char * s, size_t n, const char * format, ... );
  snprintf(msg, 120, "%d,%d,%d,%d,%d,%d,%d,%d", emontx.power1, emontx.power2, emontx.power3, emontx.power4, emontx.power5, emontx.power6, emontx.power7, emontx.Vrms);
  Serial.println("finished payload assembly");
  Serial.println(msg);

  return msg;
}


// The interrupt routine - runs each time a falling edge of a pulse is detected
//void onPulse()
//{
//  if ( (millis() - pulsetime) > min_pulsewidth) {
//    pulseCount++;         //calculate wh elapsed from time between pulses
//  }
//  pulsetime = millis();
//}
//
//void measure_temp() {
//  if (DS18B20_STATUS == 1)
//  {
//    //    digitalWrite(DS18B20_PWR, HIGH);
//    ////    Sleepy::loseSomeTime(50);
//    //    for(int j=0;j<numSensors;j++)
//    //      sensors.setResolution(allAddress[j], TEMPERATURE_PRECISION);                    // and set the a to d conversion resolution of each.
//    //    sensors.requestTemperatures();
//    //    Sleepy::loseSomeTime(ASYNC_DELAY);                                                // Must wait for conversion, since we use ASYNC mode
//    //    for(byte j=0;j<numSensors;j++)
//    //      emontx.temp[j]=get_temperature(j);
//    //    digitalWrite(DS18B20_PWR, LOW);
//  }
//}
//
//int get_temperature(byte sensor)
//{
//  float temp = (sensors.getTempC(allAddress[sensor]));
//  if ((temp < 125.0) && (temp > -55.0)) return (temp * 10);     //if reading is within range for the sensor convert float to int ready to send via RF
//
//
//

