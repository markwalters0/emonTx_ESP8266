// EmonTx firmware as modified by Mark Walters (mark.walters.0@gmail.com) to allow for use of SPI based ADC chip (MCP3008)
// Also includes WifiManager to allow for wifi and MQTT setup

// Works OK to setup Wifi and connect to MQTT server
// emonTx code runs through and data is assembled and transmitted to mqtt.
// Data arrives OK at the RPi, running mosquitto

// Need to check the analog inputs match physical wiring and need to get MQTT messages into emoncms.
// This might need mqttwarn, although the emoncms readme claims to subscribe to mqtt inputs, I havent been able to get it to work.


/*

  emonTxV3.4 Discrete Sampling

  If AC-AC adapter is detected assume emonTx is also powered from adapter (jumper shorted) and take Real Power Readings and disable sleep mode to keep load on power supply constant
  If AC-AC addapter is not detected assume powering from battereis / USB 5V AC sample is not present so take Apparent Power Readings and enable sleep mode

  Transmitt values via RFM69CW radio

   -----------------------------------------
  Part of the openenergymonitor.org project

  Authors: Glyn Hudson & Trystan Lea
  Builds upon JCW JeeLabs RF12 library and Arduino

  Licence: GNU GPL V3
*/

/*Recommended node ID allocation
  ------------------------------------------------------------------------------------------------------------
  -ID-  -Node Type-
  0 - Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes
  5-10  - Energy monitoring nodes
  11-14 --Un-assigned --
  15-16 - Base Station & logging nodes
  17-30 - Environmental sensing nodes (temperature humidity etc.)
  31  - Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
  Change Log:
  v2.3   16/11/15 Change to unsigned long for pulse count and make default node ID 8 to avoid emonHub node decoder conflict & fix counting pulses faster than 110ms, strobed meter LED http://openenergymonitor.org/emon/node/11490
  v2.2   12/11/15 Remove debug timming serial print code
  v2.1   24/10/15 Improved timing so that packets are sent just under 10s, reducing resulting data gaps in feeds + default status code for no temp sensors of 3000 which reduces corrupt packets improving data reliability
  V2.0   30/09/15 Update number of samples 1480 > 1662 to improve sampling accurancy: 1662 samples take 300 mS, which equates to 15 cycles @ 50 Hz or 18 cycles @ 60 Hz.
  V1.9   25/08/15 Fix spurious pulse readings from RJ45 port when DS18B20 but no pulse counter is connected (enable internal pull-up)
  V1.8 - 18/06/15 Increase max pulse width to 110ms
  V1.7 - 12/06/15 Fix pulse count debounce issue & enable pulse count pulse temperature
  V1.6 - Add support for multiple DS18B20 temperature sensors
  V1.5 - Add interrupt pulse counting - simplify serial print debug
  V1.4.1 - Remove filter settle routine as latest emonLib 19/01/15 does not require
  V1.4 - Support for RFM69CW, DIP switches and battery voltage reading on emonTx V3.4
  V1.3 - fix filter settle time to eliminate large inital reading
  V1.2 - fix bug which caused Vrms to be returned as zero if CT1 was not connected
  V1.1 - fix bug in startup Vrms calculation, startup Vrms startup calculation is now more accuratre
*/

