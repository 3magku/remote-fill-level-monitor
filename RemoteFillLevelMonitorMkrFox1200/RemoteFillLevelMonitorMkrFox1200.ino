/*
**
** RemoteFillLevelMonitorMkrFox1200
**
*/


/*

:Author: 3magku
:Date: 14/11/2017
:Revision: version#
:License: Public Domain

This sketch is based on the Arduino >TUTORIALS > Examples from Libraries > SigFox > EventTrigger - Example
at https://www.arduino.cc/en/Tutorial/SigFoxEventTrigger.

See also the tutorial at https://www.arduino.cc/en/Tutorial/SigFoxFirstConfiguration for first configuration 
and registration of your MKRFox1200 board.

*/

/*
   Arduino MKR Fox 1200 / VL53L0X/VL6180X / SigFox

   Board:   Arduino MKRFox1200
   Sensor:  VL6180X Time of Flight Distance Ranging Sensor (VL6180) (Adafruit breakout) or
            VL53L0X Time of Flight Distance Ranging Sensor (VL530X) (Adafruit breakout)
   Wiring:
    VL___0X   GND   ->  GND       MKRFox1200
              SCL   ->  D11 (SCL)
              SDA   ->  D12 (SCA)
              VIN   ->  3,3V
    Switches on: D0, D1 and D7 to GND
*/

/*
* Choose Time of Flight Distance Ranging Sensor (TOFDRS): VL53L0X or VL6180X 
* or leave undefined for testing SigFox connectivity only.
*/
#define TOFDRS_VL6180X
//#define TOFDRS_VL53L0X

/* *** imports *************************************************************** */

// VL53L0X & VL6180X:
#include <Wire.h>
#ifdef TOFDRS_VL53L0X
#include <Adafruit_VL53L0X.h>
#elif defined TOFDRS_VL6180X
#include "Adafruit_VL6180X.h"
#else
#warning "No ToFDRS defined!"
#endif

// SigFox
#include <SigFox.h>

// LowPower
#include <ArduinoLowPower.h>

/* *** globals *************************************************************** */

#ifdef TOFDRS_VL53L0X
Adafruit_VL53L0X vl = Adafruit_VL53L0X();
#elif defined TOFDRS_VL6180X
Adafruit_VL6180X vl = Adafruit_VL6180X();
#else
#warning "No ToFDRS defined!"
#endif

//
float vl_lux = 0;
uint8_t vl_range = 0;
uint8_t vl_status = 0;

// Set debug to false to enable continuous mode
// and disable serial prints
int debug = true;

// SigFox message
/*
    ATTENTION - the structure we are going to send MUST
    be declared "packed" otherwise we'll get padding mismatch
    on the sent data - see http://www.catb.org/esr/structure-packing/#_structure_alignment_and_padding
    for more details
*/
typedef struct __attribute__ ((packed)) sigfox_message {
  uint8_t mode[3] = {0};
  uint8_t status = 0;
  uint8_t value = 0;
  uint8_t value_low = 0;
  uint8_t value_high = 0;
  uint8_t level = 0;
} SigfoxMessage;
// stub for message which will be sent
SigfoxMessage message;

// Trigger
volatile int trigger_id = 0;

// LED
#define LED_BUILTIN 6 // MKRFox1200 LED pin

/* *** setup **************************************************************** */

void setup() {
  // LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Debugging ...
  if (debug == true) {
    /* ATTENTION:
       Using Serial1 instead than Serial, since on waking up from standby
       the USB port could get confused and become unvailable for the host.
       To copy from Serial1 connect a  3.3V USB-to-serial converter to
       pins 13-14 (TX-RX).
    */
    Serial1.begin(115200);
    while (!Serial1) {}
    Serial1.println("INFO: Starting ...");
  }

  // SigFox ...
  if ( debug == true ) {
    Serial1.println("INFO: SigFox: Initializing ...");
  }
  if (!SigFox.begin()) {
    if ( debug == true ) {
      Serial1.println("ERROR: SigFox: Failed to initialize!");
    }
    // Initialization failure, trying to reboot ...
    reboot();
  }

  if (debug == true) {
    // Enable SigFox debug prints and LED indication
    SigFox.debug();
  }

  // Send module to standby until we need to send a message
  if ( debug == true ) {
    Serial1.println("INFO: SigFox: Standby ...");
  }
  SigFox.end();

  // Trigger setup:
  // Pins 0, 1  and 7 are connected to a switch and enable the interrupt on voltage falling event
  pinMode(0, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(0, triggerEvent1, FALLING);
  pinMode(1, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(1, triggerEvent2, FALLING);
  pinMode(7, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(7, triggerEvent3, FALLING);

  // ToFDRS setup:
  vlSetup();
}

/* *** loop ***************************************************************** */

void loop()
{
  // Sleep until an event is recognized ...
  if ( debug == true ) {
    Serial1.println("INFO: Sleeping ...");
  }
  LowPower.sleep();

  // Getting here means that an event was received ...
  SigFox.begin();

  if (debug == true) {
    Serial1.println("INFO: Event occured on trigger " + String(trigger_id));
  }
  delay(100);

  // Wait a moment ...
  if ( trigger_id == 3 ) {
    waitAndBlink(1000, 10); // Wait (and blink) for 10 seconds
  }

  // Read ToFDRS ...
  vlRead();

  // Build message:
  switch (trigger_id) {
    case 1:
      // Regarding this as calibration event for low range value ...
      message.mode[0] = 'C' ; message.mode[1] = 'L'; // CL = Calibration Low
      message.value_low = vl_range;
      break;
    case 2:
      // Regarding this as calibration event for high range value ...
      message.mode[0] = 'C' ; message.mode[1] = 'H'; // CR = Calibration High
      message.value_high = vl_range;
      break;
    case 3:
      // Regarding this as the regular reading for range value ...
      message.mode[0] = 'R' ; message.mode[1] = 'R'; // RR = Regular Reading
      break;
  }

  message.status = vl_status;
  message.value = vl_range;

  //Calculate fill-level from range values ...
  float level = 0;
  if ( message.value_high != message.value_low ) {
    float v = message.value;
    float h = message.value_high;
    float l = message.value_low;
    level = (v - h) / (l - h) * 100;
    Serial1.print("INFO: Operation: Fill-level is calculated as level = ");
    Serial1.println(level);
  }
  message.level = level < 0 ? 0 : (uint8_t) level;

  if ( debug == true ) {
    Serial1.print("INFO: Operation: Message is < ");
    Serial1.print((char *) message.mode);
    Serial1.print(" + ");
    Serial1.print(message.status);
    Serial1.print(" + ");
    Serial1.print(message.value);
    Serial1.print(" + ");
    Serial1.print(message.value_low);
    Serial1.print(" + ");
    Serial1.print(message.value_high);
    Serial1.print(" + ");
    Serial1.print(message.level);
    Serial1.println(" >");
  }

  // Send message ...
  if ( debug == true ) {
    Serial1.println("INFO: SigFox: Sending ...");
  }
  SigFox.beginPacket();
  SigFox.write((uint8_t*)&message, 12);
  int ret = SigFox.endPacket();

  // Back to standby
  if ( debug == true ) {
    Serial1.println("INFO: SigFox: Standby ...");
  }
  SigFox.end();

  // Transmission status ...
  if (debug == true) {
    if (ret > 0) {
      Serial1.println("ERROR: SigFox: No transmission");
    } else {
      Serial1.println("INFO: SigFox: Transmission ok");
    }
    Serial1.println(SigFox.status(SIGFOX));
    Serial1.println(SigFox.status(ATMEL));
    // Loop forever if we are testing for a single event
    // while (1) {};
  }
}

/* *** functions ************************************************************ */

// Trigger callback

void triggerEvent1() {
  trigger_id = 1;
}

void triggerEvent2() {
  trigger_id = 2;
}

void triggerEvent3() {
  trigger_id = 3;
}

// ToFDSR

void vlSetup() {
  if ( debug == true ) {
#ifdef TOFDRS_VL53L0X
    Serial1.println("INFO: TOFDRS: VL53L0X: Setup ...");
#elif defined TOFDRS_VL6180X
    Serial1.println("INFO: TOFDRS: VL6180X: Setup ...");
#else
    Serial1.println("WARNING: No ToFDRS defined!!!");
#endif
  }
#if defined(TOFDRS_VL53L0X) || defined(TOFDRS_VL6180X)
  if (! vl.begin()) {
    if ( debug == true ) {
      Serial1.println("FATAL: TOFDRS: Failed to find sensor!");
    }
    while (1);
  }
#endif
  if ( debug == true ) {
    Serial1.println("INFO: ToFDRS: Sensor initialized!");
  }
}

#ifdef TOFDRS_VL53L0X
void vlRead() {
  VL53L0X_RangingMeasurementData_t measure;
  if ( debug == true ) {
    Serial1.println("INFO: VL53L0X: Reading ...");
  }
  vl.rangingTest(&measure, debug); // pass in 'true' to get debug data printout!
  vl_status = measure.RangeStatus;
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    vl_range = measure.RangeMilliMeter;
    if ( debug == true ) {
      Serial.print("INFO: VL53L0X: Range: "); Serial.println(measure.RangeMilliMeter);
    }
  } else {
    vl_range = 0;
    if ( debug == true ) {
      Serial.println("ERROR: VL53L0X: Out of range!");
    }
  }
  vl_lux = 0;
}
#elif defined TOFDRS_VL6180X
void vlRead() {
  if ( debug == true ) {
    Serial1.println("INFO: VL6180X: Reading ...");
  }
  vl_lux = vl.readLux(VL6180X_ALS_GAIN_5);
  vl_range = vl.readRange();
  vl_status = vl.readRangeStatus();
  if ( debug == true ) {
    Serial1.print("INFO: VL6180X: Lux: "); Serial1.println(vl_lux);
    if (vl_status == VL6180X_ERROR_NONE) {
      Serial1.print("INFO: VL6180X: Range: "); Serial1.println(vl_range);
    }
    if  ((vl_status >= VL6180X_ERROR_SYSERR_1) && (vl_status <= VL6180X_ERROR_SYSERR_5)) {
      Serial1.println("ERROR: VL6180X: System error");
    }
    else if (vl_status == VL6180X_ERROR_ECEFAIL) {
      Serial1.println("ERROR: VL6180X: ECE failure");
    }
    else if (vl_status == VL6180X_ERROR_NOCONVERGE) {
      Serial1.println("ERROR: VL6180X: No convergence");
    }
    else if (vl_status == VL6180X_ERROR_RANGEIGNORE) {
      Serial1.println("ERROR: VL6180X: Ignoring range");
    }
    else if (vl_status == VL6180X_ERROR_SNR) {
      Serial1.println("ERROR: VL6180X: Signal/Noise error");
    }
    else if (vl_status == VL6180X_ERROR_RAWUFLOW) {
      Serial1.println("ERROR: VL6180X: Raw reading underflow");
    }
    else if (vl_status == VL6180X_ERROR_RAWOFLOW) {
      Serial1.println("ERROR: VL6180X: Raw reading overflow");
    }
    else if (vl_status == VL6180X_ERROR_RANGEUFLOW) {
      Serial1.println("ERROR: VL6180X: Range reading underflow");
    }
    else if (vl_status == VL6180X_ERROR_RANGEOFLOW) {
      Serial1.println("ERROR: VL6180X: Range reading overflow");
    }
  }
}
#else
void vlRead() {
  vl_status = 0;
  vl_lux = 42;
  vl_range = vl_range + 1;
  if ( debug == true ) {
    Serial1.println("WARNING: No ToFDRS defined!!!");
    Serial1.print("INFO: DUMMY: Lux: "); Serial1.println(vl_lux);
    Serial1.print("INFO: DUMMY: Range: "); Serial1.println(vl_range);
  }
}
#endif

// Auxiliary

void waitAndBlink(int msec, int count) {
  for (int i = 0; i < count; i++) {
    if ( debug == true ) {
      Serial1.print("INFO: Waiting for ");
      Serial1.print(msec * (count - i));
      Serial1.println(" milliseconds ...");
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(msec / 2);
    digitalWrite(LED_BUILTIN, LOW);
    delay(msec / 2);
  }
}

// System

void reboot() {
  NVIC_SystemReset();
  while (1);
}
