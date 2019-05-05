/*

  Teensy LC - Pinball plunger and nudge

*/

#include <Arduino.h>
#include <Wire.h>
#include <BMI160Gen.h>
#include <Bounce2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

const int maxbuttons = 21;
Bounce debouncer[maxbuttons];
const int BUTTON_PIN[maxbuttons] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,                   11,                   12,      14, 15,        16, 17, 20, 21, 22, 23};
const int BUTTON_KEY[maxbuttons] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KEY_MEDIA_VOLUME_INC, KEY_MEDIA_VOLUME_DEC, KEY_ESC, 00, KEY_ENTER, 00, 00, 00, 00, 00, 00}; // if keyboard instead of joystick

#define led_pin 13 // LED_BUILTIN

int plungerout = 200, plungerin = 24100, plungernormal = 10000;
// 10000 needs to be calibrated !!!!
// plunger at 0x39

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
uint16_t broadband, ir, plunger;


bool DoDebug = false;

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}



void setup()
{
  for (int i = 0; i < maxbuttons; i++) {
    debouncer[i] = Bounce();
    debouncer[i].attach(BUTTON_PIN[i], INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
    debouncer[i].interval(10); // Use a debounce interval of 25 milliseconds
  }
  Joystick.useManualSend(true);

  Serial.begin(115200);
  //  while (!Serial);    // wait for the serial port to open

  Serial.println();
  Serial.println("setup");

  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, 1);

  Serial.println("Initializing IMU device...");
  const int i2c_addr = 0x69;
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 250 degrees/second
  //BMI160.setGyroRange(250);

  Serial.println("before accel calibration: ");

  delay(1000);
  uint8_t offsetx = BMI160.getXAccelOffset();
  uint8_t offsety = BMI160.getYAccelOffset();


  Serial.print("offset x: ");
  Serial.print(offsetx);
  Serial.print("\t");
  Serial.print(offsety);
  Serial.println();


  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.autoCalibrateYAccelOffset(0);
  BMI160.autoCalibrateZAccelOffset(0);
  BMI160.setAccelOffsetEnabled(true);

  BMI160.autoCalibrateGyroOffset();
  BMI160.setGyroOffsetEnabled(true);


  Serial.println("Initializing IMU device...done.");

  offsetx = BMI160.getXAccelOffset();
  offsety = BMI160.getYAccelOffset();


  Serial.print("offset x: ");
  Serial.print(offsetx);
  Serial.print("\t");
  Serial.print(offsety);
  Serial.println();


  Serial.println("Initializing plunger  device..."); // TSL2561

  if (!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.println("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
  }
  else {
    /* Setup the sensor gain and integration time */
    configureSensor();



    long plungercelibrate = 0;

    for (short i = 0; i < 5; i++) {
      tsl.getLuminosity(&broadband, &ir);
      plunger = broadband * ir;
      plungercelibrate += plunger;
      Serial.println(plungercelibrate);
      Serial.println(plunger);
    }
    plungernormal = plungercelibrate / 5;
    Serial.print("Calibrating plunger - default = ");
    Serial.println(plungernormal);
    Serial.println();
    Serial.println("Initializing plunger device...done.");
  }


  if ( !digitalRead(14)) {
    DoDebug = true;
    Serial.println("Debug an, warte 10 sec");
    delay(10000);
  }
  else
    Serial.println("Debug off!");

  keyboard_modifier_keys = 0;
  keyboard_keys[0] = 0;
  keyboard_keys[1] = 0;
  keyboard_keys[2] = 0;
  keyboard_keys[3] = 0;
  keyboard_keys[4] = 0;
  keyboard_keys[5] = 0;

}



void loop()
{

  //int gxRaw, gyRaw, gzRaw;         // raw gyro values
  int16_t xaccel, yaccel, zaccel;




  for (int i = 0; i < maxbuttons; i++) {
    debouncer[i].update(); // Update the Bounce instance
    if ( debouncer[i].fell() ) {  // Call code if button transitions from HIGH to LOW
      if (BUTTON_KEY[i]) {  //keyboard assigned
        Keyboard.press(BUTTON_KEY[i]);
        if (DoDebug) {
          Serial.print("keyboard!!: ");
          Serial.println(BUTTON_KEY[i]);
        }
      }
      else
        Joystick.button(i + 1, 1);    // Release button "num" (1 to 32)

      digitalWrite(led_pin, 1);
      if (DoDebug) {
        Serial.print("Button an: ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(BUTTON_PIN[i]);
        delay(500);
      }
    }
    if ( debouncer[i].rose() ) {  // Call code if button transitions from HIGH to LOW
      if (BUTTON_KEY[i]) {   // keyboard assigned
        Keyboard.release(BUTTON_KEY[i]);
      }
      else
        Joystick.button(i + 1, 0);    // Release button "num" (1 to 32)
      digitalWrite(led_pin, 0);
      if (DoDebug) {
        Serial.print("Button aus: ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(BUTTON_PIN[i]);
        delay(500);
      }
    }
  }


  BMI160.getAcceleration(&xaccel, &yaccel, &zaccel);

  if (DoDebug) {
    Serial.print("accel:\t");
    Serial.print(xaccel);
    Serial.print("\t");
    Serial.print(yaccel);
    Serial.println();
  }

  // xyaccel -16384 bis +16384
  xaccel = map(xaccel, -16384, 16384, 0, 1023);
  yaccel = map(yaccel, -16384, 16384, 0, 1023);

  if (DoDebug) {
    Serial.print("accel joystick:\t");
    Serial.print(xaccel);
    Serial.print("\t");
    Serial.print(yaccel);
  }

  tsl.getLuminosity(&broadband, &ir);
  plunger = broadband * ir;
  if (plunger > plungernormal)
    zaccel = map(plunger, plungernormal, plungerin, 512, 1023);
  else
    zaccel = map(plunger, plungerout, plungernormal, 0, 512);

  if (DoDebug) {
    Serial.print("\t");
    Serial.print(zaccel);
    Serial.print("\t");
    Serial.print(plunger);
    Serial.println();
  }

  Joystick.X(xaccel);            // "value" is from 0 to 1023
  Joystick.Y(yaccel);            //   512 is resting position
  Joystick.Z(zaccel);
  Joystick.send_now();


  //delay(250);

}


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}
