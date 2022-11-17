//
// American Heritage Science Olympiad Detector Building
// Henry LeCompte and Grace Jiang 11/16/2022
//

#include <ssd1306.h>
#include <ssd1306_16bit.h>
#include <ssd1306_1bit.h>
#include <ssd1306_8bit.h>
#include <ssd1306_console.h>
#include <ssd1306_fonts.h>
#include <ssd1306_generic.h>
#include <ssd1306_uart.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h> //NAU7802 Library

#define TEAMNAME "Protons"
#define TEAMNUMBER "C2"

// Setup Mass Zones and LED colors here
// For each zone, set the range from low to high

#define REDLED 11   // Digital GPIO Pin 11
#define GREENLED 12 // Digital GPIO Pin 12
#define BLUELED 13  // Digital GPIO Pin 13

#define CALIBRATION_BUTTON 10 // Digital GPIO Pin 2

// Set Sample rate per second

#define SAMPLERATE 320

// Idea is to light the corresponding LED if measured weight is in between low to high for each zone
// If the ranges overlap, unless bMultiZoneLED is set to True, the first corrsponding range LED will light
// If they want LEDS to light if the measured weight exceeds low range, then set each zone high to same very high value such as 10000gms that will never be reached.

float fMassZone1Low = 25.0;
float fMassZone1High = 120.0;

float fMassZone2Low = 190.50;
float fMassZone2High = 450.0;

float fMassZone3Low = 500.0;
float fMassZone3High = 1200.0;

float fCalibrationFactor = 1262; // Voltage to mass conversion factor

NAU7802 nau;      // Create instance of the NAU7802 class
long lNoiseFloor; // holds the moving average calculated over nn samples to detect what system noise floor is

// Get the average reading over iNumSamples at the rate of SAMPLERATE
long getAvgReading(int iNumSamples)
{
  long lAvgReading = 0;

  for (int i = 0; i < iNumSamples; i++)
  {
    if (nau.available())
    {
      lAvgReading += nau.getReading();
      delay(1000 / SAMPLERATE);
    }
  }
  return lAvgReading / iNumSamples;
}

// This function converts the ADC reading to voltage
double readingToVoltage(long lReading)
{
  return (abs(lReading - lNoiseFloor) * 0.00001 / 3.3);
}

// This function converts the voltage to mass
double getMass(double dVoltage)
{
  return 1262 * dVoltage; // 1262 is the calibration factor
}

void getCalibrationData(int samples, long &lAvg, long &lMin, long &lMax)
{
  long lCurr;
  ssd1306_printFixed(1, 0, "Calibrating...", STYLE_BOLD);
  for (int j = 0; j < samples; j++)
  {                                                      // sample rate and number of samples will determine how many secs of wall time to measure
    ssd1306_drawProgressBar(int(j / (samples / 100.0))); // show progress bar of calibration
    if (nau.available() == true)                         // long as ADC NAU7802 has a sampled value available, get empty scale noise readings
    {
      lCurr = nau.getReading(); // get reading from the ADC chip
      if (lMax < lCurr)
        lMax = lCurr; // compute local max
      if (lMin > lCurr)
        lMin = lCurr; // compute local min
      lAvg += lCurr;  // accumulate sum of reading
      delay(1);       // wait a msec to give time for ADC to get another sample ready
    }
  }

  lAvg /= samples; // compute average - now lNoiseFloor has the noise floor number.  This is the system noise with no weight on the strain gauges
}

// Setup and initialize hardware.
// This only get runs once on each power on and main program runs after this
void setup()
{
  Wire.begin();         // start two wire protocol
  Serial.begin(115200); // start serial monitor to output debug output

  // Setup OLED Display Screen
  ssd1306_128x64_i2c_init();                  // Start GeekPi 128x64 Pixel OLED Display based on SSD1306 controller
  ssd1306_fillScreen(0x00);                   // Clear screen
  ssd1306_setFixedFont(ssd1306xled_font8x16); // select an 8 pixels wide and 16 pixels high font so at most 16 chars across (128/8) and 4 lines high (64/16)

  // Setup LEDs and test them on startup
  pinMode(REDLED, OUTPUT); // Initialize Red LED
  digitalWrite(REDLED, HIGH);
  delay(1000);
  digitalWrite(REDLED, LOW);

  pinMode(GREENLED, OUTPUT); // Initialize Green LED
  digitalWrite(GREENLED, HIGH);
  delay(1000);
  digitalWrite(GREENLED, LOW);

  pinMode(BLUELED, OUTPUT); // Initialize Blue LED
  digitalWrite(BLUELED, HIGH);
  delay(1000);
  digitalWrite(BLUELED, LOW);

  pinMode(CALIBRATION_BUTTON, INPUT_PULLUP); // Initialize Calibration Button

  // Start NAU7802 24-bit ADC Bridge
  if (nau.begin(Wire, true) == false)
  {
    ssd1306_printFixed(1, 0, "NAU7802 Not Found!", STYLE_BOLD);
    Serial.println("NAU7802 not found, check wiring");
    while (1)
      ; // loop forever, we cant proceed without the NAU7802 chip
  }
  else
  {
    Serial.println("24 bit ADC NAU7802 detected!");
    Serial.println("Selecting 80 Samples per Second");
    // This used to be set to SAMPLERATE but that internally was changed to 0x7 or 320 samples per second
    nau.setSampleRate(NAU7802_SPS_320); // Set sample rate to 320 samples per second (This is the max sample rate)
    Serial.println("Setting ADC gain to 128");
    nau.setGain(NAU7802_GAIN_128);
  }

  //----------------------------------------------------------------------------------
  // Calibrating noise floor with moving averages
  // No weight on sensor and do not touch anything while its calibrating for noise
  long lMin = 100000000;
  long lMax = 0;
  int iNumSamples = SAMPLERATE * 2; // how many secs worth of samples to get, get minimum of 2 secs

  getCalibrationData(iNumSamples, lNoiseFloor, lMin, lMax);

  Serial.print("Moving Average NoiseFloor: ");
  Serial.println(lNoiseFloor);
  Serial.print("Min Max Average Noise Floor min:");
  Serial.print(lMin); // min, max sanity check that our moving average is roughly accurate
  Serial.print("  max:");
  Serial.print(lMax);
  Serial.print("  avg NoiseFloor:");
  Serial.println((lMin + lMax) / 2);

  ssd1306_clearScreen(); // Clear screen

  if (digitalRead(CALIBRATION_BUTTON) == LOW) // If button is pressed, then calibrate
  {
    ssd1306_printFixed(1, 0, "Calibration Started", STYLE_BOLD);
    ssd1306_printFixed(1, 16, "200g mass", STYLE_BOLD);
    ssd1306_printFixed(1, 32, "press button", STYLE_BOLD);

    delay(5000); // Wait 5 seconds for user to place mass on scale

    while (digitalRead(CALIBRATION_BUTTON) == HIGH)
    {
      delay(100);
    }

    ssd1306_clearScreen(); // Clear screen

    long min = 100000000;
    long max = 0;
    long cal;

    getCalibrationData(iNumSamples, cal, min, max);

    Serial.print("Moving Average Calibration: ");
    Serial.println(cal);
    Serial.print("Min Max Calibration min:");
    Serial.print(min); // min, max sanity check that our moving average is roughly accurate
    Serial.print("  max:");
    Serial.println(max);

    fCalibrationFactor = 200 / readingToVoltage(cal); // 200g mass

    ssd1306_clearScreen(); // Clear screen
  }

  ssd1306_printFixed(1, 0, TEAMNAME, STYLE_BOLD); // write our Team name on First line
}

void lightLEDs(double dWeight)
{
  digitalWrite(REDLED, LOW);
  digitalWrite(GREENLED, LOW);
  digitalWrite(BLUELED, LOW);

  if (dWeight >= fMassZone1Low && dWeight <= fMassZone1High)
  {
    digitalWrite(REDLED, HIGH);
  }
  if (dWeight >= fMassZone2Low && dWeight <= fMassZone2High)
  {
    digitalWrite(GREENLED, HIGH);
  }
  if (dWeight >= fMassZone3Low && dWeight <= fMassZone3High)
  {
    digitalWrite(BLUELED, HIGH);
  }
}

// Main Program
void loop()
{
  long lCurrentReading = 0;
  float fMeasuredMass = 0.0;
  float fTareErrorMargin = 0.0;
  char cVolt[16] = {'V', 'O', 'L', 'T', ':', ' '};
  char cWeight[16] = {'G', 'R', 'A', 'M', ':', ' '};

  if (nau.available() == true)
  {
    lCurrentReading = getAvgReading(SAMPLERATE);

    Serial.print("ADC Reading: ");
    Serial.print(lCurrentReading);

    // Calculate Voltage as millivolt per ref voltage of 3.3v
    double dVoltage = readingToVoltage(lCurrentReading);
    dtostrf(dVoltage, 7, 5, &cVolt[6]);
    Serial.print(" | ");
    Serial.print(cVolt);

    // Calculate Weight in grams
    fMeasuredMass = getMass(dVoltage);

    if (fMeasuredMass < 3.0 || digitalRead(CALIBRATION_BUTTON) == LOW)
    { // Assume all changes below 3gms are creep errors in strain gauge due to temperature and drift in system noise
      fTareErrorMargin = fMeasuredMass;
    }

    fMeasuredMass = fMeasuredMass - fTareErrorMargin; // keep taring to zero by subtracting creep error

    dtostrf(fMeasuredMass, 7, 1, &cWeight[6]);
    Serial.print(" | ");
    Serial.println(fMeasuredMass);

    ssd1306_clearScreen();
    ssd1306_printFixed(1, 0, TEAMNAME, STYLE_BOLD);    // first row
    ssd1306_printFixed(1, 16, TEAMNUMBER, STYLE_BOLD); // second row
    ssd1306_printFixed(1, 32, cVolt, STYLE_NORMAL);    // third row
    ssd1306_printFixed(1, 48, cWeight, STYLE_NORMAL);  // fourth row

    // Light LEDs according to mass zone requirements of the Science Olympiad
    lightLEDs(fMeasuredMass);

    while (1)
    { // show weight and only re-measure if current sensed weight changes by more or less than 1gms from displayed weight
      lCurrentReading = getAvgReading(SAMPLERATE / 2);
      dVoltage = readingToVoltage(lCurrentReading);
      double newWeightRaw = getMass(dVoltage);
      if ((newWeightRaw > fMeasuredMass + 1.0) || newWeightRaw < fMeasuredMass - 1.0 || digitalRead(CALIBRATION_BUTTON) == LOW)
        break;
    }
  }
}