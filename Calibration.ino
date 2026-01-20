/*
  Static Thrust Test Logger
  - Based on HX711_ADC example you provided
  - Clamps negative load readings to 0.00
  - Prints time in seconds (3 decimals) and load (2 decimals)
  - Two output modes: human-readable and CSV (CSV is suitable for Excel/Python)
  - Default sampling rate: 100 Hz (can be changed from serial)
  - Serial commands: t= tare, r= recalibrate, c= change cal factor, m=toggle mode, f= set sample freq, h=help
  - Saves calibration to EEPROM (same address = 0)

  Compile with: Arduino IDE + HX711_ADC library
*/

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// pins
const int HX711_dout = 4; // MCU -> HX711 DOUT
const int HX711_sck  = 5; // MCU -> HX711 SCK

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0; // EEPROM address to store calibration
unsigned long startTimeMs = 0;
unsigned long lastSampleMs = 0;
unsigned long sampleIntervalMs = 10; // default 10 ms => 100 Hz

bool csvMode = false; // false = human readable, true = csv
bool printingHeader = false;

// helper: clamp negative to zero
float clampZero(float v) {
  if (v < 0.0f) return 0.0f;
  return v;
}

void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting HX711 Static Thrust Logger...");

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // ms
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check wiring and pins");
    while (1);
  } else {
    // try to read saved calibration from EEPROM
    float savedCal = 1.0;
#if defined(ESP8266)|| defined(ESP32)
    EEPROM.begin(512);
#endif
    EEPROM.get(calVal_eepromAdress, savedCal);
    if (!isnan(savedCal) && savedCal != 0.0) {
      LoadCell.setCalFactor(savedCal);
      Serial.print("Loaded calFactor from EEPROM: ");
      Serial.println(savedCal, 6);
    } else {
      LoadCell.setCalFactor(1.0);
      Serial.println("Using default calFactor = 1.0 (no EEPROM value found)");
    }

    Serial.println("Startup complete");
  }

  // wait for the first dataset
  while (!LoadCell.update());

  // run calibration routine on first run
  calibrate();

  // set start time AFTER calibration so t=0 aligns with post-calibration
  startTimeMs = millis();
  lastSampleMs = startTimeMs;

  printHelp();

  // optionally print CSV header immediately if csvMode true
  if (csvMode && !printingHeader) {
    Serial.println("time_s,load");
    printingHeader = true;
  }
}

void loop() {
  // update HX711 (non-blocking)
  if (LoadCell.update()) {
    // nothing else here; we use timed sampling below
  }

  // sample at requested rate
  unsigned long now = millis();
  if (now - lastSampleMs >= sampleIntervalMs) {
    lastSampleMs += sampleIntervalMs; // keep consistent spacing

    float rawLoad = LoadCell.getData(); // already smoothed by library
    float load = clampZero(rawLoad);

    float timeSec = (now - startTimeMs) / 1000.0f;

    if (csvMode) {
      // CSV: time_s,load (ready to paste into spreadsheet)
      Serial.print(timeSec, 3);
      Serial.print(",");
      Serial.println(load, 2);
    } else {
      // Human readable: t = 0.123 s    Load = 12.34
      Serial.print("t = ");
      Serial.print(timeSec, 4);
      Serial.print(" s\t");
      Serial.print("Force(N) = ");
      Serial.println(load, 4);
    }
  }

  // check serial commands
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    // consume any trailing newline characters
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) Serial.read();

    switch (inByte) {
      case 't':
        Serial.println("Tare requested");
        LoadCell.tareNoDelay();
        break;
      case 'r':
        Serial.println("Recalibrate requested");
        calibrate();
        // reset start time after recalibration so t=0 after new cal
        startTimeMs = millis();
        lastSampleMs = startTimeMs;
        break;
      case 'c':
        changeSavedCalFactor();
        break;
      case 'm':
        csvMode = !csvMode;
        Serial.print("Mode toggled. CSV mode = "); Serial.println(csvMode ? "ON" : "OFF");
        if (csvMode && !printingHeader) {
          Serial.println("time_s,load");
          printingHeader = true;
        }
        break;
      case 'f':
        Serial.println("Enter sample frequency in Hz (e.g. 100)");
        // wait for a float/integer from serial
        {
          unsigned long startWait = millis();
          while (millis() - startWait < 5000 && Serial.available() == 0) { /* wait up to 5s */ }
          if (Serial.available() > 0) {
            float newHz = Serial.parseFloat();
            if (newHz > 0.0) {
              sampleIntervalMs = (unsigned long)(1000.0f / newHz);
              Serial.print("Sample rate set to: "); Serial.print(newHz, 1); Serial.println(" Hz");
            } else {
              Serial.println("Invalid frequency, ignored");
            }
          } else {
            Serial.println("No frequency received, ignored");
          }
        }
        break;
      case 'h':
        printHelp();
        break;
      default:
        // ignore unknown single-byte commands
        break;
    }
  }

  // optional short delay to yield CPU (not to affect sampling timing above)
  delay(1);
}

void calibrate() {
  Serial.println("*** Start calibration procedure");
  Serial.println("Place the load cell on a stable surface with no load.");
  Serial.println("Send 't' from serial monitor to tare.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') LoadCell.tareNoDelay();
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now place a known mass on the load cell and send its mass (e.g. 100.0)");
  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: "); Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet();
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass);

  Serial.print("New calibration value has been set to: ");
  Serial.println(newCalibrationValue, 6);
  Serial.print("Save this value to EEPROM address ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? (y/n)");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value "); Serial.print(newCalibrationValue, 6);
        Serial.print(" saved to EEPROM address: "); Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("*** Change calibration factor");
  Serial.print("Current value is: "); Serial.println(oldCalibrationValue, 6);
  Serial.println("Send the new value from serial monitor, e.g. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: "); Serial.println(newCalibrationValue, 6);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM address "); Serial.print(calVal_eepromAdress); Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value "); Serial.print(newCalibrationValue, 6);
        Serial.print(" saved to EEPROM address: "); Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}

void printHelp() {
  Serial.println("\nSerial commands:");
  Serial.println("  t - tare (no delay)");
  Serial.println("  r - re-run calibration routine");
  Serial.println("  c - change calibration factor manually");
  Serial.println("  m - toggle output mode (human-readable / CSV)");
  Serial.println("  f - set sample frequency in Hz (follow by number)");
  Serial.println("  h - print this help message\n");
}
