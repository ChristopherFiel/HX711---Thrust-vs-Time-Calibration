/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Modified for Static Propulsion Testing
   -------------------------------------------------------------------------------------
*/

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// Pins:
const int HX711_dout = 4; // MCU > HX711 dout pin
const int HX711_sck = 5;  // MCU > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
bool lastButtonState = HIGH; // For button debouncing/state tracking

void setup() {
  Serial.begin(57600); delay(10);
  
  // Setup hardware tare button
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP); 

  Serial.println();
  Serial.println("Starting Setup...");

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; 
  boolean _tare = true; 
  LoadCell.start(stabilizingtime, _tare);
  
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    // Attempt to read the saved calibration value from EEPROM
    float calValue;
    EEPROM.get(calVal_eepromAdress, calValue);
    
    // If EEPROM has a valid-looking float, use it. Otherwise default to 1.0
    if (calValue != 0.0 && !isnan(calValue)) {
      LoadCell.setCalFactor(calValue);
      Serial.print("Loaded calibration value from EEPROM: ");
      Serial.println(calValue);
    } else {
      LoadCell.setCalFactor(1.0); 
      Serial.println("No valid calibration found. Defaulting to 1.0.");
    }
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  
  // Print the CSV Header for data logging
  Serial.println("Time(ms),Thrust");
}

void loop() {
  static boolean newDataReady = 0;

  // 1. Check Hardware Tare Button
  bool currentButtonState = digitalRead(TARE_BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    LoadCell.tareNoDelay(); // Zero the load cell
  }
  lastButtonState = currentButtonState;

  // 2. Check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // 3. Output Data in CSV Format (Time, Thrust)
  if (newDataReady) {
      float thrust = LoadCell.getData();
      unsigned long currentTime = millis();
      
      Serial.print(currentTime);
      Serial.print(",");
      Serial.println(thrust);
      
      newDataReady = 0;
  }

  // 4. Receive command from serial terminal (for calibration/settings)
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); // tare
    else if (inByte == 'r') calibrate(); // calibrate
    else if (inByte == 'c') changeSavedCalFactor(); // edit calibration value manually
  }

  // 5. Check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
    Serial.println("Time(ms),Thrust"); // Reprint header after tare info
  }
}

// -------------------------------------------------------------------
// CALIBRATION FUNCTIONS (Unchanged from your original code)
// -------------------------------------------------------------------

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

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

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); 
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass);

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

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
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
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
  Serial.println("Time(ms),Thrust"); // Resume CSV format
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
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
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
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
  Serial.println("Time(ms),Thrust"); // Resume CSV format
}