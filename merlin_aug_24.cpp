#include <Wire.h>
#include <SparkFun_SCD4x_Arduino_Library.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_Sensor.h>
#include "thingProperties.h"

#define LED_PIN 4
#define FAN_PIN 7
#define HUMI_FAN_PIN 8
#define HUMI_M_PIN 12
#define DELAY_TIME 2000
//#define SERIAL_BAUD_RATE 9600
#define DEBUG_MESSAGE_LEVEL 2
#define SERIAL_INIT_DELAY 0

SensirionI2CScd4x scd4x;

unsigned long lastUpdate = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HUMI_FAN_PIN, OUTPUT);
  pinMode(HUMI_M_PIN, OUTPUT);

  Serial.begin(9600);
  //delay(SERIAL_INIT_DELAY); 

  Wire1.begin();
  scd4x.begin(Wire1);
  scd4x.startPeriodicMeasurement();
  
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(DEBUG_MESSAGE_LEVEL);
  ArduinoCloud.printDebugInfo();
  
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(HUMI_FAN_PIN, LOW);
  digitalWrite(HUMI_M_PIN, LOW);
  
  //pinMode(FAN_PIN, OUTPUT);
  //analogWrite(FAN_PIN, mapFanSpeedToPWM(fan_speed)); // Set initial fan speed

}

int mapFanSpeedToPWM(String speed) {
  if (speed == "ultra") return 255; // 12V
  if (speed == "high") return 191; // 9V (191/255 * 12V)
  if (speed == "medium") return 148; // 7V (148/255 * 12V)
  if (speed == "low") return 106; // 5V (106/255 * 12V)
  return 0; // Turn off the fan if the speed is not recognized
}

void loop() {
  ArduinoCloud.update();

  if (millis() - lastUpdate > DELAY_TIME) {
    lastUpdate = millis();

    // Reading CO2, Temperature, and Humidity from SCD4x
    uint16_t co2_raw;
    float temperature_raw, humidity_raw;
    if (scd4x.readMeasurement(co2_raw, temperature_raw, humidity_raw) == 0) {
      co2 = co2_raw;
      temp = round((temperature_raw * 9.0/5.0 + 32) * 100.0) / 100.0;  // Convert to Fahrenheit and round to 2 decimal places
      humi = humidity_raw;
    //} else {
      //Serial.println("Failed to read from SCD4x!");
    }

    if (!isnan(humi) && !isnan(temp) && !isnan(co2)) {
      Serial.print("Humidty: "); Serial.print(humi); Serial.print("%"); Serial.print(" | ");
      Serial.print("Temperature: "); Serial.print(temp); Serial.println("°F");
      Serial.print("Air Quality (SCD4x CO2 PPM): "); Serial.println(co2);

      // Trigger fan based on sensor readings
      triggerTubFan();
      triggerHumiFan();
    }
  }
}
void triggerTubFan() {
  static bool isFanOnDueToHighHumi = false; // Flag to track if the fan was turned on due to high humidity
  bool shouldTurnOnFan = false;
  static bool isFanOnDueToCO2 = false; // Made static so its value is retained between function calls
  float fan_threshold = (humi_high + 3);

  if (fan_switch) {
    Serial.println("Fan switch is ON.");
    shouldTurnOnFan = true;
    isFanOnDueToCO2 = false; // Reset this flag when manually switching on
    isFanOnDueToHighHumi = false; // Reset this flag when manually switching on
  } 
  else if (temp < temp_low) {
    Serial.println("Turning off fan due to temperature below temp_low.");
    shouldTurnOnFan = false; // Ensure the fan is turned off
    isFanOnDueToHighHumi = false; // Reset this flag
  }
  else if (humi < (humi_low)) {
    Serial.println("Turning off fan due to low humidity.");
    shouldTurnOnFan = false; // Ensure the fan is turned off
    isFanOnDueToHighHumi = false; // Reset this flag
  }
  else if (humi > humi_high || (isFanOnDueToHighHumi && humi > fan_threshold)) {
    Serial.println("Turning on fan due to high humidity.");
    shouldTurnOnFan = true; 
    isFanOnDueToCO2 = false; // Reset this flag when triggered by humidity
    isFanOnDueToHighHumi = true; // Set this flag when triggered by high humidity
  }
  else if (temp > temp_high) {
    Serial.println("Turning on fan due to high temperature.");
    shouldTurnOnFan = true;
    isFanOnDueToCO2 = false; // Reset this flag when triggered by temperature
    isFanOnDueToHighHumi = false; // Reset this flag
  }
  else if (co2 > co2_high) {
    Serial.println("Turning on fan due to high CO2 levels.");
    shouldTurnOnFan = true;
    isFanOnDueToCO2 = true; // Set this flag when triggered by high CO2
    isFanOnDueToHighHumi = false; // Reset this flag
  }
  else if (isFanOnDueToCO2 && co2 > (co2_high - co2_offset)) {
    // If fan was turned on due to CO2 and current CO2 is still 500 ppm above the desired level, keep it running
    Serial.println("Maintaining fan ON to lower CO2 levels.");
    shouldTurnOnFan = true;
  }
  else if (humi <= fan_threshold) {
    Serial.println("Turning off fan as humidity is back to desired levels.");
    shouldTurnOnFan = false;
    isFanOnDueToHighHumi = false; // Reset this flag when humidity is back to desired levels
  }
  else {
    isFanOnDueToCO2 = false; // Reset the flag when CO2 is back to desired levels or when other conditions don't match
    isFanOnDueToHighHumi = false; // Reset the flag when humidity is back to desired levels
  }
  
  digitalWrite(FAN_PIN, shouldTurnOnFan ? HIGH : LOW);
}

void triggerHumiFan() {
  static bool isHumiFanOnDueToLowHumi = false; // Flag to track if the fan was turned on due to low humidity
  bool shouldTurnOnHumiFan = false;
  float humi_threshold = (humi_low + 1);

  if (humi_fan_switch) {
    Serial.println("HUMI Fan switch is ON.");
    shouldTurnOnHumiFan = true;
    isHumiFanOnDueToLowHumi = false; // Reset this flag when manually switching on
  } else if (humi > humi_high) {
    Serial.println("Turning off HUMI fan due to high humidity.");
    shouldTurnOnHumiFan = false;
    isHumiFanOnDueToLowHumi = false; // Reset this flag when humidity is high
  } else if (humi < humi_low || (isHumiFanOnDueToLowHumi && humi < humi_threshold)) {
    Serial.println("Turning on HUMI fan due to low humidity.");
    shouldTurnOnHumiFan = true;
    isHumiFanOnDueToLowHumi = true; // Set this flag when humidity is low
  } else if (isHumiFanOnDueToLowHumi && humi >= humi_threshold) {
    Serial.println("Turning off HUMI fan as temperature threshold is reached.");
    shouldTurnOnHumiFan = false;
    isHumiFanOnDueToLowHumi = false; // Reset this flag when temperature threshold is reached
  }

  digitalWrite(HUMI_FAN_PIN, shouldTurnOnHumiFan ? HIGH : LOW);
}



void onLedSwitchChange() {
  digitalWrite(LED_PIN, led_switch ? HIGH : LOW);
}

void onFanSwitchChange() {
  digitalWrite(FAN_PIN, fan_switch ? HIGH : LOW);
}

void onHumiChange() {
  // Add your code here to act upon Humi change
}

void onTempThresholdChange() {
  // Add your code here to act upon temperature threshold change
}

void onHumiThresholdChange() {
  // Add your code here to act upon humidity threshold change
}

void onCo2ThresholdChange() {
  // Add your code here to act upon CO2 threshold change
}

void onHumiFanChange() {
  // Add your code here to act upon HumiFan change
  digitalWrite(HUMI_FAN_PIN, humi_fan_switch ? HIGH : LOW);
}

void onHumiMachineChange() { 
  // Add your code here to act upon HumiMachine change
  digitalWrite(HUMI_M_PIN, humi_machine_switch ? HIGH : LOW);
}

// Other functions (if any) for IoT Cloud variables


/*
  Since CO2 is READ_WRITE variable, onCO2Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onCo2Change() {
  // Add your code here to act upon Co2 change
}

/*
  Since TempLow is READ_WRITE variable, onTempLowChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTempLowChange()  {
  // Add your code here to act upon TempLow change
}
/*
  Since TempHigh is READ_WRITE variable, onTempHighChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTempHighChange()  {
  // Add your code here to act upon TempHigh change
}
/*
  Since HumiLow is READ_WRITE variable, onHumiLowChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onHumiLowChange()  {
  // Add your code here to act upon HumiLow change
}
/*
  Since HumiHigh is READ_WRITE variable, onHumiHighChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onHumiHighChange()  {
  // Add your code here to act upon HumiHigh change
}
/*
  Since Co2Low is READ_WRITE variable, onCo2LowChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onCo2LowChange()  {
  // Add your code here to act upon Co2Low change
}
/*
  Since Co2High is READ_WRITE variable, onCo2HighChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onCo2HighChange()  {
  // Add your code here to act upon Co2High change
}
/*
  Since HumiMachineSwitch is READ_WRITE variable, onHumiMachineSwitchChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onHumiMachineSwitchChange()  {
  // Add your code here to act upon HumiMachineSwitch change
}
/*
  Since HumiFanSwitch is READ_WRITE variable, onHumiFanSwitchChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onHumiFanSwitchChange()  {
  // Add your code here to act upon HumiFanSwitch change
}

void onCo2OffsetChange() {
  // Add your code here to act upon CO2 Offset change
}
/*
  Since FanSpeed is READ_WRITE variable, onFanSpeedChange() is
  executed every time a new value is received from IoT Cloud.
*/
//void onFanSpeedChange()  {
  //analogWrite(FAN_PIN, mapFanSpeedToPWM(fan_speed));
//}
/*
  Since Temp is READ_WRITE variable, onTempChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTempChange()  {
  // Add your code here to act upon Temp change
}