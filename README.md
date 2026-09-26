# Arduino Shake and Heat Monitor

This project monitors environmental vibration and temperature using an Arduino. It utilizes an ADXL345 accelerometer to detect sudden shakes and a DS18B20 sensor to track temperature. If the shake magnitude exceeds the defined threshold or the temperature rises above 31.0°C, the system triggers an alert via an LED.

## 🚀 Features
* **Vibration Detection:** Reads 3-axis acceleration data (x, y, z) from the ADXL345 sensor via I2C and calculates the magnitude to detect shakes.
* **Temperature Monitoring:** Reads precise temperature data from the DS18B20 digital sensor via the OneWire protocol.
* **Alert System:** Activates an LED (Pin 5) when a shake is detected or the temperature exceeds the 31.0°C threshold.
* **Serial Output:** Logs real-time acceleration (g) and temperature (°C) data to the Serial Monitor at 9600 baud.

## 🛠️ Hardware Requirements
* Arduino Board (e.g., Uno, Nano)
* ADXL345 Accelerometer (I2C Address: 0x53)
* DS18B20 Digital Temperature Sensor
* 1x LED (Alert indicator)
* Jumper wires & Breadboard

## 📚 Required Libraries
The required custom libraries are included in the `libraries` folder of this repository. 
* `Wire.h` (Built-in)
* `math.h` (Built-in)
* `OneWire.h`
* `DallasTemperature.h`

## ⚙️ Setup and Usage
1. Connect the ADXL345 to the I2C pins (SDA, SCL).
2. Connect the DS18B20 data pin to Arduino Pin A0.
3. Connect the alert LED to Digital Pin 5.
4. Upload the `.ino` sketch to your Arduino.
5. Open the Serial Monitor (9600 baud) to view real-time sensor data.