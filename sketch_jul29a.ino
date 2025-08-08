#include <Wire.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- ADXL345 tanımlamaları ---
#define ADXL345_ADDRESS 0x53
#define SHAKE_THRESHOLD 0.5  // g cinsinden sarsıntı eşiği

// --- DS18B20 tanımlamaları ---
OneWire oneWire(A0);
DallasTemperature DS18B20(&oneWire);
DeviceAddress DS18B20adres;
const float SICAKLIK_ESIGI = 31.0; // °C

// --- Genel tanımlar ---
const int LED_PIN = 5;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // ADXL345 başlatma
  writeRegister(0x2D, 0x08); // POWER_CTL: ölçüm modu
  writeRegister(0x31, 0x08); // DATA_FORMAT: full resolution, ±2g
  writeRegister(0x2C, 0x0A); // BW_RATE: 100 Hz

  // DS18B20 başlatma
  DS18B20.begin();
  DS18B20.getAddress(DS18B20adres, 0);
  DS18B20.setResolution(DS18B20adres, 12);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Proje basladi: Sarsinti ve Sicaklik izleniyor.");
}

void loop() {
  // --- ADXL345 Sarsıntı Okuma ---
  uint8_t data[6];
  int16_t x, y, z;
  readRegisters(0x32, 6, data);

  x = (int16_t)((data[1] << 8) | data[0]);
  y = (int16_t)((data[3] << 8) | data[2]);
  z = (int16_t)((data[5] << 8) | data[4]);

  const float scaleFactor = 0.0039; // g/LSB (±2g full resolution)
  float xg = x * scaleFactor;
  float yg = y * scaleFactor;
  float zg = z * scaleFactor;
  float magnitude = sqrt(xg * xg + yg * yg + zg * zg);

  // --- DS18B20 Sıcaklık Okuma ---
  DS18B20.requestTemperatures();
  float sicaklik = DS18B20.getTempC(DS18B20adres);

  // --- Seri monitöre yazdır ---
  Serial.print("Ivme (g): ");
  Serial.print(magnitude, 3);
  Serial.print(" | Sicaklik (C): ");
  Serial.println(sicaklik, 2);

  // --- Sarsıntı kontrol ---
  if (abs(magnitude - 1.0) > SHAKE_THRESHOLD) {
    Serial.println(">>> SARSINTI ALGILANDI <<<");
  }

  // --- Sıcaklık kontrol ve LED ---
  if (sicaklik >= SICAKLIK_ESIGI || abs(magnitude - 1.0) > SHAKE_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);  // LED yak
  } else {
    digitalWrite(LED_PIN, LOW);   // LED söndür
  }

  delay(500);
}

// --- Yardımcı fonksiyonlar ---
void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t* buffer) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADXL345_ADDRESS, (uint8_t)count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}
