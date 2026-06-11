/*
 * I2C scanner dla ESP32. Wgraj zamiast głównego sketcha żeby
 * znaleźć adres INA219 (lub stwierdzić że nie ma podłączenia).
 */
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);            // SDA=21, SCL=22 - zmień jeśli inne piny
  Serial.println("\n=== I2C Scanner (SDA=21, SCL=22) ===");
}

void loop() {
  byte count = 0;
  Serial.println("Scanning...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println("Brak urzadzen I2C. Sprawdz polaczenia.");
  else Serial.printf("Total: %d device(s)\n", count);
  Serial.println();
  delay(3000);
}
