#include <Arduino.h>
#include <RP2040PIO_I2C.h>

// Define pins for PIO I2C
// SCL must be SDA + 1
const uint PIN_SDA = 4;
const uint PIN_SCL = 5;

// Create PIO I2C instance
RP2040PIO_I2C pioWire(pio0, PIN_SDA, PIN_SCL);

const uint8_t INA219_ADDR = 0x40;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("RP2040 PIO I2C Wire-compatible Test");

  // Initialize PIO I2C
  pioWire.begin();

  // Example: Setting clock frequency
  pioWire.setClock(400000);

  // --- INA219 Configuration using Wire API ---
  uint16_t config_value = 0x399F;

  pioWire.beginTransmission(INA219_ADDR);
  pioWire.write(0x00); // Config Register
  pioWire.write((uint8_t)(config_value >> 8));
  pioWire.write((uint8_t)(config_value & 0xFF));
  uint8_t error = pioWire.endTransmission();

  if (error == 0)
  {
    Serial.println("INA219 Config Written Successfully");
  }
  else
  {
    Serial.print("Error writing to INA219: ");
    Serial.println(error);
  }

  delay(100);
}

void loop()
{
  uint16_t rawBusVoltage = 0;

  // --- Read Bus Voltage (0x02) ---
  pioWire.beginTransmission(INA219_ADDR);
  pioWire.write(0x02);
  pioWire.endTransmission(false); // Restart

  size_t bytesRead = pioWire.requestFrom(INA219_ADDR, (size_t)2);
  if (bytesRead == 2)
  {
    uint8_t msb = pioWire.read();
    uint8_t lsb = pioWire.read();
    rawBusVoltage = (uint16_t)(msb << 8 | lsb);
    float busVoltage = (float)((rawBusVoltage >> 3) * 4);

    Serial.print("Bus Voltage: ");
    Serial.print(busVoltage);
    Serial.println(" mV");
  }
  else
  {
    Serial.println("Failed to read Bus Voltage");
  }

  delay(1000);
}
