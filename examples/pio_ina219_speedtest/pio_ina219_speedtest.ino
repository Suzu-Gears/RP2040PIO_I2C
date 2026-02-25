#include <Arduino.h>
#include <RP2040PIO_I2C.h>

// I2Cピンの定義 (SCLはSDA+1である必要があります)
const uint PIN_SDA = 4;
const uint PIN_SCL = 5;

// 周波数測定用トグルピン (GP2)
const uint PIN_DEBUG_TOGGLE = 2;

// PIO I2C インスタンス
RP2040PIO_I2C pioWire(pio0, PIN_SDA, PIN_SCL);

const uint8_t INA219_ADDR = 0x40;

// テストするI2C周波数
const uint32_t I2C_FREQ = 2560000; // 2.56 MHz (36kHzで読み取り実行できた)

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  pinMode(PIN_DEBUG_TOGGLE, OUTPUT);
  digitalWrite(PIN_DEBUG_TOGGLE, LOW);

  Serial.println("RP2040 PIO I2C Speed Test with INA219");
  Serial.print("Target I2C Frequency: ");
  Serial.print(I2C_FREQ / 1000);
  Serial.println(" kHz");

  // PIO I2Cの初期化
  pioWire.begin();
  pioWire.setClock(I2C_FREQ);

  // INA219の初期化 (Bus Voltageを連続して読み込むための準備)
  // 設定はデフォルトのままでも動作しますが、念のためコンフィグ。
  pioWire.beginTransmission(INA219_ADDR);
  pioWire.write(0x00); // Config Register
  pioWire.write(0x39); // 12-bit, 8x avg (16V FSR, 320mV Range)
  pioWire.write(0x9F);
  pioWire.endTransmission();

  delay(100);
  Serial.println("Initial 10 readings for verification:");
  for (int i = 0; i < 10; i++)
  {
    pioWire.beginTransmission(INA219_ADDR);
    pioWire.write(0x02); // Bus Voltage
    pioWire.endTransmission(false);

    if (pioWire.requestFrom(INA219_ADDR, (size_t)2) == 2)
    {
      uint16_t raw = (uint16_t)pioWire.read() << 8 | pioWire.read();
      float voltage = (float)((raw >> 3) * 4);
      Serial.print("Reading ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(voltage);
      Serial.println(" mV");
    }
    else
    {
      Serial.println("Read Failed!");
    }
    delay(10);
  }

  Serial.println("Starting high-speed read loop...");
}

uint32_t iteration_count = 0;
uint32_t last_print_time = 0;

void loop()
{
  // --- デバッグピンをHIGHにして測定開始 ---
  gpio_put(PIN_DEBUG_TOGGLE, 1);

  uint16_t last_raw = 0;

  // 1. Bus Voltageレジスタ(0x02)を選択
  pioWire.beginTransmission(INA219_ADDR);
  pioWire.write(0x02);
  pioWire.endTransmission(false); // Restartビットを送る (STOPしない)

  // 2. 2バイト読み取る
  size_t bytesRead = pioWire.requestFrom(INA219_ADDR, (size_t)2);

  if (bytesRead == 2)
  {
    uint8_t msb = pioWire.read();
    uint8_t lsb = pioWire.read();
    last_raw = (uint16_t)msb << 8 | lsb;
  }

  // --- デバッグピンをLOWにして測定終了 ---
  gpio_put(PIN_DEBUG_TOGGLE, 0);

  // 1000回ごとに統計情報を出力
  iteration_count++;
  if (iteration_count >= 1000)
  {
    uint32_t now = micros();
    uint32_t delta_t = now - last_print_time;
    float freq_hz = 1000.0f / (delta_t / 1000000.0f);
    float busVoltage = (float)((last_raw >> 3) * 4);

    Serial.print("Bus: ");
    Serial.print(busVoltage);
    Serial.print(" mV, ");
    Serial.print("Rate: ");
    Serial.print(freq_hz);
    Serial.println(" Hz (readings/sec)");

    iteration_count = 0;
    last_print_time = now;
  }
}
