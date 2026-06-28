#include <Arduino.h>
#include <RP2040PIO_I2C.h>
#include <Adafruit_VL53L0X.h>

// --- PIO I2Cバスの設定 ---
// 1. SDA: 14, SCL: 15 (PIO0, SM 0)
RP2040PIO_I2C pioWire1(pio0, 14, 15, 0);

// --- VL53L0X インスタンス ---
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("PIO I2C Triple VL53L0X Test");

  // 1. 各PIO I2Cバスの初期化
  pioWire1.begin();

  // 2. センサーの初期化 (個別のPIOインスタンスを渡す)
  // すべてのアドレスはデフォルトの 0x29 です。バスが違うため衝突しません。
  Serial.println("Initializing Sensor #1 (SDA:14, SCL:15)...");
  if (!lox1.begin(0x29, false, &pioWire1))
  {
    Serial.println("Failed to boot VL53L0X #1");
  }

  Serial.println("All sensors initialized. Starting measurement...");
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measure1;

  // 各センサーから距離を取得
  lox1.rangingTest(&measure1, false);

  // 結果の表示
  Serial.print("Dist1: ");
  if (measure1.RangeStatus != 4)
    Serial.print(measure1.RangeMilliMeter);
  else
    Serial.print("Out of Range");

  Serial.println(" mm");

  delay(100);
}
