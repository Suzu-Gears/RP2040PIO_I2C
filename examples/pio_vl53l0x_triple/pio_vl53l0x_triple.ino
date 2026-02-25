#include <Arduino.h>
#include <RP2040PIO_I2C.h>
#include <Adafruit_VL53L0X.h>

// --- PIO I2Cバスの設定 ---
// 1. SDA: 14, SCL: 15 (PIO0, SM 0)
RP2040PIO_I2C pioWire1(pio0, 14, 15, 0);

// 2. SDA: 10, SCL: 11 (PIO0, SM 1)
RP2040PIO_I2C pioWire2(pio0, 10, 11, 1);

// 3. SDA: 18, SCL: 19 (PIO0, SM 2)
RP2040PIO_I2C pioWire3(pio0, 18, 19, 2);

// --- VL53L0X インスタンス ---
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("PIO I2C Triple VL53L0X Test");

  // 1. 各PIO I2Cバスの初期化
  pioWire1.begin();
  pioWire2.begin();
  pioWire3.begin();

  // 2. センサーの初期化 (個別のPIOインスタンスを渡す)
  // すべてのアドレスはデフォルトの 0x29 です。バスが違うため衝突しません。
  Serial.println("Initializing Sensor #1 (SDA:14, SCL:15)...");
  if (!lox1.begin(0x29, false, &pioWire1))
  {
    Serial.println("Failed to boot VL53L0X #1");
  }

  Serial.println("Initializing Sensor #2 (SDA:10, SCL:11)...");
  if (!lox2.begin(0x29, false, &pioWire2))
  {
    Serial.println("Failed to boot VL53L0X #2");
  }

  Serial.println("Initializing Sensor #3 (SDA:18, SCL:19)...");
  if (!lox3.begin(0x29, false, &pioWire3))
  {
    Serial.println("Failed to boot VL53L0X #3");
  }

  Serial.println("All sensors initialized. Starting measurement...");
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;

  // 各センサーから距離を取得
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  // 結果の表示
  Serial.print("Dist1: ");
  if (measure1.RangeStatus != 4)
    Serial.print(measure1.RangeMilliMeter);
  else
    Serial.print("Out of Range");

  Serial.print(" mm | Dist2: ");
  if (measure2.RangeStatus != 4)
    Serial.print(measure2.RangeMilliMeter);
  else
    Serial.print("Out of Range");

  Serial.print(" mm | Dist3: ");
  if (measure3.RangeStatus != 4)
    Serial.print(measure3.RangeMilliMeter);
  else
    Serial.print("Out of Range");

  Serial.println(" mm");

  delay(100);
}
