#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ============================================================================
// ハードウェアI2C (i2c0) のピン設定
// ============================================================================
// RP2040のハードウェアI2Cでは、コントローラ番号(i2c0 / i2c1)に対応するピンのリストから選ぶことで、
// 隣接していない離れたピン同士でも使用可能です。
// 
// i2c0 SDA候補ピン: GPIO 0, 4, 8, 12, 16, 20, 24, 28
// i2c0 SCL候補ピン: GPIO 1, 5, 9, 13, 17, 21, 25, 29

const uint SDA_PIN = 4;   // i2c0のSDA候補から選択
const uint SCL_PIN = 17;  // i2c0のSCL候補から選択 (非隣接ピン)

// センサインスタンス (標準Wireライブラリを使用)
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool sensorOK = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    delay(10);

  Serial.println("=== Hardware I2C (Wire) VL53L0X Test ===");
  Serial.print("Configured Hardware Pins -> SDA: ");
  Serial.print(SDA_PIN);
  Serial.print(", SCL: ");
  Serial.println(SCL_PIN);

  // 1. ハードウェアI2Cのピン番号を設定してから初期化
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  // 2. センサーの初期化 (標準のWireオブジェクトを渡す)
  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("ERROR: Failed to boot VL53L0X on Hardware I2C!");
    Serial.println("Please check hardware wiring and pull-up resistors.");
    sensorOK = false;
  } else {
    Serial.println("SUCCESS: VL53L0X booted successfully on Hardware I2C!");
    sensorOK = true;
  }
}

void loop() {
  if (sensorOK) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    Serial.print("Distance: ");
    if (measure.RangeStatus != 4) {
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      Serial.println("Out of Range");
    }
  } else {
    Serial.println("Waiting for sensor... (Hardware I2C failure)");
    delay(2000);
    return;
  }

  delay(100);
}
