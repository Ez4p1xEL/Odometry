#include <Wire.h>
#include <MPU6050_tockn.h>
#include <cmath>

MPU6050 mpu6050(Wire);

// LEFT PINS
const int ENC1_A = 18;
const int ENC1_B = 19;

// RIGHT PINS
const int ENC2_A = 25;
const int ENC2_B = 26;

// 脈衝計數變數 (volatile 確保中斷讀寫安全)
volatile long count1 = 0;
volatile long count2 = 0;

// 記錄上一次 A/B 腳位狀態的變數 (用於 4 倍頻狀態機)
volatile uint8_t enc1_last = 0;
volatile uint8_t enc2_last = 0;

// 4 倍頻正交解碼查詢表 (Gray Code 狀態轉移表)
// 1 代表正轉，-1 代表反轉，0 代表無效/抖動狀態
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

const float pi = std::acos(-1.0);
const float diameter = 60; // in mm
const float TICKS_PER_MILLIMETER = 44.8;
const float LENGTH_PER_CIRCLE = diameter*pi;
const float TICKS_PER_CIRCLE = LENGTH_PER_CIRCLE*TICKS_PER_MILLIMETER; // in ticks
const float L = 85.0; // 兩輪中心之間距離
float gyro_bias = 0.0f;

// 先獲取兩個輪胎的總行走距離，再得出兩數的平均數，再以cos45去計算出前進距離

/*
*/

// 一號中斷服務函數
void IRAM_ATTR isr_enc1() {
  uint8_t a = digitalRead(ENC1_A);
  uint8_t b = digitalRead(ENC1_B);
  uint8_t current = (a << 1) | b;
  uint8_t index = (enc1_last << 2) | current;
  count1 += QUAD_TABLE[index];
  enc1_last = current;
}

// 二號中斷服務函數
void IRAM_ATTR isr_enc2() {
  uint8_t a = digitalRead(ENC2_A);
  uint8_t b = digitalRead(ENC2_B);
  uint8_t current = (a << 1) | b;
  uint8_t index = (enc2_last << 2) | current;
  count2 += QUAD_TABLE[index];
  enc2_last = current;
}

/*
  計算行走距離 (mm)
*/
float calculateLength(float startingTick, float endingTick) {
  float ticks = endingTick - startingTick;
  return ticks/TICKS_PER_MILLIMETER;
}

float calculateForwardDistance(float length) {
  float adjacent = length; // 鄰邊, 實際為兩個輪胎的平均數
  float degrees = 45.0; // 直角切半，固定

  double radians = degrees * (pi / 180.0);

  float hypotenuse = adjacent / std::cos(radians); // 斜邊
  return hypotenuse;
}

/*
陀螺儀 漂移修正
*/

float previousLeft = 0.0f;
float previousRight = 0.0f;
float encoderYaw = 0.0f;

float fusedYaw = 0.0f;

void updateYaw(float dt, float gz) {
  float left = calculateLength(0, count1 * -1);
  float right = calculateLength(0, count2);

  float deltaLeft = left - previousLeft;
  float deltaRight = right - previousRight;

  previousLeft = left;
  previousRight = right;

  float encoderDelta =
      (deltaRight - deltaLeft) / L * 180.0f / pi;

  encoderYaw += encoderDelta;

  float gyroDelta = gz * dt;

  fusedYaw += gyroDelta;

  const float alpha = 0.98f;
  fusedYaw += (1.0f - alpha) * (encoderYaw - fusedYaw);
}

void measureGyroBias() {
  const int samples = 500;
  float sum = 0.0f;

  Serial.println("測量陀螺儀殘餘 Bias，請保持靜止...");

  for (int i = 0; i < samples; i++) {
    mpu6050.update();
    sum += mpu6050.getGyroZ();
    delay(5);
  }

  gyro_bias = sum / samples;

  Serial.print("gyro_bias = ");
  Serial.println(gyro_bias, 6);
}

unsigned long timer = 0;

/*
*/

float manual_yaw = 0.0;
unsigned long prev_time = 0;

void setup() {
  Serial.begin(115200);

  noInterrupts();
  count1 = 0;
  count2 = 0;
  interrupts();
  previousLeft = 0.0f;
  previousRight = 0.0f;
  encoderYaw = 0.0f;
  fusedYaw = 0.0f;
  manual_yaw = 0.0f;

  // 設置引腳為輸入並啟用內建上拉電阻
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  // 綁定中斷：當 A 相電平改變時觸發計算
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr_enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isr_enc1, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr_enc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isr_enc2, CHANGE);

  Serial.println("=================================");
  Serial.println("雙編碼器 4 倍頻模式啟動！請手動旋轉轉軸");
  Serial.println("=================================");

  while (!Serial) delay(10);

  // 2. 初始化 I2C
  Wire.begin(21, 22);
  mpu6050.begin();

  // 3. 自動靜止校準（開機請保持小車不動大約 2~3 秒）
  Serial.println("正在校準陀螺儀 Offset，請保持小車完全靜止...");
  mpu6050.calcGyroOffsets(true); // true 會在串口印出校準進度
  mpu6050.calcGyroOffsets(true);

  measureGyroBias();
  manual_yaw = 0.0f;
  fusedYaw = 0.0f;
  prev_time = micros();
  Serial.println("校準完成！系統進入主迴圈。");
}

void loop() {
  // 核心：每次 loop 必須調用 update() 刷新濾波與積分數據
  mpu6050.update();

  unsigned long current_time = micros();
  float dt = (current_time - prev_time) / 1000000.0;
  prev_time = current_time;

  float gz = mpu6050.getGyroZ() - gyro_bias;

  if (fabs(gz) < 0.05f) {
    gz = 0.0f;
  }

  manual_yaw += gz * dt;
  
  updateYaw(dt, gz);

  // 每 100ms 印出一次數據
  if (millis() - timer > 100) {
    timer = millis();

    Serial.print("L: ");
    Serial.print(calculateLength(0, count1*-1)/10, 2);
    Serial.print(" (");
    Serial.print(count1*-1);
    Serial.print(") ");
    Serial.print("cm | R: ");
    Serial.print(calculateLength(0, count2)/10, 2);
    Serial.print(" (");
    Serial.print(count2);
    Serial.print(") ");
    Serial.print("cm | 車頭 Yaw: ");
    Serial.print(manual_yaw, 2);
    Serial.println(" 度");
    
    float mean = (calculateLength(0, count1*-1) + calculateLength(0, count2))/2.0;
    float forwardDist = calculateForwardDistance(mean);
    Serial.print("前進距離: ");
    Serial.print(forwardDist/10, 2);
    Serial.println(" cm");

    Serial.print("編碼器 Yaw: ");
    Serial.print(encoderYaw, 2);
    Serial.println(" 度");

    Serial.print("融合 Yaw: ");
    Serial.print(fusedYaw, 2);
    Serial.println(" 度");
  }
}
  