#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>

MPU6050 mpu6050(Wire);

// 參數依序為：
// 1. e_mea: 測量雜訊 R (例如陀螺儀雜訊，約 0.05 ~ 0.2)
// 2. e_est: 估計誤差 P 的初始值 (一般給跟 e_mea 差不多)
// 3. q:     過程雜訊 Q (隨時間變化的快慢，約 0.001 ~ 0.05)
SimpleKalmanFilter gyroKalman(0.1, 0.1, 0.01);

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

const float pi = 3.141519;
const float diameter = 60; // in mm
const float TICKS_PER_MILLIMETER = 44.8;
const float LENGTH_PER_CIRCLE = diameter*pi;
const float TICKS_PER_CIRCLE = LENGTH_PER_CIRCLE*TICKS_PER_MILLIMETER; // in ticks
const float L = 4; // 兩輪中心之間距離

/* Legacy:
 40厘米，走8500-115=8385
 每1厘米，走8385/40=210
 輪胎直徑為6，輪胎一圈所走ticks = 210 * 6Π = 3958ticks/圈
 一圈走 6Π = 18.8cm */

// 17919 = 448

const float WHEEL_DIAMETER_MM = diameter; // in mm
const int CPR = 8192;
const float MM_PER_TICK = (3.141519 * WHEEL_DIAMETER_MM) / CPR;
const float COS45 = 0.70710678; // cos(45°)
float initial_offset = 0.0f;
const int SAMPLES = 200;
const float dynamic_offset = 0.0f;
float current_k = 0.02f;
float updated_drift = 0.0f;
float gyro_bias = 0.0f;

/*
*/

float predictAngle(float ticksL, float ticksR, float previousAngle) {
  float left = (ticksL/CPR) * pi;
  float right = (ticksR/CPR) * pi;

  // 計算滾動距離
  float distanceLeft = left * diameter;
  float distanceRight = right * diameter;

  float delta = (distanceRight - distanceLeft)/L;
  return previousAngle + delta;
  
}

float getResidual(float delta_s_l, float delta_s_r ) {
  float omega_imu = mpu6050.getGyroZ() * (pi / 180.0f);
  float omega_enc = (delta_s_r - delta_s_l) / (L*0.02);
  return omega_imu - (omega_enc + dynamic_offset);
}

void fixOffset(float leftDistance, float rightDistance, float residual, float predictedAngle) {
  float fixedAngle = predictedAngle + (current_k * residual * 0.02);
  updated_drift = gyro_bias + ((1-current_k) * residual);
}

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

float calculateLength(float startingTick, float endingTick) {
  float ticks = endingTick - startingTick;
  return ticks/TICKS_PER_MILLIMETER;
}

float calculateForwardDistance(long deltaTick1, long deltaTick2) {
  float d1 = deltaTick1 * MM_PER_TICK;
  float d2 = deltaTick2 * MM_PER_TICK;

  float forwardDist = (d1+d2) / (2.0 * sqrt(2.0));
  return forwardDist;
}

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);

  //Serial.print("TEST");

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
  Serial.println("校準完成！系統進入主迴圈。");
}

float manual_yaw = 0.0;
unsigned long prev_time = 0;

void loop() {
  // 核心：每次 loop 必須調用 update() 刷新濾波與積分數據
  mpu6050.update();

  unsigned long current_time = micros();
  float dt = (current_time - prev_time) / 1000000.0;
  prev_time = current_time;

  float gz = mpu6050.getGyroZ();

  gz = abs(gz)<0.2 ? 0.0 : gz;

  manual_yaw += gz * dt;

  // 假設這是 MPU6050 讀到的角速度 (rad/s)
    float raw_gyro_z = 0.52; 

    // 一行代碼完成卡爾曼更新！
    // 庫內部會自動更新 P，並計算當前的 K，然後輸出濾波後的估計值
    float clean_gyro_z = gyroKalman.updateEstimate(raw_gyro_z);

    // ★ 直接獲取庫當前算出來的 K 值 (卡爾曼增益，介於 0 與 1 之間)
    current_k = gyroKalman.getKalmanGain();
    
    float res = getResidual(calculateLength(0,count1), calculateLength(0,count2));
    float pA = predictAngle(count1,count2,manual_yaw);
    gyro_bias = gyro_bias + (current_k * res);
    fixOffset(calculateLength(0,count1), calculateLength(0,count2), res, pA);

  // 每 100ms 印出一次數據
  if (millis() - timer > 100) {
    timer = millis();

    // 取得 Z 軸航向角（車頭偏航角 Yaw）
    float yaw = mpu6050.getAngleZ();

    Serial.print("L: ");
    Serial.print(calculateLength(0, count1)/10, 2);
    Serial.print("cm | R: ");
    Serial.print(calculateLength(0, count2)/10, 2);
    Serial.print("cm | 車頭 Yaw: ");
    Serial.print(manual_yaw, 2);
    Serial.println(" 度");

    Serial.println(count1);
    Serial.print("預測角度: ");
    Serial.println(pA);
    Serial.print("修正角度：");
    Serial.println(yaw - updated_drift);
  }

  //delay(20); // 约 50Hz 采样频率

  delay(100);
  // Serial.println("-----");
  // Serial.print("[LEFT] Length starts from 0 tick: ");
  // Serial.print(calculateLength(0, count1),6);
  // Serial.println(" cm.");

  // Serial.print("[RIGHT] Length starts from 0 tick: ");
  // Serial.print(calculateLength(0, count2),6);
  // Serial.println(" cm.");
  // Serial.println("-----");
}

// const float len = 

// float getMiddleLine(float left, float right) {
//   float minLength = left<right ? left : right;

  
// }