#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>

//ロータの制御定数
volatile float rk1 = 939, rk2 = 39, rk3 = 22, rk4 = 6;

//車輪の制御定数
volatile float wk1 = 1800, wk2 = 160, wk3 = 32, wk4 = 8;    //30mmホイール マクロステップ1/4

// ステッピングモータが脱調しない最大のスピード
#define R_LIMIT 150
#define W_LIMIT 560

#define R_DIR   4
#define R_STEP  5
#define R_EN    6
#define W_DIR   7
#define W_STEP  8
#define W_EN    9

#define PULSE_PERIOD   20     // パルス生成のためのTimer1のタイマ割り込み周期 マイクロ秒単位
#define CONTROL_PERIOD 4 // 制御ループの周期 ミリ秒単位

//倒立振子の制御で使用する変数
volatile long lastUptime, startTime, currentTime;
volatile float dt;

//ローターの制御で使用する変数
volatile float caribGyroY;
volatile int16_t rawGyroY;
volatile float gyroY;
volatile float r_deg = 0, r_dps = 0, r_rot = 0;
volatile bool r_out = false;
volatile int r_count = 0;
volatile int r_speed = 0;
volatile float r_lpf = 0, r_lpfA = 0.9999;

//車輪の制御で使用する変数
volatile float caribGyroX;
volatile int16_t rawGyroX;
volatile float gyroX;
volatile float w_deg = 0, w_dps = 0, w_rot = 0;
volatile bool w_out = false;
volatile int w_count = 0;
volatile int w_speed = 0;
volatile float w_lpf = 0, w_lpfA = 0.9999;


// 加速度・ジャイロセンサーの制御定数
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75

// センサーへのコマンド送信
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// センサーからのデータ読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, false);
  //readで読み取れるバイト数がなければLED13を消灯
  while (! Wire.available()) {
    digitalWrite(13, LOW);
  }
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}

void setup() {
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("*******************RESTARTED********************");
  Serial.println("*******************unicycle*********************");

  // モータードライバ制御用ピンの初期化
  pinMode(R_DIR, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(W_DIR, OUTPUT);
  pinMode(W_STEP, OUTPUT);
  pinMode(W_EN, OUTPUT);

  //状態をLED 13にて表示
  pinMode(13, OUTPUT);

  // センサーの初期化
  Wire.begin();
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("WHO_AM_I error.");
    while (true) ;
  }
  else {
    Serial.println("WHO_AM_I OK.");
  }
  // see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: 0x00⇒±250dps 131LSB、0x08⇒±500dps 65.5LSB、0x10⇒±1000dps 32.8LSB、0x18⇒±2000dps 16.4LSB
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: 0x00⇒±2g 16384LSB/g、0x01⇒±4g 8192LSB/g、0x02⇒±8g 4096LSB/g、0x03⇒±16g 2048LSB/g、
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  Serial.println("MPU6050 Setup OK.");
  delay(2000);

  //ジャイロのゼロ点調整のために静止時の出力を1000回計測して平均を算出
  caribGyroX = 0;
  caribGyroY = 0;
  for (int i = 0; i < 1000  ; i++)  {
    rawGyroX = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
    rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    caribGyroX += (float) rawGyroX;
    caribGyroY += (float) rawGyroY;
  }
  caribGyroX /= 1000;
  caribGyroY /= 1000;
  Serial.println("Carib OK.");

  // dt計測用
  lastUptime = micros();

  //　倒立時間計測用
  startTime = micros();

  digitalWrite(R_EN, LOW);
  digitalWrite(W_EN, LOW);

  Timer1.initialize(PULSE_PERIOD); //パルス生成のためのタイマ割込み 引数はマイクロ秒単位
  Timer1.attachInterrupt(pulse);
  MsTimer2::set(CONTROL_PERIOD, controlloop); //制御のためのタイマ割込み 引数はミリ秒単位
  MsTimer2::start();

  //準備が出来たらLDE 13を点灯
  digitalWrite(13, HIGH);
  Serial.println("******************** GO !! *********************");
}

void pulse() {
  w_count += w_speed;
  if (w_count > 5000) {
    PORTB ^= B00000001;
    w_count -= 5000;
  }
  else if (w_count < -5000) {
    PORTB ^= B00000001;
    w_count += 5000;
  }
  r_count += r_speed;
  if (r_count > 5000) {
    PORTD ^= B00100000;
    r_count -= 5000;
  }
  else if (r_count < -5000) {
    PORTD ^= B00100000;
    r_count += 5000;
  }
}

void controlloop () {
  // 割り込みハンドラの中でI2C通信(割り込み処理を使用）を許可する
  interrupts();

  // dt計測
  currentTime = micros();
  dt = (currentTime - lastUptime) * 0.000001;
  lastUptime = currentTime;

  // 角速度を取得
  rawGyroY =  (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
  gyroY =  (float) rawGyroY - caribGyroY;
  r_dps = gyroY / 131;
  rawGyroX =  (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
  gyroX =  (float) rawGyroX - caribGyroX;
  w_dps = gyroX / 131;

  // 角速度を積算して角度を求める
  r_deg +=  r_dps * dt;
  w_deg +=  w_dps * dt;

  // 速度を積算して回転角を求める
  r_rot += r_speed * dt;
  w_rot += w_speed * dt;

  //ローパスフィルタ
  r_lpf *=  r_lpfA;
  r_lpf +=  (1 - r_lpfA) * r_deg;
  w_lpf *=  w_lpfA;
  w_lpf +=  (1 - w_lpfA) * w_deg;

  // 制御量の計算
  r_speed += (rk1 * (r_deg - r_lpf) + rk2 * r_dps + rk3 * r_rot + rk4 * r_speed) * dt;
  w_speed += (wk1 * (w_deg - w_lpf) + wk2 * w_dps + wk3 * w_rot + wk4 * w_speed) * dt;

  // ステッピングモータの最大速度を制限
  r_speed = constrain(r_speed, 0 - R_LIMIT, R_LIMIT);
  w_speed = constrain(w_speed, 0 - W_LIMIT, W_LIMIT);

  // 回転方向を指定
  if (r_speed > 0) {
    PORTD &= ~B00010000;
  }
  else {
    PORTD |= B00010000;
  }
  if (w_speed > 0) {
    PORTD &= ~B10000000;
  }
  else {
    PORTD |= B10000000;
  }

  // 倒れたらモーター停止
  if (20 < abs(r_deg - r_lpf) || 20 < abs(w_deg - w_lpf)) {
    r_speed = 0;
    w_speed = 0;
    digitalWrite(R_EN, HIGH);
    digitalWrite(W_EN, HIGH);
    Serial.println("*********************STOP***********************");

    while (1) {
      // LED13を点滅
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13, HIGH);
      delay(500);
    }
  }
}

void loop() {
}
