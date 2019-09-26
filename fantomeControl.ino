#include <PinChangeInterrupt.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
#include <MadgwickAHRS.h>
#include <Servo.h>
#include <MsTimer2.h>

#define MPU6050_PWR_MGMT_1   0x6B
#define MPU_ADDRESS  0x68
#define OUTPUT_READABLE_ACCELGYRO
Madgwick MadgwickFilter;
MPU6050 accelgyro;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long starttime;
const byte channel_pin[] = {2, 4, 7, 8, 11, 12, 13};
volatile unsigned long rising_start[] = {0, 0, 0, 0, 0, 0, 0};
volatile long channel_length[] = {0, 0, 0, 0, 0, 0, 0};
float pwmout_1,pwmout_2,pwmout_3,pwmout_4,pwmout_5;
float out1,out2,out3,out4,out5;
float roll, pitch, yaw;
float ff_ail,ff_ele, ff_thr, ff_rud;



void setup() {
  ff_ail=0.5;
  ff_ele=0.5;
  ff_thr=0.0;
  ff_rud=0.5;
  pwmout_1 = 0.5;
  pwmout_2 = 0.5;
  pwmout_3 = 0.0;
  pwmout_4 = 0.5;
  pwmout_5 = 0.5;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  out1 = 1500;
  out2= 1500;
  out3 = 1000;
  out4 = 1500;
  out5 = 1500;
  Serial.begin(57600);

  pinMode(channel_pin[0], INPUT);
  pinMode(channel_pin[1], INPUT);
  pinMode(channel_pin[2], INPUT);
  pinMode(channel_pin[3], INPUT);
  pinMode(channel_pin[4], INPUT);
  pinMode(channel_pin[5], INPUT);
  pinMode(channel_pin[6], INPUT);
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);
  
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[3]), onRising3, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[4]), onRising4, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[5]), onRising5, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[6]), onRising6, CHANGE);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  //accelgyro.testConnection();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  MadgwickFilter.begin(1.0/20.0*1000.0); //Hz
  MsTimer2::set(40, timerFire);
  MsTimer2::start();
}

void processPin(byte pin) {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));
  if(trigger == RISING) {
    rising_start[pin] = micros();
  } else if(trigger == FALLING) {
    volatile long  lenbuff =  micros() - rising_start[pin];
    if(int(lenbuff)>800 && int(lenbuff<2200)){
      channel_length[pin] = lenbuff;
    }
  }
}

void onRising0(void) {
  processPin(0);
}
void onRising1(void) {
  processPin(1);
}
void onRising2(void) {
  processPin(2);
}
void onRising3(void) {
  processPin(3);
}
void onRising4(void) {
  processPin(4);
}
void onRising5(void) {
  processPin(5);
}
void onRising6(void) {
  processPin(6);
}

void timerFire(void){
  servo1.writeMicroseconds(out1);
  servo2.writeMicroseconds(out2);
  servo3.writeMicroseconds(out3);
  servo4.writeMicroseconds(out4);
  servo5.writeMicroseconds(out5);
}

void updateAttitude(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 加速度値を分解能で割って加速度(G)に変換する
  float acc_x = ax / 16384.0;  //FS_SEL_0 16,384 LSB / g
  float acc_y = ay / 16384.0;
  float acc_z = az / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  float gyro_x = gx / 131.0;  // (度/s)
  float gyro_y = gy / 131.0;
  float gyro_z = gz / 131.0;
  //Madgwickフィルターを用いて、PRY（pitch, roll, yaw）を計算
  MadgwickFilter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
}

void loop() {
  starttime = micros();
  updateAttitude();
  //PRYの計算結果を取得する
  float LP_gyro = 0.9;
  roll  = LP_gyro*MadgwickFilter.getPitchRadians()+(1-LP_gyro)*roll+3.0/57.0;
  pitch = LP_gyro*MadgwickFilter.getRollRadians()+(1-LP_gyro)*pitch;
  yaw   = LP_gyro*MadgwickFilter.getYawRadians()+(1-LP_gyro)*yaw;

  float LP_ff = 0.8; 
  ff_ail = LP_ff*float(map(int(channel_length[0]), 1084, 1908, 0, 1000))/1000.0+(1.0-LP_ff)*ff_ail;
  ff_ele = LP_ff*float(map(int(channel_length[1]), 1096, 1932, 0, 1000))/1000.0+(1.0-LP_ff)*ff_ele;
  ff_thr = LP_ff*float(map(int(channel_length[2]), 1084, 1920, 0, 1000))/1000.0+(1.0-LP_ff)*ff_thr;
  ff_rud = LP_ff*(1.0-float(map(int(channel_length[3]), 1068, 1932, 0, 1000))/1000.0)+(1.0-LP_ff)*ff_rud;
  float autoswitch =1.0-float(map(int(channel_length[4]), 1068, 1932, 0, 1000))/1000.0;
  float missionswitch = float(map(int(channel_length[5]), 1068, 1932, 0, 1000))/1000.0;
  float bombswitch = float(map(int(channel_length[6]), 1068, 1932, 0, 1000))/1000.0;
  
  //スイッチによる切り替え
  float Kde = 0.0;
  float Kda = 0.0;
  float Kffa = 1.0;
  if(autoswitch>0.3){
    Kde = 0.7;
    Kda = 0.5;
    Kffa = 1.5;
  }

  //自動制御コマンド
  float de = -Kde*(pitch-10.0/57.0);
  float da = -Kda*roll;
  float dT = ff_thr;
  float dr = ff_rud;

  float LP_out = 0.5;
  pwmout_1 = LP_out*(de+da+ff_ele+Kffa*(ff_ail-0.5))+(1.0-LP_out)*pwmout_1;
  pwmout_2 = LP_out*(-de+da+1-(ff_ele-Kffa*(ff_ail-0.5)))+(1.0-LP_out)*pwmout_2;
  pwmout_3 = dT;
  pwmout_4 = dr-0.7*(da+(ff_ail-0.5));
  pwmout_5 = bombswitch;
  
  int pwmMax = 1800;
  int pwmMin = 1200;
  out1 = map(int(pwmout_1*50.0), 0, 50, pwmMin, pwmMax);
  if(out1<pwmMin){out1 = pwmMin;};
  if(out1>pwmMax){out1 = pwmMax;};
  out2 = map(int(pwmout_2*50.0), 0, 50, pwmMin, pwmMax);
  if(out2<pwmMin){out2 = pwmMin;};
  if(out2>pwmMax){out2 = pwmMax;};
  out3 = map(int(pwmout_3*200.0), 0, 200, 1100, 2200);
  if(out3<1100){out3 = 1100;};
  if(out3>2200){out3 = 2200;};
  out4 = map(int(pwmout_4*50.0), 0, 50, 1000, 2000);
  if(out4<1000){out4 = 1000;};
  if(out4>2000){out4 = 2000;};
  out5 = map(int(pwmout_5*50.0), 0, 50, 1100, 1900);
  if(out5<1100){out5 = 1100;};
  if(out5>1900){out5 = 1900;};

  Serial.print(ff_ele);
  Serial.print(" | ");
  Serial.print(ff_ail);
  Serial.print(" | ");
  Serial.print(ff_thr);
  Serial.print(" | ");
  Serial.print(ff_rud);
  Serial.print(" | ");
  Serial.print(autoswitch);
  Serial.print(" | ");
  Serial.print(roll*57.0);  Serial.print(",");
  Serial.print(pitch*57.0);  Serial.print(",");
  Serial.print(yaw*57.0);  Serial.print(",");
  unsigned long endtime = micros();
  Serial.print(endtime-starttime);  Serial.println("");
  delay(int(20-(endtime-starttime)/1000));
}
