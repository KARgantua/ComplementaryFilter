#include "MPU9250.h"

MPU9250 IMU(Wire,0x68);
int status;

//이전 상태
unsigned long t_prev;
float last_AngleX;
float last_AngleY;
float last_AngleZ;
float last_GyroX_Angle;
float last_GyroY_Angle;
float last_GyroZ_Angle;

void SetLastReadAngleDate(unsigned long t, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  t_prev = t;
  last_AngleX = x;
  last_AngleY = y;
  last_AngleZ = z;
  last_GyroX_Angle = x_gyro;
  last_GyroY_Angle = y_gyro;
  last_GyroZ_Angle = z_gyro;
}

//초기 상태
float base_AccelX;
float base_AccelY;
float base_AccelZ;
float base_GyroX;
float base_GyroY;
float base_GyroZ;

//현재 상태
float AcX;
float AcY;
float AcZ;
float GyX;
float GyY;
float GyZ;

void ReadAccelGyro() {
  IMU.readSensor();
  AcX = IMU.getAccelX_mss();
  AcY = IMU.getAccelY_mss();
  AcZ = IMU.getAccelZ_mss();
  GyX = IMU.getGyroX_rads();
  GyY = IMU.getGyroY_rads();
  GyZ = IMU.getGyroZ_rads();
}

//평균값으로 가속도와 자이로 센서의 초기 값을 조정하는 함수
void CalibSensors()
{
  int readNum = 10;
  float AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;
  
  for(int i = 0; i <readNum; i++) {
    ReadAccelGyro();
    AccelX += AcX;
    AccelY += AcY;
    AccelZ += AcZ;
    GyroX += GyX;
    GyroY += GyY;
    GyroZ += GyZ;
    delay(100);
  }

  base_AccelX = AccelX / readNum;
  base_AccelY = AccelY / readNum;
  base_AccelZ = AccelZ / readNum;
  base_GyroX = GyroX / readNum;
  base_GyroY = GyroY / readNum;
  base_GyroZ = GyroZ / readNum;
}

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  CalibSensors();
  SetLastReadAngleDate(millis(), 0, 0, 0, 0, 0, 0);
}

void loop() {
  unsigned long t_now = millis();
  ReadAccelGyro();
  
  //=====================================================================
  //===============================가속도=================================
  //=====================================================================
  //가속도 센서 값
  float AccelX = AcX;
  float AccelY = AcY;
  float AccelZ = AcZ;

  //가속도로부터 각 구하기
  float RADIANS_TO_DEGREES = 180/3.14159;
  float Accel_Angle_Y = atan(-1 * AccelX / sqrt(pow(AccelY,2) + pow(AccelZ,2))) * RADIANS_TO_DEGREES;
  float Accel_Angle_X = atan(AccelY / sqrt(pow(AccelX,2) + pow(AccelZ,2))) * RADIANS_TO_DEGREES;
  float Accel_Angle_Z = 0;
  //=====================================================================
  //===============================자이로=================================
  //=====================================================================
  //자이로 값을 degree/sec으로 변환
  //FS_SEL 각속도 1도/sec = 131
  float FS_SEL = 131;
  float GyroX = (GyX - base_GyroX) / FS_SEL;
  float GyroY = (GyY - base_GyroY) / FS_SEL;
  float GyroZ = (GyZ - base_GyroZ) / FS_SEL;

  //필터된 자이로 각 계산
  float dt = (t_now - t_prev) / 1000.0;
  float Gyro_Angle_X = (GyroX * dt) + last_AngleX;
  float Gyro_Angle_Y = (GyroY * dt) + last_AngleY;
  float Gyro_Angle_Z = (GyroZ * dt) + last_AngleZ;

  //드리프트된 자이로 각 계산
  float Unfiltered_Gyro_Angle_X = (GyroX * dt) + last_GyroX_Angle;
  float Unfiltered_Gyro_Angle_Y = (GyroY * dt) + last_GyroY_Angle;
  float Unfiltered_Gyro_Angle_Z = (GyroZ * dt) + last_GyroZ_Angle;

  //상보필터 적용
  float alpha = 0.96;
  float AngleX = (alpha * Gyro_Angle_X) + ((1.0 - alpha) * Accel_Angle_X);
  float AngleY = (alpha * Gyro_Angle_Y) + ((1.0 - alpha) * Accel_Angle_Y);
  float AngleZ = Gyro_Angle_Z;

  SetLastReadAngleDate(t_now, AngleX, AngleY, AngleZ, Unfiltered_Gyro_Angle_X, Unfiltered_Gyro_Angle_Y, Unfiltered_Gyro_Angle_Z);

  Serial.print("Angle:");
  Serial.print("\t");
  Serial.print(AngleX, 6);
  Serial.print("\t");
  Serial.print(AngleY, 6);
  Serial.print("\t");
  Serial.print(AngleZ, 6);
  Serial.println("");

  delay(5);
}
