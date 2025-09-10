//OLED
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//MPU6050 with Kalman Filter
#include <Wire.h>
#include <Kalman.h>
#include <MPU6050.h>
#define RESTRICT_PITCH  // Comment out to restrict roll to ±90deg instead

//bluetooth
#include "BluetoothSerial.h"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_CLK 18
#define OLED_MOSI 23
#define OLED_RESET 16
#define OLED_DC 17
#define OLED_CS 5

volatile signed int temp, counter = 0;  //ตัวแปล Encoder function
const float pi = 3.141592;              //pi
volatile bool status = false;           //กำหนดสถานะเริ่มต้น statup defalse = false
volatile bool reset = false;            //กำหนดสถานะเริ่มต้น reset defalse = false
volatile bool button_status = true;     //status setup function defalse = true
volatile bool button_reset = true;      //reset setup function defalse = true

//ตัวแปลส่งค่า sd card
volatile int SD_No = 1;
volatile float SD_Distance;
volatile float SD_Angle;
volatile float Old_Distance = -9999.00;  //ส่งออกค่าแบบไม่ซ้ำ

//ตัวแปร mpu6050
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
uint32_t timer;
double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter

//function หาค่าเฉลี่ย
const int numReadings = 50;  // จำนวนการอ่านค่าเพื่อหาค่าเฉลี่ย
float readings[numReadings];  // อาร์เรย์สำหรับเก็บค่าที่อ่านได้
int readIndex = 0;            // ดัชนีสำหรับการอ่านค่า
float total = 0;              // ผลรวมของค่าที่อ่านได้
float average = 0;            // ค่าเฉลี่ย
float kalAngleY_average;

// สร้างออบเจ็กต์ OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);  //OlED

// สร้าง object mpu6050 and Kalman Filter
MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;

// สร้างออบเจ็กต์ BluetoothSerial
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);              //ขนาดตัวอักษร
  display.setTextColor(WHITE, BLACK);  //สีจอแสดงผล

  pinMode(27, INPUT_PULLUP);  //status setup pin
  pinMode(4, INPUT_PULLUP);   //reset setup pin
  pinMode( 32, OUTPUT);       //LED Reset red
  pinMode( 33, OUTPUT);       //LED jiro green  
  D_print_NOT();              //status setup pin
  encoder_setup();            //Rotary Encoder Setup Function

  // เริ่มต้นการเชื่อมต่อ Bluetooth
  SerialBT.begin("Path survey tool Ver 1.0");  // ชื่อที่จะแสดงเมื่อค้นหาอุปกรณ์

  //Setup MPU6050
  MPU6050_Setup();

  //setup function หาค่าเฉลี่ย
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;  // กำหนดค่าเริ่มต้นให้กับอาร์เรย์
  }
}

void loop() {
  float KalmanY_angle = MPU6050_Loop();
  drawrect();
  D_print_name(String(SD_No));
  D_print_distance(encoder_value());
  D_print_angle(KalmanY_angle);
  status_function();
  reset_function();
  reset_SD_No();
  Send_New_Value();
  BT_Send_Average();
  kalAngleY_average = calculateAverage(KalmanY_angle);
  //Serial.println(SD_Name, " ", SD_Distance, " ",SD_Angle);
  if (reset == true) {
    BT_Send(true);
    digitalWrite(32, LOW);
    D_print_NOT();
  }

  delay(2);
}
//สร้างกรอบ
void drawrect() {
  display.drawRect(0, 0, 128, 16, SSD1306_WHITE);
  display.drawRect(0, 16, 128, 16, SSD1306_WHITE);
  display.drawRect(0, 32, 128, 16, SSD1306_WHITE);
  display.drawRect(0, 48, 128, 16, SSD1306_WHITE);
  display.display();
}
//แสดงผลชื่อไฟล์
void D_print_name(String Name) {
  display.setCursor(2, 4);
  display.print("FileName: ");
  display.print("No." + Name);
  display.display();
}
//แสดงระยะทาง
void D_print_distance(float encoder_rotation) {
  display.setCursor(2, 20);
  display.print("Distance: ");
  const float R = 0.05013380707395;  //รัสมีล้อ m.
  float distance = (encoder_rotation / 720) * (2 * pi * R);
  SD_Distance = distance;   //ส่งค่า SD card m.
  display.print(distance);  //R = รัสมีล้อ
  display.print(" ");
  display.setCursor(120, 20);
  display.print("m");
  display.display();
}
//แสดงมุม
void D_print_angle(float angle) {
  display.setCursor(2, 36);
  display.print("Angle: ");
  SD_Angle = angle;  //ส่งค่า SD Card
  display.print(kalAngleY_average);
  display.print("    ");
  display.setCursor(85, 36);
  display.print("Degree");
  display.display();
}
//แสดงสถานะ(เริ่มทำงาน)
void D_print_OK() {
  display.setCursor(2, 52);
  display.print("Status: ---OK---");
  display.display();
}
//แสดงสถานะ(หยุดทำงาน)
void D_print_NOT() {
  display.setCursor(2, 52);
  display.print("Status:   NOT   ");
  display.display();
}

//Rotary Encoder Function
void encoder_setup() {        //setup pin encoder
  pinMode(26, INPUT_PULLUP);  //Green
  pinMode(25, INPUT_PULLUP);  //White
  attachInterrupt(26, A_Counter, RISING);
  attachInterrupt(25, B_Counter, RISING);
}

signed int encoder_value() {  //ส่งออกค่า encoder ไม่ให้ส่งค่าซ้ำ
  if (status == false) {
    counter = 0;
    Old_Distance = -9999.00;
  }
  if (counter != temp) {
    return counter;
    temp = counter;
  }
}

void A_Counter() {  //นับสัญญาน A
  if (status == true) {
    if (digitalRead(25) == LOW) {  //25
      counter++;
    } else {
      counter--;
    }
  }
}

void B_Counter() {  //นับสัญญาน B
  if (status == true) {
    if (digitalRead(26) == LOW) {  //26
      counter--;
    } else {
      counter++;
    }
  }
}

void status_function() {  //รับค่าจากปุ่มกด startus
  if (digitalRead(27) == LOW & button_status == true) {
    if (status == false) {
      status = true;
    } else {
      status = false;
    }
    button_status = false;
  }
  if (digitalRead(27) == HIGH) {
    button_status = true;
  }
}

void reset_function() {  //รับค่าจากปุ่มกด reset
  if (digitalRead(4) == LOW & button_reset == true) {
    reset = true;
  }
  if (digitalRead(4) == LOW & button_reset == false) {
    reset = false;
  }
  button_reset = false;
  if (digitalRead(4) == HIGH) {
    button_reset = true;
  }
}

void reset_SD_No() {  //เปลี่ยนชื่อ SD No.xx
  if (status == false & SD_Distance != 0.00) {
    SD_No++;
  }
}
void Send_New_Value() {                      // ส่งข้อมูลค่าที่ไม่ซ้ำอกก BT
  if (SD_Distance - Old_Distance >= 5) {  //ทำงานทุก 5 m.
    if (SD_Distance != 0){
      digitalWrite(32, HIGH);
      D_print_OK();
      }
    Old_Distance = SD_Distance;
  }
}
void BT_Send(bool BT_ON_OFF) {  //ส่งค่าออก BT
  if (BT_ON_OFF == true) {
    SerialBT.print(",");
    SerialBT.print("No.");
    SerialBT.print(SD_No);
    SerialBT.print(",");
    SerialBT.print(SD_Distance);
    SerialBT.print(",");
    SerialBT.println(kalAngleY_average);
  }
}
//ส่งค่าที่ใกล้เคียงกับ average
 void BT_Send_Average() {
   if (abs(kalAngleY - kalAngleY_average) <= 0.15) {
     digitalWrite(33, HIGH);
   } else {
     digitalWrite(33, LOW);
   }
}
void MPU6050_Setup() {  //MPU6050 Setup Function
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
#if 0  //เป็น 1 Auto Calibrate ทำงาน, เป็น 0 Calibrate ด้วยตนเองทำงาน
  // คาลิเบรตไจโรสโคปและแอคเซลเลอโรมิเตอร์ 
  mpu.CalibrateGyro(10); 
  mpu.CalibrateAccel(10);
#else
  // กำหนดค่าการคาลิเบรตด้วยตนเอง
  mpu.setXAccelOffset(-286);  // กำหนดค่า offset ของแกน X ของ accelerometer
  mpu.setYAccelOffset(-450);  // กำหนดค่า offset ของแกน Y ของ accelerometer
  mpu.setZAccelOffset(839);   // กำหนดค่า offset ของแกน Z ของ accelerometer1
  mpu.setXGyroOffset(112);    // กำหนดค่า offset ของแกน X ของ gyroscope
  mpu.setYGyroOffset(-45);    // กำหนดค่า offset ของแกน Y ของ gyroscope
  mpu.setZGyroOffset(14);     // กำหนดค่า offset ของแกน Z ของ gyroscope
#endif

  // ตั้งค่าพารามิเตอร์ Kalman Filter
  const float Q_angle = 0.05;  //ควรต่ำลง
  const float Q_gyro = 0.05;   //ควรต่ำลง
  const float R_angle = 1;     //ควรสูงขึ้น

  kalmanX.setQangle(Q_angle);    // Q_angle
  kalmanX.setQbias(Q_gyro);      // Q_gyro
  kalmanX.setRmeasure(R_angle);  // R_angle

  kalmanY.setQangle(Q_angle);    // Q_angle
  kalmanY.setQbias(Q_gyro);      // Q_gyro
  kalmanY.setRmeasure(R_angle);  // R_angle

  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();

#ifdef RESTRICT_PITCH  // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll);  // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

float MPU6050_Loop() {  //MPU6050 Loop Function
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroZ = mpu.getRotationZ();
  tempRaw = mpu.getTemperature();

  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH  // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0;  // Convert to deg/s
  double gyroYrate = gyroY / 131.0;  // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;                           // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

    /* Print Data */
#if 0  // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

#if 1
  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX);
  Serial.print("\t");

  Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY);
  Serial.print("\t");
  Serial.print(2);
  Serial.print("\t");
  Serial.print(-2);
  Serial.print("\r\n");
  //SerialBT.print(",");
  //SerialBT.print(kalAngleX);
  //SerialBT.print(",");
  //erialBT.println(kalAngleY);
#endif

#if 0  // Set to 1 to print the temperature
    Serial.print("\t");

    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
#endif

#if 0
    Serial.print(accX);Serial.print("\t");Serial.print(accY);Serial.print("\t");Serial.print(accZ);Serial.print("\t\t");
    Serial.print(gyroX);Serial.print("\t");Serial.print(gyroY);Serial.print("\t");Serial.println(gyroZ);
#endif
  return kalAngleY;
}

//function หาค่าเฉลี่ย
float calculateAverage(float newValue) {
  total = total - readings[readIndex];  // ลบค่าที่เก่าออกจากผลรวม
  readings[readIndex] = newValue;       // เก็บค่าที่อ่านได้ใหม่ในอาร์เรย์
  total = total + readings[readIndex];  // เพิ่มค่าที่อ่านได้ใหม่ในผลรวม
  readIndex = readIndex + 1;            // เพิ่มดัชนีการอ่านค่า

  if (readIndex >= numReadings) {
    readIndex = 0;  // ถ้าดัชนีเกินจำนวนการอ่านค่า ให้เริ่มต้นใหม่
  }

  return total / numReadings;  // คำนวณและคืนค่าค่าเฉลี่ย
}
