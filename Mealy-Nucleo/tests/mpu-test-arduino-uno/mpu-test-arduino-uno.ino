#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Wire.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Serial found");

  if(!mpu.begin()){
    Serial.println("Failed to find mpu");
    while(1){
      delay(10);
    }
    
  }
  
  Serial.println("mpu found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acc x: ");
  Serial.print(a.acceleration.x);
  Serial.println();

  Serial.print("Gyro x: ");
  Serial.print(g.gyro.x);
  Serial.println();

  delay(500);

}
