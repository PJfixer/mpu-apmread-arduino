#include "MPU6000.h"

mpu6000_spi imu;

void setup() {
Serial.begin(115200);
pinMode(40,OUTPUT);
digitalWrite(40, HIGH);      
imu = mpu6000_spi();
Serial.print("whoami : ");
Serial.println(imu.whoami());
imu.init(1,BITS_DLPF_CFG_5HZ);
imu.set_acc_scale(BITS_FS_16G);
imu.set_gyro_scale(BITS_FS_2000DPS);

}

void loop() {
  // put your main code here, to run repeatedly:

   Serial.print("\nT= ");
   Serial.print(imu.read_temp());
   Serial.print(" X= ");
   Serial.print(imu.read_acc(0));  
   Serial.print(" Y= ");
   Serial.print(imu.read_acc(1)); 
   Serial.print(" Z= ");
   Serial.print(imu.read_acc(2));   
   Serial.print(" rX= ");
   Serial.print(imu.read_rot(0));  
   Serial.print(" rY= ");
   Serial.print(imu.read_rot(1)); 
   Serial.print(" rZ= ");
   Serial.print(imu.read_rot(2));
delay(100);
}
