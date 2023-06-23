/*
     6 Eksen AccGyro Veri Okuma örneği,

     Bu örnekte temel konfigürasyon ayarları yapılmaktadır.
     Sensörden gelen İvmeölçerden(Acc) X,Y,Z eksen değerleri Dönüölçerden(Gyro) X,Y,Z eksen değerleri ve C ile F cinsinden sıcaklık değerlerini
     seri termianle yazdırmaktadır.

     Bu algılayıcı I2C haberleşme protokolü ile çalışmaktadır.

     Bu örnek Deneyap 6-Eksen Ataletsel Ölçüm Birimi ve Deneyap 9-Eksen Ataletsel Ölçüm Birimi için oluşturulmuştur
        ------> www.....com <------ //docs
        ------> https://github.com/deneyapkart/deneyap-6-eksen-ataletsel-olcum-birimi-arduino-library <------

*/

#include "SensorFusion.h" //SF
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

#include <Deneyap_6EksenAtaletselOlcumBirimi.h>       // Deneyap_IvmeOlcerVeDonuOlcer.h kütüphanesi eklendi

LSM6DSM AccGyro;                                      // AccGyro icin Class tanimlamasi

void setup() {
  Wire.setSDA(16);
  Wire.setSCL(17);
  Serial.begin(115200);
  while (!Serial); // Seri haberleşme başlatıldı
  if (AccGyro.begin() != IMU_SUCCESS) {             // begin(slaveAdress) fonksiyonu ile cihazların haberleşmesi başlatıldı
    delay(2500);
    Serial.println("I2C bağlantısı başarısız ");  // I2C bağlantısı başarısız olursa seri terminale yazdırma
    while (1);
  }
}

void loop() {
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  //fusion.MahonyUpdate(abs(AccGyro.readFloatGyroX()) * DEG_TO_RAD, abs(AccGyro.readFloatGyroY()) * DEG_TO_RAD, abs(AccGyro.readFloatGyroZ()) * DEG_TO_RAD, AccGyro.readFloatAccelX(), AccGyro.readFloatAccelY(), AccGyro.readFloatAccelZ(), deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  //fusion.MadgwickUpdate(gx, gy, gz, asx, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate
  fusion.MadgwickUpdate((AccGyro.readFloatGyroX()) * DEG_TO_RAD, (AccGyro.readFloatGyroY()) * DEG_TO_RAD, (AccGyro.readFloatGyroZ()) * DEG_TO_RAD, AccGyro.readFloatAccelX(), AccGyro.readFloatAccelY(), AccGyro.readFloatAccelZ(), deltat);  //mahony is suggested if there isn't the mag and the mcu is slow

  pitch = abs(fusion.getPitch());
  roll = abs(fusion.getRoll());    //you could also use getRollRadians() ecc
  yaw = abs(fusion.getYaw());

  //Serial.print("Pitch:\t"); 
  Serial.print(pitch);
  //Serial.print("\tRoll:\t"); 
  Serial.print(","); 
  Serial.print(roll);
  Serial.print(","); 
  //Serial.print("\tYaw:\t"); 
  Serial.print(yaw);
  Serial.println();
  //delay(100);
}
