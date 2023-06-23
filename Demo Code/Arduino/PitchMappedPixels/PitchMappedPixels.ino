// Simple strand test for Adafruit Dot Star RGB LED strip.
// This is a basic diagnostic tool, NOT a graphics demo...helps confirm
// correct wiring and tests each pixel's ability to display red, green
// and blue and to forward data down the line.  By limiting the number
// and color of LEDs, it's reasonably safe to power a couple meters off
// the Arduino's 5V pin.  DON'T try that with other code!
#define DEBUG 1

#include "SensorFusion.h" //SF
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

#include <Deneyap_6EksenAtaletselOlcumBirimi.h>       // Deneyap_IvmeOlcerVeDonuOlcer.h kütüphanesi eklendi

LSM6DSM AccGyro;
#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 24 // Number of LEDs in strip

const uint8_t gamma_lut[256] = {
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,
  2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   4,   4,
  4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   6,   7,   7,   7,   8,
  8,   8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  12,  12,  12,  13,  13,
  14,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,
  22,  22,  23,  23,  24,  25,  25,  26,  27,  27,  28,  29,  29,  30,  31,  32,
  32,  33,  34,  35,  35,  36,  37,  38,  39,  40,  40,  41,  42,  43,  44,  45,
  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  60,  61,  62,
  63,  64,  65,  67,  68,  69,  70,  72,  73,  74,  76,  77,  78,  80,  81,  82,
  84,  85,  87,  88,  90,  91,  93,  94,  96,  97,  99, 101, 102, 104, 105, 107,
  109, 111, 112, 114, 116, 118, 119, 121, 123, 125, 127, 129, 131, 132, 134, 136,
  138, 140, 142, 144, 147, 149, 151, 153, 155, 157, 159, 162, 164, 166, 168, 171,
  173, 175, 178, 180, 182, 185, 187, 190, 192, 195, 197, 200, 202, 205, 207, 210,
  213, 215, 218, 221, 223, 226, 229, 232, 235, 237, 240, 243, 246, 249, 252, 255,
};
const uint8_t PROGMEM gamma8[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};
// Here's how to control the LEDs from any two pins:
#define DATAPIN    19
#define CLOCKPIN   18
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno = pin 11 for data, 13 for clock, other boards are different).
//Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BRG);
volatile uint16_t j = 0;
volatile bool flag = 0;

void setup() {

  Wire.setSDA(16);
  Wire.setSCL(17);
  if (DEBUG) {
    Serial.begin(115200);

    //while (!Serial); // Seri haberleşme başlatıldı
  }

  if (AccGyro.begin() != IMU_SUCCESS) {             // begin(slaveAdress) fonksiyonu ile cihazların haberleşmesi başlatıldı
    delay(2500);
    if (DEBUG) {
      Serial.println("I2C bağlantısı başarısız ");  // I2C bağlantısı başarısız olursa seri terminale yazdırma

    }

    while (1);
  }

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
  strip.setBrightness(24);
}


void loop() {

  if (flag == 1)
  {
    rainbowCycle(j);
    flag = 0;
  }
}

void loop1()
{
  if (flag == 0)
  {
    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
    //choose only one of these two:
    //fusion.MahonyUpdate(abs(AccGyro.readFloatGyroX()) * DEG_TO_RAD, abs(AccGyro.readFloatGyroY()) * DEG_TO_RAD, abs(AccGyro.readFloatGyroZ()) * DEG_TO_RAD, AccGyro.readFloatAccelX(), AccGyro.readFloatAccelY(), AccGyro.readFloatAccelZ(), deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    //fusion.MadgwickUpdate(gx, gy, gz, asx, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate
    fusion.MadgwickUpdate((AccGyro.readFloatGyroX()) * DEG_TO_RAD, (AccGyro.readFloatGyroY()) * DEG_TO_RAD, (AccGyro.readFloatGyroZ()) * DEG_TO_RAD, AccGyro.readFloatAccelX(), AccGyro.readFloatAccelY(), AccGyro.readFloatAccelZ(), deltat);  //mahony is suggested if there isn't the mag and the mcu is slow

    pitch = (fusion.getPitch());
    roll = (fusion.getRoll());    //you could also use getRollRadians() ecc
    yaw = (fusion.getYaw());


    if (DEBUG)
    {
      //Serial.print("Pitch:\t");
      Serial.print(pitch);
      //Serial.print("\tRoll:\t");
      Serial.print(",");
      Serial.print(roll); //this 90...
      j = map(roll, -90, 90, 0, 255);
      Serial.print(",");
      //Serial.print("\tYaw:\t");
      Serial.print(yaw);
      Serial.println();
      //delay(100);
    }
    flag = 1;

  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(volatile uint16_t pos) {
  uint16_t i;

  // for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
  for (i = 0; i < strip.numPixels() / 2; i++) {

    strip.setPixelColor(i, Wheel(((i * 256 / (strip.numPixels() / 1)) + pos) & 255));
    strip.setPixelColor((strip.numPixels() - 1) - i, Wheel(((i * 256 / (strip.numPixels() / 1)) + pos) & 255));

  }
  strip.show();
  //delay(5);
  //flag = 0;
  // }
}
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
