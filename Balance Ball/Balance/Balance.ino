// Packages :
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

FreeSixIMU sixDOF = FreeSixIMU();


// accelerometer constants :
const float accZeroLimit = 30;    // free fall if abs(acc) < accZeroLimit

// General variables :
float acc_values[3];
int bluetoothValue;
float abs_a;      // absolute value of accelerometer
float t_start;
float startTime;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  

  delay(5);
  sixDOF.init();
  delay(5);
  startTime = millis();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
}

void serialEvent() {
  if (Serial.available() > 0) {
    t_start = micros();
    bluetoothValue = Serial.parseInt();
    if (bluetoothValue == 0) {
      while (true) {
        sixDOF.getValues(acc_values);
        abs_a = sqrt(acc_values[0]*acc_values[0]+acc_values[1]*acc_values[1]+acc_values[2]*acc_values[2]);
        if (abs_a < accZeroLimit) {
          Serial.println((micros() - t_start)/1000000);
          delay(500);
          software_Reset();
          break;
        }
      }
    }
    Serial.end();
    Serial.begin(9600);
  }
}


void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");
}

void loop() {
  if (millis() > startTime + 180000) {
    software_Reset();
  }
}


