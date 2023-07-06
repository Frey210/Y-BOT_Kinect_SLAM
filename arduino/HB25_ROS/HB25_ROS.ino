// Variabel untuk menyimpan data dari Python
int pwm_left = 0;
int pwm_right = 0;
int direction_left = 0;
int direction_right = 0;

// Pin untuk mengontrol driver motor
#include <Servo.h>  //  You need to include Servo.h as it is used by the HB-25 Library
#include <HB25MotorControl.h>

const byte controlPinL = 6;
const byte controlPinR = 7;
HB25MotorControl motorControlL(controlPinL);
HB25MotorControl motorControlR(controlPinR);

void setup() {
  motorControlL.begin();
  motorControlR.begin();

  // Inisialisasi Serial
  Serial.begin(57600);
}

void loop() {
  // Jika ada data yang diterima dari Python
  if (Serial.available()) {

    // Membaca data dari Serial
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Memisahkan data menjadi PWM kiri, arah kiri, PWM kanan, dan arah kanan
    int separator1 = input.indexOf(',');
    int separator2 = input.indexOf(',', separator1 + 1);
    int separator3 = input.indexOf(',', separator2 + 1);

    pwm_left = input.substring(0, separator1).toInt();
    direction_left = input.substring(separator1 + 1, separator2).toInt();
    pwm_right = input.substring(separator2 + 1, separator3).toInt();
    direction_right = input.substring(separator3 + 1).toInt();


    // Mengontrol motor
    motorControlL.moveAtSpeed(pwm_left);
    motorControlR.moveAtSpeed(pwm_right);
  }
}