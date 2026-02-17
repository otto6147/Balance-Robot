#include <MPU6050_tockn.h>
#include <Wire.h>
#include <IRremote.h>

#define RECV_PIN 4
#define IN1 13
#define IN2 12
#define ENA 5
#define ENB 6
#define IN3 9
#define IN4 8

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long lastCode = 0;

MPU6050 mpu6050(Wire);

float kp = 25, ki = 0.2, kd = 10;
float setpoint = 0; // สมดุลคือ 0 องศา
float error, lastError, integral, derivative, output;
int speedOffset = 0; // ปรับความเร็วเพื่อเดินหน้า/ถอยหลัง
int turnOffset = 0;  // ปรับความเร็วเพื่อเลี้ยว

void pidControl(float pitch) {
  error = setpoint - pitch;
  integral += error;
  derivative = error - lastError;
  output = kp * error + ki * integral + kd * derivative;
  lastError = error;

  // รวมผล PID กับความเร็วจากการควบคุมทิศทาง
  int leftSpeed = constrain(output + speedOffset - turnOffset, -160, 160);
  int rightSpeed = constrain(output + speedOffset + turnOffset, -160, 160);

  motorcontrol(leftSpeed, rightSpeed);
}

void motorcontrol(int leftSpeed, int rightSpeed) {
  // มอเตอร์ซ้าย
  analogWrite(ENA, abs(leftSpeed));
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);

  // มอเตอร์ขวา
  analogWrite(ENB, abs(rightSpeed));
  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  irrecv.enableIRIn();
  mpu6050.begin();
  mpu6050.setGyroOffsets(0.5, 1.9, -0.17);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void loop() {
  mpu6050.update();
  float angle = mpu6050.getAngleX();

  Serial.print("angleX: "); Serial.print(angle);
  Serial.print("\tSetpoint: "); Serial.println(setpoint);

  pidControl(angle);

  // IR remote
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;
    if (code == 0xFFFFFFFF) code = lastCode;
    else lastCode = code;

    switch (code) {
      case 0xFF18E7: // Forward
        Serial.println("Forward");
        speedOffset = 30; // ปรับความเร็วเสริมเดินหน้า
        break;
      case 0xFF4AB5: // Backward
        Serial.println("Backward");
        speedOffset = -30;
        break;
      case 0xFF38C7: // Stop
        Serial.println("Stop");
        speedOffset = 0;
        turnOffset = 0;
        break;
      case 0xFF5AA5: // Left
        Serial.println("Left");
        turnOffset = -20;
        break;
      case 0xFF10EF: // Right
        Serial.println("Right");
        turnOffset = 20;
        break;
    }

    irrecv.resume();
  }

  delay(10);
}
