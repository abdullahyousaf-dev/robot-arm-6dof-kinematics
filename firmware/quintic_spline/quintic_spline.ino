#include <Servo.h>
#include <math.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

float l2 = 10.4;
float l3 = 12;
int p = 0;
const int size = 500; // Further increase the number of steps to make the movement even slower

float pt1[] = {0, 15, -5};
float pt2[] = {0, 22, -5};
float pt3[] = {5, 15, -5};
float pt4[] = {5, 22, -5};

float x0, y0, z0;
float xf, yf, zf;

float t0 = 0;
float tf = 4; // Further increase the duration to make movements slower

void setup() {
  Serial.begin(9600);

  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(6);

  // Point 1-2:
  float angle01 = 90;
  float angle02 = 47.68;
  float angle03 = 105.46;
  float angle04 = 100;
  
  float anglef1 = 90;
  float anglef2 = 20.51;
  float anglef3 = 55.70;
  float anglef4 = 100;
  
  p = 12;
  moveServos(angle01, angle02, angle03, angle04, anglef1, anglef2, anglef3, anglef4);

  // Point 2-3:
  angle01 = anglef1;
  angle02 = anglef2;
  angle03 = anglef3;
  angle04 = anglef4;
  
  anglef1 = 71.57;
  anglef2 = 45.16;
  anglef3 = 100.74;
  anglef4 = 100;
  
  p = 23;
  moveServos(angle01, angle02, angle03, angle04, anglef1, anglef2, anglef3, anglef4);

  // Point 3-4:
  angle01 = anglef1;
  angle02 = anglef2;
  angle03 = anglef3;
  angle04 = anglef4;
  
  anglef1 = 77.20;
  anglef2 = 17.29;
  anglef3 = 49.93;
  anglef4 = 100;
  
  p = 34;
  moveServos(angle01, angle02, angle03, angle04, anglef1, anglef2, anglef3, anglef4);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void moveServos(float angle01, float angle02, float angle03, float angle04, 
                float anglef1, float anglef2, float anglef3, float anglef4) {
  float a0, a1, a2, a3, a4, a5;
  float b0, b1, b2, b3, b4, b5;
  float c0, c1, c2, c3, c4, c5;

  // Calculate coefficients for the quintic spline
  calculateQuinticCoefficients(t0, tf, angle01, anglef1, 0, 0, 0, 0, a0, a1, a2, a3, a4, a5);
  calculateQuinticCoefficients(t0, tf, angle02, anglef2, 0, 0, 0, 0, b0, b1, b2, b3, b4, b5);
  calculateQuinticCoefficients(t0, tf, angle03, anglef3, 0, 0, 0, 0, c0, c1, c2, c3, c4, c5);

  float new1 = angle01;
  float new2 = angle02;
  float new3 = angle03;

  servo1.write(angle01);
  delay(300);
  servo2.write(angle02);
  delay(300);
  servo3.write(angle03);
  delay(300);
  servo4.write(angle04);
  delay(300);

  // Interpolation
  for (float t = t0; t <= tf; t += (tf - t0) / size) {
    new1 = quinticSpline(t, a0, a1, a2, a3, a4, a5);
    new2 = quinticSpline(t, b0, b1, b2, b3, b4, b5);
    new3 = quinticSpline(t, c0, c1, c2, c3, c4, c5);
	
    if (p == 12) {
      if (new1 <= angle01) {
        servo1.write(new1);
        delay(15); // Increase the delay to make the movement slower
      }

      if (new2 <= angle02) {
        servo2.write(new2);
        delay(15); // Increase the delay to make the movement slower
      }
      if (new3 <= angle03) {
        servo3.write(new3);
        delay(15); // Increase the delay to make the movement slower
      }
    }
    if (p == 23) {
      if (new1 <= angle01) {
        servo1.write(new1);
        delay(15); // Increase the delay to make the movement slower
      }

      if (new2 >= angle02) {
        servo2.write(new2);
        delay(15); // Increase the delay to make the movement slower
      }
      if (new3 >= angle03) {
        servo3.write(new3);
        delay(15); // Increase the delay to make the movement slower
      }
    }
    if (p == 34) {
      if (new1 >= angle01) {
        servo1.write(new1);
        delay(15); // Increase the delay to make the movement slower
      }

      if (new2 <= angle02) {
        servo2.write(new2);
        delay(15); // Increase the delay to make the movement slower
      }
      if (new3 <= angle03) {
        servo3.write(new3);
        delay(15); // Increase the delay to make the movement slower
      }
    }
  }

  Serial.println(new1);
  Serial.println(new2);
  Serial.println(new3);
  delay(500);
}

void calculateQuinticCoefficients(float t0, float tf, float y0, float yf, float v0, float vf, float a0, float af,
                                  float &a, float &b, float &c, float &d, float &e, float &f) {
  a = y0;
  b = v0;
  c = 0.5 * a0;
  float T = tf - t0;
  float T2 = T * T;
  float T3 = T2 * T;
  float T4 = T3 * T;
  float T5 = T4 * T;

  f = (6 * (yf - y0) - 3 * (vf + 2 * v0) * T - 0.5 * (af - 3 * a0) * T2) / T5;
  e = (0.5 * (af + 3 * a0) * T - 3 * (vf + 2 * v0)) / T4;
  d = (vf - v0) / T3 - (af - a0) / (2 * T2) - 3 * e * T / 2 - 5 * f * T2 / 6;
}

float quinticSpline(float t, float a, float b, float c, float d, float e, float f) {
  return a + b * t + c * t * t + d * t * t * t + e * t * t * t * t + f * t * t * t * t * t;
}
