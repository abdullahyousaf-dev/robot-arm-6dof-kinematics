#include <Servo.h>
#include <math.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

float l2 = 10.4;
float l3 = 12;

float theta[] = {0,0,0};

float t0[] = {0,0,0};
float tf[] = {0,0,0};

float size = 100;

float pt1[] = {0,10,0};
float pt2[] = {0,18,0};
float pt3[] = {5,10,0};
float pt4[] = {5,18,0};

float x0, y0, z0;
float xf, yf, zf;

float motor_theta[] = {t0[0],t0[1],t0[2]};

void setup() {

  Serial.begin(9600);

  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(6);

  servo1.write(motor_theta[0]);
  servo2.write(motor_theta[1]);
  servo3.write(motor_theta[2]);


  for (int i = 1; i < 4; i++) {

      int initial = i;
      int final = i + 1;

      if (initial == 1) {
        x0 = pt1[0];
        y0 = pt1[1];
        z0 = pt1[2];
      } 
      else if (initial == 2) {
        x0 = pt2[0];
        y0 = pt2[1];
        z0 = pt2[2];
      } 
      else if (initial == 3) {
        x0 = pt3[0];
        y0 = pt3[1];
        z0 = pt3[2];
      } 
      else if (initial == 4) {
        x0 = pt4[0];
        y0 = pt4[1];
        z0 = pt4[2];
      }

      if (final == 1) {
        xf = pt1[0];
        yf = pt1[1];
        zf = pt1[2];
      } 
      else if (final == 2) {
        xf = pt2[0];
        yf = pt2[1];
        zf = pt2[2];
      } 
      else if (final == 3) {
        xf = pt3[0];
        yf = pt3[1];
        zf = pt3[2];
      } 
      else if (final == 4) {
        xf = pt4[0];
        yf = pt4[1];
        zf = pt4[2];
      }

     // Initial Thetas
        t0[0] = atan2(y0, x0);

        float cos02 = (sq(x0) + sq(y0) + sq(z0) - sq(l2) - sq(l3)) / (2 * l2 * l3);
        float sin02 = sqrt(1 - sq(cos02));

        t0[2] = atan2(sin02, cos02);

        float a01 = -1 * l3 * sin(t0[2]);
        float b01 = l2 + l3 * cos(t0[2]);
        float c01 = z0;

        float sq_root = sqrt(sq(a01) + sq(b01) - sq(c01));

        t0[1] = (atan2(c01, sq_root)) - (atan2(a01, b01));

        Serial.println(t0[0]);
        Serial.println(t0[1]);
        Serial.println(t0[2]);

      // Final Thetas
        tf[0] = atan2(yf, xf);

        float cosf2 = (sq(xf) + sq(yf) + sq(zf) - sq(l2) - sq(l3)) / (2 * l2 * l3);
        float sinf2 = sqrt(1 - sq(cosf2));

        tf[2] = atan2(sinf2, cosf2);

        float af1 = -1 * l3 * sin(tf[2]);
        float bf1 = l2 + l3 * cos(tf[2]);
        float cf1 = zf;

        sq_root = sqrt(sq(af1) + sq(bf1) - sq(cf1));

        tf[1] = (atan2(cf1, sq_root)) - (atan2(af1, bf1));

        Serial.println(degrees(tf[0]));
        Serial.println(degrees(tf[1]));
        Serial.println(degrees(tf[2]));

        // Serial.print(initial);
        // Serial.print(final);
            // Serial.print(motor_theta[0]);
            // Serial.print(motor_theta[1]);
            // Serial.print(motor_theta[2]);
          for (int j=0; j < size; j+=1){

            motor_theta[0] = ((tf[0] - t0[0])/size) + motor_theta[0];
            motor_theta[1] = ((tf[1] - t0[1])/size) + motor_theta[1];
            motor_theta[2] = ((tf[2] - t0[2])/size) + motor_theta[2];

            motor_theta[0] = degrees(motor_theta[0]);
            motor_theta[1] = degrees(motor_theta[1]);
            motor_theta[2] = degrees(motor_theta[2]);

            servo1.write(motor_theta[0]);
            delay(200);
            servo2.write(motor_theta[1]);
            delay(200);
            servo3.write(motor_theta[2]);
            delay(200);

          }

          delay(1000);
            // Serial.print(motor_theta[0]);
            // Serial.print(motor_theta[1]);
            // Serial.print(motor_theta[2]);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
