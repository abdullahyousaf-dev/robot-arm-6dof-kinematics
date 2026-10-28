#include <Servo.h>
#include <math.h>

float L2 = 10.5;
float L3 = 9;
float L4 = 2.3;
float L5 = 16;
float r = 17; // Assuming r is constant
float x, y, z; // Cartesian coordinates

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

float theta[] = {0,0,0,0};

void setup() {

  Serial.begin(9600);
  
  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(6);

  initial_angles();

  Serial.println();
  Serial.print("Enter the value of x: ");
  while (!Serial.available()) {} // Wait for data to be available
  x = Serial.parseFloat(); // Read the input as a float
  Serial.println(x);

  Serial.print("Enter the value of y: ");
  while (!Serial.available()) {} // Wait for data to be available
  y = Serial.parseFloat(); // Read the input as a float
  Serial.println(y);

  Serial.print("Enter the value of z: ");
  while (!Serial.available()) {} // Wait for data to be available
  z = Serial.parseFloat(); // Read the input as a float
  Serial.println(z);

  for (int i = 0; i <= 180; i++) {
      float rad_i = radians(i);
      
      theta[3] = rad_i;
      
      // for theta [2]:
      float a2 = 2 * L2 * L3 + 2 * r * L2 * cos(theta[3]);
      float b2 = -2 * r * L2 * sin(theta[3]);
      float c2 = pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(L2, 2) - pow(L3, 2) - pow(r, 2) - 2 * r * L3 * cos(theta[3]);
      
      float sqr_root_2_value = pow(a2, 2) + pow(b2, 2) - pow(c2, 2);
      
      if (sqr_root_2_value > 0) {
        theta[2] = atan2(c2, sqrt(sqr_root_2_value)) - atan2(a2, b2);
      }
      else {
        continue;
      }
      
      // for theta[1]
      float a1 = -L3 * sin(theta[2]) - r * sin(theta[2] + theta[3]);
      float b1 = L2 + r * cos(theta[2] + theta[3]);
      float c1 = z;
      
      float sqr_root_1_value = pow(a1, 2) + pow(b1, 2) - pow(c1, 2);
      
      if (sqr_root_1_value > 0) {
        theta[1] = atan2(c1, sqrt(sqr_root_1_value)) - atan2(a1, b1);
      }
      else {
        continue;
        }
      
      float fwd_x = cos(theta[0]) * (L2 * cos(theta[1]) + L3 * cos(theta[1] - theta[2]) + r * cos(theta[1] - theta[2] - theta[3]));
      float fwd_y = sin(theta[0]) * (L2 * cos(theta[1]) + L3 * cos(theta[1] - theta[2]) + r * cos(theta[1] - theta[2] - theta[3]));
      float fwd_z = L2 * sin(theta[1]) + L3 * sin(theta[1] - theta[2]) + r * sin(theta[1] - theta[2] - theta[3]);
      
      float error_x = abs(fwd_x - x);
      float error_y = abs(fwd_y - y);
      float error_z = abs(fwd_z - z);
      
      if (error_x < 1 && error_y < 1 && error_z < 1) {
        thetas();
        break;
      }
    }
  }


void loop() {
  // put your main code here, to run repeatedly:

}

void thetas() {
  boolean s = false;

  for (int j = 0; j <= 180; j++) {
    if (!s) {
      for (int k = 0; k <= 180; k++) {
        theta[1] = radians(j);
        theta[2] = radians(k);
        
        float fwd_x = cos(theta[0]) * (L2 * cos(theta[1]) + L3 * cos(theta[1] - theta[2]) + r * cos(theta[1] - theta[2] - theta[3]));
        float fwd_y = sin(theta[0]) * (L2 * cos(theta[1]) + L3 * cos(theta[1] - theta[2]) + r * cos(theta[1] - theta[2] - theta[3]));
        float fwd_z = L2 * sin(theta[1]) + L3 * sin(theta[1] - theta[2]) + r * sin(theta[1] - theta[2] - theta[3]);
        
        float error_x = abs(fwd_x - x);
        float error_y = abs(fwd_y - y);
        float error_z = abs(fwd_z - z);
        
        if (error_x < 1 && error_y < 1 && error_z < 1) {
          theta[0] = degrees(theta[0]);
          theta[1] = j;
          theta[2] = k;
          theta[3] = degrees(theta[3]);
          
          Serial.println("\ntheta1: " + String(theta[0]));
          Serial.println("theta2: " + String(theta[1]));
          Serial.println("theta3: " + String(theta[2]));
          Serial.println("theta4: " + String(theta[3]));
          Serial.println("error_x: " + String(error_x) + "; error_y: " + String(error_y) + "; error_z: " + String(error_z) + ";");
          
          s = true;
          break;
        }
      }
    }
    else {
      break;
    }
  }
}

void initial_angles(){
  servo1.write(0);
  delay(100);
  servo2.write(0);
  delay(100);
  servo3.write(0);
  delay(100);
  servo4.write(0);
}