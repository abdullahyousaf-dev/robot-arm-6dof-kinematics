#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// float L1 = 2.5;
float L2 = 10.4;
float L3 = 9.75;
float L4 = 2.5;

float d1 = 11.3;
float d6 = 15.9;

float rad = PI/180;

float a[] = {0,0,L2,L3,L4,0};
float alpha[] = {0, 90, 0, 0, -90, 0};
float d[] = {d1,0,0,0,0,d6};

float input_theta[] = {0,0,0,0,0,0};
float theta[] = {0,0,0,0,0,0};
float additional_theta[] = {5, 12, 128, -165, 0, 0};

const int rows = 4;
const int cols = 4;
const int num_matrices = 6;
float matrices[num_matrices][rows][cols];

void setup() {
  Serial.begin(9600);

  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(6);
  servo5.attach(7);
  servo6.attach(8);

  servo1.write(0);
  servo2.write(30);
  servo3.write(50);
  servo4.write(80);
  servo5.write(90);
  servo6.write(0);

  Serial.println("");
  Serial.println("Enter 6 theta values:");
  
  for (int i = 0; i<6 ; i++) {
    while (!Serial.available()) {} // Wait until data is available
    input_theta[i] = Serial.parseFloat(); // Read the theta value
    Serial.print(" Theta ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(input_theta[i]);
  }

  for (int i=0; i<6 ; i++){

    if (i == 2){
      theta[i] = -input_theta[i] + additional_theta[i];
    }
    else {
      theta[i] = int ((94.4/100)*input_theta[i]) + additional_theta[i];
    }
  }

  for ( int j=0 ; j<num_matrices ; j++) {
    float matrix[rows][cols] = {
      {  cos(rad*theta[j])                   ,          -sin(rad*theta[j])           ,           0         ,            a[j]           },
      { (sin(rad*theta[j]))*(cos(rad*alpha[j]))  ,  (cos(rad*theta[j]))*(cos(rad*alpha[j]))  ,   -(sin(rad*alpha[j]))  ,   -(sin(rad*alpha[j]))*(d[j]) },
      { (sin(rad*theta[j]))*(sin(rad*alpha[j]))  ,  (cos(rad*theta[j]))*(sin(rad*alpha[j]))  ,     cos(rad*alpha[j])   ,      cos(rad*alpha[j])*d[j]   },
      {                0                 ,                 0                 ,           0         ,              1            }
    };
    for (int i=0 ; i<rows ; i++){
      for (int k=0 ; k<cols ; k++){
        matrices[j][i][k] = matrix[i][k];
      }
    }
  }


  float resultMatrix[rows][cols];
  multiplyMatrices(matrices , resultMatrix);

  float x = resultMatrix[0][3];
  float y = resultMatrix[1][3];
  float z = resultMatrix[2][3];

  Serial.println("");
  if (z <= 2){
    Serial.print("Workspace Restriction !!!");
    Serial.print("[\t");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t]");
  }
  else if (z <= 20 && x >= -8 && x <= 8 && y >= -8 && y <= 8) {
    Serial.println("Self Collision !!!");
    Serial.print("[\t");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t]");
  }
  else {
    for (int i=0; i<rows; i++) {
      Serial.print("[\t");
      for (int j=0; j<cols; j++) {
        Serial.print(resultMatrix[i][j]);
        Serial.print("\t");
      }
      Serial.print("]");
      Serial.println("");
    }

    signalMotors();
  }

}



void loop() {


delay(10);
}

void multiplyMatrices(float matrices[][rows][cols], float resultMatrix[][cols]) {
  // Initialize result matrix
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      resultMatrix[i][j] = matrices[0][i][j];
    }
  }

  // Multiply matrices successively
  for (int k = 1; k < 6; k++) {
    float tempMatrix[rows][cols];
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        tempMatrix[i][j] = 0;
        for (int l = 0; l < cols; l++) {
          tempMatrix[i][j] += resultMatrix[i][l] * matrices[k][l][j];
        }
      }
    }
    // Update result matrix
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        resultMatrix[i][j] = tempMatrix[i][j];
      }
    }
  }
}

void signalMotors(){
    servo1.write(input_theta[0]);
    delay(1000);
    servo2.write(input_theta[1]);
    delay(1000);
    servo3.write(input_theta[2]);
    delay(1000);
    servo4.write(input_theta[3]);
    delay(1000);
    servo5.write(input_theta[4]);
    delay(1000);
    servo6.write(input_theta[5]);
    delay(1000);

}