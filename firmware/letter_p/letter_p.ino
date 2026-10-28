#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int inner = 700;
int degree = 50;

void setup() {

  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(6);

  servo1.write(100);
  servo2.write(0);
  servo3.write(50);
  servo4.write(90);
  delay(2000);


// move up
  // servo3.write(100);
  // delay(inner);

  for (int pos=50; pos<=100; pos+=1){
    servo3.write(pos);
    delay(degree);
  }
  delay(inner);  



// move right
  // servo1.write(80);
  // delay(inner);

  for (int pos=100; pos>=80; pos-=1){
    servo1.write(pos);
    delay(degree);
  }
  delay(inner);


// move down
  // servo3.write(75);
  // delay(inner);

  for (int pos=100; pos>=70; pos-=1){
    servo3.write(pos);
    delay(degree);
  }
  delay(inner);


// move left
  // servo1.write(100);
  // delay(inner);

  for (int pos=80; pos<=100; pos+=1){
    servo1.write(pos);
    delay(degree);
  }
  delay(inner);

}

void loop() {

}
