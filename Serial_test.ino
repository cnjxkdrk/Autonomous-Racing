#include <SoftwareSerial.h>
#include <AFMotor.h>

#define L_BASE 200  // left motor base speed
#define R_BASE 200  // right motor base speed

AF_DCMotor motor_L(3);  // left motor
AF_DCMotor motor_R(4);  // right motor

String Speed;
char  LorR;
int  i, s;
int  v1, v2;
char DataToRead[6];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motor_L.run(RELEASE); // motor stop
  motor_R.run(RELEASE);
}

void loop() {
  DataToRead[5] = '\n';
  Serial.readBytesUntil(char(13), DataToRead, 5);
  
/* For Debugging, send string to RPi */
  for (i = 0; i < 6; i++) {
    Serial.write(DataToRead[i]);
    if (DataToRead[i] == '\n') break;
  }
/* End of Debugging */
  LorR = DataToRead[0];
  Speed = "";
  for (i = 1; ((DataToRead[i] != '\n') || (DataToRead[i] != '\r')) && (i < 6); i++) {
    Speed += DataToRead[i];
  }
  if(Speed.length() > 0 && (LorR == 'L' || LorR == 'R')){
    s = Speed.toInt();
    //Serial.println(LorR);
    //Serial.print("s= ");
    //Serial.println(s);
    if (LorR == 'L') {
      // Turn left wheel with speed s
      if(s<=5) {
        v1 = L_BASE;
        v2 = R_BASE + 10*s;
      }
      else if(s<10) {
        v1 = L_BASE - 5*s;
        v2 = R_BASE + 5*s;
      }
      else {
        v1 = 0;
        v2 = R_BASE + 3*s;
      }
      
      motor_L.setSpeed(v1);
      motor_R.setSpeed(v2);
      motor_L.run(FORWARD); // motor run
      motor_R.run(FORWARD);
    }
    else if (LorR == 'R') {
      // Turn right wheel with speed s
      if(s<=5) {
        v1 = L_BASE + 10*s;
        v2 = R_BASE;
      }
      else if(s<10) {
        v1 = L_BASE + 5*s;
        v2 = R_BASE - 5*s;
      }
      else {
        v1 = L_BASE + 3*s;
        v2 = 0;
      }
      motor_L.setSpeed(v1);
      motor_R.setSpeed(v2);
      motor_L.run(FORWARD); // motor run
      motor_R.run(FORWARD);
    }
  }

  delay(100);
  motor_L.run(RELEASE); // motor stop
  motor_R.run(RELEASE);
  delay(200);
}
