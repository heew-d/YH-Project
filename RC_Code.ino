/* Autonomous Vehicle */
#include <SoftwareSerial.h>

// Bluetooth communication
#define TXD 2
#define RXD 3

#define SPEED 160
#define ROTATE 255
#define STOP 0

SoftwareSerial bluetooth(TXD, RXD);

const int F_Motor = 10;
const int B_Motor = 11;

const int L_Motor = 5;
const int R_Motor = 6;

char direction;

void setup()
{
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Motor PinMode
  pinMode(F_Motor, OUTPUT);
  pinMode(B_Motor, OUTPUT);
  pinMode(L_Motor, OUTPUT);
  pinMode(R_Motor, OUTPUT);

  // initial Motor
  digitalWrite(F_Motor, LOW);
  digitalWrite(B_Motor, LOW);
  digitalWrite(L_Motor, LOW);
  digitalWrite(R_Motor, LOW);

  // Check a AT Commands
  // Serial.println("Enter AT Commands :");
}

void loop()
{
  /* Set a Bluetooth HC-06 Code */
  // if(bluetooth.available())
  // {
  //   Serial.write(bluetooth.read());
  // }
  // if(Serial.available())
  // {
  //   bluetooth.write(Serial.read());
  // }

  /* Motor Control */
  if(bluetooth.available())
  {
    direction = bluetooth.read();
    Serial.println(direction);
    for (int i = 180; i > 100; i--)
    {
      switch(direction)
      {
        case 'w':
          analogWrite(F_Motor, i);
          analogWrite(B_Motor, STOP);
          analogWrite(R_Motor, STOP);
          analogWrite(L_Motor, STOP);
          break;

        case 'l':
          analogWrite(F_Motor, i);
          analogWrite(B_Motor, STOP);
          analogWrite(R_Motor, STOP);
          analogWrite(L_Motor, ROTATE);
          break;

        case 'q':
          analogWrite(F_Motor, 200);
          analogWrite(B_Motor, STOP);
          analogWrite(L_Motor, ROTATE);
          analogWrite(R_Motor, STOP);
          break;

        case 'r':
          analogWrite(F_Motor, 120);
          analogWrite(B_Motor, STOP);
          analogWrite(R_Motor, ROTATE);
          analogWrite(L_Motor, STOP);
          break;

        case 'e':
          analogWrite(F_Motor, 120);
          analogWrite(B_Motor, STOP);
          analogWrite(R_Motor, ROTATE);
          analogWrite(L_Motor, STOP);
          break;

        case 's':
          analogWrite(F_Motor, STOP);
          analogWrite(B_Motor, 220);
          analogWrite(R_Motor, STOP);
          analogWrite(L_Motor, STOP);
          break;

        case 'a':
          digitalWrite(F_Motor, LOW);
          digitalWrite(B_Motor, HIGH);
          digitalWrite(R_Motor, LOW);
          digitalWrite(L_Motor, HIGH);
          break;

        case 'd':
          digitalWrite(F_Motor, LOW);
          digitalWrite(B_Motor, HIGH);
          digitalWrite(R_Motor, HIGH);
          digitalWrite(L_Motor, LOW);
          break;
          

        case 't':
          digitalWrite(F_Motor, LOW);
          digitalWrite(B_Motor, LOW);
          digitalWrite(L_Motor, LOW);
          digitalWrite(R_Motor, LOW);
          break;

        default:
          digitalWrite(F_Motor, LOW);
          digitalWrite(B_Motor, LOW);
          digitalWrite(L_Motor, LOW);
          digitalWrite(R_Motor, LOW);
          break;
      }
    } 
  }   
}