#include <Servo.h>
 float angle = -90;
 
 //stepper motor
 const int M1_Steps = 4;  // pin4 = steps of motor 1
 const int M1_Dir = 5;    // pin5 = direction of motor 1
 const int M2_Steps = 6;  // pin6 = steps of motor 2
 const int M2_Dir = 7;    // pin7 = direction of motor 2

//servo(trigger) motor
Servo trigger;
const int interruptPin = 2;
const int servoPin = 10; //yellow signal wire to pin 10

 float theta0 = 0, phi0 = 0;
 const int microsteps_per_step = 16;
 const int time_per_step = 2500/microsteps_per_step;



void setup() {
  // put your setup code here, to run once:
     
   pinMode(M1_Dir, OUTPUT);
   pinMode(M1_Steps, OUTPUT);
   pinMode(M2_Dir, OUTPUT);
   pinMode(M2_Steps, OUTPUT);
   digitalWrite(M1_Dir, LOW);
   digitalWrite(M1_Steps, LOW);
   digitalWrite(M2_Dir, LOW);
   digitalWrite(M2_Steps, LOW);
   
   trigger.attach(servoPin);
   Serial.begin(9600);
   trigger.write(20);  //starting position so trigger is vertical
   delay(500);
   
   demo();
}

void loop() {

}

void move(float p1, float p2){

  int s1 = p1 * microsteps_per_step*400/360;    // convert angle to steps
  int s2 = p2 * microsteps_per_step*400/360;
  int steps, dSteps;
  
  if (p1 > 0){
    digitalWrite(M1_Dir, HIGH);      // turn on M1 Direction Pin
  }
  else{
     digitalWrite(M1_Dir, LOW);
  }
    
  
  if (p2 > 0){
    digitalWrite(M2_Dir, HIGH);      // turn on M2 Direction Pin
  }
  else{
     digitalWrite(M2_Dir, LOW);
  }
  
  dSteps = abs(s1-s2);
  
  if (abs(s1) >= abs(s2)){    // check for smaller steps
    steps = abs(s2);
  }
  else{
    steps = abs(s1);
  }
  
  for (int i = 0; i < steps; i++)
  {
      // motor 1
      digitalWrite(M1_Steps, HIGH);
      digitalWrite(M2_Steps, HIGH);
      delayMicroseconds(time_per_step);      // turn on M1 Steps Pin
      digitalWrite(M1_Steps, LOW);
      digitalWrite(M2_Steps, LOW);
      delayMicroseconds(time_per_step);       // turn off M1 Steps Pin
      //Serial.println("Sent a pulse to both motors.");
  }
  
  for (int j = 0; j < dSteps; j++)
  {
    if (abs(s1) > abs (s2))
    {
      digitalWrite(M1_Steps, HIGH);
      delayMicroseconds(time_per_step);      // turn on M1 Steps Pin
      digitalWrite(M1_Steps, LOW);
      delayMicroseconds(time_per_step);
    }
    else
    {
      digitalWrite(M2_Steps, HIGH);
      delayMicroseconds(time_per_step);      // turn on M1 Steps Pin
      digitalWrite(M2_Steps, LOW);
      delayMicroseconds(time_per_step);
    }
  }


}

void moveTo(float theta, float phi){
  move(theta-theta0, phi-phi0);
}

void demo(){
//  trigger.write(55);
//  delay(500);
//  trigger.write(20);
//  delay(500);
  moveTo(45, 0);
  delay(500);
  moveTo(-90,0);
  delay(500);
  moveTo(45,0);
  delay(500);
  moveTo(0, -40);
  delay(500);
  moveTo(0, 40);
  delay(500);  
  moveTo(-45,-30);
  delay(500);
  moveTo(45, 30);
  delay(500);
  trigger.write(55);
  delay(500);
  trigger.write(20);
  delay(500);
}
