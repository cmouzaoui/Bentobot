#include <Servo.h>
  
//stepper motor
const int M1_Steps = 4;  // pin4 = steps of motor 1
const int M1_Dir = 5;    // pin5 = direction of motor 1
const int M2_Steps = 6;  // pin6 = steps of motor 2
const int M2_Dir = 7;    // pin7 = direction of motor 2

//servo(trigger) motor
Servo trigger;
const int servoPin = 10; //yellow signal wire to pin 10

float theta0 = -93, phi0 = 48.76;  //48.76 is when launcher is resting backwards, 3.88 is forward
float theta_now = theta0;
float phi_now = phi0;
const int microsteps_per_step = 32;
const int time_per_step = 5000/microsteps_per_step;
 
const int vertical_trigger = 85;  //18
const int turned_trigger = 110;  //60

String main_string;
 

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
   trigger.write(vertical_trigger);  //starting position so trigger is vertical //18
   delay(500);
   
   moveTo(0, 48.76);
//   Serial.println("setup");
//   demo();
}

void loop() {

  //look for the next valid integer in the incoming serial stream
  while (Serial.available()>0){
    char inChar = Serial.read();
    //if (inChar !='\n')
    //{
      main_string += inChar;
    //}
  }
  if (main_string[main_string.length()-1] == '\n')
  {
    Serial.println(main_string);
    int newline_index = main_string.indexOf('\n');
    if (main_string == "s\n")
    {
      shoot();
    }
    else
    {
    int comma_index = main_string.indexOf(',');
    Serial.print("Comma index: ");
    Serial.println(comma_index);
    float theta = main_string.substring(0,comma_index).toFloat();
    float phi = main_string.substring(comma_index+1).toFloat();
    moveTo(theta,phi);
    }
    main_string = main_string.substring(newline_index+1);
  }
    /*
    Serial.print("enter");
    float theta = Serial.parseFloat();  //theta value
    float phi = Serial.parseFloat();  //phi value
    Serial.print(", almost there: ");
    //look for the newline
    char c = Serial.read();
    Serial.print(c);
    if (c == '\n') {
      moveTo(theta,phi);
      Serial.print(theta); Serial.print(phi);      
    }
    else if (c == 's' && Serial.read()=='\n') {
      shoot();
    }
    */
}

void move(float p1, float p2){

  int s1 = p1 * microsteps_per_step*400/360;    // convert angle to steps
  int s2 = p2 * microsteps_per_step*400/360;
  int steps, dSteps;
  Serial.println(p1);
  Serial.println(p2);
      
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
        delayMicroseconds(time_per_step/2);      // turn on M1 Steps Pin
        digitalWrite(M1_Steps, LOW);
        digitalWrite(M2_Steps, LOW);
        delayMicroseconds(time_per_step/2);       // turn off M1 Steps Pin
        //Serial.println("Sent a pulse to both motors.");
    }
    
    for (int j = 0; j < dSteps; j++)
    {
      if (abs(s1) > abs (s2))
      {
        digitalWrite(M1_Steps, HIGH);
        delayMicroseconds(time_per_step/2);      // turn on M1 Steps Pin
        digitalWrite(M1_Steps, LOW);
        delayMicroseconds(time_per_step/2);
      }
      else
      {
        digitalWrite(M2_Steps, HIGH);
        delayMicroseconds(time_per_step/2);      // turn on M1 Steps Pin
        digitalWrite(M2_Steps, LOW);
        delayMicroseconds(time_per_step/2);
      }
    }
    Serial.print("Steps: ");
    Serial.print(s1);
    Serial.print(",");
    Serial.println(s2);
}

void moveTo(float theta, float phi){
  
  if (abs(theta)>60 && (phi < 3.88 || phi >48.76))  //both angles out of range
  {
    Serial.println("InVaLid AnGLeS"); 
  }
  else if (abs(theta) > 60 && phi>=3.8 && phi<=48.76)  // theta out of range, phi good
  {
    Serial.println("invalid theta");
    move(0, phi-phi_now);
    phi_now = phi;  //only change phi
  }
  else if ((phi > 48.76 || phi < 3.88) && abs(theta)<=60)  // phi out of range, theta good
  {
    Serial.println("invalid phi");
    move(theta-theta_now, 0);
    theta_now = theta;  //only change theta
  }
  else {
    move(theta-theta_now, phi-phi_now); // -60<theta<60 and 3.88<phi<48.76
    theta_now = theta;
    phi_now = phi;
  }
  Serial.print("Moved to: "); Serial.print(theta_now); Serial.print(","); Serial.println(phi_now);  

}

void shoot(){
  Serial.println("Fire in the hole!");
  delay(100);
  trigger.write(turned_trigger);
  delay(500);
  trigger.write(vertical_trigger);
  delay(500);
}

void demo(){

  moveTo(45, 0);
  delay(500);
  moveTo(-45,0);
  delay(500);
  moveTo(45,0);
  delay(500);
  moveTo(0, 10);
  delay(500);
  moveTo(0, 45);
  delay(500);  
  moveTo(-45,30);
  delay(500);
  moveTo(0, 45);
  delay(500);
  shoot();
}
