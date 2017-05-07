
   float angle = -90;
   const int M1_Dir = 5;
   const int M1_Steps = 4;
   const int M2_Dir = 7;
   const int M2_Steps = 6;
   float theta0 = 0, phi0 = 0;
   const int microsteps_per_step = 16;
   const int time_per_step = 2500/microsteps_per_step;

    // pin5 = direction of motor 1
    // pin4 = steps of motor 1
    // pin7 = direction of motor 2
    // pin6 = steps of motor 2

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
   demo();
}

void loop() {
  // put your main code here, to run repeatedly:
//  if (p1 > 0)
//  {
//      //motor 1
//      digitalWrite(M1_Dir, HIGH);      // turn on M1 Direction Pin
//
//      //motor 2
//      //digitalWrite(M2_Dir, HIGH);      // turn on M2 Direction Pin
//  }
//
//	// Steps for motor
//  for (int i = 0; i < abs(p1); i++)
//  {
//      // motor 1
//      digitalWrite(M1_Steps, HIGH);
//      //digitalWrite(M2_Steps, HIGH);
//      delayMicroseconds(time_per_step);      // turn on M1 Steps Pin
//      digitalWrite(M1_Steps, LOW);
//      //digitalWrite(M2_Steps, LOW);
//      delayMicroseconds(time_per_step);       // turn off M1 Steps Pin
//      //Serial.println("Sent a pulse to both motors.");
//   }

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
  moveTo(45, 0);
  delay(500);
  moveTo(-90,0);
  delay(500);
  moveTo(45,0);
  delay(500);
  moveTo(0, 20);
  delay(500);
  moveTo(0, -20);
  delay(500);  
  moveTo(-45, 30);
  delay(500);
  moveTo(45,-30);
  delay(500);
}
