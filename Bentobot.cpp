#include <iostream>
#include <wiringPi.h>
using namespace std;

void MotorControl(int p1);

int main()
{
	int pulse1;
	cout << "Input value for motor 1: ";
	cin >> pulse1;

	MotorControl(pulse1);
}

void MotorControl(int p1)
{
	// Control the motors
	// pin1 = direction of motor 1
	// pin2 = steps of motor 1
	// pin3 = direction of motor 2
	// pin4 = steps of motor 2
    wiringPiSetup();
    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
	bool isP1On = false;

	// Directions for motors
	// assume that pin 1/pin 3 on is cw
	if (p1 > 0 && !isP1On)
	{
		//turn on pin 1
        digitalWrite(1, HIGH);      // turn on pin1
        // turn on pin 3
        digitalWrite(3, HIGH);      // turn on pin3
        isP1On = true;
    }
    else if (p1 < 0 && isP1On)
    {
        digitalWrite(1, HIGH);
        digitalWrite(3, HIGH);
        isP1On = false;
    }

	// Steps for motor
	for (int i = 0; i < abs(p1); i++)
	{
		// motor 1
        digitalWrite(2, HIGH); delay(2.5);      // turn on pin2
        digitalWrite(2, LOW); delay(2.5);       // turn off pin2
		// motor 2
        digitalWrite(4, HIGH); delay(2.5);      // turn on pin4
        digitalWrite(4, LOW); delay(2.5);       // turn off pin4
	}

}
