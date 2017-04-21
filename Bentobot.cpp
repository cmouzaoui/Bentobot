#include <iostream>
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
	bool isP1On = false;

	// motor 1
	// assume that pin 1/pin 3 on is cw
	if (p1 > 0) //to check if the direction is positive
	{
		//turn on pin 1
		cout << "Turn on pin1" << endl;
		// turn on pin 3
		cout << "Turn on pin3" << endl;
	}

	// steps for motor
	for (int i = 0; i < abs(p1); i++)
	{
		// turn on pin2
		cout << "Turn on pin2" << endl;
		// turn off pin2
		cout << "Turn off pin2" << endl;
		// turn on pin4
		cout << "Turn on pin4" << endl;
		// turn off pin4
		cout << "Turn off pin4" << endl;
	}

}