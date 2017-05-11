#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <cmath>
using namespace std;

const char * port = "/dev/ttyACM0";
int main()
{
	int fd;
	string m = "30\n";

	if ((fd = serialOpen(port, 9600)) < 0)
	{
		cout << "Unable to open serial device: " << port << endl;
		return 1;
	}

	serialPuts(fd, m);

	serialClose(fd);

	float phi = 0, theta = 0;
	cartToPolar(3, 4, 5, phi, theta); 

}

void cartToPolar(float x, float y, float x, float& phi, float& theta)
{
	float r1 = 0, r2 = 0, r3, phi_o = 0, a, b;
	
	theta = atan(y/x);
	r3 = sqrt(x^2 + y^2 + (z-r1)^2 -r2^2);
	a = atan(r2/r3);
	b = atan((z-r1)/(srqt(x^2 + y^2));
	phi = b - a + phi_o;

	cout << "Phi: " << phi << endl;
	cout << "Theta: " << theta << endl;

}