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

void cartToPolar(float x, float y, float z, float& phi, float& theta)
{
	float a, b, dx = 126.748, dz = 335.61, g = 9807, v = 4886.626; // all in mm
	
	theta = -atan(y/x);
    
    a = pow(v, 2) + sqrt(pow(v, 4) - g*(g*pow(x-dx, 2) + 2*(z-dz)*pow(v, 2))); // + or - for first sign?
    b = g*(x-dx);
    phi = atan(a/b);

	cout << "Phi: " << phi << endl;
	cout << "Theta: " << theta << endl;

}
