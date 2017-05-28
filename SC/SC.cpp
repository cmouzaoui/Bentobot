#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <cmath>
#include <stdio.h>
#include <unistd.h>

using namespace std;

const int BUFLEN = 50;
//function primitives
void cartToPolar(float x, float y, float z, float& phi, float& theta);
void  anglesdemo(char * command);

const char * port = "/dev/ttyACM0";
int main()
{
	int fd;
	char m [BUFLEN];
//        anglesdemo(m);

	if ((fd = serialOpen(port, 9600)) < 0)
	{
		cout << "Unable to open serial device: " << port << endl;
	//	return 1;
	}
        float x,y,z,theta,phi;
        cout << "Test x coordinate:" << endl;
        cin >> x;
        cout << "Test y coordinate:" << endl;
        cin >> y;
        cout << "Test z coordinate:" << endl;
        cin >> z;

	cartToPolar(x, y, z, phi, theta); 
        sprintf(m,"%f,%f\ns\n",theta,phi);
        cout << "Sending over the following message: " << m << endl;
	serialPuts(fd, m);

	serialClose(fd);


}

void anglesdemo(char * command)
{
    sprintf(command,"%d,%d\n"
            "%d,%d\n"
            "%d,%d\n"
            "%d,%d\n"
            "%d,%d\n"
            "%d,%d\n"
            "%d,%d\n"
            "s\n",0,0,
            90,0,
            0,0,
            -60,0,
            0,40,
            0,0,
            30,10); 
}

void cartToPolar(float x, float y, float z, float& phi, float& theta)
{
    float r1 = 240, r2 = 24.5, r3, phi_o = 48.76, a, b;

    if (x == 0)
    {
        theta = 0;
    }
    else
    {
        theta = -atan(y/x);
    }

    if ( x == 0 && y == 0)
    {
        phi = M_PI/2;
    }
    else
    {
        r3 = sqrt(pow(x,2) + pow(y,2) + pow(z-r1,2) -pow(r2,2));
        a = atan(r2/r3);
        b = atan((z-r1)/(sqrt(pow(x,2) + pow(y,2))));
        phi = b - a;
    }

    theta = theta*180/(M_PI);
    phi = phi*180/(M_PI);
    cout << "Phi: " << phi << endl;
    cout << "Theta: " << theta << endl;
}
