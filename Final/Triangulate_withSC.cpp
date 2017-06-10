#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <raspicam/raspicam_cv.h>
#include <cmath>

#include <wiringPi.h>
#include <wiringSerial.h>
#include "threshold.h"
#include "CameraPair.h"

using namespace std;
using namespace cv;

//yml files for each camera, containing their intrinsic parameters 
//and their distortion coefficients
const string mode_string[] = {"DETECTION","CAPTURING","RECTIFIED","PNPED", "TRIANGULATING"};

const int threshold_slider_max = 255;
const int threshold_slider_min = 0;

const int DELAY = 1000;
const int BUFLEN = 50;
const char * port = "/dev/ttyACM0";

bool getball(Mat& src1, Point2f& point2, Threshold main_thresh);
void printtext(Mat& src, string msg);
void thresh_load(FileStorage fs, Threshold& t);

void cartToPolar(float x, float y, float z, float& phi, float& theta);


int main(/*int argc, char** argv*/)
{
    //Load thresholds
    Threshold orange0;
    Threshold orange1;
    FileStorage t0(threshold_name_0, FileStorage::READ);
    FileStorage t1(threshold_name_1, FileStorage::READ);
    thresh_load(t0,orange0);
    thresh_load(t1,orange1);
    t0.release();
    t1.release();
    // Initialize Videocapture
    VideoCapture cap0(0);
    raspicam::RaspiCam_Cv cap1;
    cap1.set ( CV_CAP_PROP_FRAME_WIDTH, 640);
    cap1.set ( CV_CAP_PROP_FRAME_HEIGHT, 480);
    cout<<"Opening cap1..."<<endl;
    sleep(1);
    if (!cap1.open()) {cerr<<"Error opening the camera"<<endl;return -1;}

    int fd;
    if ((fd = serialOpen(port, 9600)) < 0)
    {
        cout << "Unable to open serial device: " << port << endl;
        //	return 1;
    }


    Mat src0, src1, src0_orig;
    CameraPair camerapair;

    char c;
    int captured = 0;
    vector<Point2f> corners(width*height);
    Point2f point1;
    Point2f point2;
    clock_t prevtimestamp = 0;
    bool found = false;
    bool blink = false;
    string msg;
    Mat current_point;
    
    char m [BUFLEN];
    float phi, theta;

    Point2f reprojection(0,0);
    while(true)
    {
        blink = false;
        c = waitKey(50);

        if (c == 'g') 
        {
            camerapair.capture();
            cout << "Started Calibration" << endl;
        }

        cap0 >> src0;
        src0_orig = src0.clone();
        cap1.grab();
        cap1.retrieve (src1);

        resize(src1,src1, src0.size());

        if (camerapair.mode() == CAPTURING && c == 'c') //clock() - prevtimestamp > delay*1e-3*CLOCKS_PER_SEC)
        {
            if(camerapair.getcorners(src0,src1))
            {
                prevtimestamp = clock();
                captured++;
                blink = true;
                if (captured >= nImages)
                {
                    camerapair.rectify(src0.size());
                }
            }

            msg = format("%d/%d ", captured, nImages);
        }

        if( blink )
        {
            bitwise_not(src0, src0);
            bitwise_not(src1, src1);
        }


        if ((camerapair.mode() == RECTIFIED || camerapair.mode() == PNPED)
                && c == 'p')
        {
            camerapair.pnp(src0_orig);
        }

        if (camerapair.mode() == PNPED)
        {
            if (c == 't' && 
                getball(src0, point1, orange0) &&
                getball(src1, point2, orange1))
                {
                current_point = camerapair.triangulate(point1, point2, reprojection);

                msg = format("%0.3f,%0.3f,%0.3f",current_point.at<double>(0,0),
                        current_point.at<double>(0,1),
                        current_point.at<double>(0,2));
                cartToPolar(current_point.at<double>(0,0), current_point.at<double>(0,1), current_point.at<double>(0,2), phi, theta);
                sprintf(m,"%f,%f\ns\n",theta,phi);
                cout << "Sending over the following message: " << m << endl;
                serialPuts(fd, m);
            }
            if (c == 'f')
                camerapair.findScale(src0,src1);

            if (c == 's')
            {
                serialPuts(fd, "s\n");
            }
        }

        circle(src0,reprojection,5, Scalar(0,0,255), -1);
        printtext(src0, msg);
        imshow("Video Feed 0", src0);
        imshow("Video Feed 1", src1);
        if (c == 27) 
        {
            camerapair.save();
	    serialClose(fd);
            break;
        }
        if (c == 'm')
        {
            cout << "Camera Mode is : " << mode_string[camerapair.mode()] << endl;
        }
    }

    serialClose(fd);
    return 0;
}

bool getball(Mat& src, Point2f& point, Threshold main_thresh)
{

    Mat hsv, dst;
    //convert source image to HSV colorspace
    cvtColor(src,hsv, COLOR_BGR2HSV);
/*
    //apply color range
    inRange(hsv, Scalar(main_thresh.min[0],
                main_thresh.min[1],
                main_thresh.min[2]),
            Scalar(main_thresh.max[0],
                main_thresh.max[1],
                main_thresh.max[2]),
            dst);

    //erode and dilate for faster processing
    erode(dst, dst,Mat(),Point(-1,-1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue());
    dilate(dst, dst,Mat(), Point(-1,-1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue());

    //find contours
    vector<Mat> cnts;
    findContours(dst.clone(), cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if (cnts.size() > 0)
    {
        //find the contour with max area
        Mat c = cnts[0];
        for(int i = 0; i < cnts.size(); i++)
        {
            if (contourArea(c) < contourArea(cnts[i]))
            {
                c = cnts[i];
            }
        }

        Point2f xy;
        float radius;

        //find the minimum enclosing circle around that contour
        minEnclosingCircle(c, xy, radius);
        //place the center at the centroid of the contour
        Moments M = moments(c);
        Point center = cvPoint(int(M.m10/M.m00), int(M.m01/M.m00));
        //draw the enclosing circle and centroid
        if (radius > 10)
        {
            circle(src, xy, int(radius), Scalar(0,255,255), 2);
            circle(src, center, 5, Scalar(0,0,255), -1);
            point = center;
        }
    }

    */
    vector<Point2f> points;
    bool patternfound = findCirclesGrid(src, circlesize, points, CALIB_CB_ASYMMETRIC_GRID);
    if(patternfound)
    {
        drawChessboardCorners(src, circlesize, Mat(points), patternfound);
        point = points[22];
    }
    return patternfound;
    
}

void printtext(Mat& src, string msg)
{
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(src.cols - 2*textSize.width - 10, src.rows - 2*baseLine - 10);

        putText( src, msg, textOrigin, 1, 1,
                 Scalar(0,0,255));
}


void thresh_load(FileStorage fs, Threshold& t)
{
    if (!fs.isOpened())
    {
        cerr << "Failed to open threshold file" << endl;
        return;
    }
    fs["min1"] >> t.min[0];
    fs["min2"] >> t.min[1];
    fs["min3"] >> t.min[2];
    fs["max1"] >> t.max[0];
    fs["max2"] >> t.max[1];
    fs["max3"] >> t.max[2];

}

void cartToPolar(float x, float y, float z, float& phi, float& theta)
{
    float a, b, dx = 0.126748, dz = 0.33561, g = 9.807, v = 6.431; // all in m

    x = sqrt(pow(x-dx,2) + pow(y,2));
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
        a = pow(v, 2) - sqrt(pow(v, 4) - g*(g*pow(x, 2) + 2*(z-dz)*pow(v, 2))); // + or - for first sign?
        b = g*(x-dx);
        phi = atan(a/b);
    }

    cout << "Phi: " << phi << endl;
    cout << "Theta: " << theta << endl;

    theta = theta*180/(M_PI);
    phi = phi*180/(M_PI);
    cout << "Phi: " << phi << endl;
    cout << "Theta: " << theta << endl;
}

