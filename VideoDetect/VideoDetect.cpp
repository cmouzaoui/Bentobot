#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <deque>
#include <cmath>

using namespace std;
using namespace cv;

const int orange_min [3] = {9,26,228};
const int orange_max [3] = {42,244,255};
const int blue_min [3] = {97,54,89};
const int blue_max [3] = {127,180,189};

const int threshold_slider_max = 255;
const int threshold_slider_min = 0;

struct Threshold
{
	int min[3];
	int max[3];
};

const Threshold orange = {.min = {9,26,228},
							.max = {42,244,255}};

const int veclen = 32;
int main(int argc, char** argv)
{
	/*
	if (argc != 2)
	{
			printf("usage: DisplayImage <Image_Path>\n");
			return -1;
	}
	*/
	// Initialize Videocapture
	VideoCapture cap(0);

	Mat src, dst, hsv;
	deque<Point> pts; 
	//src = imread( argv[1], CV_LOAD_IMAGE_COLOR);
	
	//create instance of threshold
	//Threshold main_thresh = { .min={0,0,0}, .max={255,255,255}};
	Threshold main_thresh = orange;

	//create window for trackbar
	namedWindow("Threshold", 1);

	createTrackbar("H min", "Threshold",&main_thresh.min[0],threshold_slider_max);
	createTrackbar("S min", "Threshold",&main_thresh.min[1],threshold_slider_max);
	createTrackbar("V min", "Threshold",&main_thresh.min[2],threshold_slider_max);
	createTrackbar("H max", "Threshold",&main_thresh.max[0],threshold_slider_max);  
	createTrackbar("S max", "Threshold",&main_thresh.max[1],threshold_slider_max);  
	createTrackbar("V max", "Threshold",&main_thresh.max[2],threshold_slider_max);  

	//begin loop
	while(true)
	{
	cap >> src;

	if ( !src.data )
	{
			printf("No image data \n");
			return -1;
	}

	//convert source image to HSV colorspace
	cvtColor(src,hsv, COLOR_BGR2HSV);

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
			for(int i = 0; i < cnts.size(); i++)
			{
					Point2f xy;
					float radius;
					Mat c = cnts[i];
					minEnclosingCircle(c, xy, radius);
					Moments M = moments(c);
					Point center = cvPoint(int(M.m10/M.m00), int(M.m01/M.m00));
					if (radius > 10)
					{
							circle(src, xy, int(radius), Scalar(0,255,255), 2);
							circle(src, center, 5, Scalar(0,0,255), -1);
					}
					if(pts.size() == veclen)
						pts.pop_front();
					pts.push_back(center);
			}
	}
	for (int i = 0; i < pts.size(); i++)
	{
			int thickness = int(sqrt(veclen/ float(i+1)) * 2.5);
			if(pts.size()>1)
			{
			line(src, pts.back(), pts[pts.size()-2], Scalar(0,0,255), thickness);
			}
	}
	

	imshow("Display Image", src);
	imshow("Filtered IMage", dst);

	//close program when escape key is pressed
	int key = waitKey(1);
	if (key == 27) break;
	}

	return 0;
}
