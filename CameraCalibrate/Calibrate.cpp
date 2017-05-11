#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;



//function primitives
void calibrate(Mat& src0);

int main(int argc, char** argv)
{
	// Initialize Videocapture
	VideoCapture cap0(1);
//	VideoCapture cap1(1);

	Mat src0, dst0, hsv0;


//	Mat src1, dst1, hsv1; 

	while(true)
	{
	cap0 >> src0;

	//wait for the "start collecting" button to be pressed
	int key = waitKey(1);
	int counter = 101;

	if (key == 's')
	{
			counter = 0;
			cout << "Started Calibration" << endl;
	}

			calibrate(src0);
	imshow("Video Feed 1", src0);
	}
	//begin loop
	/*
	while(true)
	{
	cap1 >> src1;

//	findInRange(src0,dst0,hsv0,pts0, main_thresh);
//	findInRange(src1,dst1,hsv1,pts1, main_thresh);
	findCircle(src0,dst0);

	imshow("Display Image 0", src0);
	imshow("Filtered Image 0", dst0);
//	imshow("Display Image 1", src1);
//	imshow("Filtered Image 1", dst1);

	//close program when escape key is pressed
	int key = waitKey(1);
	if (key == 27) break;
	}
	*/

	return 0;
}


void calibrate(Mat& src0)
{
		Mat gray;
		cvtColor(src0,gray, COLOR_BGR2GRAY);
		vector<Point2f> corners;
		Size patternsize(7,7);
		bool patternfound = findChessboardCorners(gray,patternsize,corners,
						CV_CALIB_CB_NORMALIZE_IMAGE+CV_CALIB_CB_FILTER_QUADS+CALIB_CB_FAST_CHECK);
		if(patternfound)
		{
				cout << "found a chessboard pattern!" << endl;
				cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
						   TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
				drawChessboardCorners(src0, patternsize, Mat(corners), patternfound);
		}

}
