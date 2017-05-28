#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <time.h>

#include "threshold.h"

using namespace std;
using namespace cv;

//yml files for each camera, containing their intrinsic parameters 
//and their distortion coefficients
const string camera_name_1 = "../comradey_internal.yml";
const string camera_name_2 = "../logitech.yml";

const int width = 7;
const int height = 7;
const float squareSize = 0.024;

const int nImages = 3;
enum {DETECTION, CAPTURING, RECTIFIED, TRIANGULATING};
const int delay = 1000;
const Size patternsize(width,height);

const int threshold_slider_max = 255;
const int threshold_slider_min = 0;

bool getcorners(Mat& src1, vector<Point2f>& imagepoints2);
void getball(Mat& src1, Point2f& point2, Threshold main_thresh);
void printtext(Mat& src, int captured);



class CameraPair
{
    public:
        CameraPair();
        void rectify(vector<vector<Point2f> >& imagepoints1,
                vector<vector<Point2f> >& imagepoints2, Size imgsize);
        Point3f triangulate(Point2f point1, Point2f point2);
    private:
        //obtained from sample calibration program Calibration_sample
        Mat m_camMat1; //intrinsic parameters of camera 1
        Mat m_dist1; //distortion coefficients of camera 1
        Mat m_camMat2; //intrinsic parameters of camera 2
        Mat m_dist2; //distortion coefficients of camera 2
        //initialized with rectify()

        //from cv::StereoCalibrate()
        Mat m_r; //rotation matrix btwn camera coord systems
        Mat m_t; //translation matrix btwn camera coord systems
        Mat m_e; //essential matrix
        Mat m_f; //fundamental matrix
        //from cv::StereoRectify()
        Mat m_proj1; //projection matrix of camera 1
        Mat m_proj2; //projection matrix of camera 2
        Mat m_r1; //rotation matrix for camera 1
        Mat m_r2; //rotation matrix for camera 2
        Mat m_Q; //disparity-to-depth mapping matrix
        //for rectify()
        CameraPair::vector<vector<Point3f> > calcCorners();
};

CameraPair::CameraPair()
{
    FileStorage c1(camera_name_1, FileStorage::READ);
    FileStorage c2(camera_name_2, FileStorage::READ);
    c1["camera_matrix"] >> m_camMat1;
    c2["camera_matrix"] >> m_camMat2;
    c1["distortion_coefficients"] >> m_dist1;
    c2["distortion_coefficients"] >> m_dist2;
    c1.release();
    c2.release();

}

void CameraPair::rectify(vector<vector<Point2f> >& imagepoints1,
        vector<vector<Point2f> >& imagepoints2, Size imgsize)
{
    vector<vector<Point3f> > objectPoints = calcCorners();
    stereoCalibrate(objectPoints, imagepoints1, imagepoints2,
            m_camMat1, m_dist1, m_camMat2, m_dist2, imgsize,
            m_r, m_t, m_e, m_f);
    //Debug output
    cout << "Rotation Matrix:" << endl << m_r << endl;
    cout << "Translation Matrix:" << endl << m_t << endl;
    cout << "Essential Matrix:" << endl << m_e << endl;
    cout << "Fundamental Matrix:" << endl << m_f << endl;

    stereoRectify(m_camMat1, m_dist1, m_camMat2, m_dist2, imgsize,
            m_r, m_t, m_r1, m_r2, m_proj1, m_proj2, m_Q);
    cout << "Projection Marix 1:" << endl << m_proj1 << endl;
    cout << "Projection Matrix 2:" << endl << m_proj2 << endl;

}

Point3f CameraPair::triangulate(Point2f point1, Point2f point2)
{
    Mat homocoord; //reconstructed point in homogenous coordinates
    vector<Point3f> euclcoord(1); //reconstructed point in euclidean coordinates
    vector<Point2f> points1(1);
    vector<Point2f> points2(1);
    points1[0] = point1;
    points2[0] = point2;
    cout << "Point 1: " << point1 << endl;
    cout << "Point 2: " << point2 << endl;
    triangulatePoints(m_proj1, m_proj2, points1, points2, homocoord);
    cout << "Homogeneous Coords: " << homocoord.t() << endl;
    convertPointsFromHomogeneous(homocoord.t(), euclcoord);
    cout << "Euclidean Coords: " << euclcoord[0] << endl;
    return euclcoord[0];
}

CameraPair::vector<vector<Point3f> > calcCorners()
{
    vector<vector<Point3f> > corners;
    corners.resize(nImages);
    for( int i = 0; i < nImages; i++ )
    {

    for( int j = 0; j < height; j++ )
        for( int k = 0; k < width; k++ )
            corners[i].push_back(Point3f(float(k*squareSize),
                        float(j*squareSize), 0));
    }
    return corners;

}

int main(/*int argc, char** argv*/)
{
    // Initialize Videocapture
    VideoCapture cap0(0);
    VideoCapture cap1(1);

    Mat src0, src1;
    CameraPair camerapair;

    //create window for trackbar
    /*
    namedWindow("Threshold", 1);

    createTrackbar("H min", "Threshold",&main_thresh.min[0],threshold_slider_max);
    createTrackbar("S min", "Threshold",&main_thresh.min[1],threshold_slider_max);
    createTrackbar("V min", "Threshold",&main_thresh.min[2],threshold_slider_max);
    createTrackbar("H max", "Threshold",&main_thresh.max[0],threshold_slider_max);  
    createTrackbar("S max", "Threshold",&main_thresh.max[1],threshold_slider_max);  
    createTrackbar("V max", "Threshold",&main_thresh.max[2],threshold_slider_max);  
    */

    char c;
    int captured = 0;
    vector<vector<Point2f> > points1;
    vector<vector<Point2f> > points2;
    Point2f point1;
    Point2f point2;
    clock_t prevtimestamp = 0;
    bool found = false;
    int mode = DETECTION;
    bool blink = false;

    while(true)
    {
        blink = false;
        c = waitKey(50);

        if (mode == DETECTION && c == 'g') 
        {
            mode = CAPTURING;
            cout << "Started Calibration" << endl;
        }

        cap0 >> src0;
        cap1 >> src1;
        resize(src1,src1, src0.size());

        if (mode == CAPTURING && clock() - prevtimestamp > delay*1e-3*CLOCKS_PER_SEC)
        {
            vector<Point2f> corners1, corners2;
            if(camerapair.getcorners(src0,src1))
            {
                points1.push_back(corners1);
                points2.push_back(corners2);
                prevtimestamp = clock();
                captured++;
                blink = true;
                if (captured >= nImages)
                {
                    mode = RECTIFIED;
                    camerapair.rectify(points1, points2,src0.size());
                }
            }
        }

        if( blink )
        {
            bitwise_not(src0, src0);
            bitwise_not(src1, src1);
        }

        if (mode == RECTIFIED)
        {
            getball(src0, point1, ORANGE3);
            getball(src1, point2, ORANGE2);
            if (c == 't')
                camerapair.triangulate(point1, point2);
        }

        printtext(src0, captured);
        imshow("Video Feed 0", src0);
        imshow("Video Feed 1", src1);
        if (c == 27) break;
    }

    return 0;
}



bool getcorners(Mat& src, vector<Point2f> & corners) 
{
    Mat gray;
    cvtColor(src,gray, COLOR_BGR2GRAY);

    bool patternfound = findChessboardCorners(gray,patternsize,corners,
            CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE+CV_CALIB_CB_FILTER_QUADS+CALIB_CB_FAST_CHECK);
    if(patternfound)
    {
        cout << "found a chessboard pattern!" << endl;
        cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
        drawChessboardCorners(src, patternsize, Mat(corners), patternfound);
    }
    return patternfound;
}

void getball(Mat& src, Point2f& point, Threshold main_thresh)
{

    Mat hsv, dst;
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

}

void printtext(Mat& src, int captured)
{
    string msg = format("%d/%d ", captured, nImages);
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(src.cols - 2*textSize.width - 10, src.rows - 2*baseLine - 10);

        putText( src, msg, textOrigin, 1, 1,
                 Scalar(0,0,255));
}
