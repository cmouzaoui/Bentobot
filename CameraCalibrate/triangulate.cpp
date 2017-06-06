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

#include "threshold.h"

using namespace std;
using namespace cv;

//yml files for each camera, containing their intrinsic parameters 
//and their distortion coefficients
const string camera_name_1 = "../logitech.yml";
const string camera_name_2 = "../raspicam_internal.yml";
const string threshold_name_0 = "../threshold_0.yml";
const string threshold_name_1 = "../threshold_1.yml";
const string camerapair_file_name = "../camerapair.yml";
const string pnp_name = "../pnp.yml";

const int width = 7;
const int height = 7;
const float squaresize1 = 0.024;
const float squareSize2 = 0.016;
const Point3f offset(0.25+0.3683,0.02,0.413);

const int nImages = 10;
enum {DETECTION, CAPTURING, RECTIFIED, PNPED, TRIANGULATING};
enum {CHESS,CIRCLES};
const string mode_string[] = {"DETECTION","CAPTURING","RECTIFIED","PNPED", "TRIANGULATING"};
const int delay = 1000;
const Size chesssize(width,height);
const Size circlesize(4,11);

const int threshold_slider_max = 255;
const int threshold_slider_min = 0;

void getball(Mat& src1, Point2f& point2, Threshold main_thresh);
void printtext(Mat& src, string msg);
void thresh_load(FileStorage fs, Threshold& t);



class CameraPair
{
    public:
        CameraPair();
        void rectify(Size imgsize);
        Mat triangulate(Point2f point1, Point2f point2);
        bool getcorners(Mat& src1, Mat &src2);
        void save();
        int mode() {return m_mode;}
        bool pnp(Mat& src1);
        void capture() {m_mode = CAPTURING;};
        bool getcorners_single(Mat& src1, vector<Point2f> & corners, int patterntype);
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
        double m_error;
        //for rectify()

        //for pnp()
        Mat m_pnp_r;
        Mat m_pnp_t;
        //storing image points
        vector<vector<Point2f> > m_imagePoints1; 
        vector<vector<Point2f> > m_imagePoints2; 
        vector<Point3f> calcCorners(int patterntype);

        int m_mode;
};

CameraPair::CameraPair()
{
    FileStorage c3;

    if (c3.open(camerapair_file_name, FileStorage::READ))
    {
        cout << "Loading rectification data from " << camerapair_file_name << endl;
        c3["m_r"] >> m_r;
        c3["m_t"] >> m_t;
        c3["m_e"] >> m_e;
        c3["m_f"] >> m_f;
        c3["m_proj1"] >> m_proj1;
        c3["m_proj2"] >> m_proj2;
        c3["m_r1"] >> m_r1;
        c3["m_r2"] >> m_r2;
        m_mode = RECTIFIED;
        c3.release();
    }
    else 
    {
        m_mode = DETECTION;
    }
    if (c3.open(pnp_name, FileStorage::READ))
    {
        cout << "Loading pnp data from " << pnp_name << endl;
        c3["m_pnp_r"] >> m_pnp_r;
        c3["m_pnp_t"] >> m_pnp_t;
        m_mode = PNPED;
        c3.release();
    }
    FileStorage c1(camera_name_1, FileStorage::READ);
    FileStorage c2(camera_name_2, FileStorage::READ);
    c1["camera_matrix"] >> m_camMat1;
    c2["camera_matrix"] >> m_camMat2;
    c1["distortion_coefficients"] >> m_dist1;
    c2["distortion_coefficients"] >> m_dist2;
    c1.release();
    c2.release();
}

void CameraPair::save()
{
    if (m_mode == RECTIFIED || m_mode == PNPED)
    {
    FileStorage c(camerapair_file_name, FileStorage::WRITE);
    c << "m_t" << m_t;
    c << "m_r" << m_r;
    c << "m_e" << m_e;
    c << "m_f" << m_f;
    c << "m_proj1" << m_proj1;
    c << "m_proj2" << m_proj2;
    c << "m_r1" << m_r1;
    c << "m_r2" << m_r2;
    c << "m_error" << m_error;
    c.release();
    }
    if(m_mode == PNPED)
    {
    FileStorage c(pnp_name, FileStorage::WRITE);
    c << "m_pnp_r" << m_pnp_r;
    c << "m_pnp_t" << m_pnp_t;
    c.release();
    }
}

void CameraPair::rectify(Size imgsize)
{
    vector<vector<Point3f> > objectPoints;
    objectPoints.resize(nImages);
    for( int i = 0; i < nImages; i++) 
        objectPoints[i] = calcCorners(CIRCLES);

    m_error = stereoCalibrate(objectPoints, m_imagePoints1, m_imagePoints2,
            m_camMat1, m_dist1, m_camMat2, m_dist2, imgsize,
            m_r, m_t, m_e, m_f);
    //Debug output
    cout << "Rotation Matrix:" << endl << m_r << endl;
    cout << "Translation Matrix:" << endl << m_t << endl;
    cout << "Essential Matrix:" << endl << m_e << endl;
    cout << "Fundamental Matrix:" << endl << m_f << endl;
    cout << "Reprojection Error: " << m_error << endl;

    stereoRectify(m_camMat1, m_dist1, m_camMat2, m_dist2, imgsize,
            m_r, m_t, m_r1, m_r2, m_proj1, m_proj2, m_Q);
    cout << "Projection Marix 1:" << endl << m_proj1 << endl;
    cout << "Projection Matrix 2:" << endl << m_proj2 << endl;
    m_mode = PNPED;

}

bool CameraPair::pnp(Mat& src1)
{
    vector<Point2f> corners;
    if(getcorners_single(src1, corners,CHESS))
    {
        vector<Point3f> objectPoints;
        for( int j = 0; j < height; j++ )
            for( int k = 0; k < width; k++ )
                objectPoints.push_back(Point3f(offset.x,offset.y - float(k*squareSize2),
                            offset.z - float(j*squareSize2)));

        solvePnP(objectPoints, corners, m_camMat1, m_dist1, m_pnp_r, m_pnp_t);
        Rodrigues(m_pnp_r,m_pnp_r);
        cout << "Rotation matrix: " << endl << m_pnp_r << endl;
        cout << "Translation matrix " << endl << m_pnp_t << endl;
        m_pnp_r = m_pnp_r.inv();
        cout << "Inverted rotation matrix: " << endl << m_pnp_r << endl;
        m_mode = PNPED;
        save();
        return true;
    }
    return false;
}

Mat CameraPair::triangulate(Point2f point1, Point2f point2)
{
    Mat homocoord; //reconstructed point in homogenous coordinates
    vector<Point3f> euclcoord(1); //reconstructed point in euclidean coordinates
    vector<Point2f> points1(1);
    vector<Point2f> points2(1);
    points1[0] = point1;
    points2[0] = point2;
    undistortPoints(points1, points1,m_camMat1,m_dist1);
    undistortPoints(points2, points2,m_camMat2,m_dist2);
    cout << "Point 1: " << point1 << endl;
    cout << "Point 2: " << point2 << endl;
    triangulatePoints(m_proj1, m_proj2, points1, points2, homocoord);
    cout << "Homogeneous Coords: " << homocoord.t() << endl;
    convertPointsFromHomogeneous(homocoord.t(), euclcoord);
    Mat euclcoord_mat(euclcoord[0]);
    /*
    euclcoord_mat.convertTo(euclcoord_mat,CV_64F);
    euclcoord_mat = m_pnp_r*(euclcoord_mat - m_pnp_t);
    */
    cout << "Euclidean Coords: " << euclcoord_mat << endl;
    return euclcoord_mat;
}

vector<Point3f> CameraPair::calcCorners(int patterntype)
{
    vector<Point3f> corners;
    switch(patterntype)
    {
        case CHESS:
            for( int j = 0; j < height; j++ )
                for( int k = 0; k < width; k++ )
                    corners.push_back(offset + Point3f(0,float(-k*squaresize1),
                                float(-j*squaresize1)));
            break;
        case CIRCLES:
            for( int i = 0; i < circlesize.height; i++ )
                for( int j = 0; j < circlesize.width; j++ )
                    corners.push_back(Point3f(float((2*j + i % 2)*squareSize2),
                                float(i*squareSize2), 0));
            break;
    }
    return corners;

}

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

        if(camerapair.mode() == RECTIFIED)
        {
            camerapair.getcorners_single(src0, corners,CHESS);
        }

        if ((camerapair.mode() == RECTIFIED || camerapair.mode() == PNPED)
                    && c == 'p')
        {
            cout << "Solving PnP..." << endl;
            camerapair.pnp(src0_orig);
        }

        if (camerapair.mode() == PNPED)
        {
            getball(src0, point1, orange0);
            getball(src1, point2, orange1);
            if (c == 't')
            {
                current_point = camerapair.triangulate(point1, point2);
                msg = format("%0.3f,%0.3f,%0.3f",current_point.at<float>(0,0),
                        current_point.at<float>(0,1),
                        current_point.at<float>(0,2));
            }
        }

        printtext(src0, msg);
        imshow("Video Feed 0", src0);
        imshow("Video Feed 1", src1);
        if (c == 27) 
        {
            camerapair.save();
            break;
        }
        if (c == 'm')
        {
            cout << "Camera Mode is : " << mode_string[camerapair.mode()] << endl;
        }
    }

    return 0;
}



bool CameraPair::getcorners(Mat& src1, Mat &src2)
{
    vector<Point2f> points1, points2;
    if (getcorners_single(src1, points1,CIRCLES) && getcorners_single(src2, points2,CIRCLES))
    {
        m_imagePoints1.push_back(points1);
        m_imagePoints2.push_back(points2);
        return true;
    }
    return false;
}

bool CameraPair::getcorners_single(Mat& src1, vector<Point2f> & corners, int patterntype)
{
    Mat gray;
    cvtColor(src1,gray, COLOR_BGR2GRAY);

    clock_t timer = clock();
    bool patternfound;     
    switch(patterntype)
    {
        case CHESS:
            patternfound = findChessboardCorners(gray,chesssize,corners,
                    CALIB_CB_FAST_CHECK | CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
            if(patternfound)
            {
                cout << "found a chess pattern!" << endl;
                cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                        TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
                drawChessboardCorners(src1, chesssize, Mat(corners), patternfound);
            }
            break;
        case CIRCLES:
            patternfound = findCirclesGrid(gray, circlesize, corners,CALIB_CB_ASYMMETRIC_GRID);
            drawChessboardCorners(src1, circlesize, Mat(corners), patternfound);
            break;
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
