#ifndef _CameraPair_h
#define _CameraPair_h

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

const int width = 7;
const int height = 7;
const float squaresize1 = 0.024;
const float squareSize2 = 0.016;
const Point3f offset(0.25+0.3683,0.02,0.413);

const int nImages = 10;
enum {DETECTION, CAPTURING, RECTIFIED, PNPED, TRIANGULATING};
enum {CHESS,CIRCLES};
const Size chesssize(width,height);
const Size circlesize(4,11);

const string camera_name_1 = "../logitech.yml";
const string camera_name_2 = "../raspicam_internal.yml";
const string threshold_name_0 = "../threshold_0.yml";
const string threshold_name_1 = "../threshold_1.yml";
const string camerapair_file_name = "../camerapair.yml";
const string pnp_name = "../pnp.yml";

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
        void findScale(Mat& src1, Mat& src2);
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
        double m_scale;
        double m_scale_error;
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

#endif
