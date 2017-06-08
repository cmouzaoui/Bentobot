#include "CameraPair.h"

#include <numeric>
#include <math.h>

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
        c3["m_scale"] >> m_scale;
        c3["m_scale_error"] >> m_scale_error;
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
    c << "m_scale" << m_scale;
    c << "m_scale_error" << m_scale_error;
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

void CameraPair::findScale(Mat& src1, Mat& src2)
{
    vector<Point2f> corners1;
    vector<Point2f> corners2;
    if(!getcorners_single(src1, corners1,CIRCLES) || !getcorners_single(src2,corners2, CIRCLES)) 
        return;

    vector<Point3f> objectPoints = calcCorners(CIRCLES);
    
    vector<float> scale_vec;
    undistortPoints(corners1,corners1,m_camMat1,m_dist1);
    undistortPoints(corners2,corners2,m_camMat2,m_dist2);
    Mat homocoord;
    vector<Point3f> euclcoords;
    triangulatePoints(m_proj1, m_proj2, corners1, corners2, homocoord);
    convertPointsFromHomogeneous(homocoord.t(), euclcoords);
    for (int i = 0; i < objectPoints.size(); i++)
    {
    Mat object_mat(objectPoints[i]); //rectified coordinates

    object_mat.convertTo(object_mat,CV_64F);
    object_mat = m_pnp_r*object_mat + m_pnp_t;
    object_mat = m_r1*object_mat + m_t;
    Point3f objectPoint(object_mat);
    scale_vec.push_back(norm(objectPoint)/norm(euclcoords[i]));
    }
    double sum = accumulate(scale_vec.begin(),scale_vec.end(), 0.0);
    m_scale = sum/ scale_vec.size();
    cout << "Scale: " << m_scale << endl;
    double sq_sum = inner_product(scale_vec.begin(),scale_vec.end(), 
            scale_vec.begin(),0.0);
    m_scale_error = sqrt(sq_sum/scale_vec.size() - m_scale*m_scale);
    cout << "Scale stdev: " << m_scale_error/m_scale << endl;

}
bool CameraPair::pnp(Mat& src1)
{
    vector<Point2f> corners;
    if(!getcorners_single(src1, corners,CIRCLES)) 
        return false;

    vector<Point3f> objectPoints = calcCorners(CIRCLES);

    solvePnP(objectPoints, corners, m_camMat1, m_dist1, m_pnp_r, m_pnp_t);
    Rodrigues(m_pnp_r,m_pnp_r);
    cout << "Rotation matrix: " << endl << m_pnp_r << endl;
    cout << "Translation matrix " << endl << m_pnp_t << endl;

    m_mode = PNPED;
    save();
    return true;

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
    
    //Mat euclcoord_mat(Point3f(homocoord.at<float>(0,0),homocoord.at<float>(0,1),homocoord.at<float>(0,2)));
    Mat euclcoord_mat(euclcoord[0]); //rectified coordinates

    euclcoord_mat.convertTo(euclcoord_mat,CV_64F);
    euclcoord_mat = m_scale*euclcoord_mat; //scaled world coordinates
    euclcoord_mat = m_r1.inv()*(euclcoord_mat - m_t); //camera 1 coordinates
    euclcoord_mat = m_pnp_r.inv()*(euclcoord_mat - m_pnp_t); //world coordinates

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
                    corners.push_back(offset + Point3f(0,float(-(2*j + i % 2)*squareSize2),
                                float(-i*squareSize2)));
            break;
    }
    return corners;

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
