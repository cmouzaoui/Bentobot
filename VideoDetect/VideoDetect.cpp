#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <deque<Point> pts, Threshold main_thresh)
{
    if ( !src.data )
    {
        printf("No image data \n");
        return;
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
            pts.push_back(center);
        }
        if(pts.size() == VECLEN)
            pts.pop_front();
    }
    //draw history trail
    for (int i = 1; i < pts.size(); i++)
    {
        int thickness = int(sqrt(VECLEN/ float(i+1)) * 2.5);
        if(pts.size()>1)
        {
            line(src, pts[i], pts[i-1], Scalar(0,0,255), thickness);
        }
    }

}

void findCircle(Mat & src, Mat & gray)
{
    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
            gray.rows/16, // change this value to detect circles with different distances to each other
            100, 30, 1, 1000 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
            );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        circle( src, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
        circle( src, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
    }
}
