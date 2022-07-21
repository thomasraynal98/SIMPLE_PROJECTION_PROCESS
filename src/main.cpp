#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

struct Position{
    double x, y, r;
    Position(double a, double b, double c)
        : x(a)
        , y(b)
        , r(c)
        {}
};

struct Point{
    int i, j;
    Point(int a, int b)
        : i(a)
        , j(b)
        {}
};

struct Data{
    double raw, distance;
    Data(double a, double b)
        : raw(a)
        , distance(b)
        {}
};

struct Sensor{
    double raw, distance, orientation;
    int id;
    Sensor(double a, double b, int c, double d)
        : raw(a)
        , distance(b)
        , id(c)
        , orientation(d)
        {}
};

long double toRadians(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

double distance_in_pixel_from_point(Position A, Position B, int resolution_pixel)
{
    // resolution_pixel = number of pixel in 1 meter.
    return sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)) * resolution_pixel;
}

double process_angle(double idx_c1, double idx_r1, double idx_c2, double idx_r2)
{
    // Description : Process angle between camera position and new data detection.
    return atan2(idx_r2 - idx_r1, idx_c2 - idx_c1);
}

int main()
{
    Point centre(50, 50);
    int nb_col = 100;
    int nb_row = 100;
    cv::Mat environnement = cv::Mat(nb_row, nb_col, CV_8UC3, cv::Scalar(255, 255, 255));

    Position p1(0.0, 0.0, 0.0);
    Position p2(2.5, 0.0, 50.0);

    Data d1(0.0, 0.5);
    Data d2(90.0, 0.8);
    std::vector<Data> vectd;
    vectd.push_back(d1);
    vectd.push_back(d2);

    Sensor cam_ava(0.0, 0.5, 0, 0.0);
    Sensor cam_arr(90.0, 0.5, 1, 90.0);
    std::vector<Sensor> vect;
    vect.push_back(cam_ava);
    vect.push_back(cam_arr);

    // 1 GET CENTER OF ROBOT BEFORE
    double distance1 = distance_in_pixel_from_point(p1, p2, 10);
    double angle1    = process_angle(p2.y, p2.x, p1.y, p1.x);
    double diff_angl = (toRadians(p2.r) - toRadians(p1.r));

    double idx_col_1    = cos(angle1 + diff_angl) * distance1 + centre.i;
    double idx_row_2    = sin(angle1 + diff_angl) * distance1 + centre.j;

    cv::circle(environnement, cv::Point((int)(idx_col_1),(int)(idx_row_2)),1, cv::Scalar(0,0,150), cv::FILLED, 0,0);

    // // 2 GET POSITION OF SENSOR OF ROBOT BEFORE
    double distance2;
    double angle2;
    double idx_col_3; 
    double idx_row_3;

    for(int i = 0; i < vect.size(); i++)
    {
        distance2 = vect[i].distance * 10;
        angle2    = toRadians(vect[i].raw);
        double angle2b   = process_angle(idx_col_1, idx_row_2, centre.i, centre.j);

        std::cout << angle2 << std::endl;

        idx_col_3 = cos(angle2b - angle2) * distance2 + idx_col_1;
        idx_row_3 = sin(angle2b - angle2) * distance2 + idx_row_2;

        if(vect[i].id == 0)
        {
            cv::circle(environnement, cv::Point((int)(idx_col_3),(int)(idx_row_3)),1, cv::Scalar(0,255,0),   cv::FILLED, 0,0);
        }
        if(vect[i].id == 1)
        {
            cv::circle(environnement, cv::Point((int)(idx_col_3),(int)(idx_row_3)),1, cv::Scalar(0,100,0),   cv::FILLED, 0,0);
        }

        // 3 GET POSITION OF POINT FROM POSITION SENSOR OF ROBOT BEFORE

        for(int ii = 0; ii < vectd.size(); ii++)
        {
            double idx_col_4;
            double idx_row_4;

            double distance4 = vectd[ii].distance * 10;
            double angle4    = toRadians(vectd[ii].raw);

            idx_col_4 = cos(angle2b - angle4 - toRadians(vect[i].raw)) * distance4 + idx_col_3;
            idx_row_4 = sin(angle2b - angle4 - toRadians(vect[i].raw)) * distance4 + idx_row_3;

            cv::circle(environnement, cv::Point((int)(idx_col_4),(int)(idx_row_4)),0, cv::Scalar(0,0,0),   cv::FILLED, 0,0);
        }
    }




    cv::circle(environnement, cv::Point((int)(centre.i),(int)(centre.j))  ,1, cv::Scalar(0,0,255), cv::FILLED, 0,0);
    cv::namedWindow( "DEBUG_MDL_ENV_SENSING", 4);
    cv::imshow("DEBUG_MDL_ENV_SENSING", environnement);
    char d =(char)cv::waitKey(0);
}