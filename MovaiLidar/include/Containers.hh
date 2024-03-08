#ifndef CONTAINERS_HH
#define CONTAINERS_HH

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud_I;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud;
class mountingPositions {
   public:
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    void Set(std::string sensor_pose) {
        std::stringstream ss(sensor_pose);
        ss >> this->x;
        ss >> this->y;
        ss >> this->z;
        ss >> this->roll;
        ss >> this->pitch;
        ss >> this->yaw;
    };
};

class MovaiLidarParameters {
   public:
    float min_vertical_angle;
    float max_vertical_angle;
    float vertical_step_angle;

    float min_horizontal_angle;
    float max_horizontal_angle;
    float horizontal_step_angle;

    float horizontal_points_count;
    float vertical_points_count;

    bool parameters_initialized;

    float cam_fov;
    bool enable_fusion;

    std::string DnnPath;
    float numOfClasses;
    float maxSensorTrainingRange;

    float gndThreshold;
    bool gndSegMorphological;

    mountingPositions mp;

    std::vector<double> azimuthVec, elevationVec;
};


class SGM {
   public:
    cv::Mat x, y, z, r, g, b, d, i, a;
    int height, width;
    SGM(int height, int width) {
        this->x = cv::Mat(height, width, CV_32F);
        this->y = cv::Mat(height, width, CV_32F);
        this->z = cv::Mat(height, width, CV_32F);
        this->r = cv::Mat(height, width, CV_32F);
        this->g = cv::Mat(height, width, CV_32F);
        this->b = cv::Mat(height, width, CV_32F);
        this->d = cv::Mat(height, width, CV_32F);
        this->i = cv::Mat(height, width, CV_32F);
        this->a = cv::Mat(height, width, CV_32F);
        this->height = height;
        this->width = width;
    }
    cv::Mat spherical_grid_map_slopes() {
        // slope up direction
        int rows = this->x.rows;
        int cols = this->x.cols;
        cv::Mat deltaX = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);

        cv::absdiff(this->x(cv::Range(1, rows), cv::Range::all()), this->x(cv::Range(0, rows - 1), cv::Range::all()),
                    deltaX(cv::Range(1, rows), cv::Range::all()));
        cv::pow(deltaX, 2, deltaX);

        cv::Mat deltaY = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);
        cv::absdiff(this->y(cv::Range(1, rows), cv::Range::all()), this->y(cv::Range(0, rows - 1), cv::Range::all()),
                    deltaY(cv::Range(1, rows), cv::Range::all()));
        cv::pow(deltaY, 2, deltaY);
        cv::Mat deltaXY = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);
        deltaXY = deltaX + deltaY;
        cv::sqrt(deltaXY, deltaXY);

        cv::Mat deltaZ = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);
        cv::absdiff(this->z(cv::Range(1, rows), cv::Range::all()), this->z(cv::Range(0, rows - 1), cv::Range::all()),
                    deltaZ(cv::Range(1, rows), cv::Range::all()));
        // cv::pow(deltaZ, 2, deltaZ);
        // cv::sqrt(deltaZ, deltaZ);

        cv::Mat slope = deltaZ / deltaXY;
        return slope;
    }

};
#endif