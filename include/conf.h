#ifndef CONF_H
#define CONF_H

#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <math.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>


#include <opencv2/opencv.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>



#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include "cv_bridge/cv_bridge.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <tf/transform_broadcaster.h>


namespace
{

    class __GET_TICK_COUNT
    {
    public:
        __GET_TICK_COUNT()
        {
        if (gettimeofday(&tv_, NULL) != 0)
            throw 0;
        }
        timeval tv_;
    };
    __GET_TICK_COUNT timeStart;
}

using namespace std;

namespace url_common
{

inline Eigen::Matrix4f CVT_mat2eigen(cv::Mat mat);
inline cv::Mat CVT_eigen2mat(Eigen::Matrix4f mat);
inline void CVT_mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);


inline cv::Mat CVT_xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);
    inline double GaussianRand() {
        double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
        double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
        double r = u * u + v * v;
        if (r == 0 || r > 1) return GaussianRand();
        double c = sqrt(-2 * log(r) / r);
        return u * c;
    }

    inline unsigned long GetTickCount(int option = 0)   //0 : ms / 1:ns
    {
        static time_t   secStart    = timeStart.tv_.tv_sec;
        static time_t   usecStart   = timeStart.tv_.tv_usec;
                    timeval tv;
        gettimeofday(&tv, NULL);
        if(option == 0)
        return (tv.tv_sec - secStart) * 1000 + (tv.tv_usec - usecStart) / 1000;
        else
        return (tv.tv_sec - secStart) * 1000000 + (tv.tv_usec - usecStart);
    }
        // Eigen matrix
        inline geometry_msgs::Pose CVT_eigen2geoPose(Eigen::Matrix4f pose)
        {
            geometry_msgs::Pose geoPose;


            tf::Matrix3x3 m;
            m.setValue((double)pose(0,0),
                    (double)pose(0,1),
                    (double)pose(0,2),
                    (double)pose(1,0),
                    (double)pose(1,1),
                    (double)pose(1,2),
                    (double)pose(2,0),
                    (double)pose(2,1),
                    (double)pose(2,2));

            tf::Quaternion q;
            m.getRotation(q);
            geoPose.orientation.x = q.getX();
            geoPose.orientation.y = q.getY();
            geoPose.orientation.z = q.getZ();
            geoPose.orientation.w = q.getW();

            geoPose.position.x = pose(0,3);
            geoPose.position.y = pose(1,3);
            geoPose.position.z = pose(2,3);

            return geoPose;
        }

        inline Eigen::Matrix4f CVT_geoPose2eigen(geometry_msgs::Pose geoPose)
        {
            Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
            tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
            tf::Matrix3x3 m(q);
            result(0,0) = m[0][0];
            result(0,1) = m[0][1];
            result(0,2) = m[0][2];
            result(1,0) = m[1][0];
            result(1,1) = m[1][1];
            result(1,2) = m[1][2];
            result(2,0) = m[2][0];
            result(2,1) = m[2][1];
            result(2,2) = m[2][2];
            result(3,3) = 1;

            result(0,3) = geoPose.position.x;
            result(1,3) = geoPose.position.y;
            result(2,3) = geoPose.position.z;

            return result;
        }


        inline Eigen::VectorXf CVT_eigen2xyzrpy(Eigen::Matrix4f mat)
        {
            Eigen::VectorXf result(6);


            CVT_mat2xyzrpy(CVT_eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);


    //        result[0] = mat(0,3);
    //        result[1] = mat(1,3);
    //        result[2] = mat(2,3);

    //        Eigen::Matrix3f rotmat = mat.block<3,3>(0,0);
    //        Eigen::Vector3f vecEuler = rotmat.eulerAngles(2,1,0);
    //        result[3] = vecEuler[2]; // roll
    //        result[4] = vecEuler[1]; // pitch
    //        result[5] = vecEuler[0]; // yaw

            return result;
        }


        inline Eigen::Matrix4f CVT_xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
        {
            Eigen::Matrix4f result =  CVT_mat2eigen(CVT_xyzrpy2mat(x,y,z,roll,pitch,yaw));



    //        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    //        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    //        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    //        Eigen::Matrix3f rotmat = yawAngle.matrix() * pitchAngle.matrix() * rollAngle.matrix();

    //        result.block<3,3>(0,0) = rotmat;
    //        result(0,3) = x;
    //        result(1,3) = y;
    //        result(2,3) = z;

            return result;
        }

        inline Eigen::Matrix4f CVT_mat2eigen(cv::Mat mat)
        {
            Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

            result(0,0) = mat.at<float>(0,0);
            result(0,1) = mat.at<float>(0,1);
            result(0,2) = mat.at<float>(0,2);
            result(0,3) = mat.at<float>(0,3);

            result(1,0) = mat.at<float>(1,0);
            result(1,1) = mat.at<float>(1,1);
            result(1,2) = mat.at<float>(1,2);
            result(1,3) = mat.at<float>(1,3);

            result(2,0) = mat.at<float>(2,0);
            result(2,1) = mat.at<float>(2,1);
            result(2,2) = mat.at<float>(2,2);
            result(2,3) = mat.at<float>(2,3);

            result(3,0) = mat.at<float>(3,0);
            result(3,1) = mat.at<float>(3,1);
            result(3,2) = mat.at<float>(3,2);
            result(3,3) = mat.at<float>(3,3);

            return result;
        }

        inline cv::Mat CVT_eigen2mat(Eigen::Matrix4f mat)
        {
            cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
            result.at<float>(0,0) = mat(0,0);
            result.at<float>(0,1) = mat(0,1);
            result.at<float>(0,2) = mat(0,2);
            result.at<float>(0,3) = mat(0,3);

            result.at<float>(1,0) = mat(1,0);
            result.at<float>(1,1) = mat(1,1);
            result.at<float>(1,2) = mat(1,2);
            result.at<float>(1,3) = mat(1,3);

            result.at<float>(2,0) = mat(2,0);
            result.at<float>(2,1) = mat(2,1);
            result.at<float>(2,2) = mat(2,2);
            result.at<float>(2,3) = mat(2,3);

            result.at<float>(3,0) = mat(3,0);
            result.at<float>(3,1) = mat(3,1);
            result.at<float>(3,2) = mat(3,2);
            result.at<float>(3,3) = mat(3,3);

            return result;
        }


    inline geometry_msgs::Pose CVT_mat2geoPose(cv::Mat pose)
    {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double)pose.at<float>(0,0),
                (double)pose.at<float>(0,1),
                (double)pose.at<float>(0,2),
                (double)pose.at<float>(1,0),
                (double)pose.at<float>(1,1),
                (double)pose.at<float>(1,2),
                (double)pose.at<float>(2,0),
                (double)pose.at<float>(2,1),
                (double)pose.at<float>(2,2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose.at<float>(0,3);
        geoPose.position.y = pose.at<float>(1,3);
        geoPose.position.z = pose.at<float>(2,3);

        return geoPose;
    }

    inline cv::Mat CVT_geoPose2mat(geometry_msgs::Pose geoPose)
    {
        cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3 m(q);
        result.at<float>(0,0) = m[0][0];
        result.at<float>(0,1) = m[0][1];
        result.at<float>(0,2) = m[0][2];
        result.at<float>(1,0) = m[1][0];
        result.at<float>(1,1) = m[1][1];
        result.at<float>(1,2) = m[1][2];
        result.at<float>(2,0) = m[2][0];
        result.at<float>(2,1) = m[2][1];
        result.at<float>(2,2) = m[2][2];
        result.at<float>(3,3) = 1;

        result.at<float>(0,3) = geoPose.position.x;
        result.at<float>(1,3) = geoPose.position.y;
        result.at<float>(2,3) = geoPose.position.z;

        return result;
    }

    inline void CVT_mat2q(cv::Mat pose, double &qX, double &qY, double &qZ, double &qW)
    {
        tf::Matrix3x3 m;
        m.setValue((double)pose.at<float>(0,0),
                (double)pose.at<float>(0,1),
                (double)pose.at<float>(0,2),
                (double)pose.at<float>(1,0),
                (double)pose.at<float>(1,1),
                (double)pose.at<float>(1,2),
                (double)pose.at<float>(2,0),
                (double)pose.at<float>(2,1),
                (double)pose.at<float>(2,2));

        tf::Quaternion q;
        m.getRotation(q);

        qX = q.getX();
        qY = q.getY();
        qZ = q.getZ();
        qW = q.getW();
    }

    inline cv::Mat CVT_q2mat(double qX, double qY, double qZ, double qW)
    {

        cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
        tf::Quaternion q(qX, qY, qZ, qW);
        tf::Matrix3x3 m(q);
        result.at<float>(0,0) = m[0][0];
        result.at<float>(0,1) = m[0][1];
        result.at<float>(0,2) = m[0][2];
        result.at<float>(1,0) = m[1][0];
        result.at<float>(1,1) = m[1][1];
        result.at<float>(1,2) = m[1][2];
        result.at<float>(2,0) = m[2][0];
        result.at<float>(2,1) = m[2][1];
        result.at<float>(2,2) = m[2][2];
        result.at<float>(3,3) = 1;

        return result;
    }

    inline cv::Mat CVT_xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
    {
            cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
            rot_vec.at<float>(0) = roll;
            rot_vec.at<float>(1) = pitch;
            rot_vec.at<float>(2) = yaw;

            cv::Mat rot_mat;
            cv::Rodrigues(rot_vec,rot_mat);

            cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

            rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

            result.at<float>(0,3) = x;
            result.at<float>(1,3) = y;
            result.at<float>(2,3) = z;

            result.at<float>(3,3) = 1;

            return result;
    }

    inline void CVT_mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
    {
        *x = mat.at<float>(0,3);
        *y = mat.at<float>(1,3);
        *z = mat.at<float>(2,3);

        cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

        cv::Mat rot_vec;
        cv::Rodrigues(rot_mat,rot_vec);
        *roll = rot_vec.at<float>(0);
        *pitch = rot_vec.at<float>(1);
        *yaw = rot_vec.at<float>(2);

    }


    inline void CVT_mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID)
    {
        cv_bridge::CvImage bridge;
        mat.copyTo(bridge.image);
        bridge.header.frame_id = frameID;
        bridge.header.stamp = ros::Time::now();
        if(mat.type() == CV_8UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::MONO8;
        }
        else if(mat.type() == CV_8UC3)
        {
            bridge.encoding = sensor_msgs::image_encodings::BGR8;
        }
        else if(mat.type() == CV_32FC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        }
        else if(mat.type() == CV_16UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        }
        else
        {
            std::cout <<"Error : mat type" << std::endl;

        }

        bridge.toImageMsg(sensorImg);
    }

    inline cv::Mat CVT_sensorImg2mat(sensor_msgs::Image sensorImg)
    {
        static cv_bridge::CvImagePtr cv_ptr;
        cv::Mat mat;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(sensorImg, sensorImg.encoding);
            mat = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        return mat;
    }

    inline cv::Mat CVT_vector2matPY(float x,float y, float z)
    {
      float distance = sqrt(z * z + x * x);
      double pitch = asin(y/distance);
      float yaw = atan2(x, z);
      return CVT_xyzrpy2mat(0,0,0,0,pitch,yaw);
    }
    inline float CAL_distbetMat(cv::Mat pose,cv::Mat pose2)
    {
        float distance = sqrt(pow(pose.at<float>(0,3)-pose2.at<float>(0,3),2)+
                              pow(pose.at<float>(1,3)-pose2.at<float>(1,3),2)+
                              pow(pose.at<float>(2,3)-pose2.at<float>(2,3),2));
        return distance;
    }

    inline void CAL_adddist2mat(cv::Mat dist,cv::Mat & pose)
    {
        pose.at<float>(0,3) = pose.at<float>(0,3) + dist.at<float>(0);
        pose.at<float>(1,3) = pose.at<float>(1,3) + dist.at<float>(1);
        pose.at<float>(2,3) = pose.at<float>(2,3) + dist.at<float>(2);
    }
}

#endif
