#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Geometry>
#include <deque>
#include "pi_imu_ekf.h"
#include <algorithm>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat imgCallback;

using namespace std;
using namespace Eigen;

typedef std::deque<eskf::StateVector, Eigen::aligned_allocator<eskf::StateVector>> StateBuffer;
typedef std::deque<eskf::StateCov, Eigen::aligned_allocator<eskf::StateCov>> CovBuffer;

StateBuffer state_buffer;
CovBuffer cov_buffer;

ros::Publisher pub_img_pos;
struct ImuData {
    double timestamp;
    Quaterniond q;
    Vector3d acc;
    Vector3d gyro;
};
deque<ImuData> imu_buffer;

img_imu_ekf img_imu;
Vector3d mav_pos = Vector3d::Zero();
Vector3d mav_vel = Vector3d::Zero();
Vector2d mav_img = Vector2d::Zero();
// 修改：img0初始化为相机中心像素点
Vector2d img0(320, 240);
double img_f = 320;
bool get_sensor = false;
bool is_img_come = false;
bool initialized = false;

void mav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mav_pos(0) = msg->pose.position.x;
    mav_pos(1) = msg->pose.position.y;
    mav_pos(2) = msg->pose.position.z;
}

void mav_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    mav_vel(0) = msg->twist.linear.x;
    mav_vel(1) = msg->twist.linear.y;
    mav_vel(2) = msg->twist.linear.z;
}

void mav_imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    double curr_time = msg->header.stamp.toSec();
    static double last_time = curr_time;
    double dt = curr_time - last_time;

    if (dt > 1.1/IMU_FREQ) {
        ROS_WARN("IMU dt too large: %f", dt);
        dt = 1.0/IMU_FREQ;
    }

    Vector3d acc(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x,
                  msg->angular_velocity.y,
                  msg->angular_velocity.z);
    Quaterniond q(msg->orientation.w,
                  msg->orientation.x,
                  msg->orientation.y,
                  msg->orientation.z);

    imu_buffer.push_back({curr_time, q, acc, gyro});
    if(imu_buffer.size() > img_imu_ekf::BUFFER_SIZE) {
        imu_buffer.pop_front();
    }

    if (!initialized && get_sensor) {
        img_imu.sensor_init(mav_pos, mav_vel, mav_img);
        initialized = true;
        return;
    }

    if (initialized && !is_img_come) {
        img_imu.predict_state_nonlinear(q, acc, gyro, dt);
        img_imu.update_covariance_ekf(dt);

        state_buffer.push_back(img_imu.get_current_state());
        cov_buffer.push_back(img_imu.get_current_covariance());

        while(state_buffer.size() > img_imu_ekf::BUFFER_SIZE) {
            state_buffer.pop_front();
            cov_buffer.pop_front();
        }

        std_msgs::Float32MultiArray msg;
        auto state = img_imu.get_current_state();
        ROS_INFO_STREAM("EKF state: " << state.transpose());
        msg.data.push_back(state(6) * img_f + img0(0));
        msg.data.push_back(state(7) * img_f + img0(1));
        // msg.data.push_back(img_imu.get_current_state()(6) * img_f + img0(0));
        // msg.data.push_back(img_imu.get_current_state()(7) * img_f + img0(1));
        pub_img_pos.publish(msg);
    }

    last_time = curr_time;
}

void mav_img_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if (msg->data[0] < 0 || !initialized) return;

    mav_img(0) = (msg->data[0] - img0(0)) / img_f;
    mav_img(1) = (msg->data[1] - img0(1)) / img_f;

    is_img_come = true;

    int delay_idx = state_buffer.size() - img_imu_ekf::DELAY_STEPS;
    if(delay_idx >= 0) {
        img_imu.update_delayed_state(state_buffer[delay_idx],
                                     cov_buffer[delay_idx],
                                     mav_img);

        for(int i = delay_idx + 1; i < state_buffer.size(); i++) {
            const ImuData& imu = imu_buffer[i];
            double dt = imu_buffer[i].timestamp - imu_buffer[i-1].timestamp;
            img_imu.predict_state_nonlinear(imu.q, imu.acc, imu.gyro, dt);
            img_imu.update_covariance_ekf(dt);
        }
    }

    is_img_come = false;
}

void img_show_cb(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    imgCallback = cv_ptr_compressed->image;
    if (get_sensor)
    {
        cv::circle(imgCallback, cv::Point(img_imu.kf.x(6) * img_f + img0(0), img_imu.kf.x(7) * img_f + img0(1)), 5, cv::Scalar(255, 0, 0), 2);
        // cv::circle(imgCallback, cv::Point(mav_img(0) * img_f + img0(0), mav_img(1) * img_f + img0(1)), 5, cv::Scalar(255, 255, 0), 1);
    }

    cv::imshow("imgCallback", imgCallback);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "DCekf_node");
    ros::NodeHandle ekf_img;

    get_sensor = true; 

    ros::Subscriber pos_sub = ekf_img.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 10, mav_pose_cb);
    ros::Subscriber vel_sub = ekf_img.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity_local", 10, mav_vel_cb);
    ros::Subscriber imu_sub = ekf_img.subscribe<sensor_msgs::Imu>
                ("/mavros/imu/data", 10, mav_imu_cb);
    ros::Subscriber img_sub = ekf_img.subscribe<std_msgs::Float32MultiArray>
                ("tracker/pos_image", 10, mav_img_cb);
    ros::Subscriber img_show_sub = ekf_img.subscribe<sensor_msgs::CompressedImage>
                ("/camera/left/compressed", 10, img_show_cb);
    pub_img_pos = ekf_img.advertise<std_msgs::Float32MultiArray>
                ("tracker/pos_image_DCekf", 10);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}