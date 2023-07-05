//Compile with: g++ solution.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
#include "matplotlibcpp.h" //Graph Library
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
#include <Eigen/Geometry>
#include <ros/console.h>

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

// Landmarks
double landmarks[8][2] = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
random_device rd;
mt19937 gen(rd());

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();
double gen_unit_random();
double angle_diff(double a, double b);

class Robot {
public:
    Robot()
    {
        // Constructor
        x = gen_unit_random() * alpha_1; // robot's x coordinate
        y = gen_unit_random() * alpha_2; // robot's y coordinate
        z = gen_unit_random() * world_size; // robot's y coordinate
        orient = gen_unit_random() * M_PI; // robot's orientation

        forward_noise = 0.0; //noise of the forward movement
        turn_noise = 0.0; //noise of the turn
        sense_noise = 0.0; //noise of the sensing
    }

    Robot(double init_x, double init_y, double init_z, double init_orient)
    {
        // Constructor
        x = init_x + gen_unit_random() * init_pos_conv; // robot's x coordinate
        y = init_y + gen_unit_random() * init_pos_conv; // robot's y coordinate
        z = init_z + gen_unit_random() * init_pos_conv; // robot's y coordinate
        orient = angle_diff(init_orient, gen_unit_random() * init_orient_conv); // robot's orientation
    }

    Robot(double init_x, double init_y, double init_z, double init_orient, double alpha_1, double alpha_2, double alpha_3, double alpha_4, double init_pos_conv, double init_orient_conv, double sigma_p)
    {
        // Constructor
        x = init_x + gen_unit_random() * init_pos_conv; // robot's x coordinate
        y = init_y + gen_unit_random() * init_pos_conv; // robot's y coordinate
        z = init_z + gen_unit_random() * init_pos_conv; // robot's y coordinate
        orient = angle_diff(init_orient, gen_unit_random() * init_orient_conv); // robot's orientation

        set_params(alpha_1, alpha_2, alpha_3, alpha_4, init_pos_conv, init_orient_conv, sigma_p);
    }

    void set_params(double alpha_1, double alpha_2, double alpha_3, double alpha_4, double init_pos_conv, double init_orient_conv, double sigma_p)
    {
        this->alpha_1 = alpha_1;
        this->alpha_2 = alpha_2;
        this->alpha_3 = alpha_3;
        this->alpha_4 = alpha_4;
        this->init_pos_conv = init_pos_conv;
        this->init_orient_conv = init_orient_conv;
        this->sigma_p = sigma_p;
    }

    void set(double new_x, double new_y, double new_orient)
    {
        // Set robot new position and orientation
        if (new_x < 0 || new_x >= world_size)
            throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= world_size)
            throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    vector<double> sense()
    {
        // Measure the distances from the robot toward the landmarks
        vector<double> z(sizeof(landmarks) / sizeof(landmarks[0]));
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }
        return z;
    }

    void set_odoms(Vector3d mav_pos, Vector3d mav_pos_prev, double mav_yaw, double mav_yaw_prev, float dt)
    {
        Vector3d delta_pos = mav_pos - mav_pos_prev;
        delta_rot1 = angle_diff(atan2(delta_pos(1), delta_pos(0)), mav_yaw_prev);
        delta_trans = (mav_pos - mav_pos_prev).norm();
        delta_rot2 = angle_diff(angle_diff(mav_yaw, mav_yaw_prev), delta_rot1);
        delta_rot3 = atan2(delta_pos(2), sqrt(delta_pos(0)*delta_pos(0) + delta_pos(1)*delta_pos(1)));
    }

    void sample_motion_model_simple()
    {
        double rot1_noise = alpha_1 * delta_rot1 * delta_rot1 + alpha_2 * delta_trans * delta_trans;
        delta_rot1_hat = delta_rot1 - gen_gauss_random(0.0, rot1_noise);
        double rot2_noise = alpha_1 * delta_rot2 * delta_rot2 + alpha_2 * delta_trans * delta_trans;
        delta_rot2_hat = delta_rot2 - gen_gauss_random(0.0, rot2_noise);
        double rot3_noise = alpha_1 * delta_rot3 * delta_rot3 + alpha_2 * delta_trans * delta_trans;
        delta_rot3_hat = delta_rot3 - gen_gauss_random(0.0, rot3_noise);

        double trans_noise = alpha_3 * delta_trans * delta_trans + alpha_4 * (delta_rot1 * delta_rot1 + delta_rot2 * delta_rot2);
        delta_trans_hat = delta_trans - gen_gauss_random(0.0, trans_noise);

        x = x + delta_trans_hat * cos(orient + delta_rot1_hat);
        y = y + delta_trans_hat * sin(orient + delta_rot1_hat);
        z = z + delta_trans_hat * sin(delta_rot3_hat);
        orient = orient + delta_rot1_hat + delta_rot2_hat;
    }

    void landmark_model_likelyhood_simple(Vector3d p_s)
    {
        Vector3d p(-x, -y, -z);
        p.normalize();
        p_s_hat = p;

        q = gaussian(0, sigma_p, 1.0 - p_s_hat.dot(p_s));

        // ROS_INFO_STREAM("p_s_hat: " << p_s_hat << ", p_s: " << p_s);
        // ROS_INFO_STREAM("q: " << q);
    }

    Robot move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);

        // set particle
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    string show_pose()
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + to_string(x) + " y=" + to_string(y) + " z=" + to_string(z) + " orient=" + to_string(orient) + "]";
    }

    string read_sensors()
    {
        // Returns all the distances from the robot toward the landmarks
        vector<double> z = sense();
        string readings = "[";
        for (int i = 0; i < z.size(); i++) {
            readings += to_string(z[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;
    }

    double measurement_prob(vector<double> measurement)
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            prob *= gaussian(dist, sense_noise, measurement[i]);
        }

        return prob;
    }

    double x, y, z, orient; //robot poses
    double forward_noise, turn_noise, sense_noise; //robot noises
    double delta_rot1, delta_rot2, delta_rot3, delta_trans;
    double delta_rot1_hat, delta_rot2_hat, delta_rot3_hat, delta_trans_hat;
    double alpha_1 = 0.0;   // uncertainty weight
    double alpha_2 = 0.0;
    double alpha_3 = 0.0;
    double alpha_4 = 0.0;
    double init_pos_conv = 1.0;
    double init_orient_conv = 0.1;
    Vector3d p_s_hat;
    double q;
    double sigma_p = 0.5;

private:
    double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Functions
double gen_real_random()
{
    // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

double gen_unit_random()
{
    // Generate real random between -1 and 1
    uniform_real_distribution<double> real_dist(-1.0, 1.0); //Real
    double rt = real_dist(gen);
    // cout << "gen_unit_random: " << rt << endl;
    return rt;
}

double mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double evaluation(Robot r, Robot p[], int n)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        //the second part is because of world's cyclicity
        double dx = mod((p[i].x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}
double max(double arr[], int n)
{
    // Identify the max element in an array
    double max = 0;
    for (int i = 0; i < n; i++) {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

void visualization(int n, Robot robot, int step, Robot p[], Robot pr[])
{
    //Draw the robot, landmarks, particles and resampled particles on a graph

    //Graph Format
    plt::title("MCL, step " + to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);

    //Draw particles in green
    for (int i = 0; i < n; i++) {
        plt::plot({ p[i].x }, { p[i].y }, "go");
    }

    //Draw resampled particles in yellow
    for (int i = 0; i < n; i++) {
        plt::plot({ pr[i].x }, { pr[i].y }, "yo");
    }

    //Draw landmarks in red
    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        plt::plot({ landmarks[i][0] }, { landmarks[i][1] }, "ro");
    }

    //Draw robot position in blue
    plt::plot({ robot.x }, { robot.y }, "bo");

    //Save the image and close the plot
    plt::show();
    // plt::save("./Images/Step" + to_string(step) + ".png");
    plt::clf();
}

double angle_diff(double a, double b)
{
    double diff = a - b;
    if (diff < 0)
        diff += 2 * M_PI;
    if (diff < M_PI)
        return diff;
    else
        return diff - 2 * M_PI;
}