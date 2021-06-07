#ifndef _HEXAPI_H
#define _HEXAPI_H

#include "ros/ros.h"
#include <tf2/LinearMath/Vector3.h>

/**
 * This class represents the high level concept of the functions Hexapi and perform,
 * such as movement, surveillance. (All in the LH Frame with Z pointing up and X pointing forward)
 * Supports:
 * 1. Initial boot up (standing position)
 * 2. Adjusting body based on commands
 */ 

#define Z_MAX 6
#define Z_MIN 4.5

class Hexapi {
public:
    Hexapi(const ros::NodeHandle &nh);
    Hexapi();
    ~Hexapi();

    void init();
    void move(tf2::Vector3 vel);
    tf2::Vector3 move(tf2::Vector3 current, tf2::Vector3 vel, float dt);

    void setActTime(ros::Time time_to_set);
    const tf2::Vector3 getBodyCenter();
    void setBodyCenter(const tf2::Vector3 body_center);

private:
    ros::Time time_prev_act_;
    tf2::Vector3 body_center_;

    // Constants taken from the param server
    double length_leg_lower_hor_;
    double length_leg_lower_vert_;
    double z_min_;
    double z_max_;

    bool is_initialized_ = false;
};

#endif

