#include "hexapi.h"
#include "hexapi_control.h"

Hexapi::Hexapi(const ros::NodeHandle &nh) {
    // Read from paramete servers
    if (nh.param<double>("length_leg_lower_vert", length_leg_lower_vert_, LENGTH_LEG_LOWER_VERT)) {
        ROS_INFO("Length of leg lower vertical provided.");
    }

    if (nh.param<double>("length_leg_lower_hor", length_leg_lower_hor_, LENGTH_LEG_LOWER_HOR)) {
        ROS_INFO("Length of leg lower horizontal provided.");
    }

    ROS_INFO("Using the length of leg provided (%.02f, %.02f)", length_leg_lower_hor_,
            length_leg_lower_vert_);

    if (nh.param<double>("z_min", z_min_, Z_MIN)) {
        ROS_INFO("Z min provided.");
    }

    if (nh.param<double>("z_max", z_max_, Z_MAX)) {
        ROS_INFO("Z max provided.");
    }

    ROS_INFO("Using the range of Z provided [%.02f, %.02f]", z_min_,
            z_max_);

    is_initialized_ = true;
}

Hexapi::Hexapi() {

}

Hexapi::~Hexapi() {

}

void Hexapi::init() {
    if (!is_initialized_) {
        ROS_FATAL("Need to use the constructor with a node handle, [%s]", __FUNCTION__);
        return;
    }

    body_center_ = tf2::Vector3(0, 0, length_leg_lower_vert_);
}

void Hexapi::move(tf2::Vector3 vel) {
    if (!is_initialized_) {
        ROS_FATAL("Need to use the constructor with a node handle, [%s]", __FUNCTION__);
        return;
    }

    ros::Duration dt = ros::Time::now() - time_prev_act_;
    tf2::Vector3 res = move(body_center_, vel, dt.toSec());
    setBodyCenter(res);
}

tf2::Vector3 Hexapi::move(tf2::Vector3 current, tf2::Vector3 vel, float dt) {
    return current + vel * dt;
}

void Hexapi::setActTime(ros::Time time_to_set) {
    time_prev_act_ = time_to_set;
}

const tf2::Vector3 Hexapi::getBodyCenter() {
    return body_center_;
}

void Hexapi::setBodyCenter(const tf2::Vector3 body_center) {
    body_center_.setX(body_center.x());
    body_center_.setY(body_center.y());

    float z = body_center.z();
    if (z > z_max_) z = z_max_;
    else if (z < z_min_) z = z_min_;

    body_center_.setZ(z);
}