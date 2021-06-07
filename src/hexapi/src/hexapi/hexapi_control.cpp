#include <math.h> 
#include "ros/ros.h"
#include "hexapi_control.h"
#include "std_msgs/MultiArrayDimension.h"

/********************
 * Calibrated parameters for the robot
 *********************/
// #define SERVO_MIN 2800
// #define SERVO_MAX 7600
// #define SERVO_DEFAULT 4500// #define SERVO_DEFAULT 7600

#define PWM_UNCHANGED -1
#define MAX_PCA9685_INPUTS 16
#define MAX_SUPPORTED_SERVO 15
#define NUM_SERVO_LEGS 12
#define NUM_SERVO_LEGS_EACH (NUM_SERVO_LEGS / 2)

#define SERVO_MAX_ANGLE 45
#define SERVO_MIN_ANGLE -45

using hexapi_control::ErrCode;

// Length of diagonals of lower leg
static const float l_low_diag = sqrt(LENGTH_LEG_LOWER_VERT*LENGTH_LEG_LOWER_VERT + LENGTH_LEG_LOWER_HOR*LENGTH_LEG_LOWER_HOR);
static const float alpha_l_low_h_l_low_v = atan2(LENGTH_LEG_LOWER_HOR, LENGTH_LEG_LOWER_VERT);

// 0 degrees
int SERVO_DEFAULT[MAX_SUPPORTED_SERVO] = {4400, 4800, 
                                         4800, 5000,
                                         5200, 5100,
                                         4400, 4500,
                                         4800, 4800,
                                         5000, 4500,
                                         -1, -1, -1};
// - 45 degrees
int SERVO_MIN[MAX_SUPPORTED_SERVO] = {6600, 2800,
                                     6600, 2800,
                                     6600, 2800,
                                     6600, 2800,
                                     6600, 2800,
                                     6600, 2800,
                                     -1, -1, -1}; 
// + 45 degrees
int SERVO_MAX[MAX_SUPPORTED_SERVO] = {2800, 6600,
                                     2800, 6600,
                                     2800, 6600,
                                     2800, 6600,
                                     2800, 6600,
                                     2800, 6600,
                                     -1, -1, -1};

/******************************
 * Private functions
 *******************************/
std_msgs::Int32MultiArray createServoMsg(const std::vector<int> &commands_pwm_) {
    std_msgs::Int32MultiArray msg;
    msg.data.clear();

    int size_command = commands_pwm_.size();
    for (int i=0; i<MAX_PCA9685_INPUTS; i++) {
        if (i >= size_command) {
            msg.data.push_back(PWM_UNCHANGED);
        } else {
            msg.data.push_back(commands_pwm_[i]);
        }
    }

    std_msgs::MultiArrayDimension dim1;  // channels
    dim1.label = "channels";
    dim1.size = MAX_PCA9685_INPUTS;
    dim1.stride = 1;
    msg.layout.dim.push_back(dim1);
    msg.layout.data_offset = 0;

    return msg;
}

/******************************
 * Class public methods
 *******************************/
HexapiControl::HexapiControl() {
    commands_angle_ = std::vector<int> (MAX_SUPPORTED_SERVO, 0);
    commands_pwm_.clear();
    for (int i=0; i<commands_angle_.size(); i++) {
        int pwm = convertAngleToPWM(i, commands_angle_[i]);
        commands_pwm_.push_back(pwm);
    }
    is_initialized_ = true;
}

HexapiControl::~HexapiControl() {

}

ErrCode HexapiControl::init(tf2::Vector3 body_center, std_msgs::Int32MultiArray &msg) {
    if (!is_initialized_) return ErrCode::ERR_FAILURE;   
    if (commands_pwm_.size() != MAX_SUPPORTED_SERVO || commands_angle_.size() != MAX_SUPPORTED_SERVO) {
        ROS_ERROR("Never initialized properly");
        return ErrCode::ERR_FAILURE;
    }

    if (move(body_center, msg) != ErrCode::ERR_SUCCESS) {
        return ErrCode::ERR_FAILURE;
    }

    return ErrCode::ERR_SUCCESS;
}

ErrCode HexapiControl::move(tf2::Vector3 body_center, std_msgs::Int32MultiArray &msg) {
    if (!is_initialized_) return ErrCode::ERR_SUCCESS;

    std::vector<int> commands_angle;
    float z = body_center.z();
    if (computeIK(z, commands_angle) != ErrCode::ERR_SUCCESS) {
        return ErrCode::ERR_FAILURE;
    }

    for (int i=0; i<commands_angle.size(); i++) {
        commands_angle_[i] = commands_angle[i];
        ROS_DEBUG("Servonum [%d] : %d", i, commands_angle_[i]);
        commands_pwm_[i] = convertAngleToPWM(i, commands_angle_[i]);
    }

    msg = createServoMsg(commands_pwm_);
    return ErrCode::ERR_SUCCESS;
}


/******************************
 * Class private methods
 *******************************/
int HexapiControl::convertAngleToPWM(int servonum, int angle) {
    if (servonum >= MAX_SUPPORTED_SERVO) {
        ROS_ERROR("Must be within the number of servos supported. %d", servonum);
    }

    int offset, pwm_val;

    if (angle >= 0) {
        offset = ((float)angle / SERVO_MAX_ANGLE) * (SERVO_MAX[servonum] - SERVO_DEFAULT[servonum]);  // assume piecewise linear
    } else {
        offset = ((float)angle / SERVO_MIN_ANGLE) * (SERVO_MIN[servonum] - SERVO_DEFAULT[servonum]);
    }
    pwm_val = SERVO_DEFAULT[servonum] + offset;

    return pwm_val;
}

ErrCode HexapiControl::computeIK(float z, std::vector<int> &commands_angle_out) {
    float theta_low_rad, theta_high_rad;  // rad (absolute value)
    int theta_low_deg, theta_high_deg;  // degrees (absolute value)

    commands_angle_out.clear();
    for (int i=0; i<MAX_SUPPORTED_SERVO; i++) {
        if (!isLowerLeg(i)) {
            commands_angle_out.push_back(0);  // no angle
            continue;
        } 

        // Calculate the joint angles needed for z
        // theta_low = acos(z / l_low_diag) - alpha_l_low_h_l_low_v (Radians)
        theta_low_rad = acos(z / l_low_diag) - alpha_l_low_h_l_low_v;
        theta_low_deg = (int) ((theta_low_rad / M_PI) * 180);

        if (isLeftLeg(i)) commands_angle_out.push_back(-theta_low_deg);
        else commands_angle_out.push_back(theta_low_deg);
    }
    
    if (commands_angle_out.size() != MAX_SUPPORTED_SERVO) {
        ROS_ERROR("Something has gone wrong in %s", __FUNCTION__);
        return ErrCode::ERR_FAILURE;
    }
    return ErrCode::ERR_SUCCESS;
}

inline bool HexapiControl::isLowerLeg(int servonum) {
    return servonum < NUM_SERVO_LEGS && servonum % 2 == 1;
}

inline bool HexapiControl::isLeftLeg(int servonum) {
    return servonum < NUM_SERVO_LEGS_EACH;
}