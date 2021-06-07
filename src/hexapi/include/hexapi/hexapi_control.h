#ifndef _HEXAPI_CONTROL_H
#define _HEXAPI_CONTROL_H

#include <vector>
#include "ros/ros.h"
#include <tf2/LinearMath/Vector3.h>
#include "std_msgs/Int32MultiArray.h"

/**
 * Directions are defined, from the perspective of the robot facing forward.
 * Z axis is upwards, X axis is forward, Y axis follows the left hand frame rules
 * For each side, lowest number indicates leg closest to the front of the body
 * For each leg, joint 1 controls the upper limb, while joint 2 controls the lower limb
 */
#define LENGTH_LEG_LOWER_VERT 6.0  // l_low_v
#define LENGTH_LEG_LOWER_HOR 2.0   // l_low_h

namespace hexapi_control {
    enum class ErrCode {
        ERR_SUCCESS = 0,
        ERR_FAILURE
    };
};

enum class JointLabel {
    LEFT_LEG1_JOINT1 = 0,
    LEFT_LEG1_JOINT2,
    LEFT_LEG2_JOINT1,
    LEFT_LEG2_JOINT2,
    LEFT_LEG3_JOINT1,
    LEFT_LEG3_JOINT2,
    RIGHT_LEG1_JOINT1,
    RIGHT_LEG1_JOINT2,
    RIGHT_LEG2_JOINT1,
    RIGHT_LEG2_JOINT2,
    RIGHT_LEG3_JOINT1,
    RIGHT_LEG3_JOINT2
}; 


/**
 * This class contains the logic for controlling the servo drivers needed to move Hexapi.
 */
class HexapiControl {
public:
    HexapiControl();
    ~HexapiControl();

    /**
    * Initializes the hexapod to the default settings (upright)
    */
    hexapi_control::ErrCode init(tf2::Vector3 body_center, std_msgs::Int32MultiArray &msg);

    hexapi_control::ErrCode move(tf2::Vector3 body_center, std_msgs::Int32MultiArray &msg);
private:
    int convertAngleToPWM(int servonum, int angle);

    /**
     * Calculates the necessary angles for all the servos, using the LH frame convention
     */
    hexapi_control::ErrCode computeIK(float z, std::vector<int> &commands_angle_out);

    /**
     * Returns true when the servo belongs to the join of the lower leg
     */
    inline bool isLowerLeg(int servonum);

    /**
     * Returns true when the servo is part of the left leg (assumes it is part of the leg not the camera)
     */
    inline bool isLeftLeg(int servonum);

    std::vector<int> commands_pwm_;  // actual pwm sent
    std::vector<int> commands_angle_;  // actual angles (used for debugging or visualization)
    bool is_initialized_ = false;  // indicates the hexapod is lazy initialized (no commands sent to servo yet)
};

#endif