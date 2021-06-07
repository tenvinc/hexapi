#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "hexapi.h"
#include "hexapi_control.h"

#define MAX_PCA9685_INPUTS 16

#define SERVO_MIN 2800
#define SERVO_MAX 7600

Hexapi g_hexapi;
HexapiControl g_hexapi_control;

ros::Publisher g_pca9685_pub;
std_msgs::Int32MultiArray g_msg_to_send;

bool start = true;

void commandRecvCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    geometry_msgs::Vector3 vel_msg = msg->linear;
    tf2::Vector3 vel;
    tf2::convert(vel_msg, vel);
    tf2::Vector3 body_center_curr = g_hexapi.getBodyCenter();
    tf2::Vector3 body_center_next = g_hexapi.move(body_center_curr, vel, 0.1);
    g_hexapi.setBodyCenter(body_center_next);  // Clamp if needed
    body_center_next = g_hexapi.getBodyCenter();

    g_hexapi_control.move(body_center_next, g_msg_to_send);
    g_pca9685_pub.publish(g_msg_to_send); 
}

// void commandRecvCallback(const std_msgs::Float32::ConstPtr &msg) {

//     float command_z = msg->data;
//     if (command_z > Z_MAX || command_z < Z_MIN) return;

//     // g_hexapi_control.move(command_z, g_msg_to_send);

//     // int command_angle = msg->data;
//     // g_hexapi_control.move(command_angle, g_msg_to_send);
//     // g_pca9685_pub.publish(g_msg_to_send);  
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "hexapi");
    ros::NodeHandle nh;

    g_pca9685_pub = nh.advertise<std_msgs::Int32MultiArray> ("/pca9685_node/command", 10);
    ros::Subscriber cmd_sub = nh.subscribe("/hexapi_commands/string", 1000, commandRecvCallback);



    g_hexapi = Hexapi(nh);
    g_hexapi.init();
    tf2::Vector3 body_center = g_hexapi.getBodyCenter();

    if (g_hexapi_control.init(body_center, g_msg_to_send) != hexapi_control::ErrCode::ERR_SUCCESS) {
        ROS_ERROR("Something has gone wrong during the init phase.");
    }
    g_pca9685_pub.publish(g_msg_to_send);
    ROS_INFO("Initialized successfully.");

    ros::Rate loop_rate(50);

    while (ros::ok()) {
        g_pca9685_pub.publish(g_msg_to_send);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}