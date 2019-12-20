//
// Created by mojo on 20.12.2019.
//

#ifndef ROS_IIWA_FRI_MOJOCLIENT_H
#define ROS_IIWA_FRI_MOJOCLIENT_H

#include <time.h>
#include "friLBRClient.h"
#include "friClientIf.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ros_iiwa_fri/ExternalTorque.h"

/**
 * \brief Example client for Fieldbus access.
 */
class MojoClient : public KUKA::FRI::LBRClient
{
private:
    ros::NodeHandle n;
    ros::Publisher joint_state_pub;
    ros::Publisher external_torque_pub;
    sensor_msgs::JointState msg_iiwa_joint_state;
    ros_iiwa_fri::ExternalTorque msg_external_torque;

public:

    /**
     * \brief Constructor.
     *
     */
    MojoClient();

    /**
     * \brief Destructor.
     */
    ~MojoClient();

    virtual void onStateChange(KUKA::FRI::ESessionState newState, KUKA::FRI::ESessionState oldState);
    virtual void monitor();
    virtual void waitForCommand();
    virtual void command();
};

#endif //ROS_IIWA_FRI_MOJOCLIENT_H
