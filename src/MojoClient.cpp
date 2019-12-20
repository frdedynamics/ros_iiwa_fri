#include <cstring>
#include <cstdio>

#include "MojoClient.h"
#include "friLBRState.h"
#include "friLBRCommand.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ros_iiwa_fri/ExternalTorque.h"

using namespace KUKA::FRI;

MojoClient::MojoClient() :
        n(), joint_state_pub(), external_torque_pub(), msg_iiwa_joint_state(), msg_external_torque()
{
    joint_state_pub = n.advertise<sensor_msgs::JointState>("iiwa_joint_states", 1);
    external_torque_pub = n.advertise<ros_iiwa_fri::ExternalTorque>("iiwa_external_torque", 1);

    msg_iiwa_joint_state.position.resize(7);
    msg_iiwa_joint_state.velocity.resize(7);
    msg_iiwa_joint_state.effort.resize(7);
    msg_iiwa_joint_state.name.resize(7);
    for (int i=0; i<7; i++){
        msg_iiwa_joint_state.name[i] = "joint_a" + std::to_string(i+1);
    }

    msg_external_torque.values.resize(7);

    printf("MojoClient initialized:\n");
}

MojoClient::~MojoClient()
{
}

void MojoClient::onStateChange(KUKA::FRI::ESessionState newState, KUKA::FRI::ESessionState oldState)
{

    LBRClient::onStateChange(oldState, newState);

    switch (newState)
    {
        case MONITORING_WAIT:
        {
        }
        case MONITORING_READY:
        {
        }
        case COMMANDING_WAIT:
        {
        }
        case COMMANDING_ACTIVE:
        {
        }
        default:
        {
            break;
        }
    }
}

void MojoClient::monitor()
{

    LBRClient::monitor();

    double delta_t = (ros::Time::now() - msg_iiwa_joint_state.header.stamp).toSec();

    for (int i=0; i<7; i++){
        msg_iiwa_joint_state.velocity[i] = (robotState().getMeasuredJointPosition()[i]
                                            - msg_iiwa_joint_state.position[i])/delta_t;
        msg_iiwa_joint_state.position[i] = robotState().getMeasuredJointPosition()[i];
        msg_iiwa_joint_state.effort[i] = robotState().getMeasuredTorque()[i];

        msg_external_torque.values[i] = robotState().getExternalTorque()[i];
    }

    msg_iiwa_joint_state.header.stamp = ros::Time::now();
    msg_external_torque.stamp = ros::Time::now();

    joint_state_pub.publish(msg_iiwa_joint_state);
    external_torque_pub.publish(msg_external_torque);
}

void MojoClient::waitForCommand()
{
    LBRClient::waitForCommand();
}

void MojoClient::command()
{
}


// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif