#include <cstring>
#include <cstdio>

#include "MojoClient.h"
#include "friLBRState.h"
#include "friLBRCommand.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ros_iiwa_fri/ExternalTorque.h"
#include "ros_iiwa_fri/JointCommandPosition.h"

using namespace KUKA::FRI;

bool STARTED = false;
double JOINT_POSITION_COMMAND[7];
ros::Time JOINT_POSITION_COMMAND_STAMP_CURRENT;
ros::Time JOINT_POSITION_COMMAND_STAMP_PREVIOUS;

void cmdCallback(const ros_iiwa_fri::JointCommandPosition::ConstPtr& msg){

    JOINT_POSITION_COMMAND_STAMP_CURRENT = ros::Time::now();
    for (int i=0; i< LBRState::NUMBER_OF_JOINTS; i++)
    {
        JOINT_POSITION_COMMAND[i] = msg->values[i];
    }
    JOINT_POSITION_COMMAND_STAMP_PREVIOUS = JOINT_POSITION_COMMAND_STAMP_CURRENT;
}

MojoClient::MojoClient() :
        n(), joint_state_pub(), external_torque_pub(), joint_command_position_sub(), msg_iiwa_joint_state(),
        msg_external_torque(), msg_joint_command_position()
{
    joint_state_pub = n.advertise<sensor_msgs::JointState>("iiwa_joint_states", 1);
    external_torque_pub = n.advertise<ros_iiwa_fri::ExternalTorque>("iiwa_external_torque", 1);
    joint_command_position_sub = n.subscribe("iiwa_joint_cmd_pos", 1, cmdCallback);

    msg_iiwa_joint_state.position.resize(7);
    msg_iiwa_joint_state.velocity.resize(7);
    msg_iiwa_joint_state.effort.resize(7);
    msg_iiwa_joint_state.name.resize(7);
    msg_iiwa_joint_state.name[0] = "joint_a1";
    msg_iiwa_joint_state.name[1] = "joint_a2";
    msg_iiwa_joint_state.name[2] = "joint_a3";
    msg_iiwa_joint_state.name[3] = "joint_a4";
    msg_iiwa_joint_state.name[4] = "joint_a5";
    msg_iiwa_joint_state.name[5] = "joint_a6";
    msg_iiwa_joint_state.name[6] = "joint_a7";

    msg_external_torque.values.resize(7);

    msg_joint_command_position.values.resize(7);

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
            break;
        }
        case MONITORING_READY:
        {
            break;
        }
        case COMMANDING_WAIT:
        {
            break;
        }
        case COMMANDING_ACTIVE:
        {
            break;
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
    rosPublish();
}

void MojoClient::waitForCommand()
{
    LBRClient::waitForCommand();
    rosPublish();
}

void MojoClient::command()
{
    rosPublish();

    if (STARTED){
        robotCommand().setJointPosition(JOINT_POSITION_COMMAND);
    } else {
        robotCommand().setJointPosition(robotState().getCommandedJointPosition());
    }
}

void MojoClient::rosPublish(){

    double delta_t = (ros::Time::now() - msg_iiwa_joint_state.header.stamp).toSec();

    for (int i=0; i<7; i++){
        msg_iiwa_joint_state.velocity[i] = (robotState().getMeasuredJointPosition()[i]
                                            - msg_iiwa_joint_state.position[i])/delta_t;
        msg_iiwa_joint_state.position[i] = robotState().getMeasuredJointPosition()[i];
        msg_iiwa_joint_state.effort[i] = robotState().getMeasuredTorque()[i];

        msg_external_torque.values[i] = robotState().getExternalTorque()[i];

        if (not STARTED){
            JOINT_POSITION_COMMAND[i] = robotState().getCommandedJointPosition()[i];
        }
    }
    STARTED = true;

    msg_iiwa_joint_state.header.stamp = ros::Time::now();
    msg_external_torque.stamp = ros::Time::now();

    joint_state_pub.publish(msg_iiwa_joint_state);
    external_torque_pub.publish(msg_external_torque);

}

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
