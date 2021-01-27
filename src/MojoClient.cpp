#include <cstring>
#include <cstdio>

#include "MojoClient.h"
#include "friLBRState.h"
#include "friLBRCommand.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros_iiwa_fri/ExternalTorque.h"
#include "ros_iiwa_fri/JointCommandPosition.h"
#include "ros_iiwa_fri/iiwaRobotCommand.h"
#include "ros_iiwa_fri/iiwaRobotState.h"

using namespace KUKA::FRI;

struct robot_commands{
    bool LEDBlue;
    bool OutputX3Pin1;
    bool OutputX3Pin2;
    bool OutputX3Pin11;
    bool OutputX3Pin12;
    //bool SwitchOffX3Voltage;
};

bool STARTED = false;
bool USE_MEDIA_FLANGE = false;
bool USE_AXIA_FT_SENSOR = true;
double JOINT_POSITION_COMMAND[7];
ros::Time JOINT_POSITION_COMMAND_STAMP_CURRENT;
ros::Time JOINT_POSITION_COMMAND_STAMP_PREVIOUS;
robot_commands ROBOT_COMMAND;
ros::Time ROBOT_COMMAND_STAMP_CURRENT;
ros::Time ROBOT_COMMAND_STAMP_PREVIOUS;
ros::Time USER_BUTTON_CLICKED_TIME;

void cmdCallback(const ros_iiwa_fri::JointCommandPosition::ConstPtr& msg){

    JOINT_POSITION_COMMAND_STAMP_CURRENT = ros::Time::now();
    for (int i=0; i< LBRState::NUMBER_OF_JOINTS; i++)
    {
        JOINT_POSITION_COMMAND[i] = msg->values[i];
    }
    JOINT_POSITION_COMMAND_STAMP_PREVIOUS = JOINT_POSITION_COMMAND_STAMP_CURRENT;
}

void cmdRobotCallback(const ros_iiwa_fri::iiwaRobotCommand::ConstPtr& msg){

    ROBOT_COMMAND_STAMP_CURRENT = ros::Time::now();
    ROBOT_COMMAND.LEDBlue = msg->LEDBlue;
    if (msg->no_command){
        ROBOT_COMMAND.OutputX3Pin1 = false;
        ROBOT_COMMAND.OutputX3Pin11 = false;
    } else if (msg->release){
        ROBOT_COMMAND.OutputX3Pin1 = false;
        ROBOT_COMMAND.OutputX3Pin11 = true;
    } else if (msg->grip){
        ROBOT_COMMAND.OutputX3Pin1 = true;
        ROBOT_COMMAND.OutputX3Pin11 = false;
    }

    if (msg->LEDRed){
        ROBOT_COMMAND.OutputX3Pin2 = true;
        ROBOT_COMMAND.OutputX3Pin12 = true;
    } else if (msg->LEDYellow){
        ROBOT_COMMAND.OutputX3Pin2 = false;
        ROBOT_COMMAND.OutputX3Pin12 = true;
    } else if (msg->LEDGreen){
        ROBOT_COMMAND.OutputX3Pin2 = true;
        ROBOT_COMMAND.OutputX3Pin12 = false;
    } else {
        ROBOT_COMMAND.OutputX3Pin2 = false;
        ROBOT_COMMAND.OutputX3Pin12 = false;
    }

    //ROBOT_COMMAND.SwitchOffX3Voltage = msg->SwitchOffX3Voltage;
    ROBOT_COMMAND_STAMP_PREVIOUS = ROBOT_COMMAND_STAMP_CURRENT;
}

MojoClient::MojoClient() :
        n(), joint_state_pub(), external_torque_pub(), iiwa_robot_state_pub(), ati_ft_pub(),
        joint_command_position_sub(), iiwa_robot_command_sub(),
        msg_iiwa_joint_state(), msg_external_torque(), msg_joint_command_position(),
        msg_iiwa_robot_command(), msg_iiwa_robot_state(), msg_ati_ft()
{
    joint_state_pub = n.advertise<sensor_msgs::JointState>("iiwa_joint_states", 1);
    ati_ft_pub = n.advertise<geometry_msgs::WrenchStamped>("ati_ft", 1);
    external_torque_pub = n.advertise<ros_iiwa_fri::ExternalTorque>("iiwa_external_torque", 1);
    iiwa_robot_state_pub = n.advertise<ros_iiwa_fri::iiwaRobotState>("iiwa_robot_state", 1);
    joint_command_position_sub = n.subscribe("iiwa_joint_cmd_pos", 1, cmdCallback);
    iiwa_robot_command_sub = n.subscribe("iiwa_robot_command", 1, cmdRobotCallback);

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

    msg_ati_ft.header.frame_id = 'iiwa ee'

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

    ros::Time ros_time_now = ros::Time::now();

    double delta_t = (ros_time_now - msg_iiwa_joint_state.header.stamp).toSec();

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

    if (USE_AXIA_FT_SENSOR) {
        msg_ati_ft.wrench.force.x = robotState().getAnalogIOValue("AtiAxiaFtSensor.Fx");
        msg_ati_ft.wrench.force.y = robotState().getAnalogIOValue("AtiAxiaFtSensor.Fy");
        msg_ati_ft.wrench.force.z = robotState().getAnalogIOValue("AtiAxiaFtSensor.Fz");
        msg_ati_ft.wrench.torque.x = robotState().getAnalogIOValue("AtiAxiaFtSensor.Tx");
        msg_ati_ft.wrench.torque.y = robotState().getAnalogIOValue("AtiAxiaFtSensor.Ty");
        msg_ati_ft.wrench.torque.z = robotState().getAnalogIOValue("AtiAxiaFtSensor.Tz");
    }

    if (USE_MEDIA_FLANGE) {
        if (not STARTED) {
            USER_BUTTON_CLICKED_TIME = ros_time_now - ros::Duration(60);
        }

        //msg_iiwa_robot_state.InputX3Pin3 = robotState().getBooleanIOValue("MediaFlange.InputX3Pin3");
        //msg_iiwa_robot_state.InputX3Pin4 = robotState().getBooleanIOValue("MediaFlange.InputX3Pin4");
        msg_iiwa_robot_state.InputX3Pin10 = robotState().getBooleanIOValue("MediaFlange.InputX3Pin10");
        //msg_iiwa_robot_state.InputX3Pin13 = robotState().getBooleanIOValue("MediaFlange.InputX3Pin13");
        msg_iiwa_robot_state.InputX3Pin16 = robotState().getBooleanIOValue("MediaFlange.InputX3Pin16");
        msg_iiwa_robot_state.UserButton = robotState().getBooleanIOValue("MediaFlange.UserButton");
        if (msg_iiwa_robot_state.UserButton) {
            USER_BUTTON_CLICKED_TIME = ros_time_now;
        }
        if ((ros_time_now - USER_BUTTON_CLICKED_TIME).sec < 1) {
            msg_iiwa_robot_state.UserButtonPulseExtended = true;
        } else {
            msg_iiwa_robot_state.UserButtonPulseExtended = false;
        }

        if (msg_iiwa_robot_state.InputX3Pin16) {
            msg_iiwa_robot_state.released = true;
        } else {
            msg_iiwa_robot_state.released = false;
        }

        if (msg_iiwa_robot_state.InputX3Pin10) {
            msg_iiwa_robot_state.gripped = true;
        } else {
            msg_iiwa_robot_state.gripped = false;
        }

        if (not STARTED) {
            ROBOT_COMMAND.LEDBlue = robotState().getBooleanIOValue("MediaFlange.LEDBlue");
            ROBOT_COMMAND.OutputX3Pin1 = robotState().getBooleanIOValue("MediaFlange.OutputX3Pin1");
            ROBOT_COMMAND.OutputX3Pin2 = robotState().getBooleanIOValue("MediaFlange.OutputX3Pin2");
            ROBOT_COMMAND.OutputX3Pin11 = robotState().getBooleanIOValue("MediaFlange.OutputX3Pin11");
            ROBOT_COMMAND.OutputX3Pin12 = robotState().getBooleanIOValue("MediaFlange.OutputX3Pin12");
            //ROBOT_COMMAND.SwitchOffX3Voltage = robotState().getBooleanIOValue("MediaFlange.SwitchOffX3Voltage");
        }
    }
    STARTED = true;

    if (USE_MEDIA_FLANGE) {
        robotCommand().setBooleanIOValue("MediaFlange.LEDBlue", ROBOT_COMMAND.LEDBlue);
        robotCommand().setBooleanIOValue("MediaFlange.OutputX3Pin1", ROBOT_COMMAND.OutputX3Pin1);
        robotCommand().setBooleanIOValue("MediaFlange.OutputX3Pin2", ROBOT_COMMAND.OutputX3Pin2);
        robotCommand().setBooleanIOValue("MediaFlange.OutputX3Pin11", ROBOT_COMMAND.OutputX3Pin11);
        robotCommand().setBooleanIOValue("MediaFlange.OutputX3Pin12", ROBOT_COMMAND.OutputX3Pin12);
    }

    msg_iiwa_joint_state.header.stamp = ros_time_now;
    msg_external_torque.stamp = ros_time_now;
    msg_iiwa_robot_state.stamp = ros_time_now;
    msg_ati_ft.header.stamp = ros_time_now;
    msg_ati_ft.header.seq += 1;

    joint_state_pub.publish(msg_iiwa_joint_state);
    external_torque_pub.publish(msg_external_torque);
    iiwa_robot_state_pub.publish(msg_iiwa_robot_state);
    ati_ft_pub.publish(msg_ati_ft);

}

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
