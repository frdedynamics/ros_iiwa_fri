#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ros_iiwa_fri/ExternalTorque.h"
#include "ros_iiwa_fri/JointCommandPosition.h"

#include "MojoClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

using namespace KUKA::FRI;

#define DEFAULT_PORTID 30200

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fri_talker");

    if (argc > 1)
    {
        if ( strstr (argv[1],"help") != NULL)
        {
            printf(
                    "\nFRI FTW!\n\n"
                    "\tCommand line arguments:\n"
                    "\t1) remote hostname (optional)\n"
                    "\t2) port ID (optional)\n"
            );
            return 1;
        }
    }

    MojoClient lbrClient;
    printf("\nEnter Mojo Client Application\n");

    char* hostname = (argc >= 2) ? argv[1] : NULL;
    int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORTID;

    UdpConnection connection;

    ClientApplication app(connection, lbrClient);

    app.connect(port, hostname);

    bool success = true;
    while (ros::ok() and success)
    {
        // FRI loop start
        success = app.step();

        // check if we are in IDLE because the FRI session was closed
        if (lbrClient.robotState().getSessionState() == IDLE)
        {
            // In this demo application we simply quit.
            // Waiting for a new FRI session would be another possibility.
            break;
        }

        ros::spinOnce();

    }

    app.disconnect();
    printf("\nExit Mojo Client Application\n");

    return 0;
}