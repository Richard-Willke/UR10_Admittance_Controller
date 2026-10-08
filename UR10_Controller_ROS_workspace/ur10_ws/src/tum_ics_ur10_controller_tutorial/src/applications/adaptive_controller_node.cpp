#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <tum_ics_ur10_controller_tutorial/adaptive_controller.h>
#include <QApplication>
#include <std_msgs/Int32.h>


int main(int argc, char **argv)
{

    QApplication a(argc, argv);

    ros::init(argc, argv, "testRobotArmClass", ros::init_options::AnonymousName);
    QString configFilePath=argv[1];
    ROS_INFO_STREAM("Config File:" << configFilePath.toStdString().c_str());

    tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);
    if(!robot.init())
    {
        return -1;
    }

    ROS_INFO_STREAM("Create Controller");
    tum_ics_ur_robot_lli::RobotControllers::tum_ics_ur10_AdaptiveController controller(1.0, "tum_ics_AdaptiveEffortController");
    ROS_INFO_STREAM("Add Controller");
    if(!robot.add(&controller))
    {

        return -1;

    }

    controller.setQHome(robot.qHome());
    controller.setQPark(robot.qPark());


    //RUN the Node
    ROS_INFO_STREAM("Start Robot");
    robot.start();

    ROS_INFO_STREAM("Start main Thread");
    ros::spin();

    ROS_INFO_STREAM("main: Stopping Robot Arm");
    robot.stop();

    ROS_INFO_STREAM("main: Stopped");

    return 0;

}