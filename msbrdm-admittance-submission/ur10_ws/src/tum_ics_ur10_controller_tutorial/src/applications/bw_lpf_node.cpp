#include <tum_ics_ur10_controller_tutorial/LowPassFilter.h>
#include <geometry_msgs/WrenchStamped.h>



int main(int argc, char **argv)
{


    ros::init(argc, argv, "bw_lpf_node", ros::init_options::AnonymousName);

    double cutoff_freq_ = 10.0;
    double sampling_freq_ = 200.0;


    tum_ics_ur_robot_lli::RobotControllers::ButterworthLowPass lpf(cutoff_freq_, sampling_freq_);

    ros::spin();

    ROS_INFO_STREAM("Butterworth: Stopped");

    return 0;

}