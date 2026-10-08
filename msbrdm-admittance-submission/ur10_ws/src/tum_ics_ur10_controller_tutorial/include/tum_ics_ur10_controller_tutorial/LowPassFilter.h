#ifndef LowPassFilter_H
#define LowPassFilter_H

#include <cmath>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <iostream>
#include <array>
#include <vector>
#include <Math/EigenDefs.h>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <geometry_msgs/WrenchStamped.h>
#include <Math/EigenDefs.h>

namespace tum_ics_ur_robot_lli
{

    namespace RobotControllers
    {

		class ButterworthLowPass {
		private:
			double cutoff_freq;
			double sampling_freq;
			double a[3], b[3];
			std::array<std::array<double, 3>, 6> x{}, y{}; // History for 6 channels
			Vector6d wrench_vector_;
			std::array<double, 6> filtered_forces_;
			void computeCoefficients(); // Compute filter coefficients
			ros::NodeHandle nh_;
			ros::Subscriber wrench_subscriber;
			ros::Publisher ft_filtered_sensor_publisher;


		public:
			ButterworthLowPass(double cutoff, double sampling);
			void ft_sensor_wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
			void filter(const std::array<double, 6>& input, std::array<double, 6>& output);
		};

	}

}
#endif // BUTTERWORTH_LOWPASS_H
