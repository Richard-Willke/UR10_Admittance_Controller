#include <array>
#include <cmath>
#include <algorithm>
#include <tum_ics_ur10_controller_tutorial/LowPassFilter.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Math/EigenDefs.h>

using namespace Tum;

namespace tum_ics_ur_robot_lli
{

    namespace RobotControllers
    {

		ButterworthLowPass::ButterworthLowPass(double cutoff, double sampling) : cutoff_freq(cutoff), sampling_freq(sampling),
		 wrench_vector_(Vector6d::Zero()) 
		{
			computeCoefficients();
			for (auto& arr : x) arr.fill(0.0);
			for (auto& arr : y) arr.fill(0.0);
			filtered_forces_.fill(0.0);
			wrench_subscriber = nh_.subscribe("/schunk_netbox/raw", 10, &ButterworthLowPass::ft_sensor_wrenchCallback, this);
			ft_filtered_sensor_publisher = nh_.advertise<geometry_msgs::WrenchStamped>("/filtered_wrench/raw", 10);
		}
		void ButterworthLowPass::ft_sensor_wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
		{
			// Extract torque and forces [force, torque]

			wrench_vector_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
							msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

			std::array<double, 6> input;

			for(int i = 0; i<6; i++)
            {
                input[i] = wrench_vector_[i];
            }

			filter(input, filtered_forces_);


			geometry_msgs::WrenchStamped wrench_s_filtered;
            wrench_s_filtered.header.frame_id = "ft_sensor_link";
            wrench_s_filtered.header.stamp = ros::Time::now();
            wrench_s_filtered.wrench.force.x = filtered_forces_[0];
            wrench_s_filtered.wrench.force.y = filtered_forces_[1];
            wrench_s_filtered.wrench.force.z = filtered_forces_[2];
            wrench_s_filtered.wrench.torque.x = filtered_forces_[3];
            wrench_s_filtered.wrench.torque.y = filtered_forces_[4];
            wrench_s_filtered.wrench.torque.z = filtered_forces_[5];
            ft_filtered_sensor_publisher.publish(wrench_s_filtered);

		}

		void ButterworthLowPass::computeCoefficients() {
			double omega = 2.0 * M_PI * cutoff_freq / sampling_freq;
			double tan_omega = std::tan(omega / 2.0);
			double sqrt2 = std::sqrt(2.0);

			double a0 = 1.0 + sqrt2 * tan_omega + tan_omega * tan_omega;
			a[0] = (tan_omega * tan_omega) / a0;
			a[1] = 2.0 * a[0];
			a[2] = a[0];

			b[0] = 1.0;
			b[1] = 2.0 * (tan_omega * tan_omega - 1.0) / a0;
			b[2] = (1.0 - sqrt2 * tan_omega + tan_omega * tan_omega) / a0;
		}

		void ButterworthLowPass::filter(const std::array<double, 6>& input, std::array<double, 6>& output) {
			for (int i = 0; i < 6; ++i) {
				x[i][2] = x[i][1];
				x[i][1] = x[i][0];
				x[i][0] = input[i];

				y[i][2] = y[i][1];
				y[i][1] = y[i][0];

				y[i][0] = a[0] * x[i][0] + a[1] * x[i][1] + a[2] * x[i][2]
						- b[1] * y[i][1] - b[2] * y[i][2];

				output[i] = y[i][0];
			}
		}


}

}
