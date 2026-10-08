#ifndef UR_ROBOT_LLI_ADAPTIVECONTROLLER_H
#define UR_ROBOT_LLI_ADAPTIVECONTROLLER_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Math/EigenDefs.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tum_ics_lacquey_gripper_msgs/setGripperState.h>
#include <iostream>
#include <array>

/* Import Custom services, if needeed */

namespace tum_ics_ur_robot_lli 
{
    namespace RobotControllers
    
    {   


        class tum_ics_ur10_AdaptiveController : public ControlEffort
        {
            private :

                std::vector<Vector6d> desired_std_vector_for_cartesian_in_admittance;

                enum class ControllerState {
                    INIT_STATE,
                    JOINT_CONTROLLER,
                    CARTESIAN_CONTROLLER,
                    GRASP_CRATE,
                    LIFT_CRATE,
                    HOLD_POSITION,
                    ADMITTANCE_CONTROLLER,
                };

                // Required variablesss
                bool is_first_iteration_;
                
                // variable to call the service
                bool call_service_;

                int logger_counter_;

                // Variable to turn on admittance controller
                bool turn_on_admittance_controller_;
            
                // controller switching variables
                bool compute_moving_trajectory_1_;
                bool compute_moving_trajectory_2_;

                int iterator1;
                ControllerState controller_state_;
                ControllerState previous_controller_state_;
                
                double joint_spline_period_;
                double cartesian_trajectory_time_;
                double grasp_crate_time_;
                double lift_crate_time_;
                double force_limit_to_activate_admittance_control;
                double acceleration_limit_;

                // for Low Pass Filter

                double cutoff_freq_;
                double sampling_freq_;


                // Parameters for controllers
                double learning_rate_;
                double lambda_;
                double t_initial_;
                double t_final_;
                double initial_velocity_;
                double final_velocity_;
                double initial_acceleration_;
                double final_acceleration_;
                double joint_2_offset_;
                double joint_4_offset_;
                
                // position
                Vector3d ee_position_hold_;

                //  parameters for adaptive controllers
                Vector3d initial_cartesian_pos_;
                Vector3d final_cartesian_pos_;
                Vector3d integral_qi_cart_;

                Vector6d Delta_Q_;
                Vector6d Delta_Qp_;
                Vector6d integral_qi_joint_; 
                Vector6d q_start_;
                Vector6d q_start_new_;
                Vector6d q_goal_;
                Vector6d q_goal2_;

                Vector6d wrench_vector_;
                Vector6d wrench_des_;
                Vector6d wrench_vector_int_;
                Vector6d filtered_wrench_;

                VectorXd Theta_;
                        

                //Orientation control for cartesian controller
                Matrix3d desired_rotation_wrt_base_;
                Quaterniond desired_quaternion_wrt_base_;

                // for joint state controller
                Matrix6d Kp_;
                Matrix6d Kd_;
                Matrix6d Ki_;

                 // for cartesian controller
                Matrix3d Kp2_;
                Matrix3d Kd2_;
                Matrix3d Ki2_;
 

                // for admittance controller
                Matrix6d Kstiff_;
                Matrix6d Kdamp_;
                Matrix6d Lambda_adm_;
                int iteration_;

                JointState q_init_;
                JointState q_home_;
                JointState q_park_;

                // ros stuff
                ros::NodeHandle nh_;
                ros::ServiceServer turn_on_admittance_controller_service_;
                ros::Subscriber bw_lpf_wrench_sub_;
                ros::ServiceClient gripper_service_client_;
                ros::Publisher control_data_pub_;
                ros::Publisher ft_world_publisher;
                tf2_ros::TransformBroadcaster broadcaster_;

                ros::Publisher x_des_pub_;
                ros::Publisher x_dot_des_pub_;
                ros::Publisher x_dot_ref_pub_;
                ros::Publisher x_pub_;
                ros::Publisher x_dot_pub_;


            
            public :

                tum_ics_ur10_AdaptiveController(double weight = 1.0, const QString &name = "tum_ics_AdaptiveEffortController");

                ~tum_ics_ur10_AdaptiveController();

                void setQInit(const JointState &q_init);
                void setQHome(const JointState &q_home);
                void setQPark(const JointState &q_park);

            private :

                bool init();
                bool start();

                Vector6d update(const RobotTime &time, const JointState &joint_state);

                Matrix3d vector_to_skew(const Vector3d &vector);

                Vector3d rotationMatrixToRPY(const Matrix3d& R);

                Eigen::Quaterniond exp_map(const Vector3d& omega, double dt);

                // below is the callback for the FT sensor callback
                void bw_lpf_wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

                bool turn_on_admittance_controller_serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

                const Vector6d& getWrenchMatrix() const;

                void state_machine(const ControllerState controller_state, const RobotTime &time);

                Vector6d cartesian_trajectory(double &a, double &b, double &ini_vel, double &ini_acc, double &fin_vel, double &fin_acc, double &ti, double &tf);

                std::vector<Vector6d> cartesian_integration_for_admittance(Vector6d &X, Vector6d &Xdot, Vector6d &X_Ad_ddot, Quaterniond &qk, double &dt);

                bool stop();
                
        };

    } // namespace RobotControllers
    
} // namespace tumics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

