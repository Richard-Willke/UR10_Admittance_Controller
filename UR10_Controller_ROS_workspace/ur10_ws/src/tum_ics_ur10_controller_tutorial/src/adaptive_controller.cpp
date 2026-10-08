#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Math/EigenDefs.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <tum_ics_lacquey_gripper_msgs/setGripperState.h>
#include <tum_ics_ur10_controller_tutorial/adaptive_controller.h>
#include <tum_ics_ur10_controller_tutorial/ur10_model.h>
#include <tum_ics_ur10_controller_tutorial/LowPassFilter.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>

namespace tum_ics_ur_robot_lli
{

    namespace RobotControllers
    {   
        /* Constructor List initialisation */
        tum_ics_ur10_AdaptiveController::tum_ics_ur10_AdaptiveController(double weight, const QString &name): 
          ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
          is_first_iteration_(true),
          call_service_(true),
          logger_counter_(0),
          turn_on_admittance_controller_(false),
          compute_moving_trajectory_1_(true),
          compute_moving_trajectory_2_(true),
          iterator1(0),
          controller_state_(ControllerState::INIT_STATE),
          previous_controller_state_(ControllerState::INIT_STATE),

          joint_spline_period_(0.0),
          cartesian_trajectory_time_(0.0),
          grasp_crate_time_(0.0),
          lift_crate_time_(0.0),
          force_limit_to_activate_admittance_control(8.0),
          acceleration_limit_(0.05),

          learning_rate_(0.00),
          lambda_(0.00),
          t_initial_(0.0),
          t_final_(0.0),
          initial_velocity_(0.0), //always 0 
          final_velocity_(0.0), //always 0 
          initial_acceleration_(0.0), //always 0 
          final_acceleration_(0.0), //always 0 
          joint_2_offset_(0.0),
          joint_4_offset_(0.0),
          
          ee_position_hold_(Vector3d::Zero()),
          initial_cartesian_pos_(Vector3d::Zero()),
          final_cartesian_pos_(Vector3d::Zero()),
          integral_qi_cart_(Vector3d::Zero()),
          
          Delta_Q_(Vector6d::Zero()),
          Delta_Qp_(Vector6d::Zero()),
          integral_qi_joint_(Vector6d::Zero()),
          q_start_(Vector6d::Zero()),
          q_start_new_(Vector6d::Zero()),
          q_goal_(Vector6d::Zero()),
          q_goal2_(Vector6d::Zero()),
          wrench_vector_(Vector6d::Zero()),
          wrench_vector_int_(Vector6d::Zero()),
          filtered_wrench_(Vector6d::Zero()),

          Theta_(VectorXd::Zero(163)),

          // for joint space controller
          Kp_(Matrix6d::Zero()),
          Kd_(Matrix6d::Zero()),
          Ki_(Matrix6d::Zero()),
        
          // for cartesian controller
          Kp2_(Matrix3d::Zero()),
          Kd2_(Matrix3d::Zero()),
          Ki2_(Matrix3d::Zero()),
          
          // For force or admittance control  
          Kstiff_(Matrix6d::Zero()),
          Kdamp_(Matrix6d::Zero()),
          Lambda_adm_(Matrix6d::Zero()),

          desired_rotation_wrt_base_(Matrix3d::Zero()),
          desired_quaternion_wrt_base_(Vector4d::Zero())

        
        {
            control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("adaptive_controller_data", 1);
            bw_lpf_wrench_sub_ = nh_.subscribe("/filtered_wrench/raw", 10, &tum_ics_ur10_AdaptiveController::bw_lpf_wrenchCallback, this);
            gripper_service_client_ = nh_.serviceClient<tum_ics_lacquey_gripper_msgs::setGripperState>("/setGripperState");
            turn_on_admittance_controller_service_ = nh_.advertiseService("/toggle_admittance_controller", &tum_ics_ur10_AdaptiveController::turn_on_admittance_controller_serviceCallback, this);
            ft_world_publisher = nh_.advertise<geometry_msgs::WrenchStamped>("Wrench_in_World", 10);
            
            wrench_des_ << 1.5, 6.4, 24.0, 0.0845, 0.722, -0.231;
            desired_std_vector_for_cartesian_in_admittance.resize(3);

            x_des_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("x_des_", 1);
            x_dot_des_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("v_des_", 1);
            x_dot_ref_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("v_ref_", 1);
            x_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("x_", 1);
            x_dot_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("v_", 1);
            
            iteration_ = 0;
            

        }

        // Destructor for the class
        tum_ics_ur10_AdaptiveController::~tum_ics_ur10_AdaptiveController()
        {

        }

        void tum_ics_ur10_AdaptiveController::setQInit(const JointState &q_init)
        {
            q_init_ = q_init;
        }

        void tum_ics_ur10_AdaptiveController::setQHome(const JointState &q_home)
        {
            q_home_ = q_home;
        }

        void tum_ics_ur10_AdaptiveController::setQPark(const JointState &q_park)
        {
            q_park_ = q_park;
        }


        bool tum_ics_ur10_AdaptiveController::init()
        {
            ROS_WARN_STREAM("AdpativeControl::init");
            std::vector<double> vec;

            // check namespace
            std::string ns = "~adaptive_ctrl";
            if (!ros::param::has(ns))
            {
                ROS_ERROR_STREAM("AdaptiveControl init(): Control gains not defined in:" << ns);
                m_error = true;
                return false;
            }


            // Joint space gains
            ros::param::get(ns + "/gains_d", vec);
            if (vec.size() < STD_DOF)
            {
                ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            
            for (size_t i = 0; i < STD_DOF; i++)
            {
                Kd_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Kd: \n" << Kd_);

            ros::param::get(ns + "/gains_p", vec);
            if (vec.size() < STD_DOF)
            {
                ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++)
            {
                Kp_(i, i) = vec[i]; //removed divided by Kd(i,i)? / Kd(i, i)
            }
            ROS_WARN_STREAM("Kp: \n" << Kp_);


            ros::param::get(ns + "/gains_i", vec);
            if (vec.size() < STD_DOF)
            {
                ROS_ERROR_STREAM("gains_i: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < STD_DOF; i++)
            {
                Ki_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Ki: \n" << Ki_);


            // carteisan space gains
            ros::param::get(ns + "/gains_p_cart_", vec);
            if (vec.size() < 3) // TODO: prev. < STD_DOF
            {
                ROS_ERROR_STREAM("gains_p_cart_: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < 3 ; i++)
            {
                Kp2_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Kp2: \n" << Kp2_);


            // D cartesian
            ros::param::get(ns + "/gains_d_cart_", vec);
            if (vec.size() < 3)
            {
                ROS_ERROR_STREAM("gains_d_cart_: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < 3; i++)
            {
                Kd2_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Kd2: \n" << Kd2_);


            // I cartesian
            ros::param::get(ns + "/gains_i_cart_", vec);
            if (vec.size() < 3)
            {
                ROS_ERROR_STREAM("gains_i_cart_: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < 3; i++)
            {
                Ki2_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Ki2: \n" << Ki2_);


            // GOAL for Joint space
            ros::param::get(ns + "/goal", vec);
            if (vec.size() < STD_DOF)
            {
                ROS_ERROR_STREAM("goal: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++)
            {
                q_goal_(i) = vec[i];
            }
            
            ros::param::get(ns + "/goal2", vec);
            if (vec.size() < STD_DOF)
            {
                ROS_ERROR_STREAM("goal2: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++)
            {
                q_goal2_(i) = vec[i];
            }
            
            // controller switching variables
            ros::param::get(ns + "/time", joint_spline_period_);
            if (!(joint_spline_period_ > 0))
            {
                ROS_ERROR_STREAM("joint_spline_period_: is negative:" << joint_spline_period_);
                joint_spline_period_ = 30.0;
            }

            // time to reach crate 
            ros::param::get(ns + "/catesian_trajectory_time", cartesian_trajectory_time_);
            if (!(cartesian_trajectory_time_ > 0))
            {
                ROS_ERROR_STREAM("catesian_trajectory_time_: is negative:" << cartesian_trajectory_time_);
                cartesian_trajectory_time_ = 40.0;
            }

            //Time to grasp crate
            ros::param::get(ns + "/grasp_crate_time", grasp_crate_time_);
            if (!(grasp_crate_time_ > 0))
            {
                ROS_ERROR_STREAM("grasp_crate_time_: is negative:" << grasp_crate_time_);
                grasp_crate_time_ = 42.0;
            }

            //Time to lift crate
            ros::param::get(ns + "/lift_crate_time", lift_crate_time_);
            if (!(lift_crate_time_ > 0))
            {
                ROS_ERROR_STREAM("lift_crate_timesetGripperState_: is negative:" << lift_crate_time_);
                lift_crate_time_ = 57.0;
            }


            //Lambda
            ros::param::get(ns + "/lambda", lambda_);
            if (!(lambda_) )
            {
                ROS_ERROR_STREAM("Lambda not defined, setting up a value");
                lambda_ = 0.001;
            }


            // learning_rate
            ros::param::get(ns + "/learning_rate", learning_rate_);
            if (!(learning_rate_))
            {
                ROS_ERROR_STREAM("Learning rate not defined, setting up a value");
                learning_rate_ = 0.0001;
            }


            // parameters for admittance controller ( 6d position and orientation)

            ros::param::get(ns + "/Kstiff", vec);
            if (vec.size() < 6)
            {
                ROS_ERROR_STREAM("Kstiff : wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < 6; i++)
            {
                Kstiff_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Kstiff: \n" << Kstiff_);

            ros::param::get(ns + "/Kdamp", vec);
            if (vec.size() < 6)
            {
                ROS_ERROR_STREAM("Kdamp : wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < 6; i++)
            {
                Kdamp_(i, i) = vec[i];
            }

            ros::param::get(ns + "/lambda_adm", vec);
            if (vec.size() < 6)
            {
                ROS_ERROR_STREAM("lambda_adm : wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < 6; i++)
            {
                Lambda_adm_(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Lambda_adm: \n" << Lambda_adm_);




            
            ROS_WARN_STREAM("Kdamp: \n" << Kdamp_);
            ROS_WARN_STREAM("Goal [DEG]: \n" << q_goal_.transpose());
            ROS_WARN_STREAM("Total Time for Joint Spline[s]: " << joint_spline_period_);
            q_goal_ = DEG2RAD(q_goal_);
            ROS_WARN_STREAM("Goal [RAD]: \n" << q_goal_.transpose());


            return true;
        }

        bool tum_ics_ur10_AdaptiveController::start()

        {
            ROS_INFO_ONCE("AdaptiveControl::start");
            return true;
        }


        Vector3d tum_ics_ur10_AdaptiveController::rotationMatrixToRPY(const Matrix3d& R) 
        {
            
            Eigen::Quaterniond q(R);
            Eigen::Vector3d eulerAngles = q.toRotationMatrix().eulerAngles(0, 1, 2);

            return eulerAngles;  // Returns roll, pitch, yaw (in radians)
        }



        void tum_ics_ur10_AdaptiveController::bw_lpf_wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
        {
            // Extract torque and forces [force, torque]

            wrench_vector_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                            msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

        }


        // Function to get the stored wrench vector [force, torque]
        // Example how to get wrench_vector_ : const Vector6d& wrench = class_instance.getWrenchMatrix(); // NOTE: from within this file, leave out "class_instance."
        // Note since a const reference is returned, wrench value cannot be altered i.e is constant
        const Vector6d& tum_ics_ur10_AdaptiveController::getWrenchMatrix() const
        {
            return wrench_vector_;
        }

        Matrix3d tum_ics_ur10_AdaptiveController::vector_to_skew(const Vector3d &vector)
        {
            Matrix3d skew_mat = Matrix3d::Zero();

            skew_mat <<   0.0,     -vector[2],  vector[1], 
                         vector[2],    0.0,    -vector[0],
                        -vector[1], vector[0],    0.0;
  
            return skew_mat;

        }

        bool tum_ics_ur10_AdaptiveController::turn_on_admittance_controller_serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
        {
            if (req.data) 
            {
                turn_on_admittance_controller_ = true;
                res.success = true;
                res.message = "Admittance controller will be turned on now";
                integral_qi_joint_.setZero();
            }
            else
            {
                turn_on_admittance_controller_ = false;
                res.success = true;
                res.message = "Admittance controller will not be started";
            }

            // ROS_WARN_STREAM("Received request: " << (req.data ? "true" : "false") 
            //                 << " | Controller State: " << (turn_on_admittance_controller_ ? "ON" : "OFF"));

            return true;
        }

        // for position we are only integrating just velocity and not acceleration
        // function is for both position and Orientation
        std::vector<Vector6d> tum_ics_ur10_AdaptiveController::cartesian_integration_for_admittance(Vector6d &X, Vector6d &Xdot, Vector6d &X_Ad_ddot, Quaterniond &qk, double &dt)
        {

            Vector6d X_new, X_dot_new;

            //Linear part
            X_dot_new.head(3) = Xdot.head(3) + (X_Ad_ddot.head(3) * dt);
            X_new.head(3) = X.head(3) + (X_dot_new.head(3) * dt); 
            
            //Angular part
            X_dot_new.tail(3) = Xdot.tail(3) + (X_Ad_ddot.tail(3) * dt);
            Eigen::Quaterniond quat_from_exp_w = exp_map(X_dot_new.tail(3), dt);
            Eigen::Quaterniond qk_plus_1 = quat_from_exp_w * qk;
            Eigen::Matrix3d rotationMatrix = qk_plus_1.toRotationMatrix();
            X_new.tail(3) = rotationMatrix.eulerAngles(0, 1, 2); // Should be rpy
            
            desired_std_vector_for_cartesian_in_admittance[0] = X_new;
            desired_std_vector_for_cartesian_in_admittance[1] = X_dot_new;
            desired_std_vector_for_cartesian_in_admittance[2] = X_Ad_ddot;
            return desired_std_vector_for_cartesian_in_admittance;
        }

        /* This function is from the paper I read */
        Eigen::Quaterniond tum_ics_ur10_AdaptiveController::exp_map(const Vector3d& omega, double dt) 
        {
            // Calculate the magnitude of the angular velocity
            double norm_w = omega.norm();
            double theta = norm_w * dt; // Angular displacement over the time step

            if (norm_w < 1e-10) {
                return Eigen::Quaterniond(1, 0, 0, 0); // Identity quaternion
            }

            // Normalize the angular velocity vector (unit vector)
            Eigen::Vector3d axis = omega.normalized();

            // Apply the exponential map formula to compute the quaternion
            double sin_half_theta = std::sin(theta / 2.0);
            double cos_half_theta = std::cos(theta / 2.0);

            // The vector part of the quaternion
            Eigen::Vector3d vector_part = sin_half_theta * axis;

            // Return the resulting quaternion: [cos(θ/2), sin(θ/2) * axis]
            return Eigen::Quaterniond(cos_half_theta, vector_part.x(), vector_part.y(), vector_part.z());
        }

        // Important for all are Jacobians v has the first three rows ands w has the bottom3 rows 
        Vector6d tum_ics_ur10_AdaptiveController::cartesian_trajectory(double &a, double &b, double &ini_vel, double &ini_acc, double &fin_vel, double &fin_acc, double &ti, double &tf)
        {

            Matrix6d T = Matrix6d::Zero();
            Vector6d x = Vector6d::Zero();
            Vector6d traj = Vector6d::Zero();

            // RECHCEK THE FORMULA FOR T

            T << 1, ti, pow(ti,2), pow(ti,3), pow(ti,4), pow(ti,5),
                1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5),
                0, 1, 2*ti, 3*pow(ti,2), 4*pow(ti,3), 5*pow(ti,4),
                0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4),
                0, 0, 2, 6*ti, 12*pow(ti,2), 20*pow(ti,3),
                0, 0, 2, 6*tf, 12*pow(tf,2), 20*pow(tf,3);
        
            x << a, b, ini_vel, fin_vel, ini_acc, fin_acc;

            
            if (T.determinant() != 0) {

                traj = T.lu().solve(x);

            } else {

                traj = T.completeOrthogonalDecomposition().pseudoInverse() * x;

            }

            return traj;

        }
        
        void tum_ics_ur10_AdaptiveController::state_machine(const ControllerState controller_state, const RobotTime &time)
        {
            
            double t_elapsed = time.tD();
            switch (controller_state)
            {
                case ControllerState::INIT_STATE:
                    if (t_elapsed > 0.0)
                    {
                        controller_state_ = ControllerState::JOINT_CONTROLLER;
                        ROS_INFO_ONCE("AdaptiveController, started Joint Space Controller!");
                    }
                    break;
                        
                case ControllerState::JOINT_CONTROLLER:
                    if (t_elapsed > joint_spline_period_)
                    {
                        controller_state_ = ControllerState::CARTESIAN_CONTROLLER;
                        ROS_INFO_ONCE("AdaptiveController, switched from Joint Space Controller to Cartesian Space Controller!");
                    }
                    break;
                
                case ControllerState::CARTESIAN_CONTROLLER:
                    if (t_elapsed > cartesian_trajectory_time_)
                    {
                        controller_state_ = ControllerState::GRASP_CRATE;
                        ROS_INFO_ONCE("AdaptiveController, robot will now grasp the crate!");
                    }
                    break;

                case ControllerState::GRASP_CRATE:
                    if (t_elapsed > grasp_crate_time_)
                        {
                            controller_state_ = ControllerState::LIFT_CRATE;
                            ROS_INFO_ONCE("AdaptiveController, switched from Cartesian Space Controller to Joint Space Controller!");
                        }
                        break;
                
                /* Important Task */
                /* We should use a service to switch, when called then admittance controller state should be activated */
                //TODO: Add gripper controller and make admittance controller switch to be event based instead of time based i.e. through ROS service
                case ControllerState::LIFT_CRATE: 
                    if (t_elapsed > lift_crate_time_ )
                    {
                        controller_state_ = ControllerState::HOLD_POSITION;
                        ROS_INFO_ONCE("AdaptiveController, switched from Force Controller to Admittance Controller!");
                    }
                    break;
                case ControllerState::HOLD_POSITION: 
                    if (turn_on_admittance_controller_)
                    {
                        controller_state_ = ControllerState::ADMITTANCE_CONTROLLER;
                        ROS_INFO_ONCE("AdaptiveController, switched from Force Controller to Admittance Controller!");
                    }
                    break;
                case ControllerState::ADMITTANCE_CONTROLLER: 
                    if (!turn_on_admittance_controller_)
                    {
                        controller_state_ = ControllerState::HOLD_POSITION;
                        ROS_INFO_ONCE("AdaptiveController, switched from Admittance Controller to Hold Position!");
                    }
                    break;
                default:
                        ROS_INFO_ONCE("AdaptiveController, State Machine Default!");

                    break;  
            }
                    
            if (controller_state_ != previous_controller_state_)
            { 
            }
        }

        

        Vector6d tum_ics_ur10_AdaptiveController::update(const RobotTime &time, const JointState &state)
        {
            // ti_ = time.tD()  METHOD TO Access time 

            static Vector6d Qpr_last;
            static double previous_time;

            if (is_first_iteration_)
            { 
                q_start_ = state.q;
        
                ROS_WARN_STREAM("START [DEG]: \n" << q_start_.transpose());
                is_first_iteration_ = false;
                previous_time = time.tD();
                Qpr_last = Vector6d::Zero();
                return Vector6d::Zero();
            }

            // variables
            Vector6d tau = Vector6d::Zero();
            Vector6d Qpr = Vector6d::Zero();
            Vector6d Qppr = Vector6d::Zero();
            Vector6d Sq = Vector6d::Zero();

            //tum_ics_ur_robot_msgs::ControlData msg;
            Vector6d q = state.q;
            Vector6d qp = state.qp;
            Vector6d qpp = state.qpp;

            // IMPORTANT NOTE: WE DO NOT ROTATE VELOCITY THAT IS CALCULATED FROM JACOBIAN AS
            //WORLD AND BASE FRAME ARE SAME
            Matrix4d T0_W;
            T0_W << 1,    0,    0,   0,
                    0,    1,    0,   0,
                    0,    0,    1,   0,
                    0,    0,    0,   1;
            
            Matrix3d R0_W = T0_W.block(0,0,3,3);
            
            // TODO
            Matrix4d Tee_0 = get_ee_forward_kinematics(q);
            Matrix4d Tee_W = Matrix4d::Identity();  
            Tee_W << T0_W * Tee_0;
            
            //TODO DO WE NEED TO CHANGE TO WORLD OR BASE??
            Matrix3d Ree_W = Tee_W.block(0,0,3,3);
            Vector3d EE_current_position = Tee_W.block(0,3,3,1);

            Vector6d ee_current_pose = Vector6d::Zero();
            ee_current_pose.head(3) = Tee_W.block(0,3,3,1);
            ee_current_pose.tail(3) = rotationMatrixToRPY(Ree_W);

            Matrix6d jacobian = get_ee_jacobian(q);
            Vector6d end_effector_velocity = jacobian * qp;
            
            // Because Jacobian has angular part on top and linear part on bottom
            Vector3d linear_ee_velocity = R0_W * end_effector_velocity.head(3);  // VELOCITY ROTATION
            
            // publish frame EE
            Quaterniond quat(Ree_W);
            geometry_msgs::TransformStamped EE;
            EE.header.stamp = ros::Time::now();
            EE.header.frame_id = "world";
            EE.child_frame_id = "END_EFFECTOR";
            EE.transform.translation.x = EE_current_position(0);
            EE.transform.translation.y = EE_current_position(1);
            EE.transform.translation.z = EE_current_position(2);
            EE.transform.rotation.x = quat.x();
            EE.transform.rotation.y = quat.y();
            EE.transform.rotation.z = quat.z();
            EE.transform.rotation.w = quat.w();
            broadcaster_.sendTransform(EE);
            

            static Vector6d ax = Vector6d::Zero();
            static Vector6d ay = Vector6d::Zero();
            static Vector6d az = Vector6d::Zero();
            
            previous_controller_state_ = controller_state_;
            state_machine(controller_state_, time);
            
            // check if time differnce uis being calculated correctly
            double t = time.tD();
            double time_difference = t - previous_time;

            
            Eigen::Matrix3d R_ee_ft;
            R_ee_ft <<  0, 1, 0,
                        -1, 0, 0,
                        0, 0, 1;

             /* Get wrench and add a low pass filter to it */
            const Vector6d& filtered_wrench = getWrenchMatrix();

            // Adjoint to Rotate wrench from sensor to End Effector
            Eigen::Matrix<double, 6, 6> AdjT_sen_ee, AdjT_ee_w;
            AdjT_sen_ee.setZero();
            AdjT_sen_ee.block<3, 3>(0, 0) = R_ee_ft.transpose();
            AdjT_sen_ee.block<3, 3>(3, 3) = R_ee_ft.transpose(); 

            AdjT_ee_w.setZero();
            AdjT_ee_w.block<3, 3>(0, 0) = Ree_W;
            Matrix3d translation_skew = vector_to_skew(EE_current_position);
            AdjT_ee_w.block<3, 3>(3, 0) = translation_skew * Ree_W ;
            AdjT_ee_w.block<3, 3>(3, 3) = Ree_W;
            Vector6d wrench_in_world =  AdjT_ee_w * AdjT_sen_ee * filtered_wrench;  //AdjT_ee_w * 
            wrench_in_world.tail(3) = Vector3d::Zero();
            int maxIndex;
            double maxAbs = wrench_in_world.cwiseAbs().maxCoeff(&maxIndex);

            for (int i = 0; i < 3; ++i){
                if(i != maxIndex){
                    wrench_in_world(i) = 0.0;
                }
            }
            
            double max = 50;
            double min = -50;

            wrench_in_world = wrench_in_world.cwiseMax(min).cwiseMin(max);
            double current_force = wrench_in_world.head(3).norm();

            geometry_msgs::WrenchStamped wrench_s_filtered;
            wrench_s_filtered.header.frame_id = "world";
            wrench_s_filtered.header.stamp = ros::Time::now();
            wrench_s_filtered.wrench.force.x = wrench_in_world[0];
            wrench_s_filtered.wrench.force.y = wrench_in_world[1];
            wrench_s_filtered.wrench.force.z = wrench_in_world[2];
            wrench_s_filtered.wrench.torque.x = wrench_in_world[3];
            wrench_s_filtered.wrench.torque.y = wrench_in_world[4];
            wrench_s_filtered.wrench.torque.z = wrench_in_world[5];
            ft_world_publisher.publish(wrench_s_filtered);


            if (controller_state_ == ControllerState::INIT_STATE){
                return Vector6d::Zero();
            }
            else if (controller_state_ == ControllerState::JOINT_CONTROLLER)
            {
                VVector6d vel_desired;

                vel_desired = getJointPVT5(q_start_, q_goal_, time.tD(), joint_spline_period_ );

                Vector6d qd = vel_desired[0];
                Vector6d qdp = vel_desired[1];
                Vector6d qdpp = vel_desired[2];

                // Error 
                Delta_Q_ = q - qd;
                Delta_Qp_ = qp - qdp;

                // JOINT SPACE PID
                integral_qi_joint_ = integral_qi_joint_ + Delta_Q_ * time_difference;
                Qpr = qdp - Kp_ * Delta_Q_ - Ki_ * integral_qi_joint_;
                Qppr = qdpp - Kp_ * (qp - Qpr);
                Sq = qp - Qpr;

                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); //* time_difference;
                tau = -Kd_ * Sq  + Yr * Theta_;


            }

            else if(controller_state_== ControllerState::CARTESIAN_CONTROLLER) 
            {
                /* Implement a Cartesian Controller that takes the robot close to the crate and in grasping position */

                if (compute_moving_trajectory_1_){

                    t_initial_ = joint_spline_period_;
                    t_final_ = cartesian_trajectory_time_ ;

                    ee_position_hold_ = Tee_W.block(0,3,3,1);
                    
                    /* Goal is right in front of the robot */
                    double x ,y;
                    x = 0.0;
                    y = 0.0;
                    double z = 0.20;
        
                    Vector4d point = Vector4d::Ones();
                    point << x, y, z, 1.0;
        
                    Vector4d point_in_world_frame = Vector4d::Zero();
        
                    point_in_world_frame = Tee_W * point;
                    final_cartesian_pos_ << point_in_world_frame[0],point_in_world_frame[1],point_in_world_frame[2];


                    initial_cartesian_pos_ = EE_current_position;
                    
                    // Save the Orientation when Joint State controller ends, as we always want to maintain this orientation
                    desired_rotation_wrt_base_ = Ree_W;
                    desired_quaternion_wrt_base_ = Quaterniond(Ree_W);

                    ax = cartesian_trajectory(initial_cartesian_pos_[0], final_cartesian_pos_[0] , initial_velocity_, initial_acceleration_, final_velocity_, final_acceleration_, t_initial_, t_final_);
                    ay = cartesian_trajectory(initial_cartesian_pos_[1], final_cartesian_pos_[1] , initial_velocity_, initial_acceleration_, final_velocity_, final_acceleration_, t_initial_, t_final_);
                    az = cartesian_trajectory(initial_cartesian_pos_[2], final_cartesian_pos_[2] , initial_velocity_, initial_acceleration_, final_velocity_, final_acceleration_, t_initial_, t_final_);
                    compute_moving_trajectory_1_ = false;
        
                  }
        

                geometry_msgs::TransformStamped Cartesian_goal;
                Cartesian_goal.header.stamp = ros::Time::now();
                Cartesian_goal.header.frame_id = "world";
                Cartesian_goal.child_frame_id = "Crate";
                Cartesian_goal.transform.translation.x = final_cartesian_pos_(0);
                Cartesian_goal.transform.translation.y = final_cartesian_pos_(1);
                Cartesian_goal.transform.translation.z = final_cartesian_pos_(2);
                Cartesian_goal.transform.rotation.x = desired_quaternion_wrt_base_.x();
                Cartesian_goal.transform.rotation.y = desired_quaternion_wrt_base_.y();
                Cartesian_goal.transform.rotation.z = desired_quaternion_wrt_base_.z();
                Cartesian_goal.transform.rotation.w = desired_quaternion_wrt_base_.w();
                broadcaster_.sendTransform(Cartesian_goal);
                

                Matrix<double, 3, 6> T_coefficients;

                T_coefficients << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5),
                                0, 1, 2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
                                0, 0, 2, 6 * t, 12 * pow(t,2), 20 * pow(t,3);


                Vector3d X_desired = Vector3d::Zero();
                Vector3d X_desired_velocity = Vector3d::Zero();
                Vector3d X_desired_acceleration = Vector3d::Zero();


                X_desired << T_coefficients.row(0) * ax, T_coefficients.row(0) * ay, T_coefficients.row(0) * az;
                X_desired_velocity << T_coefficients.row(1) * ax, T_coefficients.row(1) * ay, T_coefficients.row(1) * az;
                X_desired_acceleration << T_coefficients.row(2) * ax, T_coefficients.row(2) * ay, T_coefficients.row(2) * az;

                

                Matrix6d Jacobian = get_ee_jacobian(q);
                Matrix6d Jacobian_dot = get_ee_jacobian_dot(q, qp);
                Matrix6d Jacobian_inv = Jacobian.transpose() *(Jacobian* Jacobian.transpose() + lambda_ * Matrix<double,6,6>::Identity()).completeOrthogonalDecomposition().pseudoInverse();
            

                // Error Linear and Angular
                Vector3d Delta_X = EE_current_position - X_desired;
                Quaterniond q_current(Ree_W);
                Quaterniond q_err = desired_quaternion_wrt_base_ * q_current.inverse();
                q_err.normalize();
                Eigen::AngleAxisd angle_axis(q_err);
                // Rotation magnitude
                double theta = angle_axis.angle();  
                // Rotation axis
                Eigen::Vector3d axis = angle_axis.axis();  
                // If theta is too small
                if (theta < 1e-6) {
                    theta = 0.0;
                }
                // Compute angular error e_ω = θ * v
                Eigen::Vector3d angular_error = theta * axis;
                                
                Vector3d Delta_X_velocity = linear_ee_velocity - X_desired_velocity; 
                integral_qi_cart_ = integral_qi_cart_ + (Delta_X  * time_difference);

                // Reference Velocity
                Vector3d X_reference_linear_velocity = X_desired_velocity - Kp2_ * Delta_X - Ki2_ * integral_qi_cart_;
                Vector6d X_reference_velocity = Vector6d::Zero();
                X_reference_velocity.head(3) = X_reference_linear_velocity;
                X_reference_velocity.tail(3) = angular_error;
                
                Vector6d new_acc = Vector6d::Zero();
                new_acc << X_desired_acceleration[0], X_desired_acceleration[1], X_desired_acceleration[2], 0.0, 0.0, 0.0;
                Qpr = Jacobian_inv * X_reference_velocity;
                Qppr = Jacobian_inv * (new_acc - (Jacobian_dot * qp));

                // Calculate Sq
                Sq = qp - Qpr;
                double w  = sqrt((Jacobian * Jacobian.transpose()).determinant());
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); 
            
                tau = -Kd_ * Sq + Yr * Theta_;


            }
            else if(controller_state_ == ControllerState::GRASP_CRATE)
            {
            

                if(call_service_)
                {   
                    /* Call service to grasp the crate */
                    tum_ics_lacquey_gripper_msgs::setGripperState srv;
                    srv.request.newState = "close";  

                    ROS_INFO_ONCE("AdaptiveController, Calling the service to grasp the crate");

                    if (gripper_service_client_.call(srv)) 
                    {
                        ROS_INFO_ONCE("Service call succeeded");
                        ROS_INFO_ONCE("AdaptiveController, Robot is grasping the crate now");
                    } 
                    else 
                    {
                        ROS_INFO_ONCE("Failed to call service setGripperState");
                    }

                    call_service_ = false;

                }

                ROS_INFO_ONCE("Unknown state in AdaptiveControl");
                Vector6d Qpr = Vector6d::Zero();
                Vector6d Qppr = Vector6d::Zero();
                Vector6d Sq = Vector6d::Zero();
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); 
            
                tau = -Kd_ * Sq + Yr * Theta_;
            
            }
            else if(controller_state_ == ControllerState::LIFT_CRATE)
            {   
                
                if (compute_moving_trajectory_2_){

                    t_initial_ = grasp_crate_time_;
                    t_final_ = lift_crate_time_ ;
                    
                    /* Goal is right in front of the robot */
                    double x , y;
                    x =  0.08;
                    y = -0.08;
                    double z = -0.2;
        
                    Vector4d point = Vector4d::Ones();
                    point << x, y, z, 1.0;
        
                    Vector4d point_in_world_frame = Vector4d::Zero();
        
                    point_in_world_frame = Tee_W * point;
                    final_cartesian_pos_ << point_in_world_frame[0],point_in_world_frame[1],point_in_world_frame[2];
                    initial_cartesian_pos_ = EE_current_position;
                    
                    // Save the Orientation when Joint State controller ends, as we always want to maintain this orientation
                    //desired_rotation_wrt_base_ = Ree_W;
                    //desired_quaternion_wrt_base_ = Quaterniond(Ree_W);

                    ax = cartesian_trajectory(initial_cartesian_pos_[0], final_cartesian_pos_[0] , initial_velocity_, initial_acceleration_, final_velocity_, final_acceleration_, t_initial_, t_final_);
                    ay = cartesian_trajectory(initial_cartesian_pos_[1], final_cartesian_pos_[1] , initial_velocity_, initial_acceleration_, final_velocity_, final_acceleration_, t_initial_, t_final_);
                    az = cartesian_trajectory(initial_cartesian_pos_[2], final_cartesian_pos_[2] , initial_velocity_, initial_acceleration_, final_velocity_, final_acceleration_, t_initial_, t_final_);
                    compute_moving_trajectory_2_ = false;
                    
                    ROS_INFO_ONCE("AdaptiveController, Robot is lifting the crate now");

                  }
        

                geometry_msgs::TransformStamped Cartesian_goal;
                Cartesian_goal.header.stamp = ros::Time::now();
                Cartesian_goal.header.frame_id = "world";
                Cartesian_goal.child_frame_id = "Lift Crate";
                Cartesian_goal.transform.translation.x = final_cartesian_pos_(0);
                Cartesian_goal.transform.translation.y = final_cartesian_pos_(1);
                Cartesian_goal.transform.translation.z = final_cartesian_pos_(2);
                Cartesian_goal.transform.rotation.x = desired_quaternion_wrt_base_.x();
                Cartesian_goal.transform.rotation.y = desired_quaternion_wrt_base_.y();
                Cartesian_goal.transform.rotation.z = desired_quaternion_wrt_base_.z();
                Cartesian_goal.transform.rotation.w = desired_quaternion_wrt_base_.w();
                broadcaster_.sendTransform(Cartesian_goal);
                
                Matrix<double, 3, 6> T_coefficients;

                T_coefficients << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5),
                                0, 1, 2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
                                0, 0, 2, 6 * t, 12 * pow(t,2), 20 * pow(t,3);


                Vector3d X_desired = Vector3d::Zero();
                Vector3d X_desired_velocity = Vector3d::Zero();
                Vector3d X_desired_acceleration = Vector3d::Zero();


                X_desired << T_coefficients.row(0) * ax, T_coefficients.row(0) * ay, T_coefficients.row(0) * az;
                X_desired_velocity << T_coefficients.row(1) * ax, T_coefficients.row(1) * ay, T_coefficients.row(1) * az;
                X_desired_acceleration << T_coefficients.row(2) * ax, T_coefficients.row(2) * ay, T_coefficients.row(2) * az;

            
                Matrix6d Jacobian = get_ee_jacobian(q);
                Matrix6d Jacobian_dot = get_ee_jacobian_dot(q, qp);
                Matrix6d Jacobian_inv = Jacobian.transpose() *(Jacobian* Jacobian.transpose() + lambda_ * Matrix<double,6,6>::Identity()).completeOrthogonalDecomposition().pseudoInverse();
            
                // Error Linear and Angular
                Vector3d Delta_X = EE_current_position - X_desired;
                Quaterniond q_current(Ree_W);
                Quaterniond q_err = desired_quaternion_wrt_base_ * q_current.inverse();
                q_err.normalize();
                Eigen::AngleAxisd angle_axis(q_err);
                // Rotation magnitude
                double theta = angle_axis.angle();  
                // Rotation axis
                Eigen::Vector3d axis = angle_axis.axis();  
                // If theta is too small
                if (theta < 1e-6) {
                    theta = 0.0;
                }
                // Compute angular error e_ω = θ * v
                Eigen::Vector3d angular_error = theta * axis;
                                
                Vector3d Delta_X_velocity = linear_ee_velocity - X_desired_velocity; 
                integral_qi_cart_ = integral_qi_cart_ + (Delta_X  * time_difference);

                // Reference Velocity
                Vector3d X_reference_linear_velocity = X_desired_velocity - Kp2_ * Delta_X - Ki2_ * integral_qi_cart_;
                Vector6d X_reference_velocity = Vector6d::Zero();
                X_reference_velocity.head(3) = X_reference_linear_velocity;
                X_reference_velocity.tail(3) = angular_error;
                
                Vector6d new_acc = Vector6d::Zero();
                new_acc << X_desired_acceleration[0], X_desired_acceleration[1], X_desired_acceleration[2], 0.0, 0.0, 0.0;
                Qpr = Jacobian_inv * X_reference_velocity;
                Qppr = Jacobian_inv * (new_acc - (Jacobian_dot * qp));

                // Calculate Sq
                Sq = qp - Qpr;
                double w  = sqrt((Jacobian * Jacobian.transpose()).determinant());
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); 

                tau = -Kd_ * Sq + Yr * Theta_;


            }
            else if(controller_state_ == ControllerState::HOLD_POSITION)
            {
                ROS_INFO_ONCE("AdaptiveController, Robot is holding position");

                Vector6d Qpr = Vector6d::Zero();
                Vector6d Qppr = Vector6d::Zero();
                Vector6d Sq = Vector6d::Zero();
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); 
                tau = -Kd_ * Sq + Yr * Theta_;

            }
            else if(controller_state_ == ControllerState::ADMITTANCE_CONTROLLER)
            {   
                
                // Zero values if the force is below a threshold
                Vector6d Qpr = Vector6d::Zero();
                Vector6d Qppr = Vector6d::Zero();
                Vector6d Sq = Vector6d::Zero();

                // Rotation to rotate the FT reading from sensor frame to ee frame

                if(current_force > force_limit_to_activate_admittance_control)
                {

                Vector6d X_desired = Vector6d::Zero();
                Vector6d X_desired_velocity = Vector6d::Zero();
                Vector6d X_desired_acceleration = Vector6d::Zero();
                
                X_desired_acceleration = Lambda_adm_.inverse() * (wrench_in_world - Kdamp_ * end_effector_velocity);
                X_desired_acceleration.tail(3) = Vector3d::Zero();
                X_desired_acceleration = X_desired_acceleration.cwiseMax(-1.0).cwiseMin(1.0);
                
                std::cout << "desired acceleration : " << X_desired_acceleration.transpose() << std::endl;
                std::cout << "force in world : " << wrench_in_world.transpose() << std::endl;
                std::cout << "" << std::endl;
                    
                
                Quaterniond q_current(Ree_W);
                auto cartesian_desired = cartesian_integration_for_admittance(ee_current_pose, end_effector_velocity, X_desired_acceleration, q_current, time_difference);
                X_desired_velocity = cartesian_desired[1];
                X_desired = cartesian_desired[0]; // [pos, rpy]

                Matrix6d Jacobian = get_ee_jacobian(q);
                Matrix6d Jacobian_dot = get_ee_jacobian_dot(q, qp);
                Matrix6d Jacobian_inv = Jacobian.transpose() *(Jacobian* Jacobian.transpose() + lambda_ * Matrix<double,6,6>::Identity()).completeOrthogonalDecomposition().pseudoInverse();                


                Vector6d theta_d_ddot = Jacobian_inv * (X_desired_acceleration - (Jacobian_dot * qp));
                Vector6d theta_d_dot = qp + theta_d_ddot * time_difference;
                Vector6d theta_d = q + theta_d_dot * time_difference;
                // Error 
                Delta_Q_ = q - theta_d;
                Delta_Qp_ = qp - theta_d_dot;
                // JOINT SPACE PID
                integral_qi_joint_ = integral_qi_joint_ + Delta_Q_ * time_difference;
                Qpr = theta_d_dot - Kp_ * Delta_Q_ - Ki_ * integral_qi_joint_;
                Qppr = theta_d_dot - Kp_ * (qp - Qpr);
                Sq = qp - Qpr;
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); //* time_difference;
                tau = -Kd_ * Sq  + Yr * Theta_;


                // /Error Linear and Angular
                Eigen::Matrix3d desiredrotationMatrix;
                desiredrotationMatrix = Eigen::AngleAxisd(X_desired[3], Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(X_desired[4], Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(X_desired[5], Eigen::Vector3d::UnitZ());
                desired_quaternion_wrt_base_ = Eigen::Quaterniond(desiredrotationMatrix);

                Quaterniond q_err = desired_quaternion_wrt_base_ * q_current.inverse();
                q_err.normalize();
                Eigen::AngleAxisd angle_axis(q_err);
                // Rotation magnitude
                double theta = angle_axis.angle();  
                // Rotation axis
                Eigen::Vector3d axis = angle_axis.axis();  
                // If theta is too small
                if (theta < 1e-3) {
                    theta = 0.0;
                }
                // Compute angular error e_ω = θ * v
                Eigen::Vector3d omega = theta * axis; 
                //omega[0] = 0.0;
                //omega[1] = 0.0;  

                Vector3d Delta_X = EE_current_position - X_desired.head(3);
                integral_qi_cart_ = integral_qi_cart_ + (Delta_X  * time_difference);
                // Reference Velocity
                Vector6d X_reference_velocity = Vector6d::Zero();
                X_reference_velocity.head(3) = X_desired_velocity.head(3) - Kp2_ * Delta_X - Ki2_ * integral_qi_cart_;
                X_reference_velocity.tail(3) = omega;
                ROS_WARN_STREAM("integral value: " << integral_qi_cart_);
                
                Vector6d Reference_Acceleration = Vector6d::Zero();
                /* Important Note check this below, if it is correct */
                Reference_Acceleration << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                Qpr = Jacobian_inv * X_reference_velocity;
                Qppr = Jacobian_inv * (Reference_Acceleration - (Jacobian_dot * qp));

                // Calculate Sq
                //Qpr[4] = 0;
                Sq = qp - Qpr;


                Quaternionf AdmittanceDesired_q;
                AdmittanceDesired_q = AngleAxisf(X_desired[3], Vector3f::UnitX())
                                    * AngleAxisf(X_desired[4], Vector3f::UnitY())
                                    * AngleAxisf(X_desired[5], Vector3f::UnitZ());
                geometry_msgs::TransformStamped AdmittanceDesired;
                EE.header.stamp = ros::Time::now();
                EE.header.frame_id = "world";
                EE.child_frame_id = "Deisred Admittance";
                EE.transform.translation.x = X_desired[0];
                EE.transform.translation.y = X_desired[1];
                EE.transform.translation.z = X_desired[2];
                EE.transform.rotation.x = AdmittanceDesired_q.x();
                EE.transform.rotation.y = AdmittanceDesired_q.y();
                EE.transform.rotation.z = AdmittanceDesired_q.z();
                EE.transform.rotation.w = AdmittanceDesired_q.w();
                broadcaster_.sendTransform(EE);


                geometry_msgs::Vector3Stamped msg1;
                msg1.header.stamp = ros::Time::now();
                msg1.vector.x = X_desired[0];
                msg1.vector.y = X_desired[1];
                msg1.vector.z = X_desired[2];
                x_des_pub_.publish(msg1);

                geometry_msgs::Vector3Stamped msg2;
                msg2.header.stamp = ros::Time::now();
                msg2.vector.x = X_desired_velocity[0];
                msg2.vector.y = X_desired_velocity[1];
                msg2.vector.z = X_desired_velocity[2];
                x_dot_des_pub_.publish(msg2);

                geometry_msgs::Vector3Stamped msg3;
                msg3.header.stamp = ros::Time::now();
                msg3.vector.x = X_reference_velocity.head(3)[0];
                msg3.vector.y = X_reference_velocity.head(3)[1];
                msg3.vector.z = X_reference_velocity.head(3)[2];
                x_dot_ref_pub_.publish(msg3);

                geometry_msgs::Vector3Stamped msg4;
                msg4.header.stamp = ros::Time::now();
                msg4.vector.x = EE_current_position[0];
                msg4.vector.y = EE_current_position[1];
                msg4.vector.z = EE_current_position[2];
                x_pub_.publish(msg4);

                geometry_msgs::Vector3Stamped msg5;
                msg5.header.stamp = ros::Time::now();
                msg5.vector.x = linear_ee_velocity[0];
                msg5.vector.y = linear_ee_velocity[1];
                msg5.vector.z = linear_ee_velocity[2];
                x_dot_pub_.publish(msg5);
                }
                
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq);
                tau = -Kd_ * Sq + Yr * Theta_;
                
            }
            else
            {

                ROS_INFO_ONCE("Unknown state in AdaptiveControl");
                Vector6d Qpr = Vector6d::Zero();
                Vector6d Qppr = Vector6d::Zero();
                Vector6d Sq = Vector6d::Zero();
                Eigen::MatrixXd Yr = get_regressor(q, qp, Qpr, Qppr);
                Theta_ = Theta_ - learning_rate_ * (Yr.transpose() * Sq); 
                tau = -Kd_ * Sq + Yr * Theta_;

            }

            previous_time = t;

            return tau;

        }


        bool tum_ics_ur10_AdaptiveController::stop()
        {

            return true;

        }



  } // namespace RobotControllers


} // namespace tum_ics_ur_robot_lli




