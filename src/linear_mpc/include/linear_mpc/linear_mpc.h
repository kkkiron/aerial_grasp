#ifndef LINEAR_MPC_NODE_H
#define LINEAR_MPC_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <std_msgs/Time.h>
#include <linear_mpc/SetPose.h>
#include <linear_mpc/SetTrajectory.h>
#include <std_msgs/UInt16.h>

#include <dynamic_reconfigure/server.h>
#include <linear_mpc/LinearMPCConfig.h>

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>

#include <solver.h>
#include <opti_msgs/Odom.h>
#include <arm_test/position.h>
#include <arm_test/track.h>

class LinearModelPredictiveController
{
    public:
    LinearModelPredictiveController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~LinearModelPredictiveController();

    private:

    // constants
    static constexpr int kStateSize = 6;
    static constexpr int kInputSize = 3;
    static constexpr int kMeasurementSize = 6;
    static constexpr int kPredictionHorizonSteps = 20;
    static constexpr double kGravity = 9.8066;

    // ros node handles
    ros::NodeHandle nh_, private_nh_;

    // publishers
    double publish_time_;
    ros::Timer publish_timer_;
    ros::Publisher cmd_roll_pitch_yawrate_thrust_pub_;
    void cmdPublisher(const ros::TimerEvent& time_event);

    // subscribers
    ros::Subscriber pose_sub_;
    void poseCallback(const opti_msgs::Odom::ConstPtr& pose_msg);
    ros::Subscriber positon_sub_;
    void positionCallback(const arm_test::position::ConstPtr& position_msg);
    ros::Subscriber track_sub_;
    void trackCallback(const arm_test::track::ConstPtr& track_msg);
    // ros::Subscriber land_sub_;
    // void landCallback(const Armtest::gripper::ConstPtr& land_msg);
    // service servers
    // ros::ServiceClient set_pose_client_;


    ros::ServiceServer set_pose_server_;
    bool setPose(
        linear_mpc::SetPose::Request& req_set_pose, 
        linear_mpc::SetPose::Response& res_set_pose);

    ros::ServiceServer set_trajectory_server_;
    bool setTrajectory(
        linear_mpc::SetTrajectory::Request& req_set_trajectory,
        linear_mpc::SetTrajectory::Response& res_set_trajectory);


    //initialize parameters
    void initializeParameters();
    bool initialized_parameters_;

    // sampling time parameters
    double sampling_time_;
    double prediction_sampling_time_;

    // system model parameters
    // Model: A, B
    // x(k+1) = A*x(k) + B*u(k)
    Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
    Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
    Eigen::Vector3d drag_coefficients_;
    double mass_;

    // dynamic configure server
    dynamic_reconfigure::Server<linear_mpc::LinearMPCConfig> dyn_config_server_;
    void DynConfigCallback(linear_mpc::LinearMPCConfig& config, uint32_t level);

    // controller parameters
    // state penalty
    Eigen::Vector3d q_position_;
    Eigen::Vector3d q_velocity_;

    Eigen::Vector3d q_final_position_;
    Eigen::Vector3d q_final_velocity_;

    // control penalty
    Eigen::Vector3d r_command_;
    Eigen::Vector3d r_command_delta;

    // control input limits
    double roll_limit_;
    double pitch_limit_;
    double yaw_rate_limit_;
    double thrust_min_;
    double thrust_max_;
    double slew_;  //du limit
    double u_factor = 1.0;  

    // debug info
    bool verbose_;
    double solve_time_average_;

    // backup LQR
    Eigen::MatrixXd LQR_K_;

    // most recent pose information
    geometry_msgs::Point pose_;

    // last pose information
    Eigen::Vector3d last_position_;

    // desired goal information
    bool received_desired_pose_;
    Eigen::Matrix<double, kStateSize, 1> x_d_;
    Eigen::Matrix<double, kStateSize, 1> x_d_actually_;
    double z_bias_;
    double x_rel_ = 0.0, y_rel_ = 0.0, z_rel_ = 0.0;

    // controller output
    Eigen::Vector3d linearized_command_roll_pitch_thrust_;
    mav_msgs::RollPitchYawrateThrust cmd_roll_pitch_yawrate_thrust_;
    
    void applyParameters();

    void calculateRollPitchYawrateThrustCommand();

    // bool is_land = false;

     

    int id_ = 1;
    int target_rigidbody_id_ = 0;
    bool is_track_ = false;
};

#endif