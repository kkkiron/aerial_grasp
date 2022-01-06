#include <linear_mpc/linear_mpc.h>

constexpr int LinearModelPredictiveController::kStateSize;
constexpr int LinearModelPredictiveController::kInputSize;
constexpr int LinearModelPredictiveController::kMeasurementSize;
constexpr double LinearModelPredictiveController::kGravity;
constexpr int LinearModelPredictiveController::kPredictionHorizonSteps;

void QuaternionToEulerAngles(
    const Eigen::Quaterniond& quaternion, Eigen::Vector3d* euler_angles)
{
    double w = quaternion.w();
    double x = quaternion.x();
    double y = quaternion.y();
    double z = quaternion.z();

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    euler_angles->x() = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        euler_angles->y() = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler_angles->y() = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    euler_angles->z() = atan2(siny_cosp, cosy_cosp);
}

LinearModelPredictiveController::LinearModelPredictiveController(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , dyn_config_server_(private_nh)
    , initialized_parameters_(false)
    , verbose_(true)
    , solve_time_average_(0.0)
    , received_desired_pose_(false)
    , last_position_(0, 0, 0)
{
    dynamic_reconfigure::Server<linear_mpc::LinearMPCConfig>::CallbackType f;
    f = boost::bind(&LinearModelPredictiveController::DynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);

    pose_sub_ = nh_.subscribe(
        "/agent/opti_odom", 1, &LinearModelPredictiveController::poseCallback, this);

    positon_sub_ = nh_.subscribe(
        "/position", 1, &LinearModelPredictiveController::positionCallback, this);

    track_sub_ = nh_.subscribe(
        "/track", 1, &LinearModelPredictiveController::trackCallback, this);

    cmd_roll_pitch_yawrate_thrust_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
        "/cmd_attitude_thrust", 1);

    set_pose_server_ = nh_.advertiseService(
        "/set_pose", &LinearModelPredictiveController::setPose, this);

    // set_pose_client_ = nh_.serviceClient<linear_mpc::SetPose>("/set_pose");
    // linear_mpc::SetPose srv;
    // srv.request

    initializeParameters();

    publish_timer_ = nh_.createTimer(ros::Duration(publish_time_), &LinearModelPredictiveController::cmdPublisher, this);
}

LinearModelPredictiveController::~LinearModelPredictiveController()
{

}

void LinearModelPredictiveController::initializeParameters()
{
    std::vector<double> drag_coefficients;
    
    //Get parameters from RosParam server
    private_nh_.param<bool>("verbose", verbose_, false);

    if (!private_nh_.getParam("mass", mass_)) {
        ROS_ERROR("mass in MPC is not loaded from ros parameter server");
        abort();
    }

    if (!private_nh_.getParam("drag_coefficients", drag_coefficients)) {
        ROS_ERROR("drag_coefficients in MPC is not loaded from ros parameter server");
        abort();
    }
    drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);

    //if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    //    ROS_ERROR("prediction_sampling_time in MPC is not loaded from ros parameter server");
    //    abort();
    //}

    if (!private_nh_.getParam("sampling_time", sampling_time_)) {
        ROS_ERROR("sampling_time in MPC is not loaded from ros parameter server");
        abort();
    }

    if (!private_nh_.getParam("publish_time", publish_time_)) {
        ROS_ERROR("sampling_time in MPC is not loaded from ros parameter server");
        abort();
    }

    model_A_ = Eigen ::MatrixXd::Zero(kStateSize, kStateSize);
    model_B_ = Eigen::MatrixXd::Zero(kStateSize, kInputSize);

    model_A_(0, 0) = 1;
    model_A_(0, 3) = 0.05;
    model_A_(1, 1) = 1;
    model_A_(1, 4) = 0.05;
    model_A_(2, 2) = 1;
    model_A_(2, 5) = 0.05;
    model_A_(3, 3) = 1;
    model_A_(4, 4) = 1;
    model_A_(5, 5) = 1;

    model_B_(0, 1) = 0.01225;
    model_B_(1, 0) = -0.01225;
    model_B_(2, 2) = 0.00125;
    model_B_(3, 1) = 0.49;
    model_B_(4, 0) = -0.49;
    model_B_(5, 2) = 0.05;

    if (verbose_) {
        ROS_INFO_STREAM("A: \n" << model_A_);
        ROS_INFO_STREAM("B: \n" << model_B_);
        ROS_INFO_STREAM("u_factor=" << u_factor);
        ROS_INFO_STREAM("roll pitch limit=" << params.u_max[0]);
        // ROS_INFO_STREAM("R: " << params.R[0] << ' ' << params.R[4] << ' ' << params.R[8]);
        // ROS_INFO_STREAM("R_delta: \n" << params.R_delta[0] << ' ' << params.R_delta[4] << ' ' << params.R_delta[8]);
    }

    //Solver initialization
    set_defaults();
    setup_indexing();

    //Solver settings
    settings.verbose = 0;

    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A_;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B_;

    initialized_parameters_ = true;
    ROS_INFO("Linear MPC: initialized correctly");
}

void LinearModelPredictiveController::DynConfigCallback(
    linear_mpc::LinearMPCConfig &config, uint32_t level)
{
    q_position_ << 100, 100, 59;
    q_velocity_ << config.q_vx, config.q_vy, config.q_vz;

    q_final_position_ << config.q_final_x, config.q_final_y, config.q_final_y;
    q_final_velocity_ << config.q_final_vx, config.q_final_vy, config.q_final_vz;

    // r_command_ << config.r_roll, config.r_pitch, config.r_thrust;
    r_command_ << 200, 200, 1;
    r_command_delta << 10000000000, 10000000000, 1;

    roll_limit_ = 0.2;
    pitch_limit_ = 0.2;
    thrust_max_ = config.thrust_max - kGravity;
    thrust_min_ = config.thrust_min - kGravity;
    

    applyParameters();
}

void LinearModelPredictiveController::applyParameters()
{
    ROS_INFO("into applyParamaters");
    Eigen::Matrix<double, kStateSize, kStateSize> Q;
    Q.setZero();
    Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
    Q_final.setZero();
    Eigen::Matrix<double, kInputSize, kInputSize> R;
    R.setZero();
    Eigen::Matrix<double, kInputSize, kInputSize> R_delta;
    R_delta.setZero();

    Q.block(0, 0, 3, 3) = q_position_.asDiagonal();
    Q.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
    R = r_command_.asDiagonal();
    R_delta = r_command_delta.asDiagonal();

    Q_final.block(0, 0, 3, 3) = q_final_position_.asDiagonal();
    Q_final.block(3, 3, 3, 3) = q_final_velocity_.asDiagonal();

    Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
    LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);

    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q), kStateSize, kStateSize) = Q;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_final), kStateSize, kStateSize) = Q_final;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R), kInputSize, kInputSize) = R;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_delta), kInputSize, kInputSize) = R_delta;

    params.u_max[0] = roll_limit_;
    params.u_max[1] = pitch_limit_;
    params.u_max[2] = thrust_max_;
    
    params.u_min[0] = -roll_limit_;
    params.u_min[1] = -pitch_limit_;
    params.u_min[2] = thrust_min_;
    
    ROS_INFO_STREAM("verbose: " << verbose_);
    ROS_INFO("Linear MPC: Tuning parameters updated...");
    if (verbose_) {
        ROS_INFO_STREAM("diag(Q) = \n" << Q);
        ROS_INFO_STREAM("diag(R) = \n" << R);
        ROS_INFO_STREAM("Q_final = \n" << Q_final);
        ROS_INFO_STREAM("R_delta = \n" << R_delta);        
    }
}

void LinearModelPredictiveController::positionCallback(
    const arm_test::position::ConstPtr& position_msg)
{   
    x_rel_ = position_msg->x_relative;
    y_rel_ = position_msg->y_relative;
    z_rel_ = position_msg->z_relative;
    x_d_actually_(0) =  x_d_(0) + x_rel_;
    x_d_actually_(1) =  x_d_(1) + y_rel_;
    x_d_actually_(2) =  x_d_(2) + z_rel_;
    Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_d)) = x_d_actually_;
    ROS_INFO_STREAM("x_d_ = \n" << x_d_);
    ROS_INFO_STREAM("x_d_actually_ = \n" << x_d_actually_);
}

void LinearModelPredictiveController::trackCallback(
    const arm_test::track::ConstPtr& track_msg)
{
    is_track_ = track_msg->is_track;
    if(is_track_) {
        ROS_INFO_STREAM("track to target");
    }
    else{
        ROS_INFO_STREAM("untrack to target");
    }
}

void LinearModelPredictiveController::poseCallback(
    const opti_msgs::Odom::ConstPtr& pose_msg)
{
    if(!received_desired_pose_)
    {
        return;
    }

    if(pose_msg->rigidBodyID == this->id_)
    {
        pose_.x = pose_msg->position.x;
        pose_.y = pose_msg->position.y;
        pose_.z = pose_msg->position.z; 
    }
    
    if(pose_msg->rigidBodyID == this->target_rigidbody_id_ && is_track_)
    {
        x_d_(0) = pose_msg->position.x;
        x_d_(1) = pose_msg->position.y;
        x_d_(2) = pose_msg->position.z + z_bias_;
        x_d_actually_(0) =  x_d_(0) + x_rel_;
        x_d_actually_(1) =  x_d_(1) + y_rel_;
        x_d_actually_(2) =  x_d_(2) + z_rel_;
        Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_d)) = x_d_actually_;
    }
    calculateRollPitchYawrateThrustCommand();
}

void LinearModelPredictiveController::cmdPublisher(
    const ros::TimerEvent& time_event)
{
    cmd_roll_pitch_yawrate_thrust_pub_.publish(cmd_roll_pitch_yawrate_thrust_);
}

void LinearModelPredictiveController::calculateRollPitchYawrateThrustCommand()
{
    assert(initialized_parameters_ == true);
    ros::WallTime starting_time = ros::WallTime::now();

    Eigen::Matrix<double, kStateSize, 1> x_0;
    Eigen::Vector3d velocity_world;


    double xdot = (pose_.x - last_position_.x()) / sampling_time_;  //dji模拟器坐标x,y和geometry_msgs::PointStamped的x,y是反的
    double ydot = (pose_.y - last_position_.y()) / sampling_time_;
    double zdot = (pose_.z - last_position_.z()) / sampling_time_;

    x_0 << pose_.x, pose_.y, pose_.z, xdot, ydot, zdot;

    Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_0)) = x_0;

    tic();
    int solver_status = solve();
    solve_time_average_ += tocq();

    linearized_command_roll_pitch_thrust_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];

    if (solver_status < 0) {  
        ROS_WARN("Linear MPC: Solver faild, use LQR backup");
        linearized_command_roll_pitch_thrust_ = LQR_K_ * (x_d_ - x_0);
        linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMax(
            Eigen::Vector3d(-roll_limit_, -pitch_limit_, thrust_min_));
        linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMin(
            Eigen::Vector3d(roll_limit_, pitch_limit_, thrust_max_));
    }

    cmd_roll_pitch_yawrate_thrust_.roll = -linearized_command_roll_pitch_thrust_(0)/u_factor;
    cmd_roll_pitch_yawrate_thrust_.pitch = linearized_command_roll_pitch_thrust_(1)/u_factor;

    cmd_roll_pitch_yawrate_thrust_.yaw_rate = 0;
    cmd_roll_pitch_yawrate_thrust_.thrust.x = 0;
    cmd_roll_pitch_yawrate_thrust_.thrust.y = 0;
    //cmd_roll_pitch_yawrate_thrust_.thrust.z *= mass_;
    cmd_roll_pitch_yawrate_thrust_.thrust.z = (linearized_command_roll_pitch_thrust_(2) + kGravity) * mass_;

    last_position_ << pose_.x, pose_.y, pose_.z;

    double diff_time = (ros::WallTime::now() - starting_time).toSec();

    // if (verbose_) {
    //     static int counter = 0;
    //     if (counter > 100) {
    //         ROS_INFO_STREAM("average solve time: " << 1000.0 * solve_time_average_ / counter << " ms");
    //         solve_time_average_ = 0.0;
            
    //         ROS_INFO_STREAM("Controller loop time : " << diff_time * 1000.0 << " ms");
            
    //         ROS_INFO_STREAM("roll ref: " << cmd_roll_pitch_yawrate_thrust_.roll
    //             << "\t" << "pitch ref : \t" << cmd_roll_pitch_yawrate_thrust_.pitch
    //             << "\t" << "yawrate ref : \t" << cmd_roll_pitch_yawrate_thrust_.yaw_rate
    //             << "\t" << "thrust ref : \t" << cmd_roll_pitch_yawrate_thrust_.thrust.z);
    //         counter = 0;
    //     }
    //     counter++;
    // }
}

bool LinearModelPredictiveController::setPose(
    linear_mpc::SetPose::Request& req_set_pose, 
    linear_mpc::SetPose::Response& res_set_pose)
{
    ROS_INFO_STREAM("Desired pose requested");

 
    double x_temp = pose_.x, y_temp = pose_.y, z_temp = pose_.z;
    double x_add = (req_set_pose.pose.position.x - pose_.x)/req_set_pose.point_num,
           y_add = (req_set_pose.pose.position.y - pose_.y)/req_set_pose.point_num,
           z_add = (req_set_pose.pose.position.z - pose_.z)/req_set_pose.point_num;
    // x_d_ << req_set_pose.pose.position.x, req_set_pose.pose.position.y, 
    //     req_set_pose.pose.position.z, 0, 0, 0;
    double sleep_time = req_set_pose.time/req_set_pose.point_num;

    if (verbose_) {
        ROS_INFO_STREAM("desired pose: \n" << req_set_pose.pose);
    }
    received_desired_pose_ = true;

    for(int i=0; i<req_set_pose.point_num; i++){
        x_temp += x_add;
        y_temp += y_add;
        z_temp += z_add;
        x_d_ << x_temp, y_temp, z_temp, 0, 0, 0;
        z_bias_ = z_temp;
        Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_d)) = x_d_;
        if(verbose_) {
            ROS_INFO_STREAM("now desired pose: \n" << x_d_);
            ROS_INFO_STREAM("z_bias_: \n" << z_bias_);
        }
        ros::Duration(sleep_time).sleep();
    }

}

bool LinearModelPredictiveController::setTrajectory(
        linear_mpc::SetTrajectory::Request& req_set_trajectory,
        linear_mpc::SetTrajectory::Response& res_set_trajectory)
{

}

// void LinearModelPredictiveController::landCallback(const Armtest::gripper::ConstPtr& land_msg)
// {
//     if(land_msg->GripSta == 0x03)
//     {
//         this->x_d_ << 0, 0, 0.2, 0, 0, 0;
//         Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_d)) = x_d_;
//         ros::Duration(4).sleep();
//         this->x_d_ << 0, 0, 0, 0, 0, 0;
//         Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_d)) = x_d_;
//         ROS_INFO("return to base point");
//         ros::Duration(1).sleep();
//         is_land = true;
//         cmd_roll_pitch_yawrate_thrust_.roll = 0;
//         cmd_roll_pitch_yawrate_thrust_.pitch= 0;
//         cmd_roll_pitch_yawrate_thrust_.thrust.z = 0;
//         cmd_roll_pitch_yawrate_thrust_pub_.publish(cmd_roll_pitch_yawrate_thrust_);
//     }

// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LinearModelPredictiveControllerNode");
    ros::NodeHandle nh, private_nh("~");

    LinearModelPredictiveController mpc(nh, private_nh);

    ros::spin();

    return 0;
}