
#include <kimm_phri_panda_husky/phri_real.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>


namespace kimm_husky_controllers
{

bool BasicHuskyFrankaController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{

  node_handle.getParam("/robot_group", group_name_);
  husky_ctrl_pub_.init(node_handle, "/" + group_name_ + "/husky/cmd_vel", 4);
  
  ctrl_type_sub_ = node_handle.subscribe("/" + group_name_ + "/real_robot/ctrl_type", 1, &BasicHuskyFrankaController::ctrltypeCallback, this);
  mob_subs_ = node_handle.subscribe("/" + group_name_ + "/real_robot/mob_type", 1, &BasicHuskyFrankaController::mobtypeCallback, this);
  odom_subs_ = node_handle.subscribe( "/" + group_name_ + "/husky/odometry/filtered", 1, &BasicHuskyFrankaController::odomCallback, this);
  husky_state_msg_.position.resize(4);
  husky_state_msg_.velocity.resize(4);
  husky_state_subs_ = node_handle.subscribe( "/" + group_name_ + "/husky/joint_states", 1, &BasicHuskyFrankaController::huskystateCallback, this);
  
  torque_state_pub_ = node_handle.advertise<mujoco_ros_msgs::JointSet>("/" + group_name_ + "/real_robot/joint_set", 5);
  joint_state_pub_ = node_handle.advertise<sensor_msgs::JointState>("/" + group_name_ + "/real_robot/joint_states", 5);
  time_pub_ = node_handle.advertise<std_msgs::Float32>("/" + group_name_ + "/time", 1);
  husky_odom_pub_ = node_handle.advertise<visualization_msgs::Marker>("husky_odom", 1);

  ee_state_pub_ = node_handle.advertise<geometry_msgs::Transform>("/" + group_name_ + "/real_robot/ee_state", 5);
  ee_state_msg_ = geometry_msgs::Transform();
  base_state_pub_ = node_handle.advertise<sensor_msgs::JointState>("/" + group_name_ + "/real_robot/base_state", 5);
  base_state_msg_.position.resize(3);
  base_state_msg_.velocity.resize(3);

  isgrasp_ = false;
  
  gripper_ac_.waitForServer();
  gripper_grasp_ac_.waitForServer();

  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }  

  //keyboard event
  mode_change_thread_ = std::thread(&BasicHuskyFrankaController::modeChangeReaderProc, this);

  ctrl_ = new RobotController::HuskyFrankaWrapper(group_name_, false, node_handle);
  ctrl_->initialize();
  
  
  return true;
}

void BasicHuskyFrankaController::starting(const ros::Time& time) {
  dq_filtered_.setZero();

  odom_lpf_prev_(0) = odom_msg_.pose.pose.position.x;
  odom_lpf_prev_(1) = odom_msg_.pose.pose.position.y;
  odom_lpf_prev_(2) = odom_msg_.pose.pose.orientation.z;

  odom_dot_lpf_prev_(0) = odom_msg_.twist.twist.linear.x;
  odom_dot_lpf_prev_(1) = odom_msg_.twist.twist.linear.y;
  odom_dot_lpf_prev_(2) = odom_msg_.twist.twist.angular.z;  
  
  time_ = 0.;
  mob_type_ = 0;

  robot_command_msg_.torque.resize(7);
  robot_state_msg_.position.resize(11);
  robot_state_msg_.velocity.resize(11);   

  husky_qvel_prev_.setZero(2);
  f_filtered_.setZero();
  
  br_ = new tf::TransformBroadcaster();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "husky_odom";
  marker.header.stamp = ros::Time();
  marker.ns = "husky_odom";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  
  husky_odom_pub_.publish( marker );
  
}


void BasicHuskyFrankaController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 49> massmatrix_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  robot_tau_ = tau_J_d;

  Eigen::Map<Vector7d> gravity(gravity_array.data());
  robot_g_ = gravity;
  Eigen::Map<Matrix7d> mass_matrix(massmatrix_array.data());
  robot_mass_ = mass_matrix;
  Eigen::Map<Vector7d> non_linear(coriolis_array.data());
  robot_nle_ = non_linear;
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  robot_J_ = jacobian;
  Eigen::Map<Vector7d> franka_q(robot_state.q.data());
  Eigen::Map<Vector7d> franka_dq(robot_state.dq.data());
  franka_q_ = franka_q;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_franka(robot_state.O_F_ext_hat_K.data());
  f_ = force_franka;

  // Filtering
  double cutoff = 20.0; // Hz //20
  double RC = 1.0 / (cutoff * 2.0 * M_PI);
  double dt = 0.001;
  double alpha = dt / (RC + dt);
  
  dq_filtered_ = alpha * franka_dq + (1 - alpha) * dq_filtered_;
  f_filtered_ = alpha * f_ + (1 - alpha) * f_filtered_;

  odom_lpf_(0) = this->lowpassFilter(0.001, odom_msg_.pose.pose.position.x, odom_lpf_prev_(0), 100);
  odom_lpf_(1) = this->lowpassFilter(0.001, odom_msg_.pose.pose.position.y, odom_lpf_prev_(1), 100);
  odom_lpf_(2) = this->lowpassFilter(0.001, odom_msg_.pose.pose.orientation.z, odom_lpf_prev_(2), 100);
  
  // if (group_name_ == "ns0")
  //   odom_lpf_(1) += 2.0;

  odom_dot_lpf_(0) = this->lowpassFilter(0.001, odom_msg_.twist.twist.linear.x, odom_dot_lpf_prev_(0), 100);
  odom_dot_lpf_(1) = this->lowpassFilter(0.001, odom_msg_.twist.twist.linear.y, odom_dot_lpf_prev_(1), 100);
  odom_dot_lpf_(2) = this->lowpassFilter(0.001, odom_msg_.twist.twist.angular.z, odom_dot_lpf_prev_(2), 100);

  odom_lpf_prev_ = odom_lpf_;
  odom_dot_lpf_prev_ = odom_dot_lpf_;
  
  tf::StampedTransform transform2;
  tf::Vector3 origin;
  tf::Quaternion q;
  
  try{
      listener_.lookupTransform("/" + group_name_ + "_map", "/" + group_name_ + "_carto_base_link" , ros::Time(0), transform2);
      origin = transform2.getOrigin();
      q = transform2.getRotation();
      pinocchio::SE3 odom;
      odom.translation()(0) = origin.getX();
      odom.translation()(1) = origin.getY();
      odom.translation()(2) = origin.getZ();
      Eigen::Quaterniond quat;
      quat.x() = q.x();
      quat.y() = q.y();
      quat.z() = q.z();
      quat.w() = q.w();
      quat.normalize();
      odom.rotation() = quat.toRotationMatrix();
    
      carto_lpf_(0) = odom.translation()(0);
      carto_lpf_(1) = odom.translation()(1);
      carto_lpf_(2) = atan2(-odom.rotation()(0, 1), odom.rotation()(0,0));

      if (!iscarto_){
        carto_lpf_prev_ = carto_lpf_;
        iscarto_ = true;
      }

      odom_lpf_(0) = this->lowpassFilter(0.001, carto_lpf_(0), carto_lpf_prev_(0), 100);
      odom_lpf_(1) = this->lowpassFilter(0.001, carto_lpf_(1), carto_lpf_prev_(1), 100);
      odom_lpf_(2) = this->lowpassFilter(0.001, carto_lpf_(2), carto_lpf_prev_(2), 100);

      carto_lpf_prev_ = carto_lpf_;  
  }
  catch (tf::TransformException ex){
    iscarto_ = true;
  }
  

  // if (print_rate_trigger_())
  // {
  //   ROS_INFO("--------------------------------------------------");
  //   ROS_WARN_STREAM(odom_lpf_);
  // }
  
  // Husky update
  wheel_vel_(0) = husky_state_msg_.velocity[1]; // left vel
  wheel_vel_(1) = husky_state_msg_.velocity[0]; // right vel (not use.)

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(odom_lpf_(0), odom_lpf_(1), 0.0 ));
  
  q.setRPY(0, 0, odom_lpf_(2));
  transform.setRotation(q);
  br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "husky_odom", group_name_ + "_rviz_base_link"));
  
  robot_state_msg_.position[0] = husky_state_msg_.position[1];
  robot_state_msg_.position[1] = husky_state_msg_.position[0];
  robot_state_msg_.velocity[0] = wheel_vel_(0);
  robot_state_msg_.velocity[1] = wheel_vel_(1);

  // Franka update
  
  for (int i=0; i<7; i++){
      robot_state_msg_.position[i+2] = franka_q(i);
      robot_state_msg_.velocity[i+2] = dq_filtered_(i);
  }

  // HQP thread
  
  if (calculation_mutex_.try_lock())
  {
    calculation_mutex_.unlock();
    if (async_calculation_thread_.joinable())
      async_calculation_thread_.join();

    async_calculation_thread_ = std::thread(&BasicHuskyFrankaController::asyncCalculationProc, this);
  }

  ros::Rate r(30000);
  for (int i = 0; i < 9; i++)
  {
    r.sleep();
    if (calculation_mutex_.try_lock())
    {
      calculation_mutex_.unlock();
      if (async_calculation_thread_.joinable())
        async_calculation_thread_.join();
      break;
    }
  }
  
  
  ctrl_->compute(time_);
  
  ctrl_->franka_output(franka_qacc_); 
  ctrl_->husky_output(husky_qacc_); 

  ctrl_->state(state_);                    //not used now

 // ctrl_->mass(robot_mass_);
  robot_mass_(4, 4) *= 6.0;
  robot_mass_(5, 5) *= 6.0;
  robot_mass_(6, 6) *= 10.0;
  
  franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;

  MatrixXd Kd(7, 7);
  Kd.setIdentity();
  Kd = 2.0 * sqrt(5.0) * Kd;
  Kd(5, 5) = 0.2;
  Kd(4, 4) = 0.2;
  Kd(6, 6) = 0.2; // this is practical term
  franka_torque_ -= Kd * dq_filtered_;  
  franka_torque_ << this->saturateTorqueRate(franka_torque_, robot_tau_);

  double thres = 1.0;
  if (ctrl_->ctrltype() != 0){
    if (mob_type_ == 1){
      franka_torque_ -= robot_J_.transpose().col(0) * f_filtered_(0) * thres;
      franka_torque_ -= robot_J_.transpose().col(1) * f_filtered_(1);
      franka_torque_ += robot_J_.transpose().col(2) * f_filtered_(2);
    }
    else if (mob_type_ == 2){
      franka_torque_ += robot_J_.transpose().col(0) * f_filtered_(0);
      franka_torque_ += robot_J_.transpose().col(1) * f_filtered_(1);
      franka_torque_ += robot_J_.transpose().col(2) * f_filtered_(2);
    }
  }

  husky_qvel_ = husky_qacc_ * 0.01;//  + husky_qvel_prev_;
  double thes_vel = 5.0;
  if (husky_qvel_(0) > thes_vel)
    husky_qvel_(0) = thes_vel;
  else if (husky_qvel_(0) < -thes_vel)
    husky_qvel_(0) = -thes_vel;
  if (husky_qvel_(1) > thes_vel)
    husky_qvel_(1) = thes_vel;
  else if (husky_qvel_(1) < -thes_vel)
    husky_qvel_(1) = -thes_vel;  
  
  if (abs(husky_qvel_(0)) < 0.1)
    husky_qvel_(0) = 0.0;
  if (abs(husky_qvel_(1)) < 0.1)
    husky_qvel_(1) = 0.0;

    
  husky_cmd_(0) = 0.165 * (husky_qvel_(0) + husky_qvel_(1)) / 2.0;
  husky_cmd_(1) = (-husky_qvel_(0) + husky_qvel_(1)) * 0.165;
  husky_cmd_ *= 1.0;

  husky_qvel_prev_ = husky_qvel_;
  if (ctrl_->reset_control_)
    husky_qvel_prev_.setZero();

  this->setFrankaCommand();
  this->setHuskyCommand();

  this->getEEState();
  this->getBaseState();

  // if (print_rate_trigger_())
  // {
  //   ROS_INFO("--------------------------------------------------");
  //   ROS_INFO_STREAM("franka_torque_ :" << franka_torque_.transpose());
  //   ROS_INFO_STREAM("husky_cmd_ :" << husky_cmd_.transpose());
  // }
  
//  franka_torque_.setZero();
//  husky_cmd_.setZero();

  for (int i = 0; i < 7; i++)
    joint_handles_[i].setCommand(franka_torque_(i));

  if (husky_base_control_trigger_())
  {
    if (husky_ctrl_pub_.trylock())
    {
      husky_ctrl_pub_.msg_.linear.x = husky_cmd_(0);
      husky_ctrl_pub_.msg_.angular.z = husky_cmd_(1);
      husky_ctrl_pub_.unlockAndPublish();
    }
  }

  time_ += 0.001;
  time_msg_.data = time_;
  time_pub_.publish(time_msg_);
  joint_state_pub_.publish(robot_state_msg_);
  torque_state_pub_.publish(robot_command_msg_);

  // if (print_rate_trigger_())
  // {
  //   ROS_INFO("--------------------------------------------------");
  //   ROS_INFO_STREAM("odom_lpf_ :" << odom_lpf_.transpose());
  // }

}
void BasicHuskyFrankaController::stopping(const ros::Time& time){
    ROS_INFO("Robot Controller::stopping");
}
void BasicHuskyFrankaController::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_msg_ = *msg;
}
void BasicHuskyFrankaController::huskystateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    if (msg->name.size()==4)
      husky_state_msg_ = *msg;

    
}
void BasicHuskyFrankaController::ctrltypeCallback(const std_msgs::Int16ConstPtr &msg){
    // calculation_mutex_.lock();
    ROS_INFO("[ctrltypeCallback] %d", msg->data);
    
    if (msg->data != 899){
        int data = msg->data;
        husky_qvel_prev_.setZero();
        ctrl_->ctrl_update(data);
        mob_type_ = 0;
        if  (msg->data == 888)
          mob_type_ = 1;
        if (msg->data == 887)
          mob_type_ = 2;
        if (msg->data == 11)
          mob_type_ = 1;

    }
    else {
        if (isgrasp_){
            isgrasp_=false;
            franka_gripper::MoveGoal goal;
            goal.speed = 0.1;
            goal.width = 0.08;
            gripper_ac_.sendGoal(goal);
        }
        else{

            isgrasp_ = true; 
            franka_gripper::GraspGoal goal;
            franka_gripper::GraspEpsilon epsilon;
            epsilon.inner = 0.02;
            epsilon.outer = 0.05;
            goal.speed = 0.1;
            goal.width = 0.02;
            goal.force = 80.0;
            goal.epsilon = epsilon;
            gripper_grasp_ac_.sendGoal(goal);
        }
    }
    // calculation_mutex_.unlock();
}
void BasicHuskyFrankaController::mobtypeCallback(const std_msgs::Int16ConstPtr &msg){
    // calculation_mutex_.lock();
    ROS_INFO("[mobtypeCallback] %d", msg->data);
    mob_type_ = msg->data;
    // if (mob_type_ == 1)
    //   ctrl_->ctrl_update(888);
    
    // calculation_mutex_.unlock();
}
void BasicHuskyFrankaController::asyncCalculationProc(){
  calculation_mutex_.lock();
  
  ctrl_->husky_update(odom_lpf_, odom_dot_lpf_, Vector2d::Zero(), wheel_vel_);
  ctrl_->franka_update(franka_q_, dq_filtered_);

  calculation_mutex_.unlock();
}

Eigen::Matrix<double, 7, 1> BasicHuskyFrankaController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void BasicHuskyFrankaController::setFrankaCommand(){  
  robot_command_msg_.MODE = 1;
  robot_command_msg_.header.stamp = ros::Time::now();
  robot_command_msg_.time = time_;

  for (int i=0; i<7; i++)
      robot_command_msg_.torque[i+2] = franka_torque_(i);   
}

void BasicHuskyFrankaController::setHuskyCommand(){
  for (int i=0; i<2; i++)
      robot_command_msg_.torque[i] = husky_qacc_(i);   
}

void BasicHuskyFrankaController::getEEState(){
    Vector3d pos;
    Quaterniond q;
    ctrl_->ee_state(pos, q);

    ee_state_msg_.translation.x = pos(0);
    ee_state_msg_.translation.y = pos(1);
    ee_state_msg_.translation.z = pos(2);

    ee_state_msg_.rotation.x = q.x();
    ee_state_msg_.rotation.y = q.y();
    ee_state_msg_.rotation.z = q.z();
    ee_state_msg_.rotation.w = q.w();
    ee_state_pub_.publish(ee_state_msg_);
}
void BasicHuskyFrankaController::getBaseState(){
    Vector3d pos;
    ctrl_->base_state(pos);

    for (int i=0; i<3; i++)
        base_state_msg_.position[i] = pos(i);

    base_state_pub_.publish(base_state_msg_);
}
void BasicHuskyFrankaController::modeChangeReaderProc(){
  while (!quit_all_proc_)
  {
    char key = getchar();
    key = tolower(key);
    calculation_mutex_.lock();

    int msg = 0;
    switch (key){
      case 'g': //gravity
          msg = 0;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Gravity mode" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;
      case 'h': //home
          msg = 1;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Initialization" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;
      case 'a': //home
          msg = 2;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;    
      case 's': //home
          msg = 3;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;    
      case 'd': //home
          msg = 4;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;  
      case 'f': //home
          msg = 5;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;   
      case 'c': //home
          msg = 888;
          ctrl_->ctrl_update(msg);
          mob_type_ = 1;

          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          husky_qvel_prev_.setZero();
          break;   
      case 'z': //grasp
          msg = 899;
          if (isgrasp_){
              cout << "Release hand" << endl;
              isgrasp_ = false;
              franka_gripper::MoveGoal goal;
              goal.speed = 0.1;
              goal.width = 0.08;
              gripper_ac_.sendGoal(goal);
          }
          else{
              cout << "Grasp object" << endl;
              isgrasp_ = true; 
              franka_gripper::GraspGoal goal;
              franka_gripper::GraspEpsilon epsilon;
              epsilon.inner = 0.02;
              epsilon.outer = 0.05;
              goal.speed = 0.1;
              goal.width = 0.02;
              goal.force = 80.0;
              goal.epsilon = epsilon;
              gripper_grasp_ac_.sendGoal(goal);
          }
          break;
      case '\n':
        break;
      case '\r':
        break;
      default:
        break;
    }
    
    calculation_mutex_.unlock();
  }
}

} // namespace kimm_franka_controllers


PLUGINLIB_EXPORT_CLASS(kimm_husky_controllers::BasicHuskyFrankaController, controller_interface::ControllerBase)
