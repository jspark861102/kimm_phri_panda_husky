
#include <kimm_phri_panda_husky/phri_real.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <pinocchio/parsers/urdf.hpp>

namespace kimm_franka_husky_controllers
{

bool pHRIFrankaHuskyController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{

  n_node_ = node_handle; 
  node_handle.getParam("/robot_group", group_name_);    
  node_handle.getParam("/load_gripper", load_gripper_);    
  node_handle.getParam("/robotiq_gripper", isrobotiq_);    
  node_handle.getParam("/mobile", ismobile_);      
  node_handle.getParam("/issimulation", issimulation_);   
  node_handle.getParam("/iscalibration", iscalibration_);       
  cout << "load_gripper   " << load_gripper_ << endl;
  cout << "robotiq_gripper   " << isrobotiq_ << endl;
  cout << "ismobile   " << ismobile_ << endl;
  cout << "issimulation   " << issimulation_ << endl;
  cout << "iscalibration   " << iscalibration_ << endl;
  
  ctrl_type_sub_ = node_handle.subscribe("/" + group_name_ + "/real_robot/ctrl_type", 1, &pHRIFrankaHuskyController::ctrltypeCallback, this);
  mob_subs_ = node_handle.subscribe("/" + group_name_ + "/real_robot/mob_type", 1, &pHRIFrankaHuskyController::mobtypeCallback, this);
  
  torque_state_pub_ = node_handle.advertise<mujoco_ros_msgs::JointSet>("/" + group_name_ + "/real_robot/joint_set", 5);
  joint_state_pub_ = node_handle.advertise<sensor_msgs::JointState>("/" + group_name_ + "/real_robot/joint_states", 5);
  ee_state_pub_ = node_handle.advertise<geometry_msgs::Transform>("/" + group_name_ + "/real_robot/ee_state", 5);  

  if (ismobile_) {
    //mobile
    husky_ctrl_pub_.init(node_handle, "/" + group_name_ + "/husky/cmd_vel", 4);
    husky_odom_pub_ = node_handle.advertise<visualization_msgs::Marker>("husky_odom", 1);    
    base_state_pub_ = node_handle.advertise<sensor_msgs::JointState>("/" + group_name_ + "/real_robot/base_state", 5);

    odom_subs_ = node_handle.subscribe( "/" + group_name_ + "/husky/odometry/filtered", 1, &pHRIFrankaHuskyController::odomCallback, this);    
    husky_state_subs_ = node_handle.subscribe( "/" + group_name_ + "/husky/joint_states", 1, &pHRIFrankaHuskyController::huskystateCallback, this);  

    teleop_joy_subs_ = node_handle.subscribe("/" + group_name_ + "/husky/joy_teleop/joy", 1, &pHRIFrankaHuskyController::teleopjoyCallback, this);            
  }

  if (load_gripper_){
    franka_gripper_state_subs_ = node_handle.subscribe( "/franka_gripper/joint_states", 1, &pHRIFrankaHuskyController::frankagripperstateCallback, this);              
  }

  if (isrobotiq_) {
    // robotiq_state_subs_ = node_handle.subscribe( "/" + group_name_ + "/robotiq/joint_states", 1, &pHRIFrankaHuskyController::robotiqstateCallback, this);              
    robotiq_state_subs_ = node_handle.subscribe( "/robotiq/joint_states", 1, &pHRIFrankaHuskyController::robotiqstateCallback, this);              
  }

  // object estimation 
  object_parameter_pub_ = node_handle.advertise<kimm_phri_msgs::ObjectParameter>("/" + group_name_ + "/real_robot/object_parameter", 5);
  Fext_local_forObjectEstimation_pub_ = node_handle.advertise<geometry_msgs::Wrench>("/" + group_name_ + "/real_robot/Fext_local_forObjectEstimation", 5);
  Fext_global_pub_ = node_handle.advertise<geometry_msgs::Wrench>("/" + group_name_ + "/real_robot/Fext_global", 5);
  vel_pub_ = node_handle.advertise<geometry_msgs::Twist>("/" + group_name_ + "/real_robot/object_velocity", 5);
  accel_pub_ = node_handle.advertise<geometry_msgs::Twist>("/" + group_name_ + "/real_robot/object_acceleration", 5);   

  test_pub_ = node_handle.advertise<geometry_msgs::Wrench>("/" + group_name_ + "/real_robot/test", 5);

  // ************ object estimation *************** //               
  isstartestimation_ = false;  
  F_ext_bias_init_flag = false;
  this->getObjParam_init();  
  isobjectdynamics_ = false;
  ismasspractical_ = true;  
  est_time_ = 0.0;    
  // ********************************************** //   
  
  isgrasp_ = false;      
  if (load_gripper_) {
    gripper_ac_.waitForServer();
    gripper_grasp_ac_.waitForServer();
  }

  if (isrobotiq_) {  
    gripper_robotiq_ = new robotiq_2f_gripper_control::RobotiqActionClient("/ns1/command_robotiq_action", true);    
  }

  std::vector<std::string> joint_names;
  std::string arm_id;  
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Exception getting joint handles: " << ex.what());
      return false;
    }
  }  

  //keyboard event, this code begins from here  
  mode_change_thread_ = std::thread(&pHRIFrankaHuskyController::modeChangeReaderProc, this);

  ctrl_ = new RobotController::FrankaHuskyWrapper(group_name_, issimulation_, ismobile_, isrobotiq_, iscalibration_, node_handle);
  ctrl_->initialize();   

  traj_length_in_time_ = ctrl_->trajectory_length_in_time();

  //dynamic_reconfigure    
  ekf_param_node_ = ros::NodeHandle("ekf_param_node");
  ekf_param_ = std::make_unique<dynamic_reconfigure::Server<kimm_phri_panda_husky::ekf_paramConfig>>(ekf_param_node_);
  ekf_param_->setCallback(boost::bind(&pHRIFrankaHuskyController::ekfParamCallback, this, _1, _2));

  // gui_model for joint names
  string model_path, urdf_name;
  node_handle.getParam("/" + group_name_ +"/gui_urdf_path", model_path);    
  node_handle.getParam("/" + group_name_ +"/gui_urdf_name", urdf_name);  
  vector<string> package_dirs;
  package_dirs.push_back(model_path);
  string urdfFileName = package_dirs[0] + urdf_name;

  pinocchio::Model gui_model;
  pinocchio::urdf::buildModel(urdfFileName, gui_model, false);    

  robot_state_msg_.name.resize(gui_model.nq);
  robot_state_msg_.position.resize(gui_model.nq);
  robot_state_msg_.velocity.resize(gui_model.nq);
  robot_state_msg_.effort.resize(gui_model.nq);

  for (int j=0; j<gui_model.nq; j++){
      robot_state_msg_.name[j] = gui_model.names[j+1]; //names[0]="universe"
      robot_state_msg_.position[j] = 0.0;
      robot_state_msg_.velocity[j] = 0.0;
      robot_state_msg_.effort[j] = 0.0;
  }

  //initialize
  mob_type_ = 0;  
  ee_state_msg_ = geometry_msgs::Transform();
  if (ismobile_) {
    robot_command_msg_.torque.resize(9); // 2 (wheel) + 7 (franka) //in real, just for monitoring   

    base_state_msg_.position.resize(3);  // from pinocchio to here (x, y, theata), just for monitoring
    base_state_msg_.velocity.resize(3);  // from pinocchio to here (x, y, theata), just for monitoring

    husky_state_msg_.position.resize(4); // from real husky to here to update state
    husky_state_msg_.velocity.resize(4); // from real husky to here to update state
  }
  else { //ismobile_ = false
    robot_command_msg_.torque.resize(7); // 7 (franka) //in real, just for monitoring         
  }

  if (load_gripper_) {
    franka_gripper_state_msg_.position.resize(2);
    franka_gripper_state_msg_.velocity.resize(2);
  }
  else if (isrobotiq_) {
    robotiq_state_msg_.position.resize(1);
    robotiq_state_msg_.velocity.resize(1);
  }

  return true;
}

void pHRIFrankaHuskyController::starting(const ros::Time& time) {  
  ROS_INFO("Robot Controller::starting");

  time_ = 0.;
  dt_ = 0.001;
  ctrl_->get_dt(dt_);      

  if (ismobile_) {    
    // *********************** husky odometry update *************************** //               
    odom_lpf_prev_(0) = odom_msg_.pose.pose.position.x;
    odom_lpf_prev_(1) = odom_msg_.pose.pose.position.y;
    odom_lpf_prev_(2) = odom_msg_.pose.pose.orientation.z;

    odom_dot_lpf_prev_(0) = odom_msg_.twist.twist.linear.x;
    odom_dot_lpf_prev_(1) = odom_msg_.twist.twist.linear.y;
    odom_dot_lpf_prev_(2) = odom_msg_.twist.twist.angular.z;  

    husky_qvel_prev_.setZero(2);

    // ************ husky odom marker publish (initial position) *************** //               
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

  dq_filtered_.setZero();
  f_filtered_.setZero();      
  fh_force_.setZero();  
  fh_force_local_.setZero();  
}

void pHRIFrankaHuskyController::update(const ros::Time& time, const ros::Duration& period) {    

  // ROS_INFO("I am here1");
  // modeChangeReaderProc(); // test for not to use of thread, it is not working well.
  // ROS_INFO("I am here4");
  
  //update franka variables----------------------------------------------------------------------//  
  //franka model_handle -------------------------------//
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 49> massmatrix_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();  

  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  robot_J_ = jacobian; //Gets the 6x7 Jacobian for the given joint relative to the base frame
  
  Eigen::Map<Vector7d> gravity(gravity_array.data());
  robot_g_ = gravity; //Calculates the gravity vector [Nm]

  Eigen::Map<Matrix7d> mass_matrix(massmatrix_array.data());
  robot_mass_ = mass_matrix; //Calculates the 7x7 mass matrix [kg*m^2]

  Eigen::Map<Vector7d> non_linear(coriolis_array.data());
  robot_nle_ = non_linear; //Calculates the Coriolis force vector (state-space equation) [Nm]

  // can be used
  //model_handle_->getpose(); //Gets the 4x4 pose matrix for the given frame in base frame
  //model_handle_->getBodyJacobian(); //Gets the 6x7 Jacobian for the given frame, relative to that frame
  
  //franka state_handle -------------------------------//
  franka::RobotState robot_state = state_handle_->getRobotState();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
  robot_tau_ = tau_J; //Measured link-side joint torque sensor signals [Nm]

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  robot_tau_d_ = tau_J_d; //Desired link-side joint torque sensor signals without gravity [Nm]    
  
  Eigen::Map<Vector7d> franka_q(robot_state.q.data());
  franka_q_ = franka_q; //Measured joint position [rad] 

  Eigen::Map<Vector7d> franka_dq(robot_state.dq.data());
  franka_dq_ = franka_dq; //Measured joint velocity [rad/s]  

  Eigen::Map<Vector7d> franka_dq_d(robot_state.dq_d.data());
  franka_dq_d_ = franka_dq_d; //Desired joint velocity [rad/s]  
  
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_franka(robot_state.O_F_ext_hat_K.data());
  f_ = -1 * force_franka; //Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame.     

  // can be used
  //robot_state.tau_ext_hat_filtered.data(); //External torque, filtered. [Nm]

  //franka End-Effector Frame -----------------------------------------------------------------//  
  // 1. Nominal end effector frame NE : The nominal end effector frame is configure outsuide of libfranka and connot changed here.
  // 2. End effector frame EE : By default, the end effector frame EE is the same as the nominal end effector frame NE (i.e, the transformation between NE and EE is the identity transformation)
  //                            With Robot::setEE, a custom transformation matrix can be set
  // 3. Stiffness frame K : The stiffness frame is used for Cartesian impedance control, and for measuring and applying forces. I can be set with Robot::setK
  
  //filtering ---------------------------------------------------------------------------------//  
  dq_filtered_ = this->lowpassFilter( dt_,  franka_dq_,  dq_filtered_,       20.0); //in Hz, Vector7d
  f_filtered_  = this->lowpassFilter( dt_,  f_,          f_filtered_,        5.0); //in Hz, Vector6d
  ctrl_->Fext_update(f_filtered_);    // send Fext to controller

  if (ismobile_) {
    //husky odom update
    odom_lpf_(0) = this->lowpassFilter(dt_, odom_msg_.pose.pose.position.x, odom_lpf_prev_(0), 100);
    odom_lpf_(1) = this->lowpassFilter(dt_, odom_msg_.pose.pose.position.y, odom_lpf_prev_(1), 100);
    odom_lpf_(2) = this->lowpassFilter(dt_, odom_msg_.pose.pose.orientation.z, odom_lpf_prev_(2), 100);    

    odom_dot_lpf_(0) = this->lowpassFilter(dt_, odom_msg_.twist.twist.linear.x, odom_dot_lpf_prev_(0), 100);
    odom_dot_lpf_(1) = this->lowpassFilter(dt_, odom_msg_.twist.twist.linear.y, odom_dot_lpf_prev_(1), 100);
    odom_dot_lpf_(2) = this->lowpassFilter(dt_, odom_msg_.twist.twist.angular.z, odom_dot_lpf_prev_(2), 100);

    odom_lpf_prev_ = odom_lpf_;
    odom_dot_lpf_prev_ = odom_dot_lpf_;

    //tf
    tf::Quaternion q;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odom_lpf_(0), odom_lpf_(1), 0.0 ));  
    q.setRPY(0, 0, odom_lpf_(2));
    transform.setRotation(q);
    br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "husky_odom", group_name_ + "_rviz_base_link"));
  }
  
  // thread for franka state update to HQP -----------------------------------------------------//
  if (calculation_mutex_.try_lock())
  {
    calculation_mutex_.unlock();
    if (async_calculation_thread_.joinable())
      async_calculation_thread_.join();

    //asyncCalculationProc -->  ctrl_->franka_update(franka_q_, dq_filtered_);
    async_calculation_thread_ = std::thread(&pHRIFrankaHuskyController::asyncCalculationProc, this);        
  }
  
  double thread_count;
  if (ismobile_) thread_count = 9;
  else           thread_count = 7;
  ros::Rate r(30000);
  for (int i = 0; i < thread_count; i++)
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
  
  // compute HQP controller --------------------------------------------------------------------//
  //obtain from panda_husky_hqp --------------------//
  ctrl_->compute(time_);  
  
  if (ismobile_) ctrl_->husky_output(husky_qacc_); 
  ctrl_->franka_output(franka_qacc_);   

  ctrl_->mass(robot_mass_pin_); 

  //for object estimation--------------------//  
  // ctrl_->g_joint7(robot_g_local_);    
  // ctrl_->JLocal(robot_J_local_);     //local joint7
  // ctrl_->dJLocal(robot_dJ_local_);   //local joint7
  ctrl_->g_local_offset(robot_g_local_);    //local
  ctrl_->JLocal_offset(robot_J_local_);     //local 
  ctrl_->dJLocal_offset(robot_dJ_local_);   //local   
  
  // ctrl_->position(oMi_);
  // ctrl_->position_offset(oMi_);
  ctrl_->position_link0_offset(oMi_);
  ctrl_->Fh_gain_matrix(Fh_, Fh_local_); //local

  //for practical manipulation--------------------//    
  robot_mass_practical_ = robot_mass_;
  robot_mass_practical_(4, 4) *= 6.0;                 //rotational inertia increasing
  robot_mass_practical_(5, 5) *= 6.0;                 //rotational inertia increasing
  robot_mass_practical_(6, 6) *= 10.0;                //rotational inertia increasing

  if (ismasspractical_) {
    franka_torque_ = robot_mass_practical_ * franka_qacc_ + robot_nle_;  
  }
  else {
    franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;  
  }

  MatrixXd Kd(7, 7); 
  Kd.setIdentity();
  Kd = 2.0 * sqrt(5.0) * Kd;
  Kd(5, 5) = 0.2;
  Kd(4, 4) = 0.2;
  Kd(6, 6) = 0.2; 
  franka_torque_ -= Kd * dq_filtered_;  //additional damping torque (independent of mass matrix)

  // apply object dynamics---------------------------------------------------------------//  
  param_d435_ << 0.07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
  param_tip_ << 0.12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //customized tip mass : 130g, original tip mass : 10g  
  // param_true_ << 0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //profile mass = 0.7kg, half for robot and half for human
  // param_true_ << 0.77, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //profile mass = 1.54kg, half for robot and half for human
  param_true_ << 1.115, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //profile mass = 2.23kg, half for robot and half for human

  if (isobjectdynamics_) {    
    // FT_object_ = objdyn.h(param_used_ + param_d435_ + param_tip_, vel_param.toVector(), acc_param.toVector(), robot_g_local_);        
    // FT_object_ = objdyn.h(param_true_ + param_d435_ + param_tip_, vel_param.toVector(), acc_param.toVector(), robot_g_local_);        
    FT_object_ = objdyn.h(param_true_ , vel_param.toVector(), acc_param.toVector(), robot_g_local_);        
  }    
  else {
    // FT_object_ = objdyn.h(              param_d435_ + param_tip_, vel_param.toVector(), acc_param.toVector(), robot_g_local_);    
    FT_object_.setZero();
  }     
  franka_torque_ -= robot_J_local_.transpose() * FT_object_;
  
  //apply Fext to the franka_torque_ for compliance or robust control--------------------//        
  fh_force_ =  Fh_ * (f_filtered_ - FT_local_to_global(FT_object_));     
  fh_force_local_ = Fh_local_ * (FT_global_to_local(f_filtered_) - FT_object_);      

  if (ctrl_->ctrltype() != 0){
    if (mob_type_ == 1){ //compliance with Fext    
        //object teaching : z axis from rx axis               
        franka_torque_ += robot_J_.transpose() * fh_force_;                      

        //object teaching : local y axis from rz axis
        franka_torque_ += robot_J_local_.transpose() * fh_force_local_;               
    }
    else if (mob_type_ == 2){ //robust with global Fext
      franka_torque_ -= robot_J_.transpose().col(0) * f_filtered_(0); //robust
      franka_torque_ -= robot_J_.transpose().col(1) * f_filtered_(1); //robust
      franka_torque_ -= robot_J_.transpose().col(2) * f_filtered_(2); //robust      
    }
  }  
  this->test_publication();  

  // torque saturation--------------------//  
  franka_torque_ << this->saturateTorqueRate(franka_torque_, robot_tau_d_);

  //send control input to franka--------------------//
  for (int i = 0; i < 7; i++)
    joint_handles_[i].setCommand(franka_torque_(i));  

  if (ismobile_) {    
    //send control input to husky--------------------//
    husky_qvel_ = husky_qacc_ * 0.01;//  + husky_qvel_prev_;    
    double thes_vel = 5.0; // husky_qacc < 500
    if (husky_qvel_(0) > thes_vel)
      husky_qvel_(0) = thes_vel;
    else if (husky_qvel_(0) < -thes_vel)
      husky_qvel_(0) = -thes_vel;
    if (husky_qvel_(1) > thes_vel)
      husky_qvel_(1) = thes_vel;
    else if (husky_qvel_(1) < -thes_vel)
      husky_qvel_(1) = -thes_vel;  
    
    if (abs(husky_qvel_(0)) < 0.07) //husky_qacc < 7
      husky_qvel_(0) = 0.0;
    if (abs(husky_qvel_(1)) < 0.07) //husky_qacc < 7
      husky_qvel_(1) = 0.0;
      
    husky_cmd_(0) = 0.165 * (husky_qvel_(0) + husky_qvel_(1)) / 2.0;
    husky_cmd_(1) = (-husky_qvel_(0) + husky_qvel_(1)) * 0.165;
    husky_cmd_ *= 1.0;

    husky_qvel_prev_ = husky_qvel_;
    if (ctrl_->reset_control_) husky_qvel_prev_.setZero();
  
    if (husky_base_control_trigger_())
    {
      if (husky_ctrl_pub_.trylock())
      {
        husky_ctrl_pub_.msg_.linear.x = husky_cmd_(0);
        husky_ctrl_pub_.msg_.angular.z = husky_cmd_(1);
        husky_ctrl_pub_.unlockAndPublish();
      }
    }
  }

  //Publish ------------------------------------------------------------------------//
  time_ += dt_;
  time_msg_.data = time_;

  if (ismobile_) this->pubBaseState(); //just update base_state_msg_ by pinocchio and publish it
  this->pubEEState();                  //just update ee_state_msg_ by pinocchio and publish it
  this->pubJointStates();
  this->pubFextGlobal();  
    
  if (ismobile_) this->setHuskyCommand();  //just update robot_command_msg_ by husky_qacc_
  this->setFrankaCommand();                //just update robot_command_msg_ by franka_torque_   
  torque_state_pub_.publish(robot_command_msg_);  

  //Object estimation --------------------------------------------------------------//
  // ************ object estimation *************** //              
  this->vel_accel_pub();        
  this->FT_measured_pub();        
  this->getObjParam();                                    // object estimation
  this->ObjectParameter_pub();                            // data plot for monitoring
  // ********************************************** //    

  //Debug ------------------------------------------------------------------------//
  if (print_rate_trigger_())
  {
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("robot_mass_ :" << robot_mass_);
    // ROS_INFO_STREAM("odom_lpf_ :" << odom_lpf_.transpose());
    // ROS_INFO_STREAM("robot_g_local" << robot_g_local_(0) << robot_g_local_(1) << robot_g_local_(2));    
  }
}

void pHRIFrankaHuskyController::stopping(const ros::Time& time){
    ROS_INFO("Robot Controller::stopping");              
} 

void pHRIFrankaHuskyController::test_publication() {    
  geometry_msgs::Wrench test_msg;  
  test_msg.force.x = fh_force_(0);
  test_msg.force.y = fh_force_(1);
  test_msg.force.z = fh_force_(2);

  // test_msg.torque.x =fh_force_(3);
  // test_msg.torque.y =fh_force_(4);
  // test_msg.torque.z =fh_force_(5);

  test_msg.torque.x =fh_force_local_(0);
  test_msg.torque.y =fh_force_local_(1);
  test_msg.torque.z =fh_force_local_(2);

  test_pub_.publish(test_msg);    
}

// ************************************************ object estimation start *************************************************** //                       
void pHRIFrankaHuskyController::ekfParamCallback(kimm_phri_panda_husky::ekf_paramConfig& config, uint32_t level) {
  Q(0,0) = config.Q0;  
  Q(1,1) = config.Q1;  
  Q(2,2) = config.Q2;  
  Q(3,3) = config.Q3;  
  Q(4,4) = config.Q4;  
  Q(5,5) = config.Q5;  
  Q(6,6) = config.Q6;  
  Q(7,7) = config.Q7;  
  Q(8,8) = config.Q8;  
  Q(9,9) = config.Q9;  

  R(0,0) = config.R0;  
  R(1,1) = config.R1;  
  R(2,2) = config.R2;  
  R(3,3) = config.R3;  
  R(4,4) = config.R4;  
  R(5,5) = config.R5;  
  
  ROS_INFO("ekf parameter from dynamic_reconfigure--------------------------------------------------");
  ROS_INFO_STREAM("Q" << "  " << Q(0,0) << "  " << Q(1,1) << "  " << Q(2,2) << "  " << Q(3,3) );
  ROS_INFO_STREAM("Q" << "  " << Q(4,4) << "  " << Q(5,5) << "  " << Q(6,6) << "  " << Q(7,7) << "  " << Q(8,8) << "  " << Q(9,9));
  ROS_INFO_STREAM("R" << "  " << R(0,0) << "  " << R(1,1) << "  " << R(2,2) << "  " << R(3,3) << "  " << R(4,4) << "  " << R(5,5) );
}

void pHRIFrankaHuskyController::vel_accel_pub(){
    //************* obtained from pinocchio ***********// 
    //**** velocity (data.v) is identical with franka velocity dq_filtered_, 
    //**** but acceleration (data.a) is not reasnoble for both local and global cases.
    // ctrl_->velocity(vel_param);                //LOCAL
    // ctrl_->acceleration(acc_param);            //LOCAL
    
    // ctrl_->velocity_origin(vel_param);         //GLOBAL
    // ctrl_->acceleration_origin2(acc_param);     //GLOBAL                

    //************* obtained from franka and pinocchio : LOCAL **************//            
    if (isstartestimation_) { //start only when the robot is in proper state      
      // -------------------------- Obtain acceleration : 1. obtain from franka torque sensor   ------------------------------------------------//
      if(tau_bias_init_flag) {   //due to the bias of torque sensor in franka, initial bias should be corrected when estimation is started (See force_example_controller.cpp in franka_ros)
        // torque_sensor_bias_ = robot_tau_ - robot_g_;  
        tau_bias_init_flag = false;
      }      
      //ddq from this method is not a acceleration behavior, but torque behavior (bias is presented in case of the no motion) 
      // franka_ddq_for_param_ = robot_mass_.inverse() * (robot_tau_ - robot_g_ - torque_sensor_bias_) - robot_nle_;       
      
      // --------------------------  Obtain acceleration : 2. obtain by derivative of dq -------------------------------------------------------//
      //ddq from derivative of dq is very noisy, but for now, there is no other way to obatin ddq (0930)
      franka_ddq_for_param_ = (franka_dq_ - franka_dq_prev_)/dt_;
      franka_dq_prev_ = franka_dq_;      

      //obtain cartesian velocity and acceleration w.r.t. LOCAL frame -----------------------------------------------------------------------//
      franka_v_.setZero();
      franka_a_.setZero();
      for (int i=0; i<7; i++){         
          franka_v_ += robot_J_local_.col(i) * dq_filtered_[i];                                                     
          franka_a_ += robot_dJ_local_.col(i) * franka_dq_[i] + robot_J_local_.col(i) * franka_ddq_for_param_[i];   
      }              
    }
    else { //no estimation state
      // torque_sensor_bias_.setZero();
      franka_ddq_for_param_.setZero();
      franka_a_.setZero();

      franka_v_.setZero();
      for (int i=0; i<7; i++){ //dq is measured signal so, show regradless of estimation on/off
          franka_v_ += robot_J_local_.col(i) * dq_filtered_[i];                                                     
      }              
    }

    // Filtering
    franka_a_filtered_ = lowpassFilter( dt_, franka_a_, franka_a_filtered_, 5.0); //in Hz, Vector6d

    //convert to pinocchio::Motion -----------------------------------------------------------------------------------------------------------//
    vel_param.linear()[0] = franka_v_(0);
    vel_param.linear()[1] = franka_v_(1);
    vel_param.linear()[2] = franka_v_(2);            
    vel_param.angular()[0] = franka_v_(3);
    vel_param.angular()[1] = franka_v_(4);
    vel_param.angular()[2] = franka_v_(5);

    acc_param.linear()[0] = franka_a_filtered_(0);
    acc_param.linear()[1] = franka_a_filtered_(1);
    acc_param.linear()[2] = franka_a_filtered_(2);
    acc_param.angular()[0] = franka_a_filtered_(3);
    acc_param.angular()[1] = franka_a_filtered_(4);
    acc_param.angular()[2] = franka_a_filtered_(5);

    // acc_param.linear()[0] = 0.0;  //1027, acc is to nosy. So, only slow motion is used and acc is set to zero.
    // acc_param.linear()[1] = 0.0;
    // acc_param.linear()[2] = 0.0;
    // acc_param.angular()[0] = 0.0;
    // acc_param.angular()[1] = 0.0;
    // acc_param.angular()[2] = 0.0;

    //publish --------------------------------------------------------------------------------------------------------------------------------//
    geometry_msgs::Twist vel_msg, accel_msg;   

    vel_msg.linear.x = vel_param.linear()[0];
    vel_msg.linear.y = vel_param.linear()[1];
    vel_msg.linear.z = vel_param.linear()[2];
    vel_msg.angular.x = vel_param.angular()[0];
    vel_msg.angular.y = vel_param.angular()[1];
    vel_msg.angular.z = vel_param.angular()[2];

    accel_msg.linear.x = acc_param.linear()[0];
    accel_msg.linear.y = acc_param.linear()[1];
    accel_msg.linear.z = acc_param.linear()[2];
    accel_msg.angular.x = acc_param.angular()[0];
    accel_msg.angular.y = acc_param.angular()[1];
    accel_msg.angular.z = acc_param.angular()[2];

    vel_pub_.publish(vel_msg);
    accel_pub_.publish(accel_msg);
}

void pHRIFrankaHuskyController::FT_measured_pub() {        
    //obtained from pinocchio for the local frame--------//    
    pinocchio::Force f_global, f_local;    
    
    if (isstartestimation_) { //start only when the robot is in proper state            
      // if(F_ext_bias_init_flag) {   //F_ext bias to be corrected 
      //   F_ext_bias_ = f_filtered_;  
      //   F_ext_bias_init_flag = false;
      // } 
      
      if (time_ - est_time_ < traj_length_in_time_){              
        fin_ >> F_ext_bias_(0);
        fin_ >> F_ext_bias_(1);
        fin_ >> F_ext_bias_(2);
        fin_ >> F_ext_bias_(3);
        fin_ >> F_ext_bias_(4);
        fin_ >> F_ext_bias_(5);                    
      }
      else {
        fin_.close();

        cout << "end estimation" << endl;
        isstartestimation_ = false;

        param_used_ = param_estimated_;
        cout << "estimated parameter" << endl;
        cout << param_used_.transpose() << endl;                
      }
      
    }
    else {
      F_ext_bias_.setZero();
    }                  
    
    //GLOBAL ---------------------------------------//
    f_global.linear()[0] = f_filtered_(0) - F_ext_bias_(0);
    f_global.linear()[1] = f_filtered_(1) - F_ext_bias_(1);
    f_global.linear()[2] = f_filtered_(2) - F_ext_bias_(2);
    f_global.angular()[0] = f_filtered_(3) - F_ext_bias_(3);
    f_global.angular()[1] = f_filtered_(4) - F_ext_bias_(4);
    f_global.angular()[2] = f_filtered_(5) - F_ext_bias_(5);

    //LOCAL ---------------------------------------//              
    f_local = oMi_.actInv(f_global); //global to local        
    
    FT_measured[0] = f_local.linear()[0];
    FT_measured[1] = f_local.linear()[1];
    FT_measured[2] = f_local.linear()[2];
    FT_measured[3] = f_local.angular()[0];
    FT_measured[4] = f_local.angular()[1];
    FT_measured[5] = f_local.angular()[2];    
    
    //publish ---------------------------------------------------//
    geometry_msgs::Wrench FT_measured_msg;  

    FT_measured_msg.force.x = saturation(FT_measured[0],50);
    FT_measured_msg.force.y = saturation(FT_measured[1],50);
    FT_measured_msg.force.z = saturation(FT_measured[2],50);
    FT_measured_msg.torque.x = saturation(FT_measured[3],10);
    FT_measured_msg.torque.y = saturation(FT_measured[4],10);
    FT_measured_msg.torque.z = saturation(FT_measured[5],10);

    Fext_local_forObjectEstimation_pub_.publish(FT_measured_msg);    
}

void pHRIFrankaHuskyController::getObjParam(){
    // *********************************************************************** //
    // ************************** object estimation ************************** //
    // *********************************************************************** //
    // franka output     : robot_state        (motion) : 7 = 7(joint)               for pose & velocity, same with rviz jointstate
    // pinocchio input   : state_.q_, v_      (motion) : 7 = 7(joint)               for q,v
    // controller output : state_.torque_     (torque) : 7 = 7(joint)               for torque, pinocchio doesn't control gripper
    // franka input      : setCommand         (torque) : 7 = 7(joint)               gripper is controlle by other action client
    
    //*--- p cross g = (py*gz-pz*gy)i + (pz*gx-px*gz)j + (px*gy-py*gx)k ---*//

    //*--- FT_measured & vel_param & acc_param is LOCAL frame ---*//

    if (isstartestimation_) {

        h = objdyn.h(param_estimated_, vel_param.toVector(), acc_param.toVector(), robot_g_local_); 
        H = objdyn.H(param_estimated_, vel_param.toVector(), acc_param.toVector(), robot_g_local_); 

        ekf->update(FT_measured, dt_, A, H, h, Q, R); //Q & R update from dynamic reconfigure
        param_estimated_ = ekf->state();  
        
    }
}

void pHRIFrankaHuskyController::ObjectParameter_pub(){             
    //publish ---------------------------------------------------//
    kimm_phri_msgs::ObjectParameter objparam_msg;

    objparam_msg.com.resize(3);
    objparam_msg.inertia.resize(6);     

    objparam_msg.mass = saturation(param_estimated_[0],5.0);                
    
    objparam_msg.com[0] = saturation(param_estimated_[1],30.0);
    objparam_msg.com[1] = saturation(param_estimated_[2],30.0);
    objparam_msg.com[2] = saturation(param_estimated_[3],30.0);    

    object_parameter_pub_.publish(objparam_msg);              
}

void pHRIFrankaHuskyController::getObjParam_init(){
    n_param = 10;
    m_FT = 6;

    A.resize(n_param, n_param); // System dynamics matrix
    H.resize(m_FT,    n_param); // Output matrix
    Q.resize(n_param, n_param); // Process noise covariance
    R.resize(m_FT,    m_FT); // Measurement noise covariance
    P.resize(n_param, n_param); // Estimate error covariance
    h.resize(m_FT,    1); // observation      
    param_estimated_.resize(n_param); 
    param_used_.resize(n_param);   
    FT_measured.resize(m_FT);        
    FT_object_.resize(6); 
    param_true_.resize(n_param);
    param_d435_.resize(n_param);
    param_tip_.resize(n_param);
    Fh_.resize(m_FT,m_FT);
    Fh_local_.resize(m_FT,m_FT);
    isbutton_pushed_.resize(14); //ps5 joystick

    A.setIdentity();         //knwon, identity
    Q.setIdentity();         //design parameter
    R.setIdentity();         //design parameter    
    P.setIdentity();         //updated parameter
    h.setZero();             //computed parameter
    H.setZero();             //computed parameter    
    param_estimated_.setZero();   
    param_used_.setZero();
    FT_measured.setZero();
    vel_param.setZero();
    acc_param.setZero();    
    franka_dq_prev_.setZero();    
    FT_object_.setZero();
    param_true_.setZero();    
    param_d435_.setZero();    
    param_tip_.setZero();    
    Fh_.setZero();                             
    Fh_local_.setZero();          
    isbutton_pushed_.setZero();                   
    
    // Construct the filter
    ekf = new EKF(dt_, A, H, Q, R, P, h);
    
    // Initialize the filter  
    ekf->init(time_, param_estimated_);
}  

double pHRIFrankaHuskyController::saturation(double x, double limit) {
    if (x > limit) return limit;
    else if (x < -limit) return -limit;
    else return x;
}
// ************************************************ object estimation end *************************************************** //                       

void pHRIFrankaHuskyController::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_msg_ = *msg;
}

void pHRIFrankaHuskyController::huskystateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    if (msg->name.size()==4)
      husky_state_msg_ = *msg;    

    wheel_vel_(0) = husky_state_msg_.velocity[1]; // left vel
    wheel_vel_(1) = husky_state_msg_.velocity[0]; // right vel (not use.)
}

void pHRIFrankaHuskyController::frankagripperstateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    franka_gripper_state_msg_ = *msg;    
}

void pHRIFrankaHuskyController::robotiqstateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    robotiq_state_msg_ = *msg;   
}

void pHRIFrankaHuskyController::ctrltypeCallback(const std_msgs::Int16ConstPtr &msg){
    // calculation_mutex_.lock();
    ROS_INFO("[ctrltypeCallback] %d", msg->data);    
    
    if (msg->data != 899){
        int data = msg->data;        
        ctrl_->ctrl_update(data);        
        if(ismobile_) husky_qvel_prev_.setZero();
    }
    else {
      if (load_gripper_) { //franka gripper
        if (isgrasp_){
            cout << "Release hand" << endl;
            isgrasp_=false;
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
      }
      else if (isrobotiq_) { //robotiq gripper
        if (isgrasp_){
            cout << "Release hand" << endl;
            isgrasp_ = false;
            gripper_robotiq_->open();
        }
        else{
            cout << "Grasp object" << endl;
            isgrasp_ = true; 
            gripper_robotiq_->close(0.5, 220, false);                
        }
      }
      else {        
      }
    }
    // calculation_mutex_.unlock();
}
void pHRIFrankaHuskyController::mobtypeCallback(const std_msgs::Int16ConstPtr &msg){    
    ROS_INFO("[mobtypeCallback] %d", msg->data);
    mob_type_ = msg->data;
}
void pHRIFrankaHuskyController::asyncCalculationProc(){
  calculation_mutex_.lock();
  
  //franka & husky update --------------------------------------------------//  
  if (ismobile_) ctrl_->husky_update(odom_lpf_, odom_dot_lpf_, Vector2d::Zero(), wheel_vel_);
  ctrl_->franka_update(franka_q_, dq_filtered_);  

  //franka update with use of pinocchio::aba algorithm--------------//
  // ctrl_->franka_update(franka_q_, dq_filtered_, robot_tau_);
  // ctrl_->franka_update(franka_q_, dq_filtered_, robot_tau_ - robot_g_ - torque_sensor_bias_);

  calculation_mutex_.unlock();
}

Eigen::Matrix<double, 7, 1> pHRIFrankaHuskyController::saturateTorqueRate(
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

void pHRIFrankaHuskyController::pubJointStates(){  
  int index_count = 0;
  robot_state_msg_.header.stamp = ros::Time::now();    

  if (ismobile_) {
    // Husky update    
    robot_state_msg_.position[0] = husky_state_msg_.position[1];
    robot_state_msg_.position[1] = husky_state_msg_.position[0];
    robot_state_msg_.position[2] = husky_state_msg_.position[1];
    robot_state_msg_.position[3] = husky_state_msg_.position[0];

    robot_state_msg_.velocity[0] = husky_state_msg_.velocity[1]; // left vel
    robot_state_msg_.velocity[1] = husky_state_msg_.velocity[0]; // right vel (not use.)
    robot_state_msg_.velocity[2] = husky_state_msg_.velocity[1]; // left vel
    robot_state_msg_.velocity[3] = husky_state_msg_.velocity[0]; // right vel (not use.)

    index_count += 4;
  }

  if (1) { 
    // franka update
    for (int i=0; i<7; i++){  
      robot_state_msg_.position[i+index_count] = franka_q_(i);
      robot_state_msg_.velocity[i+index_count] = dq_filtered_(i);        
    }
    index_count += 7;
  }

  if (load_gripper_) { //franka_gripper
    for (int i=0; i<2; i++){  
      robot_state_msg_.position[i+index_count] = franka_gripper_state_msg_.position[i];
      robot_state_msg_.velocity[i+index_count] = franka_gripper_state_msg_.velocity[i];
    }    
    index_count += 2;
  }
  else if (isrobotiq_) { //robotiq gripper    
    robot_state_msg_.position[0+index_count] = +robotiq_state_msg_.position[0];
    robot_state_msg_.velocity[0+index_count] = +robotiq_state_msg_.velocity[0];

    robot_state_msg_.position[1+index_count] = +robotiq_state_msg_.position[0];
    robot_state_msg_.velocity[1+index_count] = +robotiq_state_msg_.velocity[0];

    robot_state_msg_.position[2+index_count] = -robotiq_state_msg_.position[0];
    robot_state_msg_.velocity[2+index_count] = -robotiq_state_msg_.velocity[0];

    robot_state_msg_.position[3+index_count] = -robotiq_state_msg_.position[0];
    robot_state_msg_.velocity[3+index_count] = -robotiq_state_msg_.velocity[0];

    robot_state_msg_.position[4+index_count] = +robotiq_state_msg_.position[0];
    robot_state_msg_.velocity[4+index_count] = +robotiq_state_msg_.velocity[0];

    robot_state_msg_.position[5+index_count] = -robotiq_state_msg_.position[0];
    robot_state_msg_.velocity[5+index_count] = -robotiq_state_msg_.velocity[0];                              
       
    index_count += 6;
  }

  joint_state_pub_.publish(robot_state_msg_);
}

void pHRIFrankaHuskyController::pubFextGlobal(){  
  geometry_msgs::Wrench Fext_global_msg;  
  Fext_global_msg.force.x = f_filtered_(0);
  Fext_global_msg.force.y = f_filtered_(1);
  Fext_global_msg.force.z = f_filtered_(2);
  Fext_global_msg.torque.x =f_filtered_(3);
  Fext_global_msg.torque.y =f_filtered_(4);
  Fext_global_msg.torque.z =f_filtered_(5);

  Fext_global_pub_.publish(Fext_global_msg);  
}

void pHRIFrankaHuskyController::setFrankaCommand(){  
  robot_command_msg_.MODE = 1;
  robot_command_msg_.header.stamp = ros::Time::now();
  robot_command_msg_.time = time_;

  for (int i=0; i<7; i++)
      if (ismobile_) robot_command_msg_.torque[i+2] = franka_torque_(i);   
      else           robot_command_msg_.torque[i] = franka_torque_(i);   
}

void pHRIFrankaHuskyController::setHuskyCommand(){
  for (int i=0; i<2; i++)
      robot_command_msg_.torque[i] = husky_qacc_(i);   
}

void pHRIFrankaHuskyController::pubEEState(){
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

void pHRIFrankaHuskyController::pubBaseState(){
    Vector3d pos;
    ctrl_->base_state(pos);

    for (int i=0; i<3; i++)
        base_state_msg_.position[i] = pos(i);

    base_state_pub_.publish(base_state_msg_);
}

VectorXd pHRIFrankaHuskyController::FT_local_to_global(VectorXd & f_local){  
  pinocchio::Force f_local_pin;  
  VectorXd f_global;

  f_local_pin.linear() = f_local.head(3);
  f_local_pin.angular() = f_local.tail(3);      
  f_global = oMi_.act(f_local_pin).toVector(); 

  return f_global;
}

Vector6d pHRIFrankaHuskyController::FT_global_to_local(Vector6d & f_global){  
  pinocchio::Force f_global_pin;  
  Vector6d f_local;

  f_global_pin.linear() = f_global.head(3);
  f_global_pin.angular() = f_global.tail(3);      
  f_local = oMi_.actInv(f_global_pin).toVector(); 

  return f_local;
}

void pHRIFrankaHuskyController::teleopjoyCallback(const sensor_msgs::JoyConstPtr &joy_msg) {    
  //necessary button : gravity, home, gripper, estimation, parameter apply, impedance
  int msg = 0;
  if(joy_msg->buttons[5]){ //R1 button pushed 
    
    //gravity
    if (joy_msg->buttons[13]) { //center     button pushed
      if (!isbutton_pushed_(13)) {
        msg = 0;          
        ctrl_->ctrl_update(msg);          

        cout << " " << endl;
        cout << "Gravity mode" << endl;
        cout << " " << endl;
        if(ismobile_) husky_qvel_prev_.setZero();

        isbutton_pushed_(13) = 1;     
     }
    }
    else isbutton_pushed_(13) = 0;

    //home
    if (joy_msg->buttons[1])  { //cross      button pushed
      if (!isbutton_pushed_(1)) {
        msg = 1;
        ctrl_->ctrl_update(msg);
        cout << " " << endl;
        cout << "home position" << endl;
        cout << " " << endl;
        if(ismobile_) husky_qvel_prev_.setZero();                

        //Fext
        mob_type_ = 0;
        cout << "end applying Fext" << endl;     

        isbutton_pushed_(1) = 1;
      }
    }
    else isbutton_pushed_(1) = 0;

    //estimation
    if (joy_msg->buttons[2])  { //circle     button pushed      
      if (!isbutton_pushed_(2)) {
        cout << "start estimation" << endl;
        isstartestimation_ = true; 
        tau_bias_init_flag = true;
        F_ext_bias_init_flag = true;                        
        
        fin_.open("/home/kimm/kimm_catkin/src/kimm_phri_panda_husky/calibration/calibration.txt");
        est_time_ = time_;

        param_estimated_.setZero();
        ekf->init(time_, param_estimated_);  

        msg = 12;
        ctrl_->ctrl_update(msg);
        cout << " " << endl;
        cout << "null motion ee in -x axis for estimation" << endl;
        cout << " " << endl;
        if(ismobile_) husky_qvel_prev_.setZero();  
        
        isbutton_pushed_(2) = 1;     
      }    
    }
    else isbutton_pushed_(2) = 0;

    //set estimated parameter
    if (joy_msg->buttons[0])  { //square     button pushed
      if (!isbutton_pushed_(0)) {
        if (isobjectdynamics_){
          cout << "eliminate object dynamics" << endl;
          isobjectdynamics_ = false;                    
        }
        else{
          cout << "apply object dynamics" << endl;
          cout << param_used_.transpose() << endl;
          isobjectdynamics_ = true; 
        }

        isbutton_pushed_(0) = 1;             
      }
    }
    else isbutton_pushed_(0) = 0;

    //impedance control
    if (joy_msg->buttons[3])  { //triangle button pushed
      if (!isbutton_pushed_(3)) {
        msg = 22;
        ctrl_->ctrl_update(msg);
        cout << " " << endl;
        cout << "impedance control" << endl;
        cout << " " << endl;
        if(ismobile_) husky_qvel_prev_.setZero(); 

        //Fext
        mob_type_ = 1;
        cout << "start applying Fext" << endl;     

        isbutton_pushed_(3) = 1;     
      }              
    }  
    else isbutton_pushed_(3) = 0;        
  }

  if(joy_msg->buttons[7]){ //R2 button pushed 
    //estimation in static
    if (joy_msg->buttons[2])  { //circle button pushed      
      if (!isbutton_pushed_(2)) {
        cout << "start estimation" << endl;
        isstartestimation_ = true; 
        tau_bias_init_flag = true;
        F_ext_bias_init_flag = true;                        
        
        fin_.open("/home/kimm/kimm_catkin/src/kimm_phri_panda_husky/calibration/calibration.txt");
        est_time_ = time_;

        param_estimated_.setZero();
        ekf->init(time_, param_estimated_);  

        msg = 11;
        ctrl_->ctrl_update(msg);
        cout << " " << endl;
        cout << "null motion ee in -x axis for estimation" << endl;
        cout << " " << endl;
        if(ismobile_) husky_qvel_prev_.setZero();  
        
        isbutton_pushed_(2) = 1;     
      }    
    }
    else isbutton_pushed_(2) = 0;

    //base backward
    if (joy_msg->buttons[3])  { //triangle button pushed
      if (!isbutton_pushed_(3)) {
        msg = 42;
        ctrl_->ctrl_update(msg);
        cout << " " << endl;
        cout << "move mobile -0.1x with keeping posture" << endl;
        cout << " " << endl;
        if(ismobile_) husky_qvel_prev_.setZero();

         isbutton_pushed_(3) = 1;     
      }
    } 
    else isbutton_pushed_(3) = 0;          
    
    //gripper
    if (joy_msg->buttons[1])  { //cross button pushed
      if (!isbutton_pushed_(1)) {
        if (load_gripper_) { //franka gripper
          if (isgrasp_){
              cout << "Release hand" << endl;
              isgrasp_=false;
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
        }
        else if (isrobotiq_) { //robotiq gripper
          if (isgrasp_){
              cout << "Release hand" << endl;
              isgrasp_ = false;
              gripper_robotiq_->open();
          }
          else{
              cout << "Grasp object" << endl;
              isgrasp_ = true; 
              gripper_robotiq_->close(0.5, 250, false);                
          }
        }
        else {        
        }

        isbutton_pushed_(1) = 1;     
      }
    } 
    else isbutton_pushed_(1) = 0;      
  }
}

void pHRIFrankaHuskyController::modeChangeReaderProc(){
  // ROS_INFO("I am here2");

  while (!quit_all_proc_)
  {
    char key = getchar();
    key = tolower(key);
    calculation_mutex_.lock();

    int msg = 0;
    switch (key){
      case 'g': //gravity mode
          msg = 0;          
          ctrl_->ctrl_update(msg);          

          cout << " " << endl;
          cout << "Gravity mode" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;
      case 'h': //home
          if(fin_.is_open()) fin_.close();

          msg = 1;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "home position" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;     
      case 'w': //null motion ee
          msg = 11;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "null motion ee in -x axis" << endl;
          cout << " " << endl;    
          if(ismobile_) husky_qvel_prev_.setZero();      
          break;  
      case '1': //null motion ee
          msg = 101;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "null motion ee in -x axis" << endl;
          cout << " " << endl;    
          if(ismobile_) husky_qvel_prev_.setZero();      
          break;                 
      case 'e': //null motion ee
          msg = 12;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "null motion ee in -x axis" << endl;
          cout << " " << endl;    
          if(ismobile_) husky_qvel_prev_.setZero();      
          break;    
      case 'r': //test move ee -0.2x & -0.2z
          msg = 13;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee -0.2x & -0.2z" << endl;
          cout << " " << endl;    
          if(ismobile_) husky_qvel_prev_.setZero();      
          break;    
      case 'c': //admittance control
          msg = 21;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "admittance control" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;                          
      case 'v': //impedance control
          msg = 22;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "impedance control" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;               
      case 'm': //impedance control with nonzero K
          msg = 23;          
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "impedance control with nonzero K" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;          
      case 'i': //move ee +0.1z
          msg = 31;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee +0.1 z" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;         
      case 'k': //move ee -0.1z
          msg = 32;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee -0.1 z" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;                                             
      case 'l': //move ee +0.1x
          msg = 33;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee +0.1 x" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;                                   
      case 'j': //move ee -0.1x
          msg = 34;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee -0.1 x" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;  
      case 'u': //move ee +0.1y
          msg = 35;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee +0.1 y" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;                                   
      case 'o': //move ee -0.1y
          msg = 36;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move ee -0.1 y" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;              
      case 'a': //move mobile +0.1x with keeping posture
          msg = 41;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move mobile +0.1x with keeping posture" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break; 
      case 's': //move mobile -0.1x with keeping posture
          msg = 42;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move mobile -0.1x with keeping posture" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;
      case 'd': //move mobile +0.1x with keeping ee
          msg = 43;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move mobile +0.1x with keeping ee" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break; 
      case 'f': //move mobile -0.1x with keeping ee
          msg = 44;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "move mobile -0.1x with keeping ee" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;                
      case 't': //base backward
          msg = 45;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "base backward" << endl;
          cout << " " << endl;
          if(ismobile_) husky_qvel_prev_.setZero();
          break;           
      
      case 'q': //object estimation
          if (isstartestimation_){
              // cout << "end estimation" << endl;
              // isstartestimation_ = false;

              // param_used_ = param_estimated_;
              // cout << "estimated parameter" << endl;
              // cout << param_used_.transpose() << endl;
              
              // param_estimated_.setZero();
              // ekf->init(time_, param_estimated_);  

              cout << "estimation procedure is not finished" << endl;  
          }
          else{
              cout << "start estimation" << endl;
              isstartestimation_ = true; 
              tau_bias_init_flag = true;
              F_ext_bias_init_flag = true;              
              
              fin_.open("/home/kimm/kimm_catkin/src/kimm_phri_panda_husky/calibration/calibration.txt");
              est_time_ = time_;

              param_estimated_.setZero();
              ekf->init(time_, param_estimated_);  

              msg = 11;
              ctrl_->ctrl_update(msg);
              cout << " " << endl;
              cout << "null motion ee in -x axis for estimation" << endl;
              cout << " " << endl;
              if(ismobile_) husky_qvel_prev_.setZero();
          }
          break;    
      case 'x': //object dynamics
          if (isobjectdynamics_){
              cout << "eliminate object dynamics" << endl;
              isobjectdynamics_ = false;                    
          }
          else{
              cout << "apply object dynamics" << endl;
              cout << param_used_.transpose() << endl;
              isobjectdynamics_ = true; 
          }
          break;             
      case 'b': //apply Fext
          if (mob_type_){ //mob_type_ = 1 or 2
              mob_type_ = 0;
              cout << "end applying Fext" << endl;                            
          }
          else{ //mob_type_ = 0
              mob_type_ = 1;
              cout << "start applying Fext" << endl;                            
          }
          break;  
      case 'p': //print current EE state
          msg = 99;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "print current EE state" << endl;
          cout << " " << endl;          

          ROS_INFO_STREAM("robot_g_local" << robot_g_local_(0) << robot_g_local_(1) << robot_g_local_(2));    

          if(ismobile_) husky_qvel_prev_.setZero();
          break;      
      case 'z': //grasp
          msg = 899;
          if (load_gripper_) { //franka gripper
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
          }
          else if (isrobotiq_) { //robotiq gripper
            if (isgrasp_){
                cout << "Release hand" << endl;
                isgrasp_ = false;
                gripper_robotiq_->open(false);
            }
            else{
                cout << "Grasp object" << endl;
                isgrasp_ = true; 
                gripper_robotiq_->close(0.5, 220, false);                
            }
          }
          else{            
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
  // ROS_INFO("I am here3");
}

} // namespace kimm_franka_husky_controllers

PLUGINLIB_EXPORT_CLASS(kimm_franka_husky_controllers::pHRIFrankaHuskyController, controller_interface::ControllerBase)
