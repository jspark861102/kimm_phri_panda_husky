#include "kimm_phri_panda_husky/phri_simul.h"

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace RobotController;

int main(int argc, char **argv)
{   
    //Ros setting
    ros::init(argc, argv, "kimm_phri_panda_husky");
    ros::NodeHandle n_node;
    
    dt = 0.001;
    time_ = 0.0;
    ros::Rate loop_rate(1.0/dt);

    /////////////// Robot Wrapper ///////////////
    n_node.getParam("/robot_group", group_name);    
    n_node.getParam("/mobile", ismobile_);      
    n_node.getParam("/issimulation", issimulation_);   
    cout << "ismobile   " << ismobile_ << endl;
    cout << "issimulation   " << issimulation_ << endl;
    ctrl_ = new RobotController::FrankaHuskyWrapper(group_name, issimulation_, ismobile_, n_node);
    ctrl_->initialize();
    ctrl_->get_dt(dt);
    traj_length_in_time_ = ctrl_->trajectory_length_in_time();
    
    /////////////// mujoco sub : from mujoco to here ///////////////    
    ros::Subscriber jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));    
    ros::Subscriber mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber ctrl_type_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/ctrl_type", 1, &ctrltypeCallback, ros::TransportHints().tcpNoDelay(true));

    /////////////// mujoco pub : from here to mujoco ///////////////    
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_run_pub_ = n_node.advertise<std_msgs::Bool>("mujoco_ros/mujoco_ros_interface/sim_run", 5);

    joint_states_pub_ = n_node.advertise<sensor_msgs::JointState>("joint_states", 5);    
    object_parameter_pub_ = n_node.advertise<kimm_phri_msgs::ObjectParameter>("object_parameter", 5);
    wrench_mesured_pub_ = n_node.advertise<geometry_msgs::Wrench>("wrench_measured", 5);
    vel_pub_ = n_node.advertise<geometry_msgs::Twist>("object_velocity", 5);
    accel_pub_ = n_node.advertise<geometry_msgs::Twist>("object_acceleration", 5);

    /////////////// robot - ctrl(phri_hqp), robot(robot_wrapper) ///////////////        
    ee_state_pub_ = n_node.advertise<geometry_msgs::Transform>("mujoco_ros/mujoco_ros_interface/ee_state", 5);
    if (ismobile_) base_state_pub_ = n_node.advertise<sensor_msgs::JointState>("mujoco_ros/mujoco_ros_interface/base_state", 5);

    if (ismobile_) br_ = new tf::TransformBroadcaster();    
    
    // msg 
    ee_state_msg_ = geometry_msgs::Transform();  // obtained from "robot_->position"
    if (ismobile_) {
        robot_command_msg_.torque.resize(13);        // wheels(4) + robot (7) + gripper(2) --> from here to mujoco
        base_state_msg_.position.resize(3);          // robot_->getMobilePosition 
    }
    else {
        robot_command_msg_.torque.resize(9);         // robot (7) + gripper(2) --> from here to mujoco    
    }        

    sim_run_msg_.data = true;
    isgrasp_ = false;        

    // InitMob();

    // ************ object estimation *************** //               
    isstartestimation_ = false;
    getObjParam_init();
    isFextapplication_ = false;
    isobjectdynamics_ = false;
    isFextcalibration_ = false;
    est_time_ = 0.0;
    // ********************************************** //    

    while (ros::ok()){        
        //mujoco sim run 
        mujoco_run_pub_.publish(sim_run_msg_);
       
        //keyboard
        keyboard_event();

        // ctrl computation
        ctrl_->compute(time_); //make control input for 1kHz, joint state will be updated 1kHz from the mujoco
        
        // get output
        ctrl_->mass(robot_mass_);
        ctrl_->nle(robot_nle_);
        ctrl_->g(robot_g_);  // dim model.nv, [Nm]        
        ctrl_->state(state_);           
        ctrl_->JWorld(robot_J_world_);     //world
        
        // ctrl_->g_joint7(robot_g_local_);  //g [m/s^2] w.r.t joint7 axis
        // ctrl_->JLocal(robot_J_local_);     //local
        // ctrl_->dJLocal(robot_dJ_local_);   //local
        ctrl_->g_local_offset(robot_g_local_);    //local
        ctrl_->JLocal_offset(robot_J_local_);     //local
        ctrl_->dJLocal_offset(robot_dJ_local_);   //local

        // get control input from hqp controller
        ctrl_->franka_output(franka_qacc_); //get control input                
        franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;                 

        // apply object dynamics
        param_true_ << 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        if (isobjectdynamics_) {
            FT_object_ = objdyn.h(param_true_, vel_param.toVector(), acc_param.toVector(), robot_g_local_); //Vector3d(0,0,9.81)
            franka_torque_ -= robot_J_local_.transpose() * FT_object_;                                    
        }           

        // apply Fext for compliance control
        if (isFextapplication_) {
            if (!isFextcalibration_) {
                Fext_cali_ << robot_J_local_.transpose().col(0) * FT_measured(0);                
                Fext_cali_ += robot_J_local_.transpose().col(1) * FT_measured(1);                
                Fext_cali_ += robot_J_local_.transpose().col(2) * FT_measured(2);                
                isFextcalibration_ = true;
                cout << Fext_cali_.transpose() << endl;
            }
            else {
                franka_torque_ += 0.3*robot_J_local_.transpose().col(0) * FT_measured(0);
                franka_torque_ += 0.3*robot_J_local_.transpose().col(1) * FT_measured(1);
                franka_torque_ += 0.3*robot_J_local_.transpose().col(2) * FT_measured(2);
                franka_torque_ -= 0.3*Fext_cali_;
            }                                    
        }

        if (ismobile_) ctrl_->husky_output(husky_qacc_);   //get control input

        // get Mob
        // if (ctrl_->ctrltype() != 0)
        //     UpdateMob();
        // else
        //     InitMob();

        // set control input to mujoco
        setGripperCommand();                              //set gripper torque by trigger value
        setRobotCommand();                                //set franka and husky command 
        robot_command_pub_.publish(robot_command_msg_);   //pub total command
       
        // get state
        getEEState();                                     //obtained from "robot_->position", and publish for monitoring              
        if (ismobile_) getBaseState();                    //obtained from "robot_->getMobilePosition", and publish for monitoring

        // ************ object estimation *************** //               
        vel_accel_pub();        
        FT_measured_pub();        
        getObjParam();                                    // object estimation
        ObjectParameter_pub();                                       // data plot for monitoring
        // ********************************************** //    
        
        ros::spinOnce();
        loop_rate.sleep();        
    }//while

    return 0;
}

// ************************************************ object estimation start *************************************************** //                       
void vel_accel_pub(){
    geometry_msgs::Twist vel_msg, accel_msg;   

    //************* obtained from pinocchio ***********// 
    //**** velocity (data.v) is identical with mujoco velocity, 
    //**** but acceleration (data.a) is not reasnoble for both local and global cases.
    // ctrl_->velocity(vel_param);                //LOCAL
    // ctrl_->acceleration(acc_param);            //LOCAL
    
    // ctrl_->velocity_origin(vel_param);         //GLOBAL
    // ctrl_->acceleration_origin2(acc_param);     //GLOBAL                

    //************* obtained from mujoco : LOCAL **************//
    vel_param.linear()[0] = v_mujoco[0];
    vel_param.linear()[1] = v_mujoco[1];
    vel_param.linear()[2] = v_mujoco[2];
    vel_param.angular()[0] = v_mujoco[3];
    vel_param.angular()[1] = v_mujoco[4];
    vel_param.angular()[2] = v_mujoco[5];

    // acc_param.linear()[0] = a_mujoco[0];
    // acc_param.linear()[1] = a_mujoco[1];
    // acc_param.linear()[2] = a_mujoco[2];
    // acc_param.angular()[0] = a_mujoco[3];
    // acc_param.angular()[1] = a_mujoco[4];
    // acc_param.angular()[2] = a_mujoco[5];

    acc_param.linear()[0] = a_mujoco_filtered[0];
    acc_param.linear()[1] = a_mujoco_filtered[1];
    acc_param.linear()[2] = a_mujoco_filtered[2];
    acc_param.angular()[0] = a_mujoco_filtered[3];
    acc_param.angular()[1] = a_mujoco_filtered[4];
    acc_param.angular()[2] = a_mujoco_filtered[5];

    //************************ publish ************************//
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

void FT_measured_pub() {
    if (isstartestimation_) {
        if (time_ - est_time_ < traj_length_in_time_) {            
        }
        else {
            cout << "end estimation" << endl;
            isstartestimation_ = false;
            
            cout << "estimated parameter" << endl;
            cout << param.transpose() << endl;
            
            param.setZero();
            ekf->init(time_, param);
        }
    }


    //actually, franka_torque_ is not a measured but command torque, because measurment is not available
    tau_estimated = robot_mass_ * ddq_mujoco + robot_nle_;        
    // tau_ext = franka_torque_ - tau_estimated;      // coincide with g(0,0,-9.81)  
    tau_ext = -franka_torque_ + tau_estimated;        // coincide with g(0,0,9.81)
    
    Fext_global_ = robot_J_world_.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_ext;  //global
    ctrl_->Fext_update(Fext_global_);        
    
    FT_measured = robot_J_local_.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_ext;  //local    
    geometry_msgs::Wrench FT_measured_msg;  
    FT_measured_msg.force.x = saturation(FT_measured[0],50);
    FT_measured_msg.force.y = saturation(FT_measured[1],50);
    FT_measured_msg.force.z = saturation(FT_measured[2],50);
    FT_measured_msg.torque.x = saturation(FT_measured[3],10);
    FT_measured_msg.torque.y = saturation(FT_measured[4],10);
    FT_measured_msg.torque.z = saturation(FT_measured[5],10);
    wrench_mesured_pub_.publish(FT_measured_msg);    
}

void getObjParam(){
    // *********************************************************************** //
    // ************************** object estimation ************************** //
    // *********************************************************************** //
    // mujoco output     : JointStateCallback (motion) : 9 = 7(joint) + 2(gripper)  for pose & velocity & effort, same with rviz jointstate
    // pinocchio input   : state_.q_, v_      (motion) : 7 = 7(joint)               for q,v
    // controller output : state_.torque_     (torque) : 7 = 7(joint)               for torque, pinocchio doesn't control gripper
    // mujoco input      : robot_command_msg_ (torque) : 9 = 7(joint) + 2(gripper)  robot_command_msg_.torque.resize(9); 
    
    //*--- p cross g = (py*gz-pz*gy)i + (pz*gx-px*gz)j + (px*gy-py*gx)k ---*//

    //*--- FT_measured & vel_param & acc_param is LOCAL frame ---*//

    if (isstartestimation_) {

        h = objdyn.h(param, vel_param.toVector(), acc_param.toVector(), robot_g_local_); //Vector3d(0,0,9.81)
        H = objdyn.H(param, vel_param.toVector(), acc_param.toVector(), robot_g_local_); //Vector3d(0,0,9.81)

        ekf->update(FT_measured, dt, A, H, h);
        param = ekf->state();   

        if (fabs(fabs(robot_g_local_(0)) - 9.81) < 0.02) param[1] = 0.0; //if x axis is aligned with global gravity axis, corresponding param is not meaninful 
        if (fabs(fabs(robot_g_local_(1)) - 9.81) < 0.02) param[2] = 0.0; //if y axis is aligned with global gravity axis, corresponding param is not meaninful 
        if (fabs(fabs(robot_g_local_(2)) - 9.81) < 0.02) param[3] = 0.0; //if z axis is aligned with global gravity axis, corresponding param is not meaninful 
    }
}

void ObjectParameter_pub(){
    kimm_phri_msgs::ObjectParameter objparam_msg;  
    objparam_msg.com.resize(3);
    objparam_msg.inertia.resize(6);      

    objparam_msg.mass = saturation(param[0],5.0);
    
    //local coordinate--------------------//
    objparam_msg.com[0] = saturation(param[1],0.6); 
    objparam_msg.com[1] = saturation(param[2],0.6); 
    objparam_msg.com[2] = saturation(param[3],0.6);  
    //-------------------------------------//

    //global coordinate--------------------//
    // Eigen::Vector3d com_global;
    // SE3 oMi;
    
    // // ctrl_->position(oMi);
    // ctrl_->position_offset(oMi);
    // oMi.translation().setZero();
    // com_global = oMi.act(Vector3d(param[1], param[2], param[3])); //local to global  

    // objparam_msg.com[0] = saturation(com_global[0],0.6); 
    // objparam_msg.com[1] = saturation(com_global[1],0.6); 
    // objparam_msg.com[2] = saturation(com_global[2],0.6); 
    //-------------------------------------//


    object_parameter_pub_.publish(objparam_msg);              
}

void getObjParam_init(){
    n_param = 10;
    m_FT = 6;

    A.resize(n_param, n_param); // System dynamics matrix
    H.resize(m_FT,    n_param); // Output matrix
    Q.resize(n_param, n_param); // Process noise covariance
    R.resize(m_FT,    m_FT); // Measurement noise covariance
    P.resize(n_param, n_param); // Estimate error covariance
    h.resize(m_FT,    1); // observation      
    param.resize(n_param);    
    FT_measured.resize(m_FT);    
    ddq_mujoco.resize(7);
    tau_estimated.resize(7);
    tau_ext.resize(7);
    v_mujoco.resize(6);
    a_mujoco.resize(6);
    a_mujoco_filtered.resize(6);   
    FT_object_.resize(6); 
    param_true_.resize(n_param);
    Fext_cali_.resize(7);
    Fext_global_.resize(m_FT);

    A.setIdentity();         //knwon, identity
    Q.setIdentity();         //design parameter
    R.setIdentity();         //design parameter    
    P.setIdentity();         //updated parameter
    h.setZero();             //computed parameter
    H.setZero();             //computed parameter    
    param.setZero();   
    FT_measured.setZero();
    vel_param.setZero();
    acc_param.setZero();
    ddq_mujoco.setZero();
    tau_estimated.setZero();
    tau_ext.setZero();
    v_mujoco.setZero();
    a_mujoco.setZero();
    a_mujoco_filtered.setZero();
    FT_object_.setZero();
    param_true_.setZero();
    Fext_cali_.setZero();
    Fext_global_.setZero();

    Q(0,0) *= 0.01;
    Q(1,1) *= 0.0001;
    Q(2,2) *= 0.0001;
    Q(3,3) *= 0.0001;
    R *= 100000;
    
    // Construct the filter
    ekf = new EKF(dt, A, H, Q, R, P, h);
    
    // Initialize the filter  
    ekf->init(time_, param);
}  

double saturation(double x, double limit) {
    if (x > limit) return limit;
    else if (x < -limit) return -limit;
    else return x;
}
// ************************************************ object estimation end *************************************************** //                       


void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){ 
    // from mujoco
    // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
    // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper)    

    //update state to pinocchio
    // state_.q_      //12 odom (3 - x,y,theta) + husky (2) + franka (7)
    // state_.v_      //12
    // state_.dv_     //12
    // state_.torque_ //9  nv-3, odom dof eliminated <-- used as control command

    sensor_msgs::JointState msg_tmp;
    msg_tmp = *msg;    

    ctrl_->franka_update(msg_tmp);        
    joint_states_publish(msg_tmp);        

    if (ismobile_) {
        ctrl_->husky_update(msg_tmp);

        //tf pub of "husky_odom"
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg_tmp.position[0], msg_tmp.position[1], msg_tmp.position[2]) );
        tf::Quaternion q(msg_tmp.position[4], msg_tmp.position[5], msg_tmp.position[6], msg_tmp.position[3]);
        //tf::Quaternion q(0, 0, 0, 1);
        transform.setRotation(q);
        br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "husky_odom", group_name + "_rviz_base_link"));       

        v_mujoco.setZero();
        a_mujoco.setZero();    
        for (int i=0; i<7; i++){ 
            ddq_mujoco[i] = msg_tmp.effort[10+i];
            v_mujoco += robot_J_local_.col(i) * msg_tmp.velocity[10+i];                                           //jacobian is LOCAL
            a_mujoco += robot_dJ_local_.col(i) * msg_tmp.velocity[10+i] + robot_J_local_.col(i) * msg_tmp.effort[10+i]; //jacobian is LOCAL
        }      
    } 
    else {
        v_mujoco.setZero();
        a_mujoco.setZero();    
        for (int i=0; i<7; i++){ 
            ddq_mujoco[i] = msg_tmp.effort[i];
            v_mujoco += robot_J_local_.col(i) * msg_tmp.velocity[i];                                                 //jacobian is LOCAL
            a_mujoco += robot_dJ_local_.col(i) * msg_tmp.velocity[i] + robot_J_local_.col(i) * msg_tmp.effort[i];    //jacobian is LOCAL        
        }
    }

    // Filtering
    double cutoff = 20.0; // Hz //20
    double RC = 1.0 / (cutoff * 2.0 * M_PI);    
    double alpha = dt / (RC + dt);

    a_mujoco_filtered = alpha * a_mujoco + (1 - alpha) * a_mujoco_filtered;            
}

void joint_states_publish(const sensor_msgs::JointState& msg){
    // mujoco callback msg
    // msg.position : 7(odom) + 4(wheel) + 7(joint) + finger(2)
    // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + finger(2)

    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();    

    if (ismobile_){
        //revolute joint name in rviz urdf (husky_panda_rviz_ns0.urdf)
        joint_states.name = {"ns0_husky_front_left_wheel", "ns0_husky_front_right_wheel", "ns0_husky_rear_left_wheel", "ns0_husky_rear_right_wheel", "ns0_panda_joint1","ns0_panda_joint2","ns0_panda_joint3","ns0_panda_joint4","ns0_panda_joint5","ns0_panda_joint6","ns0_panda_joint7","ns0_panda_finger_joint1","ns0_panda_finger_joint2"};    

        joint_states.position.resize(13); //husky(4) + panda(7) + finger(2), rviz urdf doesn't have odom joint
        joint_states.velocity.resize(13); //husky(4) + panda(7) + finger(2), rviz urdf doesn't have odom joint

        for (int i=0; i<13; i++){ 
            joint_states.position[i] = msg.position[7+i];
            joint_states.velocity[i] = msg.velocity[6+i];
        }
        // joint_states.position[11] = 0.0;
        // joint_states.position[12] = 0.0;
    }
    else {
        //revolute joint name in rviz urdf (panda_arm_hand_l_rviz.urdf)
        joint_states.name = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"};    

        joint_states.position.resize(9); //panda(7) + finger(2)
        joint_states.velocity.resize(9); //panda(7) + finger(2)

        for (int i=0; i<9; i++){ 
            joint_states.position[i] = msg.position[i];
            joint_states.velocity[i] = msg.velocity[i];
        }
    }        

    joint_states_pub_.publish(joint_states);    
}

void ctrltypeCallback(const std_msgs::Int16ConstPtr &msg){
    ROS_WARN("%d", msg->data);
    
    if (msg->data != 899){
        int data = msg->data;
        ctrl_->ctrl_update(data);
    }
    else{
        if (isgrasp_)
            isgrasp_=false;
        else
            isgrasp_=true;
    }
}

void setRobotCommand(){
    robot_command_msg_.MODE = 1; //0:position control, 1:torque control
    robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;   
    
    if (ismobile_) {
        robot_command_msg_.torque[0] = husky_qacc_(0);
        robot_command_msg_.torque[2] = husky_qacc_(0);
        robot_command_msg_.torque[1] = husky_qacc_(1);
        robot_command_msg_.torque[3] = husky_qacc_(1);

        for (int i=0; i<7; i++)
            robot_command_msg_.torque[i+4] = franka_torque_(i);    
    }
    else {
        for (int i=0; i<7; i++)
            robot_command_msg_.torque[i] = franka_torque_(i);    
    }
    
}

void setGripperCommand(){
    int left_finger, right_finger;
    if (ismobile_) {
        left_finger = 11;
        right_finger = 12;        
    }
    else {
        left_finger = 7;
        right_finger = 8;        
    }
    
    if (isgrasp_){
        robot_command_msg_.torque[left_finger] = -200.0;
        robot_command_msg_.torque[right_finger] = -200.0;
    }
    else{
        robot_command_msg_.torque[left_finger] = 100.0;
        robot_command_msg_.torque[right_finger] = 100.0;
    }
}

void getEEState(){
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

void getBaseState(){
    Vector3d pos;
    ctrl_->base_state(pos);

    for (int i=0; i<3; i++)
        base_state_msg_.position[i] = pos(i);

    base_state_pub_.publish(base_state_msg_);
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'g': //gravity mode
                msg = 0;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Gravity mode" << endl;
                cout << " " << endl;
                break;
            case 'h': //home
                msg = 1;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "home position" << endl;
                cout << " " << endl;
                break;
            case 'a': //home and axis align btw base and joint 7
                msg = 2;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "home and axis align btw base and joint 7" << endl;
                cout << " " << endl;
                break;                
            case 'y': //move mobile -0.1x with keeping ee
                msg = 4;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move mobile -0.1x with keeping ee" << endl;
                cout << " " << endl;
                break;                
            case 'w': //rotate ee in -y axis
                msg = 11;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate ee in -y aixs" << endl;
                cout << " " << endl;
                break;  
            case 'r': //rotate ee in -x axis
                msg = 12;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate ee in -x axis" << endl;
                cout << " " << endl;
                break;
            case 't': //sine motion ee in -x axis
                msg = 13;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "sine motion ee in -x axis" << endl;
                cout << " " << endl;
                break;               
            case 'e': //null motion ee
                msg = 14;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "null motion ee in -x axis" << endl;
                cout << " " << endl;
                break;    
            case 'c': //admittance control
                msg = 21;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "admittance control" << endl;
                cout << " " << endl;
                break;                          
            case 'v': //impedance control
                msg = 22;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "impedance control" << endl;
                cout << " " << endl;
                break;               
            case 'm': //impedance control with nonzero K
                msg = 23;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "impedance control with nonzero K" << endl;
                cout << " " << endl;
                break;          
            case 'i': //move ee +0.1z
                msg = 31;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move ee +0.1 z" << endl;
                cout << " " << endl;
                break;         
            case 'k': //move ee -0.1z
                msg = 32;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move ee -0.1 z" << endl;
                cout << " " << endl;
                break;                                   
                break;         
            case 'l': //move ee +0.1x
                msg = 33;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move ee +0.1 x" << endl;
                cout << " " << endl;
                break;                                   
            case 'j': //move ee -0.1x
                msg = 34;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move ee -0.1 x" << endl;
                cout << " " << endl;
                break;   
            case 'q': //object estimation
                if (isstartestimation_){
                    // cout << "end estimation" << endl;
                    // isstartestimation_ = false;
                    // param.setZero();
                    // ekf->init(time_, param);
                }
                else{
                    cout << "start estimation" << endl;
                    isstartestimation_ = true; 

                    est_time_ = time_;

                    msg = 14;
                    ctrl_->ctrl_update(msg);
                    cout << " " << endl;
                    cout << "null motion ee in -x axis for estimation" << endl;
                    cout << " " << endl;
                }
                break;       
            case 'x': //object dynamics
                if (isobjectdynamics_){
                    cout << "eliminate object dynamics" << endl;
                    isobjectdynamics_ = false;                    
                }
                else{
                    cout << "apply object dynamics" << endl;
                    isobjectdynamics_ = true; 
                }
                break;   
            case 'b': //apply Fext
                if (isFextapplication_){
                    cout << "end applying Fext" << endl;
                    isFextapplication_ = false;                    
                    isFextcalibration_ = false;
                }
                else{
                    cout << "start applying Fext" << endl;
                    isFextapplication_ = true; 
                }
                break;                       
            case 'p': //print current EE state
                msg = 99;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "print current EE state" << endl;
                cout << " " << endl;
                break;           
            case 'z': //grasp
                if (isgrasp_){
                    cout << "Release hand" << endl;
                    isgrasp_ = false;
                }
                else{
                    cout << "Grasp object" << endl;
                    isgrasp_ = true; 
                }
                break;
        }
    }
}

