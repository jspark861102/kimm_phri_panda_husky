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
    ctrl_ = new RobotController::HuskyFrankaWrapper(group_name, true, n_node);
    ctrl_->initialize();
    
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
    base_state_pub_ = n_node.advertise<sensor_msgs::JointState>("mujoco_ros/mujoco_ros_interface/base_state", 5);
    ee_state_pub_ = n_node.advertise<geometry_msgs::Transform>("mujoco_ros/mujoco_ros_interface/ee_state", 5);
    
    br_ = new tf::TransformBroadcaster();    

    // msg 
    robot_command_msg_.torque.resize(13);        // wheels(4) + robot (7) + gripper(2) --> from here to mujoco
    base_state_msg_.position.resize(3);          // robot_->getMobilePosition 
    ee_state_msg_ = geometry_msgs::Transform();  // obtained from "robot_->position"

    sim_run_msg_.data = true;
    isgrasp_ = false;
    isstartestimation = false;

    // InitMob();

    // ************ object estimation *************** //               
    getObjParam_init();
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
        ctrl_->g(robot_g_);  
        ctrl_->g_joint7(robot_g_local_);         
        ctrl_->state(state_);           

        // ctrl_->JWorld(robot_J_);         //world
        ctrl_->JLocal_offset(robot_J_);     //offset applied ,local
        ctrl_->dJLocal_offset(robot_dJ_);   //offset applied ,local

        // get control input from hqp controller
        ctrl_->franka_output(franka_qacc_); //get control input
        ctrl_->husky_output(husky_qacc_);   //get control input
        franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;
        
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
        getBaseState();                                   //obtained from "robot_->getMobilePosition", and publish for monitoring
        getEEState();                                     //obtained from "robot_->position", and publish for monitoring              

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
    //**** but acceleration is not reasnoble for both local and global cases.
    // // ctrl_->velocity_global(vel_param);       //offset applied 
    // // ctrl_->acceleration_global(acc_param);   //offset applied 
    // ctrl_->velocity(vel_param);       //offset applied 
    // ctrl_->acceleration(acc_param);   //offset applied 

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
    //actually, franka_torque_ is not a measured but command torque, because measurment is not available
    tau_estimated = robot_mass_ * ddq_mujoco + robot_nle_;        
    tau_ext = franka_torque_ - tau_estimated;        
    // tau_ext = -franka_torque_ + tau_estimated;        
    FT_measured = robot_J_.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_ext;  //robot_J_ is local jacobian      

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
    // mujoco output     : JointStateCallback (motion) : 20 = 7(odom)   + 4(wheel) + 7(joint) + 2(gripper)  for pose / 6(odom) + 4(wheel) + 7(joint) + 2(gripper) velocity & effort, same with rviz jointstate
    // pinocchio input   : state_.q_, v_, dv_ (motion) : 12 = 3(odom)   + 2(wheel) + 7(joint)               for q,v,dv
    // controller output : state_.torque_     (torque) :  9 =           + 2(wheel) + 7(joint)               for torque, pinocchio doesn't control gripper
    // mujoco input      : robot_command_msg_ (torque) : 13 =             4(wheel) + 7(joint) + 2(gripper)  robot_command_msg_.torque.resize(13); 
    
    //*--- p cross g = (py*gz-pz*gy)i + (pz*gx-px*gz)j + (px*gy-py*gx)k ---*//

    //*--- FT_measured & vel_param & acc_param is LOCAL frame ---*//

    if (isstartestimation) {

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

    Eigen::Vector3d com_global;
    SE3 oMi;
    ctrl_->position(oMi);
    oMi.translation().setZero();
    com_global = oMi.act(Vector3d(param[1], param[2], param[3])); //local to global         
    
    // com_global[2] += (0.1654-0.035); //from l_husky_with_panda_hand.xml, (0.1654:l_panda_rightfinger pos, 0.035: l_panda_rightfinger's cls pos)    

    objparam_msg.mass = saturation(param[0],5.0);
    // objparam_msg.com[0] = saturation(param[1]*sin(M_PI/4)+param[2]*sin(M_PI/4),0.3); // need to check for transformation
    // objparam_msg.com[1] = saturation(param[1]*cos(M_PI/4)-param[2]*cos(M_PI/4),0.3); // need to check for transformation
    // objparam_msg.com[2] = saturation(param[3],0.3);                                  // need to check for transformation
    objparam_msg.com[0] = saturation(com_global[0],0.6); // need to check for transformation
    objparam_msg.com[1] = saturation(com_global[1],0.6); // need to check for transformation
    objparam_msg.com[2] = saturation(com_global[2],0.6); // need to check for transformation

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
    sensor_msgs::JointState msg_tmp;
    msg_tmp = *msg;
    
    //update state to pinocchio
    // state_.q_      //12 odom (3 - x,y,theta) + husky (2) + franka (7)
    // state_.v_      //12
    // state_.dv_     //12
    // state_.torque_ //9  nv-3, odom dof eliminated <-- used as control command

    ctrl_->franka_update(msg_tmp);
    ctrl_->husky_update(msg_tmp);

    //tf pub of "husky_odom"
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg_tmp.position[0], msg_tmp.position[1], msg_tmp.position[2]) );
    tf::Quaternion q(msg_tmp.position[4], msg_tmp.position[5], msg_tmp.position[6], msg_tmp.position[3]);
    //tf::Quaternion q(0, 0, 0, 1);
    transform.setRotation(q);
    br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "husky_odom", group_name + "_rviz_base_link"));        

    joint_states_publish(msg_tmp);        

    v_mujoco.setZero();
    a_mujoco.setZero();
    for (int i=0; i<7; i++){ 
        ddq_mujoco[i] = msg_tmp.effort[10+i];
        v_mujoco += robot_J_.col(i) * msg_tmp.velocity[10+i];                                           //jacobian is LOCAL
        a_mujoco += robot_dJ_.col(i) * msg_tmp.velocity[10+i] + robot_J_.col(i) * msg_tmp.effort[10+i]; //jacobian is LOCAL
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
    robot_command_msg_.MODE = 1;
    robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    
    robot_command_msg_.torque[0] = husky_qacc_(0);
    robot_command_msg_.torque[2] = husky_qacc_(0);
    robot_command_msg_.torque[1] = husky_qacc_(1);
    robot_command_msg_.torque[3] = husky_qacc_(1);

    for (int i=0; i<7; i++)
        robot_command_msg_.torque[i+4] = franka_torque_(i);    
}

void setGripperCommand(){
    if (isgrasp_){
        robot_command_msg_.torque[11] = -200.0;
        robot_command_msg_.torque[12] = -200.0;
    }
    else{
        robot_command_msg_.torque[11] = 100.0;
        robot_command_msg_.torque[12] = 100.0;
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
            case 'h': //init position
                msg = 1;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "init position" << endl;
                cout << " " << endl;
                break;
            case 'a': //approach to object
                msg = 2;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "approach to object" << endl;
                cout << " " << endl;
                break;    
            case 's': //move ee -0.1z 
                msg = 3;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move ee -0.1z " << endl;
                cout << " " << endl;
                break;    
            case 'd': //move mobile +0.1x with keeping ee
                msg = 4;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move mobile +0.1x with keeping ee" << endl;
                cout << " " << endl;
                break;  
            case 'f': //move mobile -0.1m without arm motion
                msg = 5;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move mobile -0.1m without arm motion" << endl;
                cout << " " << endl;
                break;   
            case 'j': //align base and joint7 coordinate
                msg = 6;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "align base and joint7 coordinate" << endl;
                cout << " " << endl;
                break;         
            case 'k': //rotate joint1 by 90deg
                msg = 7;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate joint1 by 90deg" << endl;
                cout << " " << endl;
                break;  
            case 'q': //rotate ee for object estimation
                msg = 10;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate ee for object estimation" << endl;
                cout << " " << endl;
                break;             
            case 'w': //rotate ee with sine motion for object estimation
                msg = 11;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate ee with sine motion for object estimation" << endl;
                cout << " " << endl;
                break;    
            case 'e': //rotate joint7 for object estimation
                msg = 12;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate joint7 for object estimation" << endl;
                cout << " " << endl;
                break;  
            case 'r': //move -0.1x with joint1 motion (not for estimation)
                msg = 13;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "move -0.1x with joint1 motion" << endl;
                cout << " " << endl;
                break;  
            case 't': //rotate joint6 for object estimation
                msg = 14;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "rotate joint6 for object estimation" << endl;
                cout << " " << endl;
                break;       
            case 'o': //rotate joint6 for object estimation
                if (isstartestimation){
                    cout << "end estimation" << endl;
                    isstartestimation = false;
                    param.setZero();
                    ekf->init(time_, param);
                }
                else{
                    cout << "start estimation" << endl;
                    isstartestimation = true; 
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

