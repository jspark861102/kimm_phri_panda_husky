#include "kimm_phri_panda_husky/panda_husky_hqp.h"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace kimmhqp;
using namespace kimmhqp::trajectory;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace kimmhqp::robot;
using namespace kimmhqp::contacts;

namespace RobotController{
    HuskyFrankaWrapper::HuskyFrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), n_node_(node)
    {
        time_ = 0.;
        mode_change_ = false;
        ctrl_mode_ = 0;
        node_index_ = 0;
        cnt_ = 0;
    }

    void HuskyFrankaWrapper::initialize(){
        // Robot
        string model_path, urdf_name;
        n_node_.getParam("/" + robot_node_ +"/robot_urdf_path", model_path);
        n_node_.getParam("/" + robot_node_ +"/robot_urdf", urdf_name);          //"husky_panda_hand_free.urdf"

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;
        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false); //false : w/o mobile, true : w/ mobile
        model_ = robot_->model();

        //nq_/nv_/na_ is # of joint w.r.t pinocchio model ("husky_panda_hand_free.urdf"), so there is no gripper joints
        nq_ = robot_->nq(); //12 : odom (3 - x,y,theta) + husky (2) + franka (7)
        nv_ = robot_->nv(); //12
        na_ = robot_->na(); //9  : nv-3, odom dof eliminated

        // State
        state_.q_.setZero(nq_);
        state_.v_.setZero(nv_);
        state_.dv_.setZero(nv_);
        state_.torque_.setZero(na_);

        // tsid
        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q_, state_.v_);
        data_ = tsid_->data();

        // tasks
        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        VectorXd posture_gain(na_ -2);
        if (!issimulation_)
        	posture_gain << 200., 200., 200., 200., 200., 200., 200.;
        else
        	// posture_gain << 4000., 4000., 4000., 4000., 4000., 4000., 4000.;
            posture_gain << 40000., 40000., 40000., 40000., 40000., 40000., 40000.;


        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());

        Vector3d ee_offset(0.0, 0, 0.0); //0.165
        ee_offset_ << ee_offset;

        Adj_mat.resize(6,6);
        Adj_mat.setIdentity();
        Adj_mat(0, 4) = -ee_offset_(2);
        Adj_mat(0, 5) = ee_offset_(1);
        Adj_mat(1, 3) = ee_offset_(2);
        Adj_mat(1, 5) = -ee_offset_(0);
        Adj_mat(2, 3) = -ee_offset_(1);
        Adj_mat(2, 4) = ee_offset_(0);

        VectorXd ee_gain(6);
        ee_gain << 100., 100., 100., 400., 400., 400.;
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset);
        eeTask_->Kp(ee_gain*Vector::Ones(6));
        eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());

        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        Vector dq_max = 500000.0*Vector::Ones(na_);
        dq_max(0) = 500.;
        dq_max(1) = 500.;
        Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);

        mobileTask_ = std::make_shared<TaskMobileEquality>("task-mobile", *robot_, true);
        mobileTask_->Kp(50.0*Vector3d::Ones());
        mobileTask_->Kd(2.5*mobileTask_->Kp().cwiseSqrt());

        mobileTask2_ = std::make_shared<TaskMobileEquality>("task-mobile2", *robot_, false);
        mobileTask2_->Kp(50.0*Vector3d::Ones());
        mobileTask2_->Kd(2.5*mobileTask2_->Kp().cwiseSqrt());

        // trajecotries
        sampleEE_.resize(12, 6); //12=3(translation)+9(rotation matrix), 6=3(translation)+3(rotation)
        samplePosture_.resize(na_- 2); //na_=9 (husky 2 + franka 7)

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");

        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);

        Vector3d Maxvel_base = Vector3d::Ones()*1.0;
        Vector3d Maxacc_base = Vector3d::Ones()*1.0;
        trajMobile_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_mobile");
        trajMobile_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_mobile_constant");
        trajMobile_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_mobile_timeopt", Maxvel_base, Maxacc_base);

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "qpoases");

        // service
        joint_action_subs_ = n_node_.subscribe("/" + robot_node_ + "_gui/kimm_joint_planner_ros_interface_server/joint_action", 1, &RobotController::HuskyFrankaWrapper::jointActionCallback, this);
        se3_action_subs_ = n_node_.subscribe("/" + robot_node_ + "_gui/kimm_se3_planner_ros_interface_server/se3_action", 1, &RobotController::HuskyFrankaWrapper::se3ActionCallback, this);
        mobile_action_subs_ = n_node_.subscribe("/" + robot_node_ + "_gui/kimm_path_planner_ros_interface_server/mobile_action", 1, &RobotController::HuskyFrankaWrapper::mobileActionCallback, this);
        //mobile_action_client_ = n_node_.serviceClient<kimm_path_planner_ros_interface::action_mobile_path>("/" + robot_node_ + "_gui/kimm_path_planner_ros_interface_server/action_mobile_path");
        //joint_action_client_ = n_node_.serviceClient<kimm_joint_planner_ros_interface::action_joint_path>("/" + robot_node_ + "_gui/kimm_joint_planner_ros_interface_server/action_joint_path");
        // se3_action_client_ = n_node_.serviceClient<kimm_se3_planner_ros_interface::action_se3_path>("/" + robot_node_ + "_gui/kimm_se3_planner_ros_interface_server/action_se3_path");
        reset_control_ = true;
        planner_res_ = false;
    }

    void HuskyFrankaWrapper::franka_update(const sensor_msgs::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper) 

        assert(issimulation_);
        for (int i=0; i< 7; i++){
            state_.q_(i+5) = msg.position[i+11];
            state_.v_(i+5) = msg.velocity[i+10];
        }
    }
    void HuskyFrankaWrapper::franka_update(const Vector7d& q, const Vector7d& qdot){ //for experiment
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;
    }
    void HuskyFrankaWrapper::husky_update(const sensor_msgs::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper)

        assert(issimulation_);
        for (int i=0; i<2; i++){ //odom x, y
            state_.q_(i) = msg.position[i];
            state_.v_(i) = msg.velocity[i];
        }

        double theta = atan2(2.* (msg.position[5] * msg.position[4] + msg.position[6] * msg.position[3]), 1- 2.*(pow( msg.position[6], 2) + pow(msg.position[5], 2))); //odom yaw
        state_.q_(2) = theta;
        state_.v_(2) = msg.velocity[5];

        // only for front wheel (not used)
        state_.q_(3) = msg.position[7];
        state_.q_(4) = msg.position[8];
        state_.v_(3) = msg.velocity[6];
        state_.v_(4) = msg.velocity[7];
    }
    void HuskyFrankaWrapper::husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel){ //for experiment
        assert(!issimulation_);
        for (int i=0; i<3; i++){
            state_.q_(i) = base_pos(i);
            state_.v_(i) = base_vel(i);
        }
        for (int i=0; i<2; i++){
            state_.q_(i+3) = wheel_pos(i);
            state_.v_(i+3) = wheel_vel(i);
        }
    }

    void HuskyFrankaWrapper::ctrl_update(const int& msg){
        ctrl_mode_ = msg;
        ROS_INFO("[ctrltypeCallback] %d", ctrl_mode_);
        mode_change_ = true;
    }

    void HuskyFrankaWrapper::compute(const double& time){
        time_ = time;

        robot_->computeAllTerms(data_, state_.q_, state_.v_);
        if (ctrl_mode_ == 0){ //g // gravity mode
            state_.torque_.setZero();
        }
        if (ctrl_mode_ == 1){ //h //init position
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                q_ref_(6) = -M_PI/ 4.0;
                // q_ref_(6) = 0.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                reset_control_ = false;
                mode_change_ = false;

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                // cout << H_ee_ref_ << endl;

            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);

            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));

        }
        if (ctrl_mode_ == 2){ //a //approach to object
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-16, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture (try to maintain current joint configuration)
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));

                //ee
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));

                SE3 T_offset;
                T_offset.setIdentity();
                T_offset.translation(ee_offset_);
                H_ee_ref_ = H_ee_ref_ * T_offset;

                trajEE_Cubic_->setInitSample(H_ee_ref_);

                H_ee_ref_.translation()(0) = 1.0+0.02;
                H_ee_ref_.translation()(2) = -0.3 + 0.2*2+0.04*2 + 0.0 - T_offset.translation()[2]-0.01;

                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                reset_control_ = false;
                mode_change_ = false;
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));

        }
        if (ctrl_mode_ == 3){ //s //move ee -0.1z
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask2_, 1.0, 0);

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));

                SE3 T_offset;
                T_offset.setIdentity();
                T_offset.translation(ee_offset_);
                H_ee_ref_ = H_ee_ref_ * T_offset;

                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(2) -= 0.1;
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(2.0);
                trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                trajMobile_Cubic_->setGoalSample(H_mobile_ref_);

                reset_control_ = false;
                mode_change_ = false;
            }

            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask2_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }
        if (ctrl_mode_ == 4){ //d //move mobile +0.1x with keeping ee
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask2_, 1.0, 0);

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));

                SE3 T_offset;
                T_offset.setIdentity();
                T_offset.translation(ee_offset_);
                H_ee_ref_ = H_ee_ref_ * T_offset;

                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 

                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(2.0);
                trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                H_mobile_ref_.translation()(0) += 0.1;
                trajMobile_Cubic_->setGoalSample(H_mobile_ref_);

                reset_control_ = false;
                mode_change_ = false;
            }

            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask2_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 5){ //f //move mobile -0.1m without arm motion
            if (mode_change_){
                stime_ = time_;

                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-5, 1);                
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);                                

                //posture (try to maintain current joint configuration)
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(5.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));                

                reset_control_ = false;
                mode_change_ = false;
            }            

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            if (stime_ + 5.0 > time_){
                state_.torque_(0) = -5.0;
                state_.torque_(1) = -5.0;
            }
            else{
                state_.torque_(0) = 0.0;
                state_.torque_(1) = 0.0;
            }            

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            // state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
            state_.torque_.tail(na_-2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);

        }

        if (ctrl_mode_ == 6){ //j //align base and joint7 coordinate
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                // q_ref_(6) = -M_PI/ 4.0;
                q_ref_(6) = 0.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                reset_control_ = false;
                mode_change_ = false;

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                // cout << H_ee_ref_ << endl;

            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);

            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 7){ //k //rotate joint1 by 90deg
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  M_PI/2.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                q_ref_(6) = -M_PI/ 4.0;
                // q_ref_(6) = 0.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(5.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                // cout << H_ee_ref_ << endl;

                reset_control_ = false;
                mode_change_ = false;                
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);

            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 10){ //q //rotate ee for object estimation
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask2_, 1.0, 0);

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));

                SE3 T_offset;
                T_offset.setIdentity();
                T_offset.translation(ee_offset_);
                H_ee_ref_ = H_ee_ref_ * T_offset;

                trajEE_Cubic_->setInitSample(H_ee_ref_);

                Matrix3d Rx, Ry, Rz;
                Rx.setIdentity();
                Ry.setIdentity();
                Rz.setIdentity();
                double angle = 15.0*M_PI/180.0;
                // rotx(angle, Rx);
                // roty(angle, Ry);
                rotz(angle, Rz);
                H_ee_ref_.rotation() = Rz * Ry * Rx * H_ee_ref_.rotation();
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(2.0);
                trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                trajMobile_Cubic_->setGoalSample(H_mobile_ref_);

                reset_control_ = false;
                mode_change_ = false;
            }

            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask2_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 11){ //w //rotate ee with sine motion for object estimation
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask2_, 1.0, 0);

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));

                SE3 T_offset;
                T_offset.setIdentity();
                T_offset.translation(ee_offset_);
                H_ee_ref_ = H_ee_ref_ * T_offset;

                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(2.0);
                trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                trajMobile_Cubic_->setGoalSample(H_mobile_ref_);

                reset_control_ = false;
                mode_change_ = false;

                est_time = time_;
            }

            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask2_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            //////////////////
            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext(); //TrajectorySample
            // eeTask_->setReference(sampleEE_); //TaskSE3Equality

            TrajectorySample m_sample;
            m_sample.resize(12, 6); //pos 12, vel 6, pos : translation 3 + rotation matrix 9
            m_sample = sampleEE_;   //take current pos

            if (time_ - est_time < 10.0){
                Matrix3d Rx, Ry, Rz;
                Rx.setIdentity();
                Ry.setIdentity();
                Rz.setIdentity();
                double anglex = 10*M_PI/180.0*sin(1*M_PI*time_);
                double angley = 10*M_PI/180.0*sin(2*M_PI*time_);
                double anglez = 30*M_PI/180.0*sin(0.2*M_PI*time_);
                // rotx(anglex, Rx);
                // roty(angley, Ry);
                rotz(anglez, Rz);

                pinocchio::SE3 H_EE_ref_estimation;
                H_EE_ref_estimation = H_ee_ref_;
                H_EE_ref_estimation.rotation() = Rz * Ry * Rx * H_ee_ref_.rotation();

                SE3ToVector(H_EE_ref_estimation, m_sample.pos);
            }

            eeTask_->setReference(m_sample);
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 12){ //e //rotate joint7 for object estimation
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;                
                q_ref_(6) = -M_PI/ 4.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(5.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);                

                reset_control_ = false;
                mode_change_ = false; 

                est_time = time_;
            }      

            //////////////////
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            // postureTask_->setReference(samplePosture_);

            TrajectorySample m_sample;
            m_sample.resize(7,7); //q 7, dq 7
            m_sample = samplePosture_;   //take current posture

            if (time_ - est_time < 10.0){
                double angle = 10*M_PI/180.0*sin(0.2*M_PI*(time_-est_time));
                m_sample.pos(6) += angle;
            }            

            postureTask_->setReference(m_sample);
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 13){ //r //move -0.1x with joint1 motion (not for)
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =                       M_PI/2;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;                
                q_ref_(6) = -M_PI/ 4.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(4.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);                

                reset_control_ = false;
                mode_change_ = false; 

                est_time = time_;
            }      

            //////////////////
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            // postureTask_->setReference(samplePosture_);

            TrajectorySample m_sample;
            m_sample.resize(7,7); //q 7, dq 7
            m_sample = samplePosture_;   //take current posture

            if (time_ - est_time < 5.0){
                // double angle = 10*M_PI/180.0*sin(0.4*M_PI*(time_-est_time));
                // m_sample.pos(6) += angle;
                // double angle = 20*M_PI/180.0*sin(0.4*M_PI*(time_-est_time));
                // m_sample.pos(5) += angle;
            }            

            postureTask_->setReference(m_sample);
            //////////////////
            
            //////////////////
            if (time_ - est_time < 8.0){
                state_.torque_(0) = -5.0;
                state_.torque_(1) = -5.0;
            }
            else{
                state_.torque_(0) = 0.0;
                state_.torque_(1) = 0.0;
            } 
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            // state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
            state_.torque_.tail(na_-2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);

        }

        if (ctrl_mode_ == 14){ //t //rotate joint6 for object estimation
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0             + M_PI/ 9.0;                
                q_ref_(6) = -M_PI/ 4.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(5.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);                

                reset_control_ = false;
                mode_change_ = false; 

                est_time = time_;
            }      

            //////////////////
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);          
            //////////////////
            
            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 99){ //p //print current ee state
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                cout << H_ee_ref_ << endl;

                reset_control_ = false;
                mode_change_ = false;
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }
    }

    void HuskyFrankaWrapper::franka_output(VectorXd & qacc) {
        qacc = state_.torque_.tail(na_-2);
    }
    void HuskyFrankaWrapper::husky_output(VectorXd & qvel) {
        qvel = state_.torque_.head(2);
    }

    void HuskyFrankaWrapper::com(Eigen::Vector3d & com){
        //The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
        com = robot_->com(data_);
    }

    void HuskyFrankaWrapper::position(pinocchio::SE3 & oMi){
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
    }

    void HuskyFrankaWrapper::velocity(pinocchio::Motion & vel){
        // vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));

        Motion v_frame;
        SE3 T_offset;
        T_offset.setIdentity();
        T_offset.translation(ee_offset_);

        robot_->frameVelocity(data_, robot_->model().getFrameId("panda_joint7"), v_frame);

        vel = T_offset.act(v_frame);

        // SE3 m_wMl_prev;
        // robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl_prev);
        // std::cout << m_wMl_prev << std::endl;
    }

    void HuskyFrankaWrapper::velocity_origin(pinocchio::Motion & vel){
        vel = robot_->velocity_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void HuskyFrankaWrapper::velocity_global(pinocchio::Motion & vel){
        SE3 m_wMl_prev, m_wMl;
        Motion v_frame;

        SE3 T_offset;
        T_offset.setIdentity();
        T_offset.translation(ee_offset_);

        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl_prev);
        m_wMl = m_wMl_prev * T_offset;

        robot_->frameVelocity(data_, robot_->model().getFrameId("panda_joint7"), v_frame);

        vel = m_wMl.act(v_frame);
    }

    void HuskyFrankaWrapper::acceleration(pinocchio::Motion & accel){        
        // accel = robot_->acceleration(data_, robot_->model().getJointId("panda_joint7"));                        
        
        Motion a_frame, m_drift;
        SE3 T_offset;
        T_offset.setIdentity();
        T_offset.translation(ee_offset_);

        robot_->frameAcceleration(data_, robot_->model().getFrameId("panda_joint7"), a_frame);
        robot_->frameClassicAcceleration(data_, robot_->model().getFrameId("panda_joint7"), m_drift);
        
        // accel = T_offset.act(a_frame+m_drift);
        accel = T_offset.act(a_frame);
    }

    void HuskyFrankaWrapper::acceleration_origin(pinocchio::Motion & accel){
        accel = robot_->acceleration_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void HuskyFrankaWrapper::acceleration_global(pinocchio::Motion & accel){
        SE3 m_wMl_prev, m_wMl;
        Motion a_frame, m_drift;

        SE3 T_offset;
        T_offset.setIdentity();
        T_offset.translation(ee_offset_);

        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl_prev);
        m_wMl = m_wMl_prev * T_offset;

        robot_->frameAcceleration(data_, robot_->model().getFrameId("panda_joint7"), a_frame);
        robot_->frameClassicAcceleration(data_, robot_->model().getFrameId("panda_joint7"), m_drift);

        // accel = m_wMl.act(a_frame+m_drift);
        accel = m_wMl.act(a_frame);
    }

    void HuskyFrankaWrapper::force(pinocchio::Force & force){
        force = robot_->force(data_, robot_->model().getJointId("panda_joint7"));
    }

    void HuskyFrankaWrapper::force_origin(pinocchio::Force & force){
        force = robot_->force_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void HuskyFrankaWrapper::force_global(pinocchio::Force & force){
        SE3 m_wMl;
        Force f_frame;
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        robot_->frameForce(data_, robot_->model().getFrameId("panda_joint7"), f_frame);
        force = m_wMl.act(f_frame);
    }

    void HuskyFrankaWrapper::tau(VectorXd & tau_vec){
        tau_vec = robot_->jointTorques(data_).tail(na_-2);
    }

    void HuskyFrankaWrapper::ddq(VectorXd & ddq_vec){
        ddq_vec = robot_->jointAcceleration(data_).tail(na_-2);
    }

    void HuskyFrankaWrapper::mass(MatrixXd & mass_mat){
        mass_mat = robot_->mass(data_).bottomRightCorner(na_-2, na_-2);
    }

    void HuskyFrankaWrapper::nle(VectorXd & nle_vec){
        nle_vec = robot_->nonLinearEffects(data_).tail(na_-2);
    }

    void HuskyFrankaWrapper::g(VectorXd & g_vec){
        g_vec = data_.g.tail(na_-2);
    }

    void HuskyFrankaWrapper::g_joint7(VectorXd & g_vec){
        Vector3d g_global;
        // g_global << 0.0, 0.0, -9.81;
        g_global << 0.0, 0.0, 9.81;
        
        SE3 m_wMl;        
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation
        g_vec = m_wMl.actInv(g_global);
    }

    void HuskyFrankaWrapper::JWorld(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        // Jo2.resize(6, 10);
        Jo2.resize(6, robot_->nv());
        robot_->jacobianWorld(data_, robot_->model().getJointId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void HuskyFrankaWrapper::JLocal_offset(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        // Jo2.resize(6, 10);
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);
        Jo2 = Adj_mat * Jo2;
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void HuskyFrankaWrapper::dJLocal_offset(MatrixXd & dJo){
        Data::Matrix6x dJo2;
        // Jo2.resize(6, 10);
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);
        dJo2 = Adj_mat * dJo2;
        dJo = dJo2.bottomRightCorner(6, 7);
    }

    void HuskyFrankaWrapper::base_state(Vector3d & base){
        base(0) = robot_->getMobilePosition(data_, 5).translation()(0); //5 means state.q_ at panda joint 1 
        base(1) = robot_->getMobilePosition(data_, 5).translation()(1); //5 means state.q_ at panda joint 1 
        base(2) = atan2(-robot_->getMobilePosition(data_, 5).rotation()(0, 1),robot_->getMobilePosition(data_, 5).rotation()(0,0)); //5 means state.q_ at panda joint 1 
    }

    void HuskyFrankaWrapper::ee_state(Vector3d & pos, Eigen::Quaterniond & quat){
        for (int i=0; i<3; i++)
            pos(i) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation()(i);

        Quaternion<double> q(robot_->position(data_, robot_->model().getJointId("panda_joint7")).rotation());
        quat = q;
    }

    void HuskyFrankaWrapper::rotx(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << 1,           0,           0;
        rot.row(1) << 0,           cos(angle), -sin(angle);
        rot.row(2) << 0,           sin(angle),  cos(angle);
    }

    void HuskyFrankaWrapper::roty(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) <<  cos(angle),           0,            sin(angle);
        rot.row(1) <<  0,                    1,            0 ;
        rot.row(2) << -sin(angle),           0,            cos(angle);
    }

    void HuskyFrankaWrapper::rotz(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << cos(angle), -sin(angle),  0;
        rot.row(1) << sin(angle),  cos(angle),  0;
        rot.row(2) << 0,                    0,  1;
    }
}// namespace
