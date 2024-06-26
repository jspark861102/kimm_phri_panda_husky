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
    FrankaHuskyWrapper::FrankaHuskyWrapper(const std::string & robot_node, const bool & issimulation, const bool & ismobile, const bool & isrobotiq, const bool & iscalibration, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), ismobile_(ismobile), isrobotiq_(isrobotiq), getcalibration_(iscalibration), n_node_(node)
    {
        time_ = 0.;        
        node_index_ = 0;
        cnt_ = 0;

        mode_change_ = false;
        ctrl_mode_ = 0;                
    }

    void FrankaHuskyWrapper::initialize(){        
        // Robot for pinocchio
        string model_path, urdf_name;
        n_node_.getParam("/" + robot_node_ +"/robot_urdf_path", model_path);
        n_node_.getParam("/" + robot_node_ +"/robot_urdf", urdf_name);        //"panda_arm_hand_l.urdf" w/o mobile, "husky_panda_hand.urdf" w/ mobile

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;

        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, ismobile_, false); //first false : w/o mobile, true : w/ mobile
        model_ = robot_->model();
        
        //nq_/nv_/na_ is # of joint w.r.t pinocchio model ("panda_arm_hand_l.urdf"), so there is no gripper joints
        nq_ = robot_->nq(); //12 : odom (3 - x,y,theta) + husky (2) + franka (7)
        nv_ = robot_->nv(); //12
        na_ = robot_->na(); //9  : nv-3, odom dof eliminated

        // State (for pinocchio)
        state_.q_.setZero(nq_);
        state_.v_.setZero(nv_);
        state_.dv_.setZero(nv_);
        state_.torque_.setZero(na_);
        state_.tau_.setZero(na_);

        // tsid
        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q_, state_.v_);
        data_ = tsid_->data();

        // tasks
        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        
        VectorXd posture_gain;
        if (ismobile_) posture_gain.resize(na_-2);
        else           posture_gain.resize(na_);
        
        if (!issimulation_) //for real
        	// posture_gain << 200., 200., 200., 200., 200., 200., 200.;            
            posture_gain << 100., 100., 100., 200., 200., 200., 200.;
        else // for simulation        	
            posture_gain << 40000., 40000., 40000., 40000., 40000., 40000., 40000.;

        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());
        
        //////////////////// EE offset ////////////////////////////////////////        
        //When offset is applied, the control characteristics seems to be changed (gain tuning is needed)
        //This offset is applied to both reference (in this code) and feedback (in task_se3_equality.cpp)        
        if (isrobotiq_) joint7_to_finger_ = 0.247; //0.247 (z-axis) = 0.222(distance from link7 to left_inner_finger) + 0.025(finger center length)         
        else            joint7_to_finger_ = 0.2054; //0.2054 (z-axis) = 0.107(distance from link7 to EE) + 0.0584(hand length) + 0.04(finger length)          
        this->eeoffset_update(); 
        
        this->R_joint7();   //make rotation matrix of joint7 from global to local          

        //////////////////// human grasp position (object length) //////////////////////////////////////// 
        //define object length 
        obj_length_global_ = Vector3d(0.0, -1.5, 0.0); // 1.5 length in global y axis                                
        obj_length_local_  = R_joint7_atHome_ * obj_length_global_; // global to local                

        this->EstimationMotion();
        
        //trajectory length in time for object parameter estimation
        traj_length_in_time_ = 10.0;    
        ///////////////////////////////////////////////////////////////////////        

        VectorXd ee_gain(6);        
        if (!issimulation_) { //for real                
            ee_gain << 100., 100., 100., 400., 400., 600.;                
        }
        else { //for simulation
            // ee_gain << 500., 500., 500., 800., 800., 1000.;
            ee_gain << 1000., 1000., 1000., 2000., 2000., 2000.;
        }                                
        
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset_); //here, ee_offset_ is applied to current pos value
        eeTask_->Kp(ee_gain*Vector::Ones(6));
        // eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());        
        eeTask_->Kd(1.0*eeTask_->Kp().cwiseSqrt());                        

        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        Vector dq_max = 500000.0*Vector::Ones(na_);
        dq_max(0) = 500.; //? 
        dq_max(1) = 500.; //?
        Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);        

        if (ismobile_) {
            mobileTask_ = std::make_shared<TaskMobileEquality>("task-mobile", *robot_, true); //ori
            mobileTask_->Kp(50.0*Vector3d::Ones());
            mobileTask_->Kd(2.5*mobileTask_->Kp().cwiseSqrt());

            mobileTask2_ = std::make_shared<TaskMobileEquality>("task-mobile2", *robot_, false); //trans
            mobileTask2_->Kp(100.0*Vector3d::Ones());
            mobileTask2_->Kd(2.5*mobileTask2_->Kp().cwiseSqrt());
        }

        // trajecotries
        sampleEE_.resize(12, 6); //12=3(translation)+9(rotation matrix), 6=3(translation)+3(rotation)

        if (ismobile_) samplePosture_.resize(na_-2); //na_=9 (husky 2 + franka 7)
        else           samplePosture_.resize(na_); //na_=7 franka 7

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");

        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);
        
        if (ismobile_) {
            sampleMobile_.resize(12, 6);
            trajMobile_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_mobile");
            trajMobile_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_mobile_constant");
            Vector3d Maxvel_base = Vector3d::Ones()*1.0;
            Vector3d Maxacc_base = Vector3d::Ones()*1.0;
            trajMobile_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_mobile_timeopt", Maxvel_base, Maxacc_base);
        }

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "qpoases");

        // service
        reset_control_ = true; 
        initial_calibration_update_ = false;                 

        //inertia shaping --------------------------------//
        Me_.resize(6,6);
        Me_inv_.resize(6,6);
        Me_inv_.setIdentity();                 
        eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
        if (ismobile_) {
            mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));
            mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));
        }
        //-----------------------------------------------//

        //Fh gain in global ------------------------------//        
        Fh_.resize(6,6);    
        Fh_.setZero();                 
        
        //case.1 :use translational axis        
        // Fh_.topLeftCorner(3,3).setIdentity();                
        
        //case.2 : only use rx-axis to move z-axis 
        Fh_(2,3) = -1.0 * 4.0;     
        //-----------------------------------------------//
        
        //Fh gain in local ------------------------------//
        Fh_local_.resize(6,6) ;        
        Fh_local_.setZero();
        
        //case.1 : use local rz-axis to move local y-axis
        Fh_local_(1,5) = -5.0;  
        //-----------------------------------------------//

        cout << " " << endl;
        cout << "Fh_" << endl;
        cout << Fh_ << endl;
        cout << " " << endl;
        cout << "Fh_local_" << endl;
        cout << Fh_local_ << endl;    
    }    
    void FrankaHuskyWrapper::CartesianInertia(MatrixXd & Me_){
        MatrixXd Mx, rGh, Pd; //local
        this->MxLocal_offset(Mx);
        
        rGh.resize(6,6);
        rGh.setIdentity();
        rGh.bottomLeftCorner(3,3) = -1.0 * skew_matrix(obj_length_local_);

        Pd.resize(6,6);
        Pd.setZero();
        Pd(1,5) = Fh_local_(1,5);
        Pd(2,4) = -1.0 * Fh_(2,3);

        Me_ = (rGh + Pd).inverse() * Mx;        

        // MatrixXd Mx, rGh, Pd; //local
        // this->MxGlobal(Mx);
        
        // rGh.resize(6,6);
        // rGh.setIdentity();
        // rGh.bottomLeftCorner(3,3) = -1.0 * skew_matrix(obj_length_global_);

        // Pd.resize(6,6);
        // Pd.setZero();
        // Pd(0,5) = 5.0;//5.0;
        // Pd(2,3) = -4.0;//-4.0;

        // Me_ = (rGh + Pd).inverse() * Mx;        
        // // Me_ = rGh.inverse() * Mx;        
        // // Me_ = Mx;        
    }

    void FrankaHuskyWrapper::EstimationMotion(){ 
        //define hGr matrix in local coordinate
        hGr_local_.resize(3,6);
        hGr_local_.topLeftCorner(3,3).setIdentity();         
        hGr_local_.topRightCorner(3,3) = -1 * skew_matrix(obj_length_local_);

        cout << " " << endl;
        cout << "hGr_local" << endl;
        cout << hGr_local_ << endl;
        
        hGr_local_pinv_.resize(6,3);
        hGr_local_pinv_ = hGr_local_.completeOrthogonalDecomposition().pseudoInverse();

        cout << " " << endl;
        cout << "hGr_local_pinv_" << endl;
        cout << hGr_local_pinv_ << endl;

        hGr_local_Null_.resize(6,6);
        hGr_local_Null_ = MatrixXd::Identity(6,6) - hGr_local_pinv_* hGr_local_;

        cout << " " << endl;
        cout << "hGr_local_Null_" << endl;
        cout << hGr_local_Null_ << endl;

        //define P matrix
        P_global_.resize(6,6);
        P_global_.setIdentity();
        P_global_.bottomLeftCorner(3,3) = -1*skew_matrix(obj_length_global_);        

        P_local_.resize(6,6);
        P_local_.setIdentity();
        P_local_.bottomLeftCorner(3,3) = -1*skew_matrix(obj_length_local_);
    }
    
    void FrankaHuskyWrapper::R_joint7(){ 
        Matrix3d Rx, Ry, Rz;
        double anglex, angley, anglez;         
        if (isrobotiq_) {
            anglex = -180.0*M_PI/180.0;
            anglez =   90.0*M_PI/180.0;
            this->rotx(anglex, Rx);
            this->rotz(anglez, Rz);   
            R_joint7_atHome_ = Rz * Rx;            
        }
        else { //franka gripper
            anglex = -180.0*M_PI/180.0;
            anglez =   45.0*M_PI/180.0;
            this->rotx(anglex, Rx);
            this->rotz(anglez, Rz);   
            R_joint7_atHome_ = Rz * Rx;            
        }       
        cout << " " << endl;
        cout << "R_joint7_atHome_" << endl;
        cout << R_joint7_atHome_ << endl;            
    }

    void FrankaHuskyWrapper::eeoffset_update(){ 
        ee_offset_ = Vector3d(0.0, 0.0, joint7_to_finger_); //w.r.t joint7

        T_offset_.setIdentity();
        T_offset_.translation(ee_offset_);                

        Adj_mat_.resize(6,6);
        Adj_mat_.setIdentity();
        Adj_mat_.topRightCorner(3,3) = -1 * skew_matrix(ee_offset_); //due to "A cross B = -B cross A"
    }

    void FrankaHuskyWrapper::franka_update(const sensor_msgs::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper) 

        assert(issimulation_);        
        for (int i=0; i< 7; i++){
            if (ismobile_) {
                state_.q_(i+5) = msg.position[i+11];
                state_.v_(i+5) = msg.velocity[i+10];            
            }
            else {                
                state_.q_(i) = msg.position[i];
                state_.v_(i) = msg.velocity[i];
            }
        }        
    }
    void FrankaHuskyWrapper::franka_update(const Vector7d& q, const Vector7d& qdot){ //for experiment
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;
    }        

    void FrankaHuskyWrapper::franka_update(const Vector7d& q, const Vector7d& qdot, const Vector7d& tau){ //for experiment, use pinocchio::aba
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;

        state_.tau_.tail(7) = tau;
    }     

    void FrankaHuskyWrapper::husky_update(const sensor_msgs::JointState& msg){ //for simulation (mujoco)
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
    void FrankaHuskyWrapper::husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel){ //for experiment
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

    void FrankaHuskyWrapper::Fext_update(const Vector6d& Fext){ //for simulation & experiment
        Fext_ = Fext;

        // cout << Fext_.transpose() << endl;
    }   

    void FrankaHuskyWrapper::ctrl_update(const int& msg){
        ctrl_mode_ = msg;
        ROS_INFO("[ctrltypeCallback] %d", ctrl_mode_);
        mode_change_ = true;
    }

    void FrankaHuskyWrapper::compute(const double& time){
        time_ = time;

        robot_->computeAllTerms(data_, state_.q_, state_.v_);
        // robot_->computeAllTerms_ABA(data_, state_.q_, state_.v_, state_.tau_); //to try to use data.ddq (only computed from ABA) However,the ddq value with ABA is not reasonalbe.        

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------ basic motion -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 0){ //g // gravity mode
            state_.torque_.setZero();
        }
        
        if (ctrl_mode_ == 1){ //h //init position
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

                //posture
                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = -M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                
                if (isrobotiq_) q_ref_(6) = -M_PI/ 2.0;                
                else            q_ref_(6) = -M_PI/ 4.0;                

                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(3.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);                

                reset_control_ = false;
                mode_change_ = false;             
            } 

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_); 
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }  

        if (ctrl_mode_ == 2){ //f //stop with maintaining current position
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

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);                
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));            

                reset_control_ = false;
                mode_change_ = false;             
            } 

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_); 
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }        

        //---------------------------------------------------------------------------------------------------------//  
        //----------------------------------------- estimation motion ---------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//      
        if (ctrl_mode_ == 11){ //w //identification motion w/o mobile motion, getcalibration available 
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

                //posture
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_; 

                //mobility                
                if (ismobile_) { 
                    target_time_ = traj_length_in_time_;

                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(target_time_);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_); 
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);  
                }
               
                reset_control_ = false;
                mode_change_ = false;
                trjectory_end_ = false;
                
                est_time_ = time_;
                T_vel_.setIdentity();

                if (getcalibration_){
                    fout_.open("/home/kimm/kimm_catkin/src/kimm_phri_panda_husky/calibration/calibration.txt");
                }
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
            }
            
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);                        
            ////////////////// 

            VectorXd vel_vec, vel_vec_pseudo, vel_vec_null, vel_vec_null_forpseudo;            
            vel_vec.resize(6);
            vel_vec_pseudo.resize(3);   
            vel_vec_null.resize(6);                                
            vel_vec_null_forpseudo.resize(6);                                           
            if (time_ - est_time_ < target_time_){  
                //for pseudo motion----------------------------------------------------------//                
                vel_vec_pseudo << 0.0, 0.0, 0.0;                

                //vel_vec_pseudo should be used here to eliminate rx motion in pseudo motion. see the relation of matrix hGr_local_pinv_ and hGr_local_Null_
                vel_vec_null_forpseudo << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;                  
                //---------------------------------------------------------------------------//                

                //for null motion------------------------------------------------------------//
                if (time_ - est_time_ < traj_length_in_time_){                                        
                    double fx = 0.4;                                
                    double fy = 0.4;                                
                    double fz = 0.4;                                
                    if (isrobotiq_) { 
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -8*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_));
                        double angley = +5*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double anglez = -3*M_PI/180.0*2.0*M_PI*fz*cos(2.0*M_PI*fz*(time_ - est_time_));                        
                        // double anglez = 0.0;                        
                    
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, anglez; //joint7 is 90deg rotated w.r.t. global coordinate
                    }
                    else {
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) -3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double angley = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) +3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));                                            
                        
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, 0.0;
                    }

                    if (getcalibration_){
                    fout_ << Fext_.transpose() << "\n";
                    }                                
                }
                else { //when estimation is finished
                    if (!trjectory_end_){                    
                        trjectory_end_ = true;
                        
                        if (getcalibration_) {
                            fout_.close();          
                        }
                    }
                    vel_vec_null.setZero();
                }
                //---------------------------------------------------------------------------//                
                    
                vel_vec = hGr_local_pinv_ * vel_vec_pseudo + hGr_local_Null_ * (vel_vec_null_forpseudo + vel_vec_null);                
            }
            else{  //when pseudo motion is end                
                // maintain zero velocity after the trajectory is finished to avoid hard stop -----------//                                                
                vel_vec.setZero();                                
                // ------------------------------------------------------------------------------------//
            }

            T_vel_ = T_vel_ * vel_to_SE3(vel_vec, dt_);   //velocity integration to make position
            pinocchio::SE3 H_EE_ref_estimation;
            H_EE_ref_estimation = H_ee_ref_;                
            H_EE_ref_estimation = H_EE_ref_estimation * T_vel_;                
            SE3ToVector(H_EE_ref_estimation, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }                   

        if (ctrl_mode_ == 12){ //e //identification motion w/ mobile motion 
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

                //posture
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;

                //mobility                
                if (ismobile_) {                    
                    target_distance_ = 1.5;
                    target_time_ = 15.0;
                    // target_velocity_ = target_distance_ / target_time_;                                        
                    
                    //*----- Use of mobileTask and eeTask together diverge (why?) -----*//
                    // tsid_->addMotionTask(*mobileTask_, 1.0, 1);                                        
                    // mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask2_, 0.1, 1);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(target_time_);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_); 

                    H_mobile_ref_.translation()(0) += target_distance_;                   
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);  
                }
               
                reset_control_ = false;
                mode_change_ = false;
                trjectory_end_ = false;
                
                est_time_ = time_;
                T_vel_.setIdentity();                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                // mobileTask_->setReference(sampleMobile_);                                
                mobileTask2_->setReference(sampleMobile_);                                
            }
            
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);                        
            ////////////////// 

            VectorXd vel_vec, vel_vec_pseudo, vel_vec_null, vel_vec_null_forpseudo;            
            vel_vec.resize(6);
            vel_vec_pseudo.resize(3);   
            vel_vec_null.resize(6);                                
            vel_vec_null_forpseudo.resize(6);                                           
            if (time_ - est_time_ < target_time_){  
                //for pseudo motion----------------------------------------------------------//                
                //constant velocity
                // vel_vec_pseudo  = R_joint7_atHome_ * Vector3d(target_velocity_, 0.0, 0.0); // -0.01 m/s in global x axis
                
                //cubic velocity
                double a1, a2, vel_tra;
                a1 = +6.0 / pow(target_time_, 2) * target_distance_;
                a2 = -6.0 / pow(target_time_, 3) * target_distance_;
                vel_tra = a1 * (time_ - est_time_) + a2 * pow(time_ - est_time_, 2);
                vel_vec_pseudo  = R_joint7_atHome_ * Vector3d(vel_tra, 0.0, 0.0); // -0.01 m/s in global x axis                

                //vel_vec_pseudo should be used here to eliminate rx motion in pseudo motion. see the relation of matrix hGr_local_pinv_ and hGr_local_Null_
                vel_vec_null_forpseudo << vel_vec_pseudo(0), vel_vec_pseudo(1), 0.0, 0.0, 0.0, 0.0;                  
                //---------------------------------------------------------------------------//                

                //for null motion------------------------------------------------------------//
                if (time_ - est_time_ < traj_length_in_time_){                                        
                    double fx = 0.4;                                
                    double fy = 0.4;                                
                    double fz = 0.4;                                
                    if (isrobotiq_) { 
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -8*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_));
                        double angley = +5*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double anglez = -3*M_PI/180.0*2.0*M_PI*fz*cos(2.0*M_PI*fz*(time_ - est_time_));                        
                        // double anglez = 0.0;                        
                    
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, anglez;
                    }
                    else {
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) -3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double angley = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) +3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));                                            
                        
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, 0.0;
                    }                    
                }
                else { //when estimation is finished
                    if (!trjectory_end_){                    
                        trjectory_end_ = true;                                             
                    }
                    vel_vec_null.setZero();
                }
                //---------------------------------------------------------------------------//                
                    
                vel_vec = hGr_local_pinv_ * vel_vec_pseudo + hGr_local_Null_ * (vel_vec_null_forpseudo + vel_vec_null);                
            }
            else{  //when pseudo motion is end                
                // maintain zero velocity after the trajectory is finished to avoid hard stop -----------//                                                
                vel_vec.setZero();                                
                // ------------------------------------------------------------------------------------//
            }

            T_vel_ = T_vel_ * vel_to_SE3(vel_vec, dt_);   //velocity integration to make position
            pinocchio::SE3 H_EE_ref_estimation;
            H_EE_ref_estimation = H_ee_ref_;                
            H_EE_ref_estimation = H_EE_ref_estimation * T_vel_;                
            SE3ToVector(H_EE_ref_estimation, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }          

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------- control motion ----------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//          
        if (ctrl_mode_ == 21){ //c //admittance control
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

                //posture
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                

                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);   
                }
               
                reset_control_ = false;
                mode_change_ = false;
                
                est_time_ = time_;      
                initial_calibration_update_ = false;          
                
                //damping gain for addmittance control
                Dinv_gain_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.03; 
                Dinv_gain_ *= dt_;
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
            }
            
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            //////////////////   
            if ((!initial_calibration_update_) && (time_ - est_time_ > 0.5)){ //update calibration after controller runs
                Fext_calibration_ = Fext_; //Fext is in GLOBAL
                initial_calibration_update_ = true;
                cout << "initial ext_calibration_" << endl;
                cout << Fext_calibration_.transpose() << endl;            

                bool a[6] = {false};
                for (int i=0; i<6; i++) {
                    a[i] = Dinv_gain_(i) * eeTask_->Kp()(i) < 1.0;
                }                
                cout << "Kf gain verification" << endl;
                cout << a[0] << "   "  << a[1] << "   "  << a[2] << "   "  << a[3] << "   "  << a[4] << "   "  << a[5] << endl;                                                                                
            }
            if (initial_calibration_update_) {                                                                                                                
                SE3 m_wMl;                
                m_wMl = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                m_wMl.translation() << 0.0, 0.0, 0.0;
                
                Force Fext_global;
                Fext_global = Fext_ - Fext_calibration_;

                Vector3d offset_vec_global, offset_vec_local;
                // offset_vec_global(0) =  Dinv_gain_(5) * noise_elimination(Fext_global.angular()(2), 0.3) * 0.0;
                // offset_vec_global(1) =  Dinv_gain_(1) * noise_elimination(Fext_global.linear()(1), 1.0) * 0.0;
                // offset_vec_global(2) = -Dinv_gain_(3) * noise_elimination(Fext_global.angular()(0), 0.0);                

                offset_vec_global(0) =  Dinv_gain_(0) * noise_elimination(Fext_global.linear()(0), 1.0);
                offset_vec_global(1) =  Dinv_gain_(1) * noise_elimination(Fext_global.linear()(1), 1.0);
                offset_vec_global(2) =  Dinv_gain_(2) * noise_elimination(Fext_global.linear()(2), 1.0);      

                offset_vec_local = m_wMl.actInv(offset_vec_global); //global to local                
                
                SE3 Fext_offset;
                Fext_offset.setIdentity();
                Fext_offset.translation() = offset_vec_local;  
                //////////////////  

                H_ee_ref_ = H_ee_ref_ * Fext_offset; //integration manner                                                                                                                             
                
                ////////////////////////////////////////////////////////
                // offset_vec_local.setZero();
                // offset_vec_local(2) = Dinv_gain_(3) * noise_elimination((Fext_-Fext_calibration_)(3), 1.0);                
                // H_ee_ref_.translation() += -offset_vec_local; //integration manner                                                                             

                // // calibration update --> hard to use, if position is changed a lot, the use of f button strategy is better than calibration update
                // if (fabs(Fext_offset.translation()(2)) < 1e-5) {
                //     Fext_calibration_ = Fext_;
                //     cout << "updated Fext_calibration_" << endl;
                //     cout << Fext_calibration_.transpose() << endl;            
                // } 
            }            
           
            SE3ToVector(H_ee_ref_, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        } 

        if (ctrl_mode_ == 22){ //v //impedance control
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

                //posture
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                

                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);   

                //mobility
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }
               
                reset_control_ = false;
                mode_change_ = false;                                                
            }            

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
            }
            
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);                        
            
            if (1) { 
                //to make K(x-xd)=0, put xd=x 
                H_ee_ref_.translation() = (robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_).translation();
                
                //inertia shaping    
                // Me_inv_ = 
                eeTask_->setDesiredinertia(Me_inv_);
            }
            else {   
                //to make K(x-xd)=0, put K=0 
                Vector6d a;
                a = eeTask_->Kp();
                a.head(3) << 0.0, 0.0, 0.0;
                eeTask_->Kp(a); //after this task, Kp should be returned to original gain
            }                        

            SE3ToVector(H_ee_ref_, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }    

        if (ctrl_mode_ == 23){ //m //impedance control with K 
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
                tsid_->addMotionTask(*eeTask_, 1, 0);

                //posture
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);    

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setDuration(3.0);
                trajEE_Cubic_->setStartTime(time_);                                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);               

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);  
                }

                reset_control_ = false;
                mode_change_ = false;     
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------- arm ee jog --------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 31){ //i //move ee +0.1z
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

                //posture
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(2) += 0.1;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);    
                }

                reset_control_ = false;
                mode_change_ = false;                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        if (ctrl_mode_ == 32){ //k //move ee -0.1z
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

                //posture 
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(2) -= 0.1;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_); 
                }

                reset_control_ = false;
                mode_change_ = false;                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        if (ctrl_mode_ == 33){ //l //move ee +0.1x
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

                //posture 
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(0) += 0.1;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);   
                }

                reset_control_ = false;
                mode_change_ = false;                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        if (ctrl_mode_ == 34){ //j //move ee -0.1x
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

                //posture 
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(0) -= 0.1;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);   
                }

                reset_control_ = false;
                mode_change_ = false;                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        if (ctrl_mode_ == 35){ //u //move ee +0.1y
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
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(1) += 0.1;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);   
                }

                reset_control_ = false;
                mode_change_ = false;                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        if (ctrl_mode_ == 36){ //o //move ee -0.1y
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
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(1) -= 0.1;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);                    
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);   
                }

                reset_control_ = false;
                mode_change_ = false;                
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        //---------------------------------------------------------------------------------------------------------//  
        //-------------------------------------------- mobile jog -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 41){ //a //move mobile +1.5x with maintaining current arm posture
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

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                // trajPosture_Cubic_->setGoalSample(q_ref_);               
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_, 1.0, 1);
                    mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask2_, 0.1, 1);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(5.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(0) += 1.5;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                reset_control_ = false;
                mode_change_ = false;
                est_time_ = time_;
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask_->setReference(sampleMobile_);
                mobileTask2_->setReference(sampleMobile_);
            }          

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));

            if (time_ > est_time_ + 5.0 + 3.0) state_.torque_.head(2).setZero();
        }

        if (ctrl_mode_ == 42){ //s //move mobile -1.5x with maintaining current arm posture
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

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                // trajPosture_Cubic_->setGoalSample(q_ref_);               
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_, 1.0, 1);                                        
                    mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask2_, 0.1, 1);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(5.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(0) -= 1.5;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                reset_control_ = false;
                mode_change_ = false;
                est_time_ = time_;
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask_->setReference(sampleMobile_);
                mobileTask2_->setReference(sampleMobile_);
            }          

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));

            if (time_ > est_time_ + 5.0 + 3.0) state_.torque_.head(2).setZero();
        }                  

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------- test motion -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 101){ //1 //test move: wholebody motion for 0.3x & -0.1z ee pos
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

                //posture 
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);

                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(0) += 0.3;                                
                H_ee_ref_.translation()(2) -= 0.1;                
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

        if (ctrl_mode_ == 102){ //2 //test move: chicken head motion with +0.1m mobile motion
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

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {
                    tsid_->addMotionTask(*mobileTask_, 1.0, 0);
                    mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(0) += 0.1;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                reset_control_ = false;
                mode_change_ = false;
                est_time_ = time_;
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask_->setReference(sampleMobile_);
                mobileTask2_->setReference(sampleMobile_);
            }          

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));

            if (time_ > est_time_ + 2.0 + 3.0) state_.torque_.head(2).setZero();
        }  

        if (ctrl_mode_ == 103){ //3 //test move: chicken head motion with -0.1m mobile motion
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

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_, 1.0, 0);
                    mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(0) -= 0.1;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                reset_control_ = false;
                mode_change_ = false;
                est_time_ = time_;
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask_->setReference(sampleMobile_);
                mobileTask2_->setReference(sampleMobile_);
            }          

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));

            if (time_ > est_time_ + 2.0 + 3.0) state_.torque_.head(2).setZero();
        }  

        if (ctrl_mode_ == 104){ //4 //test move: mobile base backward during 5sec, maintaining posture        
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

                //posture
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);                           

                reset_control_ = false;
                mode_change_ = false;
                est_time_ = time_;
            }
            // husky
            if (est_time_ + 5.0 > time_) {                
                state_.torque_(0) = -50.;
                state_.torque_(1) = -50.;                
            }          
            else{
                state_.torque_(0) = 0.0;
                state_.torque_(1) = 0.0;                
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);            
            state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);             
        }

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------------ etc ----------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
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
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));

                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(2.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                cout << "current EE" << endl;
                cout << H_ee_ref_ << endl;

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);                    
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                }

                reset_control_ = false;
                mode_change_ = false;                              
            }

            if (ismobile_) {
                // husky
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
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

        // cout << state_.torque_.transpose() << endl;
    }        

    void FrankaHuskyWrapper::franka_output(VectorXd & qacc) { //from here to main code
        if (ismobile_) qacc = state_.torque_.tail(na_-2);
        else qacc = state_.torque_.tail(na_);
    }  
    void FrankaHuskyWrapper::husky_output(VectorXd & qvel) {
        qvel = state_.torque_.head(2);
    }  

    void FrankaHuskyWrapper::com(Eigen::Vector3d & com){
        //API:Vector of subtree center of mass positions expressed in the root joint of the subtree. 
        //API:In other words, com[j] is the CoM position of the subtree supported by joint j and expressed in the joint frame .         
        //API:The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
        com = robot_->com(data_);
    }

    void FrankaHuskyWrapper::position(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::position_offset(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
    }

    void FrankaHuskyWrapper::position_link0_offset(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        SE3 panda_link0_ee;
        panda_link0_ee = robot_->framePosition(data_, robot_->model().getFrameId("panda_link0"));

        SE3 panda_joint7_offset;
        this->position_offset(panda_joint7_offset);
        
        oMi = panda_link0_ee.inverse() * panda_joint7_offset;    
    }

    void FrankaHuskyWrapper::velocity(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the centers of the joints.
        vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));

        // NOTES ////////////////////////////////////////////////////////////////////////////
        // data.v = f.placement.actInv(data.v[f.parent]), both are in LOCAL coordinate
        // T_offset.act(v_frame) is WRONG method, It means that applying offset w.r.t. global coord. to EE frame.                
        //////////////////////////////////////////////////////////////////////////////////////

        // code comparison ///////////////////////////        
        // To apply offset, Adj_mat should be used
        //////////////////////////////////////////////                                
        // cout << "data.v" << endl;
        // cout <<  robot_->velocity(data_, robot_->model().getJointId("panda_joint7")) << endl;        

        // cout << "data.v with Adj_mat_" << endl;
        // cout << vel.linear() + Adj_mat_.topRightCorner(3,3) * vel.angular() << endl;
        // cout << vel.angular() << endl;        
    }

    void FrankaHuskyWrapper::velocity_offset(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the centers of the joints.
        vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));
        
        vel.linear() = vel.linear() + Adj_mat_.topRightCorner(3,3) * vel.angular();
        vel.angular() = vel.angular(); 
    }

    void FrankaHuskyWrapper::velocity_origin(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the origin. (data.ov)
        //Same with "vel = m_wMl.act(v_frame);"
        vel = robot_->velocity_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration(pinocchio::Motion & accel){        
        //API:Vector of joint accelerations expressed at the centers of the joints frames. (data.a)
        accel = robot_->acceleration(data_, robot_->model().getJointId("panda_joint7"));                                        
    }

    void FrankaHuskyWrapper::acceleration_origin(pinocchio::Motion & accel){
        //API:Vector of joint accelerations expressed at the origin of the world. (data.oa)        
        //It is not available!!!!!!!!!!!!!!! (always zero), so acceleratioon_global is used.
        accel = robot_->acceleration_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration_origin2(pinocchio::Motion & accel){
        //It is defined becuase data.oa is not available (output always zero) 
        SE3 m_wMl;
        Motion a_frame;        
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);       //data.oMi     
        robot_->frameAcceleration(data_, robot_->model().getFrameId("panda_joint7"), a_frame); //data.a         
        accel = m_wMl.act(a_frame);
    }

    void FrankaHuskyWrapper::force(pinocchio::Force & force){
        //API:Vector of body forces expressed in the local frame of the joint. 
        //API:For each body, the force represents the sum of all external forces acting on the body.
        force = robot_->force(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::force_origin(pinocchio::Force & force){
        //API:Vector of body forces expressed in the world frame. 
        //API:For each body, the force represents the sum of all external forces acting on the body.        
        //It is not available!!!!!!!!!!!!!!! (always zero), so acceleratioon_global is used.
        force = robot_->force_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::force_origin2(pinocchio::Force & force){
        //It is defined becuase data.of is not available (output always zero)         
        SE3 m_wMl;
        Force f_frame;
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        robot_->frameForce(data_, robot_->model().getFrameId("panda_joint7"), f_frame);
        force = m_wMl.act(f_frame);
    }

    void FrankaHuskyWrapper::tau(VectorXd & tau_vec){
        //API:Vector of joint torques (dim model.nv).
        //It is not available (output always zero)
        if (ismobile_) tau_vec = robot_->jointTorques(data_).tail(na_-2);
        else           tau_vec = robot_->jointTorques(data_).tail(na_);
    }

    void FrankaHuskyWrapper::ddq(VectorXd & ddq_vec){
        //API:The joint accelerations computed from ABA.
        //It is not available (output always zero), will be available with ABA method
        //even though with ABA method, the value is not reasonable (behave like torque, not ddq)
        if (ismobile_) ddq_vec = robot_->jointAcceleration(data_).tail(na_-2);
        else           ddq_vec = robot_->jointAcceleration(data_).tail(na_);
    }

    void FrankaHuskyWrapper::mass(MatrixXd & mass_mat){
        if (ismobile_) mass_mat = robot_->mass(data_).bottomRightCorner(na_-2, na_-2);
        else           mass_mat = robot_->mass(data_).bottomRightCorner(na_, na_);
    }

    void FrankaHuskyWrapper::nle(VectorXd & nle_vec){
        if (ismobile_) nle_vec = robot_->nonLinearEffects(data_).tail(na_-2);
        else           nle_vec = robot_->nonLinearEffects(data_).tail(na_);
    }

    void FrankaHuskyWrapper::g(VectorXd & g_vec){
        //API:Vector of generalized gravity (dim model.nv).
        if (ismobile_) g_vec = data_.g.tail(na_-2);
        else           g_vec = data_.g.tail(na_);
    }

    void FrankaHuskyWrapper::g_joint7(VectorXd & g_vec){
        Vector3d g_global;
        // g_global << 0.0, 0.0, -9.81;
        g_global << 0.0, 0.0, 9.81;
        
        SE3 m_wMl;                
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation
        g_vec = m_wMl.actInv(g_global);
    }

    void FrankaHuskyWrapper::g_local_offset(VectorXd & g_vec){
        Vector3d g_global;
        g_global << 0.0, 0.0, -9.81;
        //g_global << 0.0, 0.0, 9.81;
        
        SE3 m_wMl;                
        m_wMl = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;        
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation                
        g_vec = m_wMl.actInv(g_global);
    }

    void FrankaHuskyWrapper::JWorld(MatrixXd & Jo){
        Data::Matrix6x Jo2;        
        Jo2.resize(6, robot_->nv());
        robot_->jacobianWorld(data_, robot_->model().getJointId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::JLocal(MatrixXd & Jo){
        Data::Matrix6x Jo2;        
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);        
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::JLocal_offset(MatrixXd & Jo){
        Data::Matrix6x Jo2;        
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);        
        Jo = Jo2.bottomRightCorner(6, 7);
        Jo = Adj_mat_ * Jo;
    }

    void FrankaHuskyWrapper::dJLocal(MatrixXd & dJo){
        Data::Matrix6x dJo2;        
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);        
        dJo = dJo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::dJLocal_offset(MatrixXd & dJo){
        Data::Matrix6x dJo2;        
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);        
        dJo = dJo2.bottomRightCorner(6, 7);
        dJo = Adj_mat_ * dJo;
    }

    void FrankaHuskyWrapper::MxLocal_offset(MatrixXd & Mx){
        MatrixXd M, J, J_inverse, J_transpose_inverse;
        this->mass(M);
        this->JLocal_offset(J);
        J_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        J_transpose_inverse = J.transpose().completeOrthogonalDecomposition().pseudoInverse();        

        Mx = J_transpose_inverse * M * J_inverse;    
    }

    void FrankaHuskyWrapper::MxGlobal(MatrixXd & Mx){
        MatrixXd M, J, J_inverse, J_transpose_inverse;
        this->mass(M);
        this->JWorld(J);
        J_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        J_transpose_inverse = J.transpose().completeOrthogonalDecomposition().pseudoInverse();        

        Mx = J_transpose_inverse * M * J_inverse;    
    }    

    void FrankaHuskyWrapper::Fh_gain_matrix(MatrixXd & Fh, MatrixXd & Fh_local, bool isinteractiongain){
        if (isinteractiongain) {
            Fh = Fh_;   
            Fh_local = Fh_local_;     
        }
        else {
            Fh.setZero();
            Fh_local.setZero();                      
        }
    }

    void FrankaHuskyWrapper::ee_state(Vector3d & pos, Eigen::Quaterniond & quat){
        for (int i=0; i<3; i++)
            pos(i) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation()(i);

        Quaternion<double> q(robot_->position(data_, robot_->model().getJointId("panda_joint7")).rotation());
        quat = q;
    }

    void FrankaHuskyWrapper::base_state(Vector3d & base){
        base(0) = robot_->getMobilePosition(data_, 5).translation()(0); //5 means state.q_ at panda joint 1 
        base(1) = robot_->getMobilePosition(data_, 5).translation()(1); //5 means state.q_ at panda joint 1 
        base(2) = atan2(-robot_->getMobilePosition(data_, 5).rotation()(0, 1),robot_->getMobilePosition(data_, 5).rotation()(0,0)); //5 means state.q_ at panda joint 1 
    }

    void FrankaHuskyWrapper::rotx(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << 1,           0,           0;
        rot.row(1) << 0,           cos(angle), -sin(angle);
        rot.row(2) << 0,           sin(angle),  cos(angle);
    }

    void FrankaHuskyWrapper::roty(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) <<  cos(angle),           0,            sin(angle);
        rot.row(1) <<  0,                    1,            0 ;
        rot.row(2) << -sin(angle),           0,            cos(angle);
    }

    void FrankaHuskyWrapper::rotz(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << cos(angle), -sin(angle),  0;
        rot.row(1) << sin(angle),  cos(angle),  0;
        rot.row(2) << 0,                    0,  1;
    }    
    
    MatrixXd FrankaHuskyWrapper::skew_matrix(const VectorXd& vec){
        double v1 = vec(0);
        double v2 = vec(1);
        double v3 = vec(2);

        Eigen::Matrix3d CM;
        CM <<     0, -1*v3,    v2,
                 v3,     0, -1*v1,
              -1*v2,    v1,     0;

        return CM;
    }

    pinocchio::SE3 FrankaHuskyWrapper::vel_to_SE3(VectorXd vel, double dt){        
        Quaterniond angvel_quat(1, vel(3) * dt * 0.5, vel(4) * dt * 0.5, vel(5) * dt * 0.5);        
        Matrix3d Rotm = angvel_quat.normalized().toRotationMatrix();                
        
        pinocchio::SE3 T;
        T.translation() = vel.head(3) * dt;
        T.rotation() = Rotm;

        // cout << T << endl;

        return T;
    }

    void FrankaHuskyWrapper::get_dt(double dt){
        dt_ = dt;
    }

    double FrankaHuskyWrapper::trajectory_length_in_time(){        
        return  traj_length_in_time_;           
    }

    double FrankaHuskyWrapper::noise_elimination(double x, double limit) {
        double y;
        if (abs(x) > limit) y = x;
        else y = 0.0;
        
        return y;
    }
}// namespace
