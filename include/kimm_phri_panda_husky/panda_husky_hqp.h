//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// for hqp controller
#include <kimm_hqp_controller/robot/robot_wrapper.hpp>
#include <kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp>
#include <kimm_hqp_controller/tasks/task_se3_equality.hpp>
#include <kimm_hqp_controller/tasks/task_joint_posture.hpp>
#include <kimm_hqp_controller/tasks/task_joint_bound.hpp>
#include <kimm_hqp_controller/tasks/task_mobile_base.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_euclidian.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_se3.hpp>
#include <kimm_hqp_controller/solver/solver_HQP_factory.hxx>
#include <kimm_hqp_controller/solver/util.hpp>
#include <kimm_hqp_controller/math/util.hpp>

// service 
#include "kimm_joint_planner_ros_interface/action_joint_path.h"
#include "kimm_se3_planner_ros_interface/action_se3_path.h"
#include "kimm_path_planner_ros_interface/action_mobile_path.h"
#include "kimm_joint_planner_ros_interface/JointAction.h"
#include "kimm_se3_planner_ros_interface/SE3Action.h"
#include "kimm_path_planner_ros_interface/MobileTrajectory.h"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;

typedef struct State {   
    VectorXd q_;
    VectorXd v_;
    VectorXd dv_;
    VectorXd torque_;
} state;   
typedef struct Mob{
    MatrixXd lambda_inv_;
    MatrixXd lambda_, mass_, mass_inv_;
    MatrixXd J_trans_;
    MatrixXd J_trans_inv_;
    VectorXd d_force_;
    VectorXd d_torque_, torque_d_, torque_d_prev_;
    VectorXd g_, nle_, coriolis_;
    double gamma_, beta_;
    VectorXd p_k_prev_, p_k_, alpha_k_;
} mob;
typedef struct Joint_Action{
    VectorXd kp_, kd_, q_target_;
    int type_;
    double duration_;
    bool is_succeed_{true};

} jointaction;
typedef struct Mobile_Action{
    std::vector<geometry_msgs::Pose2D> path_;
    bool is_succeed_{true};
} mobileaction;
typedef struct SE3_Action{
    VectorXd kp_, kd_;
    pinocchio::SE3 se3_target_;
    int type_;
    double duration_;
    bool is_wholebody_{true};
    bool is_succeed_{true};

} se3action;

namespace RobotController{
    class HuskyFrankaWrapper{
        public: 
            HuskyFrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node);
            ~HuskyFrankaWrapper(){};

            void initialize();
            void ctrl_update(const int& ); // msg for chaning controller
            void franka_update(const sensor_msgs::JointState&); // franka state update
            void franka_update(const Vector7d&, const Vector7d&); // franka state update

            void husky_update(const sensor_msgs::JointState&); // husky state update
            void husky_update(const Vector3d&, const Vector3d&, const Vector2d&, const Vector2d&); // husky state update

            void compute(const double &); // computation by hqp controller
            void franka_output(VectorXd & qacc); // joint torque of franka 
            void husky_output(VectorXd & qvel); // joint velocity of husky

            void position(pinocchio::SE3 & oMi);
            void com(Eigen::Vector3d& com);
            void velocity(pinocchio::Motion& vel);
            void velocity_origin(pinocchio::Motion& vel);
            void velocity_global(pinocchio::Motion& vel);
            void acceleration(pinocchio::Motion& accel);
            void acceleration_origin(pinocchio::Motion & accel);
            void acceleration_global(pinocchio::Motion & accel);
            void force(pinocchio::Force & force);
            void force_origin(pinocchio::Force & force);
            void force_global(pinocchio::Force & force);
            void tau(VectorXd & tau_vec);
            void ddq(VectorXd & ddq_vec);

            void mass(MatrixXd & mass_mat);
            void nle(VectorXd & nle_vec);
            void JWorld(MatrixXd & J);
            void JLocal_offset(MatrixXd & Jo);
            void dJLocal_offset(MatrixXd & dJo);
            void g(VectorXd & g_vec);
            void g_joint7(VectorXd & g_vec);

            void ee_state(Vector3d & pos, Eigen::Quaterniond & quat);
            void base_state(Vector3d & base);

            void rotx(double & a, Eigen::Matrix3d & rot);
            void roty(double & a, Eigen::Matrix3d & rot);
            void rotz(double & a, Eigen::Matrix3d & rot);

            int ctrltype(){
                return ctrl_mode_;
            }
            void state(State & state_robot){
                state_robot = state_;
            }
            bool reset_control_;

            void jointActionCallback(const kimm_joint_planner_ros_interface::JointActionConstPtr &msg){
                ROS_WARN("jointActionRecieved");
                joint_action_.kp_.setZero(7);
                joint_action_.kd_.setZero(7);
                joint_action_.q_target_.setZero(7);
                joint_action_.duration_ = msg->duration;

                for (int i=0; i<na_-2; i++){
                        joint_action_.kp_(i) = msg->kp[i];
                        joint_action_.kd_(i) = msg->kv[i];
                        joint_action_.q_target_(i) = msg->target_joint[0].position[i];
                }
                
                joint_action_.type_ = msg->traj_type;
                joint_action_.is_succeed_ = false;
            };
            void se3ActionCallback(const kimm_se3_planner_ros_interface::SE3ActionConstPtr &msg){
                ROS_WARN("SE3ActionRecieved");
                se3_action_.kp_.setZero(6);
                se3_action_.kd_.setZero(6);
                
                se3_action_.se3_target_.translation()(0) = msg->target_se3[0].translation.x;
                se3_action_.se3_target_.translation()(1) = msg->target_se3[0].translation.y;
                se3_action_.se3_target_.translation()(2) = msg->target_se3[0].translation.z;
                Eigen::Quaterniond quat;
                quat.x() = msg->target_se3[0].rotation.x;
                quat.y() = msg->target_se3[0].rotation.y;
                quat.z() = msg->target_se3[0].rotation.z;
                quat.w() = msg->target_se3[0].rotation.w;
                quat.normalize();
                se3_action_.se3_target_.rotation() = quat.toRotationMatrix();
                se3_action_.duration_ = msg->duration;

                for (int i=0; i<6; i++){
                    se3_action_.kp_(i) = msg->kp[i];
                    se3_action_.kd_(i) = msg->kv[i];
                }
                
                se3_action_.type_ = msg->traj_type;
                se3_action_.is_wholebody_ = msg->iswholebody.data;
                se3_action_.is_succeed_ = false;
            };
            void mobileActionCallback(const kimm_path_planner_ros_interface::MobileTrajectoryConstPtr &msg){
                ROS_WARN("MobileActionRecieved");
                ROS_WARN("%ld", msg->points.size());
                mobile_action_.path_ = msg->points;
                mobile_action_.is_succeed_ = false;
            };

        private:
            bool issimulation_, mode_change_, update_weight_, planner_res_;
            double stime_, time_, node_index_, node_num_, prev_node_;
            std::string robot_node_;
            State state_;
            Joint_Action joint_action_;
            SE3_Action se3_action_;
            Mobile_Action mobile_action_;

            ros::Subscriber joint_action_subs_, se3_action_subs_, mobile_action_subs_;
        
            int ctrl_mode_;
            Eigen::VectorXd q_ref_;
            pinocchio::SE3 H_ee_ref_, H_mobile_ref_;
            Vector3d ee_offset_;
            MatrixXd Adj_mat;
            double est_time;

            //hqp
            std::shared_ptr<kimmhqp::robot::RobotWrapper> robot_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::shared_ptr<kimmhqp::InverseDynamicsFormulationAccForce> tsid_;
            
            std::shared_ptr<kimmhqp::tasks::TaskJointPosture> postureTask_;
            std::shared_ptr<kimmhqp::tasks::TaskSE3Equality> eeTask_;
            std::shared_ptr<kimmhqp::tasks::TaskJointBounds> torqueBoundsTask_;
            std::shared_ptr<kimmhqp::tasks::TaskMobileEquality> mobileTask_, mobileTask2_;

            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianCubic> trajPosture_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianConstant> trajPosture_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianTimeopt> trajPosture_Timeopt_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Cubic> trajEE_Cubic_, trajMobile_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Constant> trajEE_Constant_, trajMobile_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Timeopt> trajEE_Timeopt_, trajMobile_Timeopt_;            

            kimmhqp::trajectory::TrajectorySample sampleMobile_, sampleEE_, samplePosture_;

            kimmhqp::solver::SolverHQPBase * solver_;
            
            int na_, nq_, nv_;
            int cnt_;

            //service
            //kimm_path_planner_ros_interface::action_mobile_path action_mobile_srv_; 
            // kimm_joint_planner_ros_interface::action_joint_path action_joint_srv_; 
            // kimm_se3_planner_ros_interface::action_se3_path action_se3_srv_; 
            ros::ServiceClient joint_action_client_, se3_action_client_, mobile_action_client_;

            //ros
            ros::NodeHandle n_node_;
    };
} // namespace


