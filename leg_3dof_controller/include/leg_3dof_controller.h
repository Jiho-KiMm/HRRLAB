#ifndef LEG_3DOF_CONTROLLER_H_
#define LEG_3DOF_CONTROLLER_H_

#include "PinocchioInterface.hpp"

#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include "Eigen/Dense"
#include <cmath>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "RobotLeg.h"
#include "TrajectoryGenerator.h"
#include "rqt_plot.h"
#include "stquad_enum.h"

// std::string urdf_path_ = "/home/jiho/example_ws/src/robots/robot_description/leg_3dof/urdf/leg_3dof.urdf";

RobotLeg robotleg[Leg_N];
TrajectoryGenerator traj[Leg_N];

std::string urdf_path_ = "/home/jiho/example_ws/src/robots/robot_description/leg_3dof/urdf/leg_3dof.urdf";

std::vector<std::string> foot_name_ = {"EE"};

// Select fixed_base or floating_base
std::string base_ = "fixed_base";
//...................

PinocchioInterface pin(urdf_path_, foot_name_, base_);

class leg_3dof_controller
{
private:
    // const std::string urdf_path_;
    // const std::vector<std::string> foot_name_;

public:
    leg_3dof_controller(ros::NodeHandle &nh,
                      std::string topic_leg_state,
                      std::string topic_leg_command,
                    //   std::string urdf_path,
                    //   std::vector<std::string> foot_name,
                      const double freq)
        : nh_(nh),
          topic_leg_state_(topic_leg_state),
          topic_leg_command_(topic_leg_command),
        //   urdf_path_(urdf_path),
        //   foot_name_(foot_name),
          freq_(freq),
          pi(M_PI),
          deg2rad(pi / 180),
          rad2deg(180 / pi)
    {}
    virtual ~leg_3dof_controller() {}

    void init();
    void run();


private:
    // Function
    void state_leg_callback(const sensor_msgs::JointState &state);
   
    
    void send_commands_to_robot();
    void init_rqt();
    void rqt_plot();


    void ChangeControlState();
    

    void command(bool flag);
    void HomingControl();
    void SimpleTaskSpacePDControl();
    void SimpleTaskSpaceCTCControl();

    void FloatingBasecommand(bool flag);

    void TaskSpaceMoving(double duration, Eigen::Vector3d initial_x, Eigen::Vector3d error_x, Eigen::Vector3d ref_x_0, Eigen::Matrix3d _Kp_, Eigen::Matrix3d _Kd_, const std::string& control_name);
    void ChangeFlag();
    void ChangeFlag2();

    // Variable
    ros::NodeHandle nh_;
    const double pi, deg2rad, rad2deg, freq_;

    const std::string topic_leg_state_, topic_leg_command_;

    //  Subscribers:
    ros::Subscriber sub_leg_state_;
    Eigen::VectorXd total_leg_q_, total_leg_dq_;

    //  Publisher:
    ros::Publisher pub_leg_cmd_;


    // Robot Parameter:
    int init_size;
    Eigen::VectorXd q_, dq_, torque_, fq_, fdq_, floating_q_, floating_dq_;
    Eigen::Matrix<double, Leg_N, 1> rms_torque;
    double tmp_Kp, tmp_Kd, tmp_rKp, tmp_rKd, homing_period;
    Eigen::VectorXd homing_ref_q, homing_Kp, homing_Kd;
    // double pi, rad2deg, deg2rad;
    Eigen::Matrix<double, Leg_N, DoF> _acc_d, _vel_d, _pos_d;
    ControlMode controlmode;
    TaskSpace taskspace;

    bool Recieved_Joint_State;
    bool Recieved_Mode[10];

    //jiho--------------------
    double t_, sub_t_;
    bool flag_;
    bool flag_2;
    Eigen::Vector3d q_0, q_f, q_d, q_f_dot, EEPos, x_, x_0_ref , x_0, x_dot;
    Eigen::VectorXd generalized_gravity, ref_x, ref_x2, top_ref_x;
    Eigen::MatrixXd J_;

    Eigen::Vector3d reference_x;
    Eigen::Vector3d reference_x_0;
    Eigen::Vector3d error_x_;
    int count_;
    double Jacodatas[60000][3];
    double datas[60000][3];
    int N;

    Eigen::Matrix3d ctc_kp, ctc_kd, I, kp, kd;

    const int BUFFER_SIZE = 10;
    std::queue<double> sQueue;
    std::mutex sQueueLock;
    std::condition_variable sEvent;

    //.......................
    ros::NodeHandle nh_plot;

protected:
};

#endif