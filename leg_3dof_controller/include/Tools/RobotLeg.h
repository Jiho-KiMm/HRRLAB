#ifndef ROBOTLEG_H_
#define ROBOTLEG_H_

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cmath>
#include "Eigen/Dense"
#include <functional>

#include "NumericalTool.h"
#include "stquad_enum.h"

class RobotLeg
{
    NumericalTool::Calculus pd_error_xdot[DoF];

public:
    enum Axis
    {
        X = 0,
        Y = 1,
        Z = 2,
        NUM_AXIS = 3
    };

    RobotLeg(){};
    virtual ~RobotLeg(){};

    void init(const double freq);

    //경환이형 코드
    void FK(); 
    void CalcJacobian();
    void IK();
    void SetDesJointXyz(double q1_d, double q2_d, double q3_d);
    void SetDesJointXyzDot(double q1_dot_d, double q2_dot_d, double q3_dot_d);
    void SetDesXyz(double x_d,double y_d,double z_d);
    void SetDesXyzDot(double x_dot_d,double y_dot_d,double z_dot_d);
    void SetJointAngle(double q1, double q2, double q3);
    void TaskSpaceCTC(Eigen::Matrix3d M, Eigen::Matrix3d C, Eigen::Vector3d G);

    //Task space
    Eigen::Matrix<double, 3, 1> &GetJointAngle() { return q_d_; };
    // Eigen::Matrix<double, 3, 1> &GetJacobian() { return J; };
    Eigen::Matrix<double, 3, 1> &GetEEPos() { return xyz_; };
    Eigen::Matrix<double, 3, 1> &GetEEVel() { return xyz_dot_; };
    Eigen::Matrix<double, 3, 1> &GetEEAcc() { return xyz_ddot_; };
    Eigen::Matrix<double, 3, 1> &GetDesEEPos() { return xyz_d_; };
    Eigen::Matrix<double, 3, 1> &GetDesEEVel() { return xyz_dot_d_; };
    Eigen::Matrix<double, 3, 1> &GetDesEEAcc() { return xyz_ddot_d_; };

    //...............................

    void SetLegInit(Eigen::VectorXd init_q);
    Eigen::VectorXd GetInitQ() { return init_q_; }

    void SetPDControl_q(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_q, Eigen::VectorXd act_q, Eigen::VectorXd ref_qdot, Eigen::VectorXd act_qdot);

    void SetPDControl_X(Eigen::MatrixXd kp, Eigen::MatrixXd kd, Eigen::VectorXd ref_x, Eigen::VectorXd act_x, Eigen::MatrixXd J);
    // void FloatingBaseSetPDControl_X(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_x, Eigen::VectorXd act_x, Eigen::MatrixXd J);

    Eigen::VectorXd GetJointTorque() { return torque_; }

    bool isLegInit;
    

    
private:
    // void init();

    int init_size_;
    int legtype_;
    double dt_;

    // 경환이형 코드
    Eigen::VectorXd init_q_;
    Eigen::VectorXd kp_, kd_, ref_q_, act_q_, ref_qdot_, act_qdot_, torque_;
    Eigen::VectorXd error_x_, error_xdot_, act_x_vel;
    Eigen::VectorXd ref_xdot_, act_xdot_;
    Eigen::VectorXd torque_x_;
    Eigen::MatrixXd J_;
    Eigen::Matrix<double, NUM_AXIS, NUM_JOINTS> J, J_dot, J_inv, J_trans, J_trans_inv; 
    NumericalTool::Calculus J_calculus_[NUM_AXIS * NUM_JOINTS];

    Eigen::Matrix4d ht_ee_wrt_0_;
    Eigen::Matrix3d rotation_ee_wrt_0_, rotation_0_wrt_ee_;

    Eigen::Matrix3d kp_T, kd_T;

    double ankle_angle_;
    double gam_;
    double l1_, l2_, l3_, l4_, l5_;
    //End Effector(position,velocity,acceleration...)
    Eigen::Matrix<double, NUM_AXIS, 1> xyz_, xyz_dot_, xyz_ddot_, xyz_ddot_ctc_;
    Eigen::Matrix<double, NUM_AXIS, 1> xyz_d_, xyz_dot_d_, xyz_ddot_d_; //desired
    Eigen::Matrix<double, NUM_AXIS, 1> tor_M_, tor_G_, tor_C_, tor_ctc_;

    Eigen::Matrix<double, DoF, 1> q_,q_dot_, q_ddot_;
    Eigen::Matrix<double, DoF,1> q_d_, q_dot_d_, q_ddot_d_;

    Eigen::Vector3d q_ddot_ctc_;


    //.............................
private:
};

#endif