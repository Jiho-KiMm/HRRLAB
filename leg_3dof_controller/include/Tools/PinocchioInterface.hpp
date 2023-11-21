#ifndef PINOCCHIOINTERFACE_H_
#define PINOCCHIOINTERFACE_H_

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include "stquad_enum.h"
#include "NumericalTool.h"

class PinocchioInterface
{
private:
    pinocchio::Model _model;
    pinocchio::Data _data;
    pinocchio::FrameIndex frame_id;
    int n;
    NumericalTool::Calculus vel_diff[Leg_N][DoF];
    NumericalTool::Calculus vel_diff_floating[Leg_N][DoF + 6];
    int check;


public:

    PinocchioInterface(const std::string urdf_file, const std::vector<std::string> foot_name, const std::string& base_state) : urdf_file_(urdf_file), foot_name_(foot_name)
    {


        if(base_state == "floating_base")
        {
            createFloatingBaseModel(urdf_file_);
            check = 0;   
        }
        else if(base_state == "fixed_base")
        {
            pinocchio::urdf::buildModel(urdf_file_,_model);
            check = 1;
        }
        else
        {
            check = 2;
        }

        
        pinocchio::Data data(_model);
        _data = data;
        Initialize();
    };
    virtual ~PinocchioInterface(){};

public:
    void Initialize();
    void SetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq);
    void SetKinematics(int legtype);
    void SetJacobian(int legtype);
    void createFloatingBaseModel(const std::string& urdfFilePath);

    void FloatingBaseSetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq);
    void FloatingBaseSetKinematics(int legtype);
    void FloatingBaseSetJacobian(int legtype);

    void PrintPin();

    void SetGravity();
    void SetDynamics(Eigen::Vector3d pos_d, Eigen::Vector3d vel_d, Eigen::Vector3d acc_d, Eigen::Matrix3d Kp, Eigen::Matrix3d Kd);
    // void SetDynamics(Eigen::MatrixXd pos_d, Eigen::MatrixXd vel_d, Eigen::MatrixXd acc_d, Eigen::Matrix3d Kp, Eigen::Matrix3d Kd);

    Eigen::Vector3d GetPos(int legtype) { return pos_[legtype]; }
    Eigen::Vector3d GetVel(int legtype) { return vel_[legtype]; }
    Eigen::Vector3d GetAcc(int legtype) { return acc_[legtype]; }
    Eigen::Matrix3d GetJacobian(int legtype) { return J_[legtype]; }
    Eigen::Matrix3d GetJacobianDot(int legtype) { return dJ_[legtype]; }


    // Eigen::MatrixXd GetFloatingBaseJacobian(int legtype) { return J_floating[0]; }
    Eigen::VectorXd GetFloatingBaseGravityCompensation() { return r_G_.block<3, 1>(6, 0); }

    Eigen::VectorXd GetDynamics();
    Eigen::VectorXd GetNLEffects() { return r_B_; }
    Eigen::VectorXd GetGravityCompensation() { return r_G_; }

private:
    // Variables for Pinocchio
    std::string urdf_file_;
    std::vector<std::string> foot_name_;
    Eigen::VectorXd _q, _dq, _ddq;
    Eigen::Vector3d _pos;
    Eigen::MatrixXd _J, _dJ;

    // Actual Angle, Angular Velocity from Model Joint States
    Eigen::Matrix<double, DoF, 1> q_[Leg_N], dq_[Leg_N], ddq_[Leg_N];

    // Actual Foot Position, Veloicy
    Eigen::Matrix<double, DoF, 1> pos_[Leg_N], vel_[Leg_N], acc_[Leg_N];

    // Jacobian
    Eigen::Matrix<double, DoF, DoF> J_[Leg_N], dJ_[Leg_N];

    // Recursive-Newon Euler
    Eigen::Matrix<double, DoF, DoF> M_[Leg_N], C_[Leg_N];
    Eigen::Matrix<double, DoF, 1> G_[Leg_N], B_[Leg_N], T_[Leg_N];

    // Desired Pos, Vel, Acc
    Eigen::Matrix<double, DoF, 1> pos_d_[Leg_N], vel_d_[Leg_N], acc_d_[Leg_N];

    // Error between Des & Act [Position, Velocity]
    Eigen::Matrix<double, DoF, 1> err_[Leg_N], err_dot_[Leg_N];

    // Control Gain
    Eigen::Matrix3d Kp_, Kd_;
 
    // Caculate Torque
    Eigen::VectorXd target_torque_;

    // return
    Eigen::Vector3d r_pos_, r_vel_, r_acc_;
    Eigen::Matrix4d r_J_, r_dJ_;
    Eigen::MatrixXd r_M_, r_C_;
    Eigen::VectorXd r_B_, r_G_, r_T_;
};

#endif
