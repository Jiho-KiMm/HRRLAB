/*
 *  RobotLeg.h
 *  Ver :2.0
 *  Created on: 2022. 01. 16.
 *  Author: KimKyungHwan(kkh9764@naver.com) , HRRLAB , SeoulTech
 * 
 *  unit: radian, meter[m], second[s]
 */

#ifndef ROBOTLEG_H_
#define ROBOTLEG_H_

#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "TrajectoryGenerator_V2.h"
#include "NumericalTool.h"

class RobotLeg
{
public:
    enum Axis
    {
        X = 0,
        Y = 1,
        Z = 2,
        NUM_COOR = 3
    };
    enum JointAxis
    {
        HR = 0,
        HP = 1,
        KP = 2,
        NUM_AXIS = 3
    };
    enum ControlMode
    {
        POSITION_CONTROL = 0,
        FORCE_CONTROL = 1,
        HYBRID_CONTROL = 2,
        POSITION_CONTROL_GROUND = 3,
        HYBRID_CONTROL_GROUND = 4,
        LOW_GAIN_POSITION_CONTROL = 8,
        JOINT_CONTROL = 10
    };
    enum EndEffectorState
    {
        STANCE = 0,
        SWING = 1
    };
    enum LegType
    {
        FL = 0, //fore left
        FR = 1, //fore right
        BL = 2, //back left
        BR = 3  //back rigth
    };

public:
    RobotLeg(int leg_type = FL);
    virtual ~RobotLeg();

    void InitializeJointAngle(const double (&actual_joint_angle)[NUM_AXIS], const double (&auxiliary_joint_angle)[NUM_AXIS]);  //if there is a defference between real pos and initial pos
    void SetJointAngle(const double (&actual_joint_angle)[NUM_AXIS], const double (&actual_joint_angular_velocity)[NUM_AXIS]); //auto calc acceleration
    void SetJointAngle(const double (&actual_joint_angle)[NUM_AXIS]);                                                          //auto calc velocity,acceleration
    void SetEndEffectorTrajectory(TrajectoryGenerator &trajectory_3D);
    void SetJointTrajectory(TrajectoryGenerator &trajectory, int axis);

    void SetImuSensor(double (&IMU)[3]);
    void SetForceSensor(double (&force)[3]);
    void SetGroundForceSensor(double (&force)[3]);
    void SetDepthCamera(double (&point)[3], double (&normal)[3]);

    void Joint_space_PD_control();
    void Task_space_CTC();
    void Joint_space_CTC();
    void GravityCompensation();
    void FrictionCompensation();
    void SimpleJointControl();
    void SimpleReciprocating(int _axis = KP);
    void Force_control(double Fx, double Fy, double Fz);
    //        void ChangeControlCoordinate();
    //        void CalcRotate_EE_wrt0();
    //        void CalcRotate_G_wrt0();
    //        void CalcRotate_EE_wrtG();//3x3
    //        void CalcTrans_G_wrt0();//3x1
    //        void CalcHT_G_wrt0();//Homogeneous Transformation 4x4
    //        void CalcHT_0_wrtG();
    //

    void SetControlMode(int controlmode) { control_mode_ = controlmode; }
    void ChangePointWrtG(double *);
    //        void ChangePointWrt0(double (&point)[NUM_COOR]);
    void SetDesForce(double dF, double Vd = -0.3)
    {
        exert_force_z_d_ = dF;
        Vd_force_control_ = Vd;
    }
    void SetControlOption(int option) { control_option_ = option; }
    int time() { return time_ms_; }
    void incre_time() { time_ms_++; }

    void Camera_shoot()
    {
        //                camera_fix=true;
        //                for (int i = 0; i < NUM_COOR; i++){
        //                        fixed_point_camera_[i]=point_camera_[i];
        //                        fixed_normal_camera_[i]=normal_camera_[i];
        //                        if(stop_flag==0&&(fabs(fixed_point_camera_[0])>0.1||fabs(fixed_point_camera_[1])>0.1||fabs(fixed_point_camera_[2])>0.6)&&camera_fix==true){
        //                                                stop_flag=2;
        //                                                printf("Point ground_yaw_error_");
        //                        }
        //                }
    }
    void Camera_free() { camera_fix = false; }

    Eigen::VectorXd GetPos() { return q_; }
    Eigen::VectorXd GetVel() { return q_dot_; }
    Eigen::VectorXd GetAcc() { return q_ddot_; }
    Eigen::VectorXd GetDesPos() { return q_d_; }
    Eigen::VectorXd GetDesVel() { return q_dot_d_; }
    Eigen::VectorXd GetDesAcc() { return q_ddot_d_; }
    Eigen::VectorXd GetTargetTorque() { return target_torque_; }
    Eigen::VectorXd GetEEPos() { return xyz_; }
    Eigen::VectorXd GetEEVel() { return xyz_dot_; }
    Eigen::VectorXd GetEEAcc() { return xyz_ddot_; }
    Eigen::VectorXd GetDesEEPos() { return xyz_d_; }
    Eigen::VectorXd GetDesEEVel() { return xyz_dot_d_; }
    Eigen::VectorXd GetDesEEAcc() { return xyz_ddot_d_; }
    Eigen::VectorXd GetTorM() { return tor_M_; }
    Eigen::VectorXd GetTorC() { return tor_C_; }
    Eigen::VectorXd GetTorG() { return tor_G_; }
    Eigen::VectorXd Getmomentum() { return momentum_; }
    Eigen::VectorXd Getforcehat() { return contact_force_hat_; }
    Eigen::VectorXd GetTorCOL() { return tor_coulomb_; }
    Eigen::VectorXd GetTorVIS() { return tor_viscous_; }
    Eigen::VectorXd GetTorTracking() { return tor_tracking_; }
    int IsHoming() { return homing_flag_; }
    int GetControlMode() { return control_mode_; }
    double GetDesForce() { return exert_force_z_d_; }
    double GetTargetTorque(int axis) { return target_torque_(axis); }
    Eigen::VectorXd GetForce() { return force_sensor_; }
    Eigen::VectorXd GetImu() { return imu_; }
    Eigen::VectorXd GetEEPosG() { return xyz_wrt_ground_; }
    bool IsInitialtize() { return is_initialize; }

private:
    const int leg_type_;
    const double kEpsilon;
    int time_ms_; //Millisecond

    //Robot Parameters
    double m1_, m2_, m3_;           //Mass of Link
    double l1_, l2_, l3_, l4_, l5_; //Length of Link
    double lc1_, lc2_, lc3_;        //Length of Link mass center(w.r.t each joint)
    double r1_, r2_, r3_;           //The ratio between l and lc
    double gam_;
    double g_; //Gravity acceleration 9.8067
    double root2_;
    double root2_inv_;                       //1/sqrt(2)
    double d1_, d2_, d3_, ground_yaw_error_; //ground tester parameters

    //joint parameters
    Eigen::Matrix<double, NUM_AXIS,1> q_, q_dot_, q_ddot_;       //Actual Joint Angle
    Eigen::Matrix<double, NUM_AXIS,1> q_d_, q_dot_d_, q_ddot_d_; //desired Joint Angle
    Eigen::Matrix<double, NUM_AXIS,1> q_ddot_ctc_;
    double initial_joint_angle_compensation_[NUM_AXIS];

    //Dyanmics
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> M; //Inertia Matrix
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> C; //Coriolis and Centrifugal matrix
    Eigen::Matrix<double, 6, 6> F;
    Eigen::Matrix<double, 6, 6> f;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 3> r;
    Eigen::Matrix<double, 3, 6> H;
    Eigen::Matrix<double, 3, 6> h;
    Eigen::Matrix<double, 6, 3> K;
    Eigen::Matrix<double, 6, 3> k;
    Eigen::Matrix<double, 6, 6> P;
    Eigen::Matrix<double, 6, 6> I_N;
    double c111, c211, c311;
    double c121, c221, c321;
    double c131, c231, c331;
    double c112, c212, c312;
    double c122, c222, c322;
    double c132, c232, c332;
    double c113, c213, c313;
    double c123, c223, c323;
    double c133, c233, c333;
    Eigen::Matrix<double, NUM_AXIS, 1> G;                                            // Gravity Matrix
    Eigen::Matrix<double, NUM_COOR, NUM_AXIS> J, J_dot, J_inv, J_trans, J_trans_inv; //Linear Velocity Jacobian
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Me;                                    //Task-space Inertia Matrix
    double ic1xx, ic1xy, ic1xz, ic1yx, ic1yy, ic1yz, ic1zx, ic1zy, ic1zz;            //Moment of inertia
    double ic2xx, ic2xy, ic2xz, ic2yx, ic2yy, ic2yz, ic2zx, ic2zy, ic2zz;
    double ic3xx, ic3xy, ic3xz, ic3yx, ic3yy, ic3yz, ic3zx, ic3zy, ic3zz;

    //End Effector(position,velocity,acceleration...)
    Eigen::Matrix<double, NUM_COOR, 1> xyz_, xyz_dot_, xyz_ddot_, xyz_ddot_ctc_;
    Eigen::Matrix<double, NUM_COOR, 1> xyz_d_, xyz_dot_d_, xyz_ddot_d_; //desired

    //End Effector w.r.t ground
    Eigen::Matrix<double, NUM_COOR, 1> xyz_wrt_ground_, xyz_dot_wrt_ground_, xyz_ddot_wrt_ground_, xyz_ddot_ctc_wrt_ground_;
    Eigen::Matrix<double, NUM_COOR, 1> xyz_d_wrt_ground_, xyz_dot_d_wrt_ground_, xyz_ddot_d_wrt_ground_;

    //Joint Torque
    Eigen::Matrix<double, NUM_AXIS,1> tor_M_, tor_G_, tor_C_, tor_ctc_;
    Eigen::Matrix<double, NUM_AXIS,1> tor_tracking_;
    Eigen::Matrix<double, NUM_AXIS,1> tor_coulomb_;
    Eigen::Matrix<double, NUM_AXIS,1> tau_bar;
    Eigen::Matrix<double, NUM_AXIS,1> tor_viscous_;
    Eigen::Matrix<double, NUM_AXIS,1> tor_exert_force_;
    Eigen::Matrix<double, NUM_AXIS,1> momentum_;

    //Other sensors(ground IMU, ground FT sensor, foot force sensor...)
    bool is_ground_IMU_enable_;
    bool is_detph_camera_enable_;
    bool is_force_sensor_enable_;
    bool is_ground_force_sensor_enable_;
    double ground_roll_, ground_pitch_, ground_yaw_;
    Eigen::Matrix<double, NUM_COOR, 1> imu_;
    Eigen::Matrix<double, NUM_COOR, 1> force_sensor_, force_sensor_dot_;
    Eigen::Matrix<double, NUM_COOR, 1> force_sensor_wrt_0_, force_sensor_dot_wrt_0_; //=wrt shoulder
    Eigen::Matrix<double, NUM_COOR, 1> ground_force_sensor_, ground_force_sensor_dot_;
    Eigen::Matrix<double, NUM_COOR, 1> ground_force_sensor_wrt_0_, ground_force_sensor_dot_wrt_0_; //=wrt shoulder
    int end_effector_state_;

    //control setting
    int control_mode_;
    int control_option_;
    int homing_flag_;

    double exert_force_z_d_;
    double force_error_, force_error_dot_, force_error_inte_;
    Eigen::Matrix<double, NUM_COOR, 1> exert_force_d_wrt_0_; //=wrt shoulder

    //depth camera
    double pitchCG_;
    double rollCG_;
    bool camera_fix;
    double point_camera_[NUM_COOR];
    double normal_camera_[NUM_COOR];
    double fixed_point_camera_[NUM_COOR];
    double fixed_normal_camera_[NUM_COOR];

    //computed torque
    Eigen::Matrix<double, NUM_AXIS,1> target_torque_;
    Eigen::Matrix<double, 6,1> x_hat_;
    Eigen::Matrix<double, 6,1> x_hat_pri_;
    Eigen::Matrix<double, 6,1> x_hat_pos_;
    Eigen::Matrix<double, 3,1> contact_tor_hat_;
    Eigen::Matrix<double, 3,1> contact_force_hat_;

    TrajectoryGenerator trajectory_;

    //numerical calculus or filter
    NumericalTool::Calculus q_calculus_[NUM_AXIS];
    NumericalTool::Calculus q_dot_calculus_[NUM_AXIS];
    NumericalTool::LowPassFilter q_dot_LPF_[NUM_AXIS];
    NumericalTool::LowPassFilter q_ddot_LPF_[NUM_AXIS];
    NumericalTool::Calculus force_calculus_[NUM_AXIS];
    NumericalTool::SimpleMovingAverage force_SMA_[NUM_AXIS];
    NumericalTool::LowPassFilter force_LPF_[NUM_AXIS];
    NumericalTool::LowPassFilter force_dot_LPF_[NUM_AXIS];
    NumericalTool::Calculus J_calculus_[NUM_COOR * NUM_AXIS];

    //Homogeneous Transformation
    Eigen::Matrix3d rotation_ee_wrt_0_, rotation_0_wrt_ee_;
    Eigen::Matrix4d ht_ee_wrt_0_;
    Eigen::Matrix3d rotation_ground_wrt_0_, rotation_0_wrt_ground_;
    Eigen::Vector3d translation_ground_wrt_0_;
    Eigen::Matrix4d ht_ground_wrt_0_, ht_0_wrt_ground_;

    Eigen::Matrix3d rotation_ground_wrt_camera_;
    Eigen::Matrix3d rotation_camera_wrt_ground_;

    //Variable for research
    double Vd_force_control_;
    bool is_initialize;

    //Private function
    void XyzToXyzGround();
    void XyzdGroundToXyzd();
    void FK();
    void IK();
    void CalcDynamics();
    void Calckalman();
    void CalcJacobian();
    void CalcEEState();
    int sgn(double val) { return (kEpsilon < val) - (val < -kEpsilon); }
    int sgnGap(double val, double gap) { return (gap < val) - (val < -gap); }
    void CalcHT();
};

RobotLeg::RobotLeg(int leg_type) : kEpsilon(0.0000001), leg_type_(leg_type)
{
    time_ms_ = 0;
    control_mode_ = POSITION_CONTROL; //defaul control mode
    control_option_ = 0;
    end_effector_state_ = SWING;
    is_initialize = false;
    for (int i = 0; i < NUM_AXIS; i++)
    {
        initial_joint_angle_compensation_[i] = 0.;
        q_(i) = 0.;
        q_dot_(i) = 0.;
        q_ddot_(i) = 0.;
        q_d_(i) = 0.;
        q_dot_d_(i) = 0.;
        q_ddot_d_(i) = 0.;
        xyz_(i) = 0;
        xyz_dot_(i) = 0;
        xyz_ddot_(i) = 0;
        xyz_d_(i) = 0;
        xyz_dot_d_(i) = 0;
        xyz_ddot_d_(i) = 0;
        target_torque_(i) = 0;
        contact_tor_hat_(i) = 0;
        contact_force_hat_(i) = 0;
    }
    for (int j = 0; j < 6; j++)
    {

        x_hat_(j) = 0;
        x_hat_pri_(j) = 0;
        x_hat_pos_(j) = 0;
    }
    P << 1,0,0,0,0,0,
       0,1,0,0,0,0,
       0,0,1,0,0,0,
       0,0,0,1,0,0,
       0,0,0,0,1,0,
       0,0,0,0,0,1;
    ic1xx = 0.00183;
    ic1xy = -0.00019;
    ic1xz = -0.00022;
    ic1yx = ic1xy;
    ic1yy = 0.00186;
    ic1yz = 0.00007;
    ic1zx = ic1xz;
    ic1zy = ic1yz;
    ic1zz = 0.00121;
    ic2xx = 0.0063;
    ic2xy = 0.00016;
    ic2xz = 0.0017;
    ic2yx = ic2xy;
    ic2yy = 0.01681;
    ic2yz = -0.00027;
    ic2zx = ic2xz;
    ic2zy = ic2yz;
    ic2zz = 0.01357;
    ic3xx = 0.00018;
    ic3xy = 0.00007;
    ic3xz = -0.00014;
    ic3yx = ic3xy;
    ic3yy = 0.00373;
    ic3yz = 0.00000;
    ic3zx = ic3xz;
    ic3zy = ic3yz;
    ic3zz = 0.00366;


    m1_ = 1.3442;
    m2_ = 2.8342;
    m3_ = 0.2663;

    //留ㅼ뒪留ㅽ떚移댁뿉�꽌 �떖蹂쇰┃ �뿰�궛�쑝濡� �쟾遺� 諛붽씀怨�
    //�깮�꽦�옄�뿉�꽌 �뒪�쐞移� 臾몄쑝濡� l,lc,諛쒕걹媛곷룄留� 諛붽씀怨�, FK ,�룞�뿭�븰,ik�떇�� �븯�굹濡� �넻�씪(low arm upper arm留� 援щ텇)
    switch (leg_type_)
    {
    case FL:
        l1_ = 0.1;
        lc1_ = -0.01741;
        break;
    case FR:
        l1_ = -0.1;
        lc1_ = 0.01741;
        break;
    case BL:
        l1_ = 0.1;
        lc1_ = -0.01741;
        break;
    case BR:
        l1_ = -0.1;
        lc1_ = 0.01741;
        break;
    };
    l2_ = 0.305;
    l3_ = 0.27;
    l4_ = 0.045;
    l5_ = 0.30349250;
    lc2_ = 0.01381;
    lc3_ = 0.05915;
    gam_ = 0.10503849; //6.01286deg
    r1_ = lc1_ / l1_;
    r2_ = lc2_ / l2_;
    r3_ = lc3_ / l3_;

    g_ = 9.8067;
    root2_ = sqrt(2);
    root2_inv_ = 1. / root2_;

    camera_fix = false;
    homing_flag_ = 0;
    is_ground_IMU_enable_ = false;
    is_detph_camera_enable_ = false;
    is_force_sensor_enable_ = false;
    is_ground_force_sensor_enable_ = false;
}
RobotLeg::~RobotLeg()
{
    // TODO Auto-generated destructor stub
}
void RobotLeg::InitializeJointAngle(const double (&joint_angle)[NUM_AXIS], const double (&auxiliary_joint_angle)[NUM_AXIS])
{
    is_initialize = true;
    for (int i = 0; i < NUM_AXIS; i++)
    {
        initial_joint_angle_compensation_[i] = auxiliary_joint_angle[i] - joint_angle[i];
    }
}
void RobotLeg::SetJointAngle(const double (&actual_joint_angle)[NUM_AXIS])
{
    for (int i = 0; i < NUM_AXIS; i++)
    {
        q_(i) = actual_joint_angle[i] + initial_joint_angle_compensation_[i];
        q_dot_(i) = q_calculus_[i].Diff(q_(i)); //Automatically calculate velocity
        q_dot_(i) = q_dot_LPF_[i].Filter(q_dot_(i), 30);
        q_ddot_(i) = q_dot_calculus_[i].Diff(q_dot_(i)); //Automatically calculate acceleration
        q_ddot_(i) = q_ddot_LPF_[i].Filter(q_ddot_(i), 30);
    }
    CalcJacobian();
    CalcEEState();
}
void RobotLeg::SetJointAngle(const double (&actual_joint_angle)[NUM_AXIS], const double (&actual_joint_angular_velocity)[NUM_AXIS])
{
    for (int i = 0; i < NUM_AXIS; i++) //0:KP 1:HP 2:HR
    {
        q_(i) = actual_joint_angle[i] + initial_joint_angle_compensation_[i];
        q_dot_(i) = actual_joint_angular_velocity[i];
        q_ddot_(i) = q_dot_calculus_[i].Diff(q_dot_(i)); //Automatically calculate acceleration
        q_ddot_(i) = q_ddot_LPF_[i].Filter(q_ddot_(i), 30);
    }
    CalcJacobian();
    CalcEEState();
}
void RobotLeg::SetEndEffectorTrajectory(TrajectoryGenerator &trajectory_3D)
{
    trajectory_3D.ComputeTrajectory();
    if (control_mode_ == POSITION_CONTROL || control_mode_ == HYBRID_CONTROL)
    {
        xyz_d_(X) = trajectory_3D.GetPosition(X);
        xyz_dot_d_(X) = trajectory_3D.GetVelocity(X);
        xyz_ddot_d_(X) = trajectory_3D.GetAcceleration(X);
        xyz_d_(Y) = trajectory_3D.GetPosition(Y);
        xyz_dot_d_(Y) = trajectory_3D.GetVelocity(Y);
        xyz_ddot_d_(Y) = trajectory_3D.GetAcceleration(Y);
        xyz_d_(Z) = trajectory_3D.GetPosition(Z);
        xyz_dot_d_(Z) = trajectory_3D.GetVelocity(Z);
        xyz_ddot_d_(Z) = trajectory_3D.GetAcceleration(Z);
    }
    else if (control_mode_ == POSITION_CONTROL_GROUND || control_mode_ == HYBRID_CONTROL_GROUND)
    {
        xyz_d_wrt_ground_(X) = trajectory_3D.GetPosition(X);
        xyz_dot_d_wrt_ground_(X) = trajectory_3D.GetVelocity(X);
        xyz_ddot_d_wrt_ground_(X) = trajectory_3D.GetAcceleration(X);
        xyz_d_wrt_ground_(Y) = trajectory_3D.GetPosition(Y);
        xyz_dot_d_wrt_ground_(Y) = trajectory_3D.GetVelocity(Y);
        xyz_ddot_d_wrt_ground_(Y) = trajectory_3D.GetAcceleration(Y);
        xyz_d_wrt_ground_(Z) = trajectory_3D.GetPosition(Z);
        xyz_dot_d_wrt_ground_(Z) = trajectory_3D.GetVelocity(Z);
        xyz_ddot_d_wrt_ground_(Z) = trajectory_3D.GetAcceleration(Z);
        XyzdGroundToXyzd();
    }
}
void RobotLeg::SetJointTrajectory(TrajectoryGenerator &trajectory, int axis)
{
    control_mode_ = JOINT_CONTROL;
    trajectory.ComputeTrajectory();
    q_d_(axis) = trajectory.GetPosition(axis);
    q_dot_d_(axis) = trajectory.GetVelocity(axis);
    q_ddot_d_(axis) = trajectory.GetAcceleration(axis);
}
void RobotLeg::Task_space_CTC()
{
    CalcDynamics();
    Calckalman();
    IK();
    Eigen::Matrix<double, NUM_AXIS,1> coulomb;
    coulomb << 1.75, 1.65, 3.80; //1.80,1.80,4.00;
    Eigen::Matrix<double, NUM_AXIS,1> viscous;
    viscous << 0.32, 0.292, 1.10; //0.295,0.295,1.125;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kd;
    Kd << 43., 0, 0,
        0, 43., 0,
        0, 0, 43.;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kp;
    Kp << 200., 0, 0,
        0, 200., 0,
        0, 0, 200.;


    if (control_mode_ == POSITION_CONTROL || control_mode_ == POSITION_CONTROL_GROUND)
    {
        xyz_ddot_ctc_ = xyz_ddot_d_; //+140*(xyz_dot_d_-xyz_dot_)+23000*(xyz_d_-xyz_);
        //xyz_ddot_ctc_ = xyz_ddot_d_ + (0 * (xyz_dot_d_ - xyz_dot_) + 5000 * (xyz_d_ - xyz_));
        q_ddot_ctc_ = J_inv * (xyz_ddot_ctc_ - J_dot * q_dot_);
        tor_M_ = M * q_ddot_ctc_;
        tor_C_ = C * q_dot_;
        tor_G_ = G;
        momentum_ = M * q_dot_;

        tor_tracking_ = J_trans * (Kd * (xyz_dot_d_ - xyz_dot_) + Kp * (xyz_d_ - xyz_));
        Eigen::Matrix<double, NUM_AXIS,1> max_tor_tracking;
        max_tor_tracking << 5, 5, 5;
        for (int i = 0; i < NUM_AXIS; i++)
        {
            if (fabs(tor_tracking_(i)) > max_tor_tracking(i))
                tor_tracking_(i) = max_tor_tracking(i) * sgn(tor_tracking_(i));
        }

        //friction compensation
        q_dot_d_ = J_inv * xyz_dot_d_;
        for (int i = 0; i < NUM_AXIS; i++)
        {
            double gap = 0.05;
            tor_coulomb_(i) = coulomb(i) * sgnGap(q_dot_d_(i), gap); //*sgn(q_dot_d_(i));//
            if (fabs(q_dot_d_(i)) <= gap)
                tor_coulomb_(i) = coulomb(i) * q_dot_d_(i) / gap;
//            if (fabs(q_dot_d_(i)) < kEpsilon)
//            {
//                // double allow_offset = 0.001;
//                // tor_coulomb_(i) = coulomb(i) * sgnGap((q_d_(i) - q_(i)), allow_offset); //*sgn(q_dot_d_(i));//
//                // if (fabs((q_d_(i) - q_(i))) <= allow_offset)
//                //     tor_coulomb_(i) = coulomb(i) * (q_d_(i) - q_(i)) / allow_offset;
//                double allow_offset = 0.1;
//                tor_coulomb_(i) = coulomb(i) * sgnGap((q_dot_(i)), allow_offset); //*sgn(q_dot_d_(i));//
//                if (fabs((q_dot_(i))) <= allow_offset)
//                    tor_coulomb_(i) = coulomb(i) * (q_dot_(i)) / allow_offset;
//            }
            tor_viscous_(i) = viscous(i) * q_dot_(i);
        }
        tor_ctc_ = tor_M_ + tor_C_ + tor_G_ + tor_coulomb_ + tor_viscous_ + tor_tracking_;
        tau_bar = tor_ctc_ - tor_C_ - tor_G_ - tor_coulomb_ - tor_viscous_;
    }
    else if (control_mode_ == HYBRID_CONTROL)
    {
        Kd << 12.9, 0, 0, 0, 12.9, 0, 0, 0, 0;
        Kp << 1380., 0, 0, 0, 1380., 0, 0, 0, 0;
        //2DOF position control
        xyz_ddot_ctc_ = xyz_ddot_d_; //+30*(xyz_dot_d_(X)-xyz_dot_(X))+5000*(xyz_d_(X)-xyz_(X));
        q_ddot_ctc_ = J_inv * (xyz_ddot_ctc_ - J_dot * q_dot_);
        tor_M_ = M * q_ddot_ctc_;
        tor_C_ = C * q_dot_;
        tor_G_ = G;
        momentum_ = M * q_dot_;

        tor_tracking_ = J_trans * (Kd * (xyz_dot_d_ - xyz_dot_) + Kp * (xyz_d_ - xyz_));
        Eigen::Matrix<double, NUM_AXIS,1> max_tor_tracking;
        max_tor_tracking << 5, 5, 5;
        for (int i = 0; i < NUM_AXIS; i++)
        {
            if (fabs(tor_tracking_(i)) > max_tor_tracking(i))
                tor_tracking_(i) = max_tor_tracking(i) * sgn(tor_tracking_(i));
        }

        double KD = 475;                                      // constant velocity control gain
        double Vd = Vd_force_control_;                        // F<Kd*Vd
        double al = Vd * 0.5;                                 // Vd/2;0.001*sgn(Vd);//
        double Kr = (KD * (Vd - al) - exert_force_z_d_) / al; // negative value
        if (fabs(Kr) < KD)
        {
            al = Vd - exert_force_z_d_ / KD;
            Kr = 0;
        }
        if (fabs(al) < 0.000001)
            al = 0.000001 * sgn(Vd);

        Eigen::Vector3d V_e;
        V_e << 0., 0., Vd - xyz_dot_(Z); //-xyz_dot_[X]  -xyz_dot_[Y]
        Eigen::Vector3d Tc = J_trans_inv * M * J_inv * J_dot * q_dot_;
        Eigen::Vector3d F_stance;
        F_stance << 0, 0, exert_force_z_d_;
        if (xyz_dot_[Z] * sgn(Vd) > al * sgn(Vd)) //constant velocity motion
        {
            exert_force_d_wrt_0_ = KD * (V_e) - (Tc);
            exert_force_d_wrt_0_(X) = 0;
            exert_force_d_wrt_0_(Y) = 0;
        }
        else if (xyz_dot_[Z] * sgn(Vd) > 0.005) //force increase
        {
            exert_force_d_wrt_0_ = Kr * xyz_dot_ - (Tc) + F_stance;
            exert_force_d_wrt_0_(X) = 0;
            exert_force_d_wrt_0_(Y) = 0;
        }
        else
        {
            exert_force_d_wrt_0_(X) = 0;
            exert_force_d_wrt_0_(Y) = 0;
            exert_force_d_wrt_0_(Z) = exert_force_z_d_;
        }
        //no velocity
        //        exert_force_d_wrt_0_(Z)=exert_force_z_d_;
        tor_exert_force_ = J_trans * exert_force_d_wrt_0_;

        //friction compensation
        Eigen::Vector3d xyz_dot_d_temp;
        Eigen::Vector3d q_dot_d_temp;
        xyz_dot_d_temp = xyz_dot_d_;
        if (end_effector_state_ == SWING)
        {
            xyz_dot_d_temp(Z) = Vd_force_control_;
        }
        else if (end_effector_state_ == STANCE)
        {
            if (fabs(exert_force_z_d_ + force_sensor_wrt_0_(Z)) < kEpsilon)
                xyz_dot_d_temp(Z) = 0;
            else
                xyz_dot_d_temp(Z) = (exert_force_z_d_ + force_sensor_wrt_0_(Z)) / fabs(exert_force_z_d_ + force_sensor_wrt_0_(Z)) * 0.110;
        }
        q_dot_d_temp = J_inv * xyz_dot_d_temp; //pseudo angular velocity
        for (int i = 0; i < NUM_AXIS; i++)
        {
            double gap = 1.;
            tor_coulomb_(i) = coulomb(i) * sgnGap(q_dot_d_temp(i), gap); //*sgn(q_dot_d_(i));//
            if (fabs(q_dot_d_temp(i)) <= gap)
                tor_coulomb_(i) = coulomb(i) * q_dot_d_temp(i) / gap;
            tor_viscous_(i) = viscous(i) * q_dot_d_temp(i);
        }

        tor_ctc_ = tor_M_ + tor_C_ + tor_G_ + tor_tracking_ + tor_exert_force_ + tor_coulomb_ + tor_viscous_;
        tau_bar = tor_ctc_ - tor_C_ - tor_G_ - tor_coulomb_ - tor_viscous_;
    }
    else if (control_mode_ == HYBRID_CONTROL_GROUND)
    {
        Kd << 12.9, 0, 0, 0, 12.9, 0, 0, 0, 0;
        Kp << 1380., 0, 0, 0, 1380., 0, 0, 0, 0;
        XyzToXyzGround();
        xyz_ddot_ctc_wrt_ground_ = xyz_ddot_d_wrt_ground_; //+30*(xyz_dot_d_(X)-xyz_dot_(X))+5000*(xyz_d_(X)-xyz_(X));

        xyz_ddot_ctc_ = rotation_ground_wrt_0_ * xyz_ddot_ctc_wrt_ground_;
        q_ddot_ctc_ = J_inv * (xyz_ddot_ctc_ - J_dot * q_dot_);
        tor_M_ = M * q_ddot_ctc_;
        tor_C_ = C * q_dot_;
        tor_G_ = G;
        momentum_ = M * q_dot_;

        Eigen::Vector3d tracking_force_wrt_ground;
        tracking_force_wrt_ground = Kd * (xyz_dot_d_wrt_ground_ - xyz_dot_wrt_ground_) + Kp * (xyz_d_wrt_ground_ - xyz_wrt_ground_);
        tor_tracking_ = J_trans * (rotation_ground_wrt_0_ * tracking_force_wrt_ground);
        Eigen::Matrix<double, NUM_AXIS,1> max_tor_tracking;
        max_tor_tracking << 5, 5, 5;
        for (int i = 0; i < NUM_AXIS; i++)
        {
            if (fabs(tor_tracking_(i)) > max_tor_tracking(i))
                tor_tracking_(i) = max_tor_tracking(i) * sgn(tor_tracking_(i));
        }

        double KD = 475;
        double Vd = Vd_force_control_;                        //F<Kd*Vd
                                                              //        double al=Vd-exert_force_z_d_/KD;
        double al = Vd * 0.5;                                 //Vd/2;0.001*sgn(Vd);//
        double Kr = (KD * (Vd - al) - exert_force_z_d_) / al; //negative value
        if (Kr > 0)
        {
            al = Vd - exert_force_z_d_ / KD;
            Kr = 0;
        }
        if (fabs(al) < 0.000001)
            al = 0.000001 * sgn(Vd);

        Eigen::Vector3d V_e;
        V_e << 0., 0., Vd - xyz_dot_wrt_ground_[Z]; //-xyz_dot_[X]  -xyz_dot_[Y]
        Eigen::Vector3d Tc = J_trans_inv * M * J_inv * J_dot * q_dot_;
        Eigen::Vector3d F_stance;
        F_stance << 0, 0, exert_force_z_d_;
        Eigen::Vector3d exert_force_d_wrt_ground;
        if (xyz_dot_wrt_ground_[Z] * sgn(Vd) > al * sgn(Vd))
        {
            exert_force_d_wrt_ground = KD * (V_e);
            exert_force_d_wrt_ground(X) = 0;
            exert_force_d_wrt_ground(Y) = 0;
        } //+0.3*Vel_calculus.Inte(V_e);
        else if (xyz_dot_wrt_ground_[Z] * sgn(Vd) > 0.005)
        {
            exert_force_d_wrt_ground = Kr * xyz_dot_wrt_ground_ + F_stance;
            exert_force_d_wrt_ground(X) = 0;
            exert_force_d_wrt_ground(Y) = 0;
        }
        else
        {
            exert_force_d_wrt_ground(X) = 0;
            exert_force_d_wrt_ground(Y) = 0;
            exert_force_d_wrt_ground(Z) = exert_force_z_d_;
        }
        //no velocity
        //        exert_force_d_wrt_0_(Z)=exert_force_z_d_;

        exert_force_d_wrt_0_ = rotation_ground_wrt_0_ * exert_force_d_wrt_ground;
        exert_force_d_wrt_0_(Z) -= (Tc(Z));
        tor_exert_force_ = J_trans * exert_force_d_wrt_0_;

        //friction compensation
        Eigen::Vector3d xyz_dot_d_temp, xyz_dot_d_wrt_ground_temp;
        Eigen::Vector3d q_dot_d_temp;
        xyz_dot_d_wrt_ground_temp = xyz_dot_d_wrt_ground_;
        if (end_effector_state_ == SWING)
        {
            xyz_dot_d_wrt_ground_temp(Z) = Vd_force_control_;
        }
        else if (end_effector_state_ == STANCE)
        {
            if (fabs(exert_force_z_d_ - force_sensor_(Z)) < kEpsilon)
                xyz_dot_d_wrt_ground_temp(Z) = 0;
            else
                xyz_dot_d_wrt_ground_temp(Z) = (exert_force_z_d_ - force_sensor_(Z)) / fabs(exert_force_z_d_ - force_sensor_(Z)) * 0.110;
        }
        xyz_dot_d_temp = rotation_ground_wrt_0_ * xyz_dot_d_wrt_ground_temp;
        q_dot_d_temp = J_inv * xyz_dot_d_temp;
        for (int i = 0; i < NUM_AXIS; i++)
        {
            double gap = 1.;
            tor_coulomb_(i) = coulomb(i) * sgnGap(q_dot_d_(i), gap); //*sgn(q_dot_d_(i));//
            if (fabs(q_dot_d_(i)) <= gap)
                tor_coulomb_(i) = coulomb(i) * q_dot_d_(i) / gap;
            tor_viscous_(i) = viscous(i) * q_dot_(i);
        }

        tor_ctc_ = tor_M_ + tor_C_ + tor_G_ + tor_tracking_ + tor_exert_force_ + tor_coulomb_ + tor_viscous_;
        tau_bar = tor_ctc_ - tor_C_ - tor_G_ - tor_coulomb_ - tor_viscous_;
    }
    x_hat_pri_ = F * x_hat_ + B * tau_bar;
    f = F * P;
    P = f * F.transpose() + Q;
    h = H * P;
    r = (h * H.transpose() + R).inverse();
    k = P * H.transpose();
    K = k * r;
    x_hat_pos_ = x_hat_pri_ + K * (momentum_ - (H * x_hat_pri_));
    P = (I_N - (K * H)) * P;
    x_hat_ = x_hat_pos_;
    contact_tor_hat_(0) = x_hat_(3);
    contact_tor_hat_(1) = x_hat_(4);
    contact_tor_hat_(2) = x_hat_(5);
    contact_force_hat_ = J_trans_inv * contact_tor_hat_;







    target_torque_ = tor_ctc_;
}
void RobotLeg::CalcDynamics()
{
    //Inertial Matrix
    M(0, 0) = (2 * ic1zz + ic2xx + ic2yy + ic3xx + ic3yy + 2 * pow(l1_, 2) * (m2_ + m3_ + m1_ * pow(r1_, 2)) + pow(l2_, 2) * (m3_ + m2_ * pow(r2_, 2)) + pow(l3_, 2) * m3_ * pow(r3_, 2) + (-ic2xx + ic2yy + pow(l2_, 2) * (m3_ + m2_ * pow(r2_, 2))) * std::cos(2 * q_(HP)) + (-ic3xx + ic3yy) * std::cos(2 * (q_(HP) + q_(KP))) + l3_ * m3_ * r3_ * (4 * l2_ * std::cos(q_(HP)) * std::cos(q_(HP) + q_(KP)) + l3_ * r3_ * std::cos(2 * (q_(HP) + q_(KP)))) + (ic2xy + ic2yx) * std::sin(2 * q_(HP)) + (ic3xy + ic3yx) * std::sin(2 * (q_(HP) + q_(KP)))) * 0.5;
    M(0, 1) = -(ic2yz * std::cos(q_(HP))) - ic3yz * std::cos(q_(HP) + q_(KP)) + (-ic2xz + l1_ * l2_ * (m3_ + m2_ * r2_)) * std::sin(q_(HP)) + (-ic3xz + l1_ * l3_ * m3_ * r3_) * std::sin(q_(HP) + q_(KP));
    M(0, 2) = -(ic3yz * std::cos(q_(HP) + q_(KP))) + (-ic3xz + l1_ * l3_ * m3_ * r3_) * std::sin(q_(HP) + q_(KP));
    M(1, 0) = M(0, 1);
    M(1, 1) = ic2zz + ic3zz + pow(l2_, 2) * (m3_ + m2_ * pow(r2_, 2)) + pow(l3_, 2) * m3_ * pow(r3_, 2) + 2 * l2_ * l3_ * m3_ * r3_ * std::cos(q_(KP));
    M(1, 2) = ic3zz + pow(l3_, 2) * m3_ * pow(r3_, 2) + l2_ * l3_ * m3_ * r3_ * std::cos(q_(KP));
    M(2, 0) = M(0, 2);
    M(2, 1) = M(1, 2);
    M(2, 2) = ic3zz + pow(l3_, 2) * m3_ * pow(r3_, 2);
    //christoffel
    c111 = 0.;
    c211 = 0.5 * ((ic2xy + ic2yx) * std::cos(2 * q_(HP)) + (ic3xy + ic3yx) * std::cos(2 * (q_(HP) + q_(KP))) + (ic2xx - ic2yy - pow(l2_, 2) * (m3_ + m2_ * pow(r2_, 2))) * std::sin(2 * q_(HP)) + (ic3xx - ic3yy - pow(l3_, 2) * m3_ * pow(r3_, 2)) * std::sin(2 * (q_(HP) + q_(KP))) - 2 * l2_ * l3_ * m3_ * r3_ * std::sin(2 * q_(HP) + q_(KP)));
    c311 = 0.25 * (2 * (ic3xy + ic3yx) * std::cos(2 * (q_(HP) + q_(KP))) - 4 * (l2_ * l3_ * m3_ * r3_ * std::cos(q_(HP)) + (-ic3xx + ic3yy + pow(l3_, 2) * m3_ * pow(r3_, 2)) * std::cos(q_(HP) + q_(KP))) * std::sin(q_(HP) + q_(KP)));
    c121 = c211;
    c221 = 1. * ((-ic2xz + l1_ * l2_ * (m3_ + m2_ * r2_)) * std::cos(q_(HP)) + (-ic3xz + l1_ * l3_ * m3_ * r3_) * std::cos(q_(HP) + q_(KP)) + ic2yz * std::sin(q_(HP)) + ic3yz * std::sin(q_(HP) + q_(KP)));
    c321 = 0.5 * (-2 * (ic3xz - l1_ * l3_ * m3_ * r3_) * std::cos(q_(HP) + q_(KP)) + 2 * ic3yz * std::sin(q_(HP) + q_(KP)));
    c131 = c311;
    c231 = c321;
    c331 = c231;
    c112 = -c211;
    c212 = 0.;
    c312 = 0.;
    c122 = (0.5 * ic2xz - 0.5 * ic2zx) * std::cos(q_(HP)) + (0.5 * ic3xz - 0.5 * ic3zx) * std::cos(q_(HP) + q_(KP)) + (-0.5 * ic2yz + 0.5 * ic2zy) * std::sin(q_(HP)) + (-0.5 * ic3yz + 0.5 * ic3zy) * std::sin(q_(HP) + q_(KP));
    c222 = 0.;
    c322 = -1. * l2_ * l3_ * m3_ * r3_ * std::sin(q_(KP));
    c132 = (0.5 * ic3xz - 0.5 * ic3zx) * std::cos(q_(HP) + q_(KP)) + (-0.5 * ic3yz + 0.5 * ic3zy) * std::sin(q_(HP) + q_(KP));
    c232 = c322;
    c332 = c322;
    c113 = -c311;
    c213 = 0.;
    c313 = 0.;
    c123 = c132;
    c223 = c322;
    c323 = 0.;
    c133 = c123;
    c233 = 0.;
    c333 = 0.;
    //centrifugal and coliolis
    C(0, 0) = c111 * q_dot_(HR) + c211 * q_dot_(HP) + c311 * q_dot_(KP);
    C(0, 1) = c121 * q_dot_(HR) + c221 * q_dot_(HP) + c321 * q_dot_(KP);
    C(0, 2) = c131 * q_dot_(HR) + c231 * q_dot_(HP) + c331 * q_dot_(KP);
    C(1, 0) = c112 * q_dot_(HR) + c212 * q_dot_(HP) + c312 * q_dot_(KP);
    C(1, 1) = c122 * q_dot_(HR) + c222 * q_dot_(HP) + c322 * q_dot_(KP);
    C(1, 2) = c132 * q_dot_(HR) + c232 * q_dot_(HP) + c332 * q_dot_(KP);
    C(2, 0) = c113 * q_dot_(HR) + c213 * q_dot_(HP) + c313 * q_dot_(KP);
    C(2, 1) = c123 * q_dot_(HR) + c223 * q_dot_(HP) + c323 * q_dot_(KP);
    C(2, 2) = c133 * q_dot_(HR) + c233 * q_dot_(HP) + c333 * q_dot_(KP);
    //gravity
    G(0) = g_ * (l1_ * (m2_ + m3_ + m1_ * r1_) * std::cos(q_(HR)) + (l2_ * (m3_ + m2_ * r2_) * std::cos(q_(HP)) + l3_ * m3_ * r3_ * std::cos(q_(HP) + q_(KP))) * std::sin(q_(HR)));
    G(1) = g_ * std::cos(q_(HR)) * (l2_ * (m3_ + m2_ * r2_) * std::sin(q_(HP)) + l3_ * m3_ * r3_ * std::sin(q_(HP) + q_(KP)));
    G(2) = g_ * l3_ * m3_ * r3_ * std::cos(q_(HR)) * std::sin(q_(HP) + q_(KP));
}
void RobotLeg::Calckalman()
{

    F(0, 0) = 1;
    F(0, 1) = 0;
    F(0, 2) = 0;
    F(0, 3) = 0.001;
    F(0, 4) = 0;
    F(0, 5) = 0;
    F(1, 0) = 0;
    F(1, 1) = 1;
    F(1, 2) = 0;
	F(1, 3) = 0;
	F(1, 4) = 0.001;
	F(1, 5) = 0;
    F(2, 0) = 0;
    F(2, 1) = 0;
    F(2, 2) = 1;
	F(2, 3) = 0;
	F(2, 4) = 0;
	F(2, 5) = 0.001;
	F(3, 0) = 0;
	F(3, 1) = 0;
	F(3, 2) = 0;
	F(3, 3) = 1;
	F(3, 4) = 0;
	F(3, 5) = 0;
	F(4, 0) = 0;
	F(4, 1) = 0;
	F(4, 2) = 0;
	F(4, 3) = 0;
	F(4, 4) = 1;
	F(4, 5) = 0;
	F(5, 0) = 0;
	F(5, 1) = 0;
	F(5, 2) = 0;
	F(5, 3) = 0;
	F(5, 4) = 0;
	F(5, 5) = 1;


    B(0, 0) = 0.001;
    B(0, 1) = 0;
    B(0, 2) = 0;
    B(1, 0) = 0;
    B(1, 1) = 0.001;
    B(1, 2) = 0;
    B(2, 0) = 0;
    B(2, 1) = 0;
    B(2, 2) = 0.001;
    B(3, 0) = 0;
    B(3, 1) = 0;
    B(3, 2) = 0;
    B(4, 0) = 0;
    B(4, 1) = 0;
    B(4, 2) = 0;
    B(5, 0) = 0;
    B(5, 1) = 0;
    B(5, 2) = 0;

    Q(0, 0) = pow(1, 2)*0.001;
    Q(0, 1) = 0;
    Q(0, 2) = 0;
    Q(0, 3) = 0;
    Q(0, 4) = 0;
    Q(0, 5) = 0;
    Q(1, 0) = 0;
    Q(1, 1) = pow(1, 2)*0.001;
    Q(1, 2) = 0;
	Q(1, 3) = 0;
	Q(1, 4) = 0;
	Q(1, 5) = 0;
    Q(2, 0) = 0;
    Q(2, 1) = 0;
    Q(2, 2) = pow(1, 2)*0.001;
	Q(2, 3) = 0;
	Q(2, 4) = 0;
	Q(2, 5) = 0;
	Q(3, 0) = 0;
	Q(3, 1) = 0;
	Q(3, 2) = 0;
	Q(3, 3) = pow(15, 2)*0.001;
	Q(3, 4) = 0;
	Q(3, 5) = 0;
	Q(4, 0) = 0;
	Q(4, 1) = 0;
	Q(4, 2) = 0;
	Q(4, 3) = 0;
	Q(4, 4) = pow(15, 2)*0.001;
	Q(4, 5) = 0;
	Q(5, 0) = 0;
	Q(5, 1) = 0;
	Q(5, 2) = 0;
	Q(5, 3) = 0;
	Q(5, 4) = 0;
	Q(5, 5) = pow(15, 2)*0.001;

    R(0, 0) = pow(0.001, 2)/0.001;
    R(0, 1) = 0;
    R(0, 2) = 0;
    R(1, 0) = 0;
    R(1, 1) = pow(0.001, 2)/0.001;
    R(1, 2) = 0;
    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = pow(0.001, 2)/0.001;

    H(0, 0) = 1;
    H(0, 1) = 0;
    H(0, 2) = 0;
    H(0, 3) = 0;
    H(0, 4) = 0;
    H(0, 5) = 0;
    H(1, 0) = 0;
    H(1, 1) = 1;
    H(1, 2) = 0;
    H(1, 3) = 0;
    H(1, 4) = 0;
    H(1, 5) = 0;
    H(2, 0) = 0;
    H(2, 1) = 0;
    H(2, 2) = 1;
    H(2, 3) = 0;
    H(2, 4) = 0;
    H(2, 5) = 0;


    I_N(0, 0) = 1;
    I_N(0, 1) = 0;
    I_N(0, 2) = 0;
    I_N(0, 3) = 0;
    I_N(0, 4) = 0;
    I_N(0, 5) = 0;
    I_N(1, 0) = 0;
    I_N(1, 1) = 1;
    I_N(1, 2) = 0;
    I_N(1, 3) = 0;
    I_N(1, 4) = 0;
    I_N(1, 5) = 0;
    I_N(2, 0) = 0;
    I_N(2, 1) = 0;
    I_N(2, 2) = 1;
    I_N(2, 3) = 0;
    I_N(2, 4) = 0;
    I_N(2, 5) = 0;
    I_N(3, 0) = 0;
    I_N(3, 1) = 0;
    I_N(3, 2) = 0;
    I_N(3, 3) = 1;
    I_N(3, 4) = 0;
    I_N(3, 5) = 0;
    I_N(4, 0) = 0;
    I_N(4, 1) = 0;
    I_N(4, 2) = 0;
    I_N(4, 3) = 0;
    I_N(4, 4) = 1;
    I_N(4, 5) = 0;
    I_N(5, 0) = 0;
    I_N(5, 1) = 0;
    I_N(5, 2) = 0;
    I_N(5, 3) = 0;
    I_N(5, 4) = 0;
    I_N(5, 5) = 1;





}
void RobotLeg::CalcJacobian()
{
    //base and E.E Jacobian
    if (leg_type_ == FL)
    {
        J(0, 0) = 0;
        J(0, 1) = -(l2_ * cos(q_(HP))) - l3_ * cos(q_(HP) + q_(KP)) + (l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(0, 2) = -(l3_ * cos(q_(HP) + q_(KP))) + (l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(1, 0) = -(l1_ * sin(q_(HR))) + cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + (l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2));
        J(1, 1) = -(sin(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) + 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(1, 2) = -(sin(q_(HR)) * (2 * l3_ * sin(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 0) = l1_ * cos(q_(HR)) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 1) = (cos(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) + 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 2) = (cos(q_(HR)) * (2 * l3_ * sin(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
    }
    if (leg_type_ == FR)
    {
        J(0, 0) = 0;
        J(0, 1) = -(l2_ * cos(q_(HP))) - l3_ * cos(q_(HP) + q_(KP)) + (l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(0, 2) = -(l3_ * cos(q_(HP) + q_(KP))) + (l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(1, 0) = l1_ * sin(q_(HR)) + cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + (l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2));
        J(1, 1) = -(sin(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) + 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(1, 2) = -(sin(q_(HR)) * (2 * l3_ * sin(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 0) = -(l1_ * cos(q_(HR))) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 1) = (cos(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) + 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 2) = (cos(q_(HR)) * (2 * l3_ * sin(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
    }
    if (leg_type_ == BL)
    {
        J(0, 0) = 0;
        J(0, 1) = -(l2_ * cos(q_(HP))) - l3_ * cos(q_(HP) + q_(KP)) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(0, 2) = -(l3_ * cos(q_(HP) + q_(KP))) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(1, 0) = -(l1_ * sin(q_(HR))) + cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
        J(1, 1) = (sin(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) - 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(1, 2) = (sin(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) - 2 * l3_ * sin(q_(HP) + q_(KP)))) / 2.;
        J(2, 0) = l1_ * cos(q_(HR)) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 1) = (cos(q_(HR)) * (sqrt(2) * l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) + 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 2) = (cos(q_(HR)) * (2 * l3_ * sin(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
    }
    if (leg_type_ == BR)
    {
        J(0, 0) = 0;
        J(0, 1) = -(l2_ * cos(q_(HP))) - l3_ * cos(q_(HP) + q_(KP)) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(0, 2) = -(l3_ * cos(q_(HP) + q_(KP))) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        J(1, 0) = l1_ * sin(q_(HR)) + cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
        J(1, 1) = (sin(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) - 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(1, 2) = (sin(q_(HR)) * (sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) - 2 * l3_ * sin(q_(HP) + q_(KP)))) / 2.;
        J(2, 0) = -(l1_ * cos(q_(HR))) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 1) = (cos(q_(HR)) * (sqrt(2) * l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) + 2 * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP))))) / 2.;
        J(2, 2) = (cos(q_(HR)) * (2 * l3_ * sin(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            J_dot(i, j) = J_calculus_[3 * i + j].Diff(J(i, j));
        }
    }
    J_inv = J.inverse();
    J_trans = J.transpose();
    J_trans_inv = J_trans.inverse();
}
void RobotLeg::CalcEEState()
{
    //End Effector pos,Vel,Acc
    FK();
    xyz_dot_ = J * q_dot_;
    xyz_ddot_ = J * q_ddot_ + J_dot * q_dot_;
    for (int i = 0; i < NUM_AXIS; i++)
    { //\C0\FA\C0\E5\BF\EB

        xyz_dot_[i] = xyz_dot_(i);
        xyz_ddot_[i] = xyz_ddot_(i);
    }
}
void RobotLeg::Joint_space_PD_control()
{
    CalcDynamics();
    Eigen::Matrix<double, NUM_AXIS,1> coulomb;
    coulomb << 1.75, 1.65, 3.80; //1.80,1.80,4.00;
    Eigen::Matrix<double, NUM_AXIS,1> viscous;
    viscous << 0.32, 0.292, 1.10; //0.295,0.295,1.125;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kd;
    Kd << 6.5, 0, 0,
        0, 8.0, 0,
        0, 0, 12.0;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kp;
    Kp << 650., 0, 0,
        0, 800., 0,
        0, 0, 1200.;
    IK();
    //	double K_i[NUM_AXIS]= { 0.0, 0.0, 0.0};//={ 300.0, 500.0, 800.0};
    if (control_mode_ == POSITION_CONTROL || control_mode_ == POSITION_CONTROL_GROUND)
    {
        for (int i = 0; i < NUM_AXIS; i++)
        {
            double gap = 0.005;
            tor_coulomb_(i) = coulomb(i) * sgnGap(q_dot_d_(i), gap); //*sgn(q_dot_d_(i));//
            if (fabs(q_dot_d_(i)) <= gap)
                tor_coulomb_(i) = coulomb(i) * q_dot_d_(i) / gap;
            if (fabs(q_dot_d_(i)) < kEpsilon)
            {
                double allow_offset = 0.001;
                tor_coulomb_(i) = coulomb(i) * sgnGap((q_d_(i) - q_(i)), allow_offset); //*sgn(q_dot_d_(i));//
                if (fabs((q_d_(i) - q_(i))) <= allow_offset)
                    tor_coulomb_(i) = coulomb(i) * (q_d_(i) - q_(i)) / allow_offset;
            }
            tor_viscous_(i) = viscous(i) * q_dot_(i);
        }
        tor_G_ = G;
        target_torque_ = tor_G_ + Kp * (q_d_ - q_) + Kd * (q_dot_d_ - q_dot_) + tor_coulomb_ + tor_viscous_;
    }
    //        if(control_mode_==HYBRID_CONTROL_GROUND) {
    //                force_error_=exert_force_z_d_-(-G_F(2));
    //                force_error_inte_+=force_error_*0.001;
    //                double pgain=0.000007;double igain=0.0020;
    ////		double dgain;
    //                double limit_p=10.;
    //                double limit_i=10.;
    //                double limit_d=10.;
    //
    //                F_p=pgain*force_error_;
    //                F_i=igain*force_error_inte_;
    ////		F_d=-dgain*(-G_Fdot(2));
    //
    //                if(F_p>=limit_p) F_p=limit_p;
    //                if(F_p<=-limit_p) F_p=-limit_p;
    //                if(F_i>=limit_i) F_i=limit_i;
    //                if(F_i<=-limit_i) F_i=-limit_i;
    //                if(F_d>=limit_d) F_d=limit_d;
    //                if(F_d<=-limit_d) F_d=-limit_d;
    //                if(initF==0) {
    //                        init_Z=xyz_d_wrt_ground_[Z];
    //                        initF=1;
    //                }
    //                des_Z=init_Z+F_p +F_i;
    //                G_XYZ_d_4x1(2)=des_Z;
    //                XyzdGroundToXyzd();
    ////                IK(xyz_d_,q_d_);
    //        }

    //	if(control_mode_==FORCE_CONTROL){
    //					F_error=-70.0-Opto_EE_wrt_G(2,0);//force_ee_wrt_g\B4\C2 \C0\BD\BC\F6
    //					LPF_F_error=-70.0-LPF_Opto_EE_wrt_G(2,0);
    //					F_integral+=LPF_F_error *0.001;//F_error*0.001;//
    //					//x_trans=0.0;
    //					//y_trans=0.02;
    //					if(Traj_time<=3000) K_p_F=0.00007*0.5*(1-std::cos( PI*(Traj_time-2500)/500 ));//p gain \BC\AD\BC\AD\C8\F7 \C1\F5\B0\A1
    //					//K_p_F=0.00005*0.5*(1-std::cos( PI*Opto_EE_wrt_G(2,0)/(-30) ));
    //					z_offset=0.02+K_p_F*LPF_F_error  +0.0020*F_integral;//+0.00004*F_error
    //					//K_p_F=0.00018*0.5*(1-std::cos( PI*Opto_EE_wrt_G(2,0)/(-30) ));
    //					//z_offset=0.02+K_p_F*F_error  +0.0015*F_integral;//+0.00004*F_error
    //					if(z_offset<=-0.2) z_offset=-0.2;
    //					point_g=Point_ground(imu_r.theta,imu_p.theta,imu_y.theta, x_trans,y_trans,z_offset);
    //					z=point_g.z;
    //					x=point_g.x;
    //					y=point_g.y;
    //					if(Traj_time==4500) {
    //						rt_printf("Control mode:%d\n",control_mode_);
    //	//					finish_time=Traj_time;
    //	//					rt_printf("finish_time:%d\n",finish_time);
    //	//					Traj[X].set_traj(x,x,finish_time,finish_time+1000);
    //	//					Traj[Y].set_traj(y,y,finish_time,finish_time+1000);
    //	//					Traj[Z].set_traj(z,z,finish_time,finish_time+1000);//
    //					}
    //				}
}
void RobotLeg::SimpleJointControl()
{
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kd;
    Kd << 2000., 0, 0,
        0, 500., 0,
        0, 0, 300.;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kp;
    Kp << 20., 0, 0,
        0, 5., 0,
        0, 0, 3.;
    target_torque_ = Kp * (q_d_ - q_) + Kd * (q_dot_d_ - q_dot_);
}
void RobotLeg::SimpleReciprocating(int _axis)
{
    //    if(trajectory_.IsArrived() && via==0) {
    //        double qf[NUM_AXIS]={-1.5708,0.,0.};
    //        trajectory_.SetInitTraj(q_,qf,1000);
    //        control_mode_=JOINT_CONTROL;
    //        via=1;
    //    }
    //    else if(trajectory_.IsArrived() && via==1 ){
    //        double qf[NUM_AXIS]={-0.174533,0.,0.};
    //        trajectory_.SetInitTraj(q_,qf,1000);
    //        control_mode_=JOINT_CONTROL;
    //        via=0;
    //    }
    //    SetEndEffectorTrajectory(trajectory_);
    //    for (int i= 0; i< NUM_AXIS; ++i) {
    //        target_torque_[i]=100*(q_d_[i]-q_[i])+1*(q_dot_d_[i]-q_dot_[i]);
    //    }
}
void RobotLeg::Joint_space_CTC()
{
    CalcDynamics();
    Eigen::Matrix<double, NUM_AXIS,1> coulomb;
    coulomb << 1.75, 1.65, 3.80; //1.80,1.80,4.00;
    Eigen::Matrix<double, NUM_AXIS,1> viscous;
    viscous << 0.32, 0.292, 1.10; //0.295,0.295,1.125;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kd;
    Kd << 12., 0, 0,
        0, 12., 0,
        0, 0, 12.;
    Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kp;
    Kp << 100., 0, 0,
        0, 100., 0,
        0, 0, 100.;
    q_ddot_ctc_ = q_ddot_d_;
    tor_tracking_ = Kp * (q_d_ - q_) + Kd * (q_dot_d_ - q_dot_);

    //friction compensation
    for (int i = 0; i < NUM_AXIS; i++)
    {
        double gap = 0.005;
        tor_coulomb_(i) = coulomb(i) * sgnGap(q_dot_d_(i), gap); //*sgn(q_dot_d_(i));//
        if (fabs(q_dot_d_(i)) <= gap)
            tor_coulomb_(i) = coulomb(i) * q_dot_d_(i) / gap;
//        if (fabs(q_dot_d_(i)) < kEpsilon)
//        {
//            double allow_offset = 0.001;
//            tor_coulomb_(i) = coulomb(i) * sgnGap((q_d_(i) - q_(i)), allow_offset); //*sgn(q_dot_d_(i));//
//            if (fabs((q_d_(i) - q_(i))) <= allow_offset)
//                tor_coulomb_(i) = coulomb(i) * (q_d_(i) - q_(i)) / allow_offset;
//        }
        tor_viscous_(i) = viscous(i) * q_dot_(i);
    }

    tor_M_ = M * q_ddot_ctc_; //at the constant velocity, it may be friction torque
    tor_C_ = C * q_dot_;
    tor_G_ = G;
    momentum_ = M * q_dot_;

    target_torque_ = tor_M_ + tor_C_ + tor_G_ + tor_tracking_ + tor_coulomb_ + tor_viscous_;
}

void RobotLeg::GravityCompensation()
{
    CalcDynamics();
    tor_C_ = C * q_dot_;
    tor_G_ = G;
    target_torque_ = tor_C_ + tor_G_;
}
// void RobotLeg::FrictionCompensation()
// {
//     CalcDynamics();
//     Eigen::Vector<double,NUM_AXIS> coulomb;
//     coulomb << 1.75, 1.65, 3.80; //1.80,1.80,4.00;
//     Eigen::Vector<double,NUM_AXIS> viscous;
//     viscous << 0.32, 0.292, 1.10; //0.295,0.295,1.125;
//     tor_C_ = C * q_dot_;
//     tor_G_ = G;
//     for (int i = 0; i < NUM_AXIS; i++)
//     {
//         double gap = 0.005;
//         tor_coulomb_(i) = coulomb(i) * sgnGap(q_dot_(i), gap); //*sgn(q_dot_d_(i));//
//         if (fabs(q_dot_(i)) <= gap)
//             tor_coulomb_(i) = coulomb(i) * q_dot_(i) / gap;
//         if (fabs(q_dot_(i)) < kEpsilon)
//         {
//             double allow_offset = 0.001;
//             tor_coulomb_(i) = coulomb(i) * sgnGap((q_d_(i) - q_(i)), allow_offset); //*sgn(q_dot_d_(i));//
//             if (fabs((q_d_(i) - q_(i))) <= allow_offset)
//                 tor_coulomb_(i) = coulomb(i) * (q_d_(i) - q_(i)) / allow_offset;
//         }
//         tor_viscous_(i) = viscous(i) * q_dot_(i);
//     }
//     target_torque_ = tor_C_ + tor_G_ + tor_coulomb_ + tor_viscous_;
// }

void RobotLeg::FK()
{
    switch (leg_type_)
    {
    case FL:
        ht_ee_wrt_0_(0, 0) = (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) / sqrt(2);
        ht_ee_wrt_0_(0, 1) = 0;
        ht_ee_wrt_0_(0, 2) = -((cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) / sqrt(2));
        ht_ee_wrt_0_(0, 3) = -(l2_ * sin(q_(HP))) - l3_ * sin(q_(HP) + q_(KP)) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 0) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 1) = -cos(q_(HR));
        ht_ee_wrt_0_(1, 2) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 3) = l1_ * cos(q_(HR)) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(2, 0) = -((cos(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
        ht_ee_wrt_0_(2, 1) = -sin(q_(HR));
        ht_ee_wrt_0_(2, 2) = (cos(q_(HR)) * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(2, 3) = l1_ * sin(q_(HR)) - (cos(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(3, 0) = 0;
        ht_ee_wrt_0_(3, 1) = 0;
        ht_ee_wrt_0_(3, 2) = 0;
        ht_ee_wrt_0_(3, 3) = 1;
        break;
    case FR:
        ht_ee_wrt_0_(0, 0) = (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) / sqrt(2);
        ht_ee_wrt_0_(0, 1) = 0;
        ht_ee_wrt_0_(0, 2) = -((cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) / sqrt(2));
        ht_ee_wrt_0_(0, 3) = -(l2_ * sin(q_(HP))) - l3_ * sin(q_(HP) + q_(KP)) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 0) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 1) = -cos(q_(HR));
        ht_ee_wrt_0_(1, 2) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 3) = -(l1_ * cos(q_(HR))) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(2, 0) = -((cos(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
        ht_ee_wrt_0_(2, 1) = -sin(q_(HR));
        ht_ee_wrt_0_(2, 2) = (cos(q_(HR)) * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(2, 3) = -(l1_ * sin(q_(HR))) - (cos(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(3, 0) = 0;
        ht_ee_wrt_0_(3, 1) = 0;
        ht_ee_wrt_0_(3, 2) = 0;
        ht_ee_wrt_0_(3, 3) = 1;
        break;
    case BL:
        ht_ee_wrt_0_(0, 0) = (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) / sqrt(2);
        ht_ee_wrt_0_(0, 1) = 0;
        ht_ee_wrt_0_(0, 2) = (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) / sqrt(2);
        ht_ee_wrt_0_(0, 3) = -(l2_ * sin(q_(HP))) + (l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2) - l3_ * sin(q_(HP) + q_(KP));
        ht_ee_wrt_0_(1, 0) = (sin(q_(HR)) * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 1) = -cos(q_(HR));
        ht_ee_wrt_0_(1, 2) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 3) = l1_ * cos(q_(HR)) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(2, 0) = (cos(q_(HR)) * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(2, 1) = -sin(q_(HR));
        ht_ee_wrt_0_(2, 2) = -((cos(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
        ht_ee_wrt_0_(2, 3) = l1_ * sin(q_(HR)) - (cos(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(3, 0) = 0;
        ht_ee_wrt_0_(3, 1) = 0;
        ht_ee_wrt_0_(3, 2) = 0;
        ht_ee_wrt_0_(3, 3) = 1;
        break;
    case BR:
        ht_ee_wrt_0_(0, 0) = (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) / sqrt(2);
        ht_ee_wrt_0_(0, 1) = 0;
        ht_ee_wrt_0_(0, 2) = (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) / sqrt(2);
        ht_ee_wrt_0_(0, 3) = -(l2_ * sin(q_(HP))) + (l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2) - l3_ * sin(q_(HP) + q_(KP));
        ht_ee_wrt_0_(1, 0) = (sin(q_(HR)) * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 1) = -cos(q_(HR));
        ht_ee_wrt_0_(1, 2) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(1, 3) = -(l1_ * cos(q_(HR))) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(2, 0) = (cos(q_(HR)) * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2);
        ht_ee_wrt_0_(2, 1) = -sin(q_(HR));
        ht_ee_wrt_0_(2, 2) = -((cos(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
        ht_ee_wrt_0_(2, 3) = -(l1_ * sin(q_(HR))) - (cos(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))))) / 2.;
        ht_ee_wrt_0_(3, 0) = 0;
        ht_ee_wrt_0_(3, 1) = 0;
        ht_ee_wrt_0_(3, 2) = 0;
        ht_ee_wrt_0_(3, 3) = 1;
        break;
    default:
        break;
    }
    rotation_ee_wrt_0_ = ht_ee_wrt_0_.block(0, 0, 3, 3);
    rotation_0_wrt_ee_ = rotation_ee_wrt_0_.transpose();
    xyz_(X) = ht_ee_wrt_0_(0, 3);
    xyz_(Y) = ht_ee_wrt_0_(1, 3);
    xyz_(Z) = ht_ee_wrt_0_(2, 3);
}
void RobotLeg::IK()
{
    double a1, b1, a2, b2, d, cos_del, del;
    double xe = xyz_d_(X);
    double ye = xyz_d_(Y);
    double ze = xyz_d_(Z);
    d = std::sqrt(xe * xe + ye * ye + ze * ze);
    a1 = atan2(ye, -ze);
    b1 = atan2(l1_, std::sqrt(ye * ye + ze * ze - l1_ * l1_));
    q_d_(HR) = a1 - b1;
    cos_del = (d * d - (l1_ * l1_ + l2_ * l2_ + l5_ * l5_)) / (2 * l2_ * l5_);
    del = atan2(-std::sqrt(1 - cos_del * cos_del), cos_del); //\C0\BD\BC\F6\B8\B8 \BB\E7\BF\EB
    q_d_(KP) = del - gam_;
    a2 = atan2(-xe, std::sqrt(ye * ye + ze * ze - l1_ * l1_));
    b2 = atan2(l5_ * std::sin(del), l2_ + l5_ * cos_del); // (-,+-)
    q_d_(HP) = a2 - b2;
}

void RobotLeg::SetImuSensor(double (&IMU)[3])
{
    is_ground_IMU_enable_ = true;
    imu_(0) = IMU[0], imu_(1) = IMU[1], imu_(2) = IMU[2];
    ground_roll_ = IMU[0], ground_pitch_ = IMU[1], ground_yaw_ = IMU[2];
    double d1_ = 0.5379; //m
    double d2_ = 0.1;
    double d3_ = 0.06248;
    double ground_yaw_error_ = -0.74; //rad
    rotation_ground_wrt_0_(0, 0) = std::cos(ground_pitch_) * std::cos(ground_yaw_error_ + ground_yaw_);
    rotation_ground_wrt_0_(0, 1) = std::cos(ground_yaw_error_ + ground_yaw_) * std::sin(ground_pitch_) * std::sin(ground_roll_) - std::cos(ground_roll_) * std::sin(ground_yaw_error_ + ground_yaw_);
    rotation_ground_wrt_0_(0, 2) = cos(ground_roll_) * cos(ground_yaw_error_ + ground_yaw_) * sin(ground_pitch_) + sin(ground_roll_) * sin(ground_yaw_error_ + ground_yaw_);
    rotation_ground_wrt_0_(1, 0) = -(cos(ground_pitch_) * sin(ground_yaw_error_ + ground_yaw_));
    rotation_ground_wrt_0_(1, 1) = -(cos(ground_roll_) * cos(ground_yaw_error_ + ground_yaw_)) - sin(ground_pitch_) * sin(ground_roll_) * sin(ground_yaw_error_ + ground_yaw_);
    rotation_ground_wrt_0_(1, 2) = cos(ground_yaw_error_ + ground_yaw_) * sin(ground_roll_) - cos(ground_roll_) * sin(ground_pitch_) * sin(ground_yaw_error_ + ground_yaw_);
    rotation_ground_wrt_0_(2, 0) = std::sin(ground_pitch_);
    rotation_ground_wrt_0_(2, 1) = -std::cos(ground_pitch_) * std::sin(ground_roll_);
    rotation_ground_wrt_0_(2, 2) = -std::cos(ground_pitch_) * std::cos(ground_roll_);

    translation_ground_wrt_0_(0) = -(d3_ * (cos(ground_roll_) * cos(ground_yaw_error_ + ground_yaw_) * sin(ground_pitch_) + sin(ground_roll_) * sin(ground_yaw_error_ + ground_yaw_)));
    translation_ground_wrt_0_(1) = d2_ + d3_ * (-(cos(ground_yaw_error_ + ground_yaw_) * sin(ground_roll_)) + cos(ground_roll_) * sin(ground_pitch_) * sin(ground_yaw_error_ + ground_yaw_));
    translation_ground_wrt_0_(2) = -d1_ + d3_ * std::cos(ground_pitch_) * std::cos(ground_roll_);

    rotation_0_wrt_ground_ = rotation_ground_wrt_0_.transpose();
    Eigen::Vector3d R_txO_minus = -rotation_0_wrt_ground_ * translation_ground_wrt_0_; //-Rt*P

    ht_ground_wrt_0_(0, 0) = rotation_ground_wrt_0_(0, 0);
    ht_ground_wrt_0_(1, 0) = rotation_ground_wrt_0_(1, 0);
    ht_ground_wrt_0_(2, 0) = rotation_ground_wrt_0_(2, 0);
    ht_ground_wrt_0_(3, 0) = 0.;
    ht_ground_wrt_0_(0, 1) = rotation_ground_wrt_0_(0, 1);
    ht_ground_wrt_0_(1, 1) = rotation_ground_wrt_0_(1, 1);
    ht_ground_wrt_0_(2, 1) = rotation_ground_wrt_0_(2, 1);
    ht_ground_wrt_0_(3, 1) = 0.;
    ht_ground_wrt_0_(0, 2) = rotation_ground_wrt_0_(0, 2);
    ht_ground_wrt_0_(1, 2) = rotation_ground_wrt_0_(1, 2);
    ht_ground_wrt_0_(2, 2) = rotation_ground_wrt_0_(2, 2);
    ht_ground_wrt_0_(3, 2) = 0.;
    ht_ground_wrt_0_(0, 3) = translation_ground_wrt_0_(0);
    ht_ground_wrt_0_(1, 3) = translation_ground_wrt_0_(1);
    ht_ground_wrt_0_(2, 3) = translation_ground_wrt_0_(2);
    ht_ground_wrt_0_(3, 3) = 1.;

    ht_0_wrt_ground_(0, 0) = rotation_0_wrt_ground_(0, 0);
    ht_0_wrt_ground_(1, 0) = rotation_0_wrt_ground_(1, 0);
    ht_0_wrt_ground_(2, 0) = rotation_0_wrt_ground_(2, 0);
    ht_0_wrt_ground_(3, 0) = 0.;
    ht_0_wrt_ground_(0, 1) = rotation_0_wrt_ground_(0, 1);
    ht_0_wrt_ground_(1, 1) = rotation_0_wrt_ground_(1, 1);
    ht_0_wrt_ground_(2, 1) = rotation_0_wrt_ground_(2, 1);
    ht_0_wrt_ground_(3, 1) = 0.;
    ht_0_wrt_ground_(0, 2) = rotation_0_wrt_ground_(0, 2);
    ht_0_wrt_ground_(1, 2) = rotation_0_wrt_ground_(1, 2);
    ht_0_wrt_ground_(2, 2) = rotation_0_wrt_ground_(2, 2);
    ht_0_wrt_ground_(3, 2) = 0.;
    ht_0_wrt_ground_(0, 3) = R_txO_minus(0);
    ht_0_wrt_ground_(1, 3) = R_txO_minus(1);
    ht_0_wrt_ground_(2, 3) = R_txO_minus(2);
    ht_0_wrt_ground_(3, 3) = 1.;
}
void RobotLeg::SetForceSensor(double (&force)[3])
{
    is_force_sensor_enable_ = true;
    if (fabs(force[2]) < 800)
    {
        force_sensor_(X) = force[X];
        force_sensor_(Y) = force[Y];
        force_sensor_(Z) = force[Z];
    }
    for (int i = 0; i < NUM_AXIS; i++)
    {
        force_sensor_(i) = force_SMA_[i].Filter(force_sensor_(i), 10);
        force_sensor_(i) = force_LPF_[i].Filter(force_sensor_(i), 5); //=force_sensor_[i];//

        force_sensor_dot_(i) = force_calculus_[i].Diff(force_sensor_(i)); //Automatically calculated force_sensor_ dot
        force_sensor_dot_(i) = force_dot_LPF_[i].Filter(force_sensor_dot_(i), 30);
    }
    force_sensor_wrt_0_ = rotation_ee_wrt_0_ * force_sensor_;

    //hysteresis On Off
    if (force_sensor_[Z] > 8)
        end_effector_state_ = STANCE;
    if (force_sensor_[Z] < 3)
        end_effector_state_ = SWING;
}
void RobotLeg::SetGroundForceSensor(double (&force)[3])
{
    is_ground_force_sensor_enable_ = true;
    if (fabs(force[2]) < 800)
    {
        ground_force_sensor_(X) = force[X];
        ground_force_sensor_(Y) = force[Y];
        ground_force_sensor_(Z) = force[Z];
    }
    for (int i = 0; i < NUM_AXIS; i++)
    {
        ground_force_sensor_(i) = force_SMA_[i].Filter(ground_force_sensor_(i), 10);
        ground_force_sensor_(i) = force_LPF_[i].Filter(ground_force_sensor_(i), 5); //=force_sensor_[i];//

        ground_force_sensor_dot_(i) = force_calculus_[i].Diff(ground_force_sensor_(i)); //Automatically calculated force_sensor_ dot
        ground_force_sensor_dot_(i) = force_dot_LPF_[i].Filter(ground_force_sensor_dot_(i), 30);
    }
    ground_force_sensor_wrt_0_ = rotation_ground_wrt_0_ * ground_force_sensor_;

    //hysteresis On Off
    if (ground_force_sensor_(Z) > 8)
        end_effector_state_ = STANCE;
    if (ground_force_sensor_(Z) < 3)
        end_effector_state_ = SWING;
}
void RobotLeg::SetDepthCamera(double (&point)[3], double (&normal)[3])
{
    is_detph_camera_enable_ = true;
    for (int i = 0; i < NUM_COOR; i++)
    {
        point_camera_[i] = point[i];
        normal_camera_[i] = normal[i];
        //		point_camera_[i]=PointSMA[i].Avg(point[i]);
        //		normal_camera_[i]=NormalSMA[i].Avg(normal[i]);
    }
}

void RobotLeg::XyzToXyzGround()
{
    xyz_wrt_ground_ = ht_0_wrt_ground_.block(0, 0, 2, 2) * xyz_ + ht_0_wrt_ground_.block(0, 3, 2, 3);
    xyz_dot_wrt_ground_ = rotation_0_wrt_ground_ * xyz_dot_;
    xyz_ddot_wrt_ground_ = rotation_0_wrt_ground_ * xyz_ddot_;
}

// //void RobotLeg::GXYZToXYZ(){
// //        XYZ_4x1=ht_ground_wrt_0_*G_XYZ_4x1;
// //        xyz_dot_=rotation_ground_wrt_0_*xyz_dot_wrt_ground_;
// //        xyz_ddot_=rotation_ground_wrt_0_*xyz_ddot_wrt_ground_;
// //        for(int i=0;i<NUM_AXIS;i++){
// //                xyz_[i]=xyz_(i)=XYZ_4x1(i);
// //                xyz_dot_[i]=xyz_dot_(i);
// //                xyz_ddot_[i]=xyz_ddot_(i);
// //        }
// //}
void RobotLeg::XyzdGroundToXyzd()
{
    xyz_d_ = rotation_ground_wrt_0_ * xyz_d_wrt_ground_ + translation_ground_wrt_0_;
    xyz_dot_d_ = rotation_ground_wrt_0_ * xyz_dot_d_wrt_ground_;
    xyz_ddot_d_ = rotation_ground_wrt_0_ * xyz_ddot_d_wrt_ground_;
}
void RobotLeg::ChangePointWrtG(double *point)
{
    Eigen::Vector4d p, Gp;
    p(0) = point[0];
    p(1) = point[1];
    p(2) = point[2];
    p(3) = 1;
    Gp = ht_0_wrt_ground_ * p;
    point[0] = Gp(0);
    point[1] = Gp(1);
    point[2] = Gp(2);
}
//void RobotLeg::ChangePointWrt0(double (&point)[NUM_COOR]){
//        Vector4d p,Gp;
//        Gp(0)=point[0];Gp(1)=point[1];Gp(2)=point[2];Gp(3)=1;
//        p=ht_ground_wrt_0_*Gp;
//        point[0]=p(0);point[1]=p(1);point[2]=p(2);
//}

void RobotLeg::Force_control(double Fx, double Fy, double Fz)
{
    Eigen::Vector3d force;
    force << Fx, Fy, Fz;
    target_torque_ = J_trans * force;
}

#endif /* ROBOTLEG_H_ */
