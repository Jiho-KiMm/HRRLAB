#include "RobotLeg.h"


void RobotLeg::init(const double freq)
{
    dt_ = 1/freq;
    kp_.setZero(DoF);
    kd_.setZero(DoF);
    init_q_.setZero(DoF);
    ref_q_.setZero(DoF);
    act_q_.setZero(DoF);
    ref_qdot_.setZero(DoF);
    act_qdot_.setZero(DoF);
    act_qdot_.setZero(DoF);
    torque_.setZero(DoF);
    error_x_.setZero(DoF);
    error_xdot_.setZero(DoF);

    act_x_vel.setZero(DoF);

    ref_xdot_.setZero(DoF);
    act_xdot_.setZero(DoF);
    torque_x_.setZero(DoF);
    J_.setZero(DoF,DoF);
    for (int i = 0; i < DoF; i++)
    {
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
    }
    l1_ = 0.1;
    ankle_angle_ = 0.785398;
    l2_ = 0.305;
    l3_ = 0.27;
    l4_ = 0.045;
    l5_ = 0.30349250;
    gam_ = 0.10503849; //6.01286deg
}

void RobotLeg::SetLegInit(Eigen::VectorXd init_q)
{
    if (!isLegInit)
    {
        init_q_ = init_q;
        isLegInit = true;
    }
}

void RobotLeg::SetPDControl_q(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_q, Eigen::VectorXd act_q, Eigen::VectorXd ref_qdot, Eigen::VectorXd act_qdot)
{
    for (size_t i = 0; i < DoF; i++)
    {
        kp_(i) = kp(i);
        kd_(i) = kd(i);
        ref_q_(i) = ref_q(i);
        act_q_(i) = act_q(i);
        ref_qdot_(i) = ref_qdot(i);
        act_qdot_(i) = act_qdot(i);
    }

    for (size_t i = 0; i < DoF; i++)
    {
        torque_(i) = kp_(i) * (ref_q_(i) - act_q_(i)) + kd_(i) * (ref_qdot_(i) - act_qdot_(i));
    }
}

void RobotLeg::SetPDControl_X(Eigen::MatrixXd kp, Eigen::MatrixXd kd, Eigen::VectorXd ref_x, Eigen::VectorXd act_x, Eigen::MatrixXd J)
{
    int cnt_=0;
    error_x_ = ref_x-act_x;
    J_ = J;
    kp_T = kp;
    kd_T = kd;



    for (size_t i = 0; i < DoF; i++)
    {
        // error_xdot_(i) = pd_error_xdot[i].Diff(error_x_(i));

        act_x_vel(i) = pd_error_xdot[i].Diff(act_x(i));
    }

   

    for (size_t i = 0; i < DoF; i++)
    {
        torque_x_(i) = kp_T(i, i)*error_x_(i) - kd_T(i, i)*act_x_vel(i);
    }
    // std::cout<<"act_x: "<<act_x<<std::endl;
    // std::cout<<"torque_x_ : "<<torque_x_<<"\n"<<std::endl;

    torque_ = J_.transpose()*torque_x_;

    // std::cout<<"error_x_: "<<error_x_<<std::endl;
    // std::cout<<"act_x_vel : "<<act_x_vel<<"\n"<<std::endl;

    // std::cout<<"J_.transpose(): "<<J_.transpose()<<std::endl;
    // std::cout<<"torque_x_ : "<<torque_x_<<"\n"<<std::endl;

    // std::cout<<"torque_ : "<<torque_<<"\n"<<std::endl;

}






    //경환이형 코드..................
void RobotLeg::FK()
{
    ht_ee_wrt_0_(0, 0) = (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))) / sqrt(2);
    ht_ee_wrt_0_(0, 1) = 0;
    ht_ee_wrt_0_(0, 2) = -((cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP))) / sqrt(2));
    //x
    ht_ee_wrt_0_(0, 3) = -(l2_ * sin(q_(HP))) - l3_ * sin(q_(HP) + q_(KP)) - l4_ * sin(ankle_angle_ + q_(HP) + q_(KP)); //-(l2_ * sin(q_(HP))) - l3_ * sin(q_(HP) + q_(KP)) - (l4_ * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);

    ht_ee_wrt_0_(1, 0) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
    ht_ee_wrt_0_(1, 1) = -cos(q_(HR));
    ht_ee_wrt_0_(1, 2) = (sin(q_(HR)) * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP)))) / sqrt(2);
    //y
    ht_ee_wrt_0_(1, 3) = l1_ * cos(q_(HR)) + (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + l4_ * cos(ankle_angle_ + q_(HP) + q_(KP))) * sin(q_(HR)); //l1_ * cos(q_(HR)) + (sin(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;

    ht_ee_wrt_0_(2, 0) = -((cos(q_(HR)) * (cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2));
    ht_ee_wrt_0_(2, 1) = -sin(q_(HR));
    ht_ee_wrt_0_(2, 2) = (cos(q_(HR)) * (-cos(q_(HP) + q_(KP)) + sin(q_(HP) + q_(KP)))) / sqrt(2);
    //z
    ht_ee_wrt_0_(2, 3) = -(cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + l4_ * cos(ankle_angle_ + q_(HP) + q_(KP)))) + l1_ * sin(q_(HR)); //l1_ * sin(q_(HR)) - (cos(q_(HR)) * (2 * l2_ * cos(q_(HP)) + 2 * l3_ * cos(q_(HP) + q_(KP)) + sqrt(2) * l4_ * (cos(q_(HP) + q_(KP)) - sin(q_(HP) + q_(KP))))) / 2.;

    ht_ee_wrt_0_(3, 0) = 0;
    ht_ee_wrt_0_(3, 1) = 0;
    ht_ee_wrt_0_(3, 2) = 0;
    ht_ee_wrt_0_(3, 3) = 1;
    rotation_ee_wrt_0_ = ht_ee_wrt_0_.block(0, 0, 3, 3);
    rotation_0_wrt_ee_ = rotation_ee_wrt_0_.transpose();
    xyz_(X) = ht_ee_wrt_0_(0, 3);
    xyz_(Y) = ht_ee_wrt_0_(1, 3);
    xyz_(Z) = ht_ee_wrt_0_(2, 3);
    
    // xyz_(X) = -(l2_ * sin(q_(HP))) - l3_ * sin(q_(HP) + q_(KP)) - l4_ * sin(ankle_angle_ + q_(HP) + q_(KP));
    // xyz_(Y) = l1_ * cos(q_(HR)) + (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + l4_ * cos(ankle_angle_ + q_(HP) + q_(KP))) * sin(q_(HR));
    // xyz_(Z) = -(cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + l4_ * cos(ankle_angle_ + q_(HP) + q_(KP)))) + l1_ * sin(q_(HR));
}

void RobotLeg::CalcJacobian()
{
    J(0, 0) = 0.;
    J(0, 1) = -(l2_ * cos(q_(HP))) - l3_ * cos(q_(HP) + q_(KP)) - l4_ * cos(ankle_angle_ + q_(HP) + q_(KP));
    J(0, 2) = -(l3_ * cos(q_(HP) + q_(KP))) - l4_ * cos(ankle_angle_ + q_(HP) + q_(KP));
    J(1, 0) = cos(q_(HR)) * (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + l4_ * cos(ankle_angle_ + q_(HP) + q_(KP))) - l1_ * sin(q_(HR));
    J(1, 1) = -(sin(q_(HR)) * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP)) + l4_ * sin(ankle_angle_ + q_(HP) + q_(KP))));
    J(1, 2) = -(sin(q_(HR)) * (l3_ * sin(q_(HP) + q_(KP)) + l4_ * sin(ankle_angle_ + q_(HP) + q_(KP))));
    J(2, 0) = l1_ * cos(q_(HR)) + (l2_ * cos(q_(HP)) + l3_ * cos(q_(HP) + q_(KP)) + l4_ * cos(ankle_angle_ + q_(HP) + q_(KP))) * sin(q_(HR));
    J(2, 1) = cos(q_(HR)) * (l2_ * sin(q_(HP)) + l3_ * sin(q_(HP) + q_(KP)) + l4_ * sin(ankle_angle_ + q_(HP) + q_(KP)));
    J(2, 2) = cos(q_(HR)) * (l3_ * sin(q_(HP) + q_(KP)) + l4_ * sin(ankle_angle_ + q_(HP) + q_(KP)));

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
    // std::cout<<"JJJ : "<< J <<std::endl;
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
    del = atan2(-std::sqrt(1 - cos_del * cos_del), cos_del); 
    q_d_(KP) = del - gam_;
    a2 = atan2(-xe, std::sqrt(ye * ye + ze * ze - l1_ * l1_));
    b2 = atan2(l5_ * std::sin(del), l2_ + l5_ * cos_del);
    q_d_(HP) = a2 - b2;
}

void RobotLeg::SetJointAngle(double q1, double q2, double q3)
{
    q_(X) = q1;
    q_(Y) = q2;
    q_(Z) = q3;
}

void RobotLeg::SetDesJointXyz(double q1_d, double q2_d, double q3_d)
{
    q_d_(X) = q1_d;
    q_d_(Y) = q2_d;
    q_d_(Z) = q3_d;
}
void RobotLeg::SetDesJointXyzDot(double q1_dot_d, double q2_dot_d, double q3_dot_d)
{
    q_dot_d_(X) = q1_dot_d;
    q_dot_d_(Y) = q2_dot_d;
    q_dot_d_(Z) = q3_dot_d;
}
void RobotLeg::SetDesXyz(double x_d, double y_d, double z_d)
{
    xyz_d_(X) = x_d;
    xyz_d_(Y) = y_d;
    xyz_d_(Z) = z_d;
}
void RobotLeg::SetDesXyzDot(double x_dot_d, double y_dot_d, double z_dot_d)
{
    xyz_dot_d_(X) = x_dot_d;
    xyz_dot_d_(Y) = y_dot_d;
    xyz_dot_d_(Z) = z_dot_d;
}



void RobotLeg::TaskSpaceCTC(Eigen::Matrix3d M, Eigen::Matrix3d C, Eigen::Vector3d G)
{
    // Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Me; // task-space inertia
    // Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kd; // differential gain
    // Eigen::Matrix<double, NUM_AXIS, NUM_AXIS> Kp; // proportional gain

    // // Me = J_trans_inv * M * J_inv; 
    // Kd.setZero();
    // Kp.setZero();
    // // double omega=10; // natural frequency
    // // double damp=0.707; // damping ratio
    // for (size_t i = 0; i < 3; i++)
    // {
    //     Kp(i, i) = 10000;
    //     Kd(i, i) = 100;
    // }
    // xyz_ddot_ctc_ =  xyz_ddot_d_+Kd * (xyz_dot_d_ - xyz_dot_) + Kp * (xyz_d_ - xyz_); //task-space CTC acceleration 
    // q_ddot_ctc_ = J_inv * (xyz_ddot_ctc_ - J_dot * q_dot_); //joint-space CTC acceleration 
    // tor_M_ = M * q_ddot_ctc_; // inertia torque
    // tor_C_ = C * q_dot_; // coriolis and centrifugal torque 
    // tor_G_= G; // gravity torque
    
    // tor_ctc_ = tor_M_ + tor_C_ + tor_G_; //calculate CTC torque
    // torque_ = tor_ctc_; //calculate target torque
}
//.................................