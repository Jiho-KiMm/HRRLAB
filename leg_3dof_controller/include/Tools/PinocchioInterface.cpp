#include "PinocchioInterface.hpp"

void PinocchioInterface::Initialize()
{
    std::cout<<"Pinocchio Init"<<std::endl;

    if(check == 0) 
    {
        std::cout << "Floating base" << std::endl;
        std::cout << pinocchio::neutral(_model) << std::endl;

        n = Leg_N*(DoF + 6);


        _q.setZero(n);
        _dq.setZero(n);
        _ddq.setZero(n);

        _J.setZero(6, n);
        _dJ.setZero(6, n);
    }
    else if(check == 1)  
    {
        std::cout << "Fixed base" << std::endl;

        n = Leg_N*DoF;

        _q.setZero(n);
        _dq.setZero(n);
        _ddq.setZero(n);

        _J.setZero(6, n);
        _dJ.setZero(6, n);
    }
    else std::cout << "Wrong base" << std::endl;

    std::cout<<"n : "<<n<<std::endl;

    target_torque_.setZero(DoF);
}

void PinocchioInterface::SetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    _q = q;
    _dq = dq;

    for (size_t i = 0; i < Leg_N; i++)
    {
        for (size_t j = 0; j < DoF; j++)
        {
            // _ddq(i + j * DoF) = vel_diff[i][j].Diff(_dq(i + j * DoF));
            _ddq(j) = vel_diff[i][j].Diff(_dq(j));
        }
    }

    for (size_t i = 0; i < Leg_N; i++)
    {
        q_[i] = _q.block<3, 1>(i * DoF, 0); //Eigen .block
        dq_[i] = _dq.block<3, 1>(i * DoF, 0);
        ddq_[i] = _ddq.block<3, 1>(i * DoF, 0);
    }

    pinocchio::forwardKinematics(_model, _data, _q);
    pinocchio::computeJointJacobians(_model, _data, _q);
    pinocchio::computeJointJacobiansTimeVariation(_model, _data, _q, _dq);
    pinocchio::computeAllTerms(_model, _data, _q, _dq);

    // std::cout << "ddq_ : " << ddq_[0] << std::endl;
}


void PinocchioInterface::SetKinematics(int legtype)
{
    // EE xyz 값 뽑는 코드
    int legtype_ = legtype;

    frame_id = _model.getFrameId(foot_name_[legtype_]);

    pinocchio::updateFramePlacement(_model, _data, frame_id);

    auto pos = _data.oMf[frame_id];

    if (DoF == 3)
        _pos << pos.translation();

    // Foot Position
    pos_[legtype_] = _pos;

    //....................................

    SetJacobian(legtype_);

    // Foot Velocity
    vel_[legtype_] = GetJacobian(legtype_) * dq_[legtype_];

    // Foot Acceleration
    acc_[legtype_] = GetJacobianDot(legtype_) * dq_[legtype_] + GetJacobian(legtype_) * ddq_[legtype_];
}

void PinocchioInterface::SetJacobian(int legtype)
{
    int legtype_ = legtype;

    pinocchio::getFrameJacobian(_model, _data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _J);

    pinocchio::getFrameJacobianTimeVariation(_model, _data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _dJ);

    Eigen::Matrix<double, 6,DoF > J[Leg_N];
    Eigen::Matrix<double, 6,DoF> dJ[Leg_N];

    // std::cout << "_J : " << _J << std::endl;s

    // Jacobian
    J[legtype_] = _J.block<6,DoF>(0, legtype_ * DoF);
    J_[legtype_] << J[legtype_].block<3, 3>(0, 0);
        // J[legtype_].block<1, 4>(4, 0);

    // Jacobian Dot
    dJ[legtype_] = _dJ.block<6,DoF>(0, legtype_ * DoF);
    dJ_[legtype_] << dJ[legtype_].block<3, 3>(0, 0);
        // dJ[legtype_].block<1, 4>(4, 0);

    // std::cout << "J_ : " << J_[0] << std::endl;
}

//jiho

void PinocchioInterface::SetGravity()
{
    pinocchio::computeGeneralizedGravity(_model, _data, _q);
    r_G_ = _data.g;
}


void PinocchioInterface::SetDynamics(Eigen::Vector3d pos_d, Eigen::Vector3d vel_d, Eigen::Vector3d acc_d, Eigen::Matrix3d Kp, Eigen::Matrix3d Kd)
{

    Kp_ = Kp;
    Kd_ = Kd;

    Eigen::VectorXd _a;

    _a.setZero(n);

    // // RNE [M, C, G, b]s
    pinocchio::crba(_model, _data, _q);
    // pinocchio::rnea(_model, _data, _q, _dq, _a);
    pinocchio::computeGeneralizedGravity(_model, _data, _q);
    pinocchio::computeCoriolisMatrix(_model, _data, _q, _dq);

    r_M_ = _data.M;
    // r_T_ = _data.tau;
    r_B_ = _data.nle;
    r_G_ = _data.g;
    // r_C_ = _data.C;

    // std::cout << "Cqdot + G" << r_C_*dq_[0] + r_G_ << std::endl;
    
    if(check == 0) 
    {
        for (size_t i = 0; i < Leg_N; i++)
        {
            // Inertia Matrix
            M_[i] = r_M_.block<3, 3>(0, 6);

            // Nonlinear Effects Matrix
            B_[i] = r_B_.block<3, 1>(6, 0);

            // tau Matrix
            // T_[i] = r_T_.block<DoF, 1>(i * Leg_N, 0);

            // // Gravity Matrix
            G_[i] = r_G_.block<3, 1>(6, 0);

            // Desired Pos, Vel, Acc
            pos_d_[i] = pos_d;
            vel_d_[i] = vel_d;
            acc_d_[i] = acc_d;

            // Error beween Des & Act
            err_[i] = pos_d_[i] - pos_[i];
            err_dot_[i] = vel_d_[i] - vel_[i];
        }
    }
    else if(check == 1)  
    {
        for (size_t i = 0; i < Leg_N; i++)
        {
            // Inertia Matrix
            M_[i] = r_M_.block<DoF, DoF>(i * Leg_N, i * Leg_N);

            // Nonlinear Effects Matrix
            B_[i] = r_B_;

            // tau Matrix
            // T_[i] = r_T_.block<DoF, 1>(i * Leg_N, 0);

            // // Gravity Matrix
            G_[i] = r_G_;

            // Desired Pos, Vel, Acc
            pos_d_[i] = pos_d;
            vel_d_[i] = vel_d;
            acc_d_[i] = acc_d;

            // Error beween Des & Act
            err_[i] = pos_d_[i] - pos_[i];
            err_dot_[i] = vel_d_[i] - vel_[i];
        }
    }

    // pinocchio::computeCentroidalMomentum(_model,_data,_q,_dq);

    // std::cout<<"B : "<<B_<<"\n"<<std::endl;
}


Eigen::VectorXd PinocchioInterface::GetDynamics()
{
    Eigen::Matrix<double, DoF, 1> tmp_torque[Leg_N];

    for (size_t i = 0; i < Leg_N; i++)
    {
        // CTC
        tmp_torque[i] = M_[i] * J_[i].inverse() * (acc_d_[i] + Kp_ * err_[i] + Kd_ * err_dot_[i] - dJ_[i] * dq_[i]) + B_[i];

        target_torque_.block<DoF, 1>(i * DoF, 0) = tmp_torque[i];

        // std::cout << "M_[i] : " << M_[0] << std::endl;
        // std::cout << "J_[i].inverse() : " << J_[0].inverse() << std::endl;
        // std::cout << "acc_d_[i] : " << acc_d_[0] << std::endl;
        // std::cout << "err_[i] : " << err_[0] << std::endl;
        // std::cout << "err_dot_[i] : " << err_dot_[0] << std::endl;

        // std::cout << "dJ_[i] : " << dJ_[0] << std::endl;
        // std::cout << "dq_[i] : " << dq_[0] << std::endl;
        // std::cout << "B_[i] : " << B_[0] << std::endl;
        // std::cout << "target_torque_ : " << target_torque_ << std::endl;
    }
    // target_torque_.setZero(3);
    // std::cout << "target_torque_ : " << tmp_torque[0] << std::endl;
    return target_torque_;
}

void PinocchioInterface::createFloatingBaseModel(const std::string& urdfFilePath)
{
    // JointModelComposite 객체 생성 및 조인트 추가

    pinocchio::JointModelComposite jointComposite(2);
    // // jointComposite.addJoint(pinocchio::JointModelFreeFlyer());
    jointComposite.addJoint(pinocchio::JointModelTranslation());
    jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

    // pinocchio::urdf::buildModel(urdfFilePath, pinocchio::JointModelFreeFlyer(), _model);

    // 로봇 모델 빌드
    pinocchio::urdf::buildModel(urdfFilePath, jointComposite, _model);
}

// Floating Base Control

void PinocchioInterface::FloatingBaseSetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    _q = q;
    _dq = dq;

    

    for (size_t j = 0; j < DoF + 6; j++)
    {
        // _ddq(i + j * DoF) = vel_diff[i][j].Diff(_dq(i + j * DoF));
        _ddq(j) = vel_diff_floating[0][j].Diff(_dq(j));
    }

    q_[0] = _q.block<3,1>(6,0);
    dq_[0] = _dq.block<3,1>(6,0);
    ddq_[0] = _ddq.block<3,1>(6,0);

    pinocchio::forwardKinematics(_model, _data, _q);
    pinocchio::computeJointJacobians(_model, _data, _q);

    pinocchio::computeJointJacobiansTimeVariation(_model, _data, _q, _dq);
    pinocchio::computeAllTerms(_model, _data, _q, _dq);
    // pinocchio::computeFrameJacobian(_model, _data, _q, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    // std::cout << "model.nq" << _model.nq << std::endl; 
}

void PinocchioInterface::FloatingBaseSetKinematics(int legtype)
{
    // EE xyz 값 뽑는 코드
    int legtype_ = legtype;

    frame_id = _model.getFrameId(foot_name_[legtype_]);

    pinocchio::updateFramePlacement(_model, _data, frame_id);

    pinocchio::forwardKinematics(_model, _data, _q);

    auto pos = _data.oMf[frame_id];

    if (DoF == 3)
        _pos << pos.translation();

    // Foot Position
    pos_[legtype_] = _pos;

    //....................................

    FloatingBaseSetJacobian(legtype_);

    // // Foot Velocity
    vel_[legtype_] = GetJacobian(legtype_) * dq_[legtype_];

    // // // Foot Acceleration
    acc_[legtype_] = GetJacobianDot(legtype_) * dq_[legtype_] + GetJacobian(legtype_) * ddq_[legtype_];
}

void PinocchioInterface::FloatingBaseSetJacobian(int legtype)
{
    int legtype_ = legtype;

    // pinocchio::computeJointJacobians(_model, _data);

    pinocchio::getFrameJacobian(_model, _data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _J);

    pinocchio::getFrameJacobianTimeVariation(_model, _data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _dJ);

    // std::cout << "_J : " << _J << std::endl;
    // Jacobian
    
    J_[0] = _J.block<3,3>(0,6);

    // Jacobian Dot
    dJ_[0] = _dJ.block<3,3>(0,6);


}


// 진우형 코드
// void PinocchioInterface::SetDynamics(Eigen::MatrixXd pos_d, Eigen::MatrixXd vel_d, Eigen::MatrixXd acc_d, Eigen::Matrix3d Kp, Eigen::Matrix3d Kd)
// {
//     Kp_ = Kp;
//     Kd_ = Kd;

//     Eigen::VectorXd _a;

//     _a.setZero(n);

//     // // RNE [M, C, G, b]s
//     pinocchio::crba(_model, _data, _q);
//     // pinocchio::rnea(_model, _data, _q, _dq, _a);
//     pinocchio::computeGeneralizedGravity(_model, _data, _q);
//     pinocchio::computeCoriolisMatrix(_model, _data, _q, _dq);

//     r_M_ = _data.M;
//     r_T_ = _data.tau;
//     // r_B_ = _data.nle;
//     r_G_ = _data.g;
//     r_C_ = _data.C;

//     std::cout << "M :" << _data.M << std::endl;
//     std::cout << "C :" << _data.C< < std::endl;

//     // for (size_t i = 0; i < Leg_N; i++)
//     // {
//     //     // Inertia Matrix
//     //     M_[i] = r_M_.block<DoF, DoF>(i * Leg_N, i * Leg_N);
//     //     // M_[i].triangularView<Eigen::Lower>() = M_[i].transpose();

//     //     // Nonlinear Effects Matrix
//     //     B_[i] = r_B_.block<DoF, 1>(i * Leg_N, 0);

//     //     // tau Matrix
//     //     T_[i] = r_T_.block<DoF, 1>(i * Leg_N, 0);

//     //     // // Coriolis Matrix
//     //     // C_[i] = r_C_.block<DoF, DoF>(i * DoF, i * DoF);

//     //     // // Gravity Matrix
//     //     // G_[i] = r_G_.block<DoF, 1>(i * DoF, 0);

//     //     // Desired Pos, Vel, Acc
//     //     pos_d_[i] = pos_d.row(i);
//     //     vel_d_[i] = vel_d.row(i);
//     //     acc_d_[i] = acc_d.row(i);

//     //     // Error beween Des & Act
//     //     err_[i] = pos_d_[i] - pos_[i];
//     //     err_dot_[i] = vel_d_[i] - vel_[i];
//     // }

//     // pinocchio::computeCentroidalMomentum(_model,_data,_q,_dq);

//     // std::cout<<"CM : "<<_data.hg<<"\n"<<std::endl;
// }


// Eigen::VectorXd PinocchioInterface::GetDynamics()
// {
//     Eigen::Matrix<double, DoF, 1> tmp_torque[Leg_N];
//     const double torque_limit_=300;

//     for (size_t i = 0; i < Leg_N; i++)
//     {
//         // CTC
//         tmp_torque[i] = M_[i] * J_[i].inverse() * (acc_d_[i] + Kp_ * err_[i] + Kd_ * err_dot_[i] - dJ_[i] * dq_[i]) + B_[i];

//         for (size_t j = 0; j < DoF; j++)
//         {
//             if(tmp_torque[i](j,0)>=torque_limit_)
//             {
//                 tmp_torque[i](j,0)=300;
//                 std::cout<<"Leg ["<<i<<"] "<<" joint ["<<j<<"] torque Max"<<std::endl;
//             }
//             if(tmp_torque[i](j,0)<=-torque_limit_)
//             {
//                 tmp_torque[i](j,0)=-300;
//                 std::cout<<"Leg ["<<i<<"] "<<" joint ["<<j<<"] torque Min"<<std::endl;
//             }
//         }
//         target_torque_.block<DoF, 1>(i * DoF, 0) = tmp_torque[i];
//     }

//     return target_torque_;
// }

void PinocchioInterface::PrintPin()
{
    pinocchio::crba(_model, _data, _q);
    // pinocchio::rnea(_model, _data, _q, _dq, _a);
    pinocchio::computeGeneralizedGravity(_model, _data, _q);
    pinocchio::computeCoriolisMatrix(_model, _data, _q, _dq);

    r_M_ = _data.M;
    // r_T_ = _data.tau;
    r_B_ = _data.nle;
    r_G_ = _data.g;

    std::cout << "r_B_ : " << r_B_ << std::endl;
}