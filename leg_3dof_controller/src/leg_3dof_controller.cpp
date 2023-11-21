#include "leg_3dof_controller.h"


void leg_3dof_controller::init()
{
    // Reset Gazebo Simualtion
    system("clear");
    std_srvs::Empty reset;
    ros::service::call("/gazebo/reset_simulation", reset);
    std::cout << "Reset Gazebo Simulation" << std::endl;
    nh_.ok();

    uint32_t queue_size = 1;

    // Subscribers
    sub_leg_state_ = nh_.subscribe(topic_leg_state_, queue_size, &leg_3dof_controller::state_leg_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Publisher
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_leg_command_, queue_size);


    // init size
    init_size = 3;

    if(base_ == "fixed_base")
    {
        q_.setZero(init_size);
        dq_.setZero(init_size);
    }
    else if(base_ == "floating_base")
    {
        fq_.setZero(9);
        fdq_.setZero(9);

        floating_q_.setZero(9);
        floating_dq_.setZero(9);

        q_.setZero(init_size);
        dq_.setZero(init_size);
    }

    torque_.setZero(init_size);
    homing_ref_q.setZero(DoF);
    homing_Kp.setZero(DoF);
    homing_Kd.setZero(DoF);

    ref_x.setZero(init_size);
    x_.setZero(init_size);
    x_0_ref.setZero(init_size);
    x_0.setZero(init_size);
    error_x_.setZero(init_size);


    ref_x2.setZero(init_size);
    top_ref_x.setZero(init_size);

    controlmode = INIT;
    taskspace = MOVING1;
    Recieved_Joint_State = false;

    //jiho.....................
    flag_ = false;
    flag_2 = false;
    t_ = 0;
    sub_t_ = 0;
    count_ = 0;

    N = 0;
    //........................
    traj[0].init(freq_);
    robotleg[0].init(freq_);
    

    pin.Initialize();
    std::cout<<"Init Setting"<<std::endl;

    homing_ref_q << 0*deg2rad, 45*deg2rad, -90*deg2rad;
    homing_Kp << 50, 100, 50;
    homing_Kd << 2, 2, 2;

    kp << 500, 0.0, 0.0,
        0.0, 500, 0.0,
        0.0, 0.0, 500;

    kd << 0.2, 0.0, 0.0,
        0.0, 0.2, 0.0,
        0.0, 0.0, 0.2;

    ctc_kp << 500, 0.0, 0.0,
        0.0, 500, 0.0,
        0.0, 0.0, 500;

    ctc_kd << 20, 0.0, 0.0,
        0.0, 20, 0.0,
        0.0, 0.0, 20;

    init_rqt();
}

void leg_3dof_controller::command(bool flag)
{
    if (flag)
    {
        pin.SetRobotParameter(q_, dq_);
        pin.SetGravity();
        generalized_gravity = pin.GetGravityCompensation();
        pin.SetKinematics(0);

        EEPos = pin.GetPos(0);
        J_ = pin.GetJacobian(0);

        switch(controlmode)
        {
        case INIT:
            if(t_ >= 4) ChangeControlState();
            break;
        
        case HOMING:
            HomingControl();
            if(t_ >= 7) 
            {
                controlmode = TaskSpacePD;
                traj[0].ResetPeriod();


                x_0 = EEPos;
                x_ = EEPos;
                flag_ = false;
            }
            break;
        case TaskSpacePD:
            // SimpleTaskSpacePDControl();
            SimpleTaskSpaceCTCControl();
            break;
        case FINISH:
            // std::cout << "FINISH " << std::endl;
            break;
        }

        std::cout << "EEPos : " << EEPos - x_0 << std::endl;

        send_commands_to_robot();
    }

    else
    {
        std::cout << "Not Connected Please Wait..." << std::endl;
    }
}

void leg_3dof_controller::send_commands_to_robot()
{
    std_msgs::Float64MultiArray msg;

    for (size_t i = 0; i < 3; i++)
    {
        if (controlmode == INIT || controlmode == FINISH)
        {
            if(base_ == "fixed_base")
            {
                torque_ = generalized_gravity; 
                msg.data.push_back(torque_(i));
            }
            else if(base_ == "floating_base")
            {
                torque_ = generalized_gravity;
                msg.data.push_back(torque_(i));
            }
        }
        else
        {
            msg.data.push_back(torque_(i));
        }

        // msg.data.push_back(torque_(i));
    }

    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}

void leg_3dof_controller::run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1 / freq_);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            // Update the last control time
            last_control_time = current_time;
            t_ += 1 / freq_ ;

            // Perform control actions here -> 코딩 하다가 가끔씩 켜서 hz 확인
            // ROS_INFO("Control loop running");
            if(base_ == "fixed_base")   command(Recieved_Joint_State);
            else if(base_ == "floating_base")   FloatingBasecommand(Recieved_Joint_State);
            
            // rqt
            rqt_plot();
            
            // Sleep to enforce the desired control loop frequency
            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }
    // FILE*Jacodata = fopen("/home/jiho/example_ws/src/Jacodatas.csv", "w");
    // for(int i = 0; i < N ; i++)
    // {
    //     for(int j = 0; j < 3 ; j++)
    //     {
    //         if(j!=3) fprintf(Jacodata, "%f," , Jacodatas[i][j]);
    //         else fprintf(Jacodata, "%f\n" , Jacodatas[i][j]);
    //     }
    // }
    // fclose(Jacodata);

    std::cout << "Control Finish" << std::endl;
}

void leg_3dof_controller::state_leg_callback(const sensor_msgs::JointState &state)
{

    if (base_ == "fixed_base")
    {
        for (size_t i = 0; i < 3; i++)
        {
            q_(i) = state.position[i];
            dq_(i) = state.velocity[i];

            Recieved_Joint_State = true;
        }
    }
    else if (base_ == "floating_base")
    {
        for (size_t i = 0; i < 9; i++)
        {
            fq_(i) = state.position[i];
            fdq_(i) = state.velocity[i];

            Recieved_Joint_State = true;
        }
        Eigen::VectorXd sq_, sdq_;
        sq_ = fq_.block<6,1>(3,0);
        sdq_ = fdq_.block<6,1>(3,0);

        for (size_t i = 0; i < 6; i++)
        {
            floating_q_(i) = sq_(i);
            floating_dq_(i) = sdq_(i);
        }
        for (size_t i = 0; i < 3; i++)
        {
            floating_q_(i + 6) = fq_(i);
            floating_dq_(i + 6) = fdq_(i);
        }
        for (size_t i = 0; i < 3; i++)
        {
            q_(i) = fq_(i);
            dq_(i) = fdq_(i);
        }
    }
}

void leg_3dof_controller::init_rqt()
{
    // Joint Angle Publisher
    p_f1_HR_q = nh_plot.advertise<std_msgs::Float64>("/HR/",1);
    p_f1_HP_q = nh_plot.advertise<std_msgs::Float64>("/HP/",1);
    p_f1_KP_q = nh_plot.advertise<std_msgs::Float64>("/KP/",1);

    p_f1_HR_q_ref = nh_plot.advertise<std_msgs::Float64>("/HR_ref/",1);
    p_f1_HP_q_ref = nh_plot.advertise<std_msgs::Float64>("/HP_ref/",1);
    p_f1_KP_q_ref = nh_plot.advertise<std_msgs::Float64>("/KP_ref/",1);
    

    // Joint Angular Velocity Publisher
    p_f1_HR_dq = nh_plot.advertise<std_msgs::Float64>("/HR_dq/",1);
    p_f1_HP_dq = nh_plot.advertise<std_msgs::Float64>("/HP_dq/",1);
    p_f1_KP_dq = nh_plot.advertise<std_msgs::Float64>("/KP_dq/",1);

    // Ref Foot Position Publisher
    p_f1_ref_x = nh_plot.advertise<std_msgs::Float64>("/f1_ref_x/",1);
    p_f1_ref_y = nh_plot.advertise<std_msgs::Float64>("/f1_ref_y/",1);
    p_f1_ref_z = nh_plot.advertise<std_msgs::Float64>("/f1_ref_z/",1);
    // p_f1_ref_rp = nh_plot.advertise<std_msgs::Float64>("/f1_ref_rp/",1);

    // Act Foot Position Publisher
    p_f1_act_x = nh_plot.advertise<std_msgs::Float64>("/f1_act_x/",1);
    p_f1_act_y = nh_plot.advertise<std_msgs::Float64>("/f1_act_y/",1);
    p_f1_act_z = nh_plot.advertise<std_msgs::Float64>("/f1_act_z/",1);
//     p_f1_act_rp = nh_plot.advertise<std_msgs::Float64>("/f1_act_rp/",1);

    // Joint Torque Publisher
    p_f1_HR_torque = nh_plot.advertise<std_msgs::Float64>("/HR_torque/",1);
    p_f1_HP_torque = nh_plot.advertise<std_msgs::Float64>("/HP_torque/",1);
    p_f1_KP_torque = nh_plot.advertise<std_msgs::Float64>("/KP_torque/",1);

//     // Joint Power Publisher
//     p_f1_hy_power = nh_plot.advertise<std_msgs::Float64>("/f1_hy_power/",1);
//     p_f1_hp_power = nh_plot.advertise<std_msgs::Float64>("/f1_hp_power/",1);
//     p_f1_hr_power = nh_plot.advertise<std_msgs::Float64>("/f1_hr_power/",1);
//     p_f1_hy_power = nh_plot.advertise<std_msgs::Float64>("/f1_kr_power/",1);
}

void leg_3dof_controller::rqt_plot()
{
    // Joint Angle
    m_f1_HR_q.data = q_(HR);
    m_f1_HP_q.data = q_(HP);
    m_f1_KP_q.data = q_(KP);

    m_f1_HR_q_ref.data = homing_ref_q(0);
    m_f1_HP_q_ref.data = homing_ref_q(1);
    m_f1_KP_q_ref.data = homing_ref_q(2);

    p_f1_HR_q.publish(m_f1_HR_q);
    p_f1_HP_q.publish(m_f1_HP_q);
    p_f1_KP_q.publish(m_f1_KP_q);

    p_f1_HR_q_ref.publish(m_f1_HR_q_ref);
    p_f1_HP_q_ref.publish(m_f1_HP_q_ref);
    p_f1_KP_q_ref.publish(m_f1_KP_q_ref);

    // Joint Angular Velocity
    m_f1_HR_dq.data = dq_(HR);
    m_f1_HP_dq.data = dq_(HP);
    m_f1_KP_dq.data = dq_(KP);

    p_f1_HR_dq.publish(m_f1_HR_dq);
    p_f1_HP_dq.publish(m_f1_HP_dq);
    p_f1_KP_dq.publish(m_f1_KP_dq);

//     // Ref Foot Position
    m_f1_ref_x.data = x_(0);
    m_f1_ref_y.data = x_(1);
    m_f1_ref_z.data = x_(2);
//     m_f1_ref_rp.data = _pos_d(F1,RP);

    p_f1_ref_x.publish(m_f1_ref_x);
    p_f1_ref_y.publish(m_f1_ref_y);
    p_f1_ref_z.publish(m_f1_ref_z);
//     p_f1_ref_rp.publish(m_f1_ref_rp);

    // Act Foot Position
    m_f1_act_x.data = EEPos(0);
    m_f1_act_y.data = EEPos(1);
    m_f1_act_z.data = EEPos(2);
//     m_f1_act_rp.data = pin.GetPos(F1)(RP);

    p_f1_act_x.publish(m_f1_act_x);
    p_f1_act_y.publish(m_f1_act_y);
    p_f1_act_z.publish(m_f1_act_z);
//     p_f1_act_rp.publish(m_f1_act_rp);

    // Torque
    m_f1_HR_torque.data = torque_(HR);
    m_f1_HP_torque.data = torque_(HP);
    m_f1_KP_torque.data = torque_(KP);

    p_f1_HR_torque.publish(m_f1_HR_torque);
    p_f1_HP_torque.publish(m_f1_HP_torque);
    p_f1_KP_torque.publish(m_f1_KP_torque);

}


void leg_3dof_controller::ChangeControlState()
{
    switch(controlmode)
    {
    case INIT:
        controlmode = HOMING;
        break;
    case HOMING:
        controlmode = WALKING;
        break;
    case WALKING:
        controlmode = WALKING2;
        break;
    case WALKING2:
        controlmode = FINISH;
        break;
    }
}

void leg_3dof_controller::HomingControl()
{
    if(flag_ == false)
    {
        q_0 = q_;
        ChangeFlag();
    }
            
    traj[0].SetSinusoidalTrajectory(homing_ref_q, q_0, 3);

    q_f = traj[0].GetRefVar();
    q_f_dot = traj[0].GetRefVarDot();

    robotleg[0].SetPDControl_q(homing_Kp, homing_Kd, q_f, q_, q_f_dot, dq_);
  
    torque_ = robotleg[0].GetJointTorque() + generalized_gravity;
    // torque_ = robotleg[0].GetJointTorque();

}

void leg_3dof_controller::SimpleTaskSpacePDControl()
{
    switch (taskspace)
    {
    case MOVING1:
        error_x_ << 0, 0, 0.2;

        TaskSpaceMoving(0.5, x_0, error_x_, x_, kp, kd, "taskspacePD");
        if (flag_2 == false) taskspace = MOVING2;

        break;

    case MOVING2:
        error_x_ << 0.2, 0, 0;

        TaskSpaceMoving(0.5, x_0, error_x_, x_, kp, kd, "taskspacePD");
        if (flag_2 == false) taskspace = MOVING3;

        break;

    case MOVING3:
        error_x_ << -0.2, 0, 0;

        TaskSpaceMoving(0.5, x_0, error_x_, x_, kp, kd, "taskspacePD");
        if (flag_2 == false) taskspace = MOVING1;
        
        break;

    case FINISHTaskSpacePD:
        error_x_ << 0, 0, 0;

        TaskSpaceMoving(3, x_0, error_x_, x_, kp, kd, "taskspacePD");

        std::cout << "Finish " << std::endl;
        if (flag_2 == false) controlmode = FINISH;

        break;
        
    }
}

void leg_3dof_controller::SimpleTaskSpaceCTCControl()
{
    switch (taskspace)
    {
    case MOVING1:
        error_x_ << 0, 0, 0.2;

        TaskSpaceMoving(0.5, x_0, error_x_, x_, ctc_kp, ctc_kd, "taskspaceCTC");
        if (flag_2 == false) taskspace = MOVING2;

        break;

    case MOVING2:
        error_x_ << 0, 0, -0.8;

        TaskSpaceMoving(0.5, x_0, error_x_, x_, ctc_kp, ctc_kd, "taskspaceCTC");
        if (flag_2 == false) taskspace = MOVING3;

        break;

    case MOVING3:
        error_x_ << 0, 0, 0.2;

        TaskSpaceMoving(0.5, x_0, error_x_, x_, ctc_kp, ctc_kd, "taskspaceCTC");
        if (flag_2 == false) taskspace = MOVING1;
        
        break;

    case FINISHTaskSpacePD:
        error_x_ << 0, 0, 0.2;

        TaskSpaceMoving(3, x_0, error_x_, x_, ctc_kp, ctc_kd, "taskspaceCTC");

        std::cout << "Finish " << std::endl;
        if (flag_2 == false) controlmode = FINISH;

        break;
        
    }
}

// Floating Body Cotrol mode

void leg_3dof_controller::FloatingBasecommand(bool flag)
{
    if (flag)
    {
        // input q_, dq_

        
        pin.FloatingBaseSetRobotParameter(floating_q_, floating_dq_);

        pin.SetGravity();
        generalized_gravity = pin.GetFloatingBaseGravityCompensation();

        pin.FloatingBaseSetKinematics(0);
       
        EEPos = pin.GetPos(0);
        J_ = pin.GetJacobian(0);

        switch(controlmode)
        {
        case INIT:
            if(t_ >= 4) ChangeControlState();
            break;
        
        case HOMING:
            HomingControl();
            if(t_ >= 7) 
            {
                controlmode = TaskSpacePD;
                traj[0].ResetPeriod();
                top_ref_x = EEPos;
                top_ref_x(2) = top_ref_x(2) + 0.2;
                x_0 = EEPos;
            }
            break;
        case TaskSpacePD:
            if (flag_ == true)
            {
                x_0_ref = EEPos;

                flag_ = false;
                sub_t_ = t_;

                traj[0].ResetPeriod();
            }

            traj[0].SetSinusoidalTrajectory(top_ref_x, x_0_ref, 0.5);
            x_ = traj[0].GetRefVar();

            robotleg[0].SetPDControl_X(kp, kd, x_, EEPos, J_);

            // pin.SetDynamics(traj[0].GetRefVar(), traj[0].GetRefVarDot(), traj[0].GetRefVarDDot(), ctc_kp, ctc_kd);

            torque_ = robotleg[0].GetJointTorque() + generalized_gravity;

            // torque_ = pin.GetDynamics();

            std::cout << "EEPos : " << EEPos << std::endl;
            std::cout << "J_ : " << J_ << std::endl;

            if (t_ - sub_t_ >= 0.5)
            {
                controlmode = FINISH;
            }

            break;

        case FINISH:
            // std::cout << "FINISH " << std::endl;
            break;
        }

    
        // output toque_(i)
        send_commands_to_robot();
    }

    else
    {
        std::cout << "Not Connected Please Wait..." << std::endl;
    }
}


void leg_3dof_controller::TaskSpaceMoving(double duration, Eigen::Vector3d initial_x, Eigen::Vector3d error_x, Eigen::Vector3d ref_x_0, Eigen::Matrix3d _Kp_, Eigen::Matrix3d _Kd_, const std::string& control_name)
{
    if (flag_ == false)
    {
        ChangeFlag();
        ChangeFlag2();
        reference_x_0 = ref_x_0;

        for(int i = 0; i < 3; i++)
        {
            reference_x(i) = initial_x(i) + error_x(i);
        }
        sub_t_ = t_;

        traj[0].ResetPeriod();
    }

    traj[0].SetSinusoidalTrajectory(reference_x, reference_x_0, duration);
    x_ = traj[0].GetRefVar();

    if(control_name == "taskspaceCTC")
    {
        pin.SetDynamics(traj[0].GetRefVar(), traj[0].GetRefVarDot(), traj[0].GetRefVarDDot(), _Kp_, _Kd_);

        torque_ = pin.GetDynamics();
    }

    if(control_name == "taskspacePD")
    {
        robotleg[0].SetPDControl_X(kp, kd, x_, EEPos, J_);

        torque_ = robotleg[0].GetJointTorque() + generalized_gravity;
    }

    if (t_ - sub_t_ >= duration)
    {
        count_ ++;
        ChangeFlag();
        ChangeFlag2();
    }

}


void leg_3dof_controller::ChangeFlag()
{
    if(flag_ == true) flag_ = false;
    else flag_ = true;
}

void leg_3dof_controller::ChangeFlag2()
{
    if(flag_2 == true) flag_2 = false;
    else flag_2 = true;
}