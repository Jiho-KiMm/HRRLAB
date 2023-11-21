#ifndef RQT_PLOT_H_
#define RQT_PLOT_H_

#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <iostream>

// Joint Angle Publisher
ros::Publisher p_f1_HR_q;
ros::Publisher p_f1_HP_q;
ros::Publisher p_f1_KP_q;

ros::Publisher p_f1_HR_q_ref;
ros::Publisher p_f1_HP_q_ref;
ros::Publisher p_f1_KP_q_ref;

// Joint Angular Velocity Publisher
ros::Publisher p_f1_HR_dq;
ros::Publisher p_f1_HP_dq;
ros::Publisher p_f1_KP_dq;

// Ref Foot Position Publisher
ros::Publisher p_f1_ref_x;
ros::Publisher p_f1_ref_y;
ros::Publisher p_f1_ref_z;
ros::Publisher p_f1_ref_rp;

// Act Foot Position Publisher
ros::Publisher p_f1_act_x;
ros::Publisher p_f1_act_y;
ros::Publisher p_f1_act_z;
ros::Publisher p_f1_act_rp;

// Joint Torque Publisher
ros::Publisher p_f1_HR_torque;
ros::Publisher p_f1_HP_torque;
ros::Publisher p_f1_KP_torque;

// Joint Power Publisher
ros::Publisher p_f1_hy_power;
ros::Publisher p_f1_hp_power;
ros::Publisher p_f1_hr_power;

// Joint Angle Float64 Message
std_msgs::Float64 m_f1_HR_q;
std_msgs::Float64 m_f1_HP_q;
std_msgs::Float64 m_f1_KP_q;

std_msgs::Float64 m_f1_HR_q_ref;
std_msgs::Float64 m_f1_HP_q_ref;
std_msgs::Float64 m_f1_KP_q_ref;

// Joint Angular Velocity Float64 Message
std_msgs::Float64 m_f1_HR_dq;
std_msgs::Float64 m_f1_HP_dq;
std_msgs::Float64 m_f1_KP_dq;

// Ref Position Float64 Message
std_msgs::Float64 m_f1_ref_x;
std_msgs::Float64 m_f1_ref_y;
std_msgs::Float64 m_f1_ref_z;
std_msgs::Float64 m_f1_ref_rp;

// Act Position Float64 Message
std_msgs::Float64 m_f1_act_x;
std_msgs::Float64 m_f1_act_y;
std_msgs::Float64 m_f1_act_z;
std_msgs::Float64 m_f1_act_rp;

// Joint Torque Float64 Message
std_msgs::Float64 m_f1_HR_torque;
std_msgs::Float64 m_f1_HP_torque;
std_msgs::Float64 m_f1_KP_torque;

// Joint Power Float64 Message
std_msgs::Float64 m_f1_hy_power;
std_msgs::Float64 m_f1_hp_power;
std_msgs::Float64 m_f1_hr_power;

#endif