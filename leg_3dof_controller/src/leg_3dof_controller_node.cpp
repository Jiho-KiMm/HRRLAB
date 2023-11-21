#include "leg_3dof_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_3dof_node");
    ros::NodeHandle nh;

    // Variable
    const double freq = 500;
    // std::string urdf_path_ = "/home/jiho/example_ws/src/robots/robot_description/leg_3dof/urdf/leg_3dof.urdf";
    // std::vector<std::string> foot_name = {"EE"};

    std::string topic_leg_state;
    std::string topic_leg_command;
    std::string topic_frame_state;
    std::string topic_frame_command;

    // Topic names
    if (!nh.getParam("topic_leg_state", topic_leg_state))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the state of the leg.");
        return -1;
    }
    if (!nh.getParam("topic_leg_command", topic_leg_command))
    {
        ROS_ERROR("Couldn't retrieve the topic name for commanding the  leg.");
        return -1;
    }

    // // frame control
    // if (!nh.getParam("topic_frame_state", topic_frame_state))
    // {
    //     ROS_ERROR("Couldn't retrieve the topic name for the state of the frame.");
    //     return -1;
    // }
    // if (!nh.getParam("topic_frame_command", topic_frame_command))
    // {
    //     ROS_ERROR("Couldn't retrieve the topic name for commanding the  frame.");
    //     return -1;
    // }

    // stquad_controller class
    // leg_3dof_controller controller(nh, topic_leg_state, topic_leg_command, urdf_path_, foot_name, freq);
    leg_3dof_controller controller(nh, topic_leg_state, topic_leg_command, freq);
    // Controller Start
    controller.init();
    controller.run();

    return 0;
}