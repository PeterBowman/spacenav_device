// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


#include "rclcpp/rclcpp.hpp"  

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <kdl/frames.hpp>

#include <ICartesianControl.h>

#include <chrono>
#include <memory>
#include <mutex>


const auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";

using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;

class SpacenavSubscriber : public rclcpp::Node
{
public:
    SpacenavSubscriber();
    ~SpacenavSubscriber();

private:
    void spnav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void set_preset_streaming_cmd(const std::string &value);
    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
    void timer_callback();


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_state_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::Client<SetParameters>::SharedPtr client_param_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist::SharedPtr last_msg_;
    std::mutex msg_mutex_;

    rclcpp::Time last_update_time_;

    tf2::Vector3 initial_position_;
    tf2::Quaternion initial_orientation_;
    tf2::Vector3 current_position_;
    tf2::Quaternion current_orientation_;
    bool initial_pose_set_;
    bool virtual_pose_set;

    roboticslab::ICartesianControl * iCartesianControl_;

    double scale_ = 0.3;
    std::string streaming_msg;
    std::string frame_;

};

