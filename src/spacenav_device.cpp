// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <memory> 
#include <mutex> 

#include <kdl/chain.hpp> 

#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp"  

constexpr auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";

class SpacenavSubscriber : public rclcpp::Node
{   
	public:
		SpacenavSubscriber() 
		: Node("spacenav_device")
		{
			subscription_spnav_ = this->create_subscription<geometry_msgs::msg::Twist>("/spacenav/twist", 10, 
                                                                                        std::bind(&SpacenavSubscriber::topic_callback, 
                                                                                        this, std::placeholders::_1));
                                                                                        
			publisher_spnav_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
		}

	private:
	    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const 
		{
			RCLCPP_INFO(this->get_logger(), "spnav: [%f %f %f] [%f %f %f]",
                  msg->linear.x, msg->linear.y, msg->linear.z,
                  msg->angular.x, msg->angular.y, msg->angular.z);


			publisher_spnav_->publish(*msg);
		}

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_;
};



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpacenavSubscriber>());
	rclcpp::shutdown();
	return 0;
}