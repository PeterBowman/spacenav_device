// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "rclcpp/rclcpp.hpp"  

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


constexpr auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";
constexpr auto SCALE = 0.3; 

class SpacenavSubscriber : public rclcpp::Node
{   
	public:
		SpacenavSubscriber() 
		: Node("spacenav_device")
		{
			last_update_time_ = now(); // Initialize last update time

			// Obtain parameters
			this->declare_parameter<std::string>("msgs_type", "twist"); // default value is "twist"
        	this->get_parameter("msgs_type", msgs_type);   
			
			// Subscribers
			subscription_spnav_ = this->create_subscription<geometry_msgs::msg::Twist>("/spacenav/twist", 10, 
                                                                                        std::bind(&SpacenavSubscriber::spnav_callback, 
                                                                                        this, std::placeholders::_1));

			subscription_state_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(DEFAULT_NODE_NAME + std::string("/state/pose"), 10, 
																						std::bind(&SpacenavSubscriber::state_callback, 
																						this, std::placeholders::_1));

			// Publisher
			if (msgs_type == "twist")
			{
				publisher_spnav_twist = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
			} 

			else if (msgs_type == "pose")
			{
				publisher_spnav_pose = this->create_publisher<geometry_msgs::msg::Pose>(DEFAULT_NODE_NAME + std::string("/command/pose"), 10);
			} 

			else
			{
				RCLCPP_ERROR(this->get_logger(), "Invalid message type. Using 'twist' by fefault.");
				msgs_type = "twist";
				publisher_spnav_twist = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
			}

		}

	private:
		void spnav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
		{
			if (msgs_type == "twist")
			{
					std::vector<double> v 
				{
					msg->linear.x,
					msg->linear.y,
					msg->linear.z,
					msg->angular.x,
					msg->angular.y,
					msg->angular.z
				};

				for(int i = 0; i < 6; i++)
				{
					v[i] *= SCALE; //mejorar la sensibilidad del desplazamiento
				}

				RCLCPP_INFO(this->get_logger(), "Spnav Twist: [%f %f %f] [%f %f %f]", v[0], v[1], v[2], v[3], v[4], v[5]);
				
				auto msg_scaled = std::make_shared<geometry_msgs::msg::Twist>();

				msg_scaled->linear.x = v[0];
				msg_scaled->linear.y = v[1];
				msg_scaled->linear.z = v[2];
				msg_scaled->angular.x = v[3];
				msg_scaled->angular.y = v[4];
				msg_scaled->angular.z = v[5];

				publisher_spnav_twist->publish(*msg_scaled);
				
			} 
			
			else if (msgs_type == "pose")
			{
				auto current_time = now();
				auto dt = (current_time - last_update_time_).seconds(); // Get elapsed time since last update from sensor input
				last_update_time_ = current_time;

				std::vector<double> msg_traslation;

				// Transform linear and angular velocities into traslations
				msg_traslation = {msg->linear.x * dt, msg->linear.y * dt, msg->linear.z * dt, 
								  msg->angular.x * dt, msg->angular.y * dt, msg->angular.z * dt};

				// Update current state adding traslation
				for(int i = 0; i < 6; i++)
				{
					current_state[i] += msg_traslation[i];
				}

				auto msg_pose = std::make_shared<geometry_msgs::msg::Pose>();

				msg_pose->position.x = current_state[0];
				msg_pose->position.y = current_state[1];
				msg_pose->position.z = current_state[2];
				msg_pose->orientation.x = current_state[3];
				msg_pose->orientation.y = current_state[4];
				msg_pose->orientation.z = current_state[5];
				//msg_pose->orientation.w = 1.0;

				//RCLCPP_INFO(this->get_logger(), "Spnav Pose: [%f %f %f] [%f %f %f]", msg_pose->position.x, msg_pose->position.y, msg_pose->position.z, 
				//								 msg_pose->orientation.x, msg_pose->orientation.y, msg_pose->orientation.z, msg_pose->orientation.w);

				publisher_spnav_pose->publish(*msg_pose);
			}	
		}


		void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
		{
			// RECIBIMOS QUATERNIOS!!!

			// Get current state in traslation and orientation aroun axis x, y, z
			current_state = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 
							 msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z};
		
		}	


		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_state_pose;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose;
		rclcpp::Time last_update_time_;
		std::string msgs_type;
		std::vector<double> current_state;
};

// -----------------------------------------------------------------------------

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpacenavSubscriber>());
	rclcpp::shutdown();
	return 0;
}