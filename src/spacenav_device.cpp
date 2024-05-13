// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "rclcpp/rclcpp.hpp"  

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <ICartesianControl.h>


constexpr auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";
constexpr auto SCALE = 0.5;

class SpacenavSubscriber : public rclcpp::Node
{   
	public:
		SpacenavSubscriber() 
		: Node("spacenav_device")
		{
			last_update_time_ = now(); // Initialize last update time

			// Obtain parameters
			rcl_interfaces::msg::ParameterDescriptor descriptor;
			descriptor.name = "streaming_msg";
			descriptor.description = "Streaming command msg type to be used by the device.";
			descriptor.read_only = false;
			descriptor.additional_constraints = "Only 'twist' or 'pose' are allowed.";
			descriptor.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
			
			this->declare_parameter<std::string>("streaming_msg", "twist"); // "twist" msgs by default
        	this->get_parameter("streaming_msg", streaming_msg);

			callback_handle_ = this->add_on_set_parameters_callback(std::bind(&SpacenavSubscriber::parameter_callback, this, std::placeholders::_1));
			
			// Subscribers
			subscription_spnav_ = this->create_subscription<geometry_msgs::msg::Twist>("/spacenav/twist", 10, 
                                                                                        std::bind(&SpacenavSubscriber::spnav_callback, 
                                                                                        this, std::placeholders::_1));

			subscription_state_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(DEFAULT_NODE_NAME + std::string("/state/pose"), 10, 
																						std::bind(&SpacenavSubscriber::state_callback, 
																						this, std::placeholders::_1));
			
			// Publisher
			if (streaming_msg == "twist")
			{
				publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);

			} 

			else if (streaming_msg == "pose")
			{
				publisher_spnav_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(DEFAULT_NODE_NAME + std::string("/command/pose"), 10);

			} 
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Invalid message type. Using 'twist' by default.");
				streaming_msg = "twist";
				publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);

			}
		}

	private:
		void spnav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
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
				v[i] *= SCALE; // Improve sensibility 
			}
			
			auto msg_scaled = std::make_shared<geometry_msgs::msg::Twist>(); 

			msg_scaled->linear.x = v[0];
			msg_scaled->linear.y = v[1];
			msg_scaled->linear.z = v[2];
			msg_scaled->angular.x = v[3];
			msg_scaled->angular.y = v[4];
			msg_scaled->angular.z = v[5];

			if (streaming_msg == "twist")
			{
				RCLCPP_INFO(this->get_logger(), "Spnav Twist: [%f %f %f] [%f %f %f]", v[0], v[1], v[2], v[3], v[4], v[5]);

				publisher_spnav_twist_->publish(*msg_scaled);
	
			} 
			
			else if (streaming_msg == "pose")
			{
				auto current_time = now();
				auto dt = (current_time - last_update_time_).seconds(); // Get elapsed time since last update from sensor input
				last_update_time_ = current_time;

				if(!initial_pose_set_)
				{
					RCLCPP_ERROR(this->get_logger(), "Initial pose not set. Please, set initial pose first.");
					return;
				}

				// Transform linear and angular velocities into space traslations (de = v*dt)			
				std::vector<double> msg_traslation;

				for (int j=0; j<6; j++)
				{
					msg_traslation.push_back(v[j] * dt);
				}

				//Create new quaternion for Pose applying angular rotation
				tf2::Quaternion q;
				q.setRPY(msg_traslation[3], msg_traslation[4], msg_traslation[5]);
				tf2::Quaternion new_orientation	= initial_orientation_ * q;
				new_orientation.normalize(); // Normalize new quaternion

				tf2::Vector3 traslation(msg_traslation[0], msg_traslation[1], msg_traslation[2]);
				tf2::Matrix3x3 rotation(new_orientation);
				traslation = rotation * traslation; // Rotate traslation vector

				tf2::Vector3 new_position = initial_position_ + traslation;

				auto msg_pose = std::make_shared<geometry_msgs::msg::Pose>();

				msg_pose->position.x = new_position.x();
				msg_pose->position.y = new_position.y();
				msg_pose->position.z = new_position.z();
				msg_pose->orientation = tf2::toMsg(new_orientation);


				RCLCPP_INFO(this->get_logger(), "Spnav Pose: [%f %f %f] [%f %f %f %f]", msg_pose->position.x, msg_pose->position.y, msg_pose->position.z, 
												 msg_pose->orientation.x, msg_pose->orientation.y, msg_pose->orientation.z, msg_pose->orientation.w);

				publisher_spnav_pose_->publish(*msg_pose);
			}	
		}


		void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
		{
			//Get current state in traslation and orientation aroun axis x, y, z (quaternions)
			tf2::fromMsg(msg->pose.position, initial_position_);
			tf2::fromMsg(msg->pose.orientation, initial_orientation_);
				
			initial_pose_set_ = true;	 
		}	


		// Callback for parameter changes

		rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
		{
			rcl_interfaces::msg::SetParametersResult result;
			result.successful = true;
			for (const auto &param: parameters)
			{
				if(param.get_name() == "streaming_msg")  
				{
					streaming_msg = param.value_to_string();

					if (streaming_msg == "twist")
					{
						RCLCPP_INFO(this->get_logger(), "Param for streaming_msg correctly stablished: %s", streaming_msg.c_str());

						publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
					}
					else if (streaming_msg == "pose")
					{
						RCLCPP_INFO(this->get_logger(),"Param for streaming_msg correctly stablished: %s", streaming_msg.c_str());

						publisher_spnav_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(DEFAULT_NODE_NAME + std::string("/command/pose"), 10);
					}
					else
					{
						result.successful = false;
						streaming_msg = "twist";
						RCLCPP_ERROR(this->get_logger(),"Invalid parameter for streaming_msg. Only 'twist' or 'pose' are allowed. Using 'twist' by default.");
						publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);

					}
				}
			}
			return result;
		}	

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_state_pose_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist_;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose_;
		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

		std::string streaming_msg;

		rclcpp::Time last_update_time_;
		tf2::Vector3 initial_position_;
		tf2::Quaternion initial_orientation_;
		// std::vector<double> initial_pose_;
		//std::vector<double> final_pose_;
		bool initial_pose_set_;

		roboticslab::ICartesianControl * iCartesianControl_;

};

// -----------------------------------------------------------------------------

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpacenavSubscriber>());
	rclcpp::shutdown();
	return 0;
}