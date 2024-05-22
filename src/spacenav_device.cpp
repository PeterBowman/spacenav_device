// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "rclcpp/rclcpp.hpp"  

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <kdl/frames.hpp>

#include <ICartesianControl.h>

#include <chrono>
#include <memory>
#include <mutex>


const auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";
const auto SCALE = 0.3;

using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;

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
			
			this->declare_parameter<std::string>("streaming_msg", "twist", descriptor); // "twist" msgs by default
        	this->get_parameter("streaming_msg", streaming_msg);

			// Callback for parameter changes
			callback_handle_ = this->add_on_set_parameters_callback(std::bind(&SpacenavSubscriber::parameter_callback, this, std::placeholders::_1));
			
			// Set parameters for external node
			client_param_ = this->create_client<SetParameters>(DEFAULT_NODE_NAME + std::string("/set_parameters"));

			while (!client_param_->wait_for_service(std::chrono::seconds(1))) 
			{
				if (!rclcpp::ok()) 
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					return;
				}
				RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
			}
			
			// Subscribers
			subscription_spnav_ = this->create_subscription<geometry_msgs::msg::Twist>("/spacenav/twist", 10, 
                                                                                        std::bind(&SpacenavSubscriber::spnav_callback, 
                                                                                        this, std::placeholders::_1));

			subscription_state_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(DEFAULT_NODE_NAME + std::string("/state/pose"), 10, 
																						std::bind(&SpacenavSubscriber::state_callback, 
																						this, std::placeholders::_1));
			
			// Timer
			timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SpacenavSubscriber::timer_callback, this));
			
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

			// Publish message
			if (streaming_msg == "twist")
			{
				std::lock_guard<std::mutex> lock(msg_mutex_);
		        last_msg_ = msg_scaled;
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

				// Set initial position as a virtual point to avoid PID compensation
				else if(initial_pose_set_ && !virtual_pose_set)
				{
					current_orientation_ = initial_orientation_;
					current_position_ = initial_position_;

					virtual_pose_set = true;
				}
				
				// Transform linear and angular velocities into space traslations (de = v*dt)			
				std::vector<double> msg_traslation;

				for (int j=0; j<6; j++)
				{
					msg_traslation.push_back(v[j] * dt);
				}

				// Create new quaternion for Pose applying angular rotation
				tf2::Quaternion q;
				//auto rot = tf2::Vector3(msg_traslation[3], msg_traslation[4], msg_traslation[5]);
				//q.setRotation(rot, rot.length()); 
				q.setRPY(msg_traslation[3], msg_traslation[4], msg_traslation[5]);
				tf2::Quaternion new_orientation	= current_orientation_ * q;
				new_orientation.normalize(); // Normalize new quaternion

				tf2::Vector3 traslation(msg_traslation[0], msg_traslation[1], msg_traslation[2]);
				tf2::Matrix3x3 rotation(new_orientation);
				traslation = rotation * traslation;

				tf2::Vector3 new_position = current_position_ + traslation;

				if(new_position!=current_position_ || new_orientation!=current_orientation_) //Only publish if there is a change
				{
					current_position_ = new_position;
					current_orientation_ = new_orientation;

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
		}

		// Get initial position and orientation from robot 

		void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
		{
			if(!initial_pose_set_)
			{
				tf2::fromMsg(msg->pose.position, initial_position_);
				tf2::fromMsg(msg->pose.orientation, initial_orientation_);
				initial_pose_set_ = true;
			}	
		}	

		// Set external node parameter

		void set_preset_streaming_cmd(const std::string &value)
    	{
			// Creating request
			auto request = std::make_shared<SetParameters::Request>();

			// Creating parameter
			Parameter param;
			param.name = "preset_streaming_cmd";
			param.value.type = rclcpp::ParameterType::PARAMETER_STRING; 
			param.value.string_value = value;
			request->parameters.push_back(param);

			// Calling service
			using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;
			
			auto response_received_callback = [this](ServiceResponseFuture future) {
				if (future.get()->results[0].successful)
				{
					RCLCPP_INFO(this->get_logger(), "Preset streaming command correctly stablished.");
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "Failed to set preset streaming command.");
				}
        	};
		
        	auto result = client_param_->async_send_request(request, response_received_callback);
		
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
						set_preset_streaming_cmd("twist");

						RCLCPP_INFO(this->get_logger(), "Param for streaming_msg correctly stablished: %s", streaming_msg.c_str());
						publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
					}
					else if (streaming_msg == "pose")
					{
						set_preset_streaming_cmd("movi");

						RCLCPP_INFO(this->get_logger(),"Param for streaming_msg correctly stablished: %s", streaming_msg.c_str());
						publisher_spnav_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(DEFAULT_NODE_NAME + std::string("/command/pose"), 10);
					}
					else
					{
						result.successful = false;
						streaming_msg = "twist";
						set_preset_streaming_cmd("twist");
						RCLCPP_ERROR(this->get_logger(),"Invalid parameter for streaming_msg. Only 'twist' or 'pose' are allowed. Using 'twist' by default.");
						publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
					}
				}
			}
			return result;
		}	


		void timer_callback()
		{
			if (streaming_msg == "twist")
			{
				geometry_msgs::msg::Twist::SharedPtr msg_to_publish;
				{
					std::lock_guard<std::mutex> lock(msg_mutex_);
					msg_to_publish = last_msg_;
				}
				bool zero_msg = (msg_to_publish->linear.x == 0.0 && msg_to_publish->linear.y == 0.0 && msg_to_publish->linear.z == 0.0 && 
								msg_to_publish->angular.x == 0.0 && msg_to_publish->angular.y == 0.0 && msg_to_publish->angular.z == 0.0);	

				if (msg_to_publish && !zero_msg)
				{
					RCLCPP_INFO(this->get_logger(), "Spnav Twist: [%f %f %f] [%f %f %f]", msg_to_publish->linear.x, msg_to_publish->linear.y, msg_to_publish->linear.z, 
													msg_to_publish->angular.x, msg_to_publish->angular.y, msg_to_publish->angular.z);
													
					publisher_spnav_twist_->publish(*msg_to_publish);
				}
			}
		}	


		// Members

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_state_pose_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist_;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose_;
		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rclcpp::Client<SetParameters>::SharedPtr client_param_;
		rclcpp::TimerBase::SharedPtr timer_;
		geometry_msgs::msg::Twist::SharedPtr last_msg_;
    	std::mutex msg_mutex_;

		std::string streaming_msg;

		rclcpp::Time last_update_time_;
		tf2::Vector3 initial_position_;
		tf2::Quaternion initial_orientation_;
		tf2::Vector3 current_position_;
		tf2::Quaternion current_orientation_;
		bool initial_pose_set_;
		bool virtual_pose_set;

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