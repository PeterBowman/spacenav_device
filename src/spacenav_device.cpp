// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <rclcpp/rclcpp.hpp>	 
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>


constexpr auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";
constexpr auto SCALE = 0.3; 

class SpacenavSubscriber : public rclcpp::Node
{   
	public:
		SpacenavSubscriber() 
		: Node("spacenav_device")
		{
			last_update_time_ = now(); // Initialize last update time
			
			// Subscriber
			subscription_spnav_ = this->create_subscription<geometry_msgs::msg::Twist>("/spacenav/twist", 10, 
                                                                                        std::bind(&SpacenavSubscriber::topic_callback, 
                                                                                        this, std::placeholders::_1));

			// Obtain parameters
			this->declare_parameter<std::string>("msgs_type", "twist"); // default value is "twist"
        	this->get_parameter("msgs_type", msgs_type);                                                                            
			
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
		void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
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

				RCLCPP_INFO(this->get_logger(), "spnav: [%f %f %f] [%f %f %f]", v[0], v[1], v[2], v[3], v[4], v[5]);
				
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
				auto dt = (current_time - last_update_time_).seconds(); // Get elapsed time since last update
				last_update_time_ = current_time;

				auto msg_pose = std::make_shared<geometry_msgs::msg::Pose>();

				// Transform linear and angular velocities to real pose in cartesian space 
				msg_pose->position.x += msg->linear.x * dt;
				msg_pose->position.y += msg->linear.y * dt;
				msg_pose->position.z += msg->linear.z * dt;
				msg_pose->orientation.x += msg->angular.x * dt;
				msg_pose->orientation.y += msg->angular.y * dt;
				msg_pose->orientation.z += msg->angular.z * dt;

				publisher_spnav_pose->publish(*msg_pose);
			}	
		}

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose;
		rclcpp::Time last_update_time_;
		std::string msgs_type;
};



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SpacenavSubscriber>());
	rclcpp::shutdown();
	return 0;
}