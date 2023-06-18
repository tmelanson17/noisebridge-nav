#include <cstdio>
#include <iostream>
#include <vector>
#include <memory>

#include <cppgpio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// TODO : Make this configurable

const std::vector<size_t> PINS = {3, 8, 5, 10};

const size_t PINS_CONFIG[][4] = {
		{true, false, true, false},
		{true, false, false, true},
		{false, true, true, false},
		{false, false, false, false},
		{false, true, false, true},
};

enum Drive {
	FORWARD=0,
	LEFT_TURN=1,
	RIGHT_TURN=2,
	STOP=3,
	BACKWARD=4,
};

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
    MinimalSubscriber()
      : Node("minimal_subscriber") 
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MinimalSubscriber::driveCallback, this, _1));
        for (const size_t pin : PINS) {
    		pins_.push_back(GPIO::DigitalOut(pin));
        }
	currentDrive_ = Drive::STOP;

    }
    private:
    void setPins(const Drive signal) { 
	    if (signal != currentDrive_) {
		    std::cout << "Signal: " << signal << std::endl;
		    for (size_t i = 0; i < 4; ++i) { 
			    if (PINS_CONFIG[size_t(signal)][i]) { 
				    pins_[i].on(); 
			    } else { 
				    pins_[i].off(); 
			    } 
		    } 
		    currentDrive_ = signal;
	    }
    }

    void driveCallback(const geometry_msgs::msg::Twist & msg) 
    {
	    // Has to be one of 4 : fwd, backward, left, right
	    const float LINEAR_THRESHOLD = 0.05f;
	    const float ANGULAR_THRESHOLD = 0.2f;
	    if (msg.linear.x > LINEAR_THRESHOLD) {
		    setPins(Drive::FORWARD);
	    } else if (msg.linear.x < -LINEAR_THRESHOLD) {
		    setPins(Drive::BACKWARD);
	    } else if (msg.angular.z > ANGULAR_THRESHOLD) {
		    setPins(Drive::LEFT_TURN);
	    } else if (msg.angular.z < -ANGULAR_THRESHOLD) {
		    setPins(Drive::RIGHT_TURN);
	    } else {
		    setPins(Drive::STOP);
	    }
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::vector<GPIO::DigitalOut> pins_;
    Drive currentDrive_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
