#include <cstdio>
#include <iostream>
#include <vector>
#include <memory>

#include <wiringPi.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// TODO : Make this configurable
// See https://pinout.xyz/pinout/wiringpi# for pin remapping

// const std::vector<size_t> PINS = {3, 8, 5, 10};
// Left motor red wire to OUT1
// Right motor red wire to OUT2
const std::vector<size_t> PINS = {2, 3, 0, 1};

const size_t PINS_CONFIG[][4] = {
		{1, 0, 1, 0},
		{1, 0, 0, 1},
		{0, 1, 1, 0},
		{0, 0, 0, 0},
		{0, 1, 0, 1},
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
    using FdGpio = int;
    static constexpr int LFLAGS = 0;
    static constexpr int OUT = 21;

    public:
    MinimalSubscriber()
      : Node("minimal_subscriber") 
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MinimalSubscriber::driveCallback, this, _1));
	wiringPiSetup();
        for (const size_t pin : PINS) {
    		// pins_.push_back(lgGpioClaimOutput(pin, LFLAGS, OUT, 0));
		pins_.push_back(pin);
		pinMode(pin, OUTPUT);
        }
	currentDrive_ = Drive::STOP;
	setPins(Drive::STOP);
	initialized_ = true;

    }

    ~MinimalSubscriber() {}
    private:
    void setPins(const Drive signal) { 
	    if (initialized_ && signal != currentDrive_) {
		    std::cout << "Signal: " << signal << std::endl;
		    for (size_t i = 0; i < 4; ++i) { 
			    size_t pinOut = PINS_CONFIG[size_t(signal)][i]; 
			    digitalWrite(pins_[i], pinOut);
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
    std::vector<FdGpio> pins_;
    Drive currentDrive_;
    bool initialized_=false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
