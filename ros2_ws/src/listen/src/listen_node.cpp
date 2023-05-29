#include <cstdio>
#include <vector>
#include <memory>

#include <cppgpio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        for (const size_t pin : PINS) {
    		pins_.push_back(GPIO::DigitalOut(pin));
        }

    }
    private:
    void setPins(Drive signal) {
		for (size_t i = 0; i < 4; ++i) {
			if (PINS_CONFIG[size_t(signal)][i]) {
				pins_[i].on();
			} else {
				pins_[i].off();
			}
		}
	}

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	std::vector<GPIO::DigitalOut> pins_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
