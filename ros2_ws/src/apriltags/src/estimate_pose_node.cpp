#include <cstdio>
#include <image_transport/image_transport.hpp>

using namespace cv;

// TODO : create td (config object)
// TODO : convert input image message into the AprilTag type
class AprilTagPoseEstimation : public rclcpp::Node
{
    public:
    AprilTagPoseEstimation()
      : Node("apriltag")
    {
	it = std::unique_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(rclcpp::Node::SharedPtr(this))
	);
	sub = it->subscribeCamera("/image_raw", 10, std::bind(&AprilTagPoseEstimation::estimatePose, this, std::placeholders::_1, std::placeholders::_2));
	pub = it->advertise("/image_with_pose", 10);
	setupApriltagDetector();
    }

    ~AprilTagPoseEstimation() {
    }

    void declareNodeParams() {
	// TODO: Add CPU descriptions
	// Enable debugging output (slow)
	this->declare_parameter("debug", false);
	// Reduce output
	this->declare_parameter("quiet", false);
	// Tag family to use
	this->declare_parameter("family", "tagStandard52h13");
	// Use this many CPU threads
	this->declare_parameter("threads", 1);
	// Decimate input image by this factor
	this->declare_parameter("decimate", 2.0);
	// Apply low-pass blur to input
	this->declare_parameter("blur", 0.0);
	// Spend more time trying to align edges of tags
	this->declare_parameter("refine-edges", true);
    }




    private:
    std::unique_ptr<image_transport::ImageTransport> it;
    image_transport::CameraSubscriber sub;
    image_transport::Publisher pub;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world apriltags package\n");
  return 0;
}
