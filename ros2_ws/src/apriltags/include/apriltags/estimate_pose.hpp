#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
// April Tags
#include <apriltag/apriltag.h>
//-----------------


void setupApriltagDetectorFromNode(rclcpp::Node* node);
class AprilTagsWrapper {
public:
    AprilTagsWrapper(apriltag_detector_t* td, apriltag_family_t* tf) : td_(td), tf_(tf), famname_(tf->name) {}
    ~AprilTagsWrapper() {
	destroyAprilTags();
    }
    void destroyAprilTags();    
    void estimatePose(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info); 
private:
    apriltag_detector_t* td_;
    apriltag_family_t* tf_;
    std::string famname_;
};
