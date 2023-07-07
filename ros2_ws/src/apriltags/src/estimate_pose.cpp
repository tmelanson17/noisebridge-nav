#include <cstdio>
// April Tags
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
//-----------------
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/hal/interface.h>

using namespace cv;

// TODO : create td (config object)
// TODO : convert input image message into the AprilTag type
class AprilTagPoseEstimation : public rclcpp::Node
{
    public:
    AprilTagPoseEstimation()
      : Node("apriltag")
    {
	declareNodeParams();
	it = std::unique_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(rclcpp::Node::SharedPtr(this))
	);
	sub = it->subscribeCamera("/image_raw", 10, std::bind(&AprilTagPoseEstimation::estimatePose, this, std::placeholders::_1, std::placeholders::_2));
	pub = it->advertise("/image_with_pose", 10);
	setupApriltagDetector();
    }

    ~AprilTagPoseEstimation() {
	destroyAprilTags();
    }

    void destroyAprilTags() {
        apriltag_detector_destroy(td_);
	
        if (famname_ == "tag36h11") {
            tag36h11_destroy(tf_);
        } else if (famname_ == "tag25h9") {
            tag25h9_destroy(tf_);
        } else if (famname_ == "tag16h5") {
            tag16h5_destroy(tf_);
        } else if (famname_ == "tagCircle21h7") {
            tagCircle21h7_destroy(tf_);
        } else if (famname_ == "tagCircle49h12") {
            tagCircle49h12_destroy(tf_);
        } else if (famname_ == "tagStandard41h12") {
            tagStandard41h12_destroy(tf_);
        } else if (famname_ == "tagStandard52h13") {
            tagStandard52h13_destroy(tf_);
        } else if (famname_ == "tagCustom48h12") {
            tagCustom48h12_destroy(tf_);
        }
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

    void setupApriltagDetector(){
        // Initialize tag detector with options
	famname_ = this->get_parameter("family").as_string();
        if (famname_ == "tag36h11") {
            tf_ = tag36h11_create();
        } else if (famname_ == "tag25h9") {
            tf_ = tag25h9_create();
        } else if (famname_ == "tag16h5") {
            tf_ = tag16h5_create();
        } else if (famname_ == "tagCircle21h7") {
            tf_ = tagCircle21h7_create();
        } else if (famname_ == "tagCircle49h12") {
            tf_ = tagCircle49h12_create();
        } else if (famname_ == "tagStandard41h12") {
            tf_ = tagStandard41h12_create();
        } else if (famname_ == "tagStandard52h13") {
            tf_ = tagStandard52h13_create();
        } else if (famname_ == "tagCustom48h12") {
            tf_ = tagCustom48h12_create();
        } else {
	    printf("Unrecognized tag family name. Use e.g. \"tagStandard52h13\".\n");
	    throw std::exception();
        }
    	td_ = apriltag_detector_create();
    	apriltag_detector_add_family(td_, tf_);


	td_->quad_decimate = this->get_parameter( "decimate").as_double();
	td_->quad_sigma = this->get_parameter("blur").as_double();
	td_->nthreads = this->get_parameter("reads").as_int();
	td_->debug = this->get_parameter("debug").as_bool();
	td_->refine_edges = this->get_parameter("refine-edges").as_bool();
    }

    void estimatePose(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
	(void)info;
	// TODO: Can you convert uint8[] to void*?
	const int typ = (CV_8UC3);
	const cv::Mat frame(msg->height, msg->width, typ);
	memcpy(frame.data, msg->data.data(), msg->data.size()*sizeof(uint8_t));
	Mat gray;
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	// Make an image_u8_t header for the Mat data
        image_u8_t im = { 
	    gray.cols,
            gray.rows,
            gray.cols,
            gray.data
        };
        zarray_t *detections = apriltag_detector_detect(td_, &im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

	    // TODO : Make this a cmake parameter
	    bool quiet = false;
            if (!quiet)
                printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                       i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
        }
	apriltag_detections_destroy(detections);

    }

    private:
    std::unique_ptr<image_transport::ImageTransport> it;
    image_transport::CameraSubscriber sub;
    image_transport::Publisher pub;
    apriltag_detector_t* td_;
    apriltag_family_t* tf_;
    std::string famname_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world apriltags package\n");
  return 0;
}
