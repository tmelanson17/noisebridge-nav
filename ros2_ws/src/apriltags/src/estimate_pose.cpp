#include <apriltags/estimate_pose.hpp>
// April Tags
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
//-----------------
//
#include <opencv2/opencv.hpp>
#include <opencv2/core/hal/interface.h>


void setupApriltagDetectorFromNode(rclcpp::Node* node){
    // Initialize tag detector with options
    std::string famname_ = node->get_parameter("family").as_string();
    apriltag_detector_t* td_;
    apriltag_family_t* tf_;
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


    td_->quad_decimate = node->get_parameter( "decimate").as_double();
    td_->quad_sigma = node->get_parameter("blur").as_double();
    td_->nthreads = node->get_parameter("reads").as_int();
    td_->debug = node->get_parameter("debug").as_bool();
    td_->refine_edges = node->get_parameter("refine-edges").as_bool();
}


void AprilTagsWrapper::destroyAprilTags() {
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

void AprilTagsWrapper::estimatePose(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
    (void)info;
    // TODO: Can you convert uint8[] to void*?
    // 8UC3 doesn't work? so get the raw value
    const int typ = ((3-1) << 3);
    const cv::Mat frame(msg->height, msg->width, typ);
    memcpy(frame.data, msg->data.data(), msg->data.size()*sizeof(uint8_t));
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
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

