#include <cstdio>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Image Transport
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
// -----------------------

// OPENCV includes
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
// -----------------------

using namespace cv;

// TODO: fix corrupted unsorted chunks issue
// TODO: why is this so slow?
class CalibrationNode : public rclcpp::Node
{
    public:
    CalibrationNode()
      : Node("calibration")
    {
	it = std::unique_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(rclcpp::Node::SharedPtr(this))
	);
	sub = it->subscribeCamera("/image_raw", 10, std::bind(&CalibrationNode::_drawChessboardCorners, this, std::placeholders::_1, std::placeholders::_2));
	pub = it->advertise("/image_with_chessboard", 10);
    }

    ~CalibrationNode() {
    }


    private:

    void _drawChessboardCorners(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
        (void)info;
	std::cout << msg->encoding << std::endl;
        // TODO: Can you convert uint8[] to void*?
        // 8UC3 doesn't work? so get the raw value
        const int typ = CV_8UC2;
        Mat viewYUV(msg->height, msg->width, typ, (uint8_t*)msg->data.data());
	Mat view;
	cvtColor(viewYUV, view, COLOR_YUV2BGR_YUY2);
	// memcpy(view.data, msg->data.data(), msg->data.size()*sizeof(uint8_t));
	// Mat viewBig = imread("/home/tj-rpi6/Downloads/chessboard.jpg");
	// Mat view;
	// resize(viewBig, view, Size(msg->height, msg->width));
	// TODO: Get board size (width, height)
	Size boardSize(9, 6);
	int winSize = 10;
	std::vector<Point2f> pointBuf;
        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
	bool found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
	if (found) {
	    std::cout << "Chessboard corners found!" << std::endl;
            // improve the found corners' coordinate accuracy for chessboard
            Mat viewGray;
	    // TODO: Make this dependent on msg->encoding
            // cvtColor(view, viewGray, COLOR_YUV2GRAY_YUYV);
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(winSize,winSize),
            Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
            // Draw the corners.
            drawChessboardCorners( view, boardSize, Mat(pointBuf), found );
        }

	// Publish as ImageTransport
	sensor_msgs::msg::Image img_msg; // >> message to be sent
	std_msgs::msg::Header header; // empty header
        header.frame_id = "";
	header.stamp = now(); // time
	cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::RGB8, view);
	img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
	pub.publish(img_msg);
        
    }
    std::unique_ptr<image_transport::ImageTransport> it;
    image_transport::CameraSubscriber sub;
    image_transport::Publisher pub;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world calibration node package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
