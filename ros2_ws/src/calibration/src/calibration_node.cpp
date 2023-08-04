#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>

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

#include <yaml-cpp/yaml.h>

// TODO: Remove cv namespace
using namespace cv;

std::string Mat2Str(const Mat& input) {
    cv::Mat oneRow = input.reshape(0, 1);    // Treat as vector
    std::ostringstream os;
    os << oneRow;                             // Put to the stream
    std::string asStr = os.str();             // Get string
    asStr.pop_back();                         // Remove brackets
    asStr.erase(0,1);
    return asStr;
}


// Collection of camera parameters to make organization easier.
struct CameraParameters {
    Mat cameraMatrix;
    Mat distCoeffs;
    Mat R;
    Mat t;
};


// TODO: fix corrupted unsorted chunks issue
// TODO: why is this so slow?
class CalibrationNode : public rclcpp::Node
{
    static constexpr size_t N_CALIBRATION_POINTS = 8;
    static constexpr int SQUARE_SIZE = 2; // cm, approximate
    public:
    CalibrationNode()
      : Node("calibration"),
	_boardSize(9, 6)
    {
	it = std::unique_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(rclcpp::Node::SharedPtr(this))
	);
	sub = it->subscribeCamera("/image_raw", 10, std::bind(&CalibrationNode::_drawChessboardCorners, this, std::placeholders::_1, std::placeholders::_2));
	pub = it->advertise("/image_with_chessboard", 10);
	_createObjectPoints();
    }

    ~CalibrationNode() {
    }


    private:
    // TODO: update this so that this is constexpr
    void _createObjectPoints() {
       for(int i = 0; i<_boardSize.height; i++) {
           for(int j = 0; j<_boardSize.width; j++) {
               _chessboardPoints.push_back(
			       Point3f(SQUARE_SIZE*j,SQUARE_SIZE*i,0));
	   }
       }
    }

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
	int winSize = 10;
	std::vector<Point2f> pointBuf;
        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
	bool found = findChessboardCorners(view, _boardSize, pointBuf, chessBoardFlags);
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
            drawChessboardCorners( view, _boardSize, Mat(pointBuf), found );
	    if (_objectPoints.size() < N_CALIBRATION_POINTS) {
	        _objectPoints.push_back(_chessboardPoints);
	        _imgPoints.push_back(pointBuf);
	    }
	    if (_objectPoints.size() == N_CALIBRATION_POINTS) {
	        _calibrate(Size(viewGray.rows, viewGray.cols));
	    }
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

    void _calibrate(const Size& imgSize) {
        /*
         * Performing camera calibration by
         * passing the value of known 3D points (objpoints)
         * and corresponding pixel coordinates of the
         * detected corners (imgPoints)
        */
        cv::calibrateCamera(_objectPoints, _imgPoints, imgSize, _params.cameraMatrix, _params.distCoeffs, _params.R, _params.t);
	std::cout << "Calibration completed!" << std::endl;

	// TODO: Make this compile
	YAML::Emitter out;
	out << YAML::BeginMap;
	out << YAML::Key << "K";
	out << YAML::Value << Mat2Str(_params.cameraMatrix);
	out << YAML::Key << "distCoeff";
	out << YAML::Value << Mat2Str(_params.distCoeffs);
	out << YAML::Key << "R";
	out << YAML::Value << Mat2Str(_params.R);
	out << YAML::Key << "t";
	out << YAML::Value << Mat2Str(_params.t);
	out << YAML::EndMap;

	std::cout << out.c_str() << std::endl;
	std::ofstream fout("config.yaml");
        fout << out.c_str();
    }

    std::unique_ptr<image_transport::ImageTransport> it;
    image_transport::CameraSubscriber sub;
    image_transport::Publisher pub;
    // TODO: Make this const
    std::vector<Point3f> _chessboardPoints;
    Size _boardSize;
    std::vector<std::vector<Point3f> > _objectPoints;
    std::vector<std::vector<Point2f> > _imgPoints;
    CameraParameters _params;
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
