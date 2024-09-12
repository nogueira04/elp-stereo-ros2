#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>


class SplitImage : public rclcpp::Node
{
  public:
    SplitImage();
    void SyncImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void init();
  
  private:
    image_transport::Subscriber sync_image_sub_;
    image_transport::CameraPublisher left_image_pub_;
    image_transport::CameraPublisher right_image_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_left_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_right_;
    std::string camera_ns; 
};


SplitImage::SplitImage() : Node("split_sync_image_node")
{
  this->declare_parameter<std::string>("camera_ns", "elp");
  this->get_parameter("camera_ns", camera_ns);

}

void SplitImage::init()
{
  std::string left_cam_info;
  std::string right_cam_info;
  this->declare_parameter<std::string>("left_cam_info", "");
  this->declare_parameter<std::string>("right_cam_info", "");

  this->get_parameter("left_cam_info", left_cam_info);
  this->get_parameter("right_cam_info", right_cam_info);

  RCLCPP_INFO(this->get_logger(), "Left camera info path: %s", left_cam_info.c_str());
  RCLCPP_INFO(this->get_logger(), "Right camera info path: %s", right_cam_info.c_str());

  cinfo_left_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "left_cam_info", "file://" + left_cam_info);
  cinfo_right_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "right_cam_info", "file://" + right_cam_info);
  
  auto node_ptr = shared_from_this();  
  image_transport::ImageTransport it(node_ptr); 
  left_image_pub_ = it.advertiseCamera("left/image_raw", 1);
  right_image_pub_ = it.advertiseCamera("right/image_raw", 1);

  sync_image_sub_ = it.subscribe("/" + camera_ns + "/image_raw", 1, 
                                 std::bind(&SplitImage::SyncImageCallback, this, std::placeholders::_1));
}

void SplitImage::SyncImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
    std::string left_frame = camera_ns + "_left_optical_frame";
    std::string right_frame = camera_ns + "_right_optical_frame";

    try {
      cv_ptr_left = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_right = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    int combined_rows = cv_ptr_left->image.rows;
    int combined_cols = cv_ptr_left->image.cols;
    int image_cols = combined_cols / 2;
    int image_rows = combined_rows;

    cv::Rect leftROI(0, 0, image_cols, image_rows);
    cv::Rect rightROI(image_cols, 0, image_cols, image_rows);
    cv::Mat leftcrop = cv_ptr_left->image(leftROI);
    cv::Mat rightcrop = cv_ptr_right->image(rightROI);

    cv_ptr_left->image = leftcrop;
    cv_ptr_right->image = rightcrop;
    cv_ptr_left->header.frame_id = left_frame;
    cv_ptr_right->header.frame_id = right_frame;

    auto ci_left_ = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_left_->getCameraInfo());
    auto ci_right_ = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_right_->getCameraInfo());
    ci_left_->header = cv_ptr_left->header;
    ci_right_->header = cv_ptr_right->header;

    left_image_pub_.publish(cv_ptr_left->toImageMsg(), ci_left_);
    right_image_pub_.publish(cv_ptr_right->toImageMsg(), ci_right_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SplitImage>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
