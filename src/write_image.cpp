#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
//#include "/home/demulab/catkin_ws/src/darknet_ros/darknet_ros/include/darknet_ros/YoloObjectDetector.h" 

darknet_ros_msgs::BoundingBoxes bbox;
static const std::string OPENCV_WINDOW = "Detect image";

const int INIT_NUMBER = 16799;

void bboxCb(const darknet_ros_msgs::BoundingBoxes & msg)
//void bboxCb(const std_msgs::String::ConstPtr& msg)
{
  bbox = msg;
  for(unsigned int i = 0; i < msg.boundingBoxes.size(); ++i) {
  //std::cout << msg << std::endl;
    std::cout << "class:"  << msg.boundingBoxes.at(i).Class << std::endl;
    std::cout << "prob :"  << msg.boundingBoxes.at(i).probability << std::endl;
    std::cout << "xmin:"  << msg.boundingBoxes.at(i).xmin << std::endl;
    std::cout << "ymin:"   << msg.boundingBoxes.at(i).ymin << std::endl;
    std::cout << "xmax:"  << msg.boundingBoxes.at(i).xmax << std::endl;
    std::cout << "ymax:"   << msg.boundingBoxes.at(i).ymax << std::endl; 
  }
}
  
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub_bbox_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/darknet_ros/detection_image", 1,
    			       &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
        try
	  {
	    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  }
	catch (cv_bridge::Exception& e)
	  {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	  }

	// Draw an example circle on the video stream
	//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	//  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	int i = 0;
	int xmin = 0, ymin = 0, xmax = 0, ymax = 0,size=0;
	std::string Class;
	
	size = bbox.boundingBoxes.size();
	if (size == 0) return;

	static int data_no = INIT_NUMBER;
	
	for (int i=0; i < size; i++) {
	  Class = bbox.boundingBoxes.at(i).Class;
	  xmin = bbox.boundingBoxes.at(i).xmin;
	  ymin = bbox.boundingBoxes.at(i).ymin;
	  xmax = bbox.boundingBoxes.at(i).xmax;
	  ymax = bbox.boundingBoxes.at(i).ymax;
	  std::cout << "Class:" << Class << "xmin:"  << xmin << " ymin:"  << ymin <<
	    " xmax:" << xmax << " ymax:"  << ymax << std::endl;
	  
	  if (Class == "person") {
	    if ((0 <= xmin && xmin <= cv_ptr->image.cols) && (0 <= ymin && ymin <= cv_ptr->image.rows)
		&& (0 <= xmax && xmax <= cv_ptr->image.cols) && (0 <= ymax && ymax <= cv_ptr->image.rows)) {
	      
	      cv::Mat detect_image(cv_ptr->image, cv::Rect(xmin,ymin,xmax-xmin,ymax-ymin));
	      std::ostringstream oss;
	      oss << data_no++;
	      
	      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	      cv::imshow(OPENCV_WINDOW, detect_image);
	      cv::imwrite(oss.str() +".jpg",detect_image);
	      //cv::waitKey(3);
	      // Output modified video stream
	      image_pub_.publish(cv_ptr->toImageMsg());
	    }
	  }
	  cv::waitKey(3);
	}
	
  }
};
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "write_image");
  ImageConverter ic;
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,bboxCb);
  
  ros::Rate loop_rate(33);
  
  
  while(ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
