#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher vel_pub_;

public:
  int x_coord;
  int cur_dis;
  int min_dis;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, 
      &ImageConverter::imageCb, this);
	image_depth_sub_ = it.subscribe("/camera/depth/image_raw", 10, &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
	  std::cout << "closing camera..." << std::endl;
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

    int row = 0; 
    int col = 0; 
    int dis_min = 1000000; 
    for (int i=0; i < cv_ptr->image.rows; i++) {
        for (int j=0; j < cv_ptr->image.cols; j++) {
            int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
            int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
            int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
            // int dis = pow((r-255)*(r-255) + g*g + b*b, 0.5); 
            int dis = (r-255)*(r-255) + g*g + b*b; 
            //ROS_INFO("b: %d", b); 
            //ROS_INFO("g: %d", g); 
            //ROS_INFO("r: %d", r); 
            //ROS_INFO("%d", dis); 
            if (dis < dis_min) {
                dis_min = dis; 
                row = i; 
                col = j; 
            }
        }
    }

    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[0]); 
    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[1]); 
    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[2]); 
    // ROS_INFO("height: %d", msg->height); 
    // ROS_INFO("width: %d", msg->width); 
    // ROS_INFO_STREAM("encoding: " << msg->encoding); 

   
    //ROS_INFO("r: %d", row); 
    //ROS_INFO("c: %d", col);
	x_coord = row;

    cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }

};

void depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		// encoding for depth images
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	}
	catch (const cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	cur_dis = cv_ptr->image.at<unsigned char>(row,col);
	ROS_INFO("Distance: %d", cur_dis);

	if(cur_dis > min_dis)
	{
		// too far
	}

	//---- do the angular and linear velocity ----//


}
/*
class Tracker
{
	ros::NodeHandle nh_;
	ros::Publisher vel_pub;
	ros::Subscriber vel_sub;

public:
	Tracker()
	{
		vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		vel_sub = nh_.subscribe("/cmd_vel", 10, velCallback);
	}

	void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
	{
		geometry_msgs::Twist new_vel;
		new_vel.angular.z = 0.75;
		vel_pub.publish(new_vel);
	}
};
*/
int main(int argc, char** argv)
{
	std::cout << "Running RGB camera..." << std::endl;
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
