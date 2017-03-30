#include <ros/ros.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";

class Follow
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher vel_pub_;

	// angular (turn left or right), linear (move forward or back)
	// can have values -1, 0 or 1
	int angular;
	int linear;
	float cur_linear;

	// stores the coord of the ball
	int ball_row, ball_col;

public:
	Follow(): it_(nh_)
	{
		// init turn and move vars
		angular = linear = cur_linear = 0;

		// subscribe to video
		image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &Follow::image_callback, this);
		depth_sub_ = it_.subscribe("camera/depth/image", 10, &Follow::depth_callback, this);

		// publish
		image_pub_ = it_.advertise("/image_converter/output_video", 10);
		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
		
		// window with video feed
		cv::namedWindow(OPENCV_WINDOW);
	}
	

	~Follow()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// gets the reddest pixel
		// based on the x coord determine turn direction
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

		for(int i = 0; i < cv_ptr->image.rows; i++)
		{
			for(int j = 0; j < cv_ptr->image.cols; j++)
			{
				int b = cv_ptr->image.at<cv::Vec3b>(i,j).val[0];
				int g = cv_ptr->image.at<cv::Vec3b>(i,j).val[1];
				int r = cv_ptr->image.at<cv::Vec3b>(i,j).val[2];

				int dis = (r - 255) * (r - 255) + g * g + b * b;

				if(dis < dis_min)
				{
					dis_min = dis;
					row = i;
					col = j;
				}
			}
		}

		ball_row = row;
		ball_col = col;

		// determine if turning left or right or not turning
		// ball on left 45% screen = left, right 45% = right else dont turn
		if(ball_col < msg->width * 0.45)
			angular = 1;
		else if(ball_col > msg->width * 0.55)
			angular = -1;
		else
			angular = 0;

		// draws circle on GUI
		cv::circle(cv_ptr->image, cv::Point(col,row), 10, CV_RGB(0,255,255));
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);

		// output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());

		//move();
	}

	void depth_callback (const sensor_msgs::Image::ConstPtr& msg)
	{
		// gets the distance to the ball using depth cam
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			float distance = cv_ptr->image.at<float>(ball_row, ball_col);
			ROS_INFO("Distance: %f", distance);
			
			// determine if moving forward/back/stationary
			if(distance < 1.0)
				linear = -1;
			else if(distance > 1.1)
				linear = 1;
			else if(isnan(distance))
				;
			else
				linear = 0;

			move();
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

	void move()
	{
		// send velocity messages to bot
		
		// these variables store the linear and ang. speed
		float lspd = 0.200;
		float aspd = 0.50;

		geometry_msgs::Twist vel_msg;

		smooth_speed(linear, aspd, 0.05);

		vel_msg.linear.y = vel_msg.linear.z = 0;
		vel_msg.angular.x = vel_msg.angular.y = 0;
		vel_msg.linear.x = cur_linear;
		vel_msg.angular.z = aspd * angular;
		
		vel_pub_.publish(vel_msg);
	}

	void smooth_speed(int dir, float aspd, float accel) {
		if(cur_linear < (dir * aspd)) 
		{
			cur_linear += accel;
		} 
		else if(cur_linear > (dir * aspd))
		{
			cur_linear -= accel;
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_follow");
	Follow follow_node;
	ros::spin();
	return 0;
}
