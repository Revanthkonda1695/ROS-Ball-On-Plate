#include<ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <geometry_msgs/Point.h>

using namespace cv;
using namespace std;

// Values for Red color ball

//int thresh1 = 0;
//int thresh2 = 15;
//int sval1 = 85;
//int sval2 = 255;
//int vval1 = 40;
//int vval2 = 255;


int thresh1 = 0;
int thresh2 = 19;
int sval1 = 96;
int sval2 = 255;
int vval1 = 153;
int vval2 = 255;


int main(int argc,char** argv) {
	ros::init(argc, argv, "camera_tracker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("point", 100); //publishing topic called point
	ros::Rate rate(100);

	VideoCapture stream1(1);   //0 is the id of video device.0 if you have only one camera.
	stream1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	stream1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	stream1.set(CV_CAP_PROP_FPS, 100);

	if (!stream1.isOpened()) { //check if video device has been initialised
		cout << "cannot open camera";
	}
	namedWindow("ctl", 1); //creating an interface
	createTrackbar("Hue", "ctl", &thresh1, 255);
	createTrackbar("Hue2", "ctl", &thresh2, 255);
	createTrackbar("Sat1", "ctl", &sval1, 255);
	createTrackbar("Sat2", "ctl", &sval2, 255);
	createTrackbar("Val1", "ctl", &vval1, 255);
	createTrackbar("Val2", "ctl", &vval2, 255);

	while(ros::ok())
	{
		Mat src, src_hsv, src_gray, dst, dst2; //Mat is a datatype used for reading images
		stream1.read(src); //reading a frame from the video into src
		cvtColor(src, src_gray, cv::COLOR_BGR2GRAY); //converting to greyscale and storing in src_gray
		cvtColor(src, src_hsv, cv::COLOR_BGR2HSV); //converting to hsv and storing in src_hsv
		inRange(src_hsv, Scalar(thresh1, sval1, vval1), Scalar(thresh2, sval2, vval2), dst); //thresholding so that only the ball is detected and storing in dst
		cv::Moments m = moments(dst, true); //calculating the moments of the shape detected in the above step (in our case the ball)
		double area = countNonZero(dst); //calculating area by counting the number of non zero pixels 
		double m10 = m.m10; 
		double m01 = m.m01;
		double m00 = m.m00;
		int posx = m10 / m00;
		int posy = m01 / m00;
		if (area < 15 * 15) { posx = 160; posy = 120; }
		cvtColor(dst, dst2, cv::COLOR_GRAY2BGR);
		circle(dst2, Point(posx, posy), 10, Scalar(0, 0, 255), -1);

		imshow("cam", src);
		imshow("fil", dst2);
		if (waitKey(30) >= 0)
			break;
		printf("%d, %d \n", posx , posy);
		geometry_msgs::Point msg;
		msg.x = posx;
		msg.y = posy;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

