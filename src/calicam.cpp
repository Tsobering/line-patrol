#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<ros/ros.h>  
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h> 
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Empty.h>
#include<geometry_msgs/Twist.h>
#include<ardrone_autonomy/Navdata.h>

#include<iostream>  
#include<stdlib.h>  
#include<math.h> 

using namespace cv;
using namespace std;

Mat picture,image_raw,pic,image_roi;

Mat jiaozheng(const Mat img){
	Size image_size;
	Mat image=img;
	image_size.width = image.cols;
	image_size.height =image.rows;
	Mat cameraMatrix=(Mat_<double>(3,3)<<28776.19244627709, 0, 652.5330511574412,
 0, 2715.441922565612, 68.03217023107095,
 0, 0, 1);	
	Mat distCoeffs=(Mat_<double>(1,5)<<21.23088641591238, 42212.6658764911, 5.821176564823733, 0.126619400207104, 12721.13193573747);
	Mat mapx = Mat(image_size,CV_32FC1);
	Mat mapy = Mat(image_size,CV_32FC1);
	Mat R = Mat::eye(3,3,CV_32F);
	initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,image_size,CV_32FC1,mapx,mapy);
	Mat newimage = image.clone();
	remap(image,newimage,mapx, mapy, INTER_LINEAR);
	imshow("after_calibration",newimage);
	return newimage;

}

Mat cut(const Mat image){
Rect rect(390, 30, 500, 300);
image_roi = image(rect);
return image_roi;
}

void video_callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& infomsg){
	
	cv_bridge::CvImagePtr cv_ptr; 
	try
	{   
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		image_raw = cv_ptr->image;
		//sleep(1);
		//cout<<"here"<<endl;
		imshow("camera_capturing", image_raw);
		waitKey(5);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception is %s", e.what());
		return;
	}
	picture=image_raw;
	//cout<<"there"<<endl;
	pic=jiaozheng(picture);
	cut(pic);
	imshow("completed_cut",image_roi);
	
}

void camera_topic_sub(){

	
 	ros::NodeHandle video;   //节点句柄
	image_transport::ImageTransport image_transport(video);//视频发布类节点
	image_transport::CameraSubscriber image_sub;//视频话题接收者
	//
	image_sub = image_transport.subscribeCamera("bebop/image_raw", 1 ,video_callback);//初始化 视频话题接收者
	
	ros::Rate loop_rate(1000);
	//picture=image_raw;
	//cout<<"there1"<<endl;
	
	while(ros::ok())
		{	
		ros::spinOnce();		
				
		 }
	loop_rate.sleep();
	//ros::spin();*/
	
}


int main (int argc,char** argv){ 
	
	ros::init(argc,argv,"cameraacquire");	
	camera_topic_sub();
	waitKey(0);
}
