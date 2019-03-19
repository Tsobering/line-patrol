//巡线飞行实现文件；两个类：巡线方法类，无人机类;
//开发者：ZT/重庆大学自动化学院智慧工程研究院无人机项目组
//定义文件(函数和变量的定义应该只包含在一个CPP文件中，否则会多重定义)

#include<bebop2_flight.h>

using namespace std;
using namespace cv;

//定义变量
struct droneData mydroneData;  //无人机传回数据

/////////////////////////////////////
///////////////函数实现///////////////
////////////////////////////////////
//图像数据获取函数
void imageData(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg)
{
	 cv_bridge::CvImagePtr cv_ptr; 
	//转换成BGR格式的原始图像并显示
	try
	{
		//转化成CVImage    
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		mydroneData.imageRow_ = cv_ptr->image;
		waitKey(5);  //必须要加这个函数，否则图像窗口显示不了
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception is %s", e.what());
		return;
	}
	imshow("row image", mydroneData.imageRow_);
}


//导航数据获取函数
void navigationData(const ardrone_autonomy::NavdataConstPtr & nav)
{
		mydroneData.altitude_ = nav->altd;
		mydroneData.battery_=nav->batteryPercent;
		cout<<"无人机剩余电量为： "<<mydroneData.battery_<<endl;
		cout<<"无人机高度为： "<<mydroneData.altitude_<<endl;	
}


///////////////////////////////////////////////
/////////////巡线方法类实现//////////////////////
//////////////////////////////////////////////

//构造函数；初始化参数，只需一次初始化的参数
PatrolFlight::PatrolFlight()
	:   flight_angle(0.3),
	    flight_dist(100),
		times(0),
		fly_s(false),
		sta_s(true),
		ss(0),
		rr(0),
		call_vf(true)
{		
	//初始化自适应参数——定高
	u_z = 0.0;
	hat_w_z = 0.0;
	hat_w_sec_z = 0.0;

	//初始化自适应参数——偏航
	u_yaw = 0.0;
	hat_w_yaw = 0.0;
	hat_w_sec_yaw = 0.0;

	//初始化自适应参数——侧移
	u_dist = 0.0;
	hat_w_dist = 0.0;
	hat_w_sec_dist = 0.0;
			
	//初始化颜色参数
	ilowh = 20;
	ihighh = 40;
	ilows = 50;
	ihighs = 255;
	ilowv = 0;
	ihighv = 255;
	
	parameterInitialization();//其他参数初始化
		
}

//析构函数
PatrolFlight::~PatrolFlight()
{
	
}

//参数初始化函数，需要多次初始化的参数
void PatrolFlight::parameterInitialization()
{
	//初始化速度
	speedpid.linear.x=0;
	speedpid.linear.y=0;
	speedpid.linear.z=0;
	speedpid.angular.z=0;
	
	my_error={0.0,0.0,0.0,0.0};  //初始化误差结构体
	pixel_dist=0.0;//每个像素点的物理长度
	line_a = 0.0;  //扫描线宽度
	line_b = 0.0;//真实线宽度
	
	//图像大小
	icolst=0; 
	irowst=0;
	imidtc=0;
	imidtr=0;
	
	//图像检测参数
	iaimr1_l=0, iaimr1_r=0;//左右两边目标点
	iaimr1=0;//第一行中间实际目标点
	iaimr10_l=0, iaimr10_r=0;//十分之一行处左右两边的目标点
	iaimr10 = 0;//十分之一行处中间实际目标点
	getaimr1 = false;//第一行检测标志
	getaimr10 = false;//第十行检测标志
	getaimr = false;//行检测标志

	iaimc1_u=0, iaimc1_d=0;//第一列上下两边目标点
	iaimc1 = 0;//第一列实际目标点
	iaimcn_u = 0, iaimcn_d = 0;//最后一列上下两边目标点
	iaimcn=0;//最后一列的实际目标点
	iaimc10_u = 0, iaimc10_d = 0;//十分之一列上下两边目标点
	iaimc10 = 0;//十分之一列的实际目标点
	iaimc_10_u = 0, iaimc_10_d = 0;//后十分之一列上下两边目标点
	iaimc_10 = 0;//十分之一列的实际目标点
	getaimc1 = false;//第一列检测标志
	getaimcn = false;//最后一列检测标志
	getaimcn_10=false;//右列检测标志
	getaimc1_10=false;//左列检测标志
	getaimc10 = false;//十分之一列检测标志
	getaimc_10 = false;//十分之一列检测标志
	getaimc = false;//列检测标志
}

//图像畸变矫正函数
Mat PatrolFlight::imgCorrection(const Mat img)
{
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
	return newimage;
}

//图片旋转函数
void PatrolFlight::imgRotate(Mat& img, Mat& newIm, double angle)
{
    Point2f pt(img.cols/2.,img.rows/2.);  //旋转中心
    Mat r = getRotationMatrix2D(pt,angle,1.0);  //1.0为缩放尺度
    warpAffine(img,newIm,r,img.size());    //旋转
}

//图像剪切函数
Mat PatrolFlight::imgCut(const Mat image)
{
	Rect rect(320, 20, 436, 204);
	Mat image_roi = image(rect);
	return image_roi;
}

//把数据写入文件
void PatrolFlight::dataTotext(const struct errors errors_)
{
	char buffer[1024] = {0};
	//buffer<<errors_.angle<<"\t"<<errors_.dist<<"\t"<<errors_.true_angle<<"\n"<<endl;  
	sprintf(buffer,"%0.2f\t%0.2f\t%0.2f\n",errors_.angle,errors_.dist,errors_.true_angle);
	FILE* file = fopen("data.txt","a+");  
    fwrite(buffer,1,strlen(buffer),file);  
    fclose(file);  
    cout <<errors_.angle<<"\t"<<errors_.dist<<"\t"<<errors_.true_angle<<"\n"<<endl;
}

//自适应定高
double PatrolFlight::adaptiveHeight(double eheight)
{
	double k_z = 0.5;
	double pho_z = 0.001;
	double delta_z = 0.001;
	
	double e_z = eheight;
	double u_1 = -k_z*e_z-pho_z*e_z*hat_w_z;
	
	hat_w_sec_z = hat_w_z+(pho_z*e_z*e_z-delta_z*hat_w_z)*0.001;
	hat_w_z = hat_w_sec_z;
	
	if(u_1>=0.15)
	{
		u_z = 0.15;
	}
	else if(u_1<=-0.15)
	{
		u_z = -0.15;
	}
	else
	{
		u_z = u_1;
	}

	return u_z;
}

//自适应调整偏航角
double PatrolFlight::adaptiveYaw(double error_yaw)
{	
	double k_yaw = 0.7;
	double pho_yaw = 0.001;
	double delta_yaw = 0.001;
	
	double e_yaw = -error_yaw;
	double u_2 = -k_yaw*e_yaw-pho_yaw*e_yaw*hat_w_yaw;
	
	hat_w_sec_yaw = hat_w_yaw+(pho_yaw*e_yaw*e_yaw-delta_yaw*hat_w_yaw)*0.01;
	hat_w_yaw = hat_w_sec_yaw;
	
	//换成角度，增大180/3.14倍
	u_2 = u_2*3.14/180;
	
	if((u_2)>=0.8)
	{
		u_yaw = 0.8;
	}
	else if((u_2)<=-0.8)
	{
		u_yaw = -0.8;
	}
	else
	{
		u_yaw = (u_2);
	}

	return u_yaw;
}

//自适应侧移
double PatrolFlight::adaptiveDist(double error_dist)
{
	double k_dist = 0.0005;
	double pho_dist = 0.0001;
	double delta_dist = 0.0001;
	
	double e_dist = -error_dist;
	double u_3 = -k_dist*e_dist-pho_dist*e_dist*hat_w_dist;
	
	hat_w_sec_dist = hat_w_dist+(pho_dist*e_dist*e_dist-delta_dist*hat_w_dist)*0.01;
	hat_w_dist = hat_w_sec_dist;
	
	if(u_3>=0.05)
	{
		u_dist = 0.05;
	}
	else if(u_3<=-0.05)
	{
		u_dist = -0.05;
	}
	else
	{
		u_dist = u_3;
	}

	return u_dist;
}

//路径检测函数
void PatrolFlight::pathDetection(const Mat& image_row)
{
	//检测参数再次初始化
	parameterInitialization();
	
	//转换为HSV格式
	Mat hsv;
	cvtColor(image_row, hsv, COLOR_BGR2HSV);

	//直方图均衡化
	vector<Mat> hsvsplit;
	split(hsv, hsvsplit);
	equalizeHist(hsvsplit[2], hsvsplit[2]);
	merge(hsvsplit, hsv);
	
	//二值化
	inRange(hsv, Scalar(ilowh, ilows, ilowv), Scalar(ihighh, ihighs, ihighv), throed);

	//开闭操作
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(throed, throed, MORPH_OPEN, element);
	morphologyEx(throed, throed, MORPH_CLOSE, element);

	//获取图像大小
	icolst = throed.cols;
	irowst = throed.rows;
	imidtc = icolst / 2;
	imidtr = irowst / 2;
	cout<<"图像宽: 图像高:"<<icolst<<" "<<irowst<<endl;

	//开始检测
	//检测最后一行目标点
	//先正向扫描目标点
	for (int i = 0; i < icolst; ++i)
	{
		if (throed.at<uchar>(irowst-1, i) == 255)
		{
			iaimr1_l = i;
			break;	
		}
	}
	//再反向扫描
	for (int j = icolst - 1; j >= 0; --j)
	{
		if (throed.at<uchar>(irowst-1, j) == 255)
		{
			iaimr1_r = j;
			break;
		}
	}
	//如果检测到两端，并且距离较长，可能存在实际目标点
	if ((iaimr1_r - iaimr1_l) > 3)
	{
		//如果中间点是目标点，则检测到实际目标点
		if (throed.at<uchar>(irowst-1, (iaimr1_l + iaimr1_r) / 2) == 255)
		{
			line_a = iaimr1_r - iaimr1_l;//最后一行线宽度
			iaimr1 = (iaimr1_l + iaimr1_r) / 2;
			getaimr1 = true;			
		}
	}	
	
	//如果检测到最后一行，说明箭头在最后一行
	if (getaimr1 == true)
	{
		//再检测倒数十分之一行
		//先正向扫描目标点
		for (int i = 0; i < icolst; ++i)
		{
			if (throed.at<uchar>(irowst*0.9, i) == 255)
			{
				iaimr10_l = i;
				break;
			}
		}
		//再反向扫描
		for (int j = icolst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(irowst*0.9, j) == 255)
			{
				iaimr10_r = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimr10_r - iaimr10_l) > 3)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>(irowst*0.9, (iaimr10_l + iaimr10_r) / 2) == 255)
			{
				line_a=(line_a + (iaimr10_r - iaimr10_l))/2;//扫描线宽度
				iaimr10 = (iaimr10_l + iaimr10_r) / 2;
				getaimr10 = true;
			}
		}
	}

	//检测到行
	if (getaimr1 && getaimr10)
	{	
		//箭头角度
		my_error.angle = atan2(-(iaimr10-iaimr1), (irowst / 10))* (180 / PI);
		//每个像素的长度,总长度为60000um
		pixel_dist=60000/(line_a * cos(fabs(my_error.angle)));
		//箭头距离,精确到MM
		my_error.dist = (imidtc-iaimr1)*(pixel_dist/1000);
		getaimr = true;
	}
	else //行检测失败，检测列
	{		
		//检测第一列
		//先向下扫描目标点
		for (int i = 0; i < irowst; ++i)
		{
			if (throed.at<uchar>(i, 0) == 255)
			{
				iaimc1_u = i;
				break;
			}
		}
		//再向上扫描目标点
		for (int j = irowst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(j, 0) == 255)
			{
				iaimc1_d = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimc1_d - iaimc1_u) > 3)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimc1_d + iaimc1_u) / 2, 0) == 255)
			{
				iaimc1 = (iaimc1_d + iaimc1_u) / 2;
				getaimc1 = true;
			}
		}
		
		//如果检测到第一列，再检测十分之一列
		if(getaimc1 = true)
		{
			//先向下扫描目标点
			for (int i = 0; i < irowst; ++i)
			{
				if (throed.at<uchar>(i, (icolst / 10)) == 255)
				{
					iaimc10_u = i;
					break;
				}
			}
			//再向上扫描目标点
			for (int j = irowst - 1; j >= 0; --j)
			{
				if (throed.at<uchar>(j, (icolst / 10)) == 255)
				{
					iaimc10_d = j;
					break;
				}
			}
			//如果检测到两端，并且距离较长，可能存在实际目标点
			if ((iaimc10_d - iaimc10_u) > 3)
			{
				//如果中间点是目标点，则检测到实际目标点
				if (throed.at<uchar>((iaimc10_d + iaimc10_u) / 2, (icolst / 10)) == 255)
				{
					line_a=((iaimc1_d - iaimc1_u) + (iaimc10_d - iaimc10_u))/2;
					iaimc10 = (iaimc10_d + iaimc10_u) / 2;
					getaimc10 = true;
					getaimc1_10=true;
				}
			}
		}

		//再检测最后一列
		//先向下扫描目标点
		for (int i = 0; i < irowst; ++i)
		{
			if (throed.at<uchar>(i, icolst - 1) == 255)
			{
				iaimcn_u = i;
				break;
			}
		}
		//再向上扫描目标点
		for (int j = irowst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(j, icolst - 1) == 255)
			{
				iaimcn_d = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimcn_d - iaimcn_u) > 3)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimcn_d + iaimcn_u) / 2, icolst - 1) == 255)
			{			
				iaimcn = (iaimcn_d + iaimcn_u) / 2;
				getaimcn = true;
			}
		}
		//如果检测到最后一列，再检测后十分之一列
		if(getaimcn = true)
		{
			//先向下扫描目标点
			for (int i = 0; i < irowst; ++i)
			{
				if (throed.at<uchar>(i, (icolst / 10) * 9) == 255)
				{
					iaimc_10_u = i;
					break;
				}
			}
			//再向上扫描目标点
			for (int j = irowst - 1; j >= 0; --j)
			{
				if (throed.at<uchar>(j, (icolst / 10) * 9) == 255)
				{
					iaimc_10_d = j;
					break;
				}
			}
			//如果检测到两端，并且距离较长，可能存在实际目标点
			if ((iaimc_10_d - iaimc_10_u) > 3)
			{
				//如果中间点是目标点，则检测到实际目标点
				if (throed.at<uchar>((iaimc_10_d + iaimc_10_u) / 2, (icolst / 10) * 9) == 255)
				{
					if(getaimc1_10==false)
					{
						line_a=((iaimcn_d - iaimcn_u)+(iaimc_10_d - iaimc_10_u))/2;
					}			
					iaimc_10 = (iaimc_10_d + iaimc_10_u) / 2;
					getaimc_10 = true;
					getaimcn_10=true;
				}
			}
		}
		
		//如果检测到
		if (getaimcn_10 == true || getaimc1_10 == true)
		{
			//如果两边都检测到
			if (getaimcn_10 == true && getaimc1_10 == true)
			{
				if (iaimc1 < iaimcn)  //算第一列
				{				
					//计算箭头弧度
					my_error.angle = atan2((icolst / 10), (iaimc10-iaimc1))* (180 / PI);
					//每个像素的长度,总长度为60000um
					pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
					//计算头距离,精确到MM
					my_error.dist = (imidtc + (irowst-iaimc1) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
					//得到角度和距离
					getaimc = true;		
				}
				else  //算最后一列
				{				
						//计算箭头弧度
						my_error.angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn))*(180 / PI);
						//每个像素的长度,总长度为60000um
						pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
						//计算箭头距离
						my_error.dist = -(imidtc + (irowst-iaimcn) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
						//
						getaimc = true;				
				}
			}
			else if (getaimc1_10 == true && getaimcn_10 == false)//如果只检测到第一列
			{		
					//计算箭头弧度
					my_error.angle = atan2((icolst / 10), (iaimc10-iaimc1))* (180 / PI);
					//每个像素的长度,总长度为60000um
					pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
					//计算头距离,精确到MM
					my_error.dist = (imidtc + (irowst-iaimc1) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
					//得到角度和距离
					getaimc = true;				
			}
			else  //如果只检测到最后一列
			{			
					//计算箭头弧度
					my_error.angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn))*(180 / PI);
					//每个像素的长度,总长度为60000um
					pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
					//计.算箭头距离
					my_error.dist = -(imidtc + (irowst-iaimcn) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
					//
					getaimc = true;			
			}		
		}
	}
	
	if (getaimc == false && getaimr == false)
	{
		++times;
		call_vf = false;//停止视频检测循环，飞机降落
	}
	else
	{
		//角度小于十度，距离小于30进入轨道飞行
		if(fabs(my_error.angle) < flight_angle && fabs(my_error.dist) < flight_dist)
		{
			fly_s = true;
			++ss;
			rr=0;
			if(ss==1)
			{
				sta_s=true;
				cout<<"到达轨道，稳定无人机准备飞行"<<endl;
			}
			if(ss>=2)
			{	
				ss=2;
			 	sta_s=false;
			}
		}	
		else
		{
			fly_s = false;
			ss=0;
			++rr;
			if(rr==1)
			{
				sta_s=true;
				cout<<"遇到弯道，稳定无人机准备拐弯"<<endl;
			}
			else
			{
				rr=2;
				sta_s=false;
			}
		}
		
		//返回偏角和距离
		cout<<"pixel_dist:" << (pixel_dist/1000)<<endl;
		cout << "偏角：                        " << my_error.angle << endl;
		cout << "距离：" << my_error.dist << endl;
	}
	
	//把数据写入文件
	dataTotext(my_error);
}

//速度配置函数,正常摄像头
geometry_msgs::Twist PatrolFlight::Adaptive()
{
	//如果在稳定态
	if(sta_s)
	{
		speedpid.linear.x =0;
		speedpid.linear.y =0;
		speedpid.linear.z =0;
		speedpid.angular.z =0;
		cout<<"稳定无人机准备飞行或转弯"<<endl;
	}
	else
	{
		//如果在飞行态
		if (fly_s)
		{
			if(fabs(my_error.angle)<0.15 && fabs(my_error.dist)<100)
			{
				speedpid.linear.x = 0.2;
			}
			else
				speedpid.linear.x = 0.1;		
			
			speedpid.linear.y =0.1*adaptiveDist(my_error.dist);//以毫米为单位
			
			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
           
			cout<<"无人机正在巡航"<<endl;
		}
		//调整态，平移和旋转
		else 
		{
			speedpid.linear.y =0.3*adaptiveDist(my_error.dist);
			speedpid.linear.x = 0.05;
			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
		
			cout<<"无人机正在转弯"<<endl;
			
		}
	}
	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "```````````````````````角速度 ：            " << speedpid.angular.z << endl;

	return speedpid;
}

//速度配置函数,畸变摄像头
geometry_msgs::Twist PatrolFlight::AdaptiveUnnormal()
{
	if(fabs(my_error.dist)<flight_dist){
		if(fabs(my_error.angle)<flight_angle){
			speedpid.linear.x = 0.05;//0.1
		
			speedpid.linear.y =0.2*adaptiveDist(my_error.dist);
		
			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
		}
		else{
			speedpid.linear.x = 0.02;//0.1
		
			speedpid.linear.y =0.2*adaptiveDist(my_error.dist);
		
			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
		}
	
		cout<<"无人机只转弯"<<endl;
	}
	else{
			speedpid.linear.y =0.5*adaptiveDist(my_error.dist);
			speedpid.linear.x = 0.01;
			speedpid.angular.z =0.0;
			cout<<"无人机只平移"<<endl;
	}

	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "```````````````````````角速度 ：            " << speedpid.angular.z << endl;

	return speedpid;
}

//更改自适应参数——定高
void PatrolFlight::ChangeControlParameters_h(double u_z_,double hat_w_z_,double hat_w_sec_z_)
{
	u_z=u_z_;
	hat_w_z=hat_w_z_;
	hat_w_sec_z=hat_w_sec_z_;
}

//更改自适应参数——偏航
void PatrolFlight::ChangeControlParameters_z(double u_yaw_,double hat_w_yaw_,double hat_w_sec_yaw_)
{
	u_yaw=u_yaw_;
	hat_w_yaw=hat_w_yaw_;
	hat_w_sec_yaw=hat_w_sec_yaw_;
}

//更改自适应参数—侧移
void PatrolFlight::ChangeControlParameters_y(double u_dist_,double hat_w_dist_,double hat_w_sec_dist_)
{
	u_dist=u_dist_;
	hat_w_dist=hat_w_dist_;
	hat_w_sec_dist=hat_w_sec_dist_;
}

//设定飞行状态阈值
void PatrolFlight::SetFlightThreshold(float flight_angle_,float flight_dist_)
{
	flight_angle=flight_angle_;
	flight_dist=flight_dist_;
}

//设定检测线的颜色
void PatrolFlight::SetLineColor(int ilowh_,int ihighh_,int ilows_,int ihighs_,int ilowv_,int ihighv_)
{
	ilowh=ilowh_;
	ihighh=ihighh_;
	ilows=ilows_;
	ihighs=ihighs_;
	ilowv=ilowv_;
	ihighv=ihighv_;
}



///////////////////////////////////////////////
/////////////无人机类实现///////////////////////
/////////////////////////////////////////////

//构造函数,初始化参数
Drone::Drone()
	:
		image_transport(drone),
		start_line(false),
		call_rh(true),
		altitude(0.0),
		battery(0.0),
		target_height(1.0),
		error_height(0.0),
		FixedFlight(false)
{
		//初始化 视频话题接收者       (压入回调函数队列)
	 	image_sub = image_transport.subscribeCamera("bebop/image_raw", 1 ,imageData); //imageData必须是已经定义(已分配地址)的函数
		//初始化 导航数据话题接收者      (压入回调函数队列) //类必须要实例化成员函数才会分配有地址
	 	rh_navdata_sub=drone.subscribe<ardrone_autonomy::Navdata>("bebop/navdata",1,navigationData);
		//初始化 无人机移动话题发布者
	 	move_pub = drone.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
		//初始化 相机移动话题发布者
		camera_move = drone.advertise<geometry_msgs::Twist>("bebop/camera_control", 1);
		//初始化 着陆话题发布者
		land_pub=drone.advertise<std_msgs::Empty>("bebop/land",1);	
		//初始化 着陆话题发布者
		takeoff_pub=drone.advertise<std_msgs::Empty>("bebop/takeoff", 1);
			
		//速度初始化
		speedadjust.linear.x=0;
		speedadjust.linear.y=0;
		speedadjust.linear.z=0;
		speedadjust.angular.z=0;
		//相机初始化
		cameraadjust.angular.y=0.0;
		cameraadjust.angular.z=0.0;

}
		
//析构函数
Drone::~Drone()
{
	
}

//参数初始化
void Drone::parameterInitialization()
{
		//速度初始化
		speedadjust.linear.x=0;
		speedadjust.linear.y=0;
		speedadjust.linear.z=0;
		speedadjust.angular.z=0;
		//相机初始化
		cameraadjust.angular.y=0.0;
		cameraadjust.angular.z=0.0;
}

//起飞
void Drone::takeoff()
{
	ros::Rate loop_rate(10);
	int second=0;
	while(ros::ok())
	{
		//at last four time can be fly steady
		if(second<5)
		{	//发布起飞话题，开始起飞
			takeoff_pub.publish(empty_off);
			cout<<"pub takeoff：无人机起飞"<<endl;
		}	
		else
			break;
	    loop_rate.sleep();
		++second;		
	}
}

//降落
void Drone::land()
{
		
	ros::Rate loop_rate(10);
	int second=0;	
	cout<<"进入land"<<endl;
	while(ros::ok())
	{
		//only one time can be landed
		if(second<5)
		{
			//发送着陆指令
			land_pub.publish(empty_land);
			cout<<"pub land：无人机开始降落"<<endl;
		}
		else
			break;		 
		++second;
	    loop_rate.sleep();		
	}
}

//转圈
void Drone::circle()
{
	ros::Rate loop_rate(10);
	
	speedadjust.linear.x=0.02;
	speedadjust.linear.y=0.0;
	speedadjust.linear.z=0.0;
	speedadjust.angular.x=0.0;
	speedadjust.angular.y=0.0;
	speedadjust.angular.z=-0.2;
	
	while(ros::ok())
	{
			move_pub.publish(speedadjust);
			cout<<"pub land：无人机绕圈"<<endl;		
	    	loop_rate.sleep();		
	}
	
}

//无人机稳定
void Drone::stable()
{	
	ros::Rate loop_rate(20);
	int second=0;

    speedadjust.linear.x=0.0;
    speedadjust.linear.y=0.0;
	speedadjust.linear.z=0.0;
	speedadjust.angular.x=0.0;
	speedadjust.angular.y=0.0;
	speedadjust.angular.z=0.0;
	
	while(ros::ok())
	{
		//only one time can be landed
		if(second<5)
		{
			move_pub.publish(speedadjust);  
			cout<<"稳定无人机"<<endl;
		}
		else
			break;	 
		++second;
	    loop_rate.sleep();		
	}
}

//巡线函数
void patrolFlight_main(Drone& myDrone,PatrolFlight& myPatrolFlight)
{
	//打开键盘文件
	int keys_fd;  
	struct input_event t;  
    keys_fd = open ("/dev/input/event4", O_RDONLY);  
	if (keys_fd <= 0)  
	{  
	  	cout<<"open /dev/input/event4 device error!\n"<<endl;  
	  	return ;  
	}  
	
	 //开始手动调整
	 while (!myDrone.start_line)
	 {
		  //读取按键控制无人机
		 if (read (keys_fd, &t, sizeof (t)) == sizeof (t))  
		 {  
			 //cout<<"按键控制"<<endl;
			 if (t.type == EV_KEY)  
					if (t.value == 1)  
				   {  
					     if(t.code==KEY_ESC)
						  break;
						 switch(t.code)
						 {
									 case KEY_O:
									{
							 			cout<<"开始起飞"<<endl;
										myDrone.takeoff();//
										break;
									}
								 	case KEY_C:
									{
							 			cout<<"开始绕圈"<<endl;
										myDrone.circle();//
										break;
									}
									case KEY_L:
									{
							 		  cout<<"开始降落"<<endl;
									  myDrone.land();
									  break;
									}
									case KEY_F:
									{ 
									  myDrone.start_line=true;
							 		  myDrone.parameterInitialization();
							          cout<<"开始巡线"<<endl;
									  break;
									}
								   case KEY_LEFT:
								   {
									 myDrone.speedadjust.linear.y=0.2;
									 cout<<"左移"<<endl;
							 		 break;

								   }
								   case KEY_RIGHT:
								   {
									 myDrone.speedadjust.linear.y=-0.2;
								     cout<<"右移"<<endl;
									 break;
								   }
									case KEY_UP_UP:
								   {
									 myDrone.speedadjust.linear.x=0.2;
									 cout<<"前进"<<endl;
									 break;
								   }
								   case KEY_DOWN_DOWN:
								   {
								 	 myDrone.speedadjust.linear.x=-0.2;
									 cout<<"后退"<<endl;
									 break;
								   }
								 	case KEY_D:
								   {
									 myDrone.speedadjust.angular.z=-0.2;
									 cout<<"右转"<<endl;
									 break;
								   }
									case KEY_A:
								   {
									 myDrone.speedadjust.angular.z=0.2;
									 cout<<"左转"<<endl;
									 break;
								   }
								  case KEY_W:
								  {
									myDrone.speedadjust.linear.z=0.2;
									cout<<"上升"<<endl;
									 break;
								  }
								  case KEY_S:
								  {
									myDrone.speedadjust.linear.z=-0.2;
								 	cout<<"下降"<<endl;
									 break;
								  }
								  case KEY_H:
								  {
									 myDrone.parameterInitialization();
								 	 cout<<"稳定"<<endl;
									 break;
								  }
								 case KEY_I:
								{
									myDrone.cameraadjust.angular.y=50;
									myDrone.cameraadjust.angular.z=0.0;
									myDrone.camera_move.publish(myDrone.cameraadjust);
									myDrone.parameterInitialization();
									cout<<"镜头向上"<<endl;
									break;
								}
								case KEY_K:
								{
									myDrone.cameraadjust.angular.y=-40;
									myDrone.cameraadjust.angular.z=0.0;
									myDrone.camera_move.publish(myDrone.cameraadjust);
									myDrone.parameterInitialization();
									cout<<"镜头向下"<<endl;
									break;
								}
								   default:
										break;	 
						 }
						 myDrone.move_pub.publish(myDrone.speedadjust);
						 myDrone.parameterInitialization();
				   }  
				  else
				  {
					 myDrone.parameterInitialization();
					 myDrone.move_pub.publish(myDrone.speedadjust);
					 cout<<"无人机稳定"<<endl;
				  }
		 } 
	
	 }	
		
	//无人机进入自主巡线，超过十秒未寻到线降落
	if(myDrone.start_line)
    {
		cout<<"自主巡线"<<endl;
		ros::Rate loop_rate(15);
		signal(SIGINT, MySigintHandler);//覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown()
		while(ros::ok())
		{	
				if(myPatrolFlight.call_vf)
				{ 
					ros::spinOnce();  //执行一次回调函数队列，获取无人机传回数据
					
					//myDrone.imageRow=mydroneData.imageRow_;
					//myPatrolFlight.image_row=myDrone.imageRow; //获得原始图像	
					//PatrolFlight.image_jz = PatrolFlight.imgCorrection(PatrolFlight.image_row);//图像进行矫正
					//PatrolFlight.imgRotate(PatrolFlight.image_jz,PatrolFlight.image_rotate,-36);//图片进行旋转
					//PatrolFlight.image_cut = PatrolFlight.imgCut(PatrolFlight.image_rotate);//图像进行剪切
					//imshow("原始图像", mydroneData.imageRow_);
					
					//图像检测函数，计算方向弧度和距离偏差
					myPatrolFlight.pathDetection(mydroneData.imageRow_);
					
					//如果检测到路径
					if (myPatrolFlight.call_vf == true)
					{	
						//运用算法配置速度
						myPatrolFlight.speedpid = myPatrolFlight.AdaptiveUnnormal();
						//发送控制指令移动无人机
						myDrone.move_pub.publish(myPatrolFlight.speedpid);
						myPatrolFlight.times=0;
					}
					else
					{
						cout << "no path have been find :没有检测到路径" << endl;
						myPatrolFlight.call_vf =true;
						if(myPatrolFlight.times==150)
						{
							myPatrolFlight.call_vf =false;
						}
					}
				}
			    else
				{
					myDrone.land();
					cout << "十秒后巡线失败开始降落" << endl;
					return ;
				}
				loop_rate.sleep();
		 }	
		close (keys_fd);			
	}
}

//紧急调用的降落函数
void land_()
{
	 ros::NodeHandle land;   //节点句柄
	 ros::Publisher land_pub;   //着陆话题发布者
	 //初始化 着陆话题发布者
   	land_pub=land.advertise<std_msgs::Empty>("bebop/land",1);
	std_msgs::Empty empty_land;
	ros::Rate loop_rate(10);
	int second=0;
	while(ros::ok())
	{
		//only one time can be landed
		if(second<5)
		{
			//发送着陆指令
			land_pub.publish(empty_land);
			cout<<"pub land：无人机开始降落"<<endl;
		}
		else
			break;		 
		++second;
	    loop_rate.sleep();	
	}	
}

//紧急调用的退出降落函数
//覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown()
//调用ros::shutdown()后，所有ROS服务已经不可以使用
void MySigintHandler(int sig)
{
	//这里主要进行退出ROS前的无人机降落和其他的数据保存、内存清理、告知其他节点等工作
	land_();//无人机降落
	ROS_INFO("shutting down ROS!");
	ros::shutdown();
}
