/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  起飞（1.2米）   悬停10秒    降落
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <time.h>
#include <stdarg.h>
#include <syslog.h>  

#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlVelYawratePub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;

// 此时不需要GPS的值
//sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;

geometry_msgs::Vector3Stamped V;

Mission square_mission;

// velocity definition
float v_x = 0.0;
float v_y = 0.0;
float v_z = 0.0;

float m_height = 0.0;

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void sLOG(const char* ms)  
{  
        char wzLog[1024] = {0};  
        char buffer[1024] = {0};  
        time_t now;  
        time(&now);  
        struct tm *local;  
        local = localtime(&now);  
        printf("%04d-%02d-%02d %02d:%02d:%02d %s\n", local->tm_year+1900, local->tm_mon,  local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog);  
        sprintf(buffer,"%04d-%02d-%02d %02d:%02d:%02d %s %s \n", local->tm_year+1900, local->tm_mon,local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog, ms);  
        FILE* file = fopen("control1.log","a+");  
        fwrite(buffer,1,strlen(buffer),file);  
        fclose(file);  
        return ;  
}   

void fLOG(const float ms)  
{  
        char wzLog[1024] = {0};  
        char buffer[1024] = {0};  
        time_t now;  
        time(&now);  
        struct tm *local;  
        local = localtime(&now);  
        printf("%04d-%02d-%02d %02d:%02d:%02d %s\n", local->tm_year+1900, local->tm_mon,  local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog);  
        sprintf(buffer,"%04d-%02d-%02d %02d:%02d:%02d %s %s sensor-height %f\n", local->tm_year+1900, local->tm_mon,local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog, ms);  
        FILE* file = fopen("testResut.log","a+");  
        fwrite(buffer,1,strlen(buffer),file);  
        fclose(file);  
        return ;  
}  



bool takeoff_land(int task);
void VelAndYaw(double velocity);

int main(int argc, char** argv)
{
  
  sLOG("log");
  ros::init(argc, argv, "demo_flight_control1");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber velocitySub = nh.subscribe("dji_sdk/velocity",10,&velocity_callback);
  ros::Subscriber height_aboveSub = nh.subscribe("dji_sdk/height_above_takeoff",10,&height_callback);

  // Publish the control signal
  ctrlVelYawratePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  //获取控制权
  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  //A3飞机起飞
  ROS_INFO("A3/N3 taking off!");
  takeoff_result = monitoredTakeoff();

  if(takeoff_result)
  {
    ros::Time start_time = ros::Time::now();
    
    // 起飞的基础之上，悬停
    while(ros::Time::now()-start_time<ros::Duration(10))
    {
       ROS_INFO("##### Start route %d ....");
    }


   ros::Duration(0.01).sleep();
  // ros::spinOnce();
    //飞机降落
    takeoff_land(6);
   
  }                
  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

void VelAndYaw(double velocity)
{
    sensor_msgs::Joy controlVelYawRate;
    
    controlVelYawRate.axes.push_back(velocity);
    controlVelYawRate.axes.push_back(velocity);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    ctrlVelYawratePub.publish(controlVelYawRate);
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
  
}


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
 
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}



void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    V = *msg;
    v_x = V.vector.x;
    v_y = V.vector.y;
    v_z = V.vector.z;


    sLOG("xyzVelocityStart");
    char * m_buffer_x = new char[100];
    sprintf(m_buffer_x,"%.3f", v_x);
    sLOG(m_buffer_x);

    char * m_buffer_y = new char[100];
    sprintf(m_buffer_y,"%.3f", v_y);
    sLOG(m_buffer_y);

    char * m_buffer_z = new char[100];
    sprintf(m_buffer_z,"%.3f", v_z);
    sLOG(m_buffer_z);

    sLOG("xyzVelocityEnd");

    ROS_INFO("VELOCITY_x = [%lf]",V.vector.x);
    ROS_INFO("VELOCITY_y = [%lf]",V.vector.y);
}


void height_callback(const std_msgs::Float32::ConstPtr& msg)
{
  m_height = msg->data;
  
  sLOG("heightStart");
  char * m_buffer_height= new char[100];
  sprintf(m_buffer_height,"%.3f", m_height);
  sLOG(m_buffer_height);

  sLOG("heightEnd");

}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) 
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) 
  {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else 
  {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) 
  {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else 
  {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) 
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}
