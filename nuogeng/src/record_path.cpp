#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h> 
#include "gps_msgs/Inspvax.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include "driverless_msgs/BaseControlState.h"
#include "driverless_msgs/PathTrackingInfo.h"

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
}gpsMsg_t;

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;

	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

class Record
{
	private:
		void gps_callback(const nav_msgs::Odometry::ConstPtr& msg, const gps_msgs::Inspvax::ConstPtr& gps_msg, 
							const driverless_msgs::BaseControlState::ConstPtr& state_msg, const driverless_msgs::PathTrackingInfo::ConstPtr& tracking_info);
		std::string file_path_;
		std::string	file_name_;
		FILE *fp;
		gpsMsg_t last_point , current_point;
		
		float sample_distance_;
//		ros::Subscriber sub_gps_;

		message_filters::Subscriber<nav_msgs::Odometry> sub_utm;
		message_filters::Subscriber<gps_msgs::Inspvax> sub_gps;
		message_filters::Subscriber<driverless_msgs::BaseControlState> sub_state;
		message_filters::Subscriber<driverless_msgs::PathTrackingInfo> sub_tracking_info;
		typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,gps_msgs::Inspvax,driverless_msgs::BaseControlState,driverless_msgs::PathTrackingInfo> syncPolicy;
		typedef message_filters::Synchronizer<syncPolicy> Sync;
		boost::shared_ptr<Sync> sync_;
		
	public:
		Record();
		~Record();
		bool init();
		void recordToFile();
};

Record::Record()
{
	last_point = {0.0,0.0,0.0,0.0,0.0};
	current_point = last_point;
}

Record::~Record()
{
	if(fp != NULL)
		fclose(fp);
}

bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_path",file_path_,"");
	private_nh.param<std::string>("file_name",file_name_,"");
	
	if(file_path_.empty() || file_name_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	std::string utm_topic = private_nh.param<std::string>("utm_topic","/ll2utm_odom");
	std::string gps_topic = private_nh.param<std::string>("gps_topic","/gps");
	std::string state_topic = private_nh.param<std::string>("state_topic","/base_control_state");
	std::string tracking_info_topic = private_nh.param<std::string>("tracking_info_topic", "/tracking_info");

//	sub_gps_ = nh.subscribe(utm_topic ,1,&Record::gps_callback,this);
	//同步两个消息的话题数据

	sub_utm.subscribe(nh, utm_topic, 1);
	sub_gps.subscribe(nh, gps_topic, 1);
	sub_state.subscribe(nh, state_topic, 1);
	sub_tracking_info.subscribe(nh, tracking_info_topic, 1);

	sync_.reset(new Sync(syncPolicy(10),sub_utm,sub_gps,sub_state, sub_tracking_info));
	sync_->registerCallback(boost::bind(&Record::gps_callback, this, _1, _2, _3, _4));
//	message_filters::TimeSynchronizer<nav_msgs::Odometry,gps_msgs::Inspvax> sync(sub_utm, sub_gps, 10);
//	sync.registerCallback(boost::bind(&Record::gps_callback, this, _1, _2));



	fp = fopen((file_path_+file_name_).c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path_+file_name_).c_str());
		return false;
	}
	ROS_INFO("Record path success!!!");
	return true;
}

void Record::gps_callback(const nav_msgs::Odometry::ConstPtr& msg, const gps_msgs::Inspvax::ConstPtr& gps_msg,
							const driverless_msgs::BaseControlState::ConstPtr& state_msg,
							const driverless_msgs::PathTrackingInfo::ConstPtr& tracking_info)
{
	static size_t  row_num = 0;
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];
	if(sample_distance_*sample_distance_ <= dis2Points(current_point,last_point,false))
	{
		fprintf(fp,"%.3f\t%.3f\t%.4f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",current_point.x,current_point.y,current_point.yaw,
													gps_msg->east_velocity, gps_msg->north_velocity,
													gps_msg->body_acceleration_x, gps_msg->body_acceleration_y,
													state_msg->roadWheelAngle, tracking_info->lateral_err);
		fflush(fp);
		
		// ROS_INFO("row:%d\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",row_num++,current_point.x,current_point.y,current_point.yaw,
		// 											gps_msg->east_velocity, gps_msg->north_velocity,
		// 											gps_msg->body_acceleration_x, gps_msg->body_acceleration_y,
		// 											state_msg->roadWheelAngle);
		last_point = current_point;
	}
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record record;
	
	if(!record.init())
		return 1;

	ros::spin();
	
	return 0;
}


