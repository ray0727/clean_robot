#include<stdio.h>

#include "clean_robot.h"
#include "map_scan.h"
//#include "self_clean.h"

class Clean_Robot{
	protected:
		ros::Subscriber map_sub,slam_pose_sub;
		ros::Publisher  point_pub;
		ros::NodeHandle node_;
		List_node *clean_map;

	public:
		//----system status
		int robot_status;//store robot status 
		enum robot_status_define{Normal_Mode,Map_Complete,Map_notComplete,Robot_Move,Robot_Scan,Search_Unknown};		
		bool odom_init; //determind code init
		bool isrobotmove=false;
		//----robot info
		Point robot_pose;//robot's now position 
		Point old_pose;//the old position of robot 

		//----map info
		Map_Scan *map_;		//get map data,and Print data[y*width+x]
		int *map_data;
		
	Clean_Robot();
	~Clean_Robot();
	void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void slamposecallback(const geometry_msgs::PoseStamped::ConstPtr& Pose);	
	bool check_getmap(){return !(map_->mapcomplete) && !isrobotmove;} //map not complete and robot not move,it can get map

};

Clean_Robot::Clean_Robot(){
	//----initial subscribe and publish
	map_sub=node_.subscribe("map",1000,&Clean_Robot::mapcallback,this);
	slam_pose_sub=node_.subscribe("slam_out_pose",1000,&Clean_Robot::slamposecallback,this);	
	point_pub=node_.advertise<geometry_msgs::Point>("robot_point",1000);
	
	//----robot initial state
	robot_pose=Point(0,0);
	old_pose=Point(0,0);
	map_=new Map_Scan[1];
	robot_status=Normal_Mode;

	//----system state
	odom_init=false;	
	isrobotmove=false;
	ROS_INFO("Clean_Robot Bulid");
	
}

Clean_Robot::~Clean_Robot(){
	delete [] map_;
	delete [] map_data;
}
void Clean_Robot::mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	ROS_INFO("In clean_robot.cpp::mapcallback");
	int width=msg->info.width;

	if(!(map_->mapinit)){
		ROS_INFO("map initail");		
		//map_initial
		double resolution=msg->info.resolution;
		map_data=new int [width*width];
		for(int i=0;i<width*width;i++)
			map_data[i]=msg->data[i];
		map_->map_init(resolution,width,map_data);
	}else if(!check_getmap()){
		map_->print_map(robot_pose);
	}else{
		for(int i=0;i<width*width;i++)
			map_data[i]=msg->data[i];
		map_[0].checkmap(map_data,robot_pose);
	}
}

void Clean_Robot::slamposecallback(const geometry_msgs::PoseStamped::ConstPtr& Pose)
{
	ROS_INFO("slam_pose:%lf,%lf",Pose->pose.position.x,Pose->pose.position.y);
	float diffx,diffy;
	if(map_->mapinit){
		if(Pose->pose.position.x>0){
			diffx=0.5;
		}else{
			diffx=-0.5;
		}
		if(Pose->pose.position.y>0){
			diffy=0.5;
		}else{
			diffy=-0.5;
		}	
		robot_pose.x=(int)((Pose->pose.position.x)/map_->map_resolution+diffx);
		robot_pose.y=(int)(Pose->pose.position.y/map_->map_resolution+diffy);
		odom_init=true;
		ROS_INFO("robot pose:%d,%d",robot_pose.x,robot_pose.y);
	}
}

int main(int argc,char *argv[]){
	
	ros::init(argc,argv,"cleanrobot_node");
	Clean_Robot main=Clean_Robot();
	
	while(ros::ok()){
		ros::spinOnce();
		
		
	}

	return 0;
}



