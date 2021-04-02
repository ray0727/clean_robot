#include<string>
#include<string.h>
#include<stdlib.h>
#include "clean_robot.h"
#include "map_scan.h"
#include "self_clean.h"

class Clean_Robot{
	public:
		ros::Subscriber map_sub,slam_pose_sub,clean_region_sub,slave_pose_sub;
		ros::Publisher  point_pub,map_pub,command_pub;//publish robot where should it go to?,publsh the master map;
		ros::NodeHandle node_;
		List_node *clean_map;

	public:
		//----system status
		int robot_status;//store robot status 
		enum robot_status_definerobot_status_define{Initial=1,Normal_Mode=2,Map_Complete=4,Map_notComplete=8,Robot_Move=16,Robot_Scan=32,Search_Unknown=64,Outer_Command=128,TWO_robot=256};		
		bool odom_init; //determind code init
		bool isrobotmove=false;
		//----robot info
		Point robot_pose;//robot's now position 
		Point old_pose;//the old position of robot
		Pointf master_pose; 

		//----map info
		Map_Scan *map_;		//get map data,and Print data[y*width+x]
		int *map_data;

		//-----self clean
		Self_Clean sc;
		Pointf region[2];
		
	Clean_Robot();
	~Clean_Robot();
	void publish_clean_robot_map();
	void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void slamposecallback(const geometry_msgs::PoseStamped::ConstPtr& Pose);
	void clean_regioncallback(const std_msgs::String::ConstPtr& region_msg);
	void slavepointcallback(const geometry_msgs::PoseStamped::ConstPtr& p);	
	bool is_init(){return (this->map_->mapinit && this->odom_init);}
	bool check_getmap(){return !(map_->mapcomplete) && !isrobotmove;} //map not complete and robot not move,it can get map

};

Clean_Robot::Clean_Robot(){
	//----initial subscribe and publish
	map_sub=node_.subscribe("master/map",1000,&Clean_Robot::mapcallback,this);
	slam_pose_sub=node_.subscribe("master/slam_out_pose",1000,&Clean_Robot::slamposecallback,this);
	clean_region_sub=node_.subscribe("clean_region",1000,&Clean_Robot::clean_regioncallback,this);
	slave_pose_sub=node_.subscribe("slave_robot_point",1000,&Clean_Robot::slavepointcallback,this);
	command_pub=node_.advertise<std_msgs::String>("master_command",1000);
	point_pub=node_.advertise<geometry_msgs::PoseStamped>("master/move_base_simple/goal",1000);
	map_pub=node_.advertise<nav_msgs::OccupancyGrid>("clean_robot_map",1000);
	
	//----robot initial state
	robot_pose=Point(0,0);
	old_pose=Point(0,0);
	map_=new Map_Scan[1];
	robot_status=Initial;

	//----system state
	odom_init=false;	
	isrobotmove=false;
	ROS_INFO("Clean_Robot Bulid");

	//----clean_robot_map
	master_pose=Pointf(0,0);
	
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
		sc.init(width,map_->map,resolution);
	}else if(!check_getmap()){
		//map_->print_map(robot_pose);
	}else{
		for(int i=0;i<width*width;i++)
			map_data[i]=msg->data[i];
		map_[0].checkmap(map_data,robot_pose);
		publish_clean_robot_map();
	}
}

void Clean_Robot::slamposecallback(const geometry_msgs::PoseStamped::ConstPtr& Pose)
{
	//ROS_INFO("slam_pose:%lf,%lf",Pose->pose.position.x,Pose->pose.position.y);
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
		//ROS_INFO("robot pose:%d,%d",robot_pose.x,robot_pose.y);
	}
	master_pose=Pointf(Pose->pose.position.x,Pose->pose.position.y);
}

void Clean_Robot::clean_regioncallback(const std_msgs::String::ConstPtr& region_msg){
	ROS_INFO("region callback");
	if(robot_status & Initial)
		return;
	robot_status|=Outer_Command;
	std::string s=region_msg->data;
	char * cstr = new char [s.length()+1];
  	std::strcpy (cstr, s.c_str());

  	// cstr now contains a c-string copy of str

  	char * p = std::strtok (cstr,"\n");
	p = std::strtok (NULL,"\n");
	int i=0;
  	for(i=0;i<2 && p!=0;i++)
  	{
		this->region[i].x=atof(p);
    	p = std::strtok(NULL,"\n");
		this->region[i].y=atof(p);
    	p = std::strtok(NULL,"\n");
  	}
	if(this->region[0].x<this->region[1].x){
		float t=this->region[0].x;
		this->region[0].x=this->region[1].x;
		this->region[1].x=t;
	}
	if(this->region[0].y<this->region[1].y){
		float t=this->region[0].y;
		this->region[0].y=this->region[1].y;
		this->region[1].y=t;
	}
	if(robot_status==(Normal_Mode | TWO_robot | Outer_Command)){
		s="";
		s+="clean_region\n";
		if(abs(region[0].x-region[1].x)>abs(region[0].y-region[1].y)){
			s+=std::to_string((region[0].x+region[1].x)/2)+"\n"+std::to_string(region[0].y)+"\n"+std::to_string(region[1].x)+"\n"+std::to_string(region[1].y)+"\n";
			region[1].x=(region[0].x+region[1].x)/2;
		}else{
			s+=std::to_string(region[0].x)+"\n"+std::to_string((region[0].y+region[1].y)/2)+"\n"+std::to_string(region[1].x)+"\n"+std::to_string(region[1].y)+"\n";
			region[1].y=(region[0].y+region[1].y)/2;
		}
		std_msgs::String send;
		send.data=s;
		this->command_pub.publish(send);
	}
  	delete[] cstr;
	
}

void Clean_Robot::publish_clean_robot_map(){
	nav_msgs::OccupancyGrid map;
	map.info.resolution=map_[0].map_resolution;
	map.info.width=map_[0].map_width;
	map.info.height=map_[0].map_width;
	map.info.origin.position.x=master_pose.x;
	map.info.origin.position.y=master_pose.y;

	std::vector<int8_t>& data=map.data;
	//std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
	data.resize(map.info.width*map.info.height);
	for(int i=0;i<map.info.width*map.info.height;i++){
		data[i]=(int8_t)(map_[0].map[i]);
	}
	map_pub.publish(map);
}

void Clean_Robot::slavepointcallback(const geometry_msgs::PoseStamped::ConstPtr& p){
	robot_status|=TWO_robot;
}

int main(int argc,char *argv[]){
	
	ros::init(argc,argv,"cleanrobot_node");
	Clean_Robot main=Clean_Robot();
	
	while(ros::ok()){
		ros::spinOnce();

		switch(main.robot_status){
			case main.Initial:
				if(main.is_init()){
					ROS_INFO("initial over");
					main.robot_status|=main.Map_Complete;
					main.robot_status^=main.Initial;
				}
				break;
			case (main.Normal_Mode | main.TWO_robot):
			case main.Normal_Mode:

				break;
			case main.Map_Complete:
				ROS_INFO("Map compleete");
				main.publish_clean_robot_map();
				main.robot_status^=main.Map_Complete;
				main.robot_status|=main.Normal_Mode;
				break;
			case (main.Normal_Mode | main.Outer_Command | main.TWO_robot):
			case (main.Normal_Mode | main.Outer_Command):
				main.sc.start_clean(main.region);
				main.robot_status^=(main.Outer_Command | main.Normal_Mode);
				main.robot_status|=main.Robot_Move;
				if(!main.sc.send_point(main.point_pub))
					main.robot_status=main.Normal_Mode;
				break;
			case (main.Robot_Move | main.TWO_robot):
			case main.Robot_Move:
				if(main.sc.is_arrive(main.robot_pose)){
					if(!main.sc.send_point(main.point_pub)){
						main.robot_status^=main.Robot_Move;
						main.robot_status|=main.Normal_Mode;
					}
				}
				break;
		}
		
	}

	return 0;
}



