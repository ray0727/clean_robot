#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include "Self_Clean.h"
#include <string>
#include<stdlib.h>

class Point{
	public:
		int x;
		int y;
	Point(){
		x=0,y=0;
	}
	Point(int x,int y){
		this->x=x;
		this->y=y;
	}
};

class Self_Clean{
	
	public:
		Point robot_pose;//robot's now position 
		Point *direct;
		char *map;		//get map data
		int map_width;	
		ros::Subscriber sub1,sub2;
		bool map_init,odom_init; //determind code init
		bool mapcomplete;//check map complete
		double map_resolution; //map resolution m/Grid
		robot_navigate robot_nv;
		ros::Subscriber map_sub,pose_sub;
	Self_Clean();
	~Self_Clean();
	void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void posecallback(const nav_msgs::Odometry::ConstPtr& Pose);
	bool is_init();
//	void straight_feedback();//help car go straight
//	bool check_mapcomplete();
	void direct_init();
	//void appcallback(); //need to set a callback to subscribe app instructment

};

Self_Clean::Self_Clean(){	
	ros::NodeHandle n1;
	map_init=false;
	odom_init=false;
	mapcomplete=false;
	map_sub=n1.subscribe("map",1000,&Self_Clean::mapcallback,this);
	pose_sub=n1.subscribe("odom",1000,&Self_Clean::posecallback,this);
	robot_pose=Point(0,0);//give robot initial condition
	map_resolution=0.1; //defalut 
	direct_init();
	ROS_INFO("1111");
	//n1.subscibe(); //app instrucment;
}

Self_Clean::~Self_Clean(){
	if(map_init){
		/*for(int i=0;i<map_width;i++){
			delete [] map[i];
		}*/
		delete [] map;
		delete [] direct;
	}
}


bool Self_Clean::is_init(){
	return (this->map_init && this->odom_init); 
}

void Self_Clean::mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int data,locate=(map_width/2-robot_pose.y)*map_width+robot_pose.x+map_width/2;//save data 0~100 obstacle probability
	ROS_INFO("mapcallback");
	std::string map_print="";
	if(mapcomplete){
			
		for(int i=0;i<map_width*map_width;i++){
			map_print+=map[i];
			if(i%map_width==63)
				map_print+="\n";
		}
		map[(map_width/2-robot_pose.y)*map_width+robot_pose.x+map_width/2]='x';	
		ROS_INFO("%s",map_print.c_str());
		return;
	}
	if(this->map_init){
		
	//	msg->info.resolution;
		mapcomplete=true;
		for(int i=0;i<map_width*map_width;i++){
			data=msg->data[i];
			if(data==-1){
				map[i]='?';
			}else if(data<=100 && data>9){
				map[i]=data/10-1+'0';
			}else{
				map[i]=' ';
				if(mapcomplete)
					for(int j=0;j<4;j++){
						if(msg->data[i+direct->y*map_width+direct->x]==-1){
							mapcomplete=false;
						}
					}
			}
			if(i==(locate)){map[i]='x';}//show robot position
			ROS_INFO("111");
			map_print+=/*std::to_string(*/map[i]/*)*/;
			if(i%map_width==63)
				map_print+="\n";
			
		}
		//map[(map_width/2-robot_pose.y)*map_width+robot_pose.x+map_width/2]='x';		
		ROS_INFO("%s",map_print.c_str());
	}
	else{
		ROS_INFO("119");
		this->map_init=true;
		map_resolution=msg->info.resolution;
		map_width=msg->info.width; //get the map Grid width
		map=new char [map_width*map_width];
		ROS_INFO("124");
		for(int i=0;i<map_width*map_width;i++){
			data=msg->data[i];
			if(data==-1){
				map[i]='?';
			}else if(data<=100 && data>=9){
				map[i]=data/10+'0';
			}else{
				map[i]=' ';
			}
			
		}
	}

}

void Self_Clean::posecallback(const nav_msgs::Odometry::ConstPtr& Pose){
	float diffx,diffy;
	if(map_init){
		this->robot_nv.orientation=Pose->pose.pose.orientation;
		//get_robot real pose , map[(width/2-y)*width+x+width/2]
		if(Pose->pose.pose.position.x>0){
			diffx=0.5;
		}else{
			diffx=-0.5;
		}
		if(Pose->pose.pose.position.y>0){
			diffy=0.5;
		}else{
			diffy=-0.5;
		}	
		robot_pose.x=(int)((Pose->pose.pose.position.x)/map_resolution+diffx);
		robot_pose.y=(int)(Pose->pose.pose.position.y/map_resolution+diffy);
		odom_init=true;
		ROS_INFO("robot pose:%d,%d",robot_pose.x,robot_pose.y);
	}
}

void Self_Clean::direct_init(){
	direct=new Point[4];
	direct[0]=Point(1,0);
	direct[1]=Point(-1,0);
	direct[2]=Point(0,1);
	direct[3]=Point(0,-1);
	//direct[4]=Point(1,1);
	//direct[5]=Point(-1,1);
	//direct[6]=Point(-1,-1);
	//direct[7]=Point(1,-1);
}

int main(int argc,char *argv[]){
	ros::init(argc,argv,"autoclean_node");
	
	Self_Clean b=Self_Clean();
	//b.close();
	ros::spin();

	return 0;
}
