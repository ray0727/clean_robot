#include<iostream>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
//#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <fstream>
#include<stdio.h>
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void poseCallback(const nav_msgs::Odometry::ConstPtr& Pose);
//void self_go();
ros::Subscriber sub1,sub2;

int x,y;
int test_bit=0;
int **map;
int map_width;

int main(int argc,char *argv[]){
	ros::init(argc,argv,"autoclean_node");
	x=0,y=0;
	ros::NodeHandle n1,n2;
	sub1=n1.subscribe("map",1000,mapCallback);	
	sub2=n2.subscribe("odom",1000,poseCallback);
	
	/*int map=new int* [map_width];
	for (int i=0;i<n;i++){
		map[i]=new int [map_width];
	}
	delete [] map;*/
	ros::spin();

	return 0;
}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int width=msg->info.width;
	map_width=width;
	ROS_INFO("map width:%d\n",msg->info.width);
	//ROS_INFO("map data size:%d\n",sizeof(msg->data)/sizeof(msg->data[0]));
	std::string str="[";	
	int j=0;
	int num=0;
	for(int i:msg->data){
		j++;
		if(j==(width/2-y)*width+x+width/2){
			str+="x";
		}
		else if(i!=-1 && i==0){
			str+=" ";

		}else if(i>0){
			str+=std::to_string((i+12)/13);
		}else{
			str+=std::to_string(0);
		}
		//str+=std::to_string(num);
		str+=",";
		if(j%msg->info.width==0){
			str+="\n";
		}

	}
	str+="]";
	std_msgs::String message;
	message.data=str;
	ROS_INFO("%s",message.data.c_str());
	//ROS_INFO("robot pose:(%d,%d)",x,y);
	//ROS_INFO("robot orentation:");
	/*if(test_bit==0){
		FILE *c1;
		
		c1=fopen("map.txt","w");
		if(c1==NULL){
			ROS_INFO("cannot open\n");
		}
		else{
			fwrite(str.c_str(),sizeof(str.c_str()),width*width,c1);		
			fclose(c1);
		}
		
		test_bit=1;
	}*/

}

void poseCallback(const nav_msgs::Odometry::ConstPtr& Pose ){
	
	x=(int)(Pose->pose.pose.position.x/0.1);
	y=(int)(Pose->pose.pose.position.y/0.1);
	geometry_msgs::Quaternion q=Pose->pose.pose.orientation;
	ROS_INFO("robot pose:(%d,%d)",x,y);
	ROS_INFO("robot orentation:(%f,%f,%f,%f)",q.x,q.y,q.z,q.w);
}


