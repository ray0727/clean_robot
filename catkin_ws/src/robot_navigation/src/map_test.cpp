#include<iostream>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include <fstream>
#include<stdio.h>
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);


ros::Subscriber sub;
int test_bit=0;
int main(int argc,char *argv[]){
	ros::init(argc,argv,"autoclean_node");

	ros::NodeHandle n;
	sub=n.subscribe("map",1000,mapCallback);	

	
	ros::spin();

	return 0;
}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	
	ROS_INFO("map width:%d\n",msg->info.width);
	//ROS_INFO("map data size:%d\n",sizeof(msg->data)/sizeof(msg->data[0]));
	std::string str="[";	
	int j=0;
	int num=0;
	for(int i:msg->data){
		j++;
		if(i!=-1 && i==0){
			str+=" ";

		}else if(i>0){
			str+=std::to_string(1);
		}else{
			str+=std::to_string(2);
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
	if(test_bit==0){
		FILE *c1;
		int width=msg->info.width;
		c1=fopen("map.txt","w");
		if(c1==NULL){
			ROS_INFO("cannot open\n");
		}
		else{
			fwrite(str.c_str(),sizeof(str.c_str()),width*width,c1);		
			fclose(c1);
		}
		
		test_bit=1;
	}

}
