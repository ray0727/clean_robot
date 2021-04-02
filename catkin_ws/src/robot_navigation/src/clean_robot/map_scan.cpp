#include"map_scan.h"
#include <stdio.h>
#include <string>
#include<stdlib.h>

Map_Scan::Map_Scan(){
	ROS_INFO("generate map_scan");
	//slavestatus_sub=node_.subscribe("slave_status",1000,&Map::slavecallback,this);
	//master_pub=node_.advertise<>
	map_resolution=0;
	map=NULL;
	map_width=0;
	mapinit=false;
	mapcomplete=false;
	direct=direct_init();
}

Map_Scan::~Map_Scan(){
	ROS_INFO("delete map_scan");
	direct_delete(direct);
	delete [] map;
}

void Map_Scan::map_init(double resolution,int width,int *msg_data){
	map_resolution=resolution;
	map_width=width; //get the map Grid width
	map=new char [map_width*map_width];
	int data;
	for(int i=0;i<map_width*map_width;i++){
		data=msg_data[i];
		if(data==-1){
			map[i]='2';
		}else if(data<=100 && data>=9){
			map[i]='1';
		}else{
			map[i]=' ';
		}	
	}
	print_map(Point(0,0));
	this->mapinit=true;
}

void Map_Scan::print_map(Point robot_pose){
	std::string map_print="";
	ROS_INFO("%d %d",robot_pose.x,robot_pose.y);
	int locate=(map_width/2+robot_pose.y)*map_width+robot_pose.x+map_width/2;
	for(int i=0;i<map_width*map_width;i++){
		map_print+=map[i];
		map_print+=' ';
		if(i%map_width==63)
			map_print+="\n";
	}
	map_print[locate*2+locate/map_width]='x';
	ROS_INFO("\n%s",map_print.c_str());	
}

bool Map_Scan::checkmap(int *msg_data,Point robot_pose){
		this->mapcomplete=true;
		int data;
		ROS_INFO("checkmap");
		for(int i=0;i<map_width*map_width;i++){
			data=msg_data[i];
			//ROS_INFO("%d",i);
			if(data==-1){
				map[i]='2';
			}else if(data==100){
				map[i]='1';
			}else{
				map[i]=' ';
				if(mapcomplete && i>2*map_width && i<(map_width*(map_width-2)-1))
					for(int j=0;j<4;j++){
						int around=i+direct[j].y*map_width+direct[j].x;
						if(msg_data[around]==-1 && (msg_data[around+map_width]>=10 || msg_data[around+1]>=10 || msg_data[around-map_width]>=10 || msg_data[around-1]>=10)){
							//if the unKnown grid beside the wall map is ok,because robot can not pass;
							mapcomplete=false;						
						}
					}
			}			
		}
		//print_map(robot_pose);
		return mapcomplete;
}

Point* Map_Scan::direct_init(){
	Point* direct=new Point[8];
	direct[0]=Point(1,0);
	direct[1]=Point(-1,0);
	direct[2]=Point(0,1);
	direct[3]=Point(0,-1);
	direct[4]=Point(1,1);
	direct[5]=Point(-1,1);
	direct[6]=Point(-1,-1);
	direct[7]=Point(1,-1);
	return direct;
}

void Map_Scan::direct_delete(Point* direct){
	for(int i=0;i<8;i++)
		delete [] direct;
}
