#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <stdio.h>
#include "Self_Clean.h"
#include <string>
#include<stdlib.h>
#define unknown_point_num 20
#define important_node_num 40

/*
1.size of robot is about 30cm*30cm
2.map_resolution is 

*/

class Point{
	public:
		int x;
		int y;
		bool is_search;
		//Point *next;
	Point(){
		x=0,y=0,is_search==false;
	}
	Point(int x,int y){
		this->x=x;
		this->y=y;
		this->is_search=false;
	}
};

//--------2020.3.02-----
class List_node{
		
	public:
		Point *point;
		List_node *next;
		List_node *previous;
	List_node(){
		next=NULL;
		point=NULL;
		previous=NULL;
	}
};
//-------------------------

class Self_Clean{
	
	public:
		Point robot_pose;//robot's now position 
		Point old_pose;//the old position of robot 
		Point *direct;
		Point *unknown;//save unknown point
		List_node *important_node;//save important_node
		List_node *start=NULL;
		List_node *search_node=NULL;
		char *map;		//get map data
		int map_width;	
		ros::Subscriber sub1,sub2;
		bool map_init,odom_init; //determind code init
		bool mapcomplete;//check map complete
		bool isrobotmove=false;
		double map_resolution; //map resolution m/Grid
		robot_navigate robot_nv;//robot info type
		int robot_status;//store robot status 
		enum robot_status_define{ Normal_Mode,Map_Complete,Map_notComplete,Robot_Move,Robot_Scan,Search_Unknown};
	protected:
		ros::Subscriber map_sub,pose_sub;
		ros::Publisher  point_pub;
		//ros::Publisher //send motor control info
	public:	
	Self_Clean();
	~Self_Clean();
	void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void posecallback(const nav_msgs::Odometry::ConstPtr& Pose);
	bool is_init();
//	void straight_feedback();//help car go straight
//	bool check_mapcomplete();
	void direct_init();
	bool check_getmap(){return map_init && mapcomplete==false && isrobotmove==false;} //map not complete and robot not move,it can get map
	void search_pathtounknown();
	bool set_mode(int status);
	bool is_arrive();
	void send();
	Point start_clean();
	//void appcallback(); //need to set a callback to subscribe app instructment

};

Point Self_Clean::start_clean(){
	
	int x,y;
	int locate;
	Point start=Point();
	bool is_break=false;
	for(x=1;x<map_width-1;x++){
		for(y=1;y<map_width-1;y++){
			locate=(y)*map_width+x;
			if(map[locate]==' ' && map[locate+map_width]==' ' && map[locate-map_width]==' ' && map[locate+1]==' ' && map[locate-1]==' '){
				start.x=x-map_width/2;
				start.y=y-map_width/2;
				is_break=true;
				break;
			}
		}	
		if(is_break)
			break;
	}
	geometry_msgs::Point point;
	point.x=(double)start.x;
	point.y=(double)start.y;
	point_pub.publish(point);
	ROS_INFO("%d,%d",x,y);
	return start;
}

bool Self_Clean::is_arrive(){


}

bool Self_Clean::set_mode(int status){
	switch(status){
		case Normal_Mode:
			isrobotmove=false;
			//mapcomplete==false;
			
			break;
		case Map_Complete:
			
			break;
		case Map_notComplete:
			 
			break;
		case Robot_Move:
			
			break;
		case Robot_Scan:			
			break;

		default:
			ROS_INFO("In Self_Clean:set mode error");
			break;
	}
}

Self_Clean::Self_Clean(){	
	ros::NodeHandle n1;
	map_init=false;
	odom_init=false;
	mapcomplete=false;
	map_sub=n1.subscribe("map",1000,&Self_Clean::mapcallback,this);
	pose_sub=n1.subscribe("odom",1000,&Self_Clean::posecallback,this);
  	point_pub=n1.advertise<geometry_msgs::Point>("robot_point",1000);
	//give robot initial condition
	robot_pose=Point(0,0);
	old_pose=Point(0,0);
	map_resolution=0.1; //defalut 
	direct_init();
	ROS_INFO("Run in Self_Clean");
	//---save the point in the map is unknown 
	unknown=new Point[unknown_point_num ];
	for(int i=0;i<unknown_point_num;i++)
	{
		unknown[i]=Point(0,0);
	}
	//---save the point it's route
	important_node=new List_node [1];
	this->robot_status=Normal_Mode;
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
	//----2020.3.01
	List_node *old_node=important_node;
	while(old_node=NULL){
		ROS_INFO("delete List_node");
		delete [] old_node->point;
		old_node=old_node->next;
	}
}

//map self_go 

bool Self_Clean::is_init(){
	return (this->map_init && this->odom_init); 
}

void Self_Clean::mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	int data,locate=(map_width/2+robot_pose.y)*map_width+robot_pose.x+map_width/2;//data save  0~100 obstacle probability, locate save robot location
	
	ROS_INFO("in mapcallback");
	std::string map_print="";
	if(check_getmap()==false && map_init==true){//if map is complete,it will not refresh  
			
		for(int i=0;i<map_width*map_width;i++){
			map_print+=map[i];
			if(i%map_width==63)
				map_print+="\n";
		}
		//show robot_pose in map
		map[(map_width/2+old_pose.y)*map_width+old_pose.x+map_width/2]=' ';	
		map[(map_width/2+robot_pose.y)*map_width+robot_pose.x+map_width/2]='x';	
		old_pose.x=robot_pose.x;
		old_pose.y=robot_pose.y;

		ROS_INFO("%s",map_print.c_str());//print map
		return;
	}
	if(check_getmap()){
		
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
				//int a=0;
				if(mapcomplete)
					for(int j=0;j<4;j++){
						int around=i+direct[j].y*map_width+direct[j].x;
						if(msg->data[around]==-1 && (msg->data[around+map_width]>=10 || msg->data[around+1]>=10 || msg->data[around-map_width]>=10 || msg->data[around-1]>=10)){
							//if the unKnown grid beside the wall map is ok,because robot can not pass;
							mapcomplete=false;
							//-----2021.3.01 search map
						/*	int a=0;
							while(!(unknown[a].x==0 && unknown[a].y==0) && ++a);
							if(a==0){
								unknown[a]=Point(around%map_width,around/map_width);
							}else{								
								if((unknown[a].x+2)>=unknown[a-1].x && (unknown[a].x-2)<=unknown[a-1].x  && (unknown[a].y+2)>=unknown[a-1].y &&  (unknown[a].y-2)<=unknown[a-1].y){
									unknown[a-1].x=(unknown[a].x+unknown[a-1].x)/2;
									unknown[a-1].y=(unknown[a].y+unknown[a-1].y)/2;
									unknown[a].x=0,unknown[a].y=0;
									a--;
								}
							}
							ROS_INFO("the pose map not complete:(%d,%d)",unknown[a].x,unknown[a].y);*/
						}
					}
			}


			map_print+=/*std::to_string(*/map[i]/*)*/;
			if(i%map_width==63)
				map_print+="\n";
			
		}
		//----2021.3.01--------
		if(mapcomplete){
			ROS_INFO("map is complete");
			robot_status=Map_Complete ; //map is complete,it can start clean
			isrobotmove=true;
			start_clean();
		}
		else{
			ROS_INFO("map is not complete");
			//robot_status=Map_notComplete; //map is not complete,it should still scan map
			//isrobotmove=false;
		}
			//-----------------------
		map_print[locate+locate/map_width]='x';//show robot position
		ROS_INFO("map is init but not complete");
		//map[(map_width/2-robot_pose.y)*map_width+robot_pose.x+map_width/2]='x';		
		ROS_INFO("%s",map_print.c_str());//print map
	}
	else if(map_init==false){ //get map info
		ROS_INFO("119");
		this->map_init=true;
		map_resolution=msg->info.resolution;
		map_width=msg->info.width; //get the map Grid width
		map=new char [map_width*map_width];
//		for(int i=0;i<map_width;i++)
//			map[i]=new char [map_width];
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
	direct=new Point[8];
	direct[0]=Point(1,0);
	direct[1]=Point(-1,0);
	direct[2]=Point(0,1);
	direct[3]=Point(0,-1);
	direct[4]=Point(1,1);
	direct[5]=Point(-1,1);
	direct[6]=Point(-1,-1);
	direct[7]=Point(1,-1);
}

void Self_Clean::search_pathtounknown(){
	int unknownnum=0;
	int dx,dy;
	int dirx=0,diry=0;
	Point *save;
	while(!(unknown[unknownnum].x==0 && unknown[unknownnum].y==0) && ++unknownnum);
	ROS_INFO("unknown num:%d",unknownnum);

	if(start==NULL){
		important_node->point=new Point[unknownnum+1];
		important_node->next=new List_node[1];
		important_node->point[0]=Point(robot_pose.x,robot_pose.y); //set_robot pose to path
		start=important_node;
		
	}
	
	if(robot_status==Search_Unknown){
		

	}else{
		List_node *t=important_node;
		for(int i=0;i<unknownnum;i++){			
			important_node->point[i+1]=Point(unknown[i].x,unknown[i].y);
			t=new List_node[1];
			t->previous=important_node;
			t=t->next;
		}
	}
	//important_node->point=new Point[unknownnum+1];
	//important_node->next=new List_node[1];
	
	

	important_node=important_node->next;
	//important_node
}


int main(int argc,char *argv[]){
	ros::init(argc,argv,"autoclean_node");
	
	Self_Clean b=Self_Clean();
	//b.close();
	while(ros::ok()){
		ros::spinOnce();//while rosspin call message we subscriber
		//switch: //0. make_map 1. robot_run 2.robot_stop
		//ros::spin();
		switch(b.robot_status){
			case 0://get_map() Normal_Mode
				break;
			case 1:
				
				break;
			case 2://map not complete
				b.search_pathtounknown();
				break;
			case 3: 
			case 4://load map from outside
			case 5://Sent Unknown
				//b.send();
			default:
				break;
		}
	}

	return 0;
}
