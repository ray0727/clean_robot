#include"self_clean.h"

Self_Clean::Self_Clean(){
	this->clean_map=NULL;
}

Self_Clean::~Self_Clean(){

}

void Self_Clean::init(int map_width,char *map,double resolution){
	this->map_width=map_width;
	this->map=map;
	this->map_resolution=resolution;
}

Point Self_Clean::start_clean(Pointf region[2]){	
	int x,y;
	int locate;
	Point start=Point();
	Point end=Point();
	bool is_break=false;
	bool change=false;
	for(x=1;x<map_width-1;x++){
		is_break=false;
		if(x>=map_width-1)
			x=map_width-2;
		if(!change)
			for(y=1;y<map_width-1;y++){
				if(!((double)x<=region[0].x && (double)x>=region[1].x && (double)y<=region[0].y && (double)y>=region[1].y ))
					continue;
				locate=(y)*map_width+x-1;		
				if(map[locate]==' ' && map[locate+map_width]==' ' && map[locate-map_width]==' ' && map[locate+1]==' ' && map[locate-1]==' '){
					if(!is_break){
						start.x=x-map_width/2-1;
						start.y=y-map_width/2-1;
						is_break=true;
						change=true;
					}else{
				 		end.x=x-map_width/2-1;
						end.y=y-map_width/2-1;
					}
				}
			}
		else	
			for(y=map_width-2;y>=1;y--){
				if(!((double)x<=region[0].x && (double)x>=region[1].x && (double)y<=region[0].y && (double)y>=region[1].y ))
					continue;
				locate=(y)*map_width+x;			
				if(map[locate]==' ' && map[locate+map_width]==' ' && map[locate-map_width]==' ' && map[locate+1]==' ' && map[locate-1]==' '){
					if(!is_break){
						start.x=x-map_width/2;
						start.y=y-map_width/2;
						is_break=true;
						change=false;
					}else{
				 		end.x=x-map_width/2;
						end.y=y-map_width/2;
					}
				}
			}
		if(is_break){
			x++;
			ROS_INFO("%d,%d",start.x,start.y);
			ROS_INFO("%d,%d",end.x,end.y);
			if(clean_map==NULL){
				clean_map_p=new List_node[1];
				clean_map_p->point=new Point[1];
				clean_map_p->point->x=start.x;
				clean_map_p->point->y=start.y;
				clean_map=clean_map_p;
			}else{
				clean_map_p->next=new List_node[1];
				clean_map_p=clean_map_p->next;
				clean_map_p->point=new Point[1];
				clean_map_p->point->x=end.x;
				clean_map_p->point->y=end.y;
			}
			clean_map_p->next=new List_node[1];
			clean_map_p=clean_map_p->next;
			clean_map_p->point=new Point[1];
			clean_map_p->point->x=end.x;
			clean_map_p->point->y=end.y;
			
		}
	}
	ROS_INFO("end start clean");
	clean_map_p=clean_map;
	//ROS_INFO("start at");
	//return start;
}

bool Self_Clean::is_arrive(Point robot_pose){
	//Point *p=clean_map_p->previous->point;
	//ROS_INFO("is_arrive");
	return /*(robot_pose.x==aim.x && robot_pose.y==aim.y);*/(robot_pose.x>=(aim.x-1) && robot_pose.x<=(aim.x+1) && robot_pose.y>=(aim.y-1) && robot_pose.y<=(aim.y+1));
}

bool Self_Clean::send_point(ros::Publisher point_pub){
	ROS_INFO("IN send point");
	List_node *t=clean_map_p;
	
	if(this->clean_map_p!=NULL){
		geometry_msgs::PoseStamped goal;
		int x=clean_map_p->point->x;
		int y=clean_map_p->point->y;
		goal.header.frame_id="map";
		goal.header.stamp=ros::Time::now();
		goal.pose.position.x=(double)x*this->map_resolution;
		goal.pose.position.y=(double)y*this->map_resolution;
		goal.pose.orientation.w = 1.0;
		ROS_INFO("go to %d,%d",x,y);
		point_pub.publish(goal);
		ROS_INFO("1");
		clean_map_p=clean_map_p->next;
		ROS_INFO("2");
		aim.x=x;
		aim.y=y;
		ROS_INFO("go to %d,%d",x,y);
		return true;
	}else{
		List_node *t;
		while(clean_map!=NULL){
			t=clean_map;
			delete [] t->point;
			delete [] t;
			clean_map=clean_map->next;
		}
		clean_map=NULL;
		ROS_INFO("end clean");	
		return false;
	}
}
