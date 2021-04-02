#ifndef CLEAN_ROBOT_H
#define CLEAN_ROBOT_H
#include<stdio.h>
#include <nav_msgs/GetMap.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
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







#endif
