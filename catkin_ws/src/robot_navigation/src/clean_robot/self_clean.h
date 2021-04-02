#include "geometry_msgs/Quaternion.h"
//------define 1.Point 2.List_node
#include "clean_robot.h"


class robot_navigate{//need to transform to python form
	public:
		int right_speed_up;	
		geometry_msgs::Quaternion orientation;
		

	robot_navigate(){
		right_speed_up=0;
		
	}
	~robot_navigate(){
	}
} ;

class Self_Clean{
	
	public:
		Point robot_pose;//robot's now position 
		Point old_pose;//the old position of robot 
		Point aim;
		List_node *clean_map;
		List_node *clean_map_p;// clean_map point
		List_node *important_node;//save important_node
		List_node *start=NULL;
		List_node *search_node=NULL;
		char *map;		//get map data
		int map_width;	
		double map_resolution; //map resolution m/Grid
	protected:
		ros::Subscriber map_sub,pose_sub;
		ros::Publisher  point_pub;
		//ros::Publisher //send motor control info
	public:	
	Self_Clean();
	~Self_Clean();
	void init(int map_width,char *map,double resolution);
	bool is_arrive(Point robot_pose);
	bool send_point(ros::Publisher point_pub);
	Point start_clean(Pointf region[2]);
};
