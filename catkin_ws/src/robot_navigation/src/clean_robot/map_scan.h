#include "clean_robot.h"


class Map_Scan{
	public:
		Point robot_pose;//robot's now position 
		Point old_pose;//the old position of robot 
		Point *direct;
		Point *unknown;//save unknown point

		
		//---slam info---
		double map_resolution; //map resolution m/Grid
		char *map;		//get map data
		int map_width;	
		bool mapinit;//determind code init
		bool mapcomplete;//check map complete
		Point origin;//use in several map

	protected:
		//ros::Subscriber map_sub,pose_sub,slam_pose_sub;
		ros::Subscriber slavestatus_sub;
		ros::Publisher  master_pub;
		ros::NodeHandle node_;

	public:
		Map_Scan();
		~Map_Scan();
		void map_init(double resolution,int width,int *data);
		void print_map(Point robot_pose);
		void direct_delete(Point* direct);
		void slavecallback(const geometry_msgs::PoseStamped::ConstPtr& status);
		Point* direct_init();
		bool checkmap(int *msg_data,Point robot_pose);
};




