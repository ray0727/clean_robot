#include "geometry_msgs/Quaternion.h"

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
