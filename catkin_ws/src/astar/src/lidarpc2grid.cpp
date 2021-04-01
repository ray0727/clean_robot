#include "depthpc2grid.h"

Pc2Grid::Pc2Grid()
{
	NodeHandle nh;
	depth_cam.reset(new PointCloud<PointXYZ>());
	pc_map = nh.advertise<sensor_msgs::PointCloud2>("transform_pc", 1);
	pub_map = nh.advertise<nav_msgs::OccupancyGrid>("pc_map", 1);

	float leaf_size = 0.1;
	voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
	passZ.setFilterFieldName("z");

	// map info parameter
	ros::param::param<std::string>("~map_frame", map_frame, "base_footprint");
	ros::param::param<double>("~map_res", map_res, 0.1);
	ros::param::param<double>("~map_x", map_x, 0);
	ros::param::param<double>("~map_y", map_y, -7);
	ros::param::param<double>("~map_z", map_z, 0);
	ros::param::param<int>("~map_height", map_height, 140);
	ros::param::param<int>("~map_width", map_width, 70);
	ros::param::param<bool>("~save_margin", save_margin, true);

	sub_map = nh.subscribe("points", 1, &Pc2Grid::pcCallback, this);
}

void Pc2Grid::pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc)
{
	fromROSMsg(*pc, *depth_cam);

	// passZ.setInputCloud(depth_cam);
	// passZ.setFilterLimits(0, 2);
	// passZ.filter(*depth_cam);

	voxel.setInputCloud(depth_cam);
	voxel.filter(*depth_cam);

	// std::cout << "die here" << endl;

	try
	{
		ros::Duration timeout(1.0);
		listener.waitForTransform("base_link", pc->header.frame_id, ros::Time(0), timeout);
		listener.lookupTransform("base_link", pc->header.frame_id, ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}

	// std::cout << "die here 1" << endl;

	pcl_ros::transformAsMatrix(transform, trans);
	pcl::transformPointCloud(*depth_cam, *depth_cam, trans);

	// std::cout << "die here 2" << endl;

	passZ.setInputCloud(depth_cam);
	passZ.setFilterLimits(0.2, 2);
	passZ.filter(*depth_cam);

	toROSMsg(*depth_cam, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "base_link";
	ros_cloud_msg.header.stamp = pc->header.stamp;
	pc_map.publish(ros_cloud_msg);

	// std::cout << "die here 3" << endl;

	// to grid ---------------------------------------------------------------------
	nav_msgs::OccupancyGrid map;
	map.header.frame_id = map_frame;
	map.header.stamp = pc->header.stamp;
	map.info.map_load_time = pc->header.stamp;
	map.info.resolution = map_res;
	map.info.height = map_height;
	map.info.width = map_width;
	map.data = std::vector<int8_t>(map.info.height * map.info.width, 0);
	for (int i = 0; i < depth_cam->size()-1; i++)
	{
		int c = int((depth_cam->points[i].x - map_x) / map.info.resolution);
		int r = int((depth_cam->points[i].y - map_y) / map.info.resolution);
		// std::cout << "r:" << r << ", c:" << c << endl;
		// std::cout << "dp:" << depth_cam->points[i].x << ", op:" << map.info.origin.position.x << endl;
		// std::cout << (depth_cam->points[i].x - map.info.origin.position.x) / map.info.resolution << endl;
		if(r >= map.info.height || r < 0)
			continue;
		if(c >= map.info.width || c < 0)
			continue;

		if (map.data[r * map.info.width + c] <= 90)
		{
			// std::cout << "add p" << endl;
			map.data[r * map.info.width + c] += 10;

			if(save_margin){
			
			  // up
			  // std::cout << "r-1:" << r-1 << endl;
			  if (r-1>=0){
				if (map.data[(r-1) * map.info.width + c] < 100){
					map.data[(r-1) * map.info.width + c] += 5;
			   	  }
			  }
			
			  // down
			  // std::cout << "r+1:" << r+1 << endl;
			  if (r+1<map.info.height){
				  if (map.data[(r+1) * map.info.width + c] < 100){
					  map.data[(r+1) * map.info.width + c] += 5;
				  }
			  }
				
			  // left
			  // std::cout << "c-1:" << c-1 << endl;
			  if (c-1>=0){
				  if (map.data[r * map.info.width + c-1] < 100){
					  map.data[r * map.info.width + c-1] += 5;
				  }
			  }
			
			  // right
			  // std::cout << "c+1:" << c+1 << endl;
			  if (c+1<map.info.width){
				  if (map.data[r * map.info.width + c+1] < 100){
					  map.data[r * map.info.width + c+1] += 5;
				  }
			  }
			
		          }
		}
		// map.data[map.info.height/4 * map.info.width + map.info.width/4] = 100;
		// map.data[map.info.height/2 * map.info.width + map.info.width/2] = 100;
	}
	int c = int((0 - map_x) / map.info.resolution);
    int r = int((0 - map_y) / map.info.resolution);
	map.data[r * map.info.width + c+1] = 0;

	// tranform map from base_link to map_frame (can be base_link as well or odom or map)
	geometry_msgs::PointStamped point;
	point.point.x = map_x;
	point.point.y = map_y;
	point.point.z = map_z;
	point.header.frame_id = "base_link";
	point.header.stamp = pc->header.stamp;

	try
	{
		ros::Duration timeout(1.0);
		listener.waitForTransform(map_frame, "base_link", ros::Time(0), timeout);
		listener.lookupTransform(map_frame, "base_link", ros::Time(0), transform);
		listener.transformPoint(map_frame, ros::Time(0), point, "base_link", point);

	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}

	map.info.origin.position = point.point;
	tf::quaternionTFToMsg(transform.getRotation(), map.info.origin.orientation);

	pub_map.publish(map);

	// std::cout << "die here 4" << endl;

	// reset ---------------------------------------------------------------------
	depth_cam->points.clear();
}

int main(int argc, char **argv)
{
	init(argc, argv, "pc2grid");
	Pc2Grid global_map;
	spin();
	return 0;
}
