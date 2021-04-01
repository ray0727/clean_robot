#include "depthpc2grid.h"

Pc2Grid::Pc2Grid()
{
	NodeHandle nh;
	depth_cam.reset(new PointCloud<PointXYZ>());
	pc_map = nh.advertise<sensor_msgs::PointCloud2>("transform_d435_pc", 1);
	pub_map = nh.advertise<nav_msgs::OccupancyGrid>("depthcam_map", 1);

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
	ros::param::param<bool>("~use_hist", use_hist, false);
	ros::param::param<double>("~hist_dist", hist_dist, 3);
	ros::param::param<std::string>("~anchor_frame", anchor_frame, "odom");

	if(use_hist){
		hist_cloud.reset(new PointCloud<PointXYZ>());
		passHistX.setFilterFieldName("x");
		passHistY.setFilterFieldName("y");
	}

	sub_map = nh.subscribe("/camera/depth_registered/points", 1, &Pc2Grid::pcCallback, this);
}

void Pc2Grid::pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc)
{
	fromROSMsg(*pc, *depth_cam);

	passZ.setInputCloud(depth_cam);
	passZ.setFilterLimits(0, 7);
	passZ.filter(*depth_cam);

	voxel.setInputCloud(depth_cam);
	voxel.filter(*depth_cam);

	try
	{
		ros::Duration timeout(1.0);
		listener.waitForTransform(map_frame, pc->header.frame_id, pc->header.stamp, timeout);
		listener.lookupTransform(map_frame, pc->header.frame_id, pc->header.stamp, transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	pcl_ros::transformAsMatrix(transform, trans);
	pcl::transformPointCloud(*depth_cam, *depth_cam, trans);

	passZ.setInputCloud(depth_cam);
	passZ.setFilterLimits(0.2, 2);
	passZ.filter(*depth_cam);

	toROSMsg(*depth_cam, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = map_frame;
	ros_cloud_msg.header.stamp = pc->header.stamp;
	pc_map.publish(ros_cloud_msg);

	// to grid ---------------------------------------------------------------------
	nav_msgs::OccupancyGrid map;
	map.header.frame_id = map_frame;
	map.header.stamp = pc->header.stamp;
	map.info.map_load_time = pc->header.stamp;
	map.info.resolution = map_res;
	map.info.origin.position.x = map_x;
	map.info.origin.position.y = map_y;
	map.info.origin.position.z = map_z;
	// map.info.resolution = 0.5;
	// map.info.origin.position.x = -25;
	// map.info.origin.position.y = -25;
	// map.info.origin.position.z = 0;
	// map.info.origin.orientation.w = 1;
	// map.info.height = 100;
	// map.info.width = 100;
	map.info.height = map_height;
	map.info.width = map_width;
	map.data = std::vector<int8_t>(map.info.height * map.info.width, 0);
	for (int i = 0; i < depth_cam->size(); i++)
	{
		int c = int((depth_cam->points[i].x - map.info.origin.position.x) / map.info.resolution);
		int r = int((depth_cam->points[i].y - map.info.origin.position.y) / map.info.resolution);
		//r = (r > 139) ? 139 : r;
		//c = (c > 69) ? 69 : c;
		if(r >= map.info.height || r < 0)
                        continue;
                if(c >= map.info.width || c < 0)
                        continue;

		if (map.data[r * map.info.width + c] <= 90)
		{
			map.data[r * map.info.width + c] += 20;
			
			// up
			// std::cout << "r-1:" << r-1 << endl;
			if (r-1>=0){
					if (map.data[(r-1) * map.info.width + c] < 100){
							map.data[(r-1) * map.info.width + c] += 20;
					}
			}

			// down
			// std::cout << "r+1:" << r+1 << endl;
			if (r+1<map.info.height){
					if (map.data[(r+1) * map.info.width + c] < 100){
							map.data[(r+1) * map.info.width + c] += 20;
					}
			}

			// left
			// std::cout << "c-1:" << c-1 << endl;
			if (c-1>=0){
					if (map.data[r * map.info.width + c-1] < 100){
							map.data[r * map.info.width + c-1] += 20;
					}
			}

			// right
			// std::cout << "c+1:" << c+1 << endl;
			if (c+1<map.info.width){
					if (map.data[r * map.info.width + c+1] < 100){
							map.data[r * map.info.width + c+1] += 20;
					}
			}

		}
	}

	while(use_hist){

		// transform history pointcloud to current frame
		try
		{
			ros::Duration timeout(1.0);
			listener.waitForTransform(map_frame, anchor_frame, pc->header.stamp, timeout);
			listener.lookupTransform(map_frame, anchor_frame, pc->header.stamp, transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			break;
		}
		pcl_ros::transformAsMatrix(transform, trans);
		pcl::transformPointCloud(*hist_cloud, *hist_cloud, trans);

		// filter out pc too far
		passHistX.setInputCloud(hist_cloud);
		passHistX.setFilterLimits(-1*hist_dist, hist_dist);
		passHistX.filter(*hist_cloud);
		passHistY.setInputCloud(hist_cloud);
		passHistY.setFilterLimits(-1*hist_dist, hist_dist);
		passHistY.filter(*hist_cloud);

		for (int i = 0; i < hist_cloud->size(); i++)
		{
			int c = int((hist_cloud->points[i].x - map.info.origin.position.x) / map.info.resolution);
			int r = int((hist_cloud->points[i].y - map.info.origin.position.y) / map.info.resolution);
			//r = (r > 139) ? 139 : r;
			//c = (c > 69) ? 69 : c;
			if(r >= map.info.height || r < 0)
							continue;
					if(c >= map.info.width || c < 0)
							continue;

			if (map.data[r * map.info.width + c] < 100)
			{
				map.data[r * map.info.width + c] += 20;
				
				// up
				// std::cout << "r-1:" << r-1 << endl;
				if (r-1>=0){
						if (map.data[(r-1) * map.info.width + c] < 100){
								map.data[(r-1) * map.info.width + c] += 20;
						}
				}

				// down
				// std::cout << "r+1:" << r+1 << endl;
				if (r+1<map.info.height){
						if (map.data[(r+1) * map.info.width + c] < 100){
								map.data[(r+1) * map.info.width + c] += 20;
						}
				}

				// left
				// std::cout << "c-1:" << c-1 << endl;
				if (c-1>=0){
						if (map.data[r * map.info.width + c-1] < 100){
								map.data[r * map.info.width + c-1] += 20;
						}
				}

				// right
				// std::cout << "c+1:" << c+1 << endl;
				if (c+1<map.info.width){
						if (map.data[r * map.info.width + c+1] < 100){
								map.data[r * map.info.width + c+1] += 20;
						}
				}

			}
		}

		// add depth cam to hist pc
		*hist_cloud = *depth_cam + *hist_cloud;
		voxel.setInputCloud(hist_cloud);
		voxel.filter(*hist_cloud);
		// add to anchor frame for next callback
		// transform current frame to anchor frame
		try
		{
			ros::Duration timeout(1.0);
			listener.waitForTransform(anchor_frame, map_frame, pc->header.stamp, timeout);
			listener.lookupTransform(anchor_frame, map_frame, pc->header.stamp, transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ROS_ERROR("Lost history pointcloud");
			hist_cloud->points.clear();
			break;
		}
		pcl_ros::transformAsMatrix(transform, trans);
		pcl::transformPointCloud(*hist_cloud, *hist_cloud, trans);

		break; // we use while here. Remeber to add break!!!
	}


	pub_map.publish(map);

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
