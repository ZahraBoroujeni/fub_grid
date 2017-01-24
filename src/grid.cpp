#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>
#include <array>

#define OCCUPIED costmap_2d::LETHAL_OBSTACLE
#define FREE costmap_2d::FREE_SPACE


using namespace std;

shared_ptr<ros::NodeHandle> nh;
shared_ptr<costmap_2d::Costmap2D> grid;
shared_ptr<costmap_2d::Costmap2DPublisher> costMapPub;
visualization_msgs::Marker points;


void createCostMap(std::array<double, 2> pos_bottom_left, std::array<double, 2> pos_top_right, double resolution) {
	unsigned int width = (unsigned int) ((pos_top_right[0] - pos_bottom_left[0]) / resolution);
	unsigned int height = (unsigned int) ((pos_top_right[1] - pos_bottom_left[1]) / resolution);

	grid = make_shared<costmap_2d::Costmap2D>(width, height, resolution, pos_bottom_left[0], pos_bottom_left[1]);
	
	for (unsigned int i = 0; i < width; ++i) {
		for (unsigned int j = 0; j < height; ++j) {
			grid->setCost(i, j, costmap_2d::NO_INFORMATION);
		}
	}

}

void publishGrid() {

	costMapPub = make_shared<costmap_2d::Costmap2DPublisher>(nh.get(), grid.get(), "map", "map/grid", true);
	costMapPub->publishCostmap();
}

void createMarkers() {
	points.header.frame_id = "map";
	points.header.stamp = ros::Time::now();
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.g = 1.0f;
	points.color.a = 1.0;
	
	geometry_msgs::Point start;
	start.x = 5 * 0.3 + 0.15;
	start.y = 2 * 0.3 + 0.15;
	start.z = 0;
	
	geometry_msgs::Point goal;
	goal.x = 5  * 0.3 + 0.15;
	goal.y = 12 * 0.3 + 0.15;
	goal.z = 0;
	
	points.points.push_back(start);
	points.points.push_back(goal);
}


/** Subscribes to fub_atlas_msgs/Atlas messages and publishes a layered CostMap2D
 **
 ** @param argc
 ** @param argv
 ** @return
 **
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "fub_grid");
	ros::NodeHandle n;
	nh = shared_ptr<ros::NodeHandle>(&n);
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	createCostMap({0,0}, {4.50,4.50}, 0.3);
	
	// example path
	grid->setCost(5, 2, FREE);
	grid->setCost(5, 3, FREE);
	grid->setCost(5, 4, FREE);
	grid->setCost(4, 4, FREE);
	grid->setCost(5, 5, OCCUPIED);
	grid->setCost(4, 5, FREE);
	grid->setCost(4, 6, FREE);
	grid->setCost(5, 6, FREE);
	grid->setCost(5, 7, FREE);
	grid->setCost(5, 8, FREE);
	grid->setCost(5, 9, FREE);
	grid->setCost(5, 10, FREE);
	grid->setCost(5, 11, FREE);
	grid->setCost(5, 12, FREE);
	
	publishGrid();
	
	createMarkers();

	ros::Rate r(1);
	while(ros::ok()) {
		marker_pub.publish(points);
		ros::spinOnce();
	}


	ros::spin();

}

