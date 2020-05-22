#include <ros/ros.h>
#include <DEP/prm.h>
#include <DEP/multi_astar.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/sync_policies/approximate_time.h> 
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
using namespace message_filters;
ros::Publisher map_vis_pub;

bool first_time = true;
int num_sample = 100;
visualization_msgs::MarkerArray map_markers;
std::vector<visualization_msgs::Marker> map_vis_array;
visualization_msgs::Marker path_marker;


void callback(const nav_msgs::OdometryConstPtr& odom, const octomap_msgs::Octomap::ConstPtr& bmap){
	AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(*bmap);
	OcTree* tree_ptr = dynamic_cast<OcTree*>(abtree);
	if (first_time){
		PRM* map = buildRoadMap(*tree_ptr, num_sample, map_vis_array);
		cout << "map size: " << map->getSize() <<endl;
		cout << "map vis array size: " << map_vis_array.size() << endl;
		for (Node* n : map->getGoalNodes()){
			print_node(*n);
		}
		
		// Test mulitgoal a star
		double x = odom->pose.pose.position.x;
		double y = odom->pose.pose.position.y;
		double z = odom->pose.pose.position.z;
		Node current_pose (point3d(x, y, z));
		Node* start = map->nearestNeighbor(&current_pose);
		std::vector<Node*> path = multiGoalAStar(map, start);
		print_node_vector(path);
		std::vector<geometry_msgs::Point> path_vis_vector;
		for (int i=0; i<path.size()-1; ++i){
			geometry_msgs::Point p1, p2;
			p1.x = path[i]->p.x();
			p1.y = path[i]->p.y();
			p1.z = path[i]->p.z();
			p2.x = path[i+1]->p.x();
			p2.y = path[i+1]->p.y();
			p2.z = path[i+1]->p.z();
			path_vis_vector.push_back(p1);
			path_vis_vector.push_back(p2);
		}
		path_marker.header.frame_id = "world";
		path_marker.points = path_vis_vector;
		path_marker.id = 1000000;
		path_marker.type = visualization_msgs::Marker::LINE_LIST;
		path_marker.scale.x = 0.05;
		path_marker.scale.y = 0.05;
		path_marker.scale.z = 0.05;
		path_marker.color.a = 1.0;
		map_vis_array.push_back(path_marker);

		first_time = false;
		map_markers.markers = map_vis_array;

	}
	
}
 

int main(int argc, char** argv){
	ros::init(argc, argv, "map_visualizer");
	ros::NodeHandle nh;
	// ros::Subscriber sub = n.subscribe("octomap_binary", 100, callback);
	map_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("map_vis_array", 0);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 100);
	message_filters::Subscriber<octomap_msgs::Octomap> map_sub(nh, "octomap_binary", 100);
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, octomap_msgs::Octomap> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync (MySyncPolicy(100), odom_sub, map_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::Rate loop_rate(10);
	while (ros::ok()){
		map_vis_pub.publish(map_markers);		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}