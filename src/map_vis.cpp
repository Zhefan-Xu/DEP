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
#include <nav_msgs/Path.h>
#include <DEP/Goal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <chrono> 
#include <iostream>
using namespace std::chrono;
using namespace message_filters;
ros::Publisher map_vis_pub;
ros::Publisher goal_pub;
ros::Publisher path_vis_pub;
ros::Publisher plan_vis_pub;

AbstractOcTree* abtree;
OcTree* tree_ptr;
PRM* roadmap;
bool new_plan = true;
// int num_sample = 100;
int unknown_thresh = 50;
int count_no_increase = 0;
int roadmap_size_last = 0;
double linear_velocity = 0.2;
double delta = 0.1; // criteria for chechking reach
visualization_msgs::MarkerArray map_markers;
std::vector<visualization_msgs::Marker> map_vis_array;
visualization_msgs::MarkerArray plan_markers;
std::vector<visualization_msgs::Marker> plan_vis_array;
visualization_msgs::Marker path_marker;
std::vector<Node*> path;
int path_idx = 0;
DEP::Goal next_goal;
int count_iteration = 0;
double total_comp_time = 0;
double total_path_length = 0;
double total_path_segment = 0;
double previous_goal_voxels = -100;


DEP::Goal getNextGoal(std::vector<Node*> path, int path_idx, const nav_msgs::OdometryConstPtr& odom);
bool isReach(const nav_msgs::OdometryConstPtr& odom, DEP::Goal next_goal);
Node* findStartNode(PRM* map, Node* current_node, OcTree& tree);
double calculatePathLength(std::vector<Node*> path);
Node Goal2Node(DEP::Goal g);
std::vector<Node*>& improvePath(std::vector<Node*> &path, int path_idx, OcTree & tree, double& least_distance);
bool evaluateGoal(Node* goal, double& previous_goal_voxels);

bool reach = false;
bool first_time = true;
bool half_stop = false;
double x, y, z;
Node current_pose;
Node last_goal_node (point3d (-100, -100, -100));
Node current_goal_node;
auto start_time_total = high_resolution_clock::now();
double least_distance = 10000;
void callback(const nav_msgs::OdometryConstPtr& odom, const octomap_msgs::Octomap::ConstPtr& bmap){
	abtree = octomap_msgs::binaryMsgToMap(*bmap);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	tree_ptr->setResolution(RES);
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	z = odom->pose.pose.position.z;
	current_pose.p.x() = x;
	current_pose.p.y() = y;
	current_pose.p.z() = z;
	plan_vis_array.clear();

	// check if we already reach the next goal
	reach = isReach(odom, next_goal);
	if (reach){
		if (path_idx >= path.size()){
			new_plan = true;
		}
		else{
			bool good_goal = evaluateGoal(*(path.end()-1), previous_goal_voxels);
			if (not good_goal){
				new_plan = true;
				half_stop = true;
				next_goal.is_last = true;
				cout << "Previous Plan is Abandoned!" << endl;
			}
			else{
				cout << "Re-evalute the Best Angle...";
				std::map<double, int> yaw_num_voxels = calculateUnknown(*tree_ptr, path[path_idx], 5);
				double best_yaw;
				double best_num_voxels = 0;
				for (double yaw: yaws){
					double num_voxels = yaw_num_voxels[yaw];
					if (num_voxels > best_num_voxels){
						best_num_voxels = num_voxels;
						best_yaw = yaw;
					}
				}
				path[path_idx]->yaw = best_yaw;
				path[path_idx]->num_voxels = best_num_voxels;
				cout << "DONE!" << "new angle: " << best_yaw*180/pi << endl;
				next_goal = getNextGoal(path, path_idx, odom);
				last_goal_node = current_goal_node;
				++path_idx;
			}
			least_distance = 10000;
		}
	}

	// Make new plan
	if (new_plan){
		// WHen we have new plan, we set it to default value
		previous_goal_voxels = -100;
		
		Node* start;
		if (first_time){
			roadmap = new PRM ();	
		}
		cout << "==============================" << count_iteration << "==============================" << endl;
		auto start_time = high_resolution_clock::now();
		if (first_time){
			roadmap = buildRoadMap(*tree_ptr, roadmap, path,  NULL, map_vis_array);
			start = findStartNode(roadmap, &current_pose, *tree_ptr);
			first_time = false;
		}
		else{
			if (half_stop){
				start = findStartNode(roadmap, &current_pose, *tree_ptr);
				half_stop = false;
			}
			else{
				start = *(path.end()-1);	
			}
			roadmap = buildRoadMap(*tree_ptr, roadmap, path,  start, map_vis_array);
			if (roadmap->getSize() - roadmap_size_last < 5){
				++count_no_increase;
			}
			else{
				count_no_increase = 0;
			}
			if (count_no_increase >= 2){
				auto stop_time_total = high_resolution_clock::now();
				auto duration_total = duration_cast<microseconds>(stop_time_total - start_time_total);
				cout << "Total: "<< duration_total.count()/1e6 << " seconds | " << "Exploration Terminates!!!!!!!!!!!" << endl;
				cout << "Total Path Segment: " << total_path_segment << endl;
				cout << "Total Path Length: " << total_path_length << endl;
				cout << "Total Computation Time: " << total_comp_time << endl;
				cout << "Avg Computation Time: " << total_comp_time/total_path_segment << endl;
				ros::shutdown();
				return;
			}
			roadmap_size_last = roadmap->getSize();
			// if (roadmap->getMaxUnknown() < unknown_thresh){
			// 	cout << "Exploration Terminates!!!!!!!!!!!" << endl;
			// ros::shutdown();
		}

		
		
		// cout << "map size: " << map->getSize() <<endl;
		// cout << "map vis array size: " << map_vis_array.size() << endl;
		// for (Node* n : map->getGoalNodes()){
		// 	print_node(*n);
		// }
		
		// Test mulitgoal a star

		// Node* start = map->nearestNeighbor(&current_pose);
		

		path = multiGoalAStar(roadmap, start, *tree_ptr);
		total_path_length += calculatePathLength(path);
		total_path_segment += path.size();
		// if (path[path.size()-1]->num_voxels < unknown_thresh){
		// 	cout << "Exploration Terminates!!!!!!!!!!!" << endl;
		// 	ros::shutdown();
		// }
		auto stop_time = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop_time - start_time);
		cout << "Time taken by this Iteration: "
         << duration.count()/1e6 << " seconds" << endl;
         total_comp_time += duration.count()/1e6;
         ++count_iteration;
         cout << "Total Computation Time: " << total_comp_time << endl;
         cout << "Path Length So Far: " << total_path_length << endl;
         cout << "==============================" << "END" << "==============================" << endl;
		// print_path(path);

		path_idx = 0;
		if (path.size() != 0){
			next_goal = getNextGoal(path, path_idx, odom);
			reach = false;
			++path_idx;
		}
		// delete roadmap;
		// ==========================VISUALIZATION=============================
		map_markers.markers = map_vis_array;
		if (path.size() != 0){
			// print_node_vector(path);
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
			// map_vis_array.push_back(path_marker);
			map_markers.markers = map_vis_array;
			new_plan = false;
		}
		// ====================================================================
		
	}
	else{

		path = improvePath(path, path_idx, *tree_ptr, least_distance);
		if (path.size() != 0){
			// ========================Path Visualization=======================
			// print_node_vector(path);
			
			std::vector<geometry_msgs::Point> path_vis_vector;
			for (int i=0; i<path.size(); ++i){
				if (i<path.size()-1){
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
				// Way Point:
				visualization_msgs::Marker way_point;
				way_point.header.frame_id = "world";
				way_point.id = 6666+i;
				way_point.type = visualization_msgs::Marker::SPHERE;
				way_point.pose.position.x = path[i]->p.x();
				way_point.pose.position.y = path[i]->p.y();
				way_point.pose.position.z = path[i]->p.z();
				way_point.scale.x = 0.1;
				way_point.scale.y = 0.1;
				way_point.scale.z = 0.1;
				way_point.color.a = 1.0;
				way_point.color.r = 1.0;
				way_point.color.g = 0.0;
				way_point.color.b = 0.0;

				// Direction
				visualization_msgs::Marker direction_ptr;
				direction_ptr.header.frame_id = "world";
				direction_ptr.id = 8888+i;
				direction_ptr.type = visualization_msgs::Marker::ARROW;
				direction_ptr.pose.position.x = path[i]->p.x();
				direction_ptr.pose.position.y = path[i]->p.y();
				direction_ptr.pose.position.z = path[i]->p.z();
				tf2::Quaternion quat;
				quat.setRPY(0, 0, path[i]->yaw);
				direction_ptr.pose.orientation.x = quat[0];
				direction_ptr.pose.orientation.y = quat[1];
				direction_ptr.pose.orientation.z = quat[2];
				direction_ptr.pose.orientation.w = quat[3];;
				direction_ptr.scale.x = 0.3;
				direction_ptr.scale.y = 0.03;
				direction_ptr.scale.z = 0.03;
				direction_ptr.color.a = 1;
				direction_ptr.color.r = 0;
				direction_ptr.color.g = 0;
				direction_ptr.color.b = 1;

				plan_vis_array.push_back(way_point);
				plan_vis_array.push_back(direction_ptr);
			}
			path_marker.header.frame_id = "world";
			path_marker.points = path_vis_vector;
			path_marker.id = 1000000;
			path_marker.type = visualization_msgs::Marker::LINE_LIST;
			path_marker.scale.x = 0.05;
			path_marker.scale.y = 0.05;
			path_marker.scale.z = 0.05;
			path_marker.color.a = 1.0;
			plan_vis_array.push_back(path_marker);
			plan_markers.markers = plan_vis_array;
			// ====================================================================

		}
		
	}
	delete abtree;
	
	
	
}
 

int main(int argc, char** argv){
	ros::init(argc, argv, "map_visualizer");
	ros::NodeHandle nh;
	// ros::Subscriber sub = n.subscribe("octomap_binary", 100, callback);
	map_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("map_vis_array", 0);
	goal_pub = nh.advertise<DEP::Goal>("goal", 0);
	path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis_array", 0);
	plan_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("plan_vis_array", 0);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 100);
	message_filters::Subscriber<octomap_msgs::Octomap> map_sub(nh, "octomap_binary", 100);
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, octomap_msgs::Octomap> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync (MySyncPolicy(100), odom_sub, map_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::Rate loop_rate(10);
	
	while (ros::ok()){
		map_vis_pub.publish(map_markers);	
		goal_pub.publish(next_goal);
		plan_vis_pub.publish(plan_markers);
		// path_vis_pub.publish(path_marker);
		// cout << tree_ptr << endl;
		// improvePath(path, path_idx, *tree_ptr);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}



//======================================================================================
DEP::Goal getNextGoal(std::vector<Node*> path, int path_idx, const nav_msgs::OdometryConstPtr& odom){
	DEP::Goal goal;
	goal.x = path[path_idx]->p.x();
	goal.y = path[path_idx]->p.y();
	goal.z = path[path_idx]->p.z();
	goal.yaw = path[path_idx]->yaw;
	goal.linear_velocity = linear_velocity;
	if (path_idx != 0){
		double distance_to_goal = path[path_idx]->p.distance(path[path_idx-1]->p);
		double dyaw = path[path_idx]->yaw - path[path_idx-1]->yaw;
		goal.angular_velocity = (double) dyaw/(distance_to_goal/linear_velocity);
	}
	else{
		double current_x = odom->pose.pose.position.x;
		double current_y = odom->pose.pose.position.y;
		double current_z = odom->pose.pose.position.z;
		geometry_msgs::Quaternion quat = odom->pose.pose.orientation;
		tf2::Quaternion tf_quat;
		tf2::convert(quat, tf_quat);
		double current_roll, current_pitch, current_yaw;
		tf2::Matrix3x3(tf_quat).getRPY(current_roll, current_pitch, current_yaw);
		double distance_to_goal = sqrt(pow(goal.x-current_x, 2) + pow(goal.y-current_y, 2) + pow(goal.z-current_z, 2));
		double dyaw = path[path_idx]->yaw - current_yaw;
		goal.angular_velocity = (double) dyaw/(distance_to_goal/linear_velocity);
	}
	// for better simulation control: NOT SURE 
	if (path_idx == path.size()-1){
		goal.is_last = true;
	}
	else{
		goal.is_last = false;
	}

	return goal; 
}

bool isReach(const nav_msgs::OdometryConstPtr& odom, DEP::Goal next_goal){
	double current_x = odom->pose.pose.position.x;
	double current_y = odom->pose.pose.position.y;
	double current_z = odom->pose.pose.position.z;
	return std::abs(current_x-next_goal.x) < delta and std::abs(current_y-next_goal.y) < delta and std::abs(current_z-next_goal.z) < delta;
}

Node* findStartNode(PRM* map, Node* current_node, OcTree& tree){
	std::vector<Node*> knn = map->kNearestNeighbor(current_node, 5);
	for (Node* n: knn){
		bool has_collision = checkCollision(tree, n, current_node);
		if (not has_collision){
			return n;
		}
	}
	cout << "No Valid Node for Start" << endl;
	return NULL;
}

double calculatePathLength(std::vector<Node*> path){
	int idx1 = 0;
	double length = 0;
	for (int idx2=1; idx2<path.size()-1; ++idx2){
		length += path[idx2]->p.distance(path[idx1]->p);
		++idx1;
	}
	return length;
}

Node Goal2Node(DEP::Goal g){
	Node n (point3d(g.x, g.y, g.z));
	return n;
}

std::vector<Node*>& improvePath(std::vector<Node*> &path, int path_idx, OcTree& tree, double& least_distance){
	// Path: Current Path
	// path_idx: goal
	// We want to improve the path for the goal after next goal
	// Shortern the distance of goal with its next and next goal.
	// Only do this when path_idx is less than path.size() - 1 which is the third last
	int count = 0;
	if (path.size() == 0){
		return path;
	}
	if (path_idx >= path.size()-1){
		return path;
	}
	else{
		// cout << "New Request~~~" << endl;
		Node* target_node = path[path_idx];

		// Take Sample Around the target node
		double origin_distance = path[path_idx]->p.distance(path[path_idx-1]->p) + path[path_idx]->p.distance(path[path_idx+1]->p);
		least_distance = std::min(least_distance, origin_distance);
		while (reach == false and ros::ok() and count < 50){
			// cout << "Improving!" << endl
			double xmin, xmax, ymin, ymax, zmin, zmax;
			double range = 1;
			xmin = target_node->p.x() - range;
			xmax = target_node->p.x() + range;
			ymin = target_node->p.y() - range;
			ymax = target_node->p.y() + range;
			zmin = target_node->p.z() - range;
			zmax = target_node->p.z() + range;
			std::vector<double> bbx {xmin, xmax, ymin, ymax, zmin, zmax};
			// cout << tree_ptr==NULL << endl;
			Node* n = randomConfigBBX(tree, bbx);
			if (checkCollision(tree, n, path[path_idx-1]) and checkCollision(tree, n, path[path_idx+1])){
				double new_distance = n->p.distance(path[path_idx-1]->p) + n->p.distance(path[path_idx+1]->p);
				if (new_distance < least_distance){
					least_distance = new_distance;
					path[path_idx] = n;
					cout << "Better Path!" << endl;
				} 
			}
			++count;
		}
		return path;
	}
}

bool evaluateGoal(Node* goal, double& previous_goal_voxels){
		cout << "Re-evalute Goal...";
		if (previous_goal_voxels < 0){
			previous_goal_voxels = goal->num_voxels;	
		}
		
		std::map<double, int> yaw_num_voxels = calculateUnknown(*tree_ptr, goal, 2);
		double best_yaw;
		double best_num_voxels = 0;
		for (double yaw: yaws){
			double num_voxels = yaw_num_voxels[yaw];
			if (num_voxels > best_num_voxels){
				best_num_voxels = num_voxels;
				best_yaw = yaw;
			}
		}
		goal->yaw = best_yaw;
		goal->num_voxels = best_num_voxels;

		cout << "Current voxels: " << best_num_voxels << "| Previous Voxels: "<< previous_goal_voxels << endl;
		if (best_num_voxels/previous_goal_voxels <= 0.3){
			return false;
		}
		else{
			return true;
		}
}