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
int num_goal_candicates = 20;
int unknown_thresh = 50;
int count_no_increase = 0;
int roadmap_size_last = 0;
double linear_velocity = 0.2;
double delta = 0.01; // criteria for chechking reach
double delta_angle = 0.1;
visualization_msgs::MarkerArray map_markers;
std::vector<visualization_msgs::Marker> map_vis_array;
visualization_msgs::MarkerArray plan_markers;


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
double calculatePathRotation(std::vector<Node*> path, double current_yaw);
Node Goal2Node(DEP::Goal g);
std::vector<Node*>& improvePath(std::vector<Node*> &path, int path_idx, OcTree & tree, double& least_distance);
std::vector<Node*> getGoalCandidates(PRM* roadmap);
std::vector<Node*> findBestPath(PRM* roadmap, Node* start, std::vector<Node*> goal_candidates, OcTree& tree);

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

	// check if we already reach the next goal
	reach = isReach(odom, next_goal);
	if (reach){
		// cout << "reach" << endl;
		if (path_idx >= path.size()){
			new_plan = true;
			// cout << "new plan" << endl;
		}
		else{
			next_goal = getNextGoal(path, path_idx, odom);
			++path_idx;
		}
	}

	// Make new plan
	if (new_plan){
		// WHen we have new plan, we set it to default value
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
			start = *(path.end()-1);	
			roadmap = buildRoadMap(*tree_ptr, roadmap, path,  start, map_vis_array);
			if (roadmap->getSize() - roadmap_size_last < 3){
				++count_no_increase;
			}
			else{
				count_no_increase = 0;
			}
			if (count_no_increase >= 3){
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
		
		auto start_time_search = high_resolution_clock::now();
		std::vector<Node*> goal_candidates = getGoalCandidates(roadmap);
		path = findBestPath(roadmap, start, goal_candidates, *tree_ptr);
		auto stop_time_search = high_resolution_clock::now();
		auto duration_search = duration_cast<microseconds>(stop_time_search - start_time_search);
		cout << "Time used for search is: " << duration_search.count()/1e6 << " Seconds" << endl;
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
		std::vector<visualization_msgs::Marker> plan_vis_array;
		std::vector<geometry_msgs::Point> path_vis_vector;
		plan_markers.markers = plan_vis_array;
		// path = improvePath(path, path_idx, *tree_ptr, least_distance);
		if (path.size() != 0){
			// ========================Path Visualization=======================
			// print_node_vector(path);


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
				way_point.id = 8888+i;
				way_point.type = visualization_msgs::Marker::SPHERE;
				way_point.pose.position.x = path[i]->p.x();
				way_point.pose.position.y = path[i]->p.y();
				way_point.pose.position.z = path[i]->p.z();
				way_point.lifetime = ros::Duration(0.5);
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
				direction_ptr.id = 6666+i;
				direction_ptr.type = visualization_msgs::Marker::ARROW;
				direction_ptr.pose.position.x = path[i]->p.x();
				direction_ptr.pose.position.y = path[i]->p.y();
				direction_ptr.pose.position.z = path[i]->p.z();
				tf2::Quaternion quat;
				quat.setRPY(0, 0, path[i]->yaw);
				direction_ptr.pose.orientation.x = quat[0];
				direction_ptr.pose.orientation.y = quat[1];
				direction_ptr.pose.orientation.z = quat[2];
				direction_ptr.pose.orientation.w = quat[3];
				direction_ptr.lifetime = ros::Duration(0.5);
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
		if (current_yaw < 0){
			current_yaw = 2 * pi - (-current_yaw);
		}
		double distance_to_goal = sqrt(pow(goal.x-current_x, 2) + pow(goal.y-current_y, 2) + pow(goal.z-current_z, 2));
		double dyaw = path[path_idx]->yaw - current_yaw;
		if (dyaw > pi){
			dyaw = -(2*pi - dyaw); 
		}
		else if (dyaw < -pi){
			dyaw = 2*pi + dyaw;
		}
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
	geometry_msgs::Quaternion quat = odom->pose.pose.orientation;
	tf2::Quaternion tf_quat;
	tf2::convert(quat, tf_quat);
	double current_roll, current_pitch, current_yaw;
	tf2::Matrix3x3(tf_quat).getRPY(current_roll, current_pitch, current_yaw);
	if (current_yaw < 0){
		current_yaw = 2 * pi - (-current_yaw);
	}
	// cout << "current_yaw: " << current_yaw << endl;
	// cout << "goal yaw: " << next_goal.yaw << endl;
	return std::abs(current_x-next_goal.x) < delta and std::abs(current_y-next_goal.y) < delta and std::abs(current_z-next_goal.z) < delta and std::abs(current_yaw-next_goal.yaw) < delta_angle;
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
	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		length += path[idx2]->p.distance(path[idx1]->p);
		++idx1;
	}
	return length;
}

double calculatePathRotation(std::vector<Node*> path, double current_yaw){
	int idx1 = 0;
	double rotation = 0;
	double start_yaw = path[idx1]->yaw;
	double dyaw_start = start_yaw - current_yaw;
	if (std::abs(dyaw_start) < pi){
		rotation += std::abs(dyaw_start);
	}
	else{
		rotation += 2*pi - std::abs(dyaw_start);
	}

	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		double this_yaw = path[idx1]->yaw;
		double next_yaw = path[idx2]->yaw;
		double dyaw = next_yaw - this_yaw;
		if (std::abs(dyaw) < pi){
			rotation += std::abs(dyaw);
		}
		else{
			rotation += 2*pi - std::abs(dyaw);
		}
		++idx1;
	}
	return rotation;
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
			double range = 0.3;
			xmin = target_node->p.x() - range;
			xmax = target_node->p.x() + range;
			ymin = target_node->p.y() - range;
			ymax = target_node->p.y() + range;
			zmin = target_node->p.z() - range;
			zmax = target_node->p.z() + range;
			std::vector<double> bbx1 {xmin, xmax, ymin, ymax, zmin, zmax}; // bounding box which is near node
			point3d midpoint = ((path[path_idx-1]->p) + (path[path_idx+1]->p));
			midpoint.x() /= 2;
			midpoint.y() /= 2;
			midpoint.z() /= 2;
			xmin = midpoint.x() - range;
			xmax = midpoint.x() + range;
			ymin = midpoint.y() - range;
			ymax = midpoint.y() + range;
			zmin = midpoint.z() - range;
			zmax = midpoint.z() + range;
			std::vector<double> bbx2 {xmin, xmax, ymin, ymax, zmin, zmax}; // bounding box which is near midpoint
			// cout << tree_ptr==NULL << endl;
			double rand_n = randomNumber(0, 10);
			Node* n;
			if (rand_n < 5){
				n = randomConfigBBX(tree, bbx1);
			}
			else{
				n = randomConfigBBX(tree, bbx2);
			}
			if (checkCollision(tree, n, path[path_idx-1]) and checkCollision(tree, n, path[path_idx+1])){
				double seg_1 = n->p.distance(path[path_idx-1]->p);
				double seg_2 = n->p.distance(path[path_idx+1]->p);
				double new_distance = seg_1 + seg_2;
				double dis_thresh = 0.6;
				if (new_distance < least_distance and seg_1 > dis_thresh and seg_2 > dis_thresh){
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

std::vector<Node*> getGoalCandidates(PRM* roadmap){
	std::vector<Node*> goal_candidates;
	double thresh = 0.1;
	double min_number = 10;
	double max_num_voxels = 0;
	// Go over node which has number of voxels larger than 0.5 maximum
	bool first_node = true;
	std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> goal_nodes = roadmap->getGoalNodes();
	while (true){
		Node* n = goal_nodes.top();
		goal_nodes.pop();

		if (n->num_voxels < max_num_voxels * thresh){
			break;
		}

		if (first_node){
			max_num_voxels = n->num_voxels;
			first_node = false;
		}
		goal_candidates.push_back(n);
	}

	if (goal_candidates.size() < min_number){
		Node* n = goal_nodes.top();
		goal_nodes.pop();
		goal_candidates.push_back(n);
	}

	return goal_candidates;
}


// Helper Function:
bool inRange(Node* n, Node* n_next, double yaw){
	point3d direction = n_next->p - n->p;
	point3d face  (cos(yaw), sin(yaw), 0);
	point3d projection (direction.x(), direction.y(), 0);
	
	double vertical_angle = projection.angleTo(direction);
	double horizontal_angle = projection.angleTo(face);

	// if (vertical_angle < FOV/2 and horizontal_angle < FOV/2){
	// double safety_discount = 2;
	if (horizontal_angle < FOV/2){
		cout << "vertical_angle: " << vertical_angle*180/pi << endl;
		cout << "horizontal_angle: " << horizontal_angle*180/pi << endl;
		return true;
	}
	else{
		return false;
	}
}


// Helper Function:
double interpolateNumVoxels(Node* n, double yaw){
	// Find the interval then interpolate
	double min_yaw, max_yaw;
	for (int i=0; i<yaws.size()-1; ++i){
		if (yaw > yaws[i] and y < yaws[i+1]){
			min_yaw = yaws[i];
			max_yaw = yaws[i+1];
			break;
		}
	}

	// Interpolate
	std::map<double, int> yaw_num_voxels = n->yaw_num_voxels;
	double num_voxels = yaw_num_voxels[min_yaw] + (yaw - min_yaw) * (yaw_num_voxels[max_yaw]-yaw_num_voxels[min_yaw])/(max_yaw - min_yaw);
	return num_voxels;
}

// Genrate Path to all candidates and evaluate them, then pick the best path
std::vector<Node*> findBestPath(PRM* roadmap, Node* start, std::vector<Node*> goal_candidates, OcTree& tree){
	double linear_velocity = 0.3;
	double angular_velocity = 0.8;
	double current_yaw = start->yaw;
	std::vector<Node*> final_path;
	// 1. Generate all the path
	std::vector<std::vector<Node*>> path_vector;
	std::vector<double> path_score; // Score = num_voxels/time
	std::vector<Node*> candidate_path;
	for (Node* goal: goal_candidates){
		candidate_path = AStar(roadmap, start, goal, tree);
		// find the appropriate yaw for nodes:
	  		// 1. sensor range from node in this yaw can see next node
		double score = 0;
		double total_num_voxels = 0;
		for (int i=0; i<candidate_path.size(); ++i){
			// last one
			if (i == candidate_path.size()-1){
				double max_yaw;
				double max_voxels = 0;
				calculateUnknown(tree, candidate_path[i], dmax);
				for (double yaw: yaws){
					if (candidate_path[i]->yaw_num_voxels[yaw] > max_voxels){
						max_voxels = candidate_path[i]->yaw_num_voxels[yaw];
						max_yaw = yaw;
					}
				}
				candidate_path[i]->yaw = max_yaw;
				total_num_voxels += max_voxels;
			}
			else{

				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*pi - (-node_yaw);
				}
				total_num_voxels += interpolateNumVoxels(this_node, node_yaw);
			}
		}

		if (candidate_path.size() != 0){
			for (int i=0; i<candidate_path.size()-1; ++i){
				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*pi - (-node_yaw);
				}
				candidate_path[i]->yaw = node_yaw;
			}
			double total_path_length = calculatePathLength(candidate_path);
			double total_path_rotation = calculatePathRotation(candidate_path, current_yaw);
			double total_time = total_path_length/linear_velocity + total_path_rotation/angular_velocity;
			score = total_num_voxels/total_time;
		}
		path_score.push_back(score);
		path_vector.push_back(candidate_path);
	}
	// cout << "check 1" << endl;
	double best_score = 0;
	double best_idx = 0;
	for (int i=0; i<path_vector.size(); ++i){
		if (path_score[i] > best_score){
			best_score = path_score[i];
			best_idx = i;
		}
	}


	// cout << "check 2" << endl;

	final_path = path_vector[best_idx];
	for (int i=0; i<final_path.size()-1; ++i){
		Node* this_node = final_path[i];
		Node* next_node = final_path[i+1];
		point3d direction = next_node->p - this_node->p;
		double node_yaw = atan2(direction.y(), direction.x());
		if (node_yaw < 0){
			node_yaw = 2*pi - (-node_yaw);
		}
		final_path[i]->yaw = node_yaw;
	}
	cout << "final path size: " << final_path.size() << endl;
	return final_path;
}

