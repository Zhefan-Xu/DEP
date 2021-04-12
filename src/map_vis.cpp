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
#include <nlopt.hpp>
#include <iostream>
using namespace std::chrono;
using namespace message_filters;
ros::Publisher map_vis_pub;
ros::Publisher goal_pub;
ros::Publisher path_vis_pub;
ros::Publisher plan_vis_pub;
ros::Publisher old_plan_vis_pub;

AbstractOcTree* abtree;
OcTree* tree_ptr;
voxblox::EsdfServer* voxblox_server_ptr;
PRM* roadmap;
Node* last_goal;
std::ofstream optimization_data;
std::ofstream replan_data;

bool no_replan = true;
bool new_plan = true;
bool replan = false;
// int num_sample = 100;
int num_goal_candicates = 20;
int unknown_thresh = 50;
int count_no_increase = 0;
int roadmap_size_last = 0;
double linear_velocity = 0.3;
double delta = 0.01; // criteria for chechking reach
double delta_angle = 0.1;
visualization_msgs::MarkerArray map_markers;
std::vector<visualization_msgs::Marker> map_vis_array;
visualization_msgs::MarkerArray plan_markers;
visualization_msgs::MarkerArray old_plan_markers;


visualization_msgs::Marker path_marker;
visualization_msgs::Marker old_path_marker;
std::vector<Node*> path;
std::vector<Node*> old_path;
int path_idx = 0;
DEP::Goal next_goal;
int count_iteration = 0;
double total_time = 0;
double total_comp_time = 0;
double total_path_length = 0;
double total_path_segment = 0;
double total_distance_to_obstacle = 0;
double previous_goal_voxels = -100;


DEP::Goal getNextGoal(std::vector<Node*> path, int path_idx, const nav_msgs::OdometryConstPtr& odom);
bool isReach(const nav_msgs::OdometryConstPtr& odom, DEP::Goal next_goal);
Node* findStartNode(PRM* map, Node* current_node, OcTree& tree);
double calculatePathLength(std::vector<Node*> path);
double calculatePathRotation(std::vector<Node*> path, double current_yaw);
Node Goal2Node(DEP::Goal g);
std::vector<Node*>& improvePath(std::vector<Node*> &path, int path_idx, OcTree & tree, double& least_distance);
std::vector<Node*> getGoalCandidates(PRM* roadmap);
std::vector<Node*> findBestPath(PRM* roadmap, Node* start, std::vector<Node*> goal_candidates, OcTree& tree, bool replan);
bool checkNextGoalCollision(Node current_pose, DEP::Goal next_goal, OcTree& tree);
double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);
std::vector<Node*> optimize_path(std::vector<Node*> path, OcTree* tree_ptr, voxblox::EsdfServer* voxblox_server, double current_yaw);
double findDistanceToWall(double x, double y, double z, voxblox::EsdfServer &voxblox_server);
bool checkPathCollision(std::vector<Node*> &path, OcTree &tree);
bool minDistanceCondition(std::vector<Node*> &path);
bool sensorConditionPath(std::vector<Node*> &path);
double calculatePathTime(std::vector<Node*> &path, double current_yaw, double linear_velocity, double angular_velocity);
double calculatePathObstacleDistance(std::vector<Node*> &path, voxblox::EsdfServer &voxblox_server, OcTree &tree);
double calculatePathGain(std::vector<Node*> path, OcTree& tree);
double calculateTotalPathObstacleDistance(std::vector<Node*> &path, voxblox::EsdfServer &voxblox_server, OcTree &tree);




bool reach = false;
bool first_time = true;
bool half_stop = false;
double x, y, z;
Node current_pose;
Node last_goal_node (point3d (-100, -100, -100));
Node collision_node;
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
	// tree_ptr->writeBinary("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_octree.bt");
	geometry_msgs::Quaternion quat = odom->pose.pose.orientation;
	tf2::Quaternion tf_quat;
	double current_roll, current_pitch, current_yaw;
	tf2::Matrix3x3(tf_quat).getRPY(current_roll, current_pitch, current_yaw);
	if (current_yaw < 0){
		current_yaw = 2 * PI_const - (-current_yaw);
	}

	// Check for collision
	if (not first_time and not no_replan){
		bool has_collision = checkNextGoalCollision(current_pose, next_goal, *tree_ptr);
		if (has_collision){
			cout << "COLLISION HAPPENS! NEED REPLANNING" << endl;
			collision_node.p.x() = next_goal.x;
			collision_node.p.y() = next_goal.y;
			collision_node.p.z() = next_goal.z;
			if (not no_replan){
				replan = true;
			}
		}
	}

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
			start = last_goal;	
			roadmap = buildRoadMap(*tree_ptr, roadmap, path,  start, map_vis_array);
			// cout << "here1" <<endl;
			if (roadmap->getSize() - roadmap_size_last < 3){
				++count_no_increase;
			}
			else{
				count_no_increase = 0;
			}
			if (count_no_increase >= 30){
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

		// cout << "here2" <<endl;
		
		// cout << "map size: " << map->getSize() <<endl;
		// cout << "map vis array size: " << map_vis_array.size() << endl;
		// for (Node* n : map->getGoalNodes()){
		// 	print_node(*n);
		// }
		
		// Test mulitgoal a star

		// Node* start = map->nearestNeighbor(&current_pose);
		
		auto start_time_search = high_resolution_clock::now();
		// cout << "here3" <<endl;
		std::vector<Node*> goal_candidates = getGoalCandidates(roadmap);
		// cout << "here4" <<endl;
		path = findBestPath(roadmap, start, goal_candidates, *tree_ptr, replan);
		old_path = path;
		last_goal = *(path.end()-1);	
		// cout << "here5" <<endl;
		auto stop_time_search = high_resolution_clock::now();
		if (path.size() >= 3){
			path = optimize_path(path, tree_ptr, voxblox_server_ptr, current_yaw);
			// // cout << "old path size: " << old_path.size() << endl;
			// // cout << "new path size: " << path.size() << endl;
			// // cout << "old_path last: " << old_path[old_path.size()-1]->p << endl;
			// // cout << "new_path last: " << path[old_path.size()-1]->p << endl;
			// // double old_gain = calculatePathGain(old_path, *tree_ptr);
			// // double new_gain = calculatePathGain(path, *tree_ptr);
			// // cout << "new path gain/old path gain: " << new_gain << "/" << old_gain << " = " << new_gain/old_gain << endl;
			// double old_obstacle_distance = calculateTotalPathObstacleDistance(old_path, *voxblox_server_ptr, *tree_ptr);
			// optimization_data << old_obstacle_distance;
			// double new_obstacle_distance = calculateTotalPathObstacleDistance(path, *voxblox_server_ptr, *tree_ptr);
			// optimization_data << " " << new_obstacle_distance;
			// double linear_velocity = 0.3, angular_velocity = 0.8;
			// double old_path_time = calculatePathTime(old_path, current_yaw, linear_velocity, angular_velocity);
			// optimization_data << " " << old_path_time;
			// double new_path_time = calculatePathTime(path, current_yaw, linear_velocity, angular_velocity);
			// optimization_data << " " << new_path_time;
			// double old_path_length = calculatePathLength(old_path);
			// optimization_data << " " << old_path_length;
			// double new_path_length = calculatePathLength(path);
			// optimization_data << " " << new_path_length << endl;
		}
		
		auto duration_search = duration_cast<microseconds>(stop_time_search - start_time_search);
		cout << "Time used for search is: " << duration_search.count()/1e6 << " Seconds" << endl;
		total_path_length += calculatePathLength(path);
		total_distance_to_obstacle += calculateTotalPathObstacleDistance(path, *voxblox_server_ptr, *tree_ptr);
		total_path_segment += path.size();
		// if (path[path.size()-1]->num_voxels < unknown_thresh){
		// 	cout << "Exploration Terminates!!!!!!!!!!!" << endl;
		// 	ros::shutdown();
		// }
		auto stop_time = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop_time - start_time);
		replan_data <<  duration.count()/1e6 << " " << duration_search.count()/1e6 << endl;
		cout << "Time taken by this Iteration: "
        << duration.count()/1e6 << " seconds" << endl;
        total_comp_time += duration.count()/1e6;
        ++count_iteration;
        cout << "Total Computation Time: " << total_comp_time << endl;
        cout << "Path Length So Far: " << total_path_length << endl;
		cout << "Avg Computation Time: " << total_comp_time/total_path_segment << endl;
        cout << "==============================" << "END" << "==============================" << endl;
		// print_path(path);

		path_idx = 0;
		if (path.size() != 0){
			next_goal = getNextGoal(path, path_idx, odom);
			reach = false;
			++path_idx;
		}
		new_plan = false;
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
			
		}
		// ====================================================================
		
	}
	else if (replan){
		auto replan_start_time = high_resolution_clock::now();

		// get goal candidates
		std::vector<Node*> goal_candidates = getGoalCandidates(roadmap);

		// Find start 
		Node* start = findStartNode(roadmap, &current_pose, *tree_ptr); 

		// Find best path based on collision node
		path = findBestPath(roadmap, start, goal_candidates, *tree_ptr, replan);
		old_path = path;
		last_goal = *(path.end()-1);	
		if (path.size() >= 3){
			path = optimize_path(path, tree_ptr, voxblox_server_ptr, current_yaw);
			// double old_obstacle_distance = calculateTotalPathObstacleDistance(old_path, *voxblox_server_ptr, *tree_ptr);
			// optimization_data << old_obstacle_distance;
			// double new_obstacle_distance = calculateTotalPathObstacleDistance(path, *voxblox_server_ptr, *tree_ptr);
			// optimization_data << " " << new_obstacle_distance;
			// double linear_velocity = 0.3, angular_velocity = 0.8;
			// double old_path_time = calculatePathTime(old_path, current_yaw, linear_velocity, angular_velocity);
			// optimization_data << " " << old_path_time;
			// double new_path_time = calculatePathTime(path, current_yaw, linear_velocity, angular_velocity);
			// optimization_data << " " << new_path_time;
			// double old_path_length = calculatePathLength(old_path);
			// optimization_data << " " << old_path_length;
			// double new_path_length = calculatePathLength(path);
			// optimization_data << " " << new_path_length << endl;
		}
		auto replan_stop_time = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(replan_stop_time - replan_start_time);
		// replan_data << "Replan: " << duration.count()/1e6 << endl;
		total_comp_time += duration.count()/1e6;
		total_path_length += calculatePathLength(path);
		total_distance_to_obstacle += calculateTotalPathObstacleDistance(path, *voxblox_server_ptr, *tree_ptr);

		path_idx = 0;
		if (path.size() != 0){
			next_goal = getNextGoal(path, path_idx, odom);
			reach = false;
			++path_idx;
		}
		replan = false;
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
			
		}

		// ====================================================================
	}
	else{
		std::vector<visualization_msgs::Marker> plan_vis_array;
		std::vector<visualization_msgs::Marker> old_plan_vis_array;
		std::vector<geometry_msgs::Point> path_vis_vector;
		std::vector<geometry_msgs::Point> old_path_vis_vector;
		plan_markers.markers = plan_vis_array;
		old_plan_markers.markers = old_plan_vis_array;
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
					if (old_path != path){
						geometry_msgs::Point p3, p4;
						p3.x = old_path[i]->p.x();
						p3.y = old_path[i]->p.y();
						p3.z = old_path[i]->p.z();
						p4.x = old_path[i+1]->p.x();
						p4.y = old_path[i+1]->p.y();
						p4.z = old_path[i+1]->p.z();
						old_path_vis_vector.push_back(p3);
						old_path_vis_vector.push_back(p4);
					}
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

				if (old_path != path){
					visualization_msgs::Marker old_way_point;
					old_way_point.header.frame_id = "world";
					old_way_point.id = 9999+i;
					old_way_point.type = visualization_msgs::Marker::SPHERE;
					old_way_point.pose.position.x = old_path[i]->p.x();
					old_way_point.pose.position.y = old_path[i]->p.y();
					old_way_point.pose.position.z = old_path[i]->p.z();
					old_way_point.lifetime = ros::Duration(0.5);
					old_way_point.scale.x = 0.1;
					old_way_point.scale.y = 0.1;
					old_way_point.scale.z = 0.1;
					old_way_point.color.a = 1.0;
					old_way_point.color.r = 1.0;
					old_way_point.color.g = 0.0;
					old_way_point.color.b = 0.0;
					// old plan Direction
					visualization_msgs::Marker old_direction_ptr;
					old_direction_ptr.header.frame_id = "world";
					old_direction_ptr.id = 6666+i;
					old_direction_ptr.type = visualization_msgs::Marker::ARROW;
					old_direction_ptr.pose.position.x = old_path[i]->p.x();
					old_direction_ptr.pose.position.y = old_path[i]->p.y();
					old_direction_ptr.pose.position.z = old_path[i]->p.z();
					tf2::Quaternion quat;
					quat.setRPY(0, 0, old_path[i]->yaw);
					old_direction_ptr.pose.orientation.x = quat[0];
					old_direction_ptr.pose.orientation.y = quat[1];
					old_direction_ptr.pose.orientation.z = quat[2];
					old_direction_ptr.pose.orientation.w = quat[3];
					old_direction_ptr.lifetime = ros::Duration(0.5);
					old_direction_ptr.scale.x = 0.3;
					old_direction_ptr.scale.y = 0.03;
					old_direction_ptr.scale.z = 0.03;
					old_direction_ptr.color.a = 1;
					old_direction_ptr.color.r = 0;
					old_direction_ptr.color.g = 0;
					old_direction_ptr.color.b = 1;

					old_plan_vis_array.push_back(old_direction_ptr);
					old_plan_vis_array.push_back(old_way_point);
				}
				// Direction
				visualization_msgs::Marker direction_ptr;
				direction_ptr.header.frame_id = "world";
				direction_ptr.id = 66666+i;
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
			path_marker.lifetime = ros::Duration(0.5);
			plan_vis_array.push_back(path_marker);
			plan_markers.markers = plan_vis_array;
			if (old_path != path){
				old_path_marker.header.frame_id = "world";
				old_path_marker.points = old_path_vis_vector;
				old_path_marker.id = 10000000;
				old_path_marker.type = visualization_msgs::Marker::LINE_LIST;
				old_path_marker.scale.x = 0.05;
				old_path_marker.scale.y = 0.05;
				old_path_marker.scale.z = 0.05;
				old_path_marker.color.a = 1.0;
				old_path_marker.color.g = 1.0;
				old_path_marker.color.r = 1.0;
				old_path_marker.lifetime = ros::Duration(0.5);
				old_plan_vis_array.push_back(old_path_marker);
				old_plan_markers.markers = old_plan_vis_array;
			}

			// ====================================================================

		}

		
	}
	delete abtree;
	
	
	
}
 

int main(int argc, char** argv){
	ros::init(argc, argv, "map_visualizer");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	voxblox_server_ptr = new voxblox::EsdfServer(nh, nh_private);
	// Eigen::Vector3d p_test (0.3, 6, 1);
	// double distance = 0;

	// ros::Subscriber sub = n.subscribe("octomap_binary", 100, callback);
	map_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("map_vis_array", 0);
	goal_pub = nh.advertise<DEP::Goal>("goal", 0);
	path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis_array", 0);
	plan_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("plan_vis_array", 0);
	old_plan_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("old_plan_vis_array", 0);

	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 100);
	message_filters::Subscriber<octomap_msgs::Octomap> map_sub(nh, "octomap_binary", 100);
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, octomap_msgs::Octomap> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync (MySyncPolicy(100), odom_sub, map_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::Rate loop_rate(10);
	
	// optimization_data.open("/home/zhefan/Desktop/Optimization_Data/tunnel/optimization_single/tunnel_opt3.txt");
	// optimization_data.open("/home/zhefan/Desktop/optimization_revise/tunnel/optimization_single/tunnel_opt3.txt");
	// optimization_data.open("/home/zhefan/Desktop/optimization_revise/auditorium/optimization_single/auditorium_opt3.txt");
	// replan_data.open("/home/zhefan/Desktop/Replan_Data_New/DEP/replan_data1.txt");
	while (ros::ok()){
		map_vis_pub.publish(map_markers);	
		goal_pub.publish(next_goal);
		plan_vis_pub.publish(plan_markers);
		if (path != old_path){
			old_plan_vis_pub.publish(old_plan_markers);
		}
		// voxblox_server.saveMap("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_voxblox.vxblx");
		// voxblox_server.loadMap("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_voxblox.vxblx");
		// path_vis_pub.publish(path_marker);
		// cout << tree_ptr << endl;
		// improvePath(path, path_idx, *tree_ptr);
		// if (success){
		// 	cout << "distance to wall: " << distance << endl;
		// }

		// }
		// else{
		// 	cout << "No Map" << endl;
		// }
		// Write stats:
		// std::ofstream data;
		// data.open("/home/zhefan/Desktop/Experiment_Stats/apartment/DEP/apartment_stats6.txt");
		// auto stop_time_total = high_resolution_clock::now();
		// auto total_duration = duration_cast<microseconds>(stop_time_total - start_time_total);
		// total_time = total_duration.count()/1e6;
		// data << "Total Exploration Time: " << total_time << endl;
		// data << "Total Computation Time: " << total_comp_time << endl;
		// data << "Total Path Length: " << total_path_length << endl; 

		std::ofstream optimization_data_sperate;
		// optimization_data_sperate.open("/home/zhefan/Desktop/Optimization_Data/tunnel/non_optimization/tunnel_nonopt_data3.txt");
		// optimization_data_sperate.open("/home/zhefan/Desktop/Optimization_Data/tunnel/optimization/tunnel_opt_data1.txt");
		// optimization_data_sperate.open("/home/zhefan/Desktop/optimization_revise/tunnel/non_optimization/tunnel_nonopt_data3.txt");
		// optimization_data_sperate.open("/home/zhefan/Desktop/optimization_revise/auditorium/optimization/auditorium_opt_data3.txt");
		// optimization_data_sperate.open("/home/zhefan/Desktop/optimization_revise/auditorium/non_optimization/auditorium_nonopt_data1.txt");
		auto stop_time_total = high_resolution_clock::now();
		auto total_duration = duration_cast<microseconds>(stop_time_total - start_time_total);
		total_time = total_duration.count()/1e6;
		optimization_data_sperate << "Total Exploration Time: " << total_time << endl;
		optimization_data_sperate << "Total Path Length: " << total_path_length << endl;
		optimization_data_sperate << "Total obstacle distance: " << total_distance_to_obstacle << endl;
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
			current_yaw = 2 * PI_const - (-current_yaw);
		}
		double distance_to_goal = sqrt(pow(goal.x-current_x, 2) + pow(goal.y-current_y, 2) + pow(goal.z-current_z, 2));
		double dyaw = path[path_idx]->yaw - current_yaw;
		if (dyaw > PI_const){
			dyaw = -(2*PI_const - dyaw); 
		}
		else if (dyaw < -PI_const){
			dyaw = 2*PI_const + dyaw;
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
		current_yaw = 2 * PI_const - (-current_yaw);
	}
	// cout << "current_yaw: " << current_yaw << endl;
	// cout << "goal yaw: " << next_goal.yaw << endl;
	return std::abs(current_x-next_goal.x) < delta and std::abs(current_y-next_goal.y) < delta and std::abs(current_z-next_goal.z) < delta and std::abs(current_yaw-next_goal.yaw) < delta_angle;
}

Node* findStartNode(PRM* map, Node* current_node, OcTree& tree){
	std::vector<Node*> knn = map->kNearestNeighbor(current_node, 15);
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
	if (std::abs(dyaw_start) < PI_const){
		rotation += std::abs(dyaw_start);
	}
	else{
		rotation += 2*PI_const - std::abs(dyaw_start);
	}

	for (int idx2=1; idx2<=path.size()-1; ++idx2){
		double this_yaw = path[idx1]->yaw;
		double next_yaw = path[idx2]->yaw;
		double dyaw = next_yaw - this_yaw;
		if (std::abs(dyaw) < PI_const){
			rotation += std::abs(dyaw);
		}
		else{
			rotation += 2*PI_const - std::abs(dyaw);
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
		cout << "vertical_angle: " << vertical_angle*180/PI_const << endl;
		cout << "horizontal_angle: " << horizontal_angle*180/PI_const << endl;
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
		if (yaw > yaws[i] and yaw < yaws[i+1]){
			min_yaw = yaws[i];
			max_yaw = yaws[i+1];
			break;
		}
	}

	// Interpolate
	std::map<double, int> yaw_num_voxels = n->yaw_num_voxels;
	double num_voxels = yaw_num_voxels[min_yaw] + (yaw - min_yaw) * (yaw_num_voxels[max_yaw]-yaw_num_voxels[min_yaw])/(max_yaw - min_yaw);
	// cout << "max angle: " << max_yaw << endl;
	// cout << "min_angle: " << min_yaw << endl;
	// cout << "yaw: " << yaw << endl;
	// cout << "max: " << yaw_num_voxels[max_yaw] << endl;
	// cout << "min: " << yaw_num_voxels[min_yaw] << endl;
	return num_voxels;
}

// Genrate Path to all candidates and evaluate them, then pick the best path
std::vector<Node*> findBestPath(PRM* roadmap,
                                Node* start,
                                std::vector<Node*> goal_candidates,
                                OcTree& tree,
                                bool replan=false){
	double linear_velocity = 0.3;
	double angular_velocity = 0.8;
	double current_yaw = start->yaw;
	double score_thresh = 0.5;

	std::vector<Node*> final_path;
	// 1. Generate all the path
	std::vector<std::vector<Node*>> path_vector;
	std::vector<double> path_score; // Score = num_voxels/time
	std::vector<Node*> candidate_path;
	int count_path_id = 0;

	for (Node* goal: goal_candidates){
		candidate_path = AStar(roadmap, start, goal, tree, replan);	

		
		// find the appropriate yaw for nodes:
	  		// 1. sensor range from node in this yaw can see next node
		double score = 0;
		double total_num_voxels = 0;
		double total_unknwon;
		for (int i=0; i<candidate_path.size(); ++i){
			// last one
			if (i == candidate_path.size()-1){
				double max_yaw = 0;
				double max_voxels = 0;
				total_unknwon = calculateUnknown(tree, candidate_path[i], dmax);
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
					node_yaw = 2*PI_const - (-node_yaw);
				}
				total_num_voxels += interpolateNumVoxels(this_node, node_yaw);
				// cout << "interpolate result: " << interpolateNumVoxels(this_node, node_yaw) << endl;
			}
		}

		if (candidate_path.size() != 0){
			for (int i=0; i<candidate_path.size()-1; ++i){
				Node* this_node = candidate_path[i];
				Node* next_node = candidate_path[i+1];
				point3d direction = next_node->p - this_node->p;
				double node_yaw = atan2(direction.y(), direction.x());
				if (node_yaw < 0){
					node_yaw = 2*PI_const - (-node_yaw);
				}
				candidate_path[i]->yaw = node_yaw;
			}
			double total_path_length = calculatePathLength(candidate_path);
			double total_path_rotation = calculatePathRotation(candidate_path, current_yaw);
			double total_time = total_path_length/linear_velocity + total_path_rotation/angular_velocity;
			score = total_num_voxels/total_time;
			// cout << "Path " << count_path_id << " score: " << score << " length: " << total_path_length << " Rotation: " << total_path_rotation << " Time: " << total_time <<  " Voxels: " << total_num_voxels << endl;
		}
		path_score.push_back(score);
		path_vector.push_back(candidate_path);
		++count_path_id;
	}






	// Find best score
	double best_score = 0;
	double best_idx = 0;
	for (int i=0; i<path_vector.size(); ++i){
		if (path_score[i] > best_score){
			best_score = path_score[i];
			best_idx = i;
		}
	}
	// cout << "initial_best_idx: " << best_idx << endl;

	// // Filter the score which is larger than threshold
	// std::vector<std::vector<Node*>> final_candidate_path;
	// std::vector<double> final_candidate_score;
	// for (int i=0; i<path_vector.size(); ++i){
	// 	if (path_score[i] > score_thresh * best_score){
	// 		final_candidate_path.push_back(path_vector[i]);
	// 		final_candidate_score.push_back(path_score[i]);
	// 	}
	// }


	// // for each of them calculate the total distance to reach other
	// std::vector<double> total_distance_to_other;
	// double shortest_distance = 100000;
	// for (int i=0; i<final_candidate_path.size(); ++i){
	// 	// calculate the total distance to other path
	// 	double total_distance = 0;
	// 	for (int j=0; j<final_candidate_path.size(); ++j){
	// 		if (j != i){
	// 			std::vector<Node*> path_to_other = AStar(roadmap, *(final_candidate_path[i].end()-1), *(final_candidate_path[j].end()-1), tree, false);
	// 			total_distance += calculatePathLength(path_to_other);
	// 		}
	// 	}
	// 	if (total_distance < shortest_distance){
	// 		shortest_distance = total_distance;
	// 	}
	// 	cout << "path: " << i <<" raw score: " << final_candidate_score[i] << endl;
	// 	cout << "path: " << i << " total distance: " << total_distance <<endl;
	// 	total_distance_to_other.push_back(total_distance);
	// }	


	// // Update final score
	// for (int i=0; i<final_candidate_path.size(); ++i){
	// 	double discount_factor = total_distance_to_other[i]/shortest_distance;
	// 	final_candidate_score[i] /= discount_factor;
	// 	cout << "path: " << i << " discount factor: " << discount_factor << endl;
	// 	cout << "path: " << i << " updated score: " << final_candidate_score[i] << endl;
	// }

	// // Find the best idx
	// double final_best_score = 0;
	// best_idx = 0;
	// for (int i=0; i<final_candidate_path.size(); ++i){
	// 	if (final_candidate_score[i] > final_best_score){
	// 		final_best_score = final_candidate_score[i];
	// 		best_idx = i;
	// 	}
	// }

	// cout << "best_idx: " << best_idx << endl;
	// Get the best path
	// cout << "check3" << endl;
	final_path = path_vector[best_idx];
	// cout << final_path.size() << endl;
	for (int i=0; i<final_path.size()-1; ++i){
		Node* this_node = final_path[i];
		Node* next_node = final_path[i+1];
		point3d direction = next_node->p - this_node->p;
		double node_yaw = atan2(direction.y(), direction.x());
		if (node_yaw < 0){
			node_yaw = 2*PI_const - (-node_yaw);
		}
		final_path[i]->yaw = node_yaw;
	}
	// cout << "check4" << endl;
	// cout << "final id: " << best_idx << endl;
	// cout << "final path size: " << final_path.size() << endl;
	return final_path;
}

bool checkNextGoalCollision(Node current_node, DEP::Goal next_goal, OcTree& tree){
	Node goal_node (point3d (next_goal.x, next_goal.y , next_goal.z));
	return checkCollision(tree, &current_node, &goal_node); // true if there is collision otherwise no collision
}

typedef struct start_goal{
	double sx, sy, sz, gx, gy, gz, current_yaw, last_yaw, lv, av, time, distance;
	OcTree* tree;
	voxblox::EsdfServer* voxblox_server;
	start_goal(){}
	start_goal(double sx_,
			   double sy_, 
			   double sz_, 
			   double gx_, 
			   double gy_, 
			   double gz_, 
			   double current_yaw_, 
			   double last_yaw_,
			   double lv_,
			   double av_,
			   double time_,
			   double distance_, 
			   OcTree* tree_ptr,
			   voxblox::EsdfServer* voxblox_server_ptr
			   ){
		sx = sx_;
		sy = sy_;
		sz = sz_;
		gx = gx_;
		gy = gy_;
		gz = gz_;
		current_yaw = current_yaw_;
		last_yaw = last_yaw_;
		lv = lv_;
		av = av_;
		time = time_;
		distance = distance_;
		tree = tree_ptr;
		voxblox_server = voxblox_server_ptr;
	}
} start_goal;

void reconstructPath(const std::vector<double> &x, start_goal *sg, std::vector<Node*> &path);

double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
	if (!grad.empty()) {}
    // x is contains all the INTERMEDIATE waypoint
	start_goal* sg = reinterpret_cast<start_goal*>(my_func_data);
	OcTree* tree_ptr = sg->tree;
	voxblox::EsdfServer* voxblox_server_ptr = sg->voxblox_server;
	double current_yaw = sg->current_yaw;
	double linear_velocity = sg->lv;
	double angular_velocity = sg->av;
	double time = sg->time;
	double distance = sg->distance;
	// 1. reconstruct the node from variables amd data:
	std::vector<Node*> path;
	reconstructPath(x, sg, path);
	// double path_length = calculatePathLength(path);
	double path_time = calculatePathTime(path, current_yaw, linear_velocity, angular_velocity);
	double mean_distance = calculatePathObstacleDistance(path, *voxblox_server_ptr, *tree_ptr);
	double collision_factor;
	if (checkPathCollision(path, *tree_ptr) == true){
		collision_factor = 100;
	}
	else{
		collision_factor = 1;
	}
	double distance_factor;
	if (minDistanceCondition(path) == false){
		distance_factor = 100;
	}
	else{
		distance_factor = 1;
	}
	double sensor_factor;
	if (sensorConditionPath(path) == false){
		sensor_factor = 100;
	}
	else{
		sensor_factor = 1;
	} 

	// path_time *= collision_factor;
	// path_time *= distance_factor;
	// path_time *= sensor_factor;
	double wp = 0.5;
	double wd = 0.5;
	double score = (wp*path_time/time + wd*(distance/mean_distance)) * collision_factor *
					distance_factor * sensor_factor;
	// cout << score << endl;
	// cout << path_time << endl;
	// print_node_vector(path);
    return score;
}

std::vector<Node*> optimize_path(std::vector<Node*> path, OcTree* tree_ptr, voxblox::EsdfServer* voxblox_server, double current_yaw){
	// print_node_vector(path);
	// Intialize optimizer
	// cout << "1" <<endl;
	int num_of_variables = (path.size()-2) * 3;
	Node* start = *(path.begin());
	Node* goal = *(path.end()-1);
	
	// start_goal sg = {start->p.x(), start->p.y(), start->p.z(), 
	// 				 goal->p.x(), goal->p.y(), goal->p.z()};
	double last_yaw = (*(path.end()-1))->yaw;
	double linear_velocity = 0.3;
	double angular_velocity = 0.8;
	// cout << "2" <<endl;
	double time = calculatePathTime(path, current_yaw, linear_velocity, angular_velocity);
	// cout << "3" <<endl;
	double distance = calculatePathObstacleDistance(path, *voxblox_server, *tree_ptr);
	// cout << "4" <<endl;
	start_goal sg (start->p.x(), start->p.y(), start->p.z(), 
	 				 goal->p.x(), goal->p.y(), goal->p.z(), current_yaw, last_yaw,
	 				 linear_velocity, angular_velocity, time, distance,
	 				 tree_ptr, voxblox_server);
	// cout << "5" <<endl;
	nlopt::opt opt(nlopt::GN_DIRECT_L, num_of_variables);
	// set bound -> +- 0.5 m
	int count_node_idx = 0;
	int count_variable = 0;
	std::vector<double> lb (num_of_variables);
	std::vector<double> ub (num_of_variables);
	// Initialize variables
	std::vector<double> x_variables (num_of_variables);
	double bound_dis = 0.5;
	while (count_node_idx < path.size()){
		if (count_node_idx != 0 and count_node_idx != path.size()-1){
			double x = path[count_node_idx]->p.x();
			double y = path[count_node_idx]->p.y();
			double z = path[count_node_idx]->p.z();
			for (int i=0; i<3; ++i){
				if (i == 0){
					lb[count_variable] = x-bound_dis;
					ub[count_variable] = x+bound_dis;
					x_variables[count_variable] = x;
				}
				else if(i == 1){
					lb[count_variable] = y-bound_dis;
					ub[count_variable] = y+bound_dis;
					x_variables[count_variable] = y;
				}
				else if(i == 2){
					lb[count_variable] = z-bound_dis;
					ub[count_variable] = z+bound_dis;
					x_variables[count_variable] = z;
				}
				++count_variable;
			}
		}
		++count_node_idx;
	}
	// cout << "6" <<endl;
	// set bound
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);
	// set objective function
	opt.set_min_objective(objective_function, &sg);
	// set termination criteria
	// opt.set_xtol_rel(0.2);
	opt.set_maxeval(1000);
	double min_obj;
	// cout << "here" << endl;

	try{
		nlopt::result result = opt.optimize(x_variables, min_obj);
		// cout << "x[0]: " << x_variables[0] << endl;
		// cout << "Time before: " << calculatePathTime(path, current_yaw, linear_velocity, angular_velocity) << endl;
		cout << "min score: " << min_obj << endl;
		// cout << "theoretical min: " << start->p.distance(goal->p);
		// for (int i=0; i < path.size()-2; ++i){
		// 	cout << x_variables[i*3+0] << " " << x_variables[i*3+1] << " " << x_variables[i*3+2] << endl; 
		// }
		// cout << "7" <<endl;
		std::vector<Node*> optimized_path;
		reconstructPath(x_variables, &sg, optimized_path);
		return optimized_path;
	}
	catch(std::exception &e){
		cout << "nlopt fail" << e.what() << endl;
		return path;
	}


}

double findDistanceToWall(double x, double y, double z, voxblox::EsdfServer &voxblox_server){
	Eigen::Vector3d p (x, y, z);
	double distance = 0;
	bool success = voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(p, &distance);
	if (success){
		return distance;
	}
	else{
		return 1000000;
	}
}

bool checkPathCollision(std::vector<Node*> &path, OcTree& tree){
	int path_idx = 0;
	while (path_idx < path.size()-1){
		Node* this_node = path[path_idx];
		Node* next_node = path[path_idx+1];
		if (checkCollision(tree, this_node, next_node) == true){
			return true;
		}
		++path_idx;
	}
	return false;
}

bool minDistanceCondition(std::vector<Node*> &path){
	int path_idx = 0;
	double dis_thresh = 0.5;
	while (path_idx < path.size()-1){
		Node* this_node = path[path_idx];
		Node* next_node = path[path_idx+1];
		if (this_node->p.distance(next_node->p) < dis_thresh){
			return false;
		}
		++path_idx;
	}
	return true;
}

bool sensorConditionPath(std::vector<Node*> &path){
	int path_idx = 0;
	while (path_idx < path.size()-1){
		Node* this_node = path[path_idx];
		Node* next_node = path[path_idx+1];
		if (sensorRangeCondition(this_node, next_node) == false){
			return false;
		}
		++path_idx;
	}
	return true;
}

double calculatePathTime(std::vector<Node*> &path, double current_yaw, double linear_velocity, double angular_velocity){
	double length = calculatePathLength(path);
	double rotaton = calculatePathRotation(path,current_yaw);
	double time = length/linear_velocity + rotaton/angular_velocity;
	return time;
}

double calculatePathObstacleDistance(std::vector<Node*> &path, voxblox::EsdfServer &voxblox_server, OcTree &tree){
	double total_distance = 0; 
	double thresh_dist = 2;
	int count_node = 0;
	int path_idx = 0;
	while (path_idx < path.size() - 1){
		Node* this_node = path[path_idx];
		Node* next_node = path[path_idx+1];
		point3d p1 = this_node->p;
		point3d p2 = next_node->p;
		std::vector<point3d> ray;
		tree.computeRay(p1, p2, ray);

		for (point3d p: ray){
			Eigen::Vector3d p_eigen (p.x(), p.y(), p.z());
			double distance = 0;
			bool success = voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(p_eigen, &distance);

			if (success){
				if (distance <= thresh_dist){
					total_distance += std::abs(distance);
				}
			}
			else{
				// cout << "error in tsdf" << endl;
			}
			++count_node;
		}
		++path_idx;
	}
	return total_distance/count_node;
}

void reconstructPath(const std::vector<double> &x, start_goal *sg, std::vector<Node*> &path){
	path.clear();
	int num_nodes = x.size()/3;
	int count = 0;
	Node* start = new Node();
	start->p.x() = sg->sx;
	start->p.y() = sg->sy;
	start->p.z() = sg->sz;
	path.push_back(start);
	while (count < num_nodes){
		int x_idx = count*3 + 0;
		int y_idx = count*3 + 1;
		int z_idx = count*3 + 2;
		double pos_x = x[x_idx];
		double pos_y = x[y_idx];
		double pos_z = x[z_idx];
		Node* n = new Node ();
		n->p.x() = pos_x; 
		n->p.y() = pos_y;
		n->p.z() = pos_z;
		// cout << pos_x << " " << pos_y << " " << pos_z << endl;
		path.push_back(n);
		++count;
	}
	Node* goal = new Node();
	goal->p.x() = sg->gx;
	goal->p.y() = sg->gy;
	goal->p.z() = sg->gz;
	path.push_back(goal);
	for (int i=0; i<path.size()-1; ++i){
		Node* this_node = path[i];
		Node* next_node = path[i+1];
		point3d direction = next_node->p - this_node->p;
		double node_yaw = atan2(direction.y(), direction.x());
		if (node_yaw < 0){
			node_yaw = 2*PI_const - (-node_yaw);
		}
		path[i]->yaw = node_yaw;
	}
	double last_yaw = sg->last_yaw;
	path[path.size()-1]->yaw = last_yaw;
}


// Calculate the total information gain of the given path
double calculatePathGain(std::vector<Node*> path, OcTree& tree){
	double total_path_gain = 0;
	// iterate through all nodes
	for (Node* n: path){
		double yaw = n->yaw;
		double total_node_gain = calculateUnknown(tree, n, dmax);
		double node_gain_yaw = interpolateNumVoxels(n, yaw);
		total_path_gain += node_gain_yaw;
	}
	return total_path_gain;
}

double calculateTotalPathObstacleDistance(std::vector<Node*> &path, voxblox::EsdfServer &voxblox_server, OcTree &tree){
	double total_distance = 0; 
	double thresh_dist = 2;
	int count_node = 0;
	int path_idx = 0;
	while (path_idx < path.size() - 1){
		Node* this_node = path[path_idx];
		Node* next_node = path[path_idx+1];
		point3d p1 = this_node->p;
		point3d p2 = next_node->p;
		std::vector<point3d> ray;
		tree.computeRay(p1, p2, ray);

		for (point3d p: ray){
			Eigen::Vector3d p_eigen (p.x(), p.y(), p.z());
			double distance = 0;
			bool success = voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(p_eigen, &distance);

			if (success){
				if (distance <= thresh_dist){
					total_distance += std::abs(distance);
				}
			}
			else{
				// cout << "error in tsdf" << endl;
			}
			++count_node;
		}
		++path_idx;
	}
	return total_distance;
}
