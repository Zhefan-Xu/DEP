#include <ros/ros.h>
#include <DEP/prm.h>
#include <DEP/multi_astar.h>
#include <nlopt.hpp>


std::vector<Node*> test_tsdf(OcTree &tree, voxblox::EsdfServer &voxblox_server);
double interpolateNumVoxels(Node* n, double yaw);
std::vector<Node*> findBestPath(PRM* roadmap,
                                Node* start,
                                std::vector<Node*> goal_candidates,
                                OcTree& tree,
                                bool replan);
std::vector<Node*> getGoalCandidates(PRM* roadmap);
Node* findStartNode(PRM* map, Node* current_node, OcTree& tree);
double calculatePathLength(std::vector<Node*> path);
double calculatePathRotation(std::vector<Node*> path, double current_yaw);
double findDistanceToWall(double x, double y, double z, voxblox::EsdfServer &voxblox_server);
bool checkPathCollision(std::vector<Node*> &path, OcTree &tree);
bool minDistanceCondition(std::vector<Node*> &path);
bool sensorConditionPath(std::vector<Node*> &path);
double calculatePathTime(std::vector<Node*> &path, double current_yaw, double linear_velocity, double angular_velocity);
double calculatePathObstacleDistance(std::vector<Node*> &path, voxblox::EsdfServer &voxblox_server, OcTree &tree);


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
	cout << score << endl;
	// cout << path_time << endl;
	// print_node_vector(path);
    return score;
}

std::vector<Node*> optimize_path(std::vector<Node*> path, OcTree* tree_ptr, voxblox::EsdfServer* voxblox_server){
	// print_node_vector(path);
	// Intialize optimizer
	int num_of_variables = (path.size()-2) * 3;
	Node* start = *(path.begin());
	Node* goal = *(path.end()-1);
	// start_goal sg = {start->p.x(), start->p.y(), start->p.z(), 
	// 				 goal->p.x(), goal->p.y(), goal->p.z()};
	double current_yaw = 0;
	double last_yaw = (*(path.end()-1))->yaw;
	double linear_velocity = 0.3;
	double angular_velocity = 0.8;
	double time = calculatePathTime(path, current_yaw, linear_velocity, angular_velocity);
	double distance = calculatePathObstacleDistance(path, *voxblox_server, *tree_ptr);
	start_goal sg (start->p.x(), start->p.y(), start->p.z(), 
	 				 goal->p.x(), goal->p.y(), goal->p.z(), current_yaw, last_yaw,
	 				 linear_velocity, angular_velocity, time, distance,
	 				 tree_ptr, voxblox_server);

	nlopt::opt opt(nlopt::GN_DIRECT_L, num_of_variables);
	// set bound -> +- 0.5 m
	int count_node_idx = 0;
	int count_variable = 0;
	std::vector<double> lb (num_of_variables);
	std::vector<double> ub (num_of_variables);
	// Initialize variables
	std::vector<double> x_variables (num_of_variables);
	double bound_dis = 0.25;
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
	// set bound
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);
	// set objective function
	opt.set_min_objective(objective_function, &sg);
	// set termination criteria
	// opt.set_xtol_rel(0.2);
	opt.set_maxeval(1000);
	double min_obj;
	cout << "here" << endl;
	try{
		nlopt::result result = opt.optimize(x_variables, min_obj);
		// cout << "x[0]: " << x_variables[0] << endl;
		// cout << "Time before: " << calculatePathTime(path, current_yaw, linear_velocity, angular_velocity) << endl;
		cout << "min score: " << min_obj << endl;
		// cout << "theoretical min: " << start->p.distance(goal->p);
		for (int i=0; i < path.size()-2; ++i){
			cout << x_variables[i*3+0] << " " << x_variables[i*3+1] << " " << x_variables[i*3+2] << endl; 
		}
		std::vector<Node*> optimized_path;
		reconstructPath(x_variables, &sg, optimized_path);
		return optimized_path;
	}
	catch(std::exception &e){
		cout << "nlopt fail" << e.what() << endl;
		return path;
	}


}













int main(int argc, char** argv){
	ros::init(argc, argv, "test_tsdf");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	// tsdf map
	voxblox::EsdfServer* voxblox_server= new voxblox::EsdfServer(nh, nh_private);
	voxblox_server->loadMap("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_voxblox.vxblx");
	// octomap
	// OcTree tree (0.2);
	OcTree* tree_ptr = new OcTree (0.2);
	tree_ptr->readBinary("/home/zhefan/catkin_ws/src/DEP/src/test/cafe_octree.bt");
	std::vector<Node*> path = test_tsdf(*tree_ptr, *voxblox_server);
	path = optimize_path(path, tree_ptr, voxblox_server);
	print_node_vector(path);
	return 0;
}

std::vector<Node*> test_tsdf(OcTree &tree, voxblox::EsdfServer &voxblox_server){
	cout << "tree resolution: " << tree.getResolution() << endl;
	Eigen::Vector3d p_test (0.3, 6, 1);
	double distance = 0;
	// bool success = voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(p_test, &distance);
	// if (success){
	// 	cout << "distance to wall: " << distance << endl;
	// }
	Node current_pose (point3d (0.3, 6, 1));
	// PRM* roadmap = new PRM ();
	// std::vector<Node*> last_path;
	// roadmap = buildRoadMap(tree, roadmap, last_path);
	// Node* start = findStartNode(roadmap, &current_pose, tree);
	// std::vector<Node*> goal_candidates = getGoalCandidates(roadmap);
	// bool replan = false;
	// std::vector<Node*> best_path = findBestPath(roadmap, start, goal_candidates, tree, replan);
	Node* p1 = new Node (point3d (0.74, 6.39, 0.79));
	// Node p1 (point3d (0.74, 6.39, 0.79));
	p1->yaw = 0;
	Node* p2 = new Node (point3d (0.48, 5.14, 0.59));
	// Node p2 (point3d (0.48, 5.14, 0.59));
	p2->yaw = 0;
	Node* p3 = new Node (point3d (0.57, 4.36, 0.95));
	// Node p3 (point3d (0.57, 4.36, 0.95));
	p3->yaw = 0;
	Node* p4 = new Node (point3d (0.50, 3.32, 1.76));
	// Node p4 (point3d (0.50, 3.32, 1.76));
	p4->yaw = 0;
	Node* p5 = new Node (point3d (-0.41, 3.65, 1.93));
	// Node p5 (point3d (-0.41, 3.65, 1.93));
	p5->yaw = 0;
	Node* p6 = new Node (point3d (-1.25, 4.32, 1.83));
	// Node p6 (point3d (-1.25, 4.32, 1.83));
	p6->yaw = 0;
	Node* p7 = new Node (point3d (-2.48, 4.71, 1.34));
	// Node p7 (point3d (-2.48, 4.71, 1.34));
	p7->yaw = 0;
	Node* p8 = new Node (point3d (-2.41, 5.85, 0.76));
	// Node p8 (point3d (-2.41, 5.85, 0.76));
	p8->yaw = 0;
	Node* p9 = new Node (point3d (-2.04, 6.64, 1.16));
	// Node p9 (point3d (-2.04, 6.64, 1.16));
	p9->yaw = 1;
	std::vector<Node*> best_path;
	best_path.push_back(p1);
	best_path.push_back(p2);
	best_path.push_back(p3);
	best_path.push_back(p4);
	best_path.push_back(p5);
	best_path.push_back(p6);
	best_path.push_back(p7);
	best_path.push_back(p8);
	best_path.push_back(p9);
	for (int i=0; i<best_path.size()-1; ++i){
		Node* this_node = best_path[i];
		Node* next_node = best_path[i+1];
		point3d direction = next_node->p - this_node->p;
		double node_yaw = atan2(direction.y(), direction.x());
		if (node_yaw < 0){
			node_yaw = 2*PI_const - (-node_yaw);
		}
		best_path[i]->yaw = node_yaw;
	}

	

	// print_node_vector(best_path);
	return best_path;
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
	final_path = path_vector[best_idx];
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
	// cout << "final id: " << best_idx << endl;
	// cout << "final path size: " << final_path.size() << endl;
	return final_path;
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
				cout << "error in tsdf" << endl;
			}
			++count_node;
		}
		++path_idx;
	}
	return total_distance/count_node;
}

void reconstructPath(const std::vector<double> &x, start_goal *sg, std::vector<Node*> &path){
	path.clear();
	int num_nodes = (x.size()) / 3;
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
