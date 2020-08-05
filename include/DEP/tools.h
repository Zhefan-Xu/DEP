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

bool checkNextGoalCollision(Node current_node, DEP::Goal next_goal, OcTree& tree){
	Node goal_node (point3d (next_goal.x, next_goal.y , next_goal.z));
	return checkCollision(tree, &current_node, &goal_node); // true if there is collision otherwise no collision
}
