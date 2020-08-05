#include <DEP/kdtree.h>
#include <DEP/utils.h>
#include <DEP/env.h>
#include <random>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>



// Depth CAMERA
double FOV = 1.8;
double dmin = 0;
double dmax = 1.0;

// Visualize Map
bool VISUALIZE_MAP = true;
// static std::vector<geometry_msgs::Point> DEFAULT_VECTOR;
static std::vector<visualization_msgs::Marker> DEFAULT_VECTOR;

std::vector<double> generate_yaws(int n){
	std::vector<double> yaws;
	for (int i=0; i<n; ++i){
		yaws.push_back(i*2*PI_const/n);
	}
	return yaws;
}
// std::vector<double> yaws {0, PI_const/4, PI_const/2, 3*PI_const/4, PI_const, 5*PI_const/4, 3*PI_const/2, 7*PI_const/4};
// std::vector<double> yaws {0, PI_const/8 ,PI_const/4, PI_const*3/8, PI_const/2, PI_const*5/8, 3*PI_const/4, PI_const*7/8, PI_const, PI_const*9/8,5*PI_const/4, PI_const*11/8,3*PI_const/2, PI_const*13/8,7*PI_const/4, PI_const*15/8};
std::vector<double> yaws = generate_yaws(32);
// Random Generator
std::random_device rd;
std::mt19937 mt(rd());
// Helper Function: Random Number
double randomNumber(double min, double max){
	std::uniform_real_distribution<double> distribution(min, max);
	return distribution(mt);
}


// Hepler Function: Check Position Validation
bool isValid(const OcTree& tree, point3d p, bool robot_size=false){
	if (robot_size == false){
		OcTreeNode* nptr = tree.search(p);
		if (nptr == NULL){return false;}
		return !tree.isNodeOccupied(nptr);
	}
	else{// we should consider the robot size in this case
		// calculate x, y, z range
		double x_min, x_max, y_min, y_max, z_min, z_max;
		x_min = p.x() - DRONE_X/2;
		x_max = p.x() + DRONE_X/2;
		y_min = p.y() - DRONE_Y/2;
		y_max = p.y() + DRONE_Y/2;
		z_min = p.z() - DRONE_Z/2;
		z_max = p.z() + DRONE_Z/2;

		for (double x=x_min; x<=x_max+DRONE_X/2; x+=RES){
			for (double y=y_min; y<=y_max+DRONE_X/2; y+=RES){
				for (double z=z_min; z<z_max; z+=RES){
					if (isValid(tree, point3d(x, y, z))){
						continue;
					}
					else{
						return false;
					}
				}
			}
		}
		return true;
	}
}

// Random Sample
// allow_not_valid = true indicates the configuration could be invalid by this generator
Node* randomConfig(const OcTree& tree, bool allow_not_valid=false){ 
	double min_x, max_x, min_y, max_y, min_z, max_z;
	tree.getMetricMax(max_x, max_y, max_z);
	tree.getMetricMin(min_x, min_y, min_z);
	min_x = std::max(min_x, env_x_min);
	max_x = std::min(max_x, env_x_max);
	min_y = std::max(min_y, env_y_min);
	max_y = std::min(max_y, env_y_max);
	min_z = std::max(min_z, env_z_min);
	max_z = std::min(max_z, env_z_max);
	bool valid = false;
	double x, y, z;
	point3d p;

	while (valid == false){
		p.x() = randomNumber(min_x, max_x);
		p.y() = randomNumber(min_y, max_y);
		p.z() = randomNumber(min_z, max_z);
		if (allow_not_valid==true){ 
			break;
		}
		valid = isValid(tree, p, true);
		// valid = isValid(tree, p);
	}

	Node* nptr = new Node(p);

	return nptr;
}


// BBX: (xmin, xmax, ymin, ymax, zmin, zmax)
Node* randomConfigBBX(const OcTree& tree, std::vector<double> &bbx){ 
	double min_x, max_x, min_y, max_y, min_z, max_z;
	tree.getMetricMax(max_x, max_y, max_z);
	tree.getMetricMin(min_x, min_y, min_z);
	min_x = std::max(min_x, bbx[0]);
	max_x = std::min(max_x, bbx[1]);
	min_y = std::max(min_y, bbx[2]);
	max_y = std::min(max_y, bbx[3]);
	min_z = std::max(min_z, bbx[4]);
	max_z = std::min(max_z, bbx[5]);
	min_x = std::max(min_x, env_x_min);
	max_x = std::min(max_x, env_x_max);
	min_y = std::max(min_y, env_y_min);
	max_y = std::min(max_y, env_y_max);
	min_z = std::max(min_z, env_z_min);
	max_z = std::min(max_z, env_z_max);
	bool valid = false;
	double x, y, z;
	point3d p;

	while (valid == false){
		
		p.x() = randomNumber(min_x, max_x);
		// Y>0
		// p.y() = randomNumber(0, max_y);
		p.y() = randomNumber(min_y, max_y);
		p.z() = randomNumber(min_z, max_z);
		// p.z() = randomNumber(min_z, 2.5); // garage and cafe
		valid = isValid(tree, p, true);
		// valid = isValid(tree, p);
	}

	Node* nptr = new Node(p);

	return nptr;
}



// Function: Do Collision Checking
bool checkCollision(OcTree& tree, Node* n1, Node* n2){
	point3d p1 = n1->p;
	point3d p2 = n2->p;
	std::vector<point3d> ray;
	tree.computeRay(p1, p2, ray);
	for (std::vector<point3d>::iterator itr=ray.begin(); itr!=ray.end(); ++itr){
		if (isValid(tree, *itr ,true)){
		// if (isValid(tree, *itr)){
			continue;
		}
		else{
			return true;
		}
	}
	return false;
}

// This function is to find the neighbor of a given octree node center
// p is octree node center, we can directly use them
std::vector<point3d> getNeighborNode(point3d p){
	std::vector<point3d> neighbors;
	std::vector<double> DX {-RES, 0, +RES}; //DX
	std::vector<double> DY {-RES, 0, +RES}; //DY
	std::vector<double> DZ {-RES, 0, +RES}; //DZ
	for (double dx: DX){
		for (double dy: DY){
			for (double dz: DZ){
				if (dx != 0 or dy != 0 or dz != 0){
					neighbors.push_back(point3d (p.x()+dx, p.y()+dy, p.z()+dz));
				}
			}
		}
	}
	return neighbors;
}

// P has to be the free cell
bool isFrontier(point3d p_request, const OcTree& tree){
	for (point3d p: getNeighborNode(p_request)){
		bool not_in_scope = p.x() > env_x_max or p.x() < env_x_min or p.y() > env_y_max or p.y() < env_y_min or p.z() > env_z_max or p.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(p);
		if (nptr != NULL and tree.isNodeOccupied(nptr) == false){

			return true;
		}
	}
	return false;
}

// P has to be a frontier in this function
bool isSurfaceFrontier(point3d p_request, const OcTree& tree){
	for (point3d p: getNeighborNode(p_request)){
		bool not_in_scope = p.x() > env_x_max or p.x() < env_x_min or p.y() > env_y_max or p.y() < env_y_min or p.z() > env_z_max or p.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(p);
		if (nptr != NULL and tree.isNodeOccupied(nptr)){
			return true;
		}
	}
	
	return false;
}

// This only count for vertical FOV not horizontal
bool isInFOV(const OcTree& tree, point3d p, point3d u, double dmax){
	double distance = p.distance(u);
	if (distance >= dmin and distance <= dmax){
		point3d direction = u - p;
		point3d face (direction.x(), direction.y(), 0);
		point3d v_direction = direction;
		double vertical_angle = face.angleTo(v_direction);
		if (vertical_angle <= FOV/2){
			point3d end;
			bool cast = tree.castRay(p, direction, end, true, distance);
			if (cast == false){
				return true;
			}
		}
	}
	return false;
}

double calculateUnknown(const OcTree& tree, Node* n, double dmax){
	// Position:
	point3d p = n->p;
	// Possible range
	double xmin, xmax, ymin, ymax, zmin, zmax;
	xmin = p.x() - dmax;
	xmax = p.x() + dmax;
	ymin = p.y() - dmax;
	ymax = p.y() + dmax;
	zmin = p.z() - dmax;
	zmax = p.z() + dmax;
	

	point3d pmin (xmin, ymin, zmin);
	point3d pmax (xmax, ymax, zmax);
	point3d_list node_centers;
	tree.getUnknownLeafCenters(node_centers, pmin, pmax);

	// Yaw candicates array;
	
	std::map<double, int> yaw_num_voxels;
	for (double yaw: yaws){
		yaw_num_voxels[yaw] = 0;
	}

	int count_total_unknown = 0;
	int count_total_frontier = 0;
	int count_total_surface_frontier = 0;
	for (std::list<point3d>::iterator itr=node_centers.begin(); 
		itr!=node_centers.end(); ++itr){
		point3d u = *itr;
		bool not_in_scope = u.x() > env_x_max or u.x() < env_x_min or u.y() > env_y_max or u.y() < env_y_min or u.z() > env_z_max or u.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(u);
		point3d direction = u - p;
		point3d face (direction.x(), direction.y(), 0);
		if (nptr == NULL){ // Unknown
			if (isInFOV(tree, p, u, dmax)){
				bool isNodeFrontier=false, isNodeSurfaceFrontier=false; 
				isNodeFrontier= isFrontier(u, tree);
				if (isNodeFrontier){
					isNodeSurfaceFrontier = isSurfaceFrontier(u, tree);
				}
				
				if (isNodeFrontier == false and isNodeSurfaceFrontier == false){
					count_total_unknown += 1;
				}
				else if (isNodeFrontier == true and isNodeSurfaceFrontier == false){
					count_total_unknown += 2;
				}
				else if (isNodeFrontier == true and isNodeSurfaceFrontier == true){
					count_total_unknown += 4;
				}


				// iterate through yaw angles
				for (double yaw: yaws){
					point3d yaw_direction (cos(yaw), sin(yaw), 0);
					double angle_to_yaw = face.angleTo(yaw_direction);
					if (angle_to_yaw <= FOV/2){
						// Give credits to some good unknown
						// case 1: it is a frontier unknown
						if (isNodeFrontier == false and isNodeSurfaceFrontier == false){
							yaw_num_voxels[yaw] += 1;
						}
						else if (isNodeFrontier == true and isNodeSurfaceFrontier == false){
							yaw_num_voxels[yaw] += 2;
						}
						else if (isNodeFrontier == true and isNodeSurfaceFrontier == true){
							yaw_num_voxels[yaw] += 4;
						}						
					}
				}
			}
		}
	}
	// cout << "+----------------------------+" << endl;
	// cout << "Total Unknown: "<< count_total_unknown << endl;
	// cout << "Total Frontier: " << count_total_frontier << endl;
	// cout << "Total Surface Frontier: " << count_total_surface_frontier << endl;
	// cout << "+----------------------------+" << endl;
	n->yaw_num_voxels = yaw_num_voxels;
	return count_total_unknown;
}


bool isNodeRequireUpdate(Node* n, std::vector<Node*> path, double& least_distance){
	double distance_thresh = 2;
	least_distance = 1000000;
	for (Node* waypoint: path){
		double current_distance = n->p.distance(waypoint->p);
		if (current_distance < least_distance){
			least_distance = current_distance;
		}
	}
	if (least_distance <= distance_thresh){
		return true;
	}
	else{
		return false;	
	}
	
}


// Ensure the vertical sensor range condition for node connection
bool sensorRangeCondition(Node* n1, Node* n2){
	point3d direction = n2->p - n1->p;
	point3d projection (direction.x(), direction.y(), 0);
	double vertical_angle = projection.angleTo(direction);
	if (vertical_angle < FOV/2){
		return true;
	}
	else{
		return false;
	}
}


PRM* buildRoadMap(OcTree &tree, 
				  PRM* map,
				  std::vector<Node*> path,
				  Node* start = NULL,  
				  std::vector<visualization_msgs::Marker> &map_vis_array = DEFAULT_VECTOR)
{
	// ==================================Sampling===========================================================
	// PRM* map = new PRM();
	map->clearGoalPQ();
	std::vector<Node*> new_nodes;
	// double threshold = 500; // HardCode threshold
	bool saturate = false;
	bool region_saturate = false;
	int count_sample = 0;
	int sample_thresh = 50;
	double distance_thresh = 0.8;
	while (not saturate){
		Node* n;
		double distance_to_nn = 0;
		// int r = (int) randomNumber(1,10);
		// cout << r << endl;
		if (region_saturate or start == NULL){
			
			int count_failure = 0;
			while (true){
				if (count_failure > sample_thresh){
					saturate = true;
					break;
				}
				n = randomConfig(tree);
				if (map->getSize() == 0){
					n->new_node = true;
					map->insert(n);
					new_nodes.push_back(n);
					++count_sample;
					break;
				}
				Node* nn = map->nearestNeighbor(n);
				distance_to_nn = n->p.distance(nn->p);
				// cout << "least distance" <<distance_to_nn << endl;
				if (distance_to_nn < distance_thresh){
					++count_failure;
					delete n;
				}
				else{
					n->new_node = true;
					map->insert(n);
					new_nodes.push_back(n);
					++count_sample;
					break;
				}
			}
		}
		else{
			int count_failure2 = 0;
			if (start != NULL){
				while (true){
					if (count_failure2 > sample_thresh){
						region_saturate = true;
						break;
					}
					// Bounding Box
					double start_yaw = start->yaw;
					double xmin = start->p.x() - 5;
					double xmax = start->p.x() + 5;
					double ymin = start->p.y() - 5;
					double ymax = start->p.y() + 5;
					double zmin = env_z_min;
					double zmax = env_z_max;
					if (start_yaw == 0){
						xmin = start->p.x()-2;
						ymax -= 2;
						ymin += 2;
					}
					else if (start_yaw == PI_const/4){
						xmin = start->p.x()-1;
						ymin = start->p.y()-1;
					}
					else if (start_yaw == PI_const/2){
						ymin = start->p.y()-2;
						xmax -= 2;
						xmin += 2;
					}
					else if (start_yaw == 3*PI_const/4){
						xmax = start->p.x()+1;
						ymin = start->p.y()-1;
					}
					else if (start_yaw == PI_const){
						xmax = start->p.x()+2;
						ymax -= 2;
						ymin += 2;
					}
					else if (start_yaw == 5*PI_const/4){
						xmax = start->p.x()+1;
						ymax = start->p.y()+1;
					}
					else if (start_yaw == 3*PI_const/2){
						ymax = start->p.y()+2;
						xmax -= 2;
						xmin += 2;
					}
					else if (start_yaw == 7*PI_const/4){
						xmin = start->p.x()-1;
						ymax = start->p.y()+1;
					}

					std::vector<double> bbx {xmin, xmax, ymin, ymax, zmin, zmax};
					n = randomConfigBBX(tree, bbx);
					Node* nn = map->nearestNeighbor(n);
					distance_to_nn = n->p.distance(nn->p);
					// cout << "least distance: " <<distance_to_nn << endl;
					if (distance_to_nn < distance_thresh){
						++count_failure2;
						delete n;
					}
					else{
						n->new_node = true;
						map->insert(n);
						new_nodes.push_back(n);
						++count_sample;
						break;
					}
				}
			}
		}
	}
	cout << "newly added: " << count_sample << " samples" << endl;


	// ========================Connect and Evaluate for new===================================================
	// Check neighbor and add edges
	for (Node* n: new_nodes){
		// Node* nearest_neighbor = map->nearestNeighbor(n);
		std::vector<Node*> knn = map->kNearestNeighbor(n, 15);

		for (Node* nearest_neighbor: knn){
			bool has_collision = checkCollision(tree, n, nearest_neighbor);
			double distance_to_knn = n->p.distance(nearest_neighbor->p);
			bool range_condition = sensorRangeCondition(n, nearest_neighbor) and sensorRangeCondition(nearest_neighbor, n);
			// if (distance_to_knn < 0.8){
			// 	cout << "bad node" << endl;
			// }
			if (has_collision == false and distance_to_knn < 1.5 and range_condition == true){
				n->adjNodes.insert(nearest_neighbor);
				nearest_neighbor->adjNodes.insert(n); 
			}
		}


		if (n->adjNodes.size() != 0){
			map->addRecord(n);
			double num_voxels = calculateUnknown(tree, n, dmax);
			n->num_voxels = num_voxels;
		}

	}

	// ==========================Update Old node===============================================================
	// Set cost and heuristics to inf
	int count_update_node = 0;
	int count_actual_update = 0;
	int total_unknown = 0;
	int max_unknown = 0;
	for (Node* n: map->getRecord()){
		n->g = 1000;
		n->f = 1000;
		n->parent = NULL;
		// Check whether the nodes need to update:
		double least_distance;
		bool update = isNodeRequireUpdate(n, path, least_distance);
		if (update and n->new_node==false){
			n->update = true;
			++count_update_node;
			// check the update condition: 
			// 1. if node is very close to trajetory: set it to zero
			// 2. if it is already 0, leave it
			// 3. if it is less than threshold (e.g 100), set it to zero
			// 4. if non of those applies, recalculate it
			double cut_off_value = 5;
			double cut_off_distance = 0.5;
			if (n->num_voxels <= cut_off_value or least_distance <= cut_off_distance){
				n->num_voxels = 0;
				for (double yaw:yaws){
					n->yaw_num_voxels[yaw] = 0;
				}
			}
			else{
				++count_actual_update;
				double num_voxels = calculateUnknown(tree, n, dmax);
				n->num_voxels = num_voxels;
			}
		}
		else{
			n->update = false;
		}
		n->new_node = false;

		if (n->num_voxels>max_unknown){
			max_unknown = n->num_voxels;
		}
		map->addGoalPQ(n);
		total_unknown += n->num_voxels;
	}
	map->setTotalUnknown(total_unknown);
	map->setMaxUnknown(max_unknown);
	cout << "Total Number of Unknown Voxels is: " << map->getTotalUnknown() << endl;
	cout << "Max Unknown Voxels is: " << map->getMaxUnknown() << endl;
	cout << "Number of nodes needed updated is: " << count_update_node << endl;
	cout << "Number of actual nodes needed updated is: " << count_actual_update << endl;

	// ====================================VISUALIZATION===============================================

	int node_point_id = 1;
	int unknown_voxel_id = 1000;
	std::vector<geometry_msgs::Point> node_vis_array;
	if (VISUALIZE_MAP){
		map_vis_array.clear();
	}

	if (VISUALIZE_MAP){
		for (Node* n: map->getRecord()){
			geometry_msgs::Point p;
			p.x = n->p.x();
			p.y = n->p.y();
			p.z = n->p.z();
			visualization_msgs::Marker node_point_vis_marker;
			node_point_vis_marker.header.frame_id = "world";
			node_point_vis_marker.id = node_point_id;
			node_point_vis_marker.header.stamp = ros::Time();
			node_point_vis_marker.type = visualization_msgs::Marker::CUBE;
			node_point_vis_marker.action = visualization_msgs::Marker::ADD;
			node_point_vis_marker.pose.position.x = p.x;
			node_point_vis_marker.pose.position.y = p.y;
			node_point_vis_marker.pose.position.z = p.z;
			node_point_vis_marker.scale.x = 0.1;
			node_point_vis_marker.scale.y = 0.1;
			node_point_vis_marker.scale.z = 0.1;
			node_point_vis_marker.color.a = 1.0; 
			node_point_vis_marker.color.r = 1.0;
			node_point_vis_marker.color.g = 0.0;
			node_point_vis_marker.color.b = 0.0;
			if (n->update == true){
				node_point_vis_marker.color.r = 0.0;
				node_point_vis_marker.color.g = 0.0;
				node_point_vis_marker.color.b = 1.0;
			}

			++node_point_id;

			visualization_msgs::Marker unknown_voxel_vis_marker;
			unknown_voxel_vis_marker.header.frame_id = "world";
			unknown_voxel_vis_marker.id = unknown_voxel_id;
			unknown_voxel_vis_marker.header.stamp = ros::Time();
			unknown_voxel_vis_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			unknown_voxel_vis_marker.action = visualization_msgs::Marker::ADD;
			unknown_voxel_vis_marker.pose.position.x = p.x;
			unknown_voxel_vis_marker.pose.position.y = p.y;
			unknown_voxel_vis_marker.pose.position.z = p.z+0.1;
			unknown_voxel_vis_marker.scale.x = 0.1;
			unknown_voxel_vis_marker.scale.y = 0.1;
			unknown_voxel_vis_marker.scale.z = 0.1;
			unknown_voxel_vis_marker.color.a = 1.0; 
			// unknown_voxel_vis_marker.color.r = 1.0;
			// unknown_voxel_vis_marker.color.g = 0.0;
			// unknown_voxel_vis_marker.color.b = 0.0;
			int num = (int) n->num_voxels;
			// int vis_angle =  n->yaw * 180/PI_const;
			// std::string sep = ", ";
			unknown_voxel_vis_marker.text = std::to_string(num); //+ sep + std::to_string(vis_angle);
			++unknown_voxel_id;
			map_vis_array.push_back(node_point_vis_marker);
			map_vis_array.push_back(unknown_voxel_vis_marker);
			for (Node* adj: n->adjNodes){
				geometry_msgs::Point p_adj;
				p_adj.x = adj->p.x();
				p_adj.y = adj->p.y();
				p_adj.z = adj->p.z();
				node_vis_array.push_back(p);
				node_vis_array.push_back(p_adj);
			}
		}
		visualization_msgs::Marker node_vis_marker;
		node_vis_marker.header.frame_id = "world";
		node_vis_marker.points = node_vis_array;
		node_vis_marker.id = 0;
		node_vis_marker.type = visualization_msgs::Marker::LINE_LIST;
		node_vis_marker.scale.x = 0.05;
		node_vis_marker.scale.y = 0.05;
		node_vis_marker.scale.z = 0.05;
		node_vis_marker.color.a = 1.0;
		node_vis_marker.color.r = 0.0;
		node_vis_marker.color.g = 1.0;
		node_vis_marker.color.b = 0.0;
		// cout << "NODE VIS ARRAY SIZE: "<< node_vis_array.size() << endl;
		map_vis_array.push_back(node_vis_marker);
	}

	return map;
}



// ===================Visualization============================


