#include <DEP/kdtree.h>
#include <DEP/utils.h>
#include <random>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// Define Drone Size:
double DRONE_X = 0.6;
double DRONE_Y = 0.6;
double DRONE_Z = 0.1;

// MAP RESOLUTION:
double RES = 0.1;

// Depth CAMERA
double FOV = 1.8;
double dmin = 0.00001;
double dmax = 1.0;

// Visualize Map
bool VISUALIZE_MAP = true;
// static std::vector<geometry_msgs::Point> DEFAULT_VECTOR;
static std::vector<visualization_msgs::Marker> DEFAULT_VECTOR;

std::vector<double> yaws {0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 3*pi/2, 7*pi/4};

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

		for (double x=x_min; x<x_max+RES; x+=RES){
			for (double y=y_min; y<y_max+RES; y+=RES){
				for (double z=z_min; z<z_max+RES; z+=RES){
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
		if (allow_not_valid==true){ 
			break;
		}
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

std::map<double, int> calculateUnknown(const OcTree& tree, Node* n){
	// Position:
	point3d p = n->p;
	// Possible range
	double xmin, xmax, ymin, ymax, zmin, zmax, distance;
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

	int count = 0;
	int jump_factor = 1;
	int jump_count = 0;
	for (std::list<point3d>::iterator itr=node_centers.begin(); 
		itr!=node_centers.end(); ++itr){
		++jump_count;
		if ((jump_count-1) % jump_factor != 0){
			continue;
		}
		point3d u = *itr;
		OcTreeNode* nptr = tree.search(u);
		if (nptr == NULL){ // Unknown
			distance = p.distance(u);

			if (distance >= dmin and distance <= dmax){
				point3d direction = u - p;
				point3d face (direction.x(), direction.y(), 0);


				point3d v_direction = direction;
				double vertical_angle = face.angleTo(v_direction);

				if (vertical_angle <= FOV/2){
					point3d end;
					bool cast = tree.castRay(p, direction, end, true, distance);
					if (cast == false){
						++count;
						// iterate through yaw angles
						for (double yaw: yaws){
							point3d yaw_direction (cos(yaw), sin(yaw), 0);
							double angle_to_yaw = face.angleTo(yaw_direction);
							if (angle_to_yaw <= FOV/2){
								yaw_num_voxels[yaw] += 1;
							}
						}
					}
				}

				// if (abs(horizawontal_angle) < horizontal_max_angle and abs(vertical_angle) < vertical_max_angle){
				// double angle = face.angleTo(direction);
				// if (angle <= FOV/2){
				// 	point3d end;
				// 	bool cast = tree.castRay(p, direction, end, true, distance);
				// 	double cast_distance = p.distance(end);
				// 	// cout << cast << endl;
				// 	if (cast == false){ // No Occupied was hit
				// 		++count;
				// 	}
				// }
			}
		}
	}

	return yaw_num_voxels;
}

PRM* buildRoadMap(OcTree &tree, 
				  int num_sample,
				  std::vector<visualization_msgs::Marker> &map_vis_array = DEFAULT_VECTOR)
{
	PRM* map = new PRM();
	std::vector<Node*> new_nodes;
	double threshold = 500; // HardCode threshold
	for (int i=0; i<num_sample; ++i){
		double distance_to_nn = 0;
		Node* n;
		while (distance_to_nn < 0.5){
			n = randomConfig(tree);
			if (map->getSize() == 0){break;}
			Node* nn = map->nearestNeighbor(n);
			distance_to_nn = n->p.distance(nn->p);
			if (distance_to_nn < 0.5){delete n;}
		}
		map->insert(n);
		new_nodes.push_back(n);
	}

	std::vector<geometry_msgs::Point> node_vis_array;
	if (VISUALIZE_MAP){
		map_vis_array.clear();
	}

	int node_point_id = 1;
	int unknown_voxel_id = 1000;
	// Check neighbor and add edges
	for (Node* n: new_nodes){
		// Node* nearest_neighbor = map->nearestNeighbor(n);
		std::vector<Node*> knn = map->kNearestNeighbor(n, 5);
		for (Node* nearest_neighbor: knn){
			bool has_collision = checkCollision(tree, n, nearest_neighbor);
			if (has_collision == false){
				n->adjNodes.insert(nearest_neighbor);
				nearest_neighbor->adjNodes.insert(n); 
			}
		}
		std::map<double, int> yaw_num_voxels = calculateUnknown(tree, n);
		double best_yaw;
		double best_num_voxels = 0;
		for (double yaw: yaws){
			double num_voxels = yaw_num_voxels[yaw];

			if (num_voxels > best_num_voxels){
				best_num_voxels = num_voxels;
				best_yaw = yaw;
			}
		}
		n->yaw = best_yaw;
		n->num_voxels = best_num_voxels;

		// if (n->num_voxels > threshold){
		map->addGoalNode(n);
		// }
		// ====================================VISUALIZATION===============================================
		// if (VISUALIZE_MAP){	

		// }
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
			int vis_angle =  n->yaw * 180/pi;
			std::string sep = ", ";
			unknown_voxel_vis_marker.text = std::to_string(num) + sep + std::to_string(vis_angle);
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

