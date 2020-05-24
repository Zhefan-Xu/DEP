#include <iostream>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <algorithm>
#include <stdlib.h>
#include <unordered_set>
using namespace octomap;
using std::cout; using std::endl;

// =====================RRT/KDTree NODE=================================
typedef struct Node{
	point3d p; // (x, y, z) see octomath::vector3
	double yaw; //radius for best yaw angle
	double num_voxels;
	struct Node* left;   // kd-tree left
	struct Node* right;  // kd-tree right
	struct Node* parent; // for backtracking
	struct Node* img_n;  // for multi goal a star
	double g; 			 // gvalue for a star search
	double f;            // fvalue in a star search
	std::unordered_set<Node*> adjNodes;
	Node(point3d _p){
		p = _p;
		left = NULL;
		right = NULL;
		parent = NULL;
		g = 10000000;
	}
} Node;

struct CompareNode{
	bool operator()(Node* n1, Node* n2){
		return n1->f > n2->f;
	}
};
//======================================================================


//===================Class Declaration==================================
typedef class KDTree{
private:
	int size;
	Node* root;
	std::vector<Node*> goal_nodes;
	std::vector<Node*> not_target;
public:
	KDTree();
	Node* getRoot();
	int getSize();
	void insert(Node* n);
	Node* nearestNeighbor(Node* n, 
					      Node* root_node,
					      Node* best_node,
						  double least_distance,
						  int depth); // return pointer to the nearest neighbor
	std::vector<Node*> kNearestNeighbor(Node* n, int num);
	void addGoalNode(Node* n);
	std::vector<Node*> getGoalNodes();
	void clear();   // empty tree
} PRM;

//======================================================================


// ===================Implementation====================================
KDTree::KDTree(){
	// root = NULL;
	size = 0;
}

Node* KDTree::getRoot(){
	return root;
}

int KDTree::getSize(){
	return size;
}

void KDTree::insert(Node* n){
	n->left = NULL;
	n->right = NULL;
	// If tree is emtpy, we add root
	if (size == 0){
		root = n;
		++size;
		return;
	}
	else{
		Node* ptr = root;
		int depth = 0;
		double value, insert_value;
		while (true){
			if (depth % 3 == 0){
				value = ptr->p.x();
				insert_value = n->p.x();
			}
			else if (depth % 3 == 1){
				value = ptr->p.y();
				insert_value = n->p.y();

			}
			else if (depth % 3 == 2){
				value = ptr->p.z();
				insert_value = n->p.z();

			}
			// Compare the value:
			// >=: right, < left
			if (insert_value >= value){
				if (ptr->right == NULL){
					ptr->right = n;
					++size;
					return;
				}
				ptr = ptr->right;
			}
			else{
				if (ptr->left == NULL){
					ptr->left = n;
					++size;
					return;
				}
				ptr = ptr->left;
			}
			++depth;
		}
	}
	return;
}

Node* KDTree::nearestNeighbor(Node* n, 
							  Node* root_node=NULL,
							  Node* best_node = NULL,
							  double least_distance=1000000,
							  int depth=0){
	point3d p = n->p; // get position/coordinate of the node
	Node* ptr;
	if (root_node == NULL){
		ptr = root;	
	}
	else{
		ptr = root_node;
	}
	// Search Good side
	// Store Bad side
	std::vector<Node*> bad_side;
	while (ptr != NULL){
		// cout << ptr->p.x() << ptr->p.y() << ptr->p.z() << endl;
		// cout << "depth: " << depth << endl;
		// Check current node againt the best node
		double distance_current = p.distance(ptr->p);

		//==============Nodes Which are not our target================
 		if (ptr == n){
			distance_current = 10000000;
		}
		for (Node* nt: this->not_target){
			if (ptr == nt){
				distance_current = 10000000;
				break;
			}
		}

		//============================================================
		if (distance_current < least_distance){	
			best_node = ptr;
			least_distance = distance_current;
		}


		// Determine which side is better:
		double value, query_value;
		if (depth % 3 == 0){
			value = ptr->p.x();
			query_value = p.x();
		}
		else if (depth % 3 == 1){
			value = ptr->p.y();
			query_value = p.y();
		}
		else if (depth % 3 == 2){
			value = ptr->p.z();
			query_value = p.z();
		}

		// if < then search left, if >= search right
		if (query_value < value){
			bad_side.push_back(ptr->right);	
			ptr = ptr->left;
			// cout << "left" << endl;
		}
		else{
			bad_side.push_back(ptr->left);	
			ptr = ptr->right;
			// cout << "right" << endl;
		}
		++depth;
	}
	// Search bad side:
	std::reverse(bad_side.begin(), bad_side.end());
	// cout << "bad side size: " << bad_side.size() << endl;
	// cout << "least distance after good search: " << least_distance << endl; 
	int count = 0;
	for (std::vector<Node*>::iterator itr = bad_side.begin(); 
		 itr != bad_side.end(); ++itr){
		// cout << "count: " << count << endl;
		++count;
		// If there is no node in bad side
		if (*itr == NULL){
			// cout << "no branch" << endl;
			--depth;
			continue;
		}
		else{
			// cout << "have branch" << endl;
			double value, query_value;
			if (depth % 3 == 0){
				value = (*itr)->p.x();
				query_value = p.x();
			}
			else if (depth % 3 == 1){
				value = (*itr)->p.y();
				query_value = p.y();
			}
			else if (depth % 3 == 2){
				value = (*itr)->p.z();
				query_value = p.z();
			}
			double best_bad_side_distance = abs(value - query_value);
			if (best_bad_side_distance >= least_distance){
				// cout << "best distance is not good enough" << endl;
				// cout << "best bad side distance: " << best_bad_side_distance << endl;
				// cout << "least distance: " << least_distance << endl;
				--depth;
				continue;
			}
			else{
				// cout << "recursive call at depth : " << depth << endl;
				best_node = nearestNeighbor(n, *itr, best_node, least_distance, depth);
				--depth;	
			}
		}	
	}
	return best_node;
}

std::vector<Node*> KDTree::kNearestNeighbor(Node* n, int num){
	std::vector<Node*> knn;
	for (int i=0; i<num; ++i){
		Node* nearest_neighbor = nearestNeighbor(n);
		knn.push_back(nearest_neighbor);
		this->not_target.push_back(nearest_neighbor);
	}
	this->not_target.clear();
	return knn;
}

void KDTree::addGoalNode(Node* n){
	this->goal_nodes.push_back(n);
}

std::vector<Node*> KDTree::getGoalNodes(){
	return this->goal_nodes;
}

void KDTree::clear(){
	root = NULL;
	size = 0;
	return;
}