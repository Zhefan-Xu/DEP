#include <unordered_map>
#include <queue>
#include <chrono> 
using namespace std::chrono;



// Helper Function: node in close:
bool inClose(Node* n, const std::unordered_set<Node*> &close){
	std::unordered_set<Node*>::const_iterator got = close.find(n);
	if (got == close.end()){
		return false;
	}
	else{
		return true;
	}
}

bool is_collision_node(Node* n, Node* collision_node){
	if (n->p.x() == collision_node->p.x() and n->p.y() == collision_node->p.y() and n->p.z() == collision_node->p.z()){
		return true;
	}
	else{
		return false;
	}
}

std::vector<Node*> AStar(PRM* map,
						 Node* start,
						 Node* goal, 
						 OcTree& tree,
						 bool replan=false)
{
	// This version only finds the path to highest info gain for debug purpose
	std::vector<Node*> path;
 
	// Open: Priority Queue
	std::priority_queue<Node*, std::vector<Node*>, CompareNode> open;
	start->g = 0;
	open.push(start);
	// Close: unordered set
	std::unordered_set<Node*> close;
	bool find_path = false;
	// Terminates when img_n is in close
	while (true){
		if (inClose(goal, close)){
			find_path = true;
			// cout << "path find success!" << endl;
			break;
		}
		if (open.size() == 0){
			// cout << "Try next goal!" << endl;
			// cout << "This gain: " << goal->num_voxels << endl;
			find_path = false;
			// cout << "No valid path" << endl;
			break;
		}
		Node* current_node = open.top();
		open.pop();
		// cout << "open size" << open.size() << endl;
		// Open may have duplicates
		if (inClose(current_node, close)){
			continue;
		}
		// Insert it into close
		close.insert(current_node);
		// Iterate Through all adjcent node:
		for (Node* neighbor_node: current_node->adjNodes){
			// Node must be not in close
			if (not inClose(neighbor_node, close)){
				if (replan){
					bool has_collision = checkCollision(tree, current_node, neighbor_node);
					if (has_collision){
						continue;
					}
				}
				
				double cost = current_node->g + current_node->p.distance(neighbor_node->p);
				// if we have better g value, we update:
				if (cost < neighbor_node->g){
					neighbor_node->g = cost;
					neighbor_node->f = cost + neighbor_node->p.distance(goal->p);
					// Simply use g value as f value
					open.push(neighbor_node);
					neighbor_node->parent = current_node;
				}
			}
		}
	}
	// Backtracking:
	Node* ptr = goal;
	while (ptr != NULL and find_path){
		path.push_back(ptr);	
		ptr = ptr->parent;
	}
	std::reverse(path.begin(), path.end());

	// Clear Search
	for (Node* n: map->getRecord()){
		n->g = 1000;
		n->f = 1000;
		n->parent = NULL;
	}
	return path;
}