#include <unordered_map>
#include <queue>



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

std::vector<Node*> multiGoalAStar(PRM* map,
								  Node* start)
{
	// This version only finds the path to highest info gain for debug purpose
	//i Goal Nodes
	std::vector<Node*> goal_nodes = map->getGoalNodes();
	Node* goal = new Node(point3d(0, 0, 0)); 

	double best_value = 0;
	for (Node* node: goal_nodes){
		if (node->num_voxels > best_value){
			best_value = node->num_voxels;
			goal = node;
		}
	}


	// Open: Priority Queue
	std::priority_queue<Node*, std::vector<Node*>, CompareNode> open;
	start->g = 0;
	open.push(start);
	// Close: unordered set
	std::unordered_set<Node*> close;

	// Terminates when img_n is in close
	while (not inClose(goal, close)){
		Node* current_node = open.top();
		open.pop();
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
				double cost = current_node->g + current_node->p.distance(neighbor_node->p);
				// if we have better g value, we update:
				if (cost < neighbor_node->g){
					neighbor_node->g = cost;
					neighbor_node->f = cost;
					// Simply use g value as f value
					open.push(neighbor_node);
					neighbor_node->parent = current_node;
				}
			}
		}
	}

	// Backtracking:
	std::vector<Node*> path;
	Node* ptr = goal;
	while (ptr != NULL){
		path.push_back(ptr);	
		ptr = ptr->parent;
	}
	std::reverse(path.begin(), path.end());
	return path;
}