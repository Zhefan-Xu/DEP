#include <DEP/prm.h>

void test_get_neighbor(){
	point3d p (1, 2, 3);
	std::vector<point3d> neighbors = getNeighborNode(p);
	// Test:
	for (point3d p: neighbors){
		cout << "x: " << p.x() << " y: " << p.y() << " z: " << p.z() << endl; 
	}
	cout << "Size of neighbors: " << neighbors.size() << endl;
}

void test_build_road_map(OcTree &tree){
	int num_sample = 100;
	// std::vector<geometry_msgs::Point> map_vis_array;
	std::vector<visualization_msgs::Marker> map_vis_array;
	PRM* map;
	std::vector<Node*> path;
	map = new PRM ();
	map = buildRoadMap(tree, map, path, NULL,map_vis_array);
	cout << "map size: " << map->getSize() <<endl;
	cout << "map vis array size: " << map_vis_array.size() << endl;
	return;
}

int main(int argc, char** argv){
	cout << "test PRM" << endl;
	OcTree tree (0.1);
	tree.readBinary("/home/zhefan/check_rrt.bt");
	// test_build_road_map(tree);
	test_get_neighbor();
	return 0;
}