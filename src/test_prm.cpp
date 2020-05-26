#include <DEP/prm.h>

void test_build_road_map(OcTree &tree){
	int num_sample = 100;
	// std::vector<geometry_msgs::Point> map_vis_array;
	std::vector<visualization_msgs::Marker> map_vis_array;
	PRM* map;
	map = new PRM ();
	map = buildRoadMap(tree, map, num_sample, map_vis_array);
	cout << "map size: " << map->getSize() <<endl;
	cout << "map vis array size: " << map_vis_array.size() << endl;
	return;
}

int main(int argc, char** argv){
	cout << "test PRM" << endl;
	OcTree tree (0.1);
	tree.readBinary("/home/zhefan/check_rrt.bt");
	test_build_road_map(tree);
	return 0;
}