#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <time.h>
#include <math.h>
// #include <rh_nbv/kdtree.h>
#define PI_const 3.1415926
using std::cout; using std::endl; using std::vector;
using std::set; using std::pair; using std::map;
using namespace octomap;
typedef pair<point3d, float> sample;

void print_point3d(point3d p){
	cout << std::fixed << std::setprecision(2)<< " Point: " << "(" << p.x() << ", " <<
	p.y() << ", " << p.z() << ")    " << endl;	
}

void print_point3d_vector(vector<point3d> p){
	int count = 0;

	for (vector<point3d>::iterator itr = p.begin(); 
			itr != p.end(); ++itr){
		print_point3d(*itr);

		/*
			cout << "Point Num: " << count+1 << endl; 
			cout 
			<< "x: " << itr->x() << endl 
			<< "y: " << itr->y() << endl 
			<< "z: " << itr->z() << endl;
		*/

	}
}

void print_node(Node n){
	cout << "+-----------Node Info----------+" << endl;
	print_point3d(n.p);
	cout << " yaw: " << n.yaw << "                    "<<  endl;
	cout << " voxels: " << n.num_voxels << "             " << endl;
	cout << "+------------------------------+" << endl;
}


void print_node_vector(std::vector<Node*> path){
	cout << "=========>Path<========" << endl;
	for (std::vector<Node*>::iterator itr=path.begin();
			itr != path.end(); ++itr){
		print_node(*(*itr));
	}

	cout << "=========>END<==========" << endl;
}

void print_path(std::vector<Node> path){
	cout << "=========>Path<========" << endl;
	for (std::vector<Node>::iterator itr=path.begin();
			itr != path.end(); ++itr){
		print_node((*itr));
	}

	cout << "=========>END<==========" << endl;
}